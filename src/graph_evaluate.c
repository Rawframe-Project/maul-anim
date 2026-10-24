// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Evaluating a graph instance: each weighed node in order
// writes its slot's pose from its inputs' slots, through a scratch of a
// pose per slot made once for the binding it last served.

#include "blend_groups.h"
#include "bytes.h"
#include "clip_view.h"
#include "fpenv.h"
#include "graph_evaluate_ik.h"
#include "graph_evaluation.h"
#include "graph_file.h"
#include "graph_inertia_apply.h"
#include "graph_instance.h"
#include "graph_learned.h"
#include "graph_machine.h"
#include "graph_match.h"
#include "graph_shot.h"
#include "graph_space.h"
#include "graph_spring.h"
#include "graph_subset.h"
#include "graph_turn.h"
#include "learned_eval.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define SCRATCH_STAMP 0x4D475343u
#define HEADER_SIZE   64u

// The scratch's head: whom its poses were made for.
typedef struct Head
{
    uint32_t stamp;
    uint32_t slotCount;
    const maniGraphBinding* binding;
    uint64_t identity;
} Head;

static size_t Round(size_t size)
{
    return (size + MANI_GRAPH_SCRATCH_ALIGNMENT - 1) / MANI_GRAPH_SCRATCH_ALIGNMENT *
           MANI_GRAPH_SCRATCH_ALIGNMENT;
}

static maniGraphEvalParts PartsOf(const maniGraphBinding* binding)
{
    maniGraphEvalParts parts;
    parts.layersAt = HEADER_SIZE;
    parts.weightsAt = parts.layersAt + Round((size_t)binding->maxInputs * sizeof(maniBlendLayer));
    parts.posesAt =
        parts.weightsAt + Round(2 * (size_t)binding->graph.skeletonBoneCount * sizeof(float));
    parts.poseSize = maniPoseSize(binding->skeleton);
    parts.size = parts.posesAt + (size_t)binding->graph.slotCount * parts.poseSize;
    parts.retargetAt = parts.size;
    if (binding->sourcePoseSize > 0)
    {
        parts.size = parts.retargetAt + 2 * binding->sourcePoseSize + parts.poseSize +
                     binding->retargetScratchSize;
    }
    return parts;
}

size_t maniGraphScratchSize(const maniGraphBinding* binding)
{
    if (binding == NULL || binding->stamp != MANI_GRAPH_BINDING_STAMP)
    {
        return 0;
    }
    return PartsOf(binding).size;
}

// The poses made for this binding, once.
static void Prepare(maniGraphEvaluation* e)
{
    void* memory = e->scratch;
    Head* head = memory;
    if (head->stamp == SCRATCH_STAMP && head->binding == e->binding &&
        head->identity == e->graph->identity)
    {
        return;
    }
    for (uint32_t slot = 0; slot < e->graph->slotCount; ++slot)
    {
        maniPose* pose = NULL;
        (void)maniInitPoseEntered(e->binding->skeleton, maniGraphEvalSlot(e, slot),
                                  e->parts.poseSize, &pose);
    }
    *head = (Head){SCRATCH_STAMP, e->graph->slotCount, e->binding, e->graph->identity};
}

static void Blend(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                  bool difference)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t n = maniLoadU32(record + 8);
    bool direct = maniGraphEvalWord(e, d) != 0;
    float total = 0.0f;
    for (uint32_t k = 0; k < n; ++k)
    {
        float v = maniGraphEvalScalar(e, d + 1 + 2 * k);
        total += v > 0.0f ? v : 0.0f;
    }
    float scale = direct || total < MANI_BLEND_THRESHOLD ? 1.0f : 1.0f / total;
    uint32_t count = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        float v = maniGraphEvalScalar(e, d + 1 + 2 * k) * scale;
        v = v > 0.0f ? (v < 1.0f ? v : 1.0f) : 0.0f;
        const maniPose* input = maniGraphEvalInput(e, record, k);
        if (v > 0.0f && input != NULL)
        {
            e->layers[count++] = (maniBlendLayer){input, v, NULL};
        }
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        maniGraphEvalNeutral(e, out, difference);
    }
}

// A blend space: its inputs at their weights, blended.
static void Space(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                  bool difference)
{
    uint32_t n = maniLoadU32(record + 8);
    float weights[MANI_GRAPH_MAX_SPACE_POINTS];
    maniGraphSpaceWeights(e->graph, e->instance->cells, maniLoadU16(record),
                          maniLoadU32(record + 12), n, weights);
    uint32_t count = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniPose* input = maniGraphEvalInput(e, record, k);
        if (weights[k] > 0.0f && input != NULL)
        {
            e->layers[count++] = (maniBlendLayer){input, weights[k], NULL};
        }
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        maniGraphEvalNeutral(e, out, difference);
    }
}

// A state machine: its states of weight above 0, blended by its weights.
static void Machine(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                    bool difference)
{
    maniGraphMachine m = {e->graph, maniLoadU32(record + 12), maniLoadU32(record + 8)};
    const uint32_t* block =
        (const uint32_t*)((const uint8_t*)e->instance + e->instance->machinesAt) +
        maniGraphMachineBlockAt(&m);
    uint32_t count = 0;
    for (uint32_t k = 0; k < m.states; ++k)
    {
        float weight = maniGraphMachineWeight(block, k);
        const maniPose* input = maniGraphEvalInput(e, record, k);
        if (weight > 0.0f && input != NULL)
        {
            e->layers[count++] = (maniBlendLayer){input, weight, NULL};
        }
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        maniGraphEvalNeutral(e, out, difference);
    }
}

static void Mask(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                 bool difference)
{
    uint32_t bones = e->graph->skeletonBoneCount;
    uint32_t mask = maniGraphEvalWord(e, maniLoadU32(record + 12));
    memcpy(e->inside, e->graph->maskWeights + 4 * (size_t)mask * bones, 4 * (size_t)bones);
    for (uint32_t b = 0; b < bones; ++b)
    {
        e->outside[b] = 1.0f - e->inside[b];
    }
    uint32_t count = 0;
    const maniPose* inside = maniGraphEvalInput(e, record, 0);
    const maniPose* outside = maniGraphEvalInput(e, record, 1);
    if (inside != NULL)
    {
        e->layers[count++] = (maniBlendLayer){inside, 1.0f, e->inside};
    }
    if (outside != NULL)
    {
        e->layers[count++] = (maniBlendLayer){outside, 1.0f, e->outside};
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        maniGraphEvalNeutral(e, out, difference);
    }
}

// A one-shot: per bone the shot at its weight times the mask's (1
// without one), the base at the rest, as a mask node of those weights.
static void Shot(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                 bool difference)
{
    maniGraphShot shot = {e->graph, maniLoadU32(record + 12), 0};
    const uint32_t* block =
        (const uint32_t*)((const uint8_t*)e->instance + e->instance->machinesAt) +
        maniGraphShotWord(&shot, maniGraphShotBlock);
    float w = 0.0f;
    memcpy(&w, &block[maniGraphShotWeight], sizeof w);
    uint32_t bones = e->graph->skeletonBoneCount;
    uint32_t mask = maniGraphShotWord(&shot, maniGraphShotMask);
    for (uint32_t b = 0; b < bones; ++b)
    {
        float m = 1.0f;
        if (mask != MANI_GRAPH_NO_MASK)
        {
            memcpy(&m, e->graph->maskWeights + 4 * ((size_t)mask * bones + b), sizeof m);
        }
        e->inside[b] = w * m;
        e->outside[b] = 1.0f - w * m;
    }
    uint32_t count = 0;
    const maniPose* base = maniGraphEvalInput(e, record, 0);
    const maniPose* played = maniGraphEvalInput(e, record, 1);
    if (played != NULL && w > 0.0f)
    {
        e->layers[count++] = (maniBlendLayer){played, 1.0f, e->inside};
    }
    if (base != NULL)
    {
        e->layers[count++] = (maniBlendLayer){base, 1.0f, e->outside};
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        maniGraphEvalNeutral(e, out, difference);
    }
}

// An inertialize node: its input eased by its block, which keeps the
// output.
static void Inertia(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                    bool difference)
{
    uint32_t data = maniLoadU32(record + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)e->instance + e->instance->machinesAt) +
                      maniGraphEvalWord(e, data + maniGraphInertiaBlock);
    const maniPose* input = maniGraphEvalInput(e, record, 0);
    if (input == NULL)
    {
        maniGraphEvalNeutral(e, out, difference);
        return;
    }
    maniGraphInertia node = maniGraphInertiaOf(e->graph, data);
    node.groups = e->groups;
    maniGraphInertiaEvaluate(block, &node, input, out);
}

static void Additive(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t n = maniLoadU32(record + 8);
    const maniPose* base = maniGraphEvalInput(e, record, 0);
    if (base == NULL)
    {
        maniGraphEvalNeutral(e, out, false);
        return;
    }
    maniPoseCopy(out, base);
    uint32_t count = 0;
    for (uint32_t k = 1; k < n; ++k)
    {
        float v = maniGraphEvalScalar(e, d + 2 * (k - 1));
        v = v > 0.0f ? (v < 1.0f ? v : 1.0f) : 0.0f;
        const maniPose* layer = maniGraphEvalInput(e, record, k);
        if (v > 0.0f && layer != NULL)
        {
            e->layers[count++] = (maniBlendLayer){layer, v, NULL};
        }
    }
    (void)maniAddPosesIn(e->binding->skeleton, out, e->layers, count, e->groups);
}

// A mapped clip's pose: sampled into a pose of the map's source and
// carried; an additive clip's difference added to the source's rest,
// carried, and taken from the source's rest carried. The room's poses
// are made again only when they are not poses of their skeletons.
static void SampleMapped(const maniGraphEvaluation* e, const maniGraphBoundMapPair* pair,
                         maniClipCursor* c, const maniClip* clip, float time, maniPose* out)
{
    uint8_t* at = e->scratch + e->parts.retargetAt;
    size_t sourceSize = e->binding->sourcePoseSize;
    maniPose* poses[3] = {(void*)at, (void*)(at + sourceSize), (void*)(at + 2 * sourceSize)};
    void* scratch = at + 2 * sourceSize + e->parts.poseSize;
    const maniSkeleton* skeleton = e->binding->skeleton;
    if (!maniPoseMatches(poses[0], pair->source) || !maniPoseMatches(poses[1], pair->source) ||
        !maniPoseMatches(poses[2], skeleton))
    {
        (void)maniInitPoseEntered(pair->source, poses[0], sourceSize, &poses[0]);
        (void)maniInitPoseEntered(pair->source, poses[1], sourceSize, &poses[1]);
        (void)maniInitPoseEntered(skeleton, poses[2], e->parts.poseSize, &poses[2]);
    }
    size_t size = e->binding->retargetScratchSize;
    (void)maniSampleClipCursorEntered(c, clip, pair->source, time, poses[0]);
    if (!maniPoseIsDifference(poses[0]))
    {
        (void)maniRetargetPose(pair->map, pair->source, poses[0], skeleton, out, scratch, size);
        return;
    }
    maniBlendLayer layer = {poses[0], 1.0f, NULL};
    (void)maniPoseSetRest(poses[1], pair->source);
    (void)maniAddPoses(pair->source, poses[1], &layer, 1);
    (void)maniRetargetPose(pair->map, pair->source, poses[1], skeleton, out, scratch, size);
    (void)maniPoseSetRest(poses[1], pair->source);
    (void)maniRetargetPose(pair->map, pair->source, poses[1], skeleton, poses[2], scratch, size);
    (void)maniPoseDifference(skeleton, out, poses[2], out);
}

// A clip at a time: through its cursor, or for a subset the plan's bones
// by random access, the same bits (the cursor's), the pose made of
// the clip's kind first.
static void SampleClip(const maniGraphEvaluation* e, maniClipCursor* c, const maniClip* clip,
                       float time, maniPose* out)
{
    if (e->plan == NULL)
    {
        (void)maniSampleClipCursorEntered(c, clip, e->binding->skeleton, time, out);
        return;
    }
    maniPoseSetKind(out, maniClipAdditiveBasis(clip) != maniAdditiveNone);
    (void)maniSampleClipBonesEntered(clip, e->binding->skeleton, time, maniGraphSubsetList(e->plan),
                                     e->plan->count, out);
}

// A clip node's pose: its clip at its playhead, through its cursor (and
// its map).
static void Sample(const maniGraphEvaluation* e, const uint8_t* record, uint32_t cursor,
                   maniPose* out, const maniGraphNodeState* s)
{
    uint32_t slot = maniGraphEvalWord(e, maniLoadU32(record + 12));
    const maniClip* clip = e->binding->clips[slot];
    void* memory =
        (uint8_t*)e->instance + e->instance->cursorsAt + (size_t)cursor * e->binding->cursorSize;
    maniClipCursor* c = memory;
    const maniGraphBoundMapPair* pair = maniGraphClipMapPair(e->binding, slot);
    if (pair != NULL)
    {
        SampleMapped(e, pair, c, clip, (float)s->time, out);
        return;
    }
    SampleClip(e, c, clip, (float)s->time, out);
}

// A motion matching node's pose: its entry's clip at its playhead,
// through its cursor; the neutral pose with no entry.
static void SampleMatch(const maniGraphEvaluation* e, const uint8_t* record, uint32_t cursor,
                        maniPose* out, const maniGraphNodeState* s)
{
    uint32_t data = maniLoadU32(record + 12);
    uint32_t slot = maniGraphEvalWord(e, data + maniGraphMatchDatabase);
    uint32_t entry = ((const uint32_t*)((const uint8_t*)e->instance + e->instance->machinesAt))
        [maniGraphEvalWord(e, data + maniGraphMatchBlock) + maniGraphMatchEntry];
    maniMatchEntry m;
    if (maniMotionDatabaseEntry(maniGraphBoundDatabase(e->binding, slot), entry, &m) !=
        mani_success)
    {
        maniGraphEvalNeutral(e, out, false);
        return;
    }
    const uint8_t* d = e->graph->databases + (size_t)slot * MANI_GRAPH_DATABASE_SIZE;
    uint32_t clipSlot =
        maniLoadU32(e->graph->databaseClips + 4 * (size_t)(maniLoadU32(d + 24) + m.clip));
    void* memory =
        (uint8_t*)e->instance + e->instance->cursorsAt + (size_t)cursor * e->binding->cursorSize;
    maniClipCursor* c = memory;
    SampleClip(e, c, e->binding->clips[clipSlot], (float)s->time, out);
    // A mirrored entry plays mirrored (its map bound as the graph's).
    if ((m.flags & MANI_MATCH_MIRRORED) != 0)
    {
        (void)maniMirrorPose(e->binding->mirrorMap, e->binding->skeleton, out, out);
    }
}

// The character's place a spring node reads: its position and rotation
// parameters, or the origin and the identity; the rotation normalized as
// maniStepSprings normalizes it.
static maniTransform Placement(const maniGraphEvaluation* e, uint32_t data)
{
    maniTransform t = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    uint32_t position = maniGraphEvalWord(e, data + maniGraphSpringPosition);
    uint32_t rotation = maniGraphEvalWord(e, data + maniGraphSpringRotation);
    if (position != MANI_NO_PARAMETER)
    {
        const float* f = e->instance->cells[position].value.f;
        t.translation = (maniVec3){f[0], f[1], f[2]};
    }
    if (rotation != MANI_NO_PARAMETER)
    {
        const float* f = e->instance->cells[rotation].value.f;
        t.rotation = maniQuatNormalize((maniQuat){f[0], f[1], f[2], f[3]});
    }
    return t;
}

// A spring node: its input with its set's joints turned toward the tails
// its block steps to.
static void Spring(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out)
{
    const maniPose* input = maniGraphEvalInput(e, record, 0);
    if (input == NULL)
    {
        maniGraphEvalNeutral(e, out, false);
        return;
    }
    maniPoseCopy(out, input);
    uint32_t data = maniLoadU32(record + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)e->instance + e->instance->machinesAt) +
                      maniGraphEvalWord(e, data + maniGraphSpringBlock);
    const maniSpringSet* set =
        maniGraphBoundSpringSet(e->binding, maniGraphEvalWord(e, data + maniGraphSpringSet));
    maniTransform placement = Placement(e, data);
    uint32_t force = maniGraphEvalWord(e, data + maniGraphSpringForce);
    maniVec3 push = {0.0f, 0.0f, 0.0f};
    if (force != MANI_NO_PARAMETER)
    {
        const float* f = e->instance->cells[force].value.f;
        push = (maniVec3){f[0], f[1], f[2]};
    }
    uint32_t* work = (uint32_t*)((uint8_t*)e->instance + e->instance->roomAt);
    maniGraphSpringPose(set, e->binding->skeleton, block, work, &placement, push, out);
}

// A turn limit node: its input with each bone turned from the rotation
// its block holds toward the input's by at most its budget.
static void TurnLimit(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out)
{
    const maniPose* input = maniGraphEvalInput(e, record, 0);
    if (input == NULL)
    {
        maniGraphEvalNeutral(e, out, false);
        return;
    }
    maniPoseCopy(out, input);
    uint32_t data = maniLoadU32(record + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)e->instance + e->instance->machinesAt) +
                      maniGraphEvalWord(e, data + maniGraphTurnBlock);
    uint32_t mask = maniGraphEvalWord(e, data + maniGraphTurnMask);
    uint32_t bones = e->graph->skeletonBoneCount;
    if (mask != MANI_GRAPH_NO_MASK)
    {
        memcpy(e->inside, e->graph->maskWeights + 4 * (size_t)mask * bones, 4 * (size_t)bones);
    }
    // Under a subset, only the planned bones: the others' inputs are not
    // computed, and their held rotations wait for an evaluation that is.
    maniGraphTurnPose(block, mask != MANI_GRAPH_NO_MASK ? e->inside : NULL,
                      maniGraphEvalScalar(e, data + maniGraphTurnSpeed),
                      e->plan != NULL ? maniGraphSubsetList(e->plan) : NULL,
                      e->plan != NULL ? e->plan->count : 0, out);
}

// A learned node's pose: its model's Decompressor at its state, worked
// in the instance's room, the hidden values its update kept taken when
// kept at the same state (the same bits); the neutral pose with no state.
static void SampleLearned(const maniGraphEvaluation* e, uint32_t p, const uint8_t* record,
                          maniPose* out)
{
    uint32_t data = maniLoadU32(record + 12);
    const maniLearnedModel* m =
        maniGraphBoundModel(e->binding, maniGraphEvalWord(e, data + maniGraphLearnedModel));
    const uint32_t* block =
        (const uint32_t*)((const uint8_t*)e->instance + e->instance->machinesAt) +
        maniGraphEvalWord(e, data + maniGraphLearnedBlock);
    if (block[maniGraphLearnedHas] == 0)
    {
        maniGraphEvalNeutral(e, out, false);
        return;
    }
    float* room = (float*)((uint8_t*)e->instance + e->instance->roomAt);
    size_t scratchSize = maniLearnedScratchSize(m);
    float* state = room + scratchSize / 4;
    uint32_t states = m->featureCount + m->latentCount;
    maniGraphLearnedState(block, states, m->stepRate, state);
    const float* cache = maniGraphLearnedCache(e->instance, p);
    bool kept =
        cache != NULL && cache[0] != 0.0f && memcmp(cache + 1, state, sizeof(float) * states) == 0;
    maniLearnedOutput output = {out, NULL, NULL, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    maniResult r = maniLearnedDecompressFrom(m, state, &output, room, scratchSize,
                                             kept ? cache + 1 + states : NULL);
    if (r != mani_success)
    {
        maniGraphEvalNeutral(e, out, false);
    }
    if (r == mani_errorFormat)
    {
        maniGraphEvalState(e, p)->flags |= MANI_GRAPH_FAILED;
        *e->failed = 1;
    }
}

// A mirror node: its input's pose mirrored (both of the bound skeleton,
// the map bound to it).
static void Mirror(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out)
{
    const maniPose* input = maniGraphEvalInput(e, record, 0);
    const maniGraphBinding* b = e->binding;
    if (input == NULL || maniMirrorPose(b->mirrorMap, b->skeleton, input, out) != mani_success)
    {
        maniGraphEvalNeutral(e, out, false);
    }
}

// Input k passed on, or the neutral pose without it.
static void PassOn(const maniGraphEvaluation* e, const uint8_t* record, uint32_t k, maniPose* out,
                   bool difference)
{
    const maniPose* input = maniGraphEvalInput(e, record, k);
    if (input != NULL)
    {
        maniPoseCopy(out, input);
    }
    else
    {
        maniGraphEvalNeutral(e, out, difference);
    }
}

static void EvaluateNode(const maniGraphEvaluation* e, uint32_t p, uint32_t cursor)
{
    const uint8_t* record = maniGraphEvalRecord(e, p);
    maniPose* out = maniGraphEvalSlot(e, maniLoadU32(record + 20));
    bool difference = (maniLoadU16(record + 2) & MANI_GRAPH_DIFFERENCE) != 0;
    switch (maniLoadU16(record))
    {
    case maniGraphNodeClip:
        Sample(e, record, cursor, out, maniGraphEvalState(e, p));
        return;
    case maniGraphNodeMatch:
        SampleMatch(e, record, cursor, out, maniGraphEvalState(e, p));
        return;
    case maniGraphNodeLearned:
        SampleLearned(e, p, record, out);
        return;
    case maniGraphNodeBlend:
        Blend(e, record, out, difference);
        return;
    case maniGraphNodeMask:
        Mask(e, record, out, difference);
        return;
    case maniGraphNodeAdditive:
        Additive(e, record, out);
        return;
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceTriangles:
    case maniGraphNodeBlendSpaceBands:
        Space(e, record, out, difference);
        return;
    case maniGraphNodeOneShot:
        Shot(e, record, out, difference);
        return;
    case maniGraphNodeInertialize:
        Inertia(e, record, out, difference);
        return;
    case maniGraphNodeSpring:
        Spring(e, record, out);
        return;
    case maniGraphNodePose:
        maniGraphPoseInput(e->instance, maniGraphEvalWord(e, maniLoadU32(record + 12)), out);
        return;
    case maniGraphNodeTwoBone:
        maniGraphEvaluateTwoBone(e, record, out);
        return;
    case maniGraphNodeAim:
        maniGraphEvaluateAim(e, record, out);
        return;
    case maniGraphNodeCcd:
    case maniGraphNodeFabrik:
        maniGraphEvaluateChain(e, record, out, maniLoadU16(record) == maniGraphNodeFabrik);
        return;
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        maniGraphEvaluateBody(e, record, out, maniLoadU16(record) == maniGraphNodeBodySdls);
        return;
    case maniGraphNodeMachine:
        Machine(e, record, out, difference);
        return;
    case maniGraphNodeMirror:
        Mirror(e, record, out);
        return;
    case maniGraphNodeTurnLimit:
        TurnLimit(e, record, out);
        return;
    default:
        PassOn(e, record,
               maniLoadU16(record) == maniGraphNodeSelect ? maniGraphEvalState(e, p)->word : 0, out,
               difference);
        return;
    }
}

// Each planned bone of slot into pose, the pose made a pose (a graph's
// output never gives differences: refused at build).
static void WritePlanned(const maniGraphSubset* plan, const maniPose* slot, maniPose* pose)
{
    const uint32_t* bones = maniGraphSubsetList(plan);
    for (uint32_t i = 0; i < plan->count; ++i)
    {
        maniPoseSetLocal(pose, bones[i], maniPoseLocal(slot, bones[i]));
    }
    maniPoseSetKind(pose, false);
    maniPoseMarkStale(pose);
}

// The rest at each planned bone.
static void RestPlanned(const maniGraphSubset* plan, const maniSkeleton* skeleton, maniPose* pose)
{
    const uint32_t* bones = maniGraphSubsetList(plan);
    for (uint32_t i = 0; i < plan->count; ++i)
    {
        maniPoseSetLocal(pose, bones[i], maniSkeletonRest(skeleton, bones[i]));
    }
    maniPoseSetKind(pose, false);
    maniPoseMarkStale(pose);
}

// Evaluates every bone, or a plan's.
static maniResult Run(maniGraphInstance* instance, const maniGraphSubset* plan, void* scratch,
                      size_t size, maniPose* pose)
{
    if (instance == NULL || instance->stamp != MANI_GRAPH_INSTANCE_STAMP || scratch == NULL ||
        (uintptr_t)scratch % MANI_GRAPH_SCRATCH_ALIGNMENT != 0 ||
        !maniPoseMatches(pose, instance->binding->skeleton))
    {
        return mani_errorInvalid;
    }
    const maniGraphBinding* binding = instance->binding;
    uint32_t failed = 0;
    maniGraphEvaluation e = {instance,
                             binding,
                             &binding->graph,
                             scratch,
                             PartsOf(binding),
                             NULL,
                             NULL,
                             NULL,
                             plan,
                             plan != NULL ? maniGraphSubsetGroups(plan) : NULL,
                             &failed};
    if (size < e.parts.size)
    {
        return mani_errorCapacity;
    }
    void* layers = e.scratch + e.parts.layersAt;
    void* weights = e.scratch + e.parts.weightsAt;
    e.layers = layers;
    e.inside = weights;
    e.outside = e.inside + e.graph->skeletonBoneCount;
    uint32_t last = e.graph->nodeCount - 1;
    if (!(maniGraphEvalState(&e, last)->weight > 0.0f))
    {
        if (plan != NULL)
        {
            RestPlanned(plan, binding->skeleton, pose);
            return mani_success;
        }
        return maniPoseSetRest(pose, binding->skeleton);
    }
    Prepare(&e);
    uint32_t cursor = 0;
    for (uint32_t p = 0; p <= last; ++p)
    {
        const uint8_t* record = maniGraphEvalRecord(&e, p);
        uint32_t kind = maniLoadU16(record);
        bool clip = kind == maniGraphNodeClip || kind == maniGraphNodeMatch;
        bool weighed = maniGraphEvalState(&e, p)->weight > 0.0f;
        if (weighed && plan != NULL && !maniGraphSubsetRuns(plan, p))
        {
            // Writing no planned bone: its input on, its state untouched.
            PassOn(&e, record, 0, maniGraphEvalSlot(&e, maniLoadU32(record + 20)),
                   (maniLoadU16(record + 2) & MANI_GRAPH_DIFFERENCE) != 0);
        }
        else if (weighed)
        {
            EvaluateNode(&e, p, cursor);
        }
        cursor += clip ? 1u : 0u;
    }
    const maniPose* output = maniGraphEvalSlot(&e, maniLoadU32(maniGraphEvalRecord(&e, last) + 20));
    if (plan != NULL)
    {
        WritePlanned(plan, output, pose);
    }
    else
    {
        maniPoseCopy(pose, output);
    }
    return failed != 0 ? mani_errorFormat : mani_success;
}

static MANI_NOINLINE maniResult EvaluateGraphWork(maniGraphInstance* instance, void* scratch,
                                                  size_t size, maniPose* pose)
{
    return Run(instance, NULL, scratch, size, pose);
}

maniResult maniEvaluateGraph(maniGraphInstance* instance, void* scratch, size_t size,
                             maniPose* pose)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = EvaluateGraphWork(instance, scratch, size, pose);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult EvaluateGraphSubsetWork(maniGraphInstance* instance,
                                                        const maniGraphSubset* plan, void* scratch,
                                                        size_t size, maniPose* pose)
{
    if (plan == NULL || plan->stamp != MANI_GRAPH_SUBSET_STAMP || instance == NULL ||
        instance->stamp != MANI_GRAPH_INSTANCE_STAMP || plan->binding != instance->binding ||
        plan->identity != instance->binding->graph.identity)
    {
        return mani_errorInvalid;
    }
    return Run(instance, plan, scratch, size, pose);
}

maniResult maniEvaluateGraphSubset(maniGraphInstance* instance, const maniGraphSubset* plan,
                                   void* scratch, size_t size, maniPose* pose)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = EvaluateGraphSubsetWork(instance, plan, scratch, size, pose);
    maniFloatEnvLeave(&env);
    return result;
}
