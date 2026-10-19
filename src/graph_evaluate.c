// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Evaluating a graph instance (mani-0023): each weighed node in order
// writes its slot's pose from its inputs' slots, through a scratch of a
// pose per slot made once for the binding it last served.

#include "blend_groups.h"
#include "bytes.h"
#include "graph_body.h"
#include "graph_chain.h"
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
#include "ik_chain.h"
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

// Where the scratch's parts begin: layers, bone weights, the poses, then
// with map slots the room a mapped clip's pose is carried in (two
// source poses, a pose of the graph's skeleton, the transfer's scratch).
typedef struct Parts
{
    size_t layersAt;
    size_t weightsAt;
    size_t posesAt;
    size_t poseSize;
    size_t retargetAt;
    size_t size;
} Parts;

static Parts PartsOf(const maniGraphBinding* binding)
{
    Parts parts;
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

typedef struct Evaluation
{
    maniGraphInstance* instance;
    const maniGraphBinding* binding;
    const maniGraph* graph;
    uint8_t* scratch;
    Parts parts;
    maniBlendLayer* layers;
    float* inside;
    float* outside;
    // A subset's plan and its groups' flags, or NULL for every bone.
    const maniGraphSubset* plan;
    const uint8_t* groups;
} Evaluation;

static maniPose* Slot(const Evaluation* e, uint32_t slot)
{
    void* memory = e->scratch + e->parts.posesAt + (size_t)slot * e->parts.poseSize;
    return memory;
}

static const uint8_t* Record(const Evaluation* e, uint32_t p)
{
    return e->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
}

static maniGraphNodeState* State(const Evaluation* e, uint32_t p)
{
    return &e->instance->cells[e->graph->parameterCount + p].node;
}

static uint32_t Word(const Evaluation* e, uint32_t at)
{
    return maniLoadU32(e->graph->data + 4 * (size_t)at);
}

static float Scalar(const Evaluation* e, uint32_t at)
{
    uint32_t parameter = Word(e, at);
    if (parameter != MANI_NO_PARAMETER)
    {
        return e->instance->cells[parameter].value.f[0];
    }
    uint32_t bits = Word(e, at + 1);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static uint32_t InputOf(const Evaluation* e, const uint8_t* record, uint32_t k)
{
    return maniLoadU32(e->graph->inputs + 4 * (size_t)(maniLoadU32(record + 4) + k));
}

// An input's pose, or NULL when its last update left it unweighed.
static const maniPose* InputPose(const Evaluation* e, const uint8_t* record, uint32_t k)
{
    uint32_t input = InputOf(e, record, k);
    if (!(State(e, input)->weight > 0.0f))
    {
        return NULL;
    }
    return Slot(e, maniLoadU32(Record(e, input) + 20));
}

// The poses made for this binding, once.
static void Prepare(Evaluation* e)
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
        (void)maniInitPose(e->binding->skeleton, Slot(e, slot), e->parts.poseSize, &pose);
    }
    *head = (Head){SCRATCH_STAMP, e->graph->slotCount, e->binding, e->graph->identity};
}

// A pose of no weight: the rest, or the difference of nothing.
static void Neutral(const Evaluation* e, maniPose* out, bool difference)
{
    (void)maniPoseSetRest(out, e->binding->skeleton);
    if (difference)
    {
        (void)maniPoseDifference(e->binding->skeleton, out, out, out);
    }
}

static void Blend(const Evaluation* e, const uint8_t* record, maniPose* out, bool difference)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t n = maniLoadU32(record + 8);
    bool direct = Word(e, d) != 0;
    float total = 0.0f;
    for (uint32_t k = 0; k < n; ++k)
    {
        float v = Scalar(e, d + 1 + 2 * k);
        total += v > 0.0f ? v : 0.0f;
    }
    float scale = direct || total < MANI_BLEND_THRESHOLD ? 1.0f : 1.0f / total;
    uint32_t count = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        float v = Scalar(e, d + 1 + 2 * k) * scale;
        v = v > 0.0f ? (v < 1.0f ? v : 1.0f) : 0.0f;
        const maniPose* input = InputPose(e, record, k);
        if (v > 0.0f && input != NULL)
        {
            e->layers[count++] = (maniBlendLayer){input, v, NULL};
        }
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        Neutral(e, out, difference);
    }
}

// A blend space: its inputs at their weights, blended.
static void Space(const Evaluation* e, const uint8_t* record, maniPose* out, bool difference)
{
    uint32_t n = maniLoadU32(record + 8);
    float weights[MANI_GRAPH_MAX_SPACE_POINTS];
    maniGraphSpaceWeights(e->graph, e->instance->cells, maniLoadU16(record),
                          maniLoadU32(record + 12), n, weights);
    uint32_t count = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniPose* input = InputPose(e, record, k);
        if (weights[k] > 0.0f && input != NULL)
        {
            e->layers[count++] = (maniBlendLayer){input, weights[k], NULL};
        }
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        Neutral(e, out, difference);
    }
}

// A state machine: its states of weight above 0, blended by its weights.
static void Machine(const Evaluation* e, const uint8_t* record, maniPose* out, bool difference)
{
    maniGraphMachine m = {e->graph, maniLoadU32(record + 12), maniLoadU32(record + 8)};
    const uint32_t* block =
        (const uint32_t*)((const uint8_t*)e->instance + e->instance->machinesAt) +
        maniGraphMachineBlockAt(&m);
    uint32_t count = 0;
    for (uint32_t k = 0; k < m.states; ++k)
    {
        float weight = maniGraphMachineWeight(block, k);
        const maniPose* input = InputPose(e, record, k);
        if (weight > 0.0f && input != NULL)
        {
            e->layers[count++] = (maniBlendLayer){input, weight, NULL};
        }
    }
    if (count == 0 || maniBlendPosesIn(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                       out, e->groups) != mani_success)
    {
        Neutral(e, out, difference);
    }
}

static void Mask(const Evaluation* e, const uint8_t* record, maniPose* out, bool difference)
{
    uint32_t bones = e->graph->skeletonBoneCount;
    uint32_t mask = Word(e, maniLoadU32(record + 12));
    memcpy(e->inside, e->graph->maskWeights + 4 * (size_t)mask * bones, 4 * (size_t)bones);
    for (uint32_t b = 0; b < bones; ++b)
    {
        e->outside[b] = 1.0f - e->inside[b];
    }
    uint32_t count = 0;
    const maniPose* inside = InputPose(e, record, 0);
    const maniPose* outside = InputPose(e, record, 1);
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
        Neutral(e, out, difference);
    }
}

// A one-shot: per bone the shot at its weight times the mask's (1
// without one), the base at the rest, as a mask node of those weights.
static void Shot(const Evaluation* e, const uint8_t* record, maniPose* out, bool difference)
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
    const maniPose* base = InputPose(e, record, 0);
    const maniPose* played = InputPose(e, record, 1);
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
        Neutral(e, out, difference);
    }
}

// An inertialize node: its input eased by its block, which keeps the
// output.
static void Inertia(const Evaluation* e, const uint8_t* record, maniPose* out, bool difference)
{
    uint32_t data = maniLoadU32(record + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)e->instance + e->instance->machinesAt) +
                      Word(e, data + maniGraphInertiaBlock);
    const maniPose* input = InputPose(e, record, 0);
    if (input == NULL)
    {
        Neutral(e, out, difference);
        return;
    }
    maniGraphInertia node = maniGraphInertiaOf(e->graph, data);
    maniGraphInertiaEvaluate(block, &node, input, out);
}

static void Additive(const Evaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t d = maniLoadU32(record + 12);
    uint32_t n = maniLoadU32(record + 8);
    const maniPose* base = InputPose(e, record, 0);
    if (base == NULL)
    {
        Neutral(e, out, false);
        return;
    }
    maniPoseCopy(out, base);
    uint32_t count = 0;
    for (uint32_t k = 1; k < n; ++k)
    {
        float v = Scalar(e, d + 2 * (k - 1));
        v = v > 0.0f ? (v < 1.0f ? v : 1.0f) : 0.0f;
        const maniPose* layer = InputPose(e, record, k);
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
static void SampleMapped(const Evaluation* e, const maniGraphBoundMapPair* pair, maniClipCursor* c,
                         const maniClip* clip, float time, maniPose* out)
{
    uint8_t* at = e->scratch + e->parts.retargetAt;
    size_t sourceSize = e->binding->sourcePoseSize;
    maniPose* poses[3] = {(void*)at, (void*)(at + sourceSize), (void*)(at + 2 * sourceSize)};
    void* scratch = at + 2 * sourceSize + e->parts.poseSize;
    const maniSkeleton* skeleton = e->binding->skeleton;
    if (!maniPoseMatches(poses[0], pair->source) || !maniPoseMatches(poses[1], pair->source) ||
        !maniPoseMatches(poses[2], skeleton))
    {
        (void)maniInitPose(pair->source, poses[0], sourceSize, &poses[0]);
        (void)maniInitPose(pair->source, poses[1], sourceSize, &poses[1]);
        (void)maniInitPose(skeleton, poses[2], e->parts.poseSize, &poses[2]);
    }
    size_t size = e->binding->retargetScratchSize;
    (void)maniSampleClipCursor(c, clip, pair->source, time, poses[0]);
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
// by random access, the same bits (mani-0018's cursor), the pose made of
// the clip's kind first.
static void SampleClip(const Evaluation* e, maniClipCursor* c, const maniClip* clip, float time,
                       maniPose* out)
{
    if (e->plan == NULL)
    {
        (void)maniSampleClipCursor(c, clip, e->binding->skeleton, time, out);
        return;
    }
    maniPoseSetKind(out, maniClipAdditiveBasis(clip) != maniAdditiveNone);
    (void)maniSampleClipBones(clip, e->binding->skeleton, time, maniGraphSubsetList(e->plan),
                              e->plan->count, out);
}

// A clip node's pose: its clip at its playhead, through its cursor (and
// its map).
static void Sample(const Evaluation* e, const uint8_t* record, uint32_t cursor, maniPose* out,
                   const maniGraphNodeState* s)
{
    uint32_t slot = Word(e, maniLoadU32(record + 12));
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
static void SampleMatch(const Evaluation* e, const uint8_t* record, uint32_t cursor, maniPose* out,
                        const maniGraphNodeState* s)
{
    uint32_t data = maniLoadU32(record + 12);
    uint32_t slot = Word(e, data + maniGraphMatchDatabase);
    uint32_t entry =
        ((const uint32_t*)((const uint8_t*)e->instance +
                           e->instance->machinesAt))[Word(e, data + maniGraphMatchBlock) +
                                                     maniGraphMatchEntry];
    maniMatchEntry m;
    if (maniMotionDatabaseEntry(maniGraphBoundDatabase(e->binding, slot), entry, &m) !=
        mani_success)
    {
        Neutral(e, out, false);
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
static maniTransform Placement(const Evaluation* e, uint32_t data)
{
    maniTransform t = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    uint32_t position = Word(e, data + maniGraphSpringPosition);
    uint32_t rotation = Word(e, data + maniGraphSpringRotation);
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
static void Spring(const Evaluation* e, const uint8_t* record, maniPose* out)
{
    const maniPose* input = InputPose(e, record, 0);
    if (input == NULL)
    {
        Neutral(e, out, false);
        return;
    }
    maniPoseCopy(out, input);
    uint32_t data = maniLoadU32(record + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)e->instance + e->instance->machinesAt) +
                      Word(e, data + maniGraphSpringBlock);
    const maniSpringSet* set =
        maniGraphBoundSpringSet(e->binding, Word(e, data + maniGraphSpringSet));
    maniTransform placement = Placement(e, data);
    uint32_t force = Word(e, data + maniGraphSpringForce);
    maniVec3 push = {0.0f, 0.0f, 0.0f};
    if (force != MANI_NO_PARAMETER)
    {
        const float* f = e->instance->cells[force].value.f;
        push = (maniVec3){f[0], f[1], f[2]};
    }
    uint32_t* work = (uint32_t*)((uint8_t*)e->instance + e->instance->roomAt);
    maniGraphSpringPose(set, e->binding->skeleton, block, work, &placement, push, out);
}

// A vec3 parameter's value, or zero for none.
static maniVec3 Vector(const Evaluation* e, uint32_t parameter)
{
    if (parameter == MANI_NO_PARAMETER)
    {
        return (maniVec3){0.0f, 0.0f, 0.0f};
    }
    const float* f = e->instance->cells[parameter].value.f;
    return (maniVec3){f[0], f[1], f[2]};
}

// A quat parameter's value, or no turn for none.
static maniQuat Rotation(const Evaluation* e, uint32_t parameter)
{
    if (parameter == MANI_NO_PARAMETER)
    {
        return (maniQuat){0.0f, 0.0f, 0.0f, 1.0f};
    }
    const float* f = e->instance->cells[parameter].value.f;
    return (maniQuat){f[0], f[1], f[2], f[3]};
}

// A binary32 data word.
static float Number(const Evaluation* e, uint32_t at)
{
    uint32_t bits = Word(e, at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// Three binary32 data words from at.
static maniVec3 Axis(const Evaluation* e, uint32_t at)
{
    float f[3];
    for (uint32_t k = 0; k < 3; ++k)
    {
        uint32_t bits = Word(e, at + k);
        memcpy(&f[k], &bits, sizeof bits);
    }
    return (maniVec3){f[0], f[1], f[2]};
}

// A scalar clamped from low to high (a parameter's value may lie past).
static float Clamped(const Evaluation* e, uint32_t at, float low, float high)
{
    float v = Scalar(e, at);
    return v < low ? low : (v > high ? high : v);
}

// The input of an IK node copied to out; false with none (out then
// neutral) or a weight of 0 at data word weightAt.
static bool IkInput(const Evaluation* e, const uint8_t* record, uint32_t weightAt, maniPose* out)
{
    const maniPose* input = InputPose(e, record, 0);
    if (input == NULL)
    {
        Neutral(e, out, false);
        return false;
    }
    maniPoseCopy(out, input);
    return Clamped(e, weightAt, 0.0f, 1.0f) > 0.0f;
}

// A two-bone node: its input with its chain solved; the input when the
// solve refuses (a crafted file's bones not each an ancestor of the
// next).
static void TwoBone(const Evaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphTwoBoneWeight, out))
    {
        return;
    }
    maniTwoBoneSolve s = {Word(e, d),
                          Word(e, d + 1),
                          Word(e, d + 2),
                          0,
                          Vector(e, Word(e, d + maniGraphTwoBoneGoal)),
                          Vector(e, Word(e, d + maniGraphTwoBonePole)),
                          Axis(e, d + maniGraphTwoBoneHinge),
                          Scalar(e, d + maniGraphTwoBoneTwist),
                          Clamped(e, d + maniGraphTwoBoneSoften, 0.0f, 1.0f),
                          Clamped(e, d + maniGraphTwoBoneWeight, 0.0f, 1.0f)};
    (void)maniSolveTwoBone(out, e->binding->skeleton, &s, NULL);
}

// An aim node: its input with its bones aimed; the input when the solve
// refuses.
static void Aim(const Evaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphAimWeight, out))
    {
        return;
    }
    maniAimSolve s = {.count = Word(e, d),
                      .goal = Vector(e, Word(e, d + maniGraphAimGoal)),
                      .pole = Vector(e, Word(e, d + maniGraphAimPole)),
                      .forward = Axis(e, d + maniGraphAimForward),
                      .offset = Axis(e, d + maniGraphAimOffset),
                      .up = Axis(e, d + maniGraphAimUp),
                      .twist = Scalar(e, d + maniGraphAimTwist),
                      .cone = Clamped(e, d + maniGraphAimCone, 0.0f, MANI_AIM_MAX_CONE),
                      .weight = Clamped(e, d + maniGraphAimWeight, 0.0f, 1.0f)};
    for (uint32_t k = 0; k < MANI_AIM_MAX_BONES; ++k)
    {
        uint32_t bits = Word(e, d + maniGraphAimShares + k);
        s.bones[k] = Word(e, d + maniGraphAimBones + k);
        memcpy(&s.shares[k], &bits, sizeof bits);
    }
    (void)maniSolveAim(out, e->binding->skeleton, &s, NULL);
}

// A chain node: its input with its chain solved; the input when the
// solve refuses (a crafted file's bones not a chain) or its limits are
// not one per turning bone of the bound skeleton's path.
static void Chain(const Evaluation* e, const uint8_t* record, maniPose* out, bool fabrik)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphChainWeight, out))
    {
        return;
    }
    const maniSkeleton* skeleton = e->binding->skeleton;
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t path = maniIkChainPath(skeleton, Word(e, d), Word(e, d + 1), bones);
    uint32_t count = Word(e, d + maniGraphChainLimitCount);
    if (count != 0 && count + 1 != path)
    {
        return;
    }
    maniJointLimit limits[MANI_IK_CHAIN_MAX_BONES] = {0};
    for (uint32_t k = 0; k < count; ++k)
    {
        limits[k] = maniGraphChainLimit(e->graph, d, k);
    }
    uint32_t rotation = Word(e, d + maniGraphChainRotation);
    maniChainSolve s = {.root = Word(e, d),
                        .end = Word(e, d + 1),
                        .limits = count > 0 ? limits : NULL,
                        .goal = Vector(e, Word(e, d + maniGraphChainGoal)),
                        .pole = Vector(e, Word(e, d + maniGraphChainPole)),
                        .rotation = Rotation(e, rotation),
                        .iterations = Word(e, d + maniGraphChainIterations),
                        .tolerance = Number(e, d + maniGraphChainTolerance),
                        .step = Number(e, d + maniGraphChainStep),
                        .rotationWeight =
                            rotation != MANI_NO_PARAMETER
                                ? Clamped(e, d + maniGraphChainRotationWeight, 0.0f, 1.0f)
                                : 0.0f,
                        .weight = Clamped(e, d + maniGraphChainWeight, 0.0f, 1.0f)};
    (void)(fabrik ? maniSolveFabrik(out, skeleton, &s, NULL)
                  : maniSolveCcd(out, skeleton, &s, NULL));
}

// Effector k of the body node whose words begin at d, its goals and
// weights the parameters' values now.
static maniBodyEffector Effector(const Evaluation* e, uint32_t d, uint32_t k)
{
    uint32_t at = maniGraphBodyEffectorAt(d, k);
    return (maniBodyEffector){
        .bone = Word(e, at + maniGraphEffectorBone),
        .position = Vector(e, Word(e, at + maniGraphEffectorPosition)),
        .rotation = Rotation(e, Word(e, at + maniGraphEffectorRotation)),
        .positionWeight = Clamped(e, at + maniGraphEffectorPositionWeight, 0.0f, 1.0f),
        .rotationWeight = Clamped(e, at + maniGraphEffectorRotationWeight, 0.0f, 1.0f),
        .strength = Number(e, at + maniGraphEffectorStrength),
        .pullChain = Number(e, at + maniGraphEffectorPull)};
}

// A full-body node: its input with the body solved, its effectors,
// settings and the solve's scratch laid in the instance's room from a
// 16-byte boundary; the input when the solve refuses (a crafted file's
// bones not a part the solve takes, a goal not finite, a room short).
static void Body(const Evaluation* e, const uint8_t* record, maniPose* out, bool sdls)
{
    uint32_t d = maniLoadU32(record + 12);
    if (!IkInput(e, record, d + maniGraphBodyWeight, out))
    {
        return;
    }
    uint32_t count = Word(e, d + maniGraphBodyEffectorCount);
    uint32_t settings = Word(e, d + maniGraphBodySettingCount);
    uint8_t* room = (uint8_t*)e->instance + e->instance->roomAt;
    size_t at = (16u - (uintptr_t)room % 16u) % 16u;
    maniBodyEffector* effectors = (void*)(room + at);
    at += maniGraphBodyRounded(count * sizeof(maniBodyEffector));
    maniBodyBone* bones = (void*)(room + at);
    at += maniGraphBodyRounded(settings * sizeof(maniBodyBone));
    // The room holds every body node's arrays (maniGraphBodyRoom); the
    // check keeps a room short of them from being written past.
    size_t have = e->instance->size - e->instance->roomAt;
    if (at > have)
    {
        return;
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        effectors[k] = Effector(e, d, k);
    }
    for (uint32_t k = 0; k < settings; ++k)
    {
        bones[k] = maniGraphBodySetting(e->graph, d, k);
    }
    const maniBodySolve s = {.top = Word(e, d + maniGraphBodyTop),
                             .topMode = (maniBodyTopMode)Word(e, d + maniGraphBodyMode),
                             .effectors = effectors,
                             .bones = bones,
                             .effectorCount = count,
                             .boneCount = settings,
                             .iterations = Word(e, d + maniGraphBodyIterations),
                             .stretch = Word(e, d + maniGraphBodyStretch) != 0,
                             .tolerance = Number(e, d + maniGraphBodyTolerance),
                             .weight = Clamped(e, d + maniGraphBodyWeight, 0.0f, 1.0f)};
    const maniSkeleton* skeleton = e->binding->skeleton;
    (void)(sdls ? maniSolveBodySdls(out, skeleton, &s, room + at, have - at, NULL)
                : maniSolveBodyXpbd(out, skeleton, &s, room + at, have - at, NULL));
}

// A learned node's pose: its model's Decompressor at its state, worked
// in the instance's room; the neutral pose with no state.
static void SampleLearned(const Evaluation* e, const uint8_t* record, maniPose* out)
{
    uint32_t data = maniLoadU32(record + 12);
    const maniLearnedModel* m =
        maniGraphBoundModel(e->binding, Word(e, data + maniGraphLearnedModel));
    const uint32_t* block =
        (const uint32_t*)((const uint8_t*)e->instance + e->instance->machinesAt) +
        Word(e, data + maniGraphLearnedBlock);
    if (block[maniGraphLearnedHas] == 0)
    {
        Neutral(e, out, false);
        return;
    }
    float* room = (float*)((uint8_t*)e->instance + e->instance->roomAt);
    size_t scratchSize = maniLearnedScratchSize(m);
    float* state = room + scratchSize / 4;
    maniGraphLearnedState(block, m->featureCount + m->latentCount, m->stepRate, state);
    maniLearnedOutput output = {out, NULL, NULL, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    if (maniLearnedDecompress(m, state, &output, room, scratchSize) != mani_success)
    {
        Neutral(e, out, false);
    }
}

// A mirror node: its input's pose mirrored (both of the bound skeleton,
// the map bound to it).
static void Mirror(const Evaluation* e, const uint8_t* record, maniPose* out)
{
    const maniPose* input = InputPose(e, record, 0);
    const maniGraphBinding* b = e->binding;
    if (input == NULL || maniMirrorPose(b->mirrorMap, b->skeleton, input, out) != mani_success)
    {
        Neutral(e, out, false);
    }
}

static void EvaluateNode(const Evaluation* e, uint32_t p, uint32_t cursor)
{
    const uint8_t* record = Record(e, p);
    maniPose* out = Slot(e, maniLoadU32(record + 20));
    bool difference = (maniLoadU16(record + 2) & MANI_GRAPH_DIFFERENCE) != 0;
    switch (maniLoadU16(record))
    {
    case maniGraphNodeClip:
        Sample(e, record, cursor, out, State(e, p));
        return;
    case maniGraphNodeMatch:
        SampleMatch(e, record, cursor, out, State(e, p));
        return;
    case maniGraphNodeLearned:
        SampleLearned(e, record, out);
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
        maniGraphPoseInput(e->instance, Word(e, maniLoadU32(record + 12)), out);
        return;
    case maniGraphNodeTwoBone:
        TwoBone(e, record, out);
        return;
    case maniGraphNodeAim:
        Aim(e, record, out);
        return;
    case maniGraphNodeCcd:
    case maniGraphNodeFabrik:
        Chain(e, record, out, maniLoadU16(record) == maniGraphNodeFabrik);
        return;
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        Body(e, record, out, maniLoadU16(record) == maniGraphNodeBodySdls);
        return;
    case maniGraphNodeMachine:
        Machine(e, record, out, difference);
        return;
    case maniGraphNodeMirror:
        Mirror(e, record, out);
        return;
    default:
    {
        uint32_t k = maniLoadU16(record) == maniGraphNodeSelect ? State(e, p)->word : 0;
        const maniPose* input = InputPose(e, record, k);
        if (input != NULL)
        {
            maniPoseCopy(out, input);
        }
        else
        {
            Neutral(e, out, difference);
        }
        return;
    }
    }
}

// Each planned bone of slot into pose, the pose made of slot's kind.
static void WritePlanned(const maniGraphSubset* plan, const maniPose* slot, maniPose* pose)
{
    const uint32_t* bones = maniGraphSubsetList(plan);
    for (uint32_t i = 0; i < plan->count; ++i)
    {
        maniPoseSetLocal(pose, bones[i], maniPoseLocal(slot, bones[i]));
    }
    maniPoseSetKind(pose, maniPoseIsDifference(slot));
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
    Evaluation e = {instance,
                    binding,
                    &binding->graph,
                    scratch,
                    PartsOf(binding),
                    NULL,
                    NULL,
                    NULL,
                    plan,
                    plan != NULL ? maniGraphSubsetGroups(plan) : NULL};
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
    if (!(State(&e, last)->weight > 0.0f))
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
        uint32_t kind = maniLoadU16(Record(&e, p));
        bool clip = kind == maniGraphNodeClip || kind == maniGraphNodeMatch;
        if (State(&e, p)->weight > 0.0f)
        {
            EvaluateNode(&e, p, cursor);
        }
        cursor += clip ? 1u : 0u;
    }
    const maniPose* output = Slot(&e, maniLoadU32(Record(&e, last) + 20));
    if (plan != NULL)
    {
        WritePlanned(plan, output, pose);
    }
    else
    {
        maniPoseCopy(pose, output);
    }
    return mani_success;
}

maniResult maniEvaluateGraph(maniGraphInstance* instance, void* scratch, size_t size,
                             maniPose* pose)
{
    return Run(instance, NULL, scratch, size, pose);
}

maniResult maniEvaluateGraphSubset(maniGraphInstance* instance, const maniGraphSubset* plan,
                                   void* scratch, size_t size, maniPose* pose)
{
    if (plan == NULL || plan->stamp != MANI_GRAPH_SUBSET_STAMP || instance == NULL ||
        instance->stamp != MANI_GRAPH_INSTANCE_STAMP || plan->binding != instance->binding ||
        plan->identity != instance->binding->graph.identity)
    {
        return mani_errorInvalid;
    }
    return Run(instance, plan, scratch, size, pose);
}
