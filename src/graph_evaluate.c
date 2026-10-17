// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Evaluating a graph instance (mani-0023): each weighed node in order
// writes its slot's pose from its inputs' slots, through a scratch of a
// pose per slot made once for the binding it last served.

#include "bytes.h"
#include "graph_file.h"
#include "graph_inertia_apply.h"
#include "graph_instance.h"
#include "graph_learned.h"
#include "graph_machine.h"
#include "graph_match.h"
#include "graph_shot.h"
#include "graph_space.h"
#include "graph_spring.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/pose.h"
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

// Where the scratch's parts begin: layers, bone weights, then the poses.
typedef struct Parts
{
    size_t layersAt;
    size_t weightsAt;
    size_t posesAt;
    size_t poseSize;
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
    if (count == 0 || maniBlendPoses(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                     out) != mani_success)
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
    if (count == 0 || maniBlendPoses(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                     out) != mani_success)
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
    if (count == 0 || maniBlendPoses(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                     out) != mani_success)
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
    if (count == 0 || maniBlendPoses(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                     out) != mani_success)
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
    if (count == 0 || maniBlendPoses(e->binding->skeleton, e->layers, count, MANI_BLEND_THRESHOLD,
                                     out) != mani_success)
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
    (void)maniAddPoses(e->binding->skeleton, out, e->layers, count);
}

// A clip node's pose: its clip at its playhead, through its cursor.
static void Sample(const Evaluation* e, const uint8_t* record, uint32_t cursor, maniPose* out,
                   const maniGraphNodeState* s)
{
    const maniClip* clip = e->binding->clips[Word(e, maniLoadU32(record + 12))];
    void* memory =
        (uint8_t*)e->instance + e->instance->cursorsAt + (size_t)cursor * e->binding->cursorSize;
    maniClipCursor* c = memory;
    (void)maniSampleClipCursor(c, clip, e->binding->skeleton, (float)s->time, out);
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
    (void)maniSampleClipCursor(c, e->binding->clips[clipSlot], e->binding->skeleton, (float)s->time,
                               out);
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
    uint32_t* work = (uint32_t*)((uint8_t*)e->instance + e->instance->roomAt);
    maniGraphSpringPose(set, e->binding->skeleton, block, work, &placement, out);
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
    case maniGraphNodeMachine:
        Machine(e, record, out, difference);
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

maniResult maniEvaluateGraph(maniGraphInstance* instance, void* scratch, size_t size,
                             maniPose* pose)
{
    if (instance == NULL || instance->stamp != MANI_GRAPH_INSTANCE_STAMP || scratch == NULL ||
        (uintptr_t)scratch % MANI_GRAPH_SCRATCH_ALIGNMENT != 0 ||
        !maniPoseMatches(pose, instance->binding->skeleton))
    {
        return mani_errorInvalid;
    }
    const maniGraphBinding* binding = instance->binding;
    Evaluation e = {instance, binding, &binding->graph, scratch, PartsOf(binding), NULL,
                    NULL,     NULL};
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
    maniPoseCopy(pose, Slot(&e, maniLoadU32(Record(&e, last) + 20)));
    return mani_success;
}
