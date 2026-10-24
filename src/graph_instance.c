// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph instances: made in caller memory, their parameters
// set and read, their state saved as bytes without pointers and
// restored after checking those bytes as external input.

#include "graph_instance.h"

#include "bytes.h"
#include "fpenv.h"
#include "graph_body.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_learned.h"
#include "graph_machine.h"
#include "graph_match.h"
#include "graph_shot.h"
#include "graph_spring.h"
#include "graph_turn.h"
#include "graph_values.h"
#include "learned_eval.h"
#include "pose_memory.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static bool InstanceValid(const maniGraphInstance* instance)
{
    return instance != NULL && instance->stamp == MANI_GRAPH_INSTANCE_STAMP;
}

static bool BindingValid(const maniGraphBinding* binding)
{
    return binding != NULL && binding->stamp == MANI_GRAPH_BINDING_STAMP;
}

// The nodes that sample clips through a cursor: clip and motion matching
// nodes.
static uint32_t ClipNodeCount(const maniGraph* graph)
{
    uint32_t count = 0;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        uint32_t kind = maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE);
        count += kind == maniGraphNodeClip || kind == maniGraphNodeMatch ? 1u : 0u;
    }
    return count;
}

static size_t CursorsAt(const maniGraph* graph)
{
    size_t cells = (size_t)graph->parameterCount + graph->nodeCount + graph->syncGroupCount;
    return offsetof(maniGraphInstance, cells) + cells * sizeof(maniGraphCell);
}

static size_t MachinesAt(const maniGraphBinding* binding)
{
    const maniGraph* graph = &binding->graph;
    return CursorsAt(graph) + (size_t)ClipNodeCount(graph) * binding->cursorSize;
}

static const uint8_t* ParameterEntry(const maniGraph* graph, uint32_t parameter)
{
    return graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE;
}

static size_t PosesAt(const maniGraphBinding* binding)
{
    size_t words = binding->graph.machineWords;
    return MachinesAt(binding) + ((4 * words + 7u) & ~(size_t)7u);
}

// The bytes a pose parameter's locals take: the skeleton's groups.
static size_t PoseBytes(const maniGraphBinding* binding)
{
    size_t groups = ((size_t)maniSkeletonBoneCount(binding->skeleton) + MANI_POSE_GROUP_BONES - 1) /
                    MANI_POSE_GROUP_BONES;
    return groups * MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS * sizeof(float);
}

// The number of pose parameters before parameter (all of them for the
// count).
static uint32_t PosesBefore(const maniGraph* graph, uint32_t parameter)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < parameter; ++i)
    {
        const uint8_t* e = graph->parameters + (size_t)i * MANI_GRAPH_PARAMETER_SIZE;
        n += maniLoadU32(e) == maniGraphParameterPose ? 1u : 0u;
    }
    return n;
}

static size_t RoomAt(const maniGraphBinding* binding)
{
    const maniGraph* graph = &binding->graph;
    return PosesAt(binding) + PosesBefore(graph, graph->parameterCount) * PoseBytes(binding);
}

float* maniGraphPoseLocals(maniGraphInstance* instance, uint32_t parameter)
{
    const maniGraphBinding* b = instance->binding;
    size_t at = instance->posesAt + PosesBefore(&b->graph, parameter) * PoseBytes(b);
    return (float*)((uint8_t*)instance + at);
}

const float* maniGraphPoseLocalsRead(const maniGraphInstance* instance, uint32_t parameter)
{
    const maniGraphBinding* b = instance->binding;
    size_t at = instance->posesAt + PosesBefore(&b->graph, parameter) * PoseBytes(b);
    return (const float*)((const uint8_t*)instance + at);
}

// Every pose parameter at the skeleton's rest.
static void RestPoses(maniGraphInstance* instance)
{
    const maniGraphBinding* b = instance->binding;
    uint32_t bones = maniSkeletonBoneCount(b->skeleton);
    for (uint32_t i = 0; i < b->graph.parameterCount; ++i)
    {
        if (maniLoadU32(ParameterEntry(&b->graph, i)) != maniGraphParameterPose)
        {
            continue;
        }
        float* locals = maniGraphPoseLocals(instance, i);
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            maniTransform t = maniSkeletonRest(b->skeleton, bone);
            const float values[MANI_POSE_COMPONENTS] = {
                t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y,
                t.rotation.z,    t.rotation.w,    t.scale.x,       t.scale.y,    t.scale.z};
            maniPoseSlotPut(maniPoseSlot(locals, bone), 0, values, MANI_POSE_COMPONENTS);
        }
    }
}

// A learned node's model.
static const maniLearnedModel* LearnedModel(const maniGraphBinding* binding, uint32_t data)
{
    const maniGraph* graph = &binding->graph;
    return maniGraphBoundModel(
        binding, maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphLearnedModel)));
}

// A learned node's cache in floats: a word set once kept (0 from the
// instance's start), the state it was kept at, then its Decompressor's
// hidden values; none for a Decompressor of one layer.
static size_t CacheFloats(const maniLearnedModel* m)
{
    uint32_t hidden = maniLearnedHiddenCount(m);
    return hidden > 0 ? 1u + (size_t)m->featureCount + m->latentCount + hidden : 0u;
}

// The floats of every learned node's cache.
static size_t Caches(const maniGraphBinding* binding)
{
    const maniGraph* graph = &binding->graph;
    size_t floats = 0;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        if (maniLoadU16(e) == maniGraphNodeLearned)
        {
            floats += CacheFloats(LearnedModel(binding, maniLoadU32(e + 12)));
        }
    }
    return floats;
}

// Each node's cache's place in floats after the table (UINT32_MAX for
// none), into the table at cachesAt; nothing without caches.
static void PlaceCaches(maniGraphInstance* instance)
{
    const maniGraphBinding* binding = instance->binding;
    const maniGraph* graph = &binding->graph;
    if (instance->cachesAt == instance->size)
    {
        return;
    }
    uint32_t* table = (uint32_t*)((uint8_t*)instance + instance->cachesAt);
    uint32_t at = 0;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        size_t floats = maniLoadU16(e) == maniGraphNodeLearned
                            ? CacheFloats(LearnedModel(binding, maniLoadU32(e + 12)))
                            : 0u;
        table[p] = floats > 0 ? at : UINT32_MAX;
        at += (uint32_t)floats;
    }
}

float* maniGraphLearnedCache(maniGraphInstance* instance, uint32_t node)
{
    if (instance->cachesAt == instance->size)
    {
        return NULL;
    }
    uint32_t* table = (uint32_t*)((uint8_t*)instance + instance->cachesAt);
    uint32_t at = table[node];
    return at != UINT32_MAX ? (float*)(table + instance->binding->graph.nodeCount) + at : NULL;
}

size_t maniGraphRoom(const maniGraphBinding* binding)
{
    const maniGraph* graph = &binding->graph;
    size_t room = 0;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        uint32_t data = maniLoadU32(e + 12);
        size_t need = 0;
        if (maniLoadU16(e) == maniGraphNodeLearned)
        {
            const maniLearnedModel* m = LearnedModel(binding, data);
            size_t tail =
                (size_t)m->outputCount - 9 * (size_t)m->boneCount - 3 * (size_t)m->scaledBoneCount;
            need = maniLearnedScratchSize(m) +
                   sizeof(float) * ((size_t)m->featureCount + m->latentCount + tail);
        }
        else if (maniLoadU16(e) == maniGraphNodeSpring)
        {
            uint32_t slot = maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphSpringSet));
            need = 4 * (size_t)maniGraphSpringWorkWords(maniGraphBoundSpringSet(binding, slot));
        }
        else if (maniLoadU16(e) == maniGraphNodeBodyXpbd || maniLoadU16(e) == maniGraphNodeBodySdls)
        {
            need = maniGraphBodyRoom(graph, data, binding->skeleton);
        }
        room = need > room ? need : room;
    }
    return (room + 7u) & ~(size_t)7u;
}

size_t maniGraphInstanceSize(const maniGraphBinding* binding)
{
    if (!BindingValid(binding))
    {
        return 0;
    }
    // The caches after a table of their places, one word a node, when any
    // node has one.
    size_t caches = Caches(binding);
    size_t table = caches > 0 ? binding->graph.nodeCount : 0u;
    return RoomAt(binding) + maniGraphRoom(binding) + sizeof(float) * (table + caches);
}

// The machine node p's view, or none (states 0) for another kind.
static maniGraphMachine MachineAt(const maniGraph* graph, uint32_t p)
{
    const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
    bool machine = maniLoadU16(e) == maniGraphNodeMachine;
    return (maniGraphMachine){graph, maniLoadU32(e + 12), machine ? maniLoadU32(e + 8) : 0u};
}

static uint32_t* Blocks(maniGraphInstance* instance)
{
    return (uint32_t*)((uint8_t*)instance + instance->machinesAt);
}

// The one-shot, inertialize, motion matching, learned or spring node p's
// block's first word, or UINT32_MAX for another kind.
static uint32_t ShotBlockAt(const maniGraph* graph, uint32_t p)
{
    const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
    maniGraphShot shot = {graph, maniLoadU32(e + 12), 0};
    if (maniLoadU16(e) == maniGraphNodeInertialize)
    {
        return maniLoadU32(graph->data + 4 * (size_t)(maniLoadU32(e + 12) + maniGraphInertiaBlock));
    }
    if (maniLoadU16(e) == maniGraphNodeMatch)
    {
        return maniLoadU32(graph->data + 4 * (size_t)(maniLoadU32(e + 12) + maniGraphMatchBlock));
    }
    if (maniLoadU16(e) == maniGraphNodeLearned)
    {
        return maniLoadU32(graph->data + 4 * (size_t)(maniLoadU32(e + 12) + maniGraphLearnedBlock));
    }
    if (maniLoadU16(e) == maniGraphNodeSpring)
    {
        return maniLoadU32(graph->data + 4 * (size_t)(maniLoadU32(e + 12) + maniGraphSpringBlock));
    }
    if (maniLoadU16(e) == maniGraphNodeTurnLimit)
    {
        return maniLoadU32(graph->data + 4 * (size_t)(maniLoadU32(e + 12) + maniGraphTurnBlock));
    }
    return maniLoadU16(e) == maniGraphNodeOneShot ? maniGraphShotWord(&shot, maniGraphShotBlock)
                                                  : UINT32_MAX;
}

static bool Inertial(const maniGraph* graph, uint32_t p)
{
    return maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) == maniGraphNodeInertialize;
}

static bool Matching(const maniGraph* graph, uint32_t p)
{
    return maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) == maniGraphNodeMatch;
}

static bool Learning(const maniGraph* graph, uint32_t p)
{
    return maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) == maniGraphNodeLearned;
}

static bool Springing(const maniGraph* graph, uint32_t p)
{
    return maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) == maniGraphNodeSpring;
}

static bool Turning(const maniGraph* graph, uint32_t p)
{
    return maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) == maniGraphNodeTurnLimit;
}

static uint32_t DataOf(const maniGraph* graph, uint32_t p)
{
    return maniLoadU32(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
}

// Inertialize node p as its words give it.
static maniGraphInertia InertiaAt(const maniGraph* graph, uint32_t p)
{
    return maniGraphInertiaOf(graph,
                              maniLoadU32(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12));
}

static void ResetMachines(maniGraphInstance* instance)
{
    const maniGraph* graph = &instance->binding->graph;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        maniGraphMachine m = MachineAt(graph, p);
        uint32_t shot = ShotBlockAt(graph, p);
        if (m.states > 0)
        {
            maniGraphResetMachine(&m, Blocks(instance) + maniGraphMachineBlockAt(&m));
        }
        else if (shot != UINT32_MAX && Inertial(graph, p))
        {
            maniGraphInertia node = InertiaAt(graph, p);
            maniGraphResetInertia(Blocks(instance) + shot, &node);
        }
        else if (shot != UINT32_MAX && Matching(graph, p))
        {
            maniGraphResetMatch(Blocks(instance) + shot);
        }
        else if (shot != UINT32_MAX && Learning(graph, p))
        {
            maniGraphResetLearned(graph, DataOf(graph, p), Blocks(instance) + shot);
        }
        else if (shot != UINT32_MAX && Springing(graph, p))
        {
            maniGraphResetSpring(graph, DataOf(graph, p), Blocks(instance) + shot);
        }
        else if (shot != UINT32_MAX && Turning(graph, p))
        {
            maniGraphResetTurn(Blocks(instance) + shot, graph->skeletonBoneCount);
        }
        else if (shot != UINT32_MAX)
        {
            maniGraphResetShot(Blocks(instance) + shot);
        }
    }
}

// Every cursor made over again, holding nothing.
static void ResetCursors(maniGraphInstance* instance)
{
    size_t size = instance->binding->cursorSize;
    for (uint32_t c = 0; c < instance->clipNodeCount; ++c)
    {
        maniClipCursor* cursor = NULL;
        uint8_t* memory = (uint8_t*)instance + instance->cursorsAt + (size_t)c * size;
        (void)maniInitClipCursor(memory, size, &cursor);
    }
}

static MANI_NOINLINE maniResult InitGraphInstanceWork(const maniGraphBinding* binding, void* memory,
                                                      size_t size, maniGraphInstance** instanceOut)
{
    if (!BindingValid(binding) || memory == NULL || instanceOut == NULL ||
        (uintptr_t)memory % MANI_GRAPH_INSTANCE_ALIGNMENT != 0)
    {
        return mani_errorInvalid;
    }
    size_t needed = maniGraphInstanceSize(binding);
    if (size < needed)
    {
        return mani_errorCapacity;
    }
    const maniGraph* graph = &binding->graph;
    maniGraphInstance* instance = memory;
    memset(instance, 0, needed);
    instance->stamp = MANI_GRAPH_INSTANCE_STAMP;
    instance->clipNodeCount = ClipNodeCount(graph);
    instance->binding = binding;
    instance->size = needed;
    instance->cursorsAt = CursorsAt(graph);
    instance->machinesAt = MachinesAt(binding);
    instance->posesAt = PosesAt(binding);
    instance->roomAt = RoomAt(binding);
    instance->cachesAt = instance->roomAt + maniGraphRoom(binding);
    for (uint32_t i = 0; i < graph->parameterCount; ++i)
    {
        instance->cells[i].value = maniGraphLoadValue(ParameterEntry(graph, i) + 24);
    }
    ResetCursors(instance);
    ResetMachines(instance);
    RestPoses(instance);
    PlaceCaches(instance);
    *instanceOut = instance;
    return mani_success;
}

maniResult maniInitGraphInstance(const maniGraphBinding* binding, void* memory, size_t size,
                                 maniGraphInstance** instanceOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = InitGraphInstanceWork(binding, memory, size, instanceOut);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult SetGraphParameterWork(maniGraphInstance* instance,
                                                      uint32_t parameter,
                                                      maniGraphParameterType type,
                                                      maniGraphValue value)
{
    if (!InstanceValid(instance) || parameter >= instance->binding->graph.parameterCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* entry = ParameterEntry(&instance->binding->graph, parameter);
    if (maniLoadU32(entry) != type || type == maniGraphParameterPose ||
        !maniGraphCanonicalValue(type, entry, &value))
    {
        return mani_errorInvalid;
    }
    instance->cells[parameter].value = value;
    return mani_success;
}

maniResult maniSetGraphParameter(maniGraphInstance* instance, uint32_t parameter,
                                 maniGraphParameterType type, maniGraphValue value)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SetGraphParameterWork(instance, parameter, type, value);
    maniFloatEnvLeave(&env);
    return result;
}

void maniGraphPoseInput(const maniGraphInstance* instance, uint32_t parameter, maniPose* out)
{
    maniPoseSetKind(out, false);
    memcpy(maniPoseLocals(out), maniGraphPoseLocalsRead(instance, parameter),
           PoseBytes(instance->binding));
    maniPoseMarkStale(out);
}

// A pose parameter of the instance's graph.
static bool PoseParameter(const maniGraphInstance* instance, uint32_t parameter)
{
    const maniGraph* graph = &instance->binding->graph;
    return parameter < graph->parameterCount &&
           maniLoadU32(ParameterEntry(graph, parameter)) == maniGraphParameterPose;
}

static MANI_NOINLINE maniResult SetGraphPoseWork(maniGraphInstance* instance, uint32_t parameter,
                                                 const maniPose* pose)
{
    if (!InstanceValid(instance) || !PoseParameter(instance, parameter) ||
        !maniPoseMatches(pose, instance->binding->skeleton) || maniPoseIsDifference(pose))
    {
        return mani_errorInvalid;
    }
    memcpy(maniGraphPoseLocals(instance, parameter), maniPoseLocalsRead(pose),
           PoseBytes(instance->binding));
    return mani_success;
}

maniResult maniSetGraphPose(maniGraphInstance* instance, uint32_t parameter, const maniPose* pose)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SetGraphPoseWork(instance, parameter, pose);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult GetGraphPoseWork(const maniGraphInstance* instance,
                                                 uint32_t parameter, maniPose* poseOut)
{
    if (!InstanceValid(instance) || !PoseParameter(instance, parameter) ||
        !maniPoseMatches(poseOut, instance->binding->skeleton))
    {
        return mani_errorInvalid;
    }
    maniGraphPoseInput(instance, parameter, poseOut);
    return mani_success;
}

maniResult maniGetGraphPose(const maniGraphInstance* instance, uint32_t parameter,
                            maniPose* poseOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = GetGraphPoseWork(instance, parameter, poseOut);
    maniFloatEnvLeave(&env);
    return result;
}

maniResult maniGetGraphParameter(const maniGraphInstance* instance, uint32_t parameter,
                                 maniGraphValue* valueOut)
{
    if (!InstanceValid(instance) || valueOut == NULL ||
        parameter >= instance->binding->graph.parameterCount)
    {
        return mani_errorInvalid;
    }
    *valueOut = instance->cells[parameter].value;
    return mani_success;
}

size_t maniGraphStateSize(const maniGraphBinding* binding)
{
    if (!BindingValid(binding))
    {
        return 0;
    }
    const maniGraph* graph = &binding->graph;
    size_t cells = (size_t)graph->parameterCount + graph->nodeCount;
    // Each pose parameter's locals, the skeleton's bones' (no padding).
    size_t poses = (size_t)PosesBefore(graph, graph->parameterCount) *
                   maniSkeletonBoneCount(binding->skeleton) * MANI_POSE_COMPONENTS * 4;
    return MANI_GRAPH_STATE_HEADER_SIZE + cells * MANI_GRAPH_VALUE_SIZE +
           4 * (size_t)graph->machineWords + poses;
}

static void StoreF32(uint8_t* p, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(p, bits);
}

// Each pose parameter's locals, bone by bone, after the machines'
// blocks.
static void SavePoses(const maniGraphInstance* instance, uint8_t* p)
{
    const maniGraph* graph = &instance->binding->graph;
    uint32_t bones = maniSkeletonBoneCount(instance->binding->skeleton);
    for (uint32_t i = 0; i < graph->parameterCount; ++i)
    {
        if (maniLoadU32(ParameterEntry(graph, i)) != maniGraphParameterPose)
        {
            continue;
        }
        const float* locals = maniGraphPoseLocalsRead(instance, i);
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            const float* slot = locals +
                                (size_t)(bone / MANI_POSE_GROUP_BONES) *
                                    (MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS) +
                                bone % MANI_POSE_GROUP_BONES;
            for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c, p += 4)
            {
                StoreF32(p, slot[c * MANI_POSE_GROUP_BONES]);
            }
        }
    }
}

static MANI_NOINLINE maniResult SaveGraphInstanceWork(const maniGraphInstance* instance, void* out,
                                                      size_t capacity)
{
    if (!InstanceValid(instance) || out == NULL)
    {
        return mani_errorInvalid;
    }
    size_t size = maniGraphStateSize(instance->binding);
    if (capacity < size)
    {
        return mani_errorCapacity;
    }
    const maniGraph* graph = &instance->binding->graph;
    uint8_t* p = out;
    maniStoreU32(p, MANI_GRAPH_STATE_MAGIC);
    maniStoreU32(p + 4, 0);
    maniStoreU64(p + 8, graph->identity);
    maniStoreU64(p + 16, size);
    p += MANI_GRAPH_STATE_HEADER_SIZE;
    for (uint32_t i = 0; i < graph->parameterCount; ++i, p += MANI_GRAPH_VALUE_SIZE)
    {
        maniGraphStoreValue(p, instance->cells[i].value);
    }
    for (uint32_t n = 0; n < graph->nodeCount; ++n, p += MANI_GRAPH_VALUE_SIZE)
    {
        const maniGraphNodeState* s = &instance->cells[graph->parameterCount + n].node;
        StoreF32(p, s->weight);
        maniStoreU32(p + 4, s->word);
        uint64_t bits = 0;
        memcpy(&bits, &s->time, sizeof bits);
        maniStoreU64(p + 8, bits);
    }
    const uint32_t* blocks = Blocks((maniGraphInstance*)instance);
    for (uint32_t w = 0; w < graph->machineWords; ++w, p += 4)
    {
        maniStoreU32(p, blocks[w]);
    }
    SavePoses(instance, p);
    return mani_success;
}

maniResult maniSaveGraphInstance(const maniGraphInstance* instance, void* out, size_t capacity)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SaveGraphInstanceWork(instance, out, capacity);
    maniFloatEnvLeave(&env);
    return result;
}

// A saved value as its parameter keeps it: unchanged by Canonical, but
// for a quaternion's last bits (unit within 1e-5).
static bool SavedValueValid(const maniGraph* graph, uint32_t parameter, const uint8_t* p)
{
    const uint8_t* entry = ParameterEntry(graph, parameter);
    maniGraphParameterType type = maniLoadU32(entry);
    maniGraphValue v = maniGraphLoadValue(p);
    maniGraphValue kept = v;
    if (!maniGraphCanonicalValue(type, entry, &kept))
    {
        return false;
    }
    if (type != maniGraphParameterQuat)
    {
        uint32_t a[4];
        uint32_t b[4];
        memcpy(a, &kept, sizeof a);
        memcpy(b, &v, sizeof b);
        return memcmp(a, b, sizeof a) == 0;
    }
    double length = 0.0;
    for (uint32_t k = 0; k < 4; ++k)
    {
        length += (double)v.f[k] * (double)v.f[k];
    }
    return fabs(length - 1.0) <= 1e-5;
}

// A saved node state: weight finite and not negative, time finite, the
// word a select's input or else 0.
static bool SavedNodeValid(const maniGraph* graph, uint32_t node, const uint8_t* p)
{
    const uint8_t* e = graph->nodes + (size_t)node * MANI_GRAPH_NODE_SIZE;
    float weight = 0.0f;
    double time = 0.0;
    uint32_t bits = maniLoadU32(p);
    uint64_t timeBits = maniLoadU64(p + 8);
    memcpy(&weight, &bits, sizeof weight);
    memcpy(&time, &timeBits, sizeof time);
    uint32_t word = maniLoadU32(p + 4);
    bool select = maniLoadU16(e) == maniGraphNodeSelect;
    bool wordValid = select ? word < maniLoadU32(e + 8) : word == 0;
    return isfinite(weight) && weight >= 0.0f && isfinite(time) && wordValid;
}

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

// A motion matching node's saved block, with its saved playhead.
static uint32_t SavedMatchValid(const maniGraphBinding* binding, uint32_t p, const uint8_t* b,
                                const uint8_t* block)
{
    const maniGraph* graph = &binding->graph;
    uint32_t data = maniLoadU32(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    uint32_t slot = maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphMatchDatabase));
    uint64_t at = MANI_GRAPH_STATE_HEADER_SIZE +
                  (uint64_t)(graph->parameterCount + p) * MANI_GRAPH_VALUE_SIZE;
    uint64_t bits = maniLoadU64(b + at + 8);
    double time = 0.0;
    memcpy(&time, &bits, sizeof time);
    return maniGraphSavedMatchValid(block, graph, data, maniGraphBoundDatabase(binding, slot),
                                    time);
}

// A spring node's saved block, against its slot's set.
static uint32_t SavedSpringValid(const maniGraphBinding* binding, uint32_t p, const uint8_t* block)
{
    const maniGraph* graph = &binding->graph;
    uint32_t data = DataOf(graph, p);
    uint32_t slot = maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphSpringSet));
    return maniGraphSavedSpringValid(block, maniGraphBoundSpringSet(binding, slot));
}

// The first wrong word of node p's saved block, m its machine (none if
// it has no states), or UINT32_MAX.
static uint32_t SavedBlockValid(const maniGraphBinding* binding, uint32_t p,
                                const maniGraphMachine* m, const uint8_t* b, const uint8_t* block)
{
    const maniGraph* graph = &binding->graph;
    maniGraphInertia node = InertiaAt(graph, p);
    uint32_t data = DataOf(graph, p);
    const maniLearnedModel* model =
        Learning(graph, p)
            ? maniGraphBoundModel(
                  binding, maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphLearnedModel)))
            : NULL;
    return m->states > 0         ? maniGraphSavedBlockValid(m, block)
           : Inertial(graph, p)  ? maniGraphSavedInertiaValid(block, &node)
           : Matching(graph, p)  ? SavedMatchValid(binding, p, b, block)
           : Springing(graph, p) ? SavedSpringValid(binding, p, block)
           : Turning(graph, p)   ? maniGraphSavedTurnValid(block, graph->skeletonBoneCount)
           : model != NULL       ? maniGraphSavedLearnedValid(block, graph, data, model)
                                 : maniGraphSavedShotValid(block);
}

// Every node's block in a saved state, the blocks starting at at.
static maniFileResult CheckBlocks(const maniGraphInstance* instance, const uint8_t* b, uint64_t at)
{
    const maniGraph* graph = &instance->binding->graph;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        maniGraphMachine m = MachineAt(graph, p);
        uint32_t shot = ShotBlockAt(graph, p);
        uint32_t first = m.states > 0 ? maniGraphMachineBlockAt(&m) : shot;
        if (first == UINT32_MAX)
        {
            continue;
        }
        uint64_t block = at + 4 * (uint64_t)first;
        uint32_t wrong = SavedBlockValid(instance->binding, p, &m, b, b + block);
        if (wrong != UINT32_MAX)
        {
            return Fail(block + 4 * (uint64_t)wrong);
        }
    }
    return (maniFileResult){mani_success, 0, 0};
}

static float SavedF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

// The first word of a saved bone's locals out of form (a word not
// finite, a rotation not of length 1 within a thousandth), or
// UINT32_MAX.
static uint32_t SavedLocalsWrong(const uint8_t* p)
{
    float f[MANI_POSE_COMPONENTS];
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        f[c] = SavedF32(p + 4 * c);
        if (!isfinite(f[c]))
        {
            return c;
        }
    }
    float length = f[3] * f[3] + f[4] * f[4] + f[5] * f[5] + f[6] * f[6];
    return length > 0.998f && length < 1.002f ? UINT32_MAX : 3u;
}

// Each pose parameter's saved locals, from at.
static maniFileResult CheckPoses(const maniGraphInstance* instance, const uint8_t* b, uint64_t at)
{
    const maniGraph* graph = &instance->binding->graph;
    uint64_t bones = (uint64_t)PosesBefore(graph, graph->parameterCount) *
                     maniSkeletonBoneCount(instance->binding->skeleton);
    for (uint64_t bone = 0; bone < bones; ++bone, at += 4 * MANI_POSE_COMPONENTS)
    {
        uint32_t wrong = SavedLocalsWrong(b + at);
        if (wrong != UINT32_MAX)
        {
            return Fail(at + 4 * (uint64_t)wrong);
        }
    }
    return (maniFileResult){mani_success, 0, 0};
}

static maniFileResult CheckState(const maniGraphInstance* instance, const uint8_t* b, size_t size)
{
    const maniGraph* graph = &instance->binding->graph;
    size_t expected = maniGraphStateSize(instance->binding);
    if (size < MANI_GRAPH_STATE_HEADER_SIZE || maniLoadU32(b) != MANI_GRAPH_STATE_MAGIC)
    {
        return Fail(0);
    }
    if (maniLoadU32(b + 4) != 0)
    {
        return Fail(4);
    }
    if (maniLoadU64(b + 8) != graph->identity)
    {
        return Fail(8);
    }
    if (maniLoadU64(b + 16) != expected || size < expected)
    {
        return Fail(16);
    }
    uint64_t at = MANI_GRAPH_STATE_HEADER_SIZE;
    for (uint32_t i = 0; i < graph->parameterCount; ++i, at += MANI_GRAPH_VALUE_SIZE)
    {
        if (!SavedValueValid(graph, i, b + at))
        {
            return Fail(at);
        }
    }
    for (uint32_t n = 0; n < graph->nodeCount; ++n, at += MANI_GRAPH_VALUE_SIZE)
    {
        if (!SavedNodeValid(graph, n, b + at))
        {
            return Fail(at);
        }
    }
    maniFileResult r = CheckBlocks(instance, b, at);
    if (r.result != mani_success)
    {
        return r;
    }
    return CheckPoses(instance, b, at + 4 * (uint64_t)graph->machineWords);
}

// Each pose parameter's locals from the saved bytes (checked).
static void RestorePoses(maniGraphInstance* instance, const uint8_t* p)
{
    const maniGraph* graph = &instance->binding->graph;
    uint32_t bones = maniSkeletonBoneCount(instance->binding->skeleton);
    for (uint32_t i = 0; i < graph->parameterCount; ++i)
    {
        if (maniLoadU32(ParameterEntry(graph, i)) != maniGraphParameterPose)
        {
            continue;
        }
        float* locals = maniGraphPoseLocals(instance, i);
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            float values[MANI_POSE_COMPONENTS];
            for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c, p += 4)
            {
                values[c] = SavedF32(p);
            }
            maniPoseSlotPut(maniPoseSlot(locals, bone), 0, values, MANI_POSE_COMPONENTS);
        }
    }
}

static MANI_NOINLINE maniFileResult RestoreGraphInstanceWork(maniGraphInstance* instance,
                                                             const void* bytes, size_t size)
{
    if (!InstanceValid(instance) || bytes == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniFileResult r = CheckState(instance, bytes, size);
    if (r.result != mani_success)
    {
        return r;
    }
    const maniGraph* graph = &instance->binding->graph;
    const uint8_t* p = (const uint8_t*)bytes + MANI_GRAPH_STATE_HEADER_SIZE;
    for (uint32_t i = 0; i < graph->parameterCount; ++i, p += MANI_GRAPH_VALUE_SIZE)
    {
        instance->cells[i].value = maniGraphLoadValue(p);
    }
    for (uint32_t n = 0; n < graph->nodeCount; ++n, p += MANI_GRAPH_VALUE_SIZE)
    {
        maniGraphNodeState* s = &instance->cells[graph->parameterCount + n].node;
        uint32_t bits = maniLoadU32(p);
        uint64_t timeBits = maniLoadU64(p + 8);
        *s = (maniGraphNodeState){0};
        memcpy(&s->weight, &bits, sizeof s->weight);
        s->word = maniLoadU32(p + 4);
        memcpy(&s->time, &timeBits, sizeof s->time);
    }
    uint32_t* blocks = Blocks(instance);
    for (uint32_t w = 0; w < graph->machineWords; ++w, p += 4)
    {
        blocks[w] = maniLoadU32(p);
    }
    RestorePoses(instance, p);
    ResetCursors(instance);
    return r;
}

maniFileResult maniRestoreGraphInstance(maniGraphInstance* instance, const void* bytes, size_t size)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniFileResult result = RestoreGraphInstanceWork(instance, bytes, size);
    maniFloatEnvLeave(&env);
    return result;
}
