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
#include "graph_nodes.h"
#include "graph_shot.h"
#include "graph_spring.h"
#include "graph_turn.h"
#include "graph_values.h"
#include "invariant.h"
#include "learned_eval.h"
#include "pose_memory.h"
#include "state_prefix.h"

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

MANI_NODISCARD static bool InstanceValid(const maniGraphInstance* instance)
{
    return instance != NULL && instance->stamp == MANI_GRAPH_INSTANCE_STAMP;
}

MANI_NODISCARD static bool BindingValid(const maniGraphBinding* binding)
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
    return CursorsAt(graph) + (size_t)binding->clipNodeCount * binding->cursorSize;
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
    size_t groups =
        ((size_t)maniSkeleton_GetBoneCount(binding->skeleton) + MANI_POSE_GROUP_BONES - 1) /
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
    return PosesAt(binding) + (size_t)binding->poseCount * PoseBytes(binding);
}

// A pose parameter's place among the poses bind counted, or UINT32_MAX
// for a parameter past the count, of another type, or past the poses
// bind counted (the graph's bytes changed since).
static uint32_t PoseSlot(const maniGraphBinding* b, uint32_t parameter)
{
    if (parameter >= b->graph.parameterCount ||
        maniLoadU32(ParameterEntry(&b->graph, parameter)) != maniGraphParameterPose)
    {
        return UINT32_MAX;
    }
    uint32_t slot = PosesBefore(&b->graph, parameter);
    return slot < b->poseCount ? slot : UINT32_MAX;
}

// Pose slot s's locals, between the machines' blocks and the room.
static size_t SlotAt(const maniGraphInstance* instance, uint32_t s)
{
    return instance->posesAt + (size_t)s * PoseBytes(instance->binding);
}

// A pose parameter's locals; NULL for none (PoseSlot).
float* maniGraphPoseLocals(maniGraphInstance* instance, uint32_t parameter)
{
    uint32_t s = PoseSlot(instance->binding, parameter);
    return s != UINT32_MAX ? (float*)((uint8_t*)instance + SlotAt(instance, s)) : NULL;
}

const float* maniGraphPoseLocalsRead(const maniGraphInstance* instance, uint32_t parameter)
{
    uint32_t s = PoseSlot(instance->binding, parameter);
    return s != UINT32_MAX ? (const float*)((const uint8_t*)instance + SlotAt(instance, s)) : NULL;
}

// Every pose parameter at the skeleton's rest.
static void RestPoses(maniGraphInstance* instance)
{
    const maniGraphBinding* b = instance->binding;
    uint32_t bones = maniSkeleton_GetBoneCount(b->skeleton);
    for (uint32_t s = 0; s < b->poseCount; ++s)
    {
        float* locals = (float*)((uint8_t*)instance + SlotAt(instance, s));
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            maniTransform t = maniSkeleton_GetRest(b->skeleton, bone);
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
    return maniGraphBoundModel(binding, maniGraphWord(graph, data + maniGraphLearnedModel));
}

// A learned node's cache in floats: a word set once kept (0 from the
// instance's start), the state it was kept at, then its Decompressor's
// hidden values; none for a Decompressor of one layer.
static size_t CacheFloats(const maniLearnedModel* m)
{
    if (m == NULL)
    {
        return 0;
    }
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
    size_t at = 0;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        size_t floats = maniLoadU16(e) == maniGraphNodeLearned
                            ? CacheFloats(LearnedModel(binding, maniLoadU32(e + 12)))
                            : 0u;
        // Only what bind counted is placed: bytes changed since leave a
        // node without a cache.
        bool fits = floats > 0 && floats <= binding->cacheFloats - at;
        table[p] = fits ? (uint32_t)at : UINT32_MAX;
        at += fits ? floats : 0u;
    }
}

float* maniGraphLearnedCache(maniGraphInstance* instance, uint32_t node,
                             const maniLearnedModel* model)
{
    const maniGraphBinding* binding = instance->binding;
    if (instance->cachesAt == instance->size || node >= binding->graph.nodeCount || model == NULL)
    {
        return NULL;
    }
    uint32_t* table = (uint32_t*)((uint8_t*)instance + instance->cachesAt);
    uint32_t at = table[node];
    size_t floats = CacheFloats(model);
    // The model the node names now may take more than the caches from its
    // place on. One that takes more than its own place, but no more than
    // that, runs into the next node's cache: within the instance, and only
    // after the bytes changed.
    bool fits = at != UINT32_MAX && floats > 0 && floats <= binding->cacheFloats - at;
    return fits ? (float*)(table + binding->graph.nodeCount) + at : NULL;
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
        const maniLearnedModel* m =
            maniLoadU16(e) == maniGraphNodeLearned ? LearnedModel(binding, data) : NULL;
        const maniSpringSet* set =
            maniLoadU16(e) == maniGraphNodeSpring
                ? maniGraphBoundSpringSet(binding, maniGraphWord(graph, data + maniGraphSpringSet))
                : NULL;
        // A node of a kind the bound assets give none of (the bytes
        // changed since the open) takes no room.
        if (m != NULL)
        {
            need = maniGraphLearnedRoom(m);
        }
        else if (set != NULL)
        {
            need = 4 * (size_t)maniGraphSpringWorkWords(set);
        }
        else if (maniLoadU16(e) == maniGraphNodeBodyXpbd || maniLoadU16(e) == maniGraphNodeBodySdls)
        {
            need = maniGraphBodyRoom(graph, data, binding->skeleton);
        }
        room = need > room ? need : room;
    }
    return (room + 7u) & ~(size_t)7u;
}

void maniGraphRecordLayout(maniGraphBinding* binding)
{
    const maniGraph* graph = &binding->graph;
    binding->clipNodeCount = ClipNodeCount(graph);
    binding->poseCount = PosesBefore(graph, graph->parameterCount);
    binding->room = maniGraphRoom(binding);
    binding->cacheFloats = Caches(binding);
}

size_t maniGraphBinding_GetInstanceSize(const maniGraphBinding* binding)
{
    if (!BindingValid(binding))
    {
        return 0;
    }
    // The caches after a table of their places, one word a node, when any
    // node has one.
    size_t caches = binding->cacheFloats;
    size_t table = caches > 0 ? binding->graph.nodeCount : 0u;
    return RoomAt(binding) + binding->room + sizeof(float) * (table + caches);
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

static uint32_t Kind(const maniGraph* graph, uint32_t p)
{
    return maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE);
}

// The one-shot, inertialize, motion matching, learned, spring or turn
// limit node p's block's first word, or UINT32_MAX for another kind.
static uint32_t ShotBlockAt(const maniGraph* graph, uint32_t p)
{
    static const uint32_t kBlockWord[][2] = {{maniGraphNodeOneShot, maniGraphShotBlock},
                                             {maniGraphNodeInertialize, maniGraphInertiaBlock},
                                             {maniGraphNodeMatch, maniGraphMatchBlock},
                                             {maniGraphNodeLearned, maniGraphLearnedBlock},
                                             {maniGraphNodeSpring, maniGraphSpringBlock},
                                             {maniGraphNodeTurnLimit, maniGraphTurnBlock}};
    uint32_t data = maniLoadU32(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    for (size_t k = 0; k < sizeof kBlockWord / sizeof kBlockWord[0]; ++k)
    {
        if (Kind(graph, p) == kBlockWord[k][0])
        {
            return maniGraphWord(graph, data + kBlockWord[k][1]);
        }
    }
    return UINT32_MAX;
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

// Node p's block's first word, or UINT32_MAX for a kind without one.
static uint32_t BlockFirst(const maniGraph* graph, uint32_t p)
{
    maniGraphMachine m = MachineAt(graph, p);
    return m.states > 0 ? maniGraphMachineBlockAt(&m) : ShotBlockAt(graph, p);
}

// Node p's block's first word when its block, as its kind and data size
// it now, fits in the machines' words; UINT32_MAX otherwise.
static uint32_t FittingBlock(const maniGraph* graph, uint32_t p)
{
    const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
    uint32_t first = BlockFirst(graph, p);
    uint64_t words =
        maniGraphBlockWords(graph, maniLoadU16(e), maniLoadU32(e + 8), maniLoadU32(e + 12));
    bool fits = first != UINT32_MAX && first <= graph->machineWords &&
                words <= (uint64_t)graph->machineWords - first;
    return fits ? first : UINT32_MAX;
}

uint32_t* maniGraphNodeBlock(maniGraphInstance* instance, uint32_t p)
{
    const maniGraph* graph = &instance->binding->graph;
    uint32_t first = p < graph->nodeCount ? FittingBlock(graph, p) : UINT32_MAX;
    return first != UINT32_MAX ? Blocks(instance) + first : NULL;
}

const uint32_t* maniGraphNodeBlockRead(const maniGraphInstance* instance, uint32_t p)
{
    return maniGraphNodeBlock((maniGraphInstance*)instance, p);
}

static void ResetMachines(maniGraphInstance* instance)
{
    const maniGraph* graph = &instance->binding->graph;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        uint32_t* block = maniGraphNodeBlock(instance, p);
        maniGraphMachine m = MachineAt(graph, p);
        uint32_t kind = Kind(graph, p);
        if (block == NULL)
        {
            continue;
        }
        if (m.states > 0)
        {
            maniGraphResetMachine(&m, block);
        }
        else if (kind == maniGraphNodeInertialize)
        {
            maniGraphInertia node = InertiaAt(graph, p);
            maniGraphResetInertia(block, &node);
        }
        else if (kind == maniGraphNodeMatch)
        {
            maniGraphResetMatch(block);
        }
        else if (kind == maniGraphNodeLearned)
        {
            maniGraphResetLearned(graph, DataOf(graph, p), block);
        }
        else if (kind == maniGraphNodeSpring)
        {
            maniGraphResetSpring(graph, DataOf(graph, p), block);
        }
        else if (kind == maniGraphNodeTurnLimit)
        {
            maniGraphResetTurn(block, graph->skeletonBoneCount);
        }
        else
        {
            maniGraphResetShot(block);
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
        // The binding's cursor size fits every clip it binds.
        maniResult made = maniInitClipCursor(memory, size, &cursor);
        MANI_ASSERT(made == mani_success);
        (void)made;
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
    size_t needed = maniGraphBinding_GetInstanceSize(binding);
    if (size < needed)
    {
        return mani_errorCapacity;
    }
    const maniGraph* graph = &binding->graph;
    maniGraphInstance* instance = memory;
    memset(instance, 0, needed);
    instance->stamp = MANI_GRAPH_INSTANCE_STAMP;
    instance->clipNodeCount = binding->clipNodeCount;
    instance->binding = binding;
    instance->size = needed;
    instance->cursorsAt = CursorsAt(graph);
    instance->machinesAt = MachinesAt(binding);
    instance->posesAt = PosesAt(binding);
    instance->roomAt = RoomAt(binding);
    instance->cachesAt = instance->roomAt + binding->room;
    MANI_ASSERT(instance->cursorsAt <= instance->machinesAt &&
                instance->machinesAt <= instance->posesAt &&
                instance->posesAt <= instance->roomAt && instance->roomAt <= instance->cachesAt &&
                instance->cachesAt <= needed);
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
    uint64_t env = maniFloatEnvEnter();
    maniResult result = InitGraphInstanceWork(binding, memory, size, instanceOut);
    maniFloatEnvLeave(env);
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
    if (maniLoadU32(entry) != (uint32_t)type || type == maniGraphParameterPose ||
        !maniGraphCanonicalValue(type, entry, &value))
    {
        return mani_errorInvalid;
    }
    instance->cells[parameter].value = value;
    return mani_success;
}

maniResult maniGraphInstance_SetParameter(maniGraphInstance* instance, uint32_t parameter,
                                          maniGraphParameterType type, maniGraphValue value)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = SetGraphParameterWork(instance, parameter, type, value);
    maniFloatEnvLeave(env);
    return result;
}

void maniGraphPoseInput(const maniGraphInstance* instance, uint32_t parameter, maniPose* out)
{
    MANI_ASSERT(maniPoseMatches(out, instance->binding->skeleton));
    const float* locals = maniGraphPoseLocalsRead(instance, parameter);
    if (locals == NULL)
    {
        // A pose node whose parameter the bytes no longer give.
        (void)maniPose_SetRest(out, instance->binding->skeleton);
        return;
    }
    maniPoseSetKind(out, false);
    memcpy(maniPoseLocals(out), locals, PoseBytes(instance->binding));
    maniPoseMarkStale(out);
}

// A pose parameter of the instance's graph, with its place among the
// poses bind counted.
MANI_NODISCARD static bool PoseParameter(const maniGraphInstance* instance, uint32_t parameter)
{
    return PoseSlot(instance->binding, parameter) != UINT32_MAX;
}

static MANI_NOINLINE maniResult SetGraphPoseWork(maniGraphInstance* instance, uint32_t parameter,
                                                 const maniPose* pose)
{
    if (!InstanceValid(instance) || !PoseParameter(instance, parameter) ||
        !maniPoseMatches(pose, instance->binding->skeleton) || maniPose_IsDifference(pose))
    {
        return mani_errorInvalid;
    }
    memcpy(maniGraphPoseLocals(instance, parameter), maniPoseLocalsRead(pose),
           PoseBytes(instance->binding));
    return mani_success;
}

maniResult maniGraphInstance_SetPose(maniGraphInstance* instance, uint32_t parameter,
                                     const maniPose* pose)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = SetGraphPoseWork(instance, parameter, pose);
    maniFloatEnvLeave(env);
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

maniResult maniGraphInstance_GetPose(const maniGraphInstance* instance, uint32_t parameter,
                                     maniPose* poseOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = GetGraphPoseWork(instance, parameter, poseOut);
    maniFloatEnvLeave(env);
    return result;
}

maniResult maniGraphInstance_GetParameter(const maniGraphInstance* instance, uint32_t parameter,
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

// The bytes an instance's saved state takes.
static size_t StateSize(const maniGraphBinding* binding)
{
    const maniGraph* graph = &binding->graph;
    size_t cells = (size_t)graph->parameterCount + graph->nodeCount;
    // Each pose parameter's locals, the skeleton's bones' (no padding).
    size_t poses = (size_t)binding->poseCount * maniSkeleton_GetBoneCount(binding->skeleton) *
                   MANI_POSE_COMPONENTS * 4;
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
// blocks: the poses bind counted, in parameter order.
static void SavePoses(const maniGraphInstance* instance, uint8_t* p)
{
    uint32_t bones = maniSkeleton_GetBoneCount(instance->binding->skeleton);
    for (uint32_t s = 0; s < instance->binding->poseCount; ++s)
    {
        const float* locals = (const float*)((const uint8_t*)instance + SlotAt(instance, s));
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
                                                      size_t capacity, size_t* sizeOut)
{
    if (!InstanceValid(instance) || (out == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    size_t size = StateSize(instance->binding);
    if (sizeOut != NULL)
    {
        *sizeOut = size;
    }
    // A NULL buffer has capacity 0, and a state is never empty.
    if (out == NULL || capacity < size)
    {
        return mani_errorCapacity;
    }
    const maniGraph* graph = &instance->binding->graph;
    uint8_t* p = out;
    maniStoreStatePrefix(p, MANI_GRAPH_STATE_MAGIC, MANI_GRAPH_STATE_VERSION,
                         MANI_GRAPH_STATE_HEADER_SIZE);
    maniStoreU64(p + 16, graph->identity);
    maniStoreU64(p + 24, size);
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
    // The poses' locals take the rest of the state.
    MANI_ASSERT((size_t)(p - (uint8_t*)out) +
                    (size_t)instance->binding->poseCount *
                        maniSkeleton_GetBoneCount(instance->binding->skeleton) *
                        MANI_POSE_COMPONENTS * 4 ==
                size);
    SavePoses(instance, p);
    return mani_success;
}

maniResult maniGraphInstance_Save(const maniGraphInstance* instance, void* bytesOut,
                                  size_t capacity, size_t* sizeOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = SaveGraphInstanceWork(instance, bytesOut, capacity, sizeOut);
    maniFloatEnvLeave(env);
    return result;
}

// A saved value as its parameter keeps it: unchanged by Canonical, but
// for a quaternion's last bits (unit within 1e-5).
MANI_NODISCARD static bool SavedValueValid(const maniGraph* graph, uint32_t parameter,
                                           const uint8_t* p)
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
MANI_NODISCARD static bool SavedNodeValid(const maniGraph* graph, uint32_t node, const uint8_t* p)
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
    uint32_t slot = maniGraphWord(graph, data + maniGraphMatchDatabase);
    uint64_t at = MANI_GRAPH_STATE_HEADER_SIZE +
                  (uint64_t)(graph->parameterCount + p) * MANI_GRAPH_VALUE_SIZE;
    uint64_t bits = maniLoadU64(b + at + 8);
    double time = 0.0;
    memcpy(&time, &bits, sizeof time);
    const maniMotionDatabase* database = maniGraphBoundDatabase(binding, slot);
    // A node without a database (the bytes changed since the open) never
    // reads its block.
    return database != NULL ? maniGraphSavedMatchValid(block, graph, data, database, time)
                            : UINT32_MAX;
}

// A spring node's saved block, against its slot's set.
static uint32_t SavedSpringValid(const maniGraphBinding* binding, uint32_t p, const uint8_t* block)
{
    const maniGraph* graph = &binding->graph;
    uint32_t data = DataOf(graph, p);
    const maniSpringSet* set =
        maniGraphBoundSpringSet(binding, maniGraphWord(graph, data + maniGraphSpringSet));
    return set != NULL ? maniGraphSavedSpringValid(block, set) : UINT32_MAX;
}

// A learned node's saved block, against its slot's model.
static uint32_t SavedLearnedValid(const maniGraphBinding* binding, uint32_t p, const uint8_t* block)
{
    const maniGraph* graph = &binding->graph;
    uint32_t data = DataOf(graph, p);
    const maniLearnedModel* model =
        maniGraphBoundModel(binding, maniGraphWord(graph, data + maniGraphLearnedModel));
    // A node without a model (the bytes changed since the open) never
    // reads its block.
    return model != NULL ? maniGraphSavedLearnedValid(block, graph, data, model) : UINT32_MAX;
}

// The first wrong word of node p's saved block, m its machine (none if
// it has no states), or UINT32_MAX.
static uint32_t SavedBlockValid(const maniGraphBinding* binding, uint32_t p,
                                const maniGraphMachine* m, const uint8_t* b, const uint8_t* block)
{
    const maniGraph* graph = &binding->graph;
    maniGraphInertia node = InertiaAt(graph, p);
    uint32_t kind = Kind(graph, p);
    return m->states > 0                      ? maniGraphSavedBlockValid(m, block)
           : kind == maniGraphNodeInertialize ? maniGraphSavedInertiaValid(block, &node)
           : kind == maniGraphNodeMatch       ? SavedMatchValid(binding, p, b, block)
           : kind == maniGraphNodeSpring      ? SavedSpringValid(binding, p, block)
           : kind == maniGraphNodeTurnLimit
               ? maniGraphSavedTurnValid(block, graph->skeletonBoneCount)
           : kind == maniGraphNodeLearned ? SavedLearnedValid(binding, p, block)
                                          : maniGraphSavedShotValid(block);
}

// Every node's block in a saved state, the blocks starting at at.
static maniFileResult CheckBlocks(const maniGraphInstance* instance, const uint8_t* b, uint64_t at)
{
    const maniGraph* graph = &instance->binding->graph;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        maniGraphMachine m = MachineAt(graph, p);
        // A block that does not fit (the bytes changed since the open) is
        // never read, so its saved words are not checked.
        uint32_t first = FittingBlock(graph, p);
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
    uint64_t bones = (uint64_t)instance->binding->poseCount *
                     maniSkeleton_GetBoneCount(instance->binding->skeleton);
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
    size_t expected = StateSize(instance->binding);
    maniFileResult prefix = maniCheckStatePrefix(
        b, size, MANI_GRAPH_STATE_MAGIC, MANI_GRAPH_STATE_VERSION, MANI_GRAPH_STATE_HEADER_SIZE);
    if (prefix.result != mani_success)
    {
        return prefix;
    }
    if (maniLoadU64(b + 16) != graph->identity)
    {
        return Fail(16);
    }
    if (maniLoadU64(b + 24) != expected || size < expected)
    {
        return Fail(24);
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
    uint32_t bones = maniSkeleton_GetBoneCount(instance->binding->skeleton);
    for (uint32_t s = 0; s < instance->binding->poseCount; ++s)
    {
        float* locals = (float*)((uint8_t*)instance + SlotAt(instance, s));
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
    MANI_ASSERT((size_t)(p - (const uint8_t*)bytes) +
                    (size_t)instance->binding->poseCount *
                        maniSkeleton_GetBoneCount(instance->binding->skeleton) *
                        MANI_POSE_COMPONENTS * 4 ==
                StateSize(instance->binding));
    RestorePoses(instance, p);
    ResetCursors(instance);
    return r;
}

maniFileResult maniGraphInstance_Restore(maniGraphInstance* instance, const void* bytes,
                                         size_t size)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = RestoreGraphInstanceWork(instance, bytes, size);
    maniFloatEnvLeave(env);
    return result;
}
