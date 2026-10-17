// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph instances (mani-0023): made in caller memory, their parameters
// set and read, their state saved as bytes without pointers and
// restored after checking those bytes as external input.

#include "graph_instance.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_learned.h"
#include "graph_machine.h"
#include "graph_match.h"
#include "graph_shot.h"
#include "graph_spring.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"

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

static size_t RoomAt(const maniGraphBinding* binding)
{
    size_t words = binding->graph.machineWords;
    return MachinesAt(binding) + ((4 * words + 7u) & ~(size_t)7u);
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
            uint32_t slot = maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphLearnedModel));
            const maniLearnedModel* m = maniGraphBoundModel(binding, slot);
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
    return RoomAt(binding) + maniGraphRoom(binding);
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
        else if (shot != UINT32_MAX)
        {
            maniGraphResetShot(Blocks(instance) + shot);
        }
    }
}

static const uint8_t* ParameterEntry(const maniGraph* graph, uint32_t parameter)
{
    return graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE;
}

static maniGraphValue LoadValue(const uint8_t* p)
{
    maniGraphValue v;
    uint32_t words[4];
    for (uint32_t k = 0; k < 4; ++k)
    {
        words[k] = maniLoadU32(p + 4 * k);
    }
    memcpy(&v, words, sizeof v);
    return v;
}

static void StoreValue(uint8_t* p, maniGraphValue v)
{
    uint32_t words[4];
    memcpy(words, &v, sizeof words);
    for (uint32_t k = 0; k < 4; ++k)
    {
        maniStoreU32(p + 4 * k, words[k]);
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

maniResult maniInitGraphInstance(const maniGraphBinding* binding, void* memory, size_t size,
                                 maniGraphInstance** instanceOut)
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
    instance->roomAt = RoomAt(binding);
    for (uint32_t i = 0; i < graph->parameterCount; ++i)
    {
        instance->cells[i].value = LoadValue(ParameterEntry(graph, i) + 24);
    }
    ResetCursors(instance);
    ResetMachines(instance);
    *instanceOut = instance;
    return mani_success;
}

static float Clamp(float v, const uint8_t* entry)
{
    float low = 0.0f;
    float high = 0.0f;
    memcpy(&low, entry + 40, sizeof low);
    memcpy(&high, entry + 44, sizeof high);
    return v < low ? low : (v > high ? high : v);
}

// The float and vector values as kept: finite, a quaternion normalized,
// a bounded float clamped; false for one out of form.
static bool CanonicalFloats(maniGraphParameterType type, const uint8_t* entry, bool bounded,
                            maniGraphValue* v)
{
    uint32_t n = type == maniGraphParameterFloat  ? 1u
                 : type == maniGraphParameterVec2 ? 2u
                 : type == maniGraphParameterVec3 ? 3u
                                                  : 4u;
    maniGraphValue out = {{0}};
    double length = 0.0;
    for (uint32_t k = 0; k < n; ++k)
    {
        if (!isfinite(v->f[k]))
        {
            return false;
        }
        out.f[k] = v->f[k];
        length += (double)v->f[k] * (double)v->f[k];
    }
    if (type == maniGraphParameterQuat)
    {
        if (!(length > 0.0) || !isfinite(length))
        {
            return false;
        }
        double scale = 1.0 / sqrt(length);
        for (uint32_t k = 0; k < 4; ++k)
        {
            out.f[k] = (float)((double)out.f[k] * scale);
        }
    }
    out.f[0] = type == maniGraphParameterFloat && bounded ? Clamp(out.f[0], entry) : out.f[0];
    *v = out;
    return true;
}

// The value as the parameter keeps it, or false for one out of form.
static bool Canonical(maniGraphParameterType type, const uint8_t* entry, maniGraphValue* v)
{
    bool bounded = (maniLoadU32(entry + 4) & MANI_GRAPH_BOUNDED) != 0;
    maniGraphValue out = {{0}};
    if (type == maniGraphParameterBool || type == maniGraphParameterTrigger)
    {
        out.b = v->b;
        *v = out;
        return out.b <= 1u;
    }
    if (type == maniGraphParameterInt)
    {
        int32_t low = (int32_t)maniLoadU32(entry + 40);
        int32_t high = (int32_t)maniLoadU32(entry + 44);
        out.i = bounded ? (v->i < low ? low : (v->i > high ? high : v->i)) : v->i;
        *v = out;
        return true;
    }
    return CanonicalFloats(type, entry, bounded, v);
}

maniResult maniSetGraphParameter(maniGraphInstance* instance, uint32_t parameter,
                                 maniGraphParameterType type, maniGraphValue value)
{
    if (!InstanceValid(instance) || parameter >= instance->binding->graph.parameterCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* entry = ParameterEntry(&instance->binding->graph, parameter);
    if (maniLoadU32(entry) != type || !Canonical(type, entry, &value))
    {
        return mani_errorInvalid;
    }
    instance->cells[parameter].value = value;
    return mani_success;
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
    size_t cells = (size_t)binding->graph.parameterCount + binding->graph.nodeCount;
    return MANI_GRAPH_STATE_HEADER_SIZE + cells * MANI_GRAPH_VALUE_SIZE +
           4 * (size_t)binding->graph.machineWords;
}

static void StoreF32(uint8_t* p, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(p, bits);
}

maniResult maniSaveGraphInstance(const maniGraphInstance* instance, void* out, size_t capacity)
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
        StoreValue(p, instance->cells[i].value);
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
    return mani_success;
}

// A saved value as its parameter keeps it: unchanged by Canonical, but
// for a quaternion's last bits (unit within 1e-5).
static bool SavedValueValid(const maniGraph* graph, uint32_t parameter, const uint8_t* p)
{
    const uint8_t* entry = ParameterEntry(graph, parameter);
    maniGraphParameterType type = maniLoadU32(entry);
    maniGraphValue v = LoadValue(p);
    maniGraphValue kept = v;
    if (!Canonical(type, entry, &kept))
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
    return CheckBlocks(instance, b, at);
}

maniFileResult maniRestoreGraphInstance(maniGraphInstance* instance, const void* bytes, size_t size)
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
        instance->cells[i].value = LoadValue(p);
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
    ResetCursors(instance);
    return r;
}
