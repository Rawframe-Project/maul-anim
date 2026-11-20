// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's bytes changed after the open, for the graph suites: each
// word of every node record, input, data word, parameter, clip record
// and database clip run set in turn to values a crafted file could hold.
// After each change, every call (parameters and poses set and read,
// every node inspected, the state saved and restored into another
// instance) runs on new instances of a binding made before any change,
// on an instance made then, and on instances of the graph bound again;
// then the getters are read. The bytes are not checked again after the
// open, so the results are some values and states; no call reads or
// writes past the bindings or the instances (heap blocks of their exact
// length, so AddressSanitizer sees it) and none traps. Each change put
// back gives the graph's state again on both bindings. Included by one
// suite each.

#ifndef MAUL_ANIM_TEST_GRAPH_CHANGED_H
#define MAUL_ANIM_TEST_GRAPH_CHANGED_H

#include "bytes.h"
#include "file_tag.h"
#include "graph_file.h"
#include "test_harness.h"

#include "maul-anim/graph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// An instance's state after the calls, as maniGraphInstance_Save writes
// it; size 0 when the save failed.
typedef struct ChangedState
{
    _Alignas(8) uint8_t bytes[16384];
    size_t size;
} ChangedState;

typedef struct ChangedRun
{
    const maniGraph* graph;
    const maniGraphAssets* assets;
    maniPose* pose;
    maniPose* spare;
    // Bound before any change, and an instance made then that every run
    // goes on with: the bytes changed after the bind.
    maniGraphBinding* binding;
    maniGraphInstance* instance;
    uint8_t* bindingFree;
    uint8_t* instanceFree;
    ChangedState out;
    ChangedState kept;
    ChangedState other;
    ChangedState reference;
} ChangedRun;

// Every call on an instance: its parameters and poses set and read,
// every node inspected, and the state saved into out and restored into
// other.
static inline void ChangedCalls(ChangedRun* c, maniGraphInstance* instance,
                                maniGraphInstance* other, ChangedState* out)
{
    const maniGraph* g = c->graph;
    for (uint32_t p = 0; p < maniGraph_GetParameterCount(g); ++p)
    {
        (void)maniGraphInstance_SetParameter(instance, p, maniGraphParameterTrigger,
                                             (maniGraphValue){.b = 1});
        (void)maniGraphInstance_SetPose(instance, p, c->pose);
        (void)maniGraphInstance_GetPose(instance, p, c->spare);
    }
    for (uint32_t p = 0; p < maniGraph_GetNodeCount(g); ++p)
    {
        maniGraphNodeView view;
        (void)maniGraphInstance_InspectNode(instance, p, &view);
    }
    if (maniGraphInstance_Save(instance, out->bytes, sizeof out->bytes, &out->size) != mani_success)
    {
        out->size = 0;
        return;
    }
    (void)maniGraphInstance_Restore(other, out->bytes, out->size);
}

// Two instances made on binding in heap blocks of their exact length
// and every call run on them, the state in out; whether they were made.
static inline bool ChangedInstances(ChangedRun* c, const maniGraphBinding* binding,
                                    ChangedState* out)
{
    size_t instanceSize = maniGraphBinding_GetInstanceSize(binding);
    uint8_t* instanceFree = NULL;
    uint8_t* otherFree = NULL;
    uint8_t* instanceMemory = TestExactCopy(instanceSize, &instanceFree);
    uint8_t* otherMemory = TestExactCopy(instanceSize, &otherFree);
    maniGraphInstance* instance = NULL;
    maniGraphInstance* other = NULL;
    bool made =
        maniInitGraphInstance(binding, instanceMemory, instanceSize, &instance) == mani_success &&
        maniInitGraphInstance(binding, otherMemory, instanceSize, &other) == mani_success;
    if (made)
    {
        ChangedCalls(c, instance, other, out);
    }
    free(otherFree);
    free(instanceFree);
    return made;
}

// The graph's getters: every name a span or none, every input and map
// within the graph or none.
static inline void ChangedGetters(const maniGraph* g)
{
    for (uint32_t p = 0; p < maniGraph_GetNodeCount(g); ++p)
    {
        for (uint32_t k = 0; k < maniGraph_GetNodeInputCount(g, p); ++k)
        {
            uint32_t input = maniGraph_GetNodeInput(g, p, k);
            CHECK(input == MANI_GRAPH_NONE || input < maniGraph_GetNodeCount(g),
                  "an input a node of the graph or none");
        }
    }
    for (uint32_t p = 0; p < maniGraph_GetParameterCount(g); ++p)
    {
        size_t length = 0;
        const char* name = maniGraph_GetParameterName(g, p, &length);
        CHECK(name != NULL || length == 0, "a name or none");
    }
    for (uint32_t k = 0; k < maniGraph_GetClipCount(g); ++k)
    {
        size_t length = 0;
        const char* name = maniGraph_GetClipName(g, k, &length);
        uint32_t map = maniGraph_GetClipRetargetMap(g, k);
        CHECK((name != NULL || length == 0) &&
                  (map == MANI_NO_TRACK || map < maniGraph_GetRetargetMapCount(g)),
              "a clip's name and map, or none");
    }
}

// One run over the bytes as they are now. After the bind: instances made
// on the binding made before any change, the state of new ones in
// c->kept, and the instance made then going on. Then bound again in a
// heap block of its exact length and instances made on that, the state
// in c->out; whether that bind and its instances succeeded. An instance
// larger than any host gives (bytes changed before the bind may ask for
// one, a full-body node's settings count, say) is not made, as a host
// would not make it.
static inline bool ChangedOnce(ChangedRun* c)
{
    const maniGraph* g = c->graph;
    if (c->binding != NULL)
    {
        (void)ChangedInstances(c, c->binding, &c->kept);
        ChangedCalls(c, c->instance, c->instance, &c->other);
    }
    size_t bindingSize = maniGraph_GetBindingSize(g);
    uint8_t* bindingFree = NULL;
    uint8_t* bindingMemory = TestExactCopy(bindingSize, &bindingFree);
    maniGraphBinding* binding = NULL;
    bool made =
        maniBindGraph(g, c->assets, bindingMemory, bindingSize, &binding, NULL) == mani_success &&
        maniGraphBinding_GetInstanceSize(binding) <= ((size_t)1 << 20) &&
        ChangedInstances(c, binding, &c->out);
    ChangedGetters(g);
    free(bindingFree);
    return made;
}

static inline bool ChangedSame(const ChangedState* a, const ChangedState* b)
{
    return a->size != 0 && a->size == b->size && memcmp(a->bytes, b->bytes, a->size) == 0;
}

// Whether the graph gives its reference state again, bound again and on
// the binding made before the change.
static inline bool ChangedBack(ChangedRun* c)
{
    return ChangedOnce(c) && ChangedSame(&c->out, &c->reference) &&
           ChangedSame(&c->kept, &c->reference);
}

// Every value a crafted word may hold at the word, each run, then the
// word put back; false when the graph does not give its state again.
static inline bool ChangedWord(ChangedRun* c, uint8_t* word)
{
    static const uint32_t kValues[] = {
        0u, 1u, 2u, 3u, 7u, 1000u, 0xFFFFu, 0x10000u, 0x7FFFFFFFu, 0xFFFFFFFEu, 0xFFFFFFFFu};
    uint32_t kept = maniLoadU32(word);
    for (size_t v = 0; v < sizeof kValues / sizeof kValues[0]; ++v)
    {
        maniStoreU32(word, kValues[v]);
        (void)ChangedOnce(c);
    }
    maniStoreU32(word, kept);
    return ChangedBack(c);
}

// Every node kind with some flags at a node record's first word.
static inline bool ChangedKind(ChangedRun* c, uint8_t* record)
{
    uint32_t kept = maniLoadU32(record);
    for (uint32_t kind = 0; kind <= 40; ++kind)
    {
        for (uint32_t flags = 0; flags < 4; ++flags)
        {
            maniStoreU32(record, kind | flags << 16);
            (void)ChangedOnce(c);
        }
    }
    maniStoreU32(record, 0xFFFFFFFFu);
    (void)ChangedOnce(c);
    maniStoreU32(record, kept);
    return ChangedBack(c);
}

// The whole sweep over an opened graph bound to the assets: its node
// records, inputs, data words, parameters' type, flags and name, and
// clip records' name, flags and map, and databases' clip runs, each
// changed after a bind and before another. what names the graph in the failures.
static inline void TestChangedGraph(const maniGraph* g, const maniGraphAssets* assets,
                                    const char* what)
{
    static _Alignas(32) uint8_t poseMemory[2][8192];
    static ChangedRun c;
    memset(&c, 0, sizeof c);
    c.graph = g;
    c.assets = assets;
    CHECK(maniInitPose(assets->skeleton, poseMemory[0], sizeof poseMemory[0], &c.pose) ==
                  mani_success &&
              maniInitPose(assets->skeleton, poseMemory[1], sizeof poseMemory[1], &c.spare) ==
                  mani_success &&
              ChangedOnce(&c) && c.out.size != 0,
          what);
    c.reference = c.out;
    size_t bindingSize = maniGraph_GetBindingSize(g);
    uint8_t* bindingMemory = TestExactCopy(bindingSize, &c.bindingFree);
    CHECK(maniBindGraph(g, assets, bindingMemory, bindingSize, &c.binding, NULL) == mani_success,
          what);
    size_t instanceSize = maniGraphBinding_GetInstanceSize(c.binding);
    uint8_t* instanceMemory = TestExactCopy(instanceSize, &c.instanceFree);
    CHECK(maniInitGraphInstance(c.binding, instanceMemory, instanceSize, &c.instance) ==
              mani_success,
          what);
    bool back = true;
    for (uint32_t p = 0; p < g->nodeCount; ++p)
    {
        uint8_t* record = (uint8_t*)(uintptr_t)g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        back = ChangedKind(&c, record) && back;
        for (uint32_t at = 4; at < MANI_GRAPH_NODE_SIZE; at += 4)
        {
            back = ChangedWord(&c, record + at) && back;
        }
    }
    for (uint32_t i = 0; i < g->inputCount; ++i)
    {
        back = ChangedWord(&c, (uint8_t*)(uintptr_t)g->inputs + 4 * (size_t)i) && back;
    }
    for (uint32_t w = 0; w < g->dataWords; ++w)
    {
        back = ChangedWord(&c, (uint8_t*)(uintptr_t)g->data + 4 * (size_t)w) && back;
    }
    for (uint32_t p = 0; p < g->parameterCount; ++p)
    {
        uint8_t* entry = (uint8_t*)(uintptr_t)g->parameters + (size_t)p * MANI_GRAPH_PARAMETER_SIZE;
        for (uint32_t at = 0; at < 16; at += 4)
        {
            back = ChangedWord(&c, entry + at) && back;
        }
    }
    for (uint32_t k = 0; k < g->clipCount; ++k)
    {
        uint8_t* entry = (uint8_t*)(uintptr_t)g->clips + (size_t)k * MANI_GRAPH_CLIP_SIZE;
        for (uint32_t at = 16; at < MANI_GRAPH_CLIP_SIZE; at += 4)
        {
            back = ChangedWord(&c, entry + at) && back;
        }
    }
    for (uint32_t k = 0; k < g->databaseCount; ++k)
    {
        // The first of its clips in the list, and their count.
        uint8_t* entry = (uint8_t*)(uintptr_t)g->databases + (size_t)k * MANI_GRAPH_DATABASE_SIZE;
        back = ChangedWord(&c, entry + 24) && back;
        back = ChangedWord(&c, entry + 28) && back;
    }
    CHECK(back, what);
    free(c.instanceFree);
    free(c.bindingFree);
}

#endif // MAUL_ANIM_TEST_GRAPH_CHANGED_H
