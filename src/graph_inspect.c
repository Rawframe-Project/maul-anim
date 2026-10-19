// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's structure and an instance's nodes read for a debugger
// (mani-0035): a node's kind, inputs and place in the source from its
// record, and what its last
// update left in its state and its kind's block (a machine's state and
// transition under way, a one-shot's phase, an inertialization's easing, a
// motion matching node's entry). Reads only.

#include "bytes.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_instance.h"
#include "graph_machine.h"
#include "graph_match.h"
#include "graph_shot.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const uint8_t* Record(const maniGraph* graph, uint32_t node)
{
    return graph->nodes + (size_t)node * MANI_GRAPH_NODE_SIZE;
}

maniGraphNodeKind maniGraphNodeKindOf(const maniGraph* graph, uint32_t node)
{
    return graph != NULL && node < graph->nodeCount ? maniLoadU16(Record(graph, node)) : 0u;
}

uint32_t maniGraphNodeInputCount(const maniGraph* graph, uint32_t node)
{
    return graph != NULL && node < graph->nodeCount ? maniLoadU32(Record(graph, node) + 8) : 0u;
}

uint32_t maniGraphNodeInput(const maniGraph* graph, uint32_t node, uint32_t input)
{
    if (input >= maniGraphNodeInputCount(graph, node))
    {
        return MANI_GRAPH_NONE;
    }
    uint32_t first = maniLoadU32(Record(graph, node) + 4);
    return maniLoadU32(graph->inputs + 4 * ((size_t)first + input));
}

uint32_t maniGraphOutputNode(const maniGraph* graph)
{
    return graph != NULL && graph->nodeCount > 0 ? graph->nodeCount - 1 : MANI_GRAPH_NONE;
}

uint32_t maniGraphNodeSourcePlace(const maniGraph* graph, uint32_t node)
{
    return graph != NULL && node < graph->nodeCount ? maniLoadU32(Record(graph, node) + 24)
                                                    : MANI_GRAPH_NONE;
}

static float FloatOf(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// Elapsed over duration, 1 for a duration of 0.
static float Progress(float elapsed, float duration)
{
    return duration > 0.0f ? elapsed / duration : 1.0f;
}

// The data word at of the node whose data words begin at data.
static uint32_t Word(const maniGraph* graph, uint32_t data, uint32_t at)
{
    return maniLoadU32(graph->data + 4 * ((size_t)data + at));
}

// A machine's state, transition under way and its fade.
static void Machine(const maniGraph* graph, const uint8_t* e, const uint32_t* blocks,
                    maniGraphNodeView* v)
{
    maniGraphMachine m = {graph, maniLoadU32(e + 12), maniLoadU32(e + 8)};
    const uint32_t* block = blocks + maniGraphMachineBlockAt(&m);
    uint32_t t = block[maniGraphBlockRunning];
    v->current = block[maniGraphBlockCurrent];
    if (t != MANI_GRAPH_NO_TRANSITION)
    {
        v->transition = t;
        v->progress = Progress(maniGraphBlockFloat(block, maniGraphBlockElapsed),
                               FloatOf(maniGraphMachineTransition(&m, t, 4)));
    }
}

// The kind's block's words: a one-shot's phase and weight, an
// inertialization's easing, a motion matching node's entry.
static void Block(const maniGraph* graph, const uint8_t* e, const uint32_t* blocks,
                  maniGraphNodeView* v)
{
    uint32_t data = maniLoadU32(e + 12);
    if (v->kind == maniGraphNodeOneShot)
    {
        maniGraphShot shot = {graph, data, 0};
        const uint32_t* block = blocks + maniGraphShotWord(&shot, maniGraphShotBlock);
        v->current = block[maniGraphShotPhase];
        v->progress = FloatOf(block[maniGraphShotWeight]);
    }
    else if (v->kind == maniGraphNodeInertialize)
    {
        const uint32_t* block = blocks + Word(graph, data, maniGraphInertiaBlock);
        bool active = (block[maniGraphInertiaFlags] & maniGraphInertiaActive) != 0;
        v->progress = active ? Progress(maniGraphInertiaGet(block, maniGraphInertiaElapsed),
                                        maniGraphInertiaGet(block, maniGraphInertiaDuration))
                             : 0.0f;
    }
    else if (v->kind == maniGraphNodeMatch)
    {
        v->current = blocks[Word(graph, data, maniGraphMatchBlock) + maniGraphMatchEntry];
    }
}

maniResult maniInspectGraphNode(const maniGraphInstance* instance, uint32_t node,
                                maniGraphNodeView* out)
{
    if (instance == NULL || instance->stamp != MANI_GRAPH_INSTANCE_STAMP || out == NULL ||
        node >= instance->binding->graph.nodeCount)
    {
        return mani_errorInvalid;
    }
    const maniGraph* graph = &instance->binding->graph;
    const maniGraphNodeState* s = &instance->cells[graph->parameterCount + node].node;
    const uint8_t* e = Record(graph, node);
    const uint32_t* blocks = (const uint32_t*)((const uint8_t*)instance + instance->machinesAt);
    maniGraphNodeView v = {maniLoadU16(e),  s->weight, 0.0, MANI_GRAPH_NONE,
                           MANI_GRAPH_NONE, 0.0f,      0u};
    if (v.kind == maniGraphNodeClip || v.kind == maniGraphNodeMatch)
    {
        v.time = s->time;
    }
    if (v.kind == maniGraphNodeSelect)
    {
        v.current = s->word;
    }
    else if (v.kind == maniGraphNodeMachine)
    {
        Machine(graph, e, blocks, &v);
    }
    else
    {
        Block(graph, e, blocks, &v);
    }
    *out = v;
    return mani_success;
}
