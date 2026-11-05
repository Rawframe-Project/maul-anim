// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's structure and an instance's nodes read for a debugger:
// a node's kind, inputs and place in the source from its
// record, and what its last
// update left in its state and its kind's block (a machine's state and
// transition under way, a one-shot's phase, an inertialization's easing, a
// motion matching node's entry). Reads only.

#include "bytes.h"
#include "fpenv.h"
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

maniGraphNodeKind maniGraph_GetNodeKind(const maniGraph* graph, uint32_t node)
{
    return graph != NULL && node < graph->nodeCount ? maniLoadU16(Record(graph, node)) : 0u;
}

uint32_t maniGraph_GetNodeInputCount(const maniGraph* graph, uint32_t node)
{
    return graph != NULL && node < graph->nodeCount
               ? maniGraphInputCount(graph, Record(graph, node))
               : 0u;
}

uint32_t maniGraph_GetNodeInput(const maniGraph* graph, uint32_t node, uint32_t input)
{
    if (input >= maniGraph_GetNodeInputCount(graph, node))
    {
        return MANI_GRAPH_NONE;
    }
    uint32_t first = maniLoadU32(Record(graph, node) + 4);
    uint32_t found = maniLoadU32(graph->inputs + 4 * ((size_t)first + input));
    return found < graph->nodeCount ? found : MANI_GRAPH_NONE;
}

uint32_t maniGraph_GetOutputNode(const maniGraph* graph)
{
    return graph != NULL && graph->nodeCount > 0 ? graph->nodeCount - 1 : MANI_GRAPH_NONE;
}

uint32_t maniGraph_GetNodeSourcePlace(const maniGraph* graph, uint32_t node)
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

// A machine's state, transition under way and its fade.
static void Machine(const maniGraph* graph, const uint8_t* e, const uint32_t* block,
                    maniGraphNodeView* v)
{
    maniGraphMachine m = {graph, maniLoadU32(e + 12), maniLoadU32(e + 8)};
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
static void Block(const uint32_t* block, maniGraphNodeView* v)
{
    if (v->kind == maniGraphNodeOneShot)
    {
        v->current = block[maniGraphShotPhase];
        v->progress = FloatOf(block[maniGraphShotWeight]);
    }
    else if (v->kind == maniGraphNodeInertialize)
    {
        bool active = (block[maniGraphInertiaFlags] & maniGraphInertiaActive) != 0;
        v->progress = active ? Progress(maniGraphInertiaGet(block, maniGraphInertiaElapsed),
                                        maniGraphInertiaGet(block, maniGraphInertiaDuration))
                             : 0.0f;
    }
    else if (v->kind == maniGraphNodeMatch)
    {
        v->current = block[maniGraphMatchEntry];
    }
}

static MANI_NOINLINE maniResult InspectGraphNodeWork(const maniGraphInstance* instance,
                                                     uint32_t node, maniGraphNodeView* out)
{
    if (instance == NULL || instance->stamp != MANI_GRAPH_INSTANCE_STAMP || out == NULL ||
        node >= instance->binding->graph.nodeCount)
    {
        return mani_errorInvalid;
    }
    const maniGraph* graph = &instance->binding->graph;
    const maniGraphNodeState* s = &instance->cells[graph->parameterCount + node].node;
    const uint8_t* e = Record(graph, node);
    // A node whose block does not fit (the bytes changed since the open)
    // shows none of it.
    const uint32_t* block = maniGraphNodeBlockRead(instance, node);
    maniGraphNodeView v = {maniLoadU16(e),
                           s->weight,
                           0.0,
                           MANI_GRAPH_NONE,
                           MANI_GRAPH_NONE,
                           0.0f,
                           (s->flags & MANI_GRAPH_FAILED) != 0 ? 1u : 0u};
    if (v.kind == maniGraphNodeClip || v.kind == maniGraphNodeMatch)
    {
        v.time = s->time;
    }
    if (v.kind == maniGraphNodeSelect)
    {
        v.current = s->word;
    }
    else if (block != NULL && v.kind == maniGraphNodeMachine)
    {
        Machine(graph, e, block, &v);
    }
    else if (block != NULL)
    {
        Block(block, &v);
    }
    *out = v;
    return mani_success;
}

maniResult maniGraphInstance_InspectNode(const maniGraphInstance* instance, uint32_t node,
                                         maniGraphNodeView* viewOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = InspectGraphNodeWork(instance, node, viewOut);
    maniFloatEnvLeave(env);
    return result;
}
