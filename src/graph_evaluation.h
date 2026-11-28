// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph evaluation's state, shared by the evaluation and the
// nodes evaluated in modules of their own: where the scratch's parts
// begin, the instance, binding and plan, and reading a node's record,
// words and inputs. The bytes stay in the caller's hands after the open:
// every index read from them is bounded where it is read.

#ifndef MAUL_ANIM_SRC_GRAPH_EVALUATION_H
#define MAUL_ANIM_SRC_GRAPH_EVALUATION_H

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_subset.h"
#include "invariant.h"

#include "maul-anim/blend.h"
#include "maul-anim/graph.h"
#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Where the scratch's parts begin: layers, bone weights, the poses, then
// with map slots the room a mapped clip's pose is carried in (two
// source poses, a pose of the graph's skeleton, the transfer's scratch).
typedef struct maniGraphEvalParts
{
    size_t layersAt;
    size_t weightsAt;
    size_t posesAt;
    size_t poseSize;
    size_t retargetAt;
    size_t size;
} maniGraphEvalParts;

typedef struct maniGraphEvaluation
{
    maniGraphInstance* instance;
    const maniGraphBinding* binding;
    const maniGraph* graph;
    uint8_t* scratch;
    maniGraphEvalParts parts;
    maniBlendLayer* layers;
    float* inside;
    float* outside;
    // A subset's plan and its groups' flags, or NULL for every bone.
    const maniGraphSubset* plan;
    const uint8_t* groups;
    // Set nonzero once a node's data failed: the evaluation reports
    // mani_errorFormat.
    uint32_t* failed;
} maniGraphEvaluation;

// A pose slot read from the bytes: opening refused one past the file's
// count, and one past it since names the last (maniGraphBoundSlot; a graph
// has a slot for its output).
static inline maniPose* maniGraphEvalSlot(const maniGraphEvaluation* e, uint32_t slot)
{
    slot = maniGraphBoundSlot(slot, e->graph->slotCount);
    void* memory = e->scratch + e->parts.posesAt + (size_t)slot * e->parts.poseSize;
    return memory;
}

static inline const uint8_t* maniGraphEvalRecord(const maniGraphEvaluation* e, uint32_t p)
{
    MANI_ASSERT(p < e->graph->nodeCount);
    return e->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
}

static inline maniGraphNodeState* maniGraphEvalState(const maniGraphEvaluation* e, uint32_t p)
{
    return &e->instance->cells[e->graph->parameterCount + p].node;
}

// Data word at, 0 past the words (maniGraphWord).
static inline uint32_t maniGraphEvalWord(const maniGraphEvaluation* e, uint32_t at)
{
    return maniGraphWord(e->graph, at);
}

// A parameter's value; NULL for none and past the count.
static inline const maniGraphValue* maniGraphEvalValue(const maniGraphEvaluation* e,
                                                       uint32_t parameter)
{
    return maniGraphParameterValue(e->graph, e->instance->cells, parameter);
}

// A scalar's value: its float parameter's, or its literal (also for a
// parameter past the count).
static inline float maniGraphEvalScalar(const maniGraphEvaluation* e, uint32_t at)
{
    const maniGraphValue* v = maniGraphEvalValue(e, maniGraphEvalWord(e, at));
    if (v != NULL)
    {
        return v->f[0];
    }
    uint32_t bits = maniGraphEvalWord(e, at + 1);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// A node's input k; MANI_GRAPH_NO_NODE past its inputs
// (maniGraphInputCount) or past the nodes.
static inline uint32_t maniGraphEvalInputOf(const maniGraphEvaluation* e, const uint8_t* record,
                                            uint32_t k)
{
    if (k >= maniGraphInputCount(e->graph, record))
    {
        return MANI_GRAPH_NO_NODE;
    }
    uint32_t input = maniLoadU32(e->graph->inputs + 4 * (size_t)(maniLoadU32(record + 4) + k));
    return input < e->graph->nodeCount ? input : MANI_GRAPH_NO_NODE;
}

// A node's input count, at most the layers the scratch holds (the
// binding's most inputs).
static inline uint32_t maniGraphEvalInputCount(const maniGraphEvaluation* e, const uint8_t* record)
{
    uint32_t n = maniGraphInputCount(e->graph, record);
    return n < e->binding->maxInputs ? n : e->binding->maxInputs;
}

// An input's pose, or NULL for none or when its last update left it
// unweighed.
static inline const maniPose* maniGraphEvalInput(const maniGraphEvaluation* e,
                                                 const uint8_t* record, uint32_t k)
{
    uint32_t input = maniGraphEvalInputOf(e, record, k);
    if (input == MANI_GRAPH_NO_NODE || !(maniGraphEvalState(e, input)->weight > 0.0f))
    {
        return NULL;
    }
    return maniGraphEvalSlot(e, maniLoadU32(maniGraphEvalRecord(e, input) + 20));
}

// A pose of no weight: the rest, or the difference of nothing.
static inline void maniGraphEvalNeutral(const maniGraphEvaluation* e, maniPose* out,
                                        bool difference)
{
    (void)maniPose_SetRest(out, e->binding->skeleton);
    if (difference)
    {
        (void)maniSkeleton_ComputePoseDifference(e->binding->skeleton, out, out, out);
    }
}

#endif // MAUL_ANIM_SRC_GRAPH_EVALUATION_H
