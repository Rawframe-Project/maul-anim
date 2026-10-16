// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An update in progress (mani-0023), shared by the update and the motion
// matching nodes' step: its instance, events and root motion, the events
// a stretch of a clip passes, and an inertialize node's request.

#ifndef MAUL_ANIM_SRC_GRAPH_EMIT_H
#define MAUL_ANIM_SRC_GRAPH_EMIT_H

#include "bytes.h"
#include "graph_instance.h"
#include "root_motion.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct maniGraphUpdate
{
    maniGraphInstance* instance;
    const maniGraph* graph;
    const maniGraphBinding* binding;
    double delta;
    maniGraphEvent* events;
    uint32_t capacity;
    uint32_t reserved;
    uint64_t count;
    maniRootMotionSum root;
} maniGraphUpdate;

static inline maniGraphNodeState* maniGraphUpdateNode(const maniGraphUpdate* u, uint32_t p)
{
    return &u->instance->cells[u->graph->parameterCount + p].node;
}

static inline uint32_t maniGraphUpdateWord(const maniGraphUpdate* u, uint32_t at)
{
    return maniLoadU32(u->graph->data + 4 * (size_t)at);
}

// A scalar's value: its float parameter's, or its literal.
static inline float maniGraphUpdateScalar(const maniGraphUpdate* u, uint32_t at)
{
    uint32_t parameter = maniGraphUpdateWord(u, at);
    if (parameter != MANI_NO_PARAMETER)
    {
        return u->instance->cells[parameter].value.f[0];
    }
    uint32_t bits = maniGraphUpdateWord(u, at + 1);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// The events of one stretch of a clip node's (or motion matching node's)
// clip slot, from one clip time to another, listed while the buffer
// lasts and counted.
void maniGraphEmit(maniGraphUpdate* u, uint32_t node, uint32_t slot, const maniClip* clip,
                   double from, double to);

// Whole loops of a stretch (from to to, either way): listed while the
// buffer lasts, counted beyond it (at most 2^32 loops).
void maniGraphEmitLoops(maniGraphUpdate* u, uint32_t node, uint32_t slot, const maniClip* clip,
                        double from, double to, double loops);

// An inertialize node's request of seconds (none for MANI_GRAPH_NO_NODE
// or seconds not at least 0): the least of an update's.
void maniGraphRequest(const maniGraphUpdate* u, uint32_t node, float seconds);

#endif // MAUL_ANIM_SRC_GRAPH_EMIT_H
