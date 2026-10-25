// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The playheads of an update's clip nodes: a clip node's clip and where
// a time falls in it, shared by the update and its sync groups
// (graph_sync.c), which lead each group and move its members.

#ifndef MAUL_ANIM_SRC_GRAPH_SYNC_H
#define MAUL_ANIM_SRC_GRAPH_SYNC_H

#include "bytes.h"
#include "graph_emit.h"
#include "graph_file.h"

#include "maul-anim/clip.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static inline const uint8_t* maniGraphUpdateRecord(const maniGraphUpdate* u, uint32_t p)
{
    return u->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
}

// A clip node's sync group (or a number past the groups for none).
static inline uint32_t maniGraphUpdateGroupOf(const maniGraphUpdate* u, const uint8_t* record)
{
    return maniGraphUpdateWord(u, maniLoadU32(record + 12) + 1);
}

// A clip node's clip, its loop as the node says.
static inline maniClip maniGraphUpdateClip(const maniGraphUpdate* u, uint32_t data)
{
    maniClip clip = *u->binding->clips[maniGraphUpdateWord(u, data)];
    uint32_t loop = maniGraphUpdateWord(u, data + 2);
    clip.loop = loop == 0 ? clip.loop : (loop == 1 ? maniLoopClamp : maniLoopRepeat);
    return clip;
}

static inline bool maniGraphClipRepeats(const maniClip* clip)
{
    return clip->loop == maniLoopRepeat && clip->duration > 0.0f;
}

// A time placed in the clip: wrapped into it, or clamped to it.
static inline double maniGraphClipPlace(const maniClip* clip, double t)
{
    double d = (double)clip->duration;
    if (!maniGraphClipRepeats(clip))
    {
        return t < 0.0 ? 0.0 : (t > d ? d : t);
    }
    double w = fmod(t, d);
    w = w < 0.0 ? w + d : w;
    return w < d ? w : 0.0;
}

// How a clip node moves this update: from t0 by step, or set to t0 by a
// seek (nothing passed); with its events or not.
typedef struct maniGraphMotion
{
    double t0;
    double step;
    bool seeking;
    bool events;
} maniGraphMotion;

// Each sync group's leader chosen, after the weights, and how it moves.
void maniGraphLead(const maniGraphUpdate* u);

// Clip node p's motion this update, in group (or none): a follower's as
// its group's leader moves, else its own by its speed and time factor
// or its seek.
maniGraphMotion maniGraphClipMotion(const maniGraphUpdate* u, uint32_t p, uint32_t group,
                                    const maniClip* clip, uint32_t data);

#endif // MAUL_ANIM_SRC_GRAPH_SYNC_H
