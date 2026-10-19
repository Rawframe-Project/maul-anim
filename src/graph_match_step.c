// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node reached with a weight moves forward by the magnitude of its time
// factor times the update's delta, through its entry (one unweighed, as
// under a state or select not chosen, stays and does not search; a seek
// above clears it even so): a looping one wraps from its end
// to its start (events of each stretch listed, the root's move composed
// across), a clamped one stops at its end. A seek above it clears it.
// Then it searches when it has no entry, its clamped entry reached its
// end, its trigger is set (consumed) or its period passed since the last
// search. The query's bone channels are the continuing frame's features
// as the database keeps them (normalized; 0, their mean, with no entry);
// its trajectory and float channels its parameters, normalized as the
// database's. At a clamped end the continuing frame is not a candidate
// (Holden's controller searches from none there), so the end exclusion
// keeps it from picking frames it cannot play. A pick other than the
// continuing frame is jumped to, kept in the history, and, when the node
// had an entry, requests inertialization of the nearest node above for
// the node's duration. A query that does not stay finite once normalized
// searches nothing.

#include "graph_match_step.h"

#include "bytes.h"
#include "graph_emit.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_match.h"
#include "match_file.h"
#include "match_move.h"
#include "match_search.h"
#include "root_motion.h"

#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/match.h"
#include "maul-anim/mirror.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float FloatOf(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static uint32_t BitsOf(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

// The node being moved: its words, block, state and database.
typedef struct Match
{
    maniGraphUpdate* u;
    maniGraphNodeState* s;
    uint32_t* block;
    const maniMotionDatabase* d;
    uint32_t p;
    uint32_t data;
} Match;

static uint32_t Word(const Match* m, uint32_t k)
{
    return maniGraphUpdateWord(m->u, m->data + k);
}

// The clip slot of the database's clip.
static uint32_t ClipSlot(const Match* m, uint32_t clip)
{
    const maniGraph* g = m->u->graph;
    const uint8_t* e =
        g->databases + (size_t)Word(m, maniGraphMatchDatabase) * MANI_GRAPH_DATABASE_SIZE;
    return maniLoadU32(g->databaseClips + 4 * (size_t)(maniLoadU32(e + 24) + clip));
}

// The events of a move from t0 by step through a span, across wraps.
static void Stretches(const Match* m, uint32_t slot, const maniClip* clip, const maniMatchEntry* e,
                      const maniMatchPlace* at)
{
    double t0 = m->s->from;
    if (at->wraps == 0.0)
    {
        maniGraphEmit(m->u, m->p, slot, clip, t0, (double)at->time);
        return;
    }
    maniGraphEmit(m->u, m->p, slot, clip, t0, (double)e->end);
    maniGraphEmitLoops(m->u, m->p, slot, clip, (double)e->start, (double)e->end, at->wraps - 1.0);
    maniGraphEmit(m->u, m->p, slot, clip, (double)e->start, (double)at->time);
}

// The events listed; a mirrored entry's renamed through the map (again
// over the node's own renaming, which the map undoes).
static void Events(const Match* m, uint32_t slot, const maniClip* clip, const maniMatchEntry* e,
                   const maniMatchPlace* at)
{
    maniGraphUpdate* u = m->u;
    uint64_t first = u->count;
    Stretches(m, slot, clip, e, at);
    for (uint64_t i = first;
         (e->flags & MANI_MATCH_MIRRORED) != 0 && i < u->count && i < u->capacity; ++i)
    {
        u->events[i].nameHash = maniMirrorNameHash(u->binding->mirrorMap, u->events[i].nameHash);
    }
}

// The playhead moved through its entry; whether a clamped entry reached
// its end.
static bool Play(const Match* m, double step)
{
    maniGraphNodeState* s = m->s;
    s->from = s->time;
    s->step = 0.0;
    maniMatchEntry e;
    if (maniMotionDatabaseEntry(m->d, m->block[maniGraphMatchEntry], &e) != mani_success)
    {
        return false;
    }
    uint32_t slot = ClipSlot(m, e.clip);
    maniClip clip = *m->u->binding->clips[slot];
    clip.loop = maniLoopClamp;
    bool loop = (e.flags & MANI_MATCH_LOOP) != 0;
    maniMatchPlace at =
        maniMatchLocate(&clip, (maniMatchSpan){e.start, e.end, loop}, (float)s->time, step);
    if (s->weight > MANI_GRAPH_EVENT_THRESHOLD)
    {
        s->flags |= MANI_GRAPH_LISTED;
        Events(m, slot, &clip, &e, &at);
    }
    if (s->rootWeight > 0.0f)
    {
        // A mirrored entry's move reflected, then the node's.
        maniRootMove move = (e.flags & MANI_MATCH_MIRRORED) != 0
                                ? maniMirrorRootMotion(m->u->binding->mirrorMap, at.move)
                                : at.move;
        move = maniGraphNodeMove(m->u, m->p, move);
        maniRootMotionSumAdd(&m->u->root, &move, (double)s->rootWeight);
    }
    s->step = step;
    s->time = (double)at.time;
    return !loop && !(at.time < e.end);
}

// The frame of the entry playing: the last whose time is at most the
// playhead (MANI_MATCH_NO_FRAME with no entry).
static uint32_t Continuing(const Match* m)
{
    maniMatchEntry e;
    if (maniMotionDatabaseEntry(m->d, m->block[maniGraphMatchEntry], &e) != mani_success)
    {
        return MANI_MATCH_NO_FRAME;
    }
    float rate = m->d->sampleRate;
    double t = m->s->time;
    double k = floor((t - (double)e.start) * (double)rate);
    uint32_t last = e.frameCount - 1;
    uint32_t n = k <= 0.0 ? 0u : (k >= (double)last ? last : (uint32_t)k);
    if (n < last && (double)maniMatchFrameTime(e.start, n + 1, rate) <= t)
    {
        ++n;
    }
    else if (n > 0 && (double)maniMatchFrameTime(e.start, n, rate) > t)
    {
        --n;
    }
    return e.firstFrame + n;
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// The query: the continuing frame's bone channels, the parameters'
// trajectory and float channels, normalized.
static void Query(const Match* m, uint32_t current, float* out)
{
    const maniMotionDatabase* d = m->d;
    uint32_t n = d->featureCount;
    if (maniMotionDatabaseFeatures(d, current, out) != mani_success)
    {
        memset(out, 0, (size_t)n * sizeof(float));
    }
    uint32_t q = 0;
    for (uint32_t c = 0; c < d->channelCount; ++c)
    {
        maniMatchChannel ch;
        (void)maniMotionDatabaseChannel(d, c, &ch);
        uint32_t width = maniMatchChannelWidth(ch.kind);
        bool vector =
            ch.kind == maniMatchTrajectoryPosition || ch.kind == maniMatchTrajectoryFacing;
        for (uint32_t t = 0; t < (vector || ch.kind == maniMatchFloatTrack ? ch.timeCount : 0u);
             ++t, ++q)
        {
            const maniGraphValue* v =
                &m->u->instance->cells[Word(m, maniGraphMatchQuery + q)].value;
            for (uint32_t i = 0; i < width; ++i)
            {
                uint32_t j = ch.firstFeature + t * width + i;
                float offset = LoadF32(d->normalization + 4 * (size_t)j);
                float scale = LoadF32(d->normalization + 4 * (size_t)(n + j));
                out[j] = (v->f[i] - offset) / scale;
            }
        }
    }
}

// Tags: an int parameter's bits, or the literal.
static uint32_t Tags(const Match* m, uint32_t k)
{
    uint32_t parameter = Word(m, k);
    return parameter == MANI_NO_PARAMETER ? Word(m, k + 1)
                                          : (uint32_t)m->u->instance->cells[parameter].value.i;
}

// The pick jumped to and kept; inertialization requested when the node
// had an entry.
static void Jump(const Match* m, uint32_t frame)
{
    uint32_t entry = 0;
    float time = 0.0f;
    (void)maniMotionDatabaseFrame(m->d, frame, &entry, &time);
    bool had = m->block[maniGraphMatchEntry] != MANI_GRAPH_MATCH_NO_ENTRY;
    m->block[maniGraphMatchEntry] = entry;
    m->s->time = (double)time;
    uint32_t history = Word(m, maniGraphMatchHistoryCount);
    if (history > 0)
    {
        uint32_t next = m->block[maniGraphMatchNext];
        m->block[maniGraphMatchHistory + next] = frame;
        m->block[maniGraphMatchNext] = next + 1 < history ? next + 1 : 0u;
        uint32_t picks = m->block[maniGraphMatchPicks];
        m->block[maniGraphMatchPicks] = picks < history ? picks + 1 : picks;
    }
    if (had)
    {
        maniGraphRequest(m->u, m->s->inertia,
                         maniGraphUpdateScalar(m->u, m->data + maniGraphMatchDurationScalar));
    }
}

static void Search(const Match* m, bool ended)
{
    float features[MANI_MATCH_MAX_FEATURES];
    uint32_t current = Continuing(m);
    Query(m, current, features);
    maniMatchQuery q = {
        .features = features,
        .current = ended ? MANI_MATCH_NO_FRAME : current,
        .requiredTags = Tags(m, maniGraphMatchRequiredTags),
        .excludedTags = Tags(m, maniGraphMatchExcludedTags),
        .historyCount = m->block[maniGraphMatchPicks],
        .continuingBias = FloatOf(Word(m, maniGraphMatchContinuingBias)),
        .transitionCost = FloatOf(Word(m, maniGraphMatchTransitionCost)),
        .jumpThreshold = FloatOf(Word(m, maniGraphMatchJumpThreshold)),
        .endExclusion = FloatOf(Word(m, maniGraphMatchEndExclusion)),
        .historyTime = FloatOf(Word(m, maniGraphMatchHistoryTime)),
    };
    memcpy(q.history, m->block + maniGraphMatchHistory, sizeof q.history);
    maniMatchResult r;
    if (maniMatchSearch(m->d, &q, true, &r) == mani_success && r.frame != MANI_MATCH_NO_FRAME &&
        r.frame != current)
    {
        Jump(m, r.frame);
    }
}

void maniGraphAdvanceMatch(maniGraphUpdate* u, uint32_t p)
{
    maniGraphNodeState* s = maniGraphUpdateNode(u, p);
    if ((s->flags & MANI_GRAPH_REACHED) == 0)
    {
        s->from = s->time;
        s->step = 0.0;
        return;
    }
    uint32_t data = maniLoadU32(u->graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    uint32_t* block = (uint32_t*)((uint8_t*)u->instance + u->instance->machinesAt) +
                      maniGraphUpdateWord(u, data + maniGraphMatchBlock);
    Match m = {u, s,   block, maniGraphBoundDatabase(u->binding, maniGraphUpdateWord(u, data)),
               p, data};
    if ((s->flags & MANI_GRAPH_SEEKING) != 0)
    {
        maniGraphResetMatch(block);
    }
    if (!(s->weight > 0.0f))
    {
        s->from = s->time;
        s->step = 0.0;
        return;
    }
    double step = fabs(u->delta * (double)s->factor);
    bool ended = Play(&m, step);
    double clock = (double)FloatOf(block[maniGraphMatchClock]) + step;
    uint32_t trigger = Word(&m, maniGraphMatchTrigger);
    bool triggered = trigger != MANI_NO_PARAMETER && u->instance->cells[trigger].value.b != 0;
    if (triggered)
    {
        u->instance->cells[trigger].value.b = 0;
    }
    bool due = block[maniGraphMatchEntry] == MANI_GRAPH_MATCH_NO_ENTRY || ended || triggered ||
               clock >= (double)FloatOf(Word(&m, maniGraphMatchPeriod));
    block[maniGraphMatchClock] = BitsOf(due ? 0.0f : (float)clock);
    if (due)
    {
        Search(&m, ended);
    }
}
