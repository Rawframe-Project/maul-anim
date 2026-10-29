// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The exact search: the query normalized as the features
// were; the continuing frame's cost the first best; then the tree's
// nodes nearest first, a node skipped when its box's bound shows it can
// hold no better frame, and each leaf's frames scored, every sum stopped
// once it passes the best. A frame's cost is a constant (its entry's
// bias plus the transition cost, or for the continuing frame the
// continuing bias) plus its squared distances in feature order. A box's
// bound starts from the transition cost plus the least bias of its
// frames' entries and adds its squared distances to the box in the same
// order: each term is at most any of its frames' and binary32 rounding is
// monotone, so a box never claims less than one of its frames would
// cost, and the search finds the plain scan's frame and cost bit for
// bit. Frames come out of order, so ties are settled as the scan settles
// them: the continuing frame first, then the lowest frame; a node whose
// bound equals the best is skipped only when it cannot hold a lower
// frame.

#include "match_search.h"

#include "bytes.h"
#include "fpenv.h"
#include "invariant.h"
#include "match_file.h"

#include "maul-anim/match.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

typedef struct Entry
{
    uint32_t flags;
    float start;
    float end;
    float bias;
    uint32_t tags;
    uint32_t first;
    uint32_t count;
} Entry;

static Entry EntryAt(const maniMotionDatabase* d, uint32_t i)
{
    const uint8_t* e = d->entries + (size_t)i * MANI_MATCH_ENTRY_SIZE;
    return (Entry){maniLoadU32(e + 4),  LoadF32(e + 8),      LoadF32(e + 12),    LoadF32(e + 16),
                   maniLoadU32(e + 20), maniLoadU32(e + 24), maniLoadU32(e + 28)};
}

// The entry holding a frame, as its position names it.
static uint32_t EntryOf(const maniMotionDatabase* d, uint32_t frame)
{
    size_t position = maniLoadU32(d->places + 4 * (size_t)frame);
    return maniLoadU32(d->positions + position * MANI_MATCH_POSITION_SIZE + 4);
}

// What a search holds: the database, the query, the normalized query,
// the earlier picks' entries, the best so far and whether it is the
// continuing frame, the exclusions' times as frames apart, and the last
// entry a leaf's frame came from with its frames' end for the search.
typedef struct Search
{
    const maniMotionDatabase* d;
    const maniMatchQuery* q;
    float query[MANI_MAX_MATCH_FEATURES];
    uint32_t historyEntries[MANI_MAX_MATCH_HISTORY];
    uint32_t best;
    float bestCost;
    bool continuing;
    uint32_t jumpFrames;
    uint32_t historyFrames;
    uint32_t featureCount;
    uint32_t historyCount;
    uint32_t lastEntry;
    Entry last;
    uint32_t lastStop;
    bool lastCandidate;
} Search;

static bool Candidate(const Search* s, const Entry* e)
{
    return (e->tags & s->q->requiredTags) == s->q->requiredTags &&
           (e->tags & s->q->excludedTags) == 0;
}

// Frames a and b of one entry apart: around a looping one's span the
// shorter way.
static uint32_t Apart(const Entry* e, uint32_t a, uint32_t b)
{
    uint32_t frames = a > b ? a - b : b - a;
    bool loop = (e->flags & MANI_MATCH_LOOP) != 0;
    return loop && e->count - frames < frames ? e->count - frames : frames;
}

// The fewest frames apart that are not nearer than seconds: frames
// apart are nearer exactly when fewer (frames / rate grows with the
// frames), so the count is guessed from the times and settled by that
// test; the frame count when every distance in the database is nearer.
static uint32_t FramesApart(const maniMotionDatabase* d, float seconds)
{
    double rate = (double)d->sampleRate;
    double guess = (double)seconds * rate;
    uint32_t k = guess >= (double)d->frameCount ? d->frameCount : (uint32_t)guess;
    while (k > 0 && (double)(k - 1) / rate >= (double)seconds)
    {
        --k;
    }
    while (k < d->frameCount && (double)k / rate < (double)seconds)
    {
        ++k;
    }
    return k;
}

// Excluded: near the continuing frame on its entry, or near an earlier
// pick on its.
static bool Excluded(const Search* s, uint32_t entry, const Entry* e, uint32_t frame)
{
    const maniMatchQuery* q = s->q;
    uint32_t current = q->current;
    if (current != MANI_MATCH_NO_FRAME && frame == current)
    {
        return true;
    }
    if (current != MANI_MATCH_NO_FRAME && current >= e->first && current - e->first < e->count &&
        Apart(e, frame, current) < s->jumpFrames)
    {
        return true;
    }
    for (uint32_t k = 0; k < s->historyCount; ++k)
    {
        if (s->historyEntries[k] == entry && Apart(e, frame, q->history[k]) < s->historyFrames)
        {
            return true;
        }
    }
    return false;
}

// Whether a clamped entry's frame m (from its first) is within the end
// exclusion of its end; once one is, every later one is (each step is
// monotone).
static bool NearEnd(const Search* s, const Entry* e, uint32_t m)
{
    double t = (double)e->start + (double)m / (double)s->d->sampleRate;
    return (double)e->end - t < (double)s->q->endExclusion;
}

// The end of the frames of an entry a search reaches: a clamped entry's
// frames within the end exclusion of its end left out. The first
// excluded frame is guessed from the times and then settled by NearEnd,
// so the guess's rounding never moves it.
static uint32_t Stop(const Search* s, const Entry* e)
{
    if ((e->flags & MANI_MATCH_LOOP) != 0)
    {
        return e->first + e->count;
    }
    double guess =
        ((double)e->end - (double)s->q->endExclusion - (double)e->start) * (double)s->d->sampleRate;
    uint32_t m = guess <= 0.0 ? 0 : guess >= (double)e->count ? e->count : (uint32_t)guess;
    while (m > 0 && NearEnd(s, e, m - 1))
    {
        --m;
    }
    while (m < e->count && !NearEnd(s, e, m))
    {
        ++m;
    }
    return e->first + m;
}

// The cost of features f (a frame's in the database, or the continuing
// frame's a caller gives) plus constant, stopped once past the best: a
// cost equal to it may still win a tie.
static float Cost(const Search* s, const uint8_t* f, float constant)
{
    uint32_t n = s->featureCount;
    float best = s->bestCost;
    float cost = constant;
    for (uint32_t j = 0; j < n; ++j)
    {
        float d = s->query[j] - LoadF32(f + 4 * (size_t)j);
        cost += d * d;
        if (cost > best)
        {
            break;
        }
    }
    return cost;
}

// A node's bound: from the transition cost and its least bias, its box's
// squared distances, stopped once past the best.
static float Bound(const Search* s, uint32_t node)
{
    uint32_t n = s->featureCount;
    const uint8_t* low = s->d->boxes + 4 * 2 * (size_t)node * n;
    const uint8_t* high = low + 4 * (size_t)n;
    float best = s->bestCost;
    float cost =
        s->q->transitionCost + LoadF32(s->d->nodes + (size_t)node * MANI_MATCH_NODE_SIZE + 8);
    for (uint32_t j = 0; j < n; ++j)
    {
        float q = s->query[j];
        float a = LoadF32(low + 4 * (size_t)j);
        float b = LoadF32(high + 4 * (size_t)j);
        float near = q < a ? a : (q > b ? b : q);
        float gap = q - near;
        cost += gap * gap;
        if (cost > best)
        {
            break;
        }
    }
    return cost;
}

// Whether a node of that bound can hold no better frame: its bound past
// the best, or equal to it with no frame to win the tie (the best is the
// continuing frame, or lower than every frame of the node).
static bool Pruned(const Search* s, uint32_t node, float bound)
{
    if (bound != s->bestCost)
    {
        return bound > s->bestCost;
    }
    uint32_t lowest = maniLoadU32(s->d->nodes + (size_t)node * MANI_MATCH_NODE_SIZE + 4);
    return s->continuing || lowest > s->best;
}

// The entry of a leaf's frame, decoded once for a run of its frames.
static const Entry* Use(Search* s, uint32_t entry)
{
    if (entry != s->lastEntry)
    {
        s->lastEntry = entry;
        s->last = EntryAt(s->d, entry);
        s->lastCandidate = Candidate(s, &s->last) && (s->last.flags & MANI_MATCH_NO_JUMP) == 0;
        s->lastStop = s->lastCandidate ? Stop(s, &s->last) : 0;
    }
    return &s->last;
}

static void ScanLeaf(Search* s, uint32_t first, uint32_t count)
{
    uint32_t n = s->featureCount;
    for (uint32_t p = first; p < first + count; ++p)
    {
        const uint8_t* at = s->d->positions + (size_t)p * MANI_MATCH_POSITION_SIZE;
        uint32_t frame = maniLoadU32(at);
        uint32_t entry = maniLoadU32(at + 4);
        const Entry* e = Use(s, entry);
        if (!s->lastCandidate || frame >= s->lastStop || Excluded(s, entry, e, frame))
        {
            continue;
        }
        float cost = Cost(s, s->d->features + 4 * (size_t)p * n, s->q->transitionCost + e->bias);
        // A tie wins over a lower best only when there is a best.
        bool better = cost < s->bestCost || (cost == s->bestCost && !s->continuing &&
                                             s->best != MANI_MATCH_NO_FRAME && frame < s->best);
        if (better)
        {
            s->best = frame;
            s->bestCost = cost;
            s->continuing = false;
        }
    }
}

// Pending nodes: the walk down holds at most one a level of the tree
// (MANI_MATCH_WALK_DEPTH) and the two of the node it opens.
typedef struct Pending
{
    uint32_t node[MANI_MATCH_WALK_DEPTH + 2];
    uint32_t first[MANI_MATCH_WALK_DEPTH + 2];
    uint32_t count[MANI_MATCH_WALK_DEPTH + 2];
    float bound[MANI_MATCH_WALK_DEPTH + 2];
    uint32_t top;
} Pending;

static void Push(Pending* p, uint32_t node, uint32_t first, uint32_t count, float bound)
{
    MANI_ASSERT(p->top < MANI_MATCH_WALK_DEPTH + 2);
    p->node[p->top] = node;
    p->first[p->top] = first;
    p->count[p->top] = count;
    p->bound[p->top] = bound;
    ++p->top;
}

// The tree from the root, nearest half first.
static void Walk(Search* s)
{
    const maniMotionDatabase* d = s->d;
    Pending p;
    p.top = 0;
    Push(&p, 0, 0, d->frameCount, Bound(s, 0));
    while (p.top > 0)
    {
        --p.top;
        uint32_t node = p.node[p.top];
        uint32_t first = p.first[p.top];
        uint32_t count = p.count[p.top];
        if (Pruned(s, node, p.bound[p.top]))
        {
            continue;
        }
        uint32_t second = maniLoadU32(d->nodes + (size_t)node * MANI_MATCH_NODE_SIZE);
        if (second == 0)
        {
            ScanLeaf(s, first, count);
            continue;
        }
        uint32_t half = count / 2;
        float a = Bound(s, node + 1);
        float b = Bound(s, second);
        // The farther pushed first, so the nearer is opened next.
        if (b < a)
        {
            Push(&p, node + 1, first, half, a);
            Push(&p, second, first + half, count - half, b);
        }
        else
        {
            Push(&p, second, first + half, count - half, b);
            Push(&p, node + 1, first, half, a);
        }
    }
}

static bool QueryValid(const maniMotionDatabase* d, const maniMatchQuery* q)
{
    const float seconds[5] = {q->continuingBias, q->transitionCost, q->jumpThreshold,
                              q->endExclusion, q->historyTime};
    for (uint32_t k = 0; k < 5; ++k)
    {
        if (!(fabsf(seconds[k]) < HUGE_VALF) || (k >= 2 && seconds[k] < 0.0f))
        {
            return false;
        }
    }
    if (q->features == NULL || (q->current != MANI_MATCH_NO_FRAME && q->current >= d->frameCount))
    {
        return false;
    }
    for (uint32_t k = 0; k < q->historyCount; ++k)
    {
        if (q->history[k] >= d->frameCount)
        {
            return false;
        }
    }
    for (uint32_t j = 0; j < d->featureCount; ++j)
    {
        if (!(fabsf(q->features[j]) < HUGE_VALF))
        {
            return false;
        }
    }
    return true;
}

void maniMatchFacings(const maniMotionDatabase* database, float* features)
{
    uint32_t n = database->featureCount;
    for (uint32_t c = 0; c < database->channelCount; ++c)
    {
        maniMatchChannel ch;
        if (maniMotionDatabase_GetChannel(database, c, &ch) != mani_success ||
            ch.kind != maniMatchBoneFacing)
        {
            continue;
        }
        for (uint32_t t = 0; t < ch.timeCount; ++t)
        {
            uint32_t at = ch.firstFeature + 3 * t;
            float offset[3];
            float scale[3];
            float raw[3];
            for (uint32_t i = 0; i < 3; ++i)
            {
                offset[i] = LoadF32(database->normalization + 4 * (size_t)(at + i));
                scale[i] = LoadF32(database->normalization + 4 * (size_t)(n + at + i));
                raw[i] = features[at + i] * scale[i] + offset[i];
            }
            float length = sqrtf((raw[0] * raw[0] + raw[1] * raw[1]) + raw[2] * raw[2]);
            // Shorter, the directions were near opposite and the blend's
            // direction is the rounding's.
            for (uint32_t i = 0; i < 3 && length > 1e-3f; ++i)
            {
                features[at + i] = (raw[i] / length - offset[i]) / scale[i];
            }
        }
    }
}

static MANI_NOINLINE maniResult SearchMotionDatabaseWork(const maniMotionDatabase* database,
                                                         const maniMatchQuery* query,
                                                         maniMatchResult* resultOut)
{
    return maniMatchSearchFrom(database, query, false, NULL, resultOut);
}

maniResult maniMotionDatabase_Search(const maniMotionDatabase* database,
                                     const maniMatchQuery* query, maniMatchResult* resultOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = SearchMotionDatabaseWork(database, query, resultOut);
    maniFloatEnvLeave(env);
    return result;
}

maniResult maniMatchSearch(const maniMotionDatabase* database, const maniMatchQuery* query,
                           bool normalized, maniMatchResult* resultOut)
{
    return maniMatchSearchFrom(database, query, normalized, NULL, resultOut);
}

maniResult maniMatchSearchFrom(const maniMotionDatabase* database, const maniMatchQuery* query,
                               bool normalized, const float* continuing, maniMatchResult* resultOut)
{
    if (database == NULL || query == NULL || resultOut == NULL)
    {
        return mani_errorInvalid;
    }
    if (query->historyCount > MANI_MAX_MATCH_HISTORY)
    {
        return mani_errorLimit;
    }
    if (!QueryValid(database, query))
    {
        return mani_errorInvalid;
    }
    // Set field by field: an initializer would clear the whole query
    // array, which the search reads only to the feature count (held
    // here, with the picks' count, so every loop runs to the count its
    // array was filled to).
    Search s;
    s.d = database;
    s.q = query;
    s.best = MANI_MATCH_NO_FRAME;
    s.bestCost = HUGE_VALF;
    s.continuing = false;
    uint32_t n = database->featureCount;
    s.featureCount = n;
    s.historyCount = query->historyCount;
    // No entry decoded yet: the first leaf frame decodes its own.
    s.lastEntry = UINT32_MAX;
    s.last = (Entry){0, 0.0f, 0.0f, 0.0f, 0, 0, 0};
    s.lastStop = 0;
    s.lastCandidate = false;
    s.jumpFrames = FramesApart(database, query->jumpThreshold);
    s.historyFrames = FramesApart(database, query->historyTime);
    for (uint32_t j = 0; j < n; ++j)
    {
        float offset = LoadF32(database->normalization + 4 * (size_t)j);
        float scale = LoadF32(database->normalization + 4 * (size_t)(n + j));
        s.query[j] = normalized ? query->features[j] : (query->features[j] - offset) / scale;
    }
    for (uint32_t k = 0; k < s.historyCount; ++k)
    {
        s.historyEntries[k] = EntryOf(database, query->history[k]);
    }
    if (query->current != MANI_MATCH_NO_FRAME)
    {
        Entry e = EntryAt(database, EntryOf(database, query->current));
        if (Candidate(&s, &e))
        {
            size_t position = maniLoadU32(database->places + 4 * (size_t)query->current);
            const uint8_t* f = continuing != NULL ? (const uint8_t*)continuing
                                                  : database->features + 4 * position * n;
            s.bestCost = Cost(&s, f, query->continuingBias + e.bias);
            s.best = query->current;
            s.continuing = true;
        }
    }
    Walk(&s);
    // A pick is a frame of the database; no pick keeps the infinite cost.
    MANI_ASSERT(s.best == MANI_MATCH_NO_FRAME ? s.bestCost == HUGE_VALF
                                              : s.best < database->frameCount);
    *resultOut = (maniMatchResult){s.best, s.bestCost};
    return mani_success;
}
