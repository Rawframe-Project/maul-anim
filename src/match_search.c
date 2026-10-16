// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The exact search (mani-0024), Holden's: the query normalized as the
// features were; the continuing frame's cost the first best; then each
// candidate entry's frames in order, a large run skipped when its box is
// already no nearer than the best, then a small run, then each frame,
// every sum stopped once it reaches the best. A frame's cost is a
// constant (its entry's bias plus the transition cost, or for the
// continuing frame the continuing bias) plus its squared distances in
// feature order. Scanning one entry at a time, a box's bound starts from
// that entry's constant and adds its squared distances to the box in the
// same order: each term is at most any of its frames' and binary32
// rounding is monotone, so a box never claims less than one of the
// entry's frames in it would cost, and the search finds the plain scan's
// frame and cost bit for bit. Ties go to the continuing frame, then to
// the lowest frame.

#include "bytes.h"
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

// The entry holding a frame: the last whose first frame is at most it.
static uint32_t EntryOf(const maniMotionDatabase* d, uint32_t frame)
{
    uint32_t low = 0;
    uint32_t high = d->entryCount - 1;
    while (low < high)
    {
        uint32_t mid = low + (high - low + 1) / 2;
        uint32_t first = maniLoadU32(d->entries + (size_t)mid * MANI_MATCH_ENTRY_SIZE + 24);
        low = first <= frame ? mid : low;
        high = first <= frame ? high : mid - 1;
    }
    return low;
}

// What a search holds: the database, the query, the normalized query,
// the earlier picks' entries, the best so far.
typedef struct Search
{
    const maniMotionDatabase* d;
    const maniMatchQuery* q;
    float query[MANI_MATCH_MAX_FEATURES];
    uint32_t historyEntries[MANI_MATCH_MAX_HISTORY];
    uint32_t best;
    float bestCost;
} Search;

static bool Candidate(const Search* s, const Entry* e)
{
    return (e->tags & s->q->requiredTags) == s->q->requiredTags &&
           (e->tags & s->q->excludedTags) == 0;
}

// Frames a and b of one entry apart, in seconds: around a looping one's
// span the shorter way.
static double Apart(const maniMotionDatabase* d, const Entry* e, uint32_t a, uint32_t b)
{
    uint32_t frames = a > b ? a - b : b - a;
    bool loop = (e->flags & MANI_MATCH_LOOP) != 0;
    frames = loop && e->count - frames < frames ? e->count - frames : frames;
    return (double)frames / (double)d->sampleRate;
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
        Apart(s->d, e, frame, current) < (double)q->jumpThreshold)
    {
        return true;
    }
    for (uint32_t k = 0; k < q->historyCount; ++k)
    {
        if (s->historyEntries[k] == entry &&
            Apart(s->d, e, frame, q->history[k]) < (double)q->historyTime)
        {
            return true;
        }
    }
    return false;
}

// The frames of an entry a search reaches: a clamped entry's frames
// within the end exclusion of its end left out.
static uint32_t Stop(const Search* s, const Entry* e)
{
    uint32_t stop = e->first + e->count;
    if ((e->flags & MANI_MATCH_LOOP) != 0)
    {
        return stop;
    }
    double rate = (double)s->d->sampleRate;
    while (stop > e->first &&
           (double)e->end - ((double)e->start + (double)(stop - 1 - e->first) / rate) <
               (double)s->q->endExclusion)
    {
        --stop;
    }
    return stop;
}

static float Frame(const Search* s, uint32_t frame, float constant)
{
    uint32_t n = s->d->featureCount;
    const uint8_t* f = s->d->features + 4 * (size_t)frame * n;
    float cost = constant;
    for (uint32_t j = 0; j < n && cost < s->bestCost; ++j)
    {
        float d = s->query[j] - LoadF32(f + 4 * (size_t)j);
        cost += d * d;
    }
    return cost;
}

// Run r's box of runs of size frames: its bound from the constant.
static float Box(const Search* s, uint32_t size, uint32_t r, float constant)
{
    const maniMotionDatabase* d = s->d;
    uint32_t n = d->featureCount;
    uint32_t runs = maniMatchRuns(d->frameCount, size);
    uint64_t first = size == MANI_MATCH_SMALL_RUN
                         ? 0
                         : 2 * (uint64_t)maniMatchRuns(d->frameCount, MANI_MATCH_SMALL_RUN) * n;
    const uint8_t* low = d->bounds + 4 * (first + (uint64_t)r * n);
    const uint8_t* high = low + 4 * (uint64_t)runs * n;
    float cost = constant;
    for (uint32_t j = 0; j < n && cost < s->bestCost; ++j)
    {
        float q = s->query[j];
        float a = LoadF32(low + 4 * (size_t)j);
        float b = LoadF32(high + 4 * (size_t)j);
        float near = q < a ? a : (q > b ? b : q);
        float gap = q - near;
        cost += gap * gap;
    }
    return cost;
}

// One large run's frames from i to end: small runs pruned, frames
// scored.
static void ScanRun(Search* s, uint32_t entry, const Entry* e, uint32_t i, uint32_t end,
                    float constant)
{
    while (i < end)
    {
        uint32_t small = i / MANI_MATCH_SMALL_RUN;
        uint32_t smallEnd = (small + 1) * MANI_MATCH_SMALL_RUN;
        smallEnd = smallEnd < end ? smallEnd : end;
        if (Box(s, MANI_MATCH_SMALL_RUN, small, constant) >= s->bestCost)
        {
            i = smallEnd;
            continue;
        }
        for (; i < smallEnd; ++i)
        {
            float cost = Excluded(s, entry, e, i) ? HUGE_VALF : Frame(s, i, constant);
            s->best = cost < s->bestCost ? i : s->best;
            s->bestCost = cost < s->bestCost ? cost : s->bestCost;
        }
    }
}

static void Scan(Search* s, uint32_t entry, const Entry* e)
{
    float constant = s->q->transitionCost + e->bias;
    uint32_t stop = Stop(s, e);
    for (uint32_t i = e->first; i < stop;)
    {
        uint32_t large = i / MANI_MATCH_LARGE_RUN;
        uint32_t end = (large + 1) * MANI_MATCH_LARGE_RUN;
        end = end < stop ? end : stop;
        if (Box(s, MANI_MATCH_LARGE_RUN, large, constant) < s->bestCost)
        {
            ScanRun(s, entry, e, i, end, constant);
        }
        i = end;
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
    if (q->features == NULL || (q->current != MANI_MATCH_NO_FRAME && q->current >= d->frameCount) ||
        q->historyCount > MANI_MATCH_MAX_HISTORY)
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

maniResult maniSearchMotionDatabase(const maniMotionDatabase* database, const maniMatchQuery* query,
                                    maniMatchResult* resultOut)
{
    if (database == NULL || query == NULL || resultOut == NULL || !QueryValid(database, query))
    {
        return mani_errorInvalid;
    }
    Search s = {.d = database, .q = query, .best = MANI_MATCH_NO_FRAME, .bestCost = HUGE_VALF};
    uint32_t n = database->featureCount;
    for (uint32_t j = 0; j < n; ++j)
    {
        float offset = LoadF32(database->normalization + 4 * (size_t)j);
        float scale = LoadF32(database->normalization + 4 * (size_t)(n + j));
        s.query[j] = (query->features[j] - offset) / scale;
    }
    for (uint32_t k = 0; k < query->historyCount; ++k)
    {
        s.historyEntries[k] = EntryOf(database, query->history[k]);
    }
    if (query->current != MANI_MATCH_NO_FRAME)
    {
        Entry e = EntryAt(database, EntryOf(database, query->current));
        if (Candidate(&s, &e))
        {
            s.bestCost = Frame(&s, query->current, query->continuingBias + e.bias);
            s.best = query->current;
        }
    }
    for (uint32_t i = 0; i < database->entryCount; ++i)
    {
        Entry e = EntryAt(database, i);
        if (Candidate(&s, &e) && (e.flags & MANI_MATCH_NO_JUMP) == 0)
        {
            Scan(&s, i, &e);
        }
    }
    *resultOut = (maniMatchResult){s.best, s.bestCost};
    return mani_success;
}
