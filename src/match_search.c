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

#include "match_search.h"

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
// the earlier picks' entries, the best so far, per level of runs (small,
// large) where its boxes' lows start and how far past a low its high
// is, and the exclusions' times as frames apart.
typedef struct Search
{
    const maniMotionDatabase* d;
    const maniMatchQuery* q;
    float query[MANI_MATCH_MAX_FEATURES];
    uint32_t historyEntries[MANI_MATCH_MAX_HISTORY];
    uint32_t best;
    float bestCost;
    const uint8_t* lows[2];
    size_t highs[2];
    uint32_t jumpFrames;
    uint32_t historyFrames;
    uint32_t featureCount;
    uint32_t historyCount;
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

// The frames of an entry a search reaches: a clamped entry's frames
// within the end exclusion of its end left out. The first excluded
// frame is guessed from the times and then settled by NearEnd, so the
// guess's rounding never moves it.
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
// frame's a caller gives) plus constant, stopped once it reaches the best.
static float Cost(const Search* s, const uint8_t* f, float constant)
{
    uint32_t n = s->featureCount;
    float best = s->bestCost;
    float cost = constant;
    for (uint32_t j = 0; j < n; ++j)
    {
        float d = s->query[j] - LoadF32(f + 4 * (size_t)j);
        cost += d * d;
        if (cost >= best)
        {
            break;
        }
    }
    return cost;
}

static float Frame(const Search* s, uint32_t frame, float constant)
{
    return Cost(s, s->d->features + 4 * (size_t)frame * s->featureCount, constant);
}

// Run r's box at level 0 (the small runs) or 1 (the large): its bound
// from the constant.
static float Box(const Search* s, uint32_t level, uint32_t r, float constant)
{
    uint32_t n = s->featureCount;
    const uint8_t* low = s->lows[level] + 4 * (size_t)r * n;
    const uint8_t* high = low + s->highs[level];
    float best = s->bestCost;
    float cost = constant;
    for (uint32_t j = 0; j < n; ++j)
    {
        float q = s->query[j];
        float a = LoadF32(low + 4 * (size_t)j);
        float b = LoadF32(high + 4 * (size_t)j);
        float near = q < a ? a : (q > b ? b : q);
        float gap = q - near;
        cost += gap * gap;
        if (cost >= best)
        {
            break;
        }
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
        if (Box(s, 0, small, constant) >= s->bestCost)
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
        if (Box(s, 1, large, constant) < s->bestCost)
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

void maniMatchFacings(const maniMotionDatabase* database, float* features)
{
    uint32_t n = database->featureCount;
    for (uint32_t c = 0; c < database->channelCount; ++c)
    {
        maniMatchChannel ch;
        if (maniMotionDatabaseChannel(database, c, &ch) != mani_success ||
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

maniResult maniSearchMotionDatabase(const maniMotionDatabase* database, const maniMatchQuery* query,
                                    maniMatchResult* resultOut)
{
    return maniMatchSearchFrom(database, query, false, NULL, resultOut);
}

maniResult maniMatchSearch(const maniMotionDatabase* database, const maniMatchQuery* query,
                           bool normalized, maniMatchResult* resultOut)
{
    return maniMatchSearchFrom(database, query, normalized, NULL, resultOut);
}

maniResult maniMatchSearchFrom(const maniMotionDatabase* database, const maniMatchQuery* query,
                               bool normalized, const float* continuing, maniMatchResult* resultOut)
{
    if (database == NULL || query == NULL || resultOut == NULL || !QueryValid(database, query))
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
    uint32_t n = database->featureCount;
    s.featureCount = n;
    s.historyCount = query->historyCount;
    size_t smallRuns = maniMatchRuns(database->frameCount, MANI_MATCH_SMALL_RUN);
    size_t largeRuns = maniMatchRuns(database->frameCount, MANI_MATCH_LARGE_RUN);
    s.lows[0] = database->bounds;
    s.lows[1] = database->bounds + 4 * 2 * smallRuns * n;
    s.highs[0] = 4 * smallRuns * n;
    s.highs[1] = 4 * largeRuns * n;
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
            const uint8_t* f = continuing != NULL
                                   ? (const uint8_t*)continuing
                                   : database->features + 4 * (size_t)query->current * n;
            s.bestCost = Cost(&s, f, query->continuingBias + e.bias);
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
