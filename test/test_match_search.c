// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The exact search: random databases written as files
// (entries looping and clamped, biases, tags, no-jump entries, frames
// repeated so costs tie; each leaf size, the tree in the builder's
// order or a random one) and random searches (continuing frames, biases,
// transition costs, tag masks, exclusions near the continuing frame, at
// clamped ends and around earlier picks) give the plain scan's frame and
// cost bit for bit; then each rule on a database small enough to name
// its frames; the refusals; and a blend between frames, bone facings made
// unit again.

#include "bytes.h"
#include "container.h"
#include "match_file.h"
#include "match_search.h"
#include "match_tree.h"
#include "test_harness.h"

#include "maul-anim/match.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint64_t s_state = 0x9E3779B97F4A7C15u;

static uint32_t Next(void)
{
    s_state ^= s_state << 13;
    s_state ^= s_state >> 7;
    s_state ^= s_state << 17;
    return (uint32_t)(s_state >> 32);
}

// Uniform in [low, high).
static float Uniform(float low, float high)
{
    return low + (high - low) * (float)(Next() >> 8) / 16777216.0f;
}

#define RATE        30.0f
#define MAX_ENTRIES 8
#define MAX_FRAMES  (MAX_ENTRIES * 120)
#define MAX_WIDTH   24

// A database as a test lays it out: per entry its flags, end, bias,
// tags; per feature its offset and scale; per frame its normalized
// features.
typedef struct Layout
{
    // The tree's leaf size, and whether its order is random rather than
    // the builder's.
    uint32_t leaf;
    bool shuffled;
    uint32_t channels;
    // Each channel's kind, a bone position unless set.
    uint32_t kinds[MAX_WIDTH / 3];
    uint32_t entryCount;
    uint32_t frames;
    uint32_t flags[MAX_ENTRIES];
    float ends[MAX_ENTRIES];
    float biases[MAX_ENTRIES];
    uint32_t tags[MAX_ENTRIES];
    uint32_t counts[MAX_ENTRIES];
    float offsets[MAX_WIDTH];
    float scales[MAX_WIDTH];
    float features[MAX_FRAMES * MAX_WIDTH];
} Layout;

static Layout s_layout;
static _Alignas(16) uint8_t s_file[1 << 19];
static _Alignas(8) uint8_t s_sections[1 << 19];

static void PutF32(uint8_t* p, float v)
{
    memcpy(p, &v, sizeof v);
}

static uint32_t s_order[MAX_FRAMES];
static uint64_t s_keys[MAX_FRAMES];
static uint32_t s_sorted[MAX_FRAMES];
static uint32_t s_scratch[MAX_FRAMES];
static uint32_t s_entryOf[MAX_FRAMES];

// The tree's sections from out (each 8-byte aligned): the features in
// its order, the boxes, the nodes, the positions and the places; at and
// ends receive where each begins and ends.
static void Tree(const Layout* l, uint8_t* out, uint8_t* at[5], uint8_t* ends[5])
{
    uint32_t n = 3 * l->channels;
    uint32_t frame = 0;
    for (uint32_t i = 0; i < l->entryCount; ++i)
    {
        for (uint32_t k = 0; k < l->counts[i]; ++k)
        {
            s_entryOf[frame++] = i;
        }
    }
    if (l->shuffled)
    {
        for (uint32_t f = 0; f < l->frames; ++f)
        {
            uint32_t k = Next() % (f + 1);
            s_order[f] = s_order[k];
            s_order[k] = f;
        }
    }
    else
    {
        maniMatchTreeOrder(l->features, n, l->frames, l->leaf, s_order, s_keys, s_sorted,
                           s_scratch);
    }
    uint32_t nodes = maniMatchNodeCount(l->frames, l->leaf);
    const size_t sizes[5] = {4 * (size_t)l->frames * n, 8 * (size_t)nodes * n,
                             (size_t)nodes * MANI_MATCH_NODE_SIZE,
                             (size_t)l->frames * MANI_MATCH_POSITION_SIZE, 4 * (size_t)l->frames};
    for (uint32_t k = 0; k < 5; ++k)
    {
        at[k] = k == 0 ? out : at[k - 1] + (sizes[k - 1] + 7) / 8 * 8;
        ends[k] = at[k] + sizes[k];
    }
    const maniMatchTreeSource source = {l->features, s_order,   s_entryOf, l->biases,
                                        n,           l->frames, l->leaf,   nodes};
    maniMatchLayTree(&source, (float*)(void*)at[0], (float*)(void*)at[1], at[2], at[3], at[4]);
}

static uint8_t* Header(const Layout* l, uint8_t* h)
{
    const uint32_t counts[7] = {
        1, l->frames, 3 * l->channels, l->channels, l->channels, l->entryCount, 1};
    maniStoreU64(h, 7);
    for (uint32_t k = 0; k < 7; ++k)
    {
        maniStoreU32(h + 8 + 4 * k, counts[k]);
    }
    PutF32(h + 36, RATE);
    maniStoreU32(h + 40, 1);
    maniStoreU32(h + 44, 2);
    maniStoreU32(h + 48, l->leaf);
    maniStoreU32(h + 52, maniMatchNodeCount(l->frames, l->leaf));
    return h + MANI_MATCH_HEADER_SIZE;
}

// The channels (each the hips' position now) and their times.
static uint8_t* Channels(const Layout* l, uint8_t* c)
{
    for (uint32_t k = 0; k < l->channels; ++k)
    {
        uint8_t* e = c + (size_t)k * MANI_MATCH_CHANNEL_SIZE;
        memset(e, 0, MANI_MATCH_CHANNEL_SIZE);
        const uint32_t words[7] = {l->kinds[k], 0, 0, 1, k, 3 * k, 3};
        for (uint32_t w = 0; w < 7; ++w)
        {
            maniStoreU32(e + 4 * w, words[w]);
        }
        PutF32(e + 28, 1.0f);
    }
    return c + (size_t)l->channels * MANI_MATCH_CHANNEL_SIZE;
}

static uint8_t* Entries(const Layout* l, uint8_t* e)
{
    uint32_t first = 0;
    for (uint32_t i = 0; i < l->entryCount; ++i)
    {
        uint8_t* at = e + (size_t)i * MANI_MATCH_ENTRY_SIZE;
        maniStoreU32(at, 0);
        maniStoreU32(at + 4, l->flags[i]);
        PutF32(at + 8, 0.0f);
        PutF32(at + 12, l->ends[i]);
        PutF32(at + 16, l->biases[i]);
        maniStoreU32(at + 20, l->tags[i]);
        maniStoreU32(at + 24, first);
        maniStoreU32(at + 28, l->counts[i]);
        first += l->counts[i];
    }
    return e + (size_t)l->entryCount * MANI_MATCH_ENTRY_SIZE;
}

// Lays out the sections and writes the file; opens it.
static void Write(const Layout* l, maniMotionDatabase* d)
{
    uint32_t n = 3 * l->channels;
    uint8_t* at[MANI_MATCH_SECTIONS + 1];
    uint8_t* ends[MANI_MATCH_SECTIONS];
    at[0] = s_sections;
    ends[0] = Header(l, at[0]);
    at[1] = ends[0];
    ends[1] = Channels(l, at[1]);
    at[2] = ends[1];
    memset(at[2], 0, 4 * (size_t)l->channels);
    ends[2] = at[2] + 4 * (size_t)l->channels;
    at[3] = ends[2];
    maniStoreU64(at[3], 11);
    ends[3] = at[3] + 8;
    at[4] = ends[3];
    ends[4] = Entries(l, at[4]);
    at[5] = ends[4];
    memcpy(at[5], l->offsets, 4 * (size_t)n);
    memcpy(at[5] + 4 * (size_t)n, l->scales, 4 * (size_t)n);
    ends[5] = at[5] + 8 * (size_t)n;
    uint8_t* tree[5];
    uint8_t* treeEnds[5];
    Tree(l, ends[5] + (8 - (uintptr_t)ends[5] % 8) % 8, tree, treeEnds);
    // The features and boxes, no names, no mirror, then the nodes,
    // positions and places.
    const uint32_t tags[5] = {6, 7, 10, 11, 12};
    for (uint32_t k = 0; k < 5; ++k)
    {
        at[tags[k]] = tree[k];
        ends[tags[k]] = treeEnds[k];
    }
    at[8] = at[9] = ends[8] = ends[9] = treeEnds[4];
    const uint8_t* data[MANI_MATCH_SECTIONS];
    uint64_t sizes[MANI_MATCH_SECTIONS];
    maniSectionSource sections[MANI_MATCH_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_MATCH_SECTIONS; ++i)
    {
        data[i] = at[i];
        sizes[i] = (uint64_t)(ends[i] - at[i]);
        if (sizes[i] > 0)
        {
            sections[count++] = (maniSectionSource){i + 1, data[i], sizes[i]};
        }
    }
    uint64_t size = 0;
    CHECK(maniContainerSize(sections, count, &size) && size <= sizeof s_file, "fits");
    maniWriteContainer(s_file, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION,
                       maniMatchComputeIdentity(data, sizes), sections, count);
    maniFileResult r = maniOpenMotionDatabase(s_file, (size_t)size, d);
    if (r.result != mani_success)
    {
        printf("  opened %d at %llu\n", (int)r.result, (unsigned long long)r.offset);
    }
    CHECK(r.result == mani_success, "a database written and opened");
}

// A random layout: entries of 1 to 120 frames, a third looping, a sixth
// never jumped to, some biased; features in [-2, 2), a fifth of the
// frames repeating an earlier one.
static void Randomize(Layout* l)
{
    l->leaf = 8u << (Next() % 3);
    l->shuffled = Next() % 2 == 0;
    l->channels = 1 + Next() % 8;
    l->entryCount = 1 + Next() % MAX_ENTRIES;
    uint32_t n = 3 * l->channels;
    l->frames = 0;
    for (uint32_t i = 0; i < l->entryCount; ++i)
    {
        bool loop = Next() % 3 == 0;
        uint32_t frames = 1 + Next() % 120;
        l->flags[i] = (loop ? MANI_MATCH_LOOP : 0u) | (Next() % 6 == 0 ? MANI_MATCH_NO_JUMP : 0u);
        l->ends[i] = (float)(loop ? frames : frames - 1) / RATE;
        l->ends[i] = loop && l->ends[i] == 0.0f ? 1.0f / RATE : l->ends[i];
        l->biases[i] = Next() % 2 == 0 ? 0.0f : Uniform(-0.5f, 0.5f);
        l->tags[i] = Next() % 16;
        l->counts[i] = maniMatchEntryFrames(0.0f, l->ends[i], RATE, loop);
        l->frames += l->counts[i];
    }
    for (uint32_t j = 0; j < n; ++j)
    {
        l->offsets[j] = Uniform(-1.0f, 1.0f);
        l->scales[j] = Uniform(0.5f, 2.0f);
    }
    for (uint32_t f = 0; f < l->frames; ++f)
    {
        uint32_t copy = f > 0 && Next() % 5 == 0 ? Next() % f : f;
        for (uint32_t j = 0; j < n; ++j)
        {
            l->features[f * n + j] = copy != f ? l->features[copy * n + j] : Uniform(-2.0f, 2.0f);
        }
    }
}

// The plain scan, from the rules as documented.
static uint32_t EntryOf(const Layout* l, uint32_t frame, uint32_t* firstOut)
{
    uint32_t first = 0;
    uint32_t i = 0;
    while (frame >= first + l->counts[i])
    {
        first += l->counts[i++];
    }
    *firstOut = first;
    return i;
}

static double Apart(const Layout* l, uint32_t e, uint32_t a, uint32_t b)
{
    uint32_t d = a > b ? a - b : b - a;
    if ((l->flags[e] & MANI_MATCH_LOOP) != 0 && l->counts[e] - d < d)
    {
        d = l->counts[e] - d;
    }
    return (double)d / (double)RATE;
}

static bool Passes(const Layout* l, const maniMatchQuery* q, uint32_t e)
{
    return (l->tags[e] & q->requiredTags) == q->requiredTags && (l->tags[e] & q->excludedTags) == 0;
}

static bool Out(const Layout* l, const maniMatchQuery* q, uint32_t f)
{
    uint32_t first = 0;
    uint32_t e = EntryOf(l, f, &first);
    if (!Passes(l, q, e) || (l->flags[e] & MANI_MATCH_NO_JUMP) != 0 || f == q->current)
    {
        return true;
    }
    if ((l->flags[e] & MANI_MATCH_LOOP) == 0 &&
        (double)l->ends[e] - (double)(f - first) / (double)RATE < (double)q->endExclusion)
    {
        return true;
    }
    uint32_t other = 0;
    if (q->current != MANI_MATCH_NO_FRAME && EntryOf(l, q->current, &other) == e &&
        Apart(l, e, f, q->current) < (double)q->jumpThreshold)
    {
        return true;
    }
    for (uint32_t k = 0; k < q->historyCount; ++k)
    {
        if (EntryOf(l, q->history[k], &other) == e &&
            Apart(l, e, f, q->history[k]) < (double)q->historyTime)
        {
            return true;
        }
    }
    return false;
}

static float Cost(const Layout* l, const maniMatchQuery* q, uint32_t f, float constant)
{
    uint32_t n = 3 * l->channels;
    float cost = constant;
    for (uint32_t j = 0; j < n; ++j)
    {
        float v = (q->features[j] - l->offsets[j]) / l->scales[j];
        float d = v - l->features[f * n + j];
        cost += d * d;
    }
    return cost;
}

static maniMatchResult Plain(const Layout* l, const maniMatchQuery* q)
{
    maniMatchResult best = {MANI_MATCH_NO_FRAME, HUGE_VALF};
    uint32_t first = 0;
    if (q->current != MANI_MATCH_NO_FRAME)
    {
        uint32_t e = EntryOf(l, q->current, &first);
        if (Passes(l, q, e))
        {
            best = (maniMatchResult){q->current,
                                     Cost(l, q, q->current, q->continuingBias + l->biases[e])};
        }
    }
    for (uint32_t f = 0; f < l->frames; ++f)
    {
        if (Out(l, q, f))
        {
            continue;
        }
        float cost = Cost(l, q, f, q->transitionCost + l->biases[EntryOf(l, f, &first)]);
        best = cost < best.cost ? (maniMatchResult){f, cost} : best;
    }
    return best;
}

static float s_query[MAX_WIDTH];

// A threshold in seconds: none, any, a whole number of frames (where
// the frames apart that are nearer change), or past every frame.
static float Seconds(void)
{
    switch (Next() % 5)
    {
    case 0:
        return 0.0f;
    case 1:
        return (float)(Next() % 16) / RATE;
    case 2:
        return 1.0e6f;
    default:
        return Uniform(0.0f, 0.5f);
    }
}

static maniMatchQuery RandomQuery(const Layout* l)
{
    uint32_t n = 3 * l->channels;
    // Half the time near a frame, so the bounds prune.
    uint32_t near = Next() % l->frames;
    for (uint32_t j = 0; j < n; ++j)
    {
        float v = Next() % 2 == 0 ? l->features[near * n + j] + Uniform(-0.1f, 0.1f)
                                  : Uniform(-2.0f, 2.0f);
        s_query[j] = v * l->scales[j] + l->offsets[j];
    }
    maniMatchQuery q = {.features = s_query,
                        .current = Next() % 3 == 0 ? MANI_MATCH_NO_FRAME : Next() % l->frames,
                        .requiredTags = Next() % 4 == 0 ? 1u << (Next() % 4) : 0u,
                        .excludedTags = Next() % 4 == 0 ? 1u << (Next() % 4) : 0u,
                        .historyCount = Next() % (MANI_MATCH_MAX_HISTORY + 1),
                        .continuingBias = Next() % 2 == 0 ? 0.0f : Uniform(-0.5f, 0.5f),
                        .transitionCost = Next() % 2 == 0 ? 0.0f : Uniform(0.0f, 0.5f),
                        .jumpThreshold = Seconds(),
                        .endExclusion = Seconds(),
                        .historyTime = Seconds()};
    for (uint32_t k = 0; k < q.historyCount; ++k)
    {
        q.history[k] = Next() % l->frames;
    }
    return q;
}

static void TestAgainstScan(void)
{
    bool same = true;
    uint32_t found = 0;
    for (int trial = 0; trial < 300; ++trial)
    {
        Randomize(&s_layout);
        maniMotionDatabase d;
        Write(&s_layout, &d);
        for (int k = 0; k < 20; ++k)
        {
            maniMatchQuery q = RandomQuery(&s_layout);
            maniMatchResult want = Plain(&s_layout, &q);
            maniMatchResult got = {0, 0.0f};
            bool ok = maniSearchMotionDatabase(&d, &q, &got) == mani_success &&
                      got.frame == want.frame && memcmp(&got.cost, &want.cost, 4) == 0;
            if (!ok && same)
            {
                printf("  trial %d: got %u %.9g, want %u %.9g\n", trial, got.frame,
                       (double)got.cost, want.frame, (double)want.cost);
            }
            same = same && ok;
            found += want.frame != MANI_MATCH_NO_FRAME ? 1u : 0u;
        }
    }
    CHECK(same, "the plain scan's frame and cost, bit for bit");
    CHECK(found > 5000, "most searches find a frame");
}

// Thresholds of whole frames, where the frames left out change: a
// query at each of an entry's last frames, near the end and near a
// continuing frame six back, against the plain scan.
static void TestBoundaries(void)
{
    bool same = true;
    uint32_t searches = 0;
    for (int trial = 0; trial < 40; ++trial)
    {
        Randomize(&s_layout);
        const Layout* l = &s_layout;
        maniMotionDatabase d;
        Write(l, &d);
        uint32_t n = 3 * l->channels;
        uint32_t first = 0;
        for (uint32_t e = 0; e < l->entryCount; first += l->counts[e++])
        {
            uint32_t end = first + l->counts[e];
            for (uint32_t f = end > first + 10 ? end - 10 : first; f < end; ++f)
            {
                for (uint32_t j = 0; j < n; ++j)
                {
                    s_query[j] = l->features[f * n + j] * l->scales[j] + l->offsets[j];
                }
                for (uint32_t k = 0; k < 13; ++k)
                {
                    maniMatchQuery q = {.features = s_query,
                                        .current = MANI_MATCH_NO_FRAME,
                                        .endExclusion = (float)k / RATE};
                    if (k % 2 == 1)
                    {
                        q = (maniMatchQuery){.features = s_query,
                                             .current = f >= first + 6 ? f - 6 : first,
                                             .continuingBias = 100.0f,
                                             .jumpThreshold = (float)k / RATE};
                    }
                    maniMatchResult want = Plain(l, &q);
                    maniMatchResult got = {0, 0.0f};
                    bool ok = maniSearchMotionDatabase(&d, &q, &got) == mani_success &&
                              got.frame == want.frame && memcmp(&got.cost, &want.cost, 4) == 0;
                    if (!ok && same)
                    {
                        printf("  trial %d frame %u k %u: got %u, want %u\n", trial, f, k,
                               got.frame, want.frame);
                    }
                    same = same && ok;
                    ++searches;
                }
            }
        }
    }
    CHECK(same, "the plain scan's frame at every whole-frame threshold");
    CHECK(searches > 1000, "searched near every entry's end");
}

// Two entries of four frames: a clamped one (frames 0 to 3, tag 1) and
// a looping one (4 to 7, tag 2); one feature, frame f at f.
static void Small(maniMotionDatabase* d, uint32_t flags1, float bias1)
{
    Layout* l = &s_layout;
    *l = (Layout){.leaf = 8, .channels = 1, .entryCount = 2, .frames = 8};
    l->flags[0] = 0;
    l->ends[0] = 3.0f / RATE;
    l->tags[0] = 1;
    l->flags[1] = MANI_MATCH_LOOP | flags1;
    l->ends[1] = 4.0f / RATE;
    l->biases[1] = bias1;
    l->tags[1] = 2;
    l->counts[0] = maniMatchEntryFrames(0.0f, l->ends[0], RATE, false);
    l->counts[1] = maniMatchEntryFrames(0.0f, l->ends[1], RATE, true);
    for (uint32_t j = 0; j < 3; ++j)
    {
        l->scales[j] = 1.0f;
    }
    for (uint32_t f = 0; f < 8; ++f)
    {
        l->features[f * 3] = (float)f;
    }
    Write(l, d);
}

static uint32_t Find(const maniMotionDatabase* d, maniMatchQuery q, float at)
{
    const float features[3] = {at, 0.0f, 0.0f};
    q.features = features;
    maniMatchResult r = {0, 0.0f};
    CHECK(maniSearchMotionDatabase(d, &q, &r) == mani_success, "searched");
    return r.frame;
}

static void TestRules(void)
{
    maniMotionDatabase d;
    Small(&d, 0, 0.0f);
    maniMatchQuery q = {.current = MANI_MATCH_NO_FRAME};
    CHECK(Find(&d, q, 5.2f) == 5 && Find(&d, q, 100.0f) == 7, "the nearest");
    q.current = 2;
    CHECK(Find(&d, q, 2.5f) == 2, "a tie: the continuing frame");
    q.continuingBias = 0.1f;
    CHECK(Find(&d, q, 2.5f) == 3, "its bias lost it");
    q.transitionCost = 0.2f;
    CHECK(Find(&d, q, 2.5f) == 2, "the transition cost kept it");
    q = (maniMatchQuery){.current = MANI_MATCH_NO_FRAME, .requiredTags = 2};
    CHECK(Find(&d, q, 1.0f) == 4, "only the looping entry has tag 2");
    q = (maniMatchQuery){.current = MANI_MATCH_NO_FRAME, .excludedTags = 2};
    CHECK(Find(&d, q, 6.0f) == 3, "the looping entry excluded");
    q = (maniMatchQuery){.current = MANI_MATCH_NO_FRAME, .endExclusion = 1.5f / RATE};
    CHECK(Find(&d, q, 3.0f) == 4, "frames 2 and 3 too near the clamped end; the loop has none");
    q = (maniMatchQuery){.current = 4, .continuingBias = 100.0f, .jumpThreshold = 1.5f / RATE};
    CHECK(Find(&d, q, 7.0f) == 6, "frames 5 and 7 (around the loop) too near 4");
    q = (maniMatchQuery){
        .current = MANI_MATCH_NO_FRAME, .historyCount = 1, .historyTime = 0.5f / RATE};
    q.history[0] = 1;
    CHECK(Find(&d, q, 1.0f) == 0, "the earlier pick excluded");
    q = (maniMatchQuery){.current = MANI_MATCH_NO_FRAME, .requiredTags = 4};
    maniMatchResult r = {0, 0.0f};
    const float zero[3] = {0};
    q.features = zero;
    CHECK(maniSearchMotionDatabase(&d, &q, &r) == mani_success && r.frame == MANI_MATCH_NO_FRAME &&
              r.cost == HUGE_VALF,
          "no candidate");
    Small(&d, MANI_MATCH_NO_JUMP, -10.0f);
    q = (maniMatchQuery){.current = MANI_MATCH_NO_FRAME};
    CHECK(Find(&d, q, 6.0f) == 3, "never jumped to");
    q.current = 6;
    CHECK(Find(&d, q, 6.0f) == 6, "but played on, with its bias");
}

static void TestRefusals(void)
{
    maniMotionDatabase d;
    Small(&d, 0, 0.0f);
    const float features[3] = {0};
    const float nan[3] = {NAN, 0.0f, 0.0f};
    maniMatchResult r;
    maniMatchQuery q = {.features = features, .current = MANI_MATCH_NO_FRAME};
    maniMatchQuery bad[8];
    for (int k = 0; k < 8; ++k)
    {
        bad[k] = q;
    }
    bad[0].features = NULL;
    bad[1].features = nan;
    bad[2].current = 8;
    bad[3].historyCount = MANI_MATCH_MAX_HISTORY + 1;
    bad[4].historyCount = 1;
    bad[4].history[0] = 8;
    bad[5].jumpThreshold = -1.0f;
    bad[6].transitionCost = HUGE_VALF;
    bad[7].continuingBias = NAN;
    bool refused = true;
    for (int k = 0; k < 8; ++k)
    {
        refused = refused && maniSearchMotionDatabase(&d, &bad[k], &r) == mani_errorInvalid;
    }
    CHECK(refused && maniSearchMotionDatabase(NULL, &q, &r) == mani_errorInvalid &&
              maniSearchMotionDatabase(&d, NULL, &r) == mani_errorInvalid &&
              maniSearchMotionDatabase(&d, &q, NULL) == mani_errorInvalid,
          "refused searches");
}

// Half way from a to b, as a motion matching node blends.
static void Blend(const maniMotionDatabase* d, const float* a, const float* b, float* out)
{
    for (uint32_t j = 0; j < 6; ++j)
    {
        out[j] = a[j] + 0.5f * (b[j] - a[j]);
    }
    maniMatchFacings(d, out);
}

// Two frames of a bone position (0 then 2 along x) and a bone facing (x
// then y, then x then -x), normalized by offsets and a scale of 2: half
// way, the position at 1 and the facing unit between x and y; opposite
// facings, of no length half way but the rounding's, left as blended.
static void TestBlend(void)
{
    Layout* l = &s_layout;
    *l = (Layout){.leaf = 8, .channels = 2, .entryCount = 1, .frames = 2};
    l->kinds[1] = maniMatchBoneFacing;
    l->ends[0] = 1.0f / RATE;
    l->counts[0] = maniMatchEntryFrames(0.0f, l->ends[0], RATE, false);
    const float offsets[6] = {0.0f, 0.0f, 0.0f, 0.1f, -0.2f, 0.3f};
    const float raw[2][6] = {{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
                             {2.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}};
    for (uint32_t j = 0; j < 6; ++j)
    {
        l->offsets[j] = offsets[j];
        l->scales[j] = j < 3 ? 1.0f : 2.0f;
        for (uint32_t f = 0; f < 2; ++f)
        {
            l->features[f * 6 + j] = (raw[f][j] - offsets[j]) / l->scales[j];
        }
    }
    maniMotionDatabase d;
    Write(l, &d);
    float out[6];
    Blend(&d, l->features, l->features + 6, out);
    float facing[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        facing[i] = out[3 + i] * 2.0f + offsets[3 + i];
    }
    double x = (double)facing[0];
    double y = (double)facing[1];
    double z = (double)facing[2];
    double length = sqrt(x * x + y * y + z * z);
    CHECK(out[0] == 1.0f && out[1] == 0.0f && fabs(length - 1.0) < 1e-6 &&
              fabs(x - sqrt(0.5)) < 1e-6 && fabs(z) < 1e-6,
          "the position blended, the facing unit half way");
    float opposite[6];
    memcpy(opposite, l->features + 6, sizeof opposite);
    opposite[3] = (-1.0f - offsets[3]) / 2.0f;
    opposite[4] = (0.0f - offsets[4]) / 2.0f;
    Blend(&d, l->features, opposite, out);
    bool blended = true;
    for (uint32_t j = 3; j < 6; ++j)
    {
        blended = blended && out[j] == l->features[j] + 0.5f * (opposite[j] - l->features[j]);
    }
    CHECK(blended, "opposite facings left as blended");
}

int main(void)
{
    TestAgainstScan();
    TestBoundaries();
    TestRules();
    TestRefusals();
    TestBlend();
    return s_failures == 0 ? 0 : 1;
}
