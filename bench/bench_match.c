// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion matching's search, the library's, against the prototype of an
// exact k-d tree it was chosen by, on the same databases and queries.
// Until format 3 of the motion database the library pruned by bounds over
// runs of 16 and 64 frames, and this program compared them; the library
// now searches its own tree at the leaf size given, so the columns
// compare the two trees (docs/internals/motion-matching.md keeps the
// bounds' counts). The
// databases are synthetic locomotion: an eight-bone rig whose clips of a
// minute walk, run, turn and stand by a random walk of speed and turn
// rate, the legs swinging at a gait phase that follows the speed, the
// hips carrying root motion. Each size (about 5,000, 20,000, 100,000 and
// 300,000 frames at 30 a second) is built with three channel sets: 15
// features (the feet's positions, the hips' velocity, the path ahead),
// Holden's 27 (also the feet's velocities and the facing ahead) and 45
// (also the shins' positions and the path and facing behind).
//
// Queries come in three shapes, a thousand each: steady (a frame's bone
// features, its path nudged, that frame continuing), sudden (a frame's
// bone features, another frame's path, the first continuing) and cold
// (another frame's features nudged, nothing continuing). The tree
// prototype here returns the library's frame and cost bit for bit, which
// the program checks for every query. For each, it prints both searches'
// user instructions a query (Linux, from the kernel's counter) and
// nanoseconds a query (the best of three passes), and each index's
// bytes; before each database's rows, the user instructions of opening
// it, which checks every byte. --leaf sets both trees' leaf size (16 by default); --frames
// limits the sizes to those up to a count.

#ifdef __linux__
#define _GNU_SOURCE
#endif
#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "maul-anim/build.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef __linux__
#include <linux/perf_event.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#define RATE    30.0f
#define MINUTE  60.0f
#define SAMPLES 1801u
#define QUERIES 1000u
#define NONE    UINT32_MAX

enum
{
    boneHips,
    boneSpine,
    boneThighL,
    boneShinL,
    boneFootL,
    boneThighR,
    boneShinR,
    boneFootR,
    BONES
};

static uint64_t s_seed;

static uint64_t Next(void)
{
    uint64_t z = (s_seed += 0x9E3779B97F4A7C15u);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9u;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBu;
    return z ^ (z >> 31);
}

static double Uniform(void)
{
    return (double)(Next() >> 11) * (1.0 / 9007199254740992.0);
}

static double Gauss(void)
{
    double u = Uniform();
    double v = Uniform();
    return sqrt(-2.0 * log(u + 1e-300)) * cos(6.283185307179586 * v);
}

static void Stop(const char* what)
{
    fprintf(stderr, "bench_match: %s failed\n", what);
    exit(1);
}

// size bytes aligned to 64, kept for the program's life.
static void* Take(size_t size)
{
    unsigned char* block = malloc(size + 64);
    if (block == NULL)
    {
        Stop("an allocation");
    }
    return block + (64 - (uintptr_t)block % 64);
}

static void* Keep(maniBuilder* builder, maniBuildResult built, const char* what, size_t* size)
{
    if (built.result != mani_success)
    {
        fprintf(stderr, "bench_match: %s refused: result %d, reason %u, item %u\n", what,
                (int)built.result, (unsigned)built.reason, (unsigned)built.item);
        exit(1);
    }
    (void)maniCopyBuilt(builder, NULL, 0, size);
    void* file = Take(*size);
    if (maniCopyBuilt(builder, file, *size, NULL) != mani_success)
    {
        Stop(what);
    }
    return file;
}

static maniSkeleton s_skeleton;

static void BuildRig(maniBuilder* builder)
{
    static const char* const kNames[BONES] = {"hips",  "spine",  "thighL", "shinL",
                                              "footL", "thighR", "shinR",  "footR"};
    static const int32_t kParents[BONES] = {-1, 0, 0, 2, 3, 0, 5, 6};
    static const float kOffsets[BONES][3] = {{0.0f, 1.0f, 0.0f},     {0.0f, 0.2f, 0.0f},
                                             {0.1f, -0.05f, 0.0f},   {0.0f, -0.45f, 0.01f},
                                             {0.0f, -0.42f, -0.01f}, {-0.1f, -0.05f, 0.0f},
                                             {0.0f, -0.45f, 0.01f},  {0.0f, -0.42f, -0.01f}};
    maniBoneSource bones[BONES];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        bones[b] = (maniBoneSource){kNames[b],
                                    (uint32_t)strlen(kNames[b]),
                                    kParents[b],
                                    {{kOffsets[b][0], kOffsets[b][1], kOffsets[b][2]},
                                     {0.0f, 0.0f, 0.0f, 1.0f},
                                     {1.0f, 1.0f, 1.0f}}};
    }
    maniSkeletonSource source = {bones, BONES};
    size_t size = 0;
    void* file = Keep(builder, maniBuildSkeleton(builder, &source), "the rig", &size);
    if (maniOpenSkeleton(file, size, &s_skeleton).result != mani_success)
    {
        Stop("the rig");
    }
}

// A turn of a about x then b about y.
static void Turn(double a, double b, float* out)
{
    double s1 = sin(a / 2.0);
    double c1 = cos(a / 2.0);
    double s2 = sin(b / 2.0);
    double c2 = cos(b / 2.0);
    out[0] = (float)(s1 * c2);
    out[1] = (float)(c1 * s2);
    out[2] = (float)(s1 * s2);
    out[3] = (float)(c1 * c2);
}

// Clip k: a minute of a random walk of speed (a sixth of the time
// standing, else 0.5 to 3 m/s) and turn rate (half the time straight),
// new targets every 1 to 3 s approached smoothly.
static void BuildWalk(maniBuilder* builder, uint32_t k, float* keys, maniClip* clip)
{
    static float times[SAMPLES];
    const double dt = 1.0 / (double)RATE;
    const double tau = 6.283185307179586;
    float* walk = keys + (size_t)BONES * SAMPLES * 4;
    s_seed = 0x5EED0000u + k;
    double speed = 0.0;
    double turn = 0.0;
    double heading = Uniform() * tau;
    double x = 0.0;
    double z = 0.0;
    double phase = 0.0;
    double targetSpeed = 0.0;
    double targetTurn = 0.0;
    double next = 0.0;
    for (uint32_t s = 0; s < SAMPLES; ++s)
    {
        double t = (double)s * dt;
        times[s] = (float)t;
        if (t >= next)
        {
            targetSpeed = Uniform() < 1.0 / 6.0 ? 0.0 : 0.5 + 2.5 * Uniform();
            targetTurn = Uniform() < 0.5 ? 0.0 : 2.4 * Uniform() - 1.2;
            next = t + 1.0 + 2.0 * Uniform();
        }
        speed += (targetSpeed - speed) * 2.0 * dt;
        turn += (targetTurn - turn) * 3.0 * dt;
        heading += turn * dt;
        x += speed * sin(heading) * dt;
        z += speed * cos(heading) * dt;
        phase += tau * (0.6 + 0.45 * speed) * dt;
        double a = speed < 1.5 ? speed / 1.5 : 1.0;
        float* q = keys + (size_t)s * 4;
        size_t bone = (size_t)SAMPLES * 4;
        Turn(0.05 * a * sin(phase), heading, q + boneHips * bone);
        Turn(0.1 * a + 0.02 * sin(0.7 * t + (double)k), 0.0, q + boneSpine * bone);
        Turn(0.5 * a * sin(phase), 0.0, q + boneThighL * bone);
        Turn(-0.6 * a * fmax(0.0, sin(phase + 0.6)), 0.0, q + boneShinL * bone);
        Turn(0.2 * a * sin(phase + 1.2), 0.0, q + boneFootL * bone);
        Turn(0.5 * a * sin(phase + tau / 2.0), 0.0, q + boneThighR * bone);
        Turn(-0.6 * a * fmax(0.0, sin(phase + tau / 2.0 + 0.6)), 0.0, q + boneShinR * bone);
        Turn(0.2 * a * sin(phase + tau / 2.0 + 1.2), 0.0, q + boneFootR * bone);
        walk[3 * s] = (float)x;
        walk[3 * s + 1] = (float)(1.0 + 0.03 * a * sin(2.0 * phase) - 0.02 * a);
        walk[3 * s + 2] = (float)z;
    }
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    maniBoneTrackSource tracks[BONES];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        tracks[b] = (maniBoneTrackSource){
            b,
            kNone,
            {times, keys + (size_t)b * SAMPLES * 4, SAMPLES, maniInterpolationLinear},
            kNone};
    }
    tracks[boneHips].translation =
        (maniChannelSource){times, walk, SAMPLES, maniInterpolationLinear};
    const maniRootMotionSource root = {
        boneHips, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
        1,        maniRootReferenceRest,         maniRootReferenceRest,
        0};
    maniClipSource source = {.skeleton = &s_skeleton,
                             .boneTracks = tracks,
                             .boneTrackCount = BONES,
                             .loop = maniLoopClamp,
                             .sampleRate = RATE,
                             .duration = MINUTE,
                             .rootMotion = &root};
    size_t size = 0;
    void* file = Keep(builder, maniBuildClip(builder, &source), "a clip", &size);
    if (maniOpenClip(file, size, clip).result != mani_success)
    {
        Stop("a clip");
    }
}

// The channel sets: 15, 27 and 45 features.
static uint32_t Channels(uint32_t set, maniMatchChannelSource* out)
{
    static const float kNow[1] = {0.0f};
    static const float kAhead[3] = {0.33f, 0.67f, 1.0f};
    static const float kAround[6] = {-1.0f, -0.67f, -0.33f, 0.33f, 0.67f, 1.0f};
    uint32_t c = 0;
    out[c++] = (maniMatchChannelSource){maniMatchBonePosition, boneFootL, 0, 1, kNow, NULL, 0, 1};
    out[c++] = (maniMatchChannelSource){maniMatchBonePosition, boneFootR, 0, 1, kNow, NULL, 0, 1};
    out[c++] = (maniMatchChannelSource){maniMatchBoneVelocity, boneHips, 0, 1, kNow, NULL, 0, 1};
    if (set == 0)
    {
        out[c++] =
            (maniMatchChannelSource){maniMatchTrajectoryPosition, 0, 0, 3, kAhead, NULL, 0, 1};
        return c;
    }
    out[c++] = (maniMatchChannelSource){maniMatchBoneVelocity, boneFootL, 0, 1, kNow, NULL, 0, 1};
    out[c++] = (maniMatchChannelSource){maniMatchBoneVelocity, boneFootR, 0, 1, kNow, NULL, 0, 1};
    const float* times = set == 1 ? kAhead : kAround;
    uint32_t count = set == 1 ? 3u : 6u;
    out[c++] =
        (maniMatchChannelSource){maniMatchTrajectoryPosition, 0, 0, count, times, NULL, 0, 1};
    out[c++] = (maniMatchChannelSource){maniMatchTrajectoryFacing, 0, 0, count, times, NULL, 0, 1};
    if (set == 2)
    {
        out[c++] =
            (maniMatchChannelSource){maniMatchBonePosition, boneShinL, 0, 1, kNow, NULL, 0, 1};
        out[c++] =
            (maniMatchChannelSource){maniMatchBonePosition, boneShinR, 0, 1, kNow, NULL, 0, 1};
    }
    return c;
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// The tree: nodes over ranges of frames in tree order, each with its
// box (n lows, n highs), its lowest frame and its entries' least bias;
// leaves of at most leaf frames, split at the median of the widest
// feature.
typedef struct Node
{
    uint32_t first;
    uint32_t count;
    uint32_t left;
    uint32_t right;
    uint32_t minFrame;
    float minBias;
} Node;

typedef struct Tree
{
    uint32_t n;
    uint32_t frames;
    uint32_t leaf;
    uint32_t nodeCount;
    Node* nodes;
    float* boxes;
    uint32_t* order;
    float* features;
    uint32_t* entryOf;
    maniMatchEntry* entries;
    uint32_t entryCount;
    uint32_t* stops;
} Tree;

static const float* s_sortFeatures;
static uint32_t s_sortDim;
static uint32_t s_sortN;

static int ByFeature(const void* a, const void* b)
{
    uint32_t fa = *(const uint32_t*)a;
    uint32_t fb = *(const uint32_t*)b;
    float va = s_sortFeatures[(size_t)fa * s_sortN + s_sortDim];
    float vb = s_sortFeatures[(size_t)fb * s_sortN + s_sortDim];
    return va < vb ? -1 : va > vb ? 1 : fa < fb ? -1 : fa > fb ? 1 : 0;
}

static uint32_t BuildNode(Tree* t, const float* all, uint32_t first, uint32_t count)
{
    uint32_t id = t->nodeCount++;
    uint32_t n = t->n;
    float* low = t->boxes + (size_t)id * 2 * n;
    float* high = low + n;
    Node node = {first, count, NONE, NONE, NONE, HUGE_VALF};
    for (uint32_t j = 0; j < n; ++j)
    {
        low[j] = HUGE_VALF;
        high[j] = -HUGE_VALF;
    }
    for (uint32_t i = first; i < first + count; ++i)
    {
        uint32_t frame = t->order[i];
        node.minFrame = frame < node.minFrame ? frame : node.minFrame;
        float bias = t->entries[t->entryOf[frame]].costBias;
        node.minBias = bias < node.minBias ? bias : node.minBias;
        for (uint32_t j = 0; j < n; ++j)
        {
            float v = all[(size_t)frame * n + j];
            low[j] = v < low[j] ? v : low[j];
            high[j] = v > high[j] ? v : high[j];
        }
    }
    if (count > t->leaf)
    {
        uint32_t widest = 0;
        for (uint32_t j = 1; j < n; ++j)
        {
            widest = high[j] - low[j] > high[widest] - low[widest] ? j : widest;
        }
        s_sortFeatures = all;
        s_sortDim = widest;
        s_sortN = n;
        qsort(t->order + first, count, sizeof(uint32_t), ByFeature);
        uint32_t half = count / 2;
        node.left = BuildNode(t, all, first, half);
        node.right = BuildNode(t, all, first + half, count - half);
    }
    t->nodes[id] = node;
    return id;
}

static void BuildTree(Tree* t, const maniMotionDatabase* d, uint32_t leaf)
{
    uint32_t n = d->featureCount;
    uint32_t frames = d->frameCount;
    *t = (Tree){.n = n, .frames = frames, .leaf = leaf, .entryCount = d->entryCount};
    size_t most = 4 * (size_t)frames / leaf + 8;
    t->nodes = Take(most * sizeof(Node));
    t->boxes = Take(most * 2 * n * sizeof(float));
    t->order = Take((size_t)frames * sizeof(uint32_t));
    t->features = Take((size_t)frames * n * sizeof(float));
    t->entryOf = Take((size_t)frames * sizeof(uint32_t));
    t->entries = Take((size_t)d->entryCount * sizeof(maniMatchEntry));
    t->stops = Take((size_t)d->entryCount * sizeof(uint32_t));
    float* all = malloc((size_t)frames * n * sizeof(float));
    if (all == NULL)
    {
        Stop("the tree's features");
    }
    for (uint32_t e = 0; e < d->entryCount; ++e)
    {
        if (maniMotionDatabaseEntry(d, e, &t->entries[e]) != mani_success)
        {
            Stop("an entry");
        }
        for (uint32_t k = 0; k < t->entries[e].frameCount; ++k)
        {
            t->entryOf[t->entries[e].firstFrame + k] = e;
        }
    }
    for (uint32_t f = 0; f < frames; ++f)
    {
        t->order[f] = f;
        if (maniMotionDatabaseFeatures(d, f, all + (size_t)f * n, d->featureCount, NULL) !=
            mani_success)
        {
            Stop("the features");
        }
    }
    BuildNode(t, all, 0, frames);
    for (uint32_t i = 0; i < frames; ++i)
    {
        memcpy(t->features + (size_t)i * n, all + (size_t)t->order[i] * n, n * sizeof(float));
    }
    free(all);
}

// The bytes a file would add: each node's box and six words, and a frame
// index and an entry index a frame.
static size_t TreeBytes(const Tree* t)
{
    return (size_t)t->nodeCount * (2 * (size_t)t->n * 4 + 24) + (size_t)t->frames * 8;
}

// A search through the tree, as maniSearchMotionDatabase's semantics:
// the query checked and normalized, the continuing frame scored first,
// then the nodes nearest first, pruned by their boxes' bounds.
typedef struct TreeSearch
{
    const Tree* t;
    const maniMotionDatabase* d;
    const maniMatchQuery* q;
    float query[MANI_MAX_MATCH_FEATURES];
    uint32_t historyEntries[MANI_MAX_MATCH_HISTORY];
    uint32_t best;
    float bestCost;
    bool continuing;
    uint32_t jumpFrames;
    uint32_t historyFrames;
} TreeSearch;

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
    if (q->features == NULL || (q->current != NONE && q->current >= d->frameCount) ||
        q->historyCount > MANI_MAX_MATCH_HISTORY)
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

static uint32_t Apart(const maniMatchEntry* e, uint32_t a, uint32_t b)
{
    uint32_t frames = a > b ? a - b : b - a;
    bool loop = (e->flags & MANI_MATCH_LOOP) != 0;
    return loop && e->frameCount - frames < frames ? e->frameCount - frames : frames;
}

static bool NearEnd(const TreeSearch* s, const maniMatchEntry* e, uint32_t m)
{
    double t = (double)e->start + (double)m / (double)s->d->sampleRate;
    return (double)e->end - t < (double)s->q->endExclusion;
}

// An entry's first frame past what the search reaches, found once a
// search.
static uint32_t StopOf(const TreeSearch* s, uint32_t entry)
{
    uint32_t* stop = &s->t->stops[entry];
    if (*stop != NONE)
    {
        return *stop;
    }
    const maniMatchEntry* e = &s->t->entries[entry];
    if ((e->flags & MANI_MATCH_LOOP) != 0)
    {
        *stop = e->firstFrame + e->frameCount;
        return *stop;
    }
    double guess =
        ((double)e->end - (double)s->q->endExclusion - (double)e->start) * (double)s->d->sampleRate;
    uint32_t m = guess <= 0.0                     ? 0
                 : guess >= (double)e->frameCount ? e->frameCount
                                                  : (uint32_t)guess;
    while (m > 0 && NearEnd(s, e, m - 1))
    {
        --m;
    }
    while (m < e->frameCount && !NearEnd(s, e, m))
    {
        ++m;
    }
    *stop = e->firstFrame + m;
    return *stop;
}

static bool Excluded(const TreeSearch* s, uint32_t entry, const maniMatchEntry* e, uint32_t frame)
{
    uint32_t current = s->q->current;
    if (current != NONE && frame == current)
    {
        return true;
    }
    if (current != NONE && current >= e->firstFrame && current - e->firstFrame < e->frameCount &&
        Apart(e, frame, current) < s->jumpFrames)
    {
        return true;
    }
    for (uint32_t k = 0; k < s->q->historyCount; ++k)
    {
        if (s->historyEntries[k] == entry && Apart(e, frame, s->q->history[k]) < s->historyFrames)
        {
            return true;
        }
    }
    return false;
}

// Features' cost from constant, stopped once past the best: a sum equal
// to the best may still win a tie.
static float Cost(const TreeSearch* s, const float* f, float constant)
{
    float best = s->bestCost;
    float cost = constant;
    for (uint32_t j = 0; j < s->t->n; ++j)
    {
        float d = s->query[j] - f[j];
        cost += d * d;
        if (cost > best)
        {
            break;
        }
    }
    return cost;
}

static float Bound(const TreeSearch* s, uint32_t node)
{
    uint32_t n = s->t->n;
    const float* low = s->t->boxes + (size_t)node * 2 * n;
    const float* high = low + n;
    float best = s->bestCost;
    float cost = s->q->transitionCost + s->t->nodes[node].minBias;
    for (uint32_t j = 0; j < n; ++j)
    {
        float q = s->query[j];
        float a = low[j];
        float b = high[j];
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

// Whether a node of that bound can hold no better frame: past the best,
// or equal to it with no lower frame to win the tie.
static bool Pruned(const TreeSearch* s, uint32_t node, float bound)
{
    return bound > s->bestCost ||
           (bound == s->bestCost && (s->continuing || s->t->nodes[node].minFrame > s->best));
}

static void ScanLeaf(TreeSearch* s, const Node* node)
{
    const Tree* t = s->t;
    for (uint32_t i = node->first; i < node->first + node->count; ++i)
    {
        uint32_t frame = t->order[i];
        uint32_t entry = t->entryOf[frame];
        const maniMatchEntry* e = &t->entries[entry];
        if ((e->tags & s->q->requiredTags) != s->q->requiredTags ||
            (e->tags & s->q->excludedTags) != 0 || (e->flags & MANI_MATCH_NO_JUMP) != 0 ||
            frame >= StopOf(s, entry) || Excluded(s, entry, e, frame))
        {
            continue;
        }
        float cost = Cost(s, t->features + (size_t)i * t->n, s->q->transitionCost + e->costBias);
        if (cost < s->bestCost || (cost == s->bestCost && !s->continuing && frame < s->best))
        {
            s->best = frame;
            s->bestCost = cost;
            s->continuing = false;
        }
    }
}

static uint32_t EntryHolding(const Tree* t, uint32_t frame)
{
    return t->entryOf[frame];
}

static maniResult SearchTree(const Tree* t, const maniMotionDatabase* d, const maniMatchQuery* q,
                             maniMatchResult* out)
{
    if (!QueryValid(d, q))
    {
        return mani_errorInvalid;
    }
    TreeSearch s;
    s.t = t;
    s.d = d;
    s.q = q;
    s.best = NONE;
    s.bestCost = HUGE_VALF;
    s.continuing = false;
    s.jumpFrames = FramesApart(d, q->jumpThreshold);
    s.historyFrames = FramesApart(d, q->historyTime);
    uint32_t n = t->n;
    for (uint32_t j = 0; j < n; ++j)
    {
        float offset = LoadF32(d->normalization + 4 * (size_t)j);
        float scale = LoadF32(d->normalization + 4 * (size_t)(n + j));
        s.query[j] = (q->features[j] - offset) / scale;
    }
    for (uint32_t k = 0; k < q->historyCount; ++k)
    {
        s.historyEntries[k] = EntryHolding(t, q->history[k]);
    }
    memset(t->stops, 0xFF, (size_t)t->entryCount * sizeof(uint32_t));
    if (q->current != NONE)
    {
        const maniMatchEntry* e = &t->entries[EntryHolding(t, q->current)];
        if ((e->tags & q->requiredTags) == q->requiredTags && (e->tags & q->excludedTags) == 0)
        {
            float continuing[MANI_MAX_MATCH_FEATURES];
            (void)maniMotionDatabaseFeatures(d, q->current, continuing, d->featureCount, NULL);
            s.bestCost = Cost(&s, continuing, q->continuingBias + e->costBias);
            s.best = q->current;
            s.continuing = true;
        }
    }
    uint32_t stack[128];
    float bounds[128];
    uint32_t top = 0;
    stack[top] = 0;
    bounds[top++] = Bound(&s, 0);
    while (top > 0)
    {
        --top;
        uint32_t id = stack[top];
        if (Pruned(&s, id, bounds[top]))
        {
            continue;
        }
        const Node* node = &t->nodes[id];
        if (node->left == NONE)
        {
            ScanLeaf(&s, node);
            continue;
        }
        float left = Bound(&s, node->left);
        float right = Bound(&s, node->right);
        uint32_t near = left <= right ? node->left : node->right;
        uint32_t far = left <= right ? node->right : node->left;
        float nearBound = left <= right ? left : right;
        float farBound = left <= right ? right : left;
        if (!Pruned(&s, far, farBound))
        {
            stack[top] = far;
            bounds[top++] = farBound;
        }
        if (!Pruned(&s, near, nearBound))
        {
            stack[top] = near;
            bounds[top++] = nearBound;
        }
    }
    *out = (maniMatchResult){s.best, s.bestCost};
    return mani_success;
}

#ifdef __linux__
static int s_counter = -1;

static void OpenCounter(void)
{
    struct perf_event_attr attr;
    memset(&attr, 0, sizeof attr);
    attr.type = PERF_TYPE_HARDWARE;
    attr.size = sizeof attr;
    attr.config = PERF_COUNT_HW_INSTRUCTIONS;
    attr.disabled = 1;
    attr.exclude_kernel = 1;
    attr.exclude_hv = 1;
    s_counter = (int)syscall(SYS_perf_event_open, &attr, 0, -1, -1, 0);
}

static void StartCounter(void)
{
    if (s_counter >= 0)
    {
        ioctl(s_counter, PERF_EVENT_IOC_RESET, 0);
        ioctl(s_counter, PERF_EVENT_IOC_ENABLE, 0);
    }
}

// The instructions since StartCounter, or 0 without a counter.
static uint64_t StopCounter(void)
{
    uint64_t count = 0;
    if (s_counter >= 0)
    {
        ioctl(s_counter, PERF_EVENT_IOC_DISABLE, 0);
        if (read(s_counter, &count, sizeof count) != (ssize_t)sizeof count)
        {
            count = 0;
        }
    }
    return count;
}
#else
static void OpenCounter(void)
{
}
static void StartCounter(void)
{
}
static uint64_t StopCounter(void)
{
    return 0;
}
#endif

static double Seconds(void)
{
    struct timespec now;
    timespec_get(&now, TIME_UTC);
    return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

typedef struct Measure
{
    double instructions;
    double ns;
} Measure;

static maniMatchResult s_results[2][QUERIES];

// Both searches over the queries: instructions and the best of three
// passes' time a query; results kept to compare.
static Measure Run(int method, const Tree* t, const maniMotionDatabase* d,
                   const maniMatchQuery* queries)
{
    Measure m = {0.0, HUGE_VAL};
    for (int pass = 0; pass < 4; ++pass)
    {
        StartCounter();
        double start = Seconds();
        for (uint32_t i = 0; i < QUERIES; ++i)
        {
            maniResult r = method == 0 ? maniSearchMotionDatabase(d, &queries[i], &s_results[0][i])
                                       : SearchTree(t, d, &queries[i], &s_results[1][i]);
            if (r != mani_success)
            {
                Stop("a search");
            }
        }
        double took = Seconds() - start;
        uint64_t counted = StopCounter();
        // The first pass warms the caches; its count is the same.
        if (pass > 0)
        {
            m.ns = fmin(m.ns, took * 1e9 / QUERIES);
            m.instructions = (double)counted / QUERIES;
        }
    }
    return m;
}

static const char* const kShapes[3] = {"steady", "sudden", "cold"};

// Queries of a shape: raw features from normalized ones nudged.
static void MakeQueries(const maniMotionDatabase* d, uint32_t shape, float* features,
                        maniMatchQuery* queries)
{
    uint32_t n = d->featureCount;
    bool* path = calloc(n, sizeof(bool));
    float* other = malloc(n * sizeof(float));
    float* offsets = malloc(n * sizeof(float));
    float* scales = malloc(n * sizeof(float));
    if (path == NULL || other == NULL || offsets == NULL || scales == NULL ||
        maniMotionDatabaseNormalization(d, offsets, scales, d->featureCount, NULL) != mani_success)
    {
        Stop("the queries");
    }
    for (uint32_t c = 0; c < d->channelCount; ++c)
    {
        maniMatchChannel ch;
        if (maniMotionDatabaseChannel(d, c, &ch) != mani_success)
        {
            Stop("a channel");
        }
        bool trajectory =
            ch.kind == maniMatchTrajectoryPosition || ch.kind == maniMatchTrajectoryFacing;
        for (uint32_t j = 0; j < ch.featureCount && trajectory; ++j)
        {
            path[ch.firstFeature + j] = true;
        }
    }
    s_seed = 0xC0FFEEu + shape;
    for (uint32_t i = 0; i < QUERIES; ++i)
    {
        uint32_t f = (uint32_t)(Uniform() * (double)d->frameCount);
        uint32_t g = (uint32_t)(Uniform() * (double)d->frameCount);
        float* q = features + (size_t)i * n;
        if (maniMotionDatabaseFeatures(d, shape == 2 ? g : f, q, d->featureCount, NULL) !=
                mani_success ||
            maniMotionDatabaseFeatures(d, g, other, d->featureCount, NULL) != mani_success)
        {
            Stop("the features");
        }
        for (uint32_t j = 0; j < n; ++j)
        {
            if (shape == 1 && path[j])
            {
                q[j] = other[j];
            }
            if (shape == 2 || path[j])
            {
                q[j] += (float)(0.25 * Gauss());
            }
            q[j] = q[j] * scales[j] + offsets[j];
        }
        queries[i] = (maniMatchQuery){.features = q,
                                      .current = shape == 2 ? NONE : f,
                                      .continuingBias = 0.0f,
                                      .transitionCost = 0.0f,
                                      .jumpThreshold = 0.2f,
                                      .endExclusion = 0.2f,
                                      .historyTime = 0.0f};
    }
    free(path);
    free(other);
    free(offsets);
    free(scales);
}

int main(int argc, char** argv)
{
    uint32_t leaf = 16;
    uint32_t most = UINT32_MAX;
    for (int a = 1; a + 1 < argc; a += 2)
    {
        unsigned long v = strtoul(argv[a + 1], NULL, 10);
        if (strcmp(argv[a], "--leaf") == 0 && v >= 2 && v <= 1024)
        {
            leaf = (uint32_t)v;
        }
        else if (strcmp(argv[a], "--frames") == 0 && v > 0)
        {
            most = v > UINT32_MAX ? UINT32_MAX : (uint32_t)v;
        }
        else
        {
            fprintf(stderr, "usage: %s [--leaf <frames>] [--frames <most>]\n", argv[0]);
            return 2;
        }
    }
    OpenCounter();
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        Stop("the builder");
    }
    BuildRig(builder);
    static const uint32_t kClips[4] = {3, 11, 56, 167};
    static maniClip clips[167];
    const maniClip* listed[167];
    float* keys = malloc(((size_t)BONES * 4 + 3) * SAMPLES * sizeof(float));
    if (keys == NULL)
    {
        Stop("the keys");
    }
    for (uint32_t k = 0; k < 167; ++k)
    {
        BuildWalk(builder, k, keys, &clips[k]);
        listed[k] = &clips[k];
    }
    free(keys);
    printf("# leaf %u; instructions and ns a query, %u queries a shape\n", (unsigned)leaf,
           (unsigned)QUERIES);
    printf("%-8s %-8s %-7s %13s %13s %7s %10s %10s %10s %10s %s\n", "frames", "features", "shape",
           "library instr", "proto instr", "ratio", "library ns", "proto ns", "lib KiB",
           "proto KiB", "picks");
    maniMatchEntrySource entries[167];
    for (uint32_t k = 0; k < 167; ++k)
    {
        entries[k] = (maniMatchEntrySource){k, 0, 0.0f, MINUTE, 0.0f, 1u};
    }
    for (uint32_t size = 0; size < 4; ++size)
    {
        for (uint32_t set = 0; set < 3; ++set)
        {
            maniMatchChannelSource channels[16];
            uint32_t channelCount = Channels(set, channels);
            const maniMotionDatabaseSource source = {
                &s_skeleton,  listed, channels,       entries,        kClips[size], channelCount,
                kClips[size], RATE,   maniMatchAxisY, maniMatchAxisZ, NULL,         leaf};
            size_t bytes = 0;
            void* file =
                Keep(builder, maniBuildMotionDatabase(builder, &source), "a database", &bytes);
            maniMotionDatabase d;
            if (maniOpenMotionDatabase(file, bytes, &d).result != mani_success)
            {
                Stop("a database");
            }
            if (d.frameCount > most)
            {
                continue;
            }
            // Opening it again, which checks every byte.
            StartCounter();
            maniFileResult reopened = maniOpenMotionDatabase(file, bytes, &d);
            uint64_t opening = StopCounter();
            if (reopened.result != mani_success)
            {
                Stop("a database");
            }
            printf("# open: %u frames, %u features, %zu bytes: %llu instructions\n",
                   (unsigned)d.frameCount, (unsigned)d.featureCount, bytes,
                   (unsigned long long)opening);
            Tree tree;
            BuildTree(&tree, &d, leaf);
            // The library's index: each node's box and record, each
            // frame's position and place.
            size_t libraryBytes =
                (8 * (size_t)d.featureCount + 12) * d.nodeCount + 12 * (size_t)d.frameCount;
            float* features = Take((size_t)QUERIES * d.featureCount * sizeof(float));
            static maniMatchQuery queries[QUERIES];
            for (uint32_t shape = 0; shape < 3; ++shape)
            {
                MakeQueries(&d, shape, features, queries);
                Measure library = Run(0, &tree, &d, queries);
                Measure treed = Run(1, &tree, &d, queries);
                uint32_t differ = 0;
                for (uint32_t i = 0; i < QUERIES; ++i)
                {
                    differ += s_results[0][i].frame != s_results[1][i].frame ||
                                      memcmp(&s_results[0][i].cost, &s_results[1][i].cost,
                                             sizeof(float)) != 0
                                  ? 1u
                                  : 0u;
                }
                printf("%-8u %-8u %-7s %13.0f %13.0f %7.2f %10.0f %10.0f %10.0f %10.0f %s\n",
                       (unsigned)d.frameCount, (unsigned)d.featureCount, kShapes[shape],
                       library.instructions, treed.instructions,
                       treed.instructions > 0.0 ? library.instructions / treed.instructions : 0.0,
                       library.ns, treed.ns, (double)libraryBytes / 1024.0,
                       (double)TreeBytes(&tree) / 1024.0, differ == 0 ? "same" : "DIFFER");
                if (differ != 0)
                {
                    fprintf(stderr, "bench_match: %u picks differ\n", (unsigned)differ);
                    return 1;
                }
            }
            fflush(stdout);
        }
    }
    maniDestroyBuilder(builder);
    return 0;
}
