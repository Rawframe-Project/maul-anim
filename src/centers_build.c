// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Centers of rotation (Le and Hodgins 2016, Eq. 4), exactly:
// each vertex's center is the triangles' centroids weighted by area and
// by the similarity of the triangle's average weights to the vertex's,
// a sum over joint pairs j < k of w_vj w_vk w_tj w_tk
// exp(-(w_vj w_tk - w_vk w_tj)^2 / sigma^2). A term needs both to carry j
// and k, so triangles are indexed by the pairs they carry and each vertex
// visits only its own pairs' triangles. Binary64, the exponential the
// library's own (binary32: the similarity weighs, it need not be exact),
// sums in a fixed order: the same bits on every platform.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "elementary_float.h"
#include "influence_file.h"
#include "sort.h"

#include "maul-anim/build.h"
#include "maul-anim/skinning.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

#define MAX_PER    (MANI_INFLUENCE_MAX_GROUPS * MANI_INFLUENCE_GROUP)
#define MAX_MERGED (3 * MAX_PER)

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item)
{
    return (maniBuildResult){result, reason, item, 0};
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// A vertex's weighted influences in slot order (the heaviest first).
static uint32_t Weighted(const maniInfluences* v, uint32_t vertex, uint32_t* joints,
                         double* weights)
{
    uint32_t per = v->groupCount * MANI_INFLUENCE_GROUP;
    size_t base = (size_t)vertex * per;
    uint32_t n = 0;
    for (uint32_t slot = 0; slot < per; ++slot)
    {
        float w = LoadF32(v->weights + 4 * (base + slot));
        if (w == 0.0f)
        {
            break;
        }
        joints[n] = maniLoadU16(v->joints + 2 * (base + slot));
        weights[n] = (double)w;
        ++n;
    }
    return n;
}

// A triangle's average weights: its vertices' influences merged by joint,
// in order of first appearance, divided by 3.
static uint32_t Average(const maniInfluences* v, const uint32_t corners[3], uint32_t* joints,
                        double* weights)
{
    uint32_t n = 0;
    for (int c = 0; c < 3; ++c)
    {
        uint32_t j[MAX_PER];
        double w[MAX_PER];
        uint32_t m = Weighted(v, corners[c], j, w);
        for (uint32_t i = 0; i < m; ++i)
        {
            uint32_t at = 0;
            while (at < n && joints[at] != j[i])
            {
                ++at;
            }
            if (at == n)
            {
                joints[n] = j[i];
                weights[n++] = 0.0;
            }
            weights[at] += w[i];
        }
    }
    for (uint32_t i = 0; i < n; ++i)
    {
        weights[i] /= 3.0;
    }
    return n;
}

// The key of an unordered joint pair: the lower joint above.
static uint64_t Key(uint32_t a, uint32_t b)
{
    return a < b ? ((uint64_t)a << 16) | b : ((uint64_t)b << 16) | a;
}

// One triangle's pair: its average weights for the pair's lower and higher
// joint.
typedef struct Entry
{
    uint32_t triangle;
    double lower;
    double higher;
} Entry;

typedef struct Work
{
    const maniCenterSource* source;
    maniAllocator* allocator;
    uint32_t count;
    uint64_t* keys;
    Entry* entries;
    uint32_t* order;
    uint32_t* scratch;
    uint64_t* sortedKeys;
    Entry* sorted;
    double* geometry;
    float* centers;
} Work;

static const float* Position(const maniCenterSource* s, uint32_t vertex)
{
    return (const float*)((const uint8_t*)s->positions + (size_t)vertex * s->stride);
}

static maniBuildResult Check(const maniCenterSource* s)
{
    if (s->stride < 12)
    {
        return Result(mani_errorInvalid, maniBuildReasonCount, 0);
    }
    if (!(s->sigma >= 0.0f && s->sigma <= FLT_MAX))
    {
        return Result(mani_errorInvalid, maniBuildReasonOption, 0);
    }
    uint32_t vertices = s->influences->vertexCount;
    for (uint32_t v = 0; v < vertices; ++v)
    {
        const float* p = Position(s, v);
        for (int k = 0; k < 3; ++k)
        {
            if (!(p[k] >= -FLT_MAX && p[k] <= FLT_MAX))
            {
                return Result(mani_errorInvalid, maniBuildReasonNotFinite, v);
            }
        }
    }
    for (uint32_t t = 0; t < s->triangleCount; ++t)
    {
        const uint32_t* c = s->triangles + 3 * (size_t)t;
        if (c[0] >= vertices || c[1] >= vertices || c[2] >= vertices)
        {
            return Result(mani_errorInvalid, maniBuildReasonTriangle, t);
        }
    }
    return Result(mani_success, maniBuildReasonNone, 0);
}

// Pairs of every triangle (count only, or written), and its area and
// centroid.
static uint64_t Pairs(Work* w, bool write)
{
    const maniCenterSource* s = w->source;
    uint64_t count = 0;
    uint32_t joints[MAX_MERGED];
    double weights[MAX_MERGED];
    for (uint32_t t = 0; t < s->triangleCount; ++t)
    {
        const uint32_t* c = s->triangles + 3 * (size_t)t;
        uint32_t n = Average(s->influences, c, joints, weights);
        for (uint32_t a = 0; a < n; ++a)
        {
            for (uint32_t b = a + 1; b < n; ++b)
            {
                if (write)
                {
                    bool lowerFirst = joints[a] < joints[b];
                    w->keys[count] = Key(joints[a], joints[b]);
                    w->entries[count] = (Entry){t, lowerFirst ? weights[a] : weights[b],
                                                lowerFirst ? weights[b] : weights[a]};
                }
                ++count;
            }
        }
        if (write)
        {
            const float* p[3] = {Position(s, c[0]), Position(s, c[1]), Position(s, c[2])};
            double e1[3];
            double e2[3];
            double* g = w->geometry + 4 * (size_t)t;
            for (int k = 0; k < 3; ++k)
            {
                e1[k] = (double)p[1][k] - (double)p[0][k];
                e2[k] = (double)p[2][k] - (double)p[0][k];
                g[1 + k] = ((double)p[0][k] + (double)p[1][k] + (double)p[2][k]) / 3.0;
            }
            double x = e1[1] * e2[2] - e1[2] * e2[1];
            double y = e1[2] * e2[0] - e1[0] * e2[2];
            double z = e1[0] * e2[1] - e1[1] * e2[0];
            g[0] = 0.5 * sqrt(x * x + y * y + z * z);
        }
    }
    return count;
}

// The first sorted entry whose key is at least key.
static uint32_t LowerBound(const Work* w, uint64_t key)
{
    uint32_t low = 0;
    uint32_t high = w->count;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (w->sortedKeys[mid] < key)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    return low;
}

// One vertex's center, or its position when no triangle is similar.
static void Center(const Work* w, uint32_t vertex, double sigma)
{
    uint32_t joints[MAX_PER];
    double weights[MAX_PER];
    uint32_t n = Weighted(w->source->influences, vertex, joints, weights);
    double sum[3] = {0.0, 0.0, 0.0};
    double total = 0.0;
    for (uint32_t a = 0; a < n; ++a)
    {
        for (uint32_t b = a + 1; b < n; ++b)
        {
            uint64_t key = Key(joints[a], joints[b]);
            bool lowerFirst = joints[a] < joints[b];
            double lower = lowerFirst ? weights[a] : weights[b];
            double higher = lowerFirst ? weights[b] : weights[a];
            for (uint32_t i = LowerBound(w, key); i < w->count && w->sortedKeys[i] == key; ++i)
            {
                const Entry* e = &w->sorted[i];
                double d = lower * e->higher - higher * e->lower;
                double similarity = lower * higher * e->lower * e->higher *
                                    (double)maniExpF((float)(-(d * d) / (sigma * sigma)));
                const double* g = w->geometry + 4 * (size_t)e->triangle;
                double weight = similarity * g[0];
                for (int k = 0; k < 3; ++k)
                {
                    sum[k] += weight * g[1 + k];
                }
                total += weight;
            }
        }
    }
    const float* p = Position(w->source, vertex);
    float* c = w->centers + 3 * (size_t)vertex;
    for (int k = 0; k < 3; ++k)
    {
        c[k] = total > 0.0 ? (float)(sum[k] / total) : p[k];
    }
}

static void Release(Work* w, size_t triangles, size_t vertices)
{
    maniAllocator* a = w->allocator;
    maniRelease(a, w->keys, w->count + 1, sizeof(uint64_t), 8);
    maniRelease(a, w->entries, w->count + 1, sizeof(Entry), 8);
    maniRelease(a, w->order, w->count + 1, sizeof(uint32_t), 8);
    maniRelease(a, w->scratch, w->count + 1, sizeof(uint32_t), 8);
    maniRelease(a, w->sortedKeys, w->count + 1, sizeof(uint64_t), 8);
    maniRelease(a, w->sorted, w->count + 1, sizeof(Entry), 8);
    maniRelease(a, w->geometry, 4 * triangles + 1, sizeof(double), 8);
    maniRelease(a, w->centers, 3 * vertices, sizeof(float), 8);
}

static bool Allocate(Work* w, size_t triangles, size_t vertices)
{
    maniAllocator* a = w->allocator;
    size_t n = (size_t)w->count + 1;
    w->keys = maniAllocate(a, n, sizeof(uint64_t), 8);
    w->entries = maniAllocate(a, n, sizeof(Entry), 8);
    w->order = maniAllocate(a, n, sizeof(uint32_t), 8);
    w->scratch = maniAllocate(a, n, sizeof(uint32_t), 8);
    w->sortedKeys = maniAllocate(a, n, sizeof(uint64_t), 8);
    w->sorted = maniAllocate(a, n, sizeof(Entry), 8);
    w->geometry = maniAllocate(a, 4 * triangles + 1, sizeof(double), 8);
    w->centers = maniAllocate(a, 3 * vertices, sizeof(float), 8);
    return w->keys != NULL && w->entries != NULL && w->order != NULL && w->scratch != NULL &&
           w->sortedKeys != NULL && w->sorted != NULL && w->geometry != NULL && w->centers != NULL;
}

// The influences again, their centers added.
static maniBuildResult Write(maniBuilder* builder, const maniInfluences* v, const float* centers)
{
    uint64_t slots = (uint64_t)v->vertexCount * v->groupCount * MANI_INFLUENCE_GROUP;
    uint8_t binding[MANI_INFLUENCE_BINDING_SIZE] = {0};
    maniStoreU64(binding, v->skinIdentity);
    maniStoreU32(binding + 8, v->skinJointCount);
    maniStoreU32(binding + 12, v->vertexCount);
    maniStoreU32(binding + 16, v->groupCount);
    static const uint8_t kNone = 0;
    const uint8_t* data[MANI_INFLUENCE_SECTIONS] = {
        binding, v->joints, v->weights,
        v->dualQuaternionWeights != NULL ? v->dualQuaternionWeights : &kNone,
        (const uint8_t*)centers};
    const uint64_t sizes[MANI_INFLUENCE_SECTIONS] = {
        sizeof binding, 2 * slots, 4 * slots,
        v->dualQuaternionWeights != NULL ? 4 * (uint64_t)v->vertexCount : 0,
        12 * (uint64_t)v->vertexCount};
    maniSectionSource sections[MANI_INFLUENCE_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_INFLUENCE_SECTIONS; ++i)
    {
        if (sizes[i] > 0)
        {
            sections[count++] = (maniSectionSource){i + 1, data[i], sizes[i]};
        }
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0);
    }
    maniWriteContainer(file, maniKindInfluences, MANI_INFLUENCE_FORMAT_VERSION,
                       maniInfluenceComputeIdentity(data, sizes), sections, count);
    maniBuilderKeep(builder, maniKindInfluences, file, (size_t)fileSize, NULL, 0);
    return Result(mani_success, maniBuildReasonNone, 0);
}

maniBuildResult maniBuildCenters(maniBuilder* builder, const maniCenterSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0);
    }
    if (source == NULL || source->influences == NULL || source->positions == NULL ||
        (source->triangles == NULL && source->triangleCount > 0))
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0);
    }
    maniBuildResult r = Check(source);
    if (r.result != mani_success)
    {
        return r;
    }
    Work w = {.source = source, .allocator = &builder->allocator};
    uint64_t pairs = Pairs(&w, false);
    size_t triangles = source->triangleCount;
    size_t vertices = source->influences->vertexCount;
    size_t check = 0;
    if (pairs >= UINT32_MAX || ckd_mul(&check, triangles, (size_t)4 * sizeof(double)))
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0);
    }
    w.count = (uint32_t)pairs;
    if (!Allocate(&w, triangles, vertices))
    {
        Release(&w, triangles, vertices);
        return Result(mani_errorCapacity, maniBuildReasonNone, 0);
    }
    (void)Pairs(&w, true);
    maniSortByKey(w.keys, w.count, w.order, w.scratch);
    for (uint32_t i = 0; i < w.count; ++i)
    {
        w.sortedKeys[i] = w.keys[w.order[i]];
        w.sorted[i] = w.entries[w.order[i]];
    }
    double sigma = source->sigma == 0.0f ? 0.1 : (double)source->sigma;
    for (uint32_t v = 0; v < vertices; ++v)
    {
        Center(&w, v, sigma);
    }
    r = Write(builder, source->influences, w.centers);
    Release(&w, triangles, vertices);
    return r;
}
