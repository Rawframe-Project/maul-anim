// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a morph targets file (mani-0021): channels and targets
// checked, each sparse target's vertices sorted with a vertex given twice
// summed in binary64 (as ufbx adds FBX's repeated indices), a target
// touching every vertex stored densely.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "morph_file.h"
#include "sort.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"

#include <stdbool.h>
#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static uint32_t Attributes(const maniMorphTargetSource* t)
{
    return (t->positions != NULL ? maniMorphPositions : 0u) |
           (t->normals != NULL ? maniMorphNormals : 0u) |
           (t->tangents != NULL ? maniMorphTangents : 0u);
}

// A target's given values: indices below the vertex count, deltas finite;
// t its index over the channels.
static maniBuildResult CheckValues(const maniMorphTargetSource* s, uint32_t vertices, uint32_t t)
{
    if (s->indices == NULL && s->count != vertices)
    {
        return Result(mani_errorInvalid, maniBuildReasonCount, t, 0);
    }
    const float* const lists[3] = {s->positions, s->normals, s->tangents};
    for (uint32_t i = 0; i < s->count; ++i)
    {
        if (s->indices != NULL && s->indices[i] >= vertices)
        {
            return Result(mani_errorInvalid, maniBuildReasonVertex, t, i);
        }
        for (int a = 0; a < 3; ++a)
        {
            const float* d = lists[a] == NULL ? NULL : lists[a] + 3 * (size_t)i;
            if (d != NULL && !(Finite(d[0]) && Finite(d[1]) && Finite(d[2])))
            {
                return Result(mani_errorInvalid, maniBuildReasonNotFinite, t, i);
            }
        }
    }
    return kOk;
}

// The channels' names: not empty, UTF-8, unique (by hash then bytes).
static maniBuildResult CheckNames(const maniMorphSource* s, const maniAllocator* a)
{
    uint32_t n = s->channelCount;
    for (uint32_t c = 0; c < n; ++c)
    {
        const maniMorphChannelSource* ch = &s->channels[c];
        if (ch->name == NULL || ch->nameLength == 0 || ch->nameLength > INT32_MAX ||
            !maniValidUtf8((const uint8_t*)ch->name, ch->nameLength))
        {
            return Result(mani_errorInvalid, maniBuildReasonName, c, 0);
        }
    }
    uint64_t* keys = maniAllocate(a, n, sizeof(uint64_t), 8);
    uint32_t* order = maniAllocate(a, n, sizeof(uint32_t), 8);
    uint32_t* scratch = maniAllocate(a, n, sizeof(uint32_t), 8);
    maniBuildResult r = kOk;
    if (keys == NULL || order == NULL || scratch == NULL)
    {
        r = Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    for (uint32_t c = 0; c < n && r.result == mani_success; ++c)
    {
        keys[c] =
            maniHash64(MANI_HASH_INIT, s->channels[c].name, (int32_t)s->channels[c].nameLength);
    }
    if (r.result == mani_success)
    {
        maniSortByKey(keys, n, order, scratch);
    }
    // Within a run of equal hashes (sorted stably, so in channel order),
    // the first later channel repeating an earlier one's name.
    for (uint32_t i = 0; i < n && r.result == mani_success; ++i)
    {
        for (uint32_t j = i + 1; j < n && keys[order[j]] == keys[order[i]]; ++j)
        {
            const maniMorphChannelSource* x = &s->channels[order[i]];
            const maniMorphChannelSource* y = &s->channels[order[j]];
            if (x->nameLength == y->nameLength && memcmp(x->name, y->name, x->nameLength) == 0)
            {
                r = Result(mani_errorInvalid, maniBuildReasonDuplicateName, order[j], order[i]);
                break;
            }
        }
    }
    maniRelease(a, keys, n, sizeof(uint64_t), 8);
    maniRelease(a, order, n, sizeof(uint32_t), 8);
    maniRelease(a, scratch, n, sizeof(uint32_t), 8);
    return r;
}

// Every channel and target; the target count out.
static maniBuildResult Check(const maniMorphSource* s, uint32_t* targetCountOut)
{
    if (s->channelCount == 0 || s->vertexCount == 0)
    {
        return Result(mani_errorInvalid, maniBuildReasonCount, 0, 0);
    }
    uint32_t t = 0;
    for (uint32_t c = 0; c < s->channelCount; ++c)
    {
        const maniMorphChannelSource* ch = &s->channels[c];
        if (ch->targetCount == 0 || ch->targets == NULL)
        {
            return Result(mani_errorInvalid, maniBuildReasonCount, c, 0);
        }
        float previous = 0.0f;
        for (uint32_t k = 0; k < ch->targetCount; ++k, ++t)
        {
            if (t == UINT32_MAX)
            {
                return Result(mani_errorInvalid, maniBuildReasonCount, c, 0);
            }
            float full = ch->targets[k].fullWeight;
            if (!Finite(full) || full == 0.0f || (k > 0 && !(full > previous)))
            {
                return Result(mani_errorInvalid, maniBuildReasonWeight, t, 0);
            }
            previous = full;
            maniBuildResult r = CheckValues(&ch->targets[k], s->vertexCount, t);
            if (r.result != mani_success)
            {
                return r;
            }
        }
    }
    *targetCountOut = t;
    return kOk;
}

// One target merged: its vertices in increasing order, a repeated one
// summed in binary64, and its deltas attribute by attribute.
typedef struct Merged
{
    uint32_t count;
    bool dense;
    uint32_t* indices;
    float* deltas;
} Merged;

typedef struct Work
{
    const maniMorphSource* source;
    const maniAllocator* allocator;
    uint32_t targetCount;
    // The largest given count, for the sort's buffers.
    uint32_t widest;
    // The targets over the channels in order.
    const maniMorphTargetSource** sources;
    uint64_t* keys;
    uint32_t* order;
    uint32_t* scratch;
    double* sums;
    Merged* merged;
} Work;

// count times k elements of size bytes, in a size_t; false past it.
static bool Fits(uint64_t count, uint64_t k, size_t size)
{
    uint64_t bytes = 0;
    return !ckd_mul(&bytes, count, k) && !ckd_mul(&bytes, bytes, (uint64_t)size) &&
           bytes < SIZE_MAX;
}

// One attribute's deltas of the source's i-th given vertex.
static const float* Given(const maniMorphTargetSource* s, uint32_t attribute, uint32_t i)
{
    const float* list = attribute == maniMorphPositions
                            ? s->positions
                            : (attribute == maniMorphNormals ? s->normals : s->tangents);
    return list + 3 * (size_t)i;
}

// The merged vertices of a target: the sorted runs of equal indices.
static uint32_t Runs(Work* w, const maniMorphTargetSource* s, uint32_t* indicesOut)
{
    if (s->indices == NULL)
    {
        for (uint32_t i = 0; i < s->count; ++i)
        {
            w->order[i] = i;
            indicesOut[i] = i;
        }
        return s->count;
    }
    for (uint32_t i = 0; i < s->count; ++i)
    {
        w->keys[i] = s->indices[i];
    }
    maniSortByKey(w->keys, s->count, w->order, w->scratch);
    uint32_t n = 0;
    for (uint32_t i = 0; i < s->count; ++i)
    {
        uint32_t v = s->indices[w->order[i]];
        if (n == 0 || indicesOut[n - 1] != v)
        {
            indicesOut[n++] = v;
        }
    }
    return n;
}

// The vertex the i-th sorted entry gives.
static uint32_t SortedVertex(const Work* w, const maniMorphTargetSource* s, uint32_t i)
{
    return s->indices == NULL ? w->order[i] : s->indices[w->order[i]];
}

// One attribute's deltas summed by run (in the source's order, the sort
// being stable), rounded into out: three floats a merged vertex.
static void SumAttribute(Work* w, const maniMorphTargetSource* s, uint32_t attribute,
                         uint32_t merged, float* out)
{
    uint32_t run = 0;
    for (uint32_t i = 0; i < s->count; ++i)
    {
        bool fresh = i == 0 || SortedVertex(w, s, i) != SortedVertex(w, s, i - 1);
        run += fresh && i > 0 ? 1 : 0;
        const float* d = Given(s, attribute, w->order[i]);
        for (int k = 0; k < 3; ++k)
        {
            double* sum = &w->sums[3 * (size_t)run + k];
            *sum = (fresh ? 0.0 : *sum) + (double)d[k];
        }
    }
    for (size_t k = 0; k < 3 * (size_t)merged; ++k)
    {
        out[k] = (float)w->sums[k];
    }
}

static bool Merge(Work* w, uint32_t t)
{
    const maniMorphTargetSource* s = w->sources[t];
    Merged* m = &w->merged[t];
    uint32_t attributes = Attributes(s);
    if (!Fits(s->count, 9, sizeof(float)))
    {
        return false;
    }
    m->indices = maniAllocate(w->allocator, (size_t)s->count + 1, sizeof(uint32_t), 8);
    m->deltas = maniAllocate(w->allocator, (size_t)s->count * 9 + 1, sizeof(float), 8);
    if (m->indices == NULL || m->deltas == NULL)
    {
        return false;
    }
    m->count = Runs(w, s, m->indices);
    m->dense = m->count == w->source->vertexCount;
    float* out = m->deltas;
    for (uint32_t a = maniMorphPositions; a <= maniMorphTangents; a <<= 1)
    {
        if ((attributes & a) != 0)
        {
            SumAttribute(w, s, a, m->count, out);
            out += 3 * (size_t)m->count;
        }
    }
    return true;
}

static void Release(Work* w)
{
    const maniAllocator* a = w->allocator;
    size_t n = (size_t)w->widest + 1;
    maniRelease(a, w->keys, n, sizeof(uint64_t), 8);
    maniRelease(a, w->order, n, sizeof(uint32_t), 8);
    maniRelease(a, w->scratch, n, sizeof(uint32_t), 8);
    maniRelease(a, w->sums, 3 * n, sizeof(double), 8);
    for (uint32_t t = 0; w->merged != NULL && t < w->targetCount; ++t)
    {
        uint32_t given = w->sources[t]->count;
        maniRelease(a, w->merged[t].indices, (size_t)given + 1, sizeof(uint32_t), 8);
        maniRelease(a, w->merged[t].deltas, (size_t)given * 9 + 1, sizeof(float), 8);
    }
    maniRelease(a, w->merged, w->targetCount, sizeof(Merged), 8);
    maniRelease(a, (void*)w->sources, w->targetCount, sizeof(void*), 8);
}

static bool Allocate(Work* w)
{
    const maniAllocator* a = w->allocator;
    size_t n = (size_t)w->widest + 1;
    if (!Fits(n, 3, sizeof(double)))
    {
        return false;
    }
    w->keys = maniAllocate(a, n, sizeof(uint64_t), 8);
    w->order = maniAllocate(a, n, sizeof(uint32_t), 8);
    w->scratch = maniAllocate(a, n, sizeof(uint32_t), 8);
    w->sums = maniAllocate(a, 3 * n, sizeof(double), 8);
    w->merged = maniAllocate(a, w->targetCount, sizeof(Merged), 8);
    if (w->merged != NULL)
    {
        memset(w->merged, 0, (size_t)w->targetCount * sizeof(Merged));
    }
    return w->keys != NULL && w->order != NULL && w->scratch != NULL && w->sums != NULL &&
           w->merged != NULL;
}

// The sections' sizes: indices and deltas in elements, names in bytes.
typedef struct Totals
{
    uint64_t indices;
    uint64_t deltas;
    uint64_t names;
} Totals;

static Totals Sum(const Work* w)
{
    Totals totals = {0, 0, 0};
    for (uint32_t t = 0; t < w->targetCount; ++t)
    {
        const Merged* m = &w->merged[t];
        totals.indices += m->dense ? 0 : m->count;
        totals.deltas += (uint64_t)m->count * maniMorphFloatsPerVertex(Attributes(w->sources[t]));
    }
    for (uint32_t c = 0; c < w->source->channelCount; ++c)
    {
        totals.names += w->source->channels[c].nameLength;
    }
    return totals;
}

// The header, channel and target records, the indices, deltas and names
// laid into the section buffers.
static void Lay(const Work* w, uint8_t* header, uint8_t* channels, uint8_t* targets,
                uint8_t* indices, uint8_t* deltas, uint8_t* names)
{
    const maniMorphSource* s = w->source;
    maniStoreU32(header, s->vertexCount);
    maniStoreU32(header + 4, s->channelCount);
    maniStoreU32(header + 8, w->targetCount);
    maniStoreU32(header + 12, 0);
    uint32_t first = 0;
    uint32_t name = 0;
    for (uint32_t c = 0; c < s->channelCount; ++c)
    {
        uint8_t* e = channels + (size_t)c * MANI_MORPH_CHANNEL_SIZE;
        maniStoreU32(e, first);
        maniStoreU32(e + 4, s->channels[c].targetCount);
        maniStoreU32(e + 8, name);
        maniStoreU32(e + 12, s->channels[c].nameLength);
        memcpy(names + name, s->channels[c].name, s->channels[c].nameLength);
        first += s->channels[c].targetCount;
        name += s->channels[c].nameLength;
    }
    uint64_t index = 0;
    uint64_t delta = 0;
    for (uint32_t t = 0; t < w->targetCount; ++t)
    {
        const maniMorphTargetSource* src = w->sources[t];
        const Merged* m = &w->merged[t];
        uint32_t attributes = Attributes(src) | (m->dense ? maniMorphDense : 0u);
        uint8_t* e = targets + (size_t)t * MANI_MORPH_TARGET_SIZE;
        memcpy(e, &src->fullWeight, sizeof(float));
        maniStoreU32(e + 4, attributes);
        maniStoreU32(e + 8, m->count);
        maniStoreU32(e + 12, 0);
        maniStoreU64(e + 16, index);
        maniStoreU64(e + 24, delta);
        uint64_t floats = (uint64_t)m->count * maniMorphFloatsPerVertex(attributes);
        if (!m->dense)
        {
            for (uint32_t i = 0; i < m->count; ++i)
            {
                maniStoreU32(indices + 4 * (index + i), m->indices[i]);
            }
            index += m->count;
        }
        memcpy(deltas + 4 * delta, m->deltas, 4 * floats);
        delta += floats;
    }
}

static maniBuildResult Write(maniBuilder* builder, const Work* w)
{
    Totals totals = Sum(w);
    const maniMorphSource* s = w->source;
    const uint64_t sizes[MANI_MORPH_SECTIONS] = {MANI_MORPH_HEADER_SIZE,
                                                 (uint64_t)s->channelCount *
                                                     MANI_MORPH_CHANNEL_SIZE,
                                                 (uint64_t)w->targetCount * MANI_MORPH_TARGET_SIZE,
                                                 4 * totals.indices,
                                                 4 * totals.deltas,
                                                 totals.names};
    uint64_t all = 0;
    for (uint32_t i = 0; i < MANI_MORPH_SECTIONS; ++i)
    {
        all += sizes[i];
    }
    uint8_t* staging = NULL;
    if (all <= SIZE_MAX - 1)
    {
        staging = maniAllocate(&builder->allocator, (size_t)all + 1, 1, 8);
    }
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    const uint8_t* data[MANI_MORPH_SECTIONS];
    uint8_t* at = staging;
    for (uint32_t i = 0; i < MANI_MORPH_SECTIONS; ++i)
    {
        data[i] = at;
        at += sizes[i];
    }
    Lay(w, staging, (uint8_t*)data[1], (uint8_t*)data[2], (uint8_t*)data[3], (uint8_t*)data[4],
        (uint8_t*)data[5]);
    maniSectionSource sections[MANI_MORPH_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_MORPH_SECTIONS; ++i)
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
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindMorphTargets, MANI_MORPH_FORMAT_VERSION,
                           maniMorphComputeIdentity(data, sizes), sections, count);
        maniBuilderKeep(builder, maniKindMorphTargets, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, (size_t)all + 1, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

maniBuildResult maniBuildMorphTargets(maniBuilder* builder, const maniMorphSource* source)
{
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (source == NULL || (source->channels == NULL && source->channelCount > 0))
    {
        ++builder->misuses;
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    uint32_t targetCount = 0;
    maniBuildResult r = Check(source, &targetCount);
    if (r.result == mani_success)
    {
        r = CheckNames(source, &builder->allocator);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    Work w = {.source = source, .allocator = &builder->allocator, .targetCount = targetCount};
    w.sources =
        (const maniMorphTargetSource**)maniAllocate(w.allocator, targetCount, sizeof(void*), 8);
    for (uint32_t c = 0, t = 0; w.sources != NULL && c < source->channelCount; ++c)
    {
        for (uint32_t k = 0; k < source->channels[c].targetCount; ++k, ++t)
        {
            w.sources[t] = &source->channels[c].targets[k];
            uint32_t given = w.sources[t]->count;
            w.widest = given > w.widest ? given : w.widest;
        }
    }
    bool ready = w.sources != NULL && Allocate(&w);
    for (uint32_t t = 0; ready && t < targetCount; ++t)
    {
        ready = Merge(&w, t);
    }
    r = ready ? Write(builder, &w) : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    Release(&w);
    return r;
}
