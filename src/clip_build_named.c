// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clip's named items at build: float tracks sampled on the
// grid, discrete tracks, events and markers, each checked, tracks sorted
// by name hash (repeats refused) and events and markers by time (ties in
// source order), their names laid end to end.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "clip_build.h"
#include "clip_file.h"
#include "container.h"
#include "sort.h"
#include "text.h"

#include "maul-anim/build.h"

#include <stdckdint.h>
#include <string.h>

enum
{
    kFloat = 0,
    kDiscrete = 1,
    kEvents = 2,
    kMarkers = 3,
};

static const maniBuildReason s_lists[4] = {maniBuildReasonFloatTrack, maniBuildReasonDiscreteTrack,
                                           maniBuildReasonEvent, maniBuildReasonMarker};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static maniBuildResult Outcome(maniResult result)
{
    return (maniBuildResult){result, maniBuildReasonNone, 0, 0};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static bool NameValid(const char* name, uint32_t length)
{
    return name != NULL && length > 0 && maniValidUtf8((const uint8_t*)name, length);
}

static float Max(float a, float b)
{
    return a > b ? a : b;
}

static maniBuildResult CheckDiscreteTrack(const maniDiscreteTrackSource* d, uint32_t i)
{
    maniBuildReason list = maniBuildReasonDiscreteTrack;
    if (!NameValid(d->name, d->nameLength))
    {
        return Refuse(list | maniBuildReasonName, i, 0);
    }
    if (d->keyCount == 0 || d->times == NULL || d->values == NULL)
    {
        return Refuse(list | maniBuildReasonKeyCount, i, 0);
    }
    for (uint32_t k = 0; k < d->keyCount; ++k)
    {
        if (!Finite(d->times[k]) || d->times[k] < 0.0f || (k > 0 && d->times[k] <= d->times[k - 1]))
        {
            return Refuse(list | maniBuildReasonKeyTime, i, 0);
        }
    }
    return Outcome(mani_success);
}

static maniBuildResult CheckTimed(const maniEventSource* e, uint32_t count, maniBuildReason list,
                                  float* last)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!Finite(e[i].time) || e[i].time < 0.0f)
        {
            return Refuse(list | maniBuildReasonKeyTime, i, 0);
        }
        if (!NameValid(e[i].name, e[i].nameLength))
        {
            return Refuse(list | maniBuildReasonName, i, 0);
        }
        *last = Max(*last, e[i].time);
    }
    return Outcome(mani_success);
}

maniBuildResult maniCheckClipNamedSource(const maniClipSource* source, float* lastOut)
{
    const uint32_t counts[4] = {source->floatTrackCount, source->discreteTrackCount,
                                source->eventCount, source->markerCount};
    const void* lists[4] = {source->floatTracks, source->discreteTracks, source->events,
                            source->markers};
    for (uint32_t l = 0; l < 4; ++l)
    {
        if (counts[l] > 0 && lists[l] == NULL)
        {
            return Refuse(s_lists[l] | maniBuildReasonCount, 0, 0);
        }
    }
    float last = 0.0f;
    for (uint32_t i = 0; i < source->floatTrackCount; ++i)
    {
        const maniFloatTrackSource* f = &source->floatTracks[i];
        if (!NameValid(f->name, f->nameLength))
        {
            return Refuse(maniBuildReasonFloatTrack | maniBuildReasonName, i, 0);
        }
        if (f->keys.keyCount == 0)
        {
            return Refuse(maniBuildReasonFloatTrack | maniBuildReasonKeyCount, i, 0);
        }
        maniBuildResult r = maniCheckChannel(&f->keys, 1, i, maniBuildReasonFloatTrack);
        if (r.result != mani_success)
        {
            return r;
        }
        last = Max(last, f->keys.times[f->keys.keyCount - 1]);
    }
    for (uint32_t i = 0; i < source->discreteTrackCount; ++i)
    {
        maniBuildResult r = CheckDiscreteTrack(&source->discreteTracks[i], i);
        if (r.result != mani_success)
        {
            return r;
        }
        const maniDiscreteTrackSource* d = &source->discreteTracks[i];
        last = Max(last, d->times[d->keyCount - 1]);
    }
    maniBuildResult r = CheckTimed(source->events, source->eventCount, maniBuildReasonEvent, &last);
    if (r.result == mani_success)
    {
        r = CheckTimed(source->markers, source->markerCount, maniBuildReasonMarker, &last);
    }
    *lastOut = last;
    return r;
}

static uint64_t NameHash(const char* name, uint32_t length)
{
    return maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length);
}

// A time as a key that sorts as the time does: non-negative binary32
// orders as its bits, -0 made +0 first.
static uint64_t TimeKey(float time)
{
    float t = time + 0.0f;
    uint32_t bits = 0;
    memcpy(&bits, &t, sizeof bits);
    return bits;
}

// An item's name, by list and source index.
static const char* ItemName(const maniClipSource* source, uint32_t list, uint32_t i,
                            uint32_t* lengthOut)
{
    if (list == kFloat)
    {
        *lengthOut = source->floatTracks[i].nameLength;
        return source->floatTracks[i].name;
    }
    if (list == kDiscrete)
    {
        *lengthOut = source->discreteTracks[i].nameLength;
        return source->discreteTracks[i].name;
    }
    const maniEventSource* e = list == kEvents ? source->events : source->markers;
    *lengthOut = e[i].nameLength;
    return e[i].name;
}

// Equal hashes next to each other in sorted: a repeated name or a
// collision, reported with source indices.
static maniBuildResult CheckRepeats(const maniClipSource* source, uint32_t list,
                                    const uint64_t* hashes, const uint32_t* sorted, uint32_t n)
{
    for (uint32_t k = 1; k < n; ++k)
    {
        uint32_t a = sorted[k - 1];
        uint32_t b = sorted[k];
        if (hashes[a] != hashes[b])
        {
            continue;
        }
        uint32_t lengthA = 0;
        uint32_t lengthB = 0;
        const char* nameA = ItemName(source, list, a, &lengthA);
        const char* nameB = ItemName(source, list, b, &lengthB);
        uint32_t low = a < b ? a : b;
        uint32_t high = a < b ? b : a;
        bool same = lengthA == lengthB && memcmp(nameA, nameB, lengthA) == 0;
        return Refuse(s_lists[list] | (same ? maniBuildReasonTrack : maniBuildReasonNameHash), high,
                      low);
    }
    return Outcome(mani_success);
}

// The work arrays of one list: keys, sort order and merge scratch.
typedef struct Work
{
    uint64_t* keys;
    uint32_t* sorted;
    uint32_t* scratch;
} Work;

typedef struct Sizes
{
    uint64_t sections[7];
    uint64_t names;
} Sizes;

static void FillWork(const maniClipSource* source, uint32_t list, Work* w)
{
    const uint32_t counts[4] = {source->floatTrackCount, source->discreteTrackCount,
                                source->eventCount, source->markerCount};
    for (uint32_t i = 0; i < counts[list]; ++i)
    {
        uint32_t length = 0;
        const char* name = ItemName(source, list, i, &length);
        w->keys[i] = list < kEvents ? NameHash(name, length)
                                    : TimeKey(list == kEvents ? source->events[i].time
                                                              : source->markers[i].time);
    }
    maniSortByKey(w->keys, counts[list], w->sorted, w->scratch);
}

// Writes a name into the names and its span at entry.
static void PutName(uint8_t* entry, const char* name, uint32_t length, uint8_t* names, uint64_t* at)
{
    maniStoreU32(entry, (uint32_t)*at);
    maniStoreU32(entry + 4, length);
    maniStoreU64(entry + 8, NameHash(name, length));
    memcpy(names + *at, name, length);
    *at += length;
}

static void WriteFloat(const maniClipSource* source, const Work* w, float rate, float duration,
                       uint32_t samples, maniClipNamedLayout* l, uint64_t* nameAt)
{
    uint32_t n = source->floatTrackCount;
    uint8_t* tracks = (uint8_t*)l->data[0];
    uint8_t* values = (uint8_t*)l->data[1];
    uint8_t* names = (uint8_t*)l->data[6];
    for (uint32_t i = 0; i < n; ++i)
    {
        const maniFloatTrackSource* f = &source->floatTracks[w->sorted[i]];
        uint8_t* entry = tracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        PutName(entry, f->name, f->nameLength, names, nameAt);
        maniStoreU32(entry + 16, f->keys.interpolation == maniInterpolationStep ? 1u : 0u);
        maniStoreU32(entry + 20, 0);
        for (uint32_t k = 0; k < samples; ++k)
        {
            double v[4];
            maniEvaluateChannel(&f->keys, 1, maniClipSampleTime(k, samples, rate, duration), v);
            float value = (float)v[0];
            memcpy(values + 4 * ((size_t)k * n + i), &value, sizeof value);
        }
    }
}

static void WriteDiscrete(const maniClipSource* source, const Work* w, maniClipNamedLayout* l,
                          uint64_t* nameAt)
{
    uint8_t* tracks = (uint8_t*)l->data[2];
    uint8_t* keys = (uint8_t*)l->data[3];
    uint8_t* names = (uint8_t*)l->data[6];
    uint32_t next = 0;
    for (uint32_t i = 0; i < source->discreteTrackCount; ++i)
    {
        const maniDiscreteTrackSource* d = &source->discreteTracks[w->sorted[i]];
        uint8_t* entry = tracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        PutName(entry, d->name, d->nameLength, names, nameAt);
        maniStoreU32(entry + 16, next);
        maniStoreU32(entry + 20, d->keyCount);
        for (uint32_t k = 0; k < d->keyCount; ++k)
        {
            uint8_t* key = keys + (size_t)(next + k) * MANI_CLIP_KEY_SIZE;
            float time = d->times[k] + 0.0f;
            memcpy(key, &time, sizeof time);
            maniStoreU32(key + 4, (uint32_t)d->values[k]);
        }
        next += d->keyCount;
    }
}

static void WriteTimed(const maniEventSource* items, uint32_t count, const Work* w, bool markers,
                       uint8_t* out, uint8_t* names, uint64_t* nameAt)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        const maniEventSource* e = &items[w->sorted[i]];
        uint8_t* entry = out + (size_t)i * MANI_CLIP_EVENT_SIZE;
        float time = e->time + 0.0f;
        memcpy(entry, &time, sizeof time);
        maniStoreU32(entry + 4, markers ? 0u : (uint32_t)e->payload);
        PutName(entry + 8, e->name, e->nameLength, names, nameAt);
    }
}

// Section sizes and the total names; false when they do not fit.
static bool Measure(const maniClipSource* source, uint32_t samples, Sizes* s)
{
    uint64_t keys = 0;
    for (uint32_t i = 0; i < source->discreteTrackCount; ++i)
    {
        keys += source->discreteTracks[i].keyCount;
    }
    uint64_t names = 0;
    for (uint32_t i = 0; i < source->floatTrackCount; ++i)
    {
        names += source->floatTracks[i].nameLength;
    }
    for (uint32_t i = 0; i < source->discreteTrackCount; ++i)
    {
        names += source->discreteTracks[i].nameLength;
    }
    for (uint32_t i = 0; i < source->eventCount; ++i)
    {
        names += source->events[i].nameLength;
    }
    for (uint32_t i = 0; i < source->markerCount; ++i)
    {
        names += source->markers[i].nameLength;
    }
    uint64_t floatValues = 0;
    if (names > UINT32_MAX || keys > UINT32_MAX ||
        ckd_mul(&floatValues, (uint64_t)samples, 4 * (uint64_t)source->floatTrackCount))
    {
        return false;
    }
    s->sections[0] = (uint64_t)source->floatTrackCount * MANI_CLIP_NAMED_TRACK_SIZE;
    s->sections[1] = floatValues;
    s->sections[2] = (uint64_t)source->discreteTrackCount * MANI_CLIP_NAMED_TRACK_SIZE;
    s->sections[3] = keys * MANI_CLIP_KEY_SIZE;
    s->sections[4] = (uint64_t)source->eventCount * MANI_CLIP_EVENT_SIZE;
    s->sections[5] = (uint64_t)source->markerCount * MANI_CLIP_EVENT_SIZE;
    s->sections[6] = names;
    return true;
}

// Discrete keys, events and markers past the duration.
maniBuildResult maniCheckClipNamedDuration(const maniClipSource* source, float duration)
{
    for (uint32_t i = 0; i < source->discreteTrackCount; ++i)
    {
        const maniDiscreteTrackSource* d = &source->discreteTracks[i];
        if (d->times[d->keyCount - 1] > duration)
        {
            return Refuse(maniBuildReasonDiscreteTrack | maniBuildReasonKeyTime, i, 0);
        }
    }
    for (uint32_t i = 0; i < source->eventCount; ++i)
    {
        if (source->events[i].time > duration)
        {
            return Refuse(maniBuildReasonEvent | maniBuildReasonKeyTime, i, 0);
        }
    }
    for (uint32_t i = 0; i < source->markerCount; ++i)
    {
        if (source->markers[i].time > duration)
        {
            return Refuse(maniBuildReasonMarker | maniBuildReasonKeyTime, i, 0);
        }
    }
    return Outcome(mani_success);
}

static maniBuildResult Lay(const maniClipSource* source, float duration, float rate,
                           uint32_t samples, Work work[4], maniClipNamedLayout* layout)
{
    for (uint32_t l = kFloat; l <= kMarkers; ++l)
    {
        FillWork(source, l, &work[l]);
    }
    const uint32_t counts[4] = {source->floatTrackCount, source->discreteTrackCount,
                                source->eventCount, source->markerCount};
    for (uint32_t l = kFloat; l <= kDiscrete; ++l)
    {
        maniBuildResult r = CheckRepeats(source, l, work[l].keys, work[l].sorted, counts[l]);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    uint64_t nameAt = 0;
    WriteFloat(source, &work[kFloat], rate, duration, samples, layout, &nameAt);
    WriteDiscrete(source, &work[kDiscrete], layout, &nameAt);
    WriteTimed(source->events, source->eventCount, &work[kEvents], false, (uint8_t*)layout->data[4],
               (uint8_t*)layout->data[6], &nameAt);
    WriteTimed(source->markers, source->markerCount, &work[kMarkers], true,
               (uint8_t*)layout->data[5], (uint8_t*)layout->data[6], &nameAt);
    for (uint32_t l = 0; l < 4; ++l)
    {
        layout->counts[l] = counts[l];
    }
    return Outcome(mani_success);
}

maniBuildResult maniLayClipNamed(maniBuilder* builder, const maniClipSource* source, float duration,
                                 float rate, uint32_t sampleCount, maniClipNamedLayout* layout)
{
    *layout = (maniClipNamedLayout){0};
    Sizes sizes;
    if (!Measure(source, sampleCount, &sizes))
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    // One block for the seven sections, each at a multiple of 8, and the
    // work arrays: a key and two u32 per item.
    const uint32_t counts[4] = {source->floatTrackCount, source->discreteTrackCount,
                                source->eventCount, source->markerCount};
    uint64_t total = 0;
    uint64_t offsets[7];
    for (uint32_t i = 0; i < 7; ++i)
    {
        offsets[i] = total;
        total += (sizes.sections[i] + 7) / 8 * 8;
    }
    size_t items = (size_t)counts[0] + counts[1] + counts[2] + counts[3];
    if (total > SIZE_MAX)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    uint8_t* block = maniAllocate(&builder->allocator, (size_t)total + 8, 1, 8);
    uint64_t* keys = maniAllocate(&builder->allocator, items + 1, sizeof(uint64_t), 8);
    uint32_t* words = maniAllocate(&builder->allocator, 2 * items + 1, sizeof(uint32_t), 8);
    if (block == NULL || keys == NULL || words == NULL)
    {
        maniRelease(&builder->allocator, block, (size_t)total + 8, 1, 8);
        maniRelease(&builder->allocator, keys, items + 1, sizeof(uint64_t), 8);
        maniRelease(&builder->allocator, words, 2 * items + 1, sizeof(uint32_t), 8);
        return Outcome(mani_errorCapacity);
    }
    memset(block, 0, (size_t)total + 8);
    layout->block = block;
    layout->blockSize = total + 8;
    for (uint32_t i = 0; i < 7; ++i)
    {
        layout->data[i] = block + offsets[i];
        layout->sizes[i] = sizes.sections[i];
    }
    Work work[4];
    size_t at = 0;
    for (uint32_t l = 0; l < 4; ++l)
    {
        work[l].keys = keys + at;
        work[l].sorted = words + 2 * at;
        work[l].scratch = work[l].sorted + counts[l];
        at += counts[l];
    }
    maniBuildResult r = Lay(source, duration, rate, sampleCount, work, layout);
    maniRelease(&builder->allocator, keys, items + 1, sizeof(uint64_t), 8);
    maniRelease(&builder->allocator, words, 2 * items + 1, sizeof(uint32_t), 8);
    if (r.result != mani_success)
    {
        maniReleaseClipNamed(builder, layout);
    }
    return r;
}

void maniReleaseClipNamed(maniBuilder* builder, maniClipNamedLayout* layout)
{
    maniRelease(&builder->allocator, layout->block, (size_t)layout->blockSize, 1, 8);
    layout->block = NULL;
}
