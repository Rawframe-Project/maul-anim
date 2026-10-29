// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clip's float and discrete tracks, events and markers:
// their checks at open and their reading. Tracks are kept by increasing
// name hash, so a name is found by binary search; events and markers by
// increasing time, so a window of time is a range of indices.

#include "bytes.h"
#include "clip_file.h"
#include "clip_view.h"
#include "container.h"
#include "fpenv.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"

#include <math.h>
#include <stdckdint.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static maniFileResult Success(void)
{
    return (maniFileResult){mani_success, 0, 0};
}

static float LoadF32(const uint8_t* p)
{
    float value = 0.0f;
    memcpy(&value, p, sizeof value);
    return value;
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// A name at entry + offset: a span inside the names, not empty, UTF-8,
// with the hash at entry + offset + 8.
static maniFileResult CheckName(const uint8_t* entry, uint64_t at, uint32_t offset,
                                const uint8_t* names, uint64_t nameSize)
{
    uint64_t start = maniLoadU32(entry + offset);
    uint64_t length = maniLoadU32(entry + offset + 4);
    if (length == 0 || start > nameSize || length > nameSize - start)
    {
        return Fail(at + offset);
    }
    if (!maniValidUtf8(names + start, (size_t)length))
    {
        return Fail(at + offset);
    }
    if (maniHashBytes(MANI_HASH_INIT, names + start, length) != maniLoadU64(entry + offset + 8))
    {
        return Fail(at + offset + 8);
    }
    return Success();
}

// Float or discrete tracks: names checked and hashes strictly increasing.
static maniFileResult CheckNamedTracks(const uint8_t* tracks, uint64_t at, uint32_t count,
                                       const uint8_t* names, uint64_t nameSize)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint8_t* entry = tracks + (uint64_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        uint64_t entryAt = at + (uint64_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        maniFileResult r = CheckName(entry, entryAt, 0, names, nameSize);
        if (r.result != mani_success)
        {
            return r;
        }
        if (i > 0 && maniLoadU64(entry + 8) <= maniLoadU64(entry + 8 - MANI_CLIP_NAMED_TRACK_SIZE))
        {
            return Fail(entryAt + 8);
        }
    }
    return Success();
}

static maniFileResult CheckFloatTracks(const uint8_t* const data[], const uint64_t at[],
                                       const maniClip* clip, uint64_t nameSize)
{
    maniFileResult r = CheckNamedTracks(data[3], at[3], clip->floatTrackCount, data[9], nameSize);
    for (uint32_t i = 0; r.result == mani_success && i < clip->floatTrackCount; ++i)
    {
        const uint8_t* entry = data[3] + (uint64_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        uint64_t entryAt = at[3] + (uint64_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        if (maniLoadU32(entry + 16) > 1)
        {
            r = Fail(entryAt + 16);
        }
        else if (maniLoadU32(entry + 20) != 0)
        {
            r = Fail(entryAt + 20);
        }
    }
    // A compressed clip's float samples are in its segments.
    uint64_t values = clip->compressed ? 0 : (uint64_t)clip->sampleCount * clip->floatTrackCount;
    for (uint64_t k = 0; r.result == mani_success && k < values; ++k)
    {
        if (!Finite(LoadF32(data[4] + 4 * k)))
        {
            r = Fail(at[4] + 4 * k);
        }
    }
    return r;
}

// Discrete tracks: keys contiguous in track order, each track's times
// strictly increasing within [0, duration].
static maniFileResult CheckDiscrete(const uint8_t* const data[], const uint64_t sizes[],
                                    const uint64_t at[], uint64_t keysEntry, const maniClip* clip,
                                    uint64_t nameSize)
{
    maniFileResult r =
        CheckNamedTracks(data[5], at[5], clip->discreteTrackCount, data[9], nameSize);
    uint64_t next = 0;
    for (uint32_t i = 0; r.result == mani_success && i < clip->discreteTrackCount; ++i)
    {
        const uint8_t* entry = data[5] + (uint64_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        uint64_t entryAt = at[5] + (uint64_t)i * MANI_CLIP_NAMED_TRACK_SIZE;
        uint64_t first = maniLoadU32(entry + 16);
        uint64_t count = maniLoadU32(entry + 20);
        if (first != next)
        {
            return Fail(entryAt + 16);
        }
        if (count == 0 || count > sizes[6] / MANI_CLIP_KEY_SIZE - first)
        {
            return Fail(entryAt + 20);
        }
        for (uint64_t k = first; k < first + count; ++k)
        {
            float time = LoadF32(data[6] + k * MANI_CLIP_KEY_SIZE);
            if (!Finite(time) || time < 0.0f || time > clip->duration ||
                (k > first && time <= LoadF32(data[6] + (k - 1) * MANI_CLIP_KEY_SIZE)))
            {
                return Fail(at[6] + k * MANI_CLIP_KEY_SIZE);
            }
        }
        next = first + count;
    }
    if (r.result == mani_success && next * MANI_CLIP_KEY_SIZE != sizes[6])
    {
        return Fail(keysEntry);
    }
    return r;
}

// Events or markers: times in [0, duration] not decreasing, names
// checked, markers' payloads zero.
static maniFileResult CheckTimed(const uint8_t* entries, uint64_t at, uint32_t count,
                                 const maniClip* clip, const uint8_t* names, uint64_t nameSize,
                                 bool markers)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint8_t* entry = entries + (uint64_t)i * MANI_CLIP_EVENT_SIZE;
        uint64_t entryAt = at + (uint64_t)i * MANI_CLIP_EVENT_SIZE;
        float time = LoadF32(entry);
        if (!Finite(time) || time < 0.0f || time > clip->duration ||
            (i > 0 && time < LoadF32(entry - MANI_CLIP_EVENT_SIZE)))
        {
            return Fail(entryAt);
        }
        if (markers && maniLoadU32(entry + 4) != 0)
        {
            return Fail(entryAt + 4);
        }
        maniFileResult r = CheckName(entry, entryAt, 8, names, nameSize);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    return Success();
}

maniFileResult maniCheckClipNamed(const uint8_t* const* data, const uint64_t* sizes,
                                  const uint64_t* at, const uint64_t* entry, maniClip* clip)
{
    uint64_t floatValues = 0;
    const uint64_t expected[5] = {
        (uint64_t)clip->floatTrackCount * MANI_CLIP_NAMED_TRACK_SIZE,
        clip->compressed ? 0
        : ckd_mul(&floatValues, (uint64_t)clip->sampleCount, 4 * (uint64_t)clip->floatTrackCount)
            ? UINT64_MAX
            : floatValues,
        (uint64_t)clip->discreteTrackCount * MANI_CLIP_NAMED_TRACK_SIZE,
        (uint64_t)clip->eventCount * MANI_CLIP_EVENT_SIZE,
        (uint64_t)clip->markerCount * MANI_CLIP_EVENT_SIZE,
    };
    const uint32_t sections[5] = {3, 4, 5, 7, 8};
    for (uint32_t i = 0; i < 5; ++i)
    {
        if (sizes[sections[i]] != expected[i])
        {
            return Fail(entry[sections[i]]);
        }
    }
    maniFileResult r = CheckFloatTracks(data, at, clip, sizes[9]);
    if (r.result == mani_success)
    {
        r = CheckDiscrete(data, sizes, at, entry[6], clip, sizes[9]);
    }
    if (r.result == mani_success)
    {
        r = CheckTimed(data[7], at[7], clip->eventCount, clip, data[9], sizes[9], false);
    }
    if (r.result == mani_success)
    {
        r = CheckTimed(data[8], at[8], clip->markerCount, clip, data[9], sizes[9], true);
    }
    clip->floatTracks = data[3];
    clip->floatSamples = data[4];
    clip->discreteTracks = data[5];
    clip->discreteKeys = data[6];
    clip->events = data[7];
    clip->markers = data[8];
    clip->names = data[9];
    clip->namesSize = sizes[9];
    return r;
}

static const char* Name(const maniClip* clip, const uint8_t* entry, size_t* lengthOut)
{
    if (lengthOut != NULL)
    {
        *lengthOut = maniLoadU32(entry + 4);
    }
    return (const char*)(clip->names + maniLoadU32(entry));
}

static const char* TrackName(const maniClip* clip, const uint8_t* tracks, uint32_t count,
                             uint32_t track, size_t* lengthOut)
{
    if (track >= count)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return Name(clip, tracks + (size_t)track * MANI_CLIP_NAMED_TRACK_SIZE, lengthOut);
}

static uint32_t FindTrack(const maniClip* clip, const uint8_t* tracks, uint32_t count,
                          const char* name, size_t length)
{
    uint64_t hash = maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length);
    uint32_t low = 0;
    uint32_t high = count;
    while (low < high)
    {
        uint32_t middle = low + (high - low) / 2;
        const uint8_t* entry = tracks + (size_t)middle * MANI_CLIP_NAMED_TRACK_SIZE;
        uint64_t found = maniLoadU64(entry + 8);
        if (found == hash)
        {
            size_t foundLength = 0;
            const char* foundName = Name(clip, entry, &foundLength);
            bool same = foundLength == length && memcmp(foundName, name, length) == 0;
            return same ? middle : MANI_NO_TRACK;
        }
        if (found < hash)
        {
            low = middle + 1;
        }
        else
        {
            high = middle;
        }
    }
    return MANI_NO_TRACK;
}

uint32_t maniClip_GetFloatTrackCount(const maniClip* clip)
{
    return clip->floatTrackCount;
}

const char* maniClip_GetFloatTrackName(const maniClip* clip, uint32_t track, size_t* lengthOut)
{
    return TrackName(clip, clip->floatTracks, clip->floatTrackCount, track, lengthOut);
}

uint32_t maniClip_FindFloatTrack(const maniClip* clip, const char* name, size_t length)
{
    return FindTrack(clip, clip->floatTracks, clip->floatTrackCount, name, length);
}

uint32_t maniClip_GetDiscreteTrackCount(const maniClip* clip)
{
    return clip->discreteTrackCount;
}

const char* maniClip_GetDiscreteTrackName(const maniClip* clip, uint32_t track, size_t* lengthOut)
{
    return TrackName(clip, clip->discreteTracks, clip->discreteTrackCount, track, lengthOut);
}

uint32_t maniClip_FindDiscreteTrack(const maniClip* clip, const char* name, size_t length)
{
    return FindTrack(clip, clip->discreteTracks, clip->discreteTrackCount, name, length);
}

static MANI_NOINLINE int32_t ClipDiscreteValueWork(const maniClip* clip, uint32_t track, float time)
{
    if (track >= clip->discreteTrackCount)
    {
        return 0;
    }
    const uint8_t* entry = clip->discreteTracks + (size_t)track * MANI_CLIP_NAMED_TRACK_SIZE;
    const uint8_t* keys = clip->discreteKeys + (size_t)maniLoadU32(entry + 16) * MANI_CLIP_KEY_SIZE;
    uint32_t count = maniLoadU32(entry + 20);
    // The number of keys at or before the time; the value is the last of
    // them, or the first key when there is none, NaN included (no
    // comparison with NaN holds), which is the value at 0.
    uint32_t low = 0;
    uint32_t high = count;
    while (low < high)
    {
        uint32_t middle = low + (high - low) / 2;
        if (LoadF32(keys + (size_t)middle * MANI_CLIP_KEY_SIZE) <= time)
        {
            low = middle + 1;
        }
        else
        {
            high = middle;
        }
    }
    uint32_t key = low > 0 ? low - 1 : 0;
    return (int32_t)maniLoadU32(keys + (size_t)key * MANI_CLIP_KEY_SIZE + 4);
}

int32_t maniClip_GetDiscreteValue(const maniClip* clip, uint32_t track, float time)
{
    uint64_t env = maniFloatEnvEnter();
    int32_t result = ClipDiscreteValueWork(clip, track, time);
    maniFloatEnvLeave(env);
    return result;
}

static maniClipEvent Timed(const maniClip* clip, const uint8_t* entries, uint32_t count,
                           uint32_t index)
{
    maniClipEvent e = {0};
    if (index < count)
    {
        const uint8_t* entry = entries + (size_t)index * MANI_CLIP_EVENT_SIZE;
        e.time = LoadF32(entry);
        e.payload = (int32_t)maniLoadU32(entry + 4);
        e.nameHash = maniLoadU64(entry + 16);
        e.name = Name(clip, entry + 8, &e.nameLength);
    }
    return e;
}

uint32_t maniClip_GetEventCount(const maniClip* clip)
{
    return clip->eventCount;
}

maniClipEvent maniClip_GetEvent(const maniClip* clip, uint32_t index)
{
    return Timed(clip, clip->events, clip->eventCount, index);
}

uint32_t maniClip_GetMarkerCount(const maniClip* clip)
{
    return clip->markerCount;
}

maniClipEvent maniClip_GetMarker(const maniClip* clip, uint32_t index)
{
    return Timed(clip, clip->markers, clip->markerCount, index);
}

// The number of events with time below t (or at most t when inclusive).
static uint32_t CountBefore(const maniClip* clip, float t, bool inclusive)
{
    uint32_t low = 0;
    uint32_t high = clip->eventCount;
    while (low < high)
    {
        uint32_t middle = low + (high - low) / 2;
        float time = LoadF32(clip->events + (size_t)middle * MANI_CLIP_EVENT_SIZE);
        if (time < t || (inclusive && time == t))
        {
            low = middle + 1;
        }
        else
        {
            high = middle;
        }
    }
    return low;
}

MANI_NOINLINE void maniClipEventRangeEntered(const maniClip* clip, float from, float to,
                                             uint32_t* firstOut, uint32_t* countOut)
{
    uint32_t first = 0;
    uint32_t end = 0;
    if (from < to)
    {
        first = CountBefore(clip, from, false);
        end = CountBefore(clip, to, false);
    }
    else if (to < from)
    {
        first = CountBefore(clip, to, true);
        end = CountBefore(clip, from, true);
    }
    *firstOut = first;
    *countOut = end - first;
}

void maniClip_GetEventRange(const maniClip* clip, float from, float to, uint32_t* firstOut,
                            uint32_t* countOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniClipEventRangeEntered(clip, from, to, firstOut, countOut);
    maniFloatEnvLeave(env);
}
