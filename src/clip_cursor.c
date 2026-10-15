// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The forward cursor (mani-0006): two cached
// samples of a compressed clip, decoded once and interpolated by the
// sampler's own code, so its poses are random access's bits. Bound to a
// clip by identity and address, as ACL's context binds by a hash.

#include "clip_view.h"
#include "pose_memory.h"

#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

#include <stddef.h>
#include <stdint.h>

#define CURSOR_TAG 0x63727372u
#define NO_SAMPLE  UINT32_MAX

struct maniClipCursor
{
    uint64_t identity;
    const maniClip* clip;
    uint32_t tag;
    // The floats each of the two slots can hold, and holds for the clip.
    uint32_t capacity;
    uint32_t floats;
    uint32_t samples[2];
    float values[];
};

static size_t CacheFloats(const maniClip* clip)
{
    return clip->compressed ? maniCompressedSampleFloats(clip) : 0;
}

size_t maniClipCursorSize(const maniClip* clip)
{
    if (clip == NULL)
    {
        return 0;
    }
    return offsetof(maniClipCursor, values) + 2 * CacheFloats(clip) * sizeof(float);
}

maniResult maniInitClipCursor(void* memory, size_t size, maniClipCursor** cursorOut)
{
    if (memory == NULL || cursorOut == NULL || (uintptr_t)memory % MANI_CLIP_CURSOR_ALIGNMENT != 0)
    {
        return mani_errorInvalid;
    }
    size_t header = offsetof(maniClipCursor, values);
    if (size < header)
    {
        return mani_errorCapacity;
    }
    size_t slot = (size - header) / (2 * sizeof(float));
    maniClipCursor* cursor = memory;
    *cursor = (maniClipCursor){
        .tag = CURSOR_TAG,
        .capacity = slot < UINT32_MAX ? (uint32_t)slot : UINT32_MAX,
        .samples = {NO_SAMPLE, NO_SAMPLE},
    };
    *cursorOut = cursor;
    return mani_success;
}

// The cursor bound to the clip: false when it is too small for it. The
// shape is compared too, so even a crafted clip whose identity collides
// at the same address cannot make it read past its memory.
static bool Bind(maniClipCursor* cursor, const maniClip* clip)
{
    size_t floats = CacheFloats(clip);
    if (cursor->clip == clip && cursor->identity == clip->identity && cursor->floats == floats)
    {
        return true;
    }
    if (floats > cursor->capacity)
    {
        return false;
    }
    cursor->clip = clip;
    cursor->identity = clip->identity;
    cursor->floats = (uint32_t)floats;
    cursor->samples[0] = NO_SAMPLE;
    cursor->samples[1] = NO_SAMPLE;
    return true;
}

// The slot holding a sample, or 2.
static uint32_t SlotOf(const maniClipCursor* cursor, uint32_t sample)
{
    return cursor->samples[0] == sample ? 0 : cursor->samples[1] == sample ? 1 : 2;
}

static const float* Slot(const maniClipCursor* cursor, uint32_t slot, size_t floats)
{
    return cursor->values + slot * floats;
}

// The pair's two samples decoded into the slots, reusing either held.
static void Load(maniClipCursor* cursor, const maniClip* clip, maniClipPair* pair)
{
    if (!clip->compressed)
    {
        return;
    }
    size_t floats = CacheFloats(clip);
    uint32_t a = SlotOf(cursor, pair->first);
    uint32_t b = SlotOf(cursor, pair->second);
    if (a == 2)
    {
        a = b == 0 ? 1 : 0;
        maniDecodeCompressedSample(clip, pair->first, cursor->values + a * floats);
        cursor->samples[a] = pair->first;
    }
    if (b == 2)
    {
        b = pair->second == pair->first ? a : 1 - a;
        if (b != a)
        {
            maniDecodeCompressedSample(clip, pair->second, cursor->values + b * floats);
            cursor->samples[b] = pair->second;
        }
    }
    pair->cacheA = Slot(cursor, a, floats);
    pair->cacheB = Slot(cursor, b, floats);
}

static bool Valid(const maniClipCursor* cursor)
{
    return cursor != NULL && cursor->tag == CURSOR_TAG;
}

maniResult maniSampleClipCursor(maniClipCursor* cursor, const maniClip* clip,
                                const maniSkeleton* skeleton, float time, maniPose* pose)
{
    if (!Valid(cursor) || !maniClipFits(clip, skeleton, pose))
    {
        return mani_errorInvalid;
    }
    if (!Bind(cursor, clip))
    {
        return mani_errorCapacity;
    }
    maniClipPair pair = maniClipLocate(clip, time);
    Load(cursor, clip, &pair);
    maniSampleClipPair(clip, skeleton, &pair, NULL, pose);
    return mani_success;
}

maniResult maniSampleClipFloatsCursor(maniClipCursor* cursor, const maniClip* clip, float time,
                                      float* values, uint32_t capacity, uint32_t* countOut)
{
    if (!Valid(cursor) || clip == NULL || (values == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = clip->floatTrackCount;
    }
    if (capacity < clip->floatTrackCount || !Bind(cursor, clip))
    {
        return mani_errorCapacity;
    }
    maniClipPair pair = maniClipLocate(clip, time);
    Load(cursor, clip, &pair);
    maniSampleClipPairFloats(clip, &pair, values);
    return mani_success;
}
