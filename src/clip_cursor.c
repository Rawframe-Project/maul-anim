// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The forward cursor: two cached
// samples of a compressed clip, decoded once and interpolated by the
// sampler's own code, so its poses are random access's bits. Bound to a
// clip by identity and address, as ACL's context binds by a hash. On a
// vector backend the samples are also held in the pose's groups, and a
// pose is one interpolation a group (clip_lanes.h).

#include "clip_lanes.h"
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
    // The floats the cursor can hold, and the decoded floats of one sample
    // for the clip; after the two decoded samples, on a vector backend,
    // the two in the pose's groups and the groups' masks.
    uint32_t capacity;
    uint32_t floats;
    // The clip's bones, which size the samples held in groups.
    uint32_t bones;
    uint32_t samples[2];
    uint32_t expanded[2];
    float values[];
};

#define GROUP_FLOATS (MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS)
#define MASK_FLOATS  (MANI_POSE_GROUP_BONES * 3)

#if defined(MANI_CLIP_LANES)
// A clip's pose groups.
static size_t Groups(const maniClip* clip)
{
    return ((size_t)clip->skeletonBoneCount + MANI_POSE_GROUP_BONES - 1) / MANI_POSE_GROUP_BONES;
}
#endif

static size_t CacheFloats(const maniClip* clip)
{
    return clip->compressed ? maniCompressedSampleFloats(clip) : 0;
}

// The floats held for a clip: two decoded samples, and on a vector
// backend, when grouped, two samples in groups and the masks.
static size_t HeldFloats(const maniClip* clip, size_t floats, bool grouped)
{
#if defined(MANI_CLIP_LANES)
    size_t groups = grouped && floats > 0 ? Groups(clip) * (2 * GROUP_FLOATS + MASK_FLOATS) : 0;
    return 2 * floats + groups;
#else
    (void)clip;
    (void)grouped;
    return 2 * floats;
#endif
}

size_t maniClipCursorSize(const maniClip* clip)
{
    if (clip == NULL)
    {
        return 0;
    }
    return offsetof(maniClipCursor, values) +
           HeldFloats(clip, CacheFloats(clip), true) * sizeof(float);
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
    size_t floats = (size - header) / sizeof(float);
    maniClipCursor* cursor = memory;
    *cursor = (maniClipCursor){
        .tag = CURSOR_TAG,
        .capacity = floats < UINT32_MAX ? (uint32_t)floats : UINT32_MAX,
        .samples = {NO_SAMPLE, NO_SAMPLE},
        .expanded = {NO_SAMPLE, NO_SAMPLE},
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
    if (cursor->clip == clip && cursor->identity == clip->identity && cursor->floats == floats &&
        cursor->bones == clip->skeletonBoneCount)
    {
        return true;
    }
    if (HeldFloats(clip, floats, false) > cursor->capacity)
    {
        return false;
    }
    cursor->clip = clip;
    cursor->identity = clip->identity;
    cursor->floats = (uint32_t)floats;
    cursor->bones = clip->skeletonBoneCount;
    cursor->samples[0] = NO_SAMPLE;
    cursor->samples[1] = NO_SAMPLE;
    cursor->expanded[0] = NO_SAMPLE;
    cursor->expanded[1] = NO_SAMPLE;
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

#if defined(MANI_CLIP_LANES)
// The pair's samples laid out in groups once each, rests included (every
// skeleton the clip fits has its identity, which covers the rests), then
// one interpolation a group into the pose.
static void Lanes(maniClipCursor* cursor, const maniClip* clip, const maniSkeleton* skeleton,
                  const maniClipPair* pair, maniPose* pose)
{
    size_t floats = cursor->floats;
    size_t held = Groups(clip) * GROUP_FLOATS;
    float* groups = cursor->values + 2 * floats;
    float* masks = groups + 2 * held;
    for (uint32_t slot = 0; slot < 2; ++slot)
    {
        if (cursor->samples[slot] != NO_SAMPLE && cursor->expanded[slot] != cursor->samples[slot])
        {
            maniExpandCompressedSample(clip, skeleton, cursor->values + slot * floats,
                                       groups + slot * held, masks);
            cursor->expanded[slot] = cursor->samples[slot];
        }
    }
    const float* a = groups + (pair->cacheA == cursor->values ? 0 : held);
    const float* b = groups + (pair->cacheB == cursor->values ? 0 : held);
    float* locals = maniPoseLocals(pose);
    for (size_t g = 0; g < Groups(clip); ++g)
    {
        maniClipLanesInterpolate(a + g * GROUP_FLOATS, b + g * GROUP_FLOATS,
                                 masks + g * MASK_FLOATS, pair->alpha, locals + g * GROUP_FLOATS);
    }
    maniPoseMarkStale(pose);
    maniPoseSetKind(pose, clip->additive != maniAdditiveNone);
}
#endif

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
#if defined(MANI_CLIP_LANES)
    // The groups when the memory holds them for this clip,
    // else the decoded samples alone, walked: the same bits either way.
    if (clip->compressed && cursor->floats > 0 &&
        HeldFloats(clip, cursor->floats, true) <= cursor->capacity)
    {
        Lanes(cursor, clip, skeleton, &pair, pose);
        return mani_success;
    }
#endif
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

size_t maniClipCursorCompactSize(const maniClip* clip)
{
#if defined(MANI_CLIP_LANES)
    if (clip == NULL)
    {
        return 0;
    }
    return offsetof(maniClipCursor, values) +
           HeldFloats(clip, CacheFloats(clip), false) * sizeof(float);
#else
    // The scalar backend holds no groups: the one size.
    return maniClipCursorSize(clip);
#endif
}
