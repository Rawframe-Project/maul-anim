// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What clip.c, clip_tracks.c and clip_compressed.c share: the checks of
// a clip's float and discrete tracks, events, markers and names, and of
// a compressed clip's sections, at open; a compressed clip's samplers.

#ifndef MAUL_ANIM_SRC_CLIP_VIEW_H
#define MAUL_ANIM_SRC_CLIP_VIEW_H

#include "bytes.h"
#include "clip_file.h"
#include "pose_memory.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
#endif

// Checks sections 4 to 10 against the counts the header gave (in clip)
// and fills clip's pointers to them.
// Each takes the sections by tag (index tag - 1): their bytes, sizes,
// offsets, and the offsets of their size fields in the table (of the
// section count for an absent one), where a wrong size is reported.
maniFileResult maniCheckClipNamed(const uint8_t* const* data, const uint64_t* sizes,
                                  const uint64_t* at, const uint64_t* entry, maniClip* clip);

// The bytes before a compressed clip's segment's samples: its rates and,
// with several segments, its ranges.
static inline uint64_t maniCompressedSegmentHead(const maniClip* clip)
{
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    uint64_t ranges = 6 * (uint64_t)clip->animatedChannels + 2 * (uint64_t)clip->animatedFloats;
    return n + (clip->segmentCount > 1 ? ranges : 0);
}

// A kind's bitset of the tracks whose channel is constant (animated 0)
// or animated (1).
static inline const uint8_t* maniCompressedBitset(const maniClip* clip, uint32_t kind,
                                                  uint32_t animated)
{
    return clip->forms + (2 * kind + animated) * maniClipBitsetBytes(clip->trackCount);
}

static inline const uint8_t* maniCompressedDropped(const maniClip* clip)
{
    return clip->forms + 2 * maniClipKinds * maniClipBitsetBytes(clip->trackCount);
}

// A track's 2 bits among the dropped components': 3 less the component
// its stored rotation leaves out.
static inline uint32_t maniCompressedDroppedField(const uint8_t* dropped, uint32_t t)
{
    return (dropped[t / 4] >> (2 * (t % 4))) & 3u;
}

static inline const uint8_t* maniCompressedFloatForms(const maniClip* clip)
{
    return maniCompressedDropped(clip) + ((uint64_t)clip->trackCount + 3) / 4;
}

static inline uint32_t maniClipTrackBone(const maniClip* clip, uint32_t t)
{
    return maniLoadU16(clip->tracks + (size_t)t * MANI_CLIP_TRACK_SIZE);
}

// The lowest set bit's index of a word not 0.
static inline uint32_t maniLowestBit(uint64_t word)
{
#if defined(_MSC_VER) && !defined(__clang__)
    unsigned long index = 0;
    _BitScanForward64(&index, word);
    return (uint32_t)index;
#else
    return (uint32_t)__builtin_ctzll(word);
#endif
}

// A walk over a bitset's tracks in increasing order, a word at a time.
typedef struct maniTrackWalk
{
    const uint8_t* set;
    uint64_t bytes;
    uint64_t next;
    uint64_t word;
    uint32_t base;
} maniTrackWalk;

static inline maniTrackWalk maniWalkTracks(const maniClip* clip, uint32_t kind, uint32_t animated)
{
    return (maniTrackWalk){maniCompressedBitset(clip, kind, animated),
                           maniClipBitsetBytes(clip->trackCount), 0, 0, 0};
}

// The next track in the set; the caller takes no more than the set holds
// (counted at open).
static inline uint32_t maniNextTrack(maniTrackWalk* w)
{
    while (w->word == 0)
    {
        uint64_t left = w->bytes - w->next;
        uint64_t word = 0;
        if (left >= 8)
        {
            word = maniLoadU64(w->set + w->next);
        }
        for (uint64_t k = 0; left < 8 && k < left; ++k)
        {
            word |= (uint64_t)w->set[w->next + k] << (8 * k);
        }
        w->base = (uint32_t)(8 * w->next);
        w->next += left < 8 ? left : 8;
        w->word = word;
    }
    uint32_t t = w->base + maniLowestBit(w->word);
    w->word &= w->word - 1;
    return t;
}

// Checks a compressed clip's sections 11 to 15 against its header and
// tracks (in clip) and fills clip's pointers and counts.
maniFileResult maniCheckClipCompressed(const uint8_t* const* data, const uint64_t* sizes,
                                       const uint64_t* at, const uint64_t* entry, maniClip* clip);

// The two samples a time falls between and the factor between them;
// with caches (not NULL), their decoded values (maniDecodeCompressedSample).
typedef struct maniClipPair
{
    uint32_t first;
    uint32_t second;
    float alpha;
    const float* cacheA;
    const float* cacheB;
} maniClipPair;

// A list of bones, strictly increasing.
typedef struct maniBoneList
{
    const uint32_t* bones;
    uint32_t count;
} maniBoneList;

// Whether a clip, a skeleton and a pose go together (none NULL).
bool maniClipFits(const maniClip* clip, const maniSkeleton* skeleton, const maniPose* pose);

// The two samples a time falls between and the factor; no caches.
maniClipPair maniClipLocate(const maniClip* clip, float time);

// A clip of either form at a pair into a pose, every bone (or the listed
// ones) written once; its float tracks.
void maniSampleClipPair(const maniClip* clip, const maniSkeleton* skeleton,
                        const maniClipPair* pair, const maniBoneList* list, maniPose* pose);
void maniSampleClipPairFloats(const maniClip* clip, const maniClipPair* pair, float* values);

// Sample k of a clip of either form, exactly: the clip's channels into
// the pose (the rest elsewhere) and, when values is not NULL, its float
// tracks. The clip and skeleton are known to match the pose.
void maniSampleClipAt(const maniClip* clip, const maniSkeleton* skeleton, uint32_t k,
                      maniPose* pose, float* values);

// A bone's channel (0 translation, 1 rotation, 2 scale) set to its rest,
// or for an additive clip to the identity, in a pose slot; first is its
// first component, count how many.
static inline uint32_t maniChannelFirst(uint32_t channel)
{
    return channel == 0 ? 0u : (channel == 1 ? 3u : 7u);
}

static inline void maniSlotRest(float* slot, const maniSkeleton* skeleton, uint32_t bone,
                                uint32_t channel, bool identity)
{
    static const float kIdentity[MANI_POSE_COMPONENTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                          0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    uint32_t first = maniChannelFirst(channel);
    uint32_t count = channel == 1 ? 4u : 3u;
    float rest[4];
    if (identity)
    {
        memcpy(rest, kIdentity + first, sizeof(float) * count);
    }
    else
    {
        memcpy(rest,
               skeleton->rests + MANI_POSE_COMPONENTS * sizeof(float) * (size_t)bone +
                   sizeof(float) * first,
               sizeof(float) * count);
    }
    maniPoseSlotPut(slot, first, rest, count);
}

// Whether a list (or none, every bone) takes a bone; next walks the list
// along increasing bones.
bool maniBoneListed(const maniBoneList* list, uint32_t bone, uint32_t* next);

// A compressed clip at a pair into a pose: every bone (or only the listed
// ones, list not NULL) written once, its rest and then its channels; the
// model marked stale.
void maniSampleCompressedBones(const maniClip* clip, const maniSkeleton* skeleton,
                               const maniClipPair* pair, const maniBoneList* list, maniPose* pose);

// A compressed clip's float tracks at a pair, in track order; a step
// track holds the first sample below alpha 1.
void maniSampleCompressedFloats(const maniClip* clip, const maniClipPair* pair, float* values);

// The floats a decoded sample of a compressed clip holds, in the file's
// kind order: four per animated rotation (rebuilt), three per animated
// translation and scale, one per animated float track.
uint32_t maniCompressedSampleFloats(const maniClip* clip);
void maniDecodeCompressedSample(const maniClip* clip, uint32_t sample, float* out);
// A decoded sample laid out as a pose's groups for the forward cursor,
// with the masks of the channels the walk interpolates (clip_lanes.h).
void maniExpandCompressedSample(const maniClip* clip, const maniSkeleton* skeleton,
                                const float* cache, float* groups, float* masks);

// maniClip_GetEventRange, maniClipCursor_Sample and maniClip_SampleBones for
// a caller already in the library's floating-point environment
// (fpenv.h), without entering it again.
void maniClipEventRangeEntered(const maniClip* clip, float from, float to, uint32_t* firstOut,
                               uint32_t* countOut);
maniResult maniSampleClipCursorEntered(maniClipCursor* cursor, const maniClip* clip,
                                       const maniSkeleton* skeleton, float time, maniPose* pose);
maniResult maniSampleClipBonesEntered(const maniClip* clip, const maniSkeleton* skeleton,
                                      float time, const uint32_t* bones, uint32_t count,
                                      maniPose* pose);

#endif // MAUL_ANIM_SRC_CLIP_VIEW_H
