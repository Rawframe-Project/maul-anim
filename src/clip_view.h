// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What clip.c, clip_tracks.c and clip_compressed.c share: the checks of
// a clip's float and discrete tracks, events, markers and names, and of
// a compressed clip's sections, at open; a compressed clip's samplers.

#ifndef MAUL_ANIM_SRC_CLIP_VIEW_H
#define MAUL_ANIM_SRC_CLIP_VIEW_H

#include "clip_file.h"
#include "pose_memory.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Checks sections 4 to 10 against the counts the header gave (in clip)
// and fills clip's pointers to them.
// Each takes the sections by tag (index tag - 1): their bytes, sizes,
// offsets, and the offsets of their size fields in the table (of the
// section count for an absent one), where a wrong size is reported.
maniFileResult maniCheckClipNamed(const uint8_t* const* data, const uint64_t* sizes,
                                  const uint64_t* at, const uint64_t* entry, maniClip* clip);

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

// A bone's channel (0 translation, 1 rotation, 2 scale) set to its rest
// in a pose slot; first is its first component, count how many.
static inline uint32_t maniChannelFirst(uint32_t channel)
{
    return channel == 0 ? 0u : (channel == 1 ? 3u : 7u);
}

static inline void maniSlotRest(float* slot, const maniSkeleton* skeleton, uint32_t bone,
                                uint32_t channel)
{
    uint32_t first = maniChannelFirst(channel);
    uint32_t count = channel == 1 ? 4u : 3u;
    float rest[4];
    memcpy(rest,
           skeleton->rests + MANI_POSE_COMPONENTS * sizeof(float) * (size_t)bone +
               sizeof(float) * first,
           sizeof(float) * count);
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

// The floats a decoded sample of a compressed clip holds: three per
// animated bone channel, one per animated float track, in that order.
uint32_t maniCompressedSampleFloats(const maniClip* clip);
void maniDecodeCompressedSample(const maniClip* clip, uint32_t sample, float* out);

#endif // MAUL_ANIM_SRC_CLIP_VIEW_H
