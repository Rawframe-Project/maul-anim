// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What clip.c, clip_tracks.c and clip_compressed.c share: the checks of
// a clip's float and discrete tracks, events, markers and names, and of
// a compressed clip's sections, at open; a compressed clip's samplers.

#ifndef MAUL_ANIM_SRC_CLIP_VIEW_H
#define MAUL_ANIM_SRC_CLIP_VIEW_H

#include "clip_file.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

#include <stdint.h>

// Checks sections 4 to 10 against the counts the header gave (in clip)
// and fills clip's pointers to them.
maniFileResult maniCheckClipNamed(const uint8_t* const* data, const uint64_t* sizes,
                                  const uint64_t* at, maniClip* clip);

// Checks a compressed clip's sections 11 to 15 against its header and
// tracks (in clip) and fills clip's pointers and counts.
maniFileResult maniCheckClipCompressed(const uint8_t* const* data, const uint64_t* sizes,
                                       const uint64_t* at, maniClip* clip);

// Sample k of a clip of either form, exactly: the clip's channels into
// the pose (the rest elsewhere) and, when values is not NULL, its float
// tracks. The clip and skeleton are known to match the pose.
void maniSampleClipAt(const maniClip* clip, const maniSkeleton* skeleton, uint32_t k,
                      maniPose* pose, float* values);

// A compressed clip's bone channels between samples first and second
// at alpha, written over a pose whose locals hold the rest.
void maniSampleCompressedBones(const maniClip* clip, uint32_t first, uint32_t second, float alpha,
                               maniPose* pose);

// A compressed clip's float tracks between two samples, in track order;
// a step track (step[i] true) holds the first below alpha 1.
void maniSampleCompressedFloats(const maniClip* clip, uint32_t first, uint32_t second, float alpha,
                                float* values);

#endif // MAUL_ANIM_SRC_CLIP_VIEW_H
