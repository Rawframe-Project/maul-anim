// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The compressor's plan (mani-0006): a clip's sub-tracks, their values
// and how a rate plays them, shared by the folding and the search
// (clip_search.c) and the writing and the report (clip_compress.c).

#ifndef MAUL_ANIM_SRC_CLIP_PLAN_H
#define MAUL_ANIM_SRC_CLIP_PLAN_H

#include "allocator.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
    maniSubTranslation = 0,
    maniSubRotation = 1,
    maniSubScale = 2,
    maniSubFloat = 3,
};

// A sub-track: a bone track's channel, or a float track.
typedef struct maniSubTrack
{
    // The bone track and channel (and the bone), or the float track
    // (channel maniSubFloat).
    uint32_t track;
    uint32_t channel;
    uint32_t bone;
    // Where its value starts in a sample: floats into the bone samples,
    // or the float track's index.
    uint32_t offset;
    // A maniClipForm, and its index among the animated sub-tracks.
    uint32_t form;
    uint32_t animated;
    // A rotation's component left out (0 x to 3 w), made positive.
    uint32_t dropped;
    // A float track's precision.
    float tolerance;
    // The first sample's value (kept by a constant), and the clip range.
    float constant[3];
    float minimum[3];
    float extent[3];
} maniSubTrack;

typedef struct maniCompressPlan
{
    const maniClip* clip;
    const maniSkeleton* skeleton;
    const maniCompressOptions* options;
    const maniAllocator* allocator;
    maniSubTrack* subs;
    uint32_t subCount;
    // Per bone, its first sub-track, or UINT32_MAX without a track.
    uint32_t* boneSubs;
    uint32_t animatedChannels;
    uint32_t animatedFloats;
    uint32_t constantChannels;
    uint32_t constantFloats;
    uint32_t segmentCount;
    // First samples, segmentCount + 1 of them.
    uint32_t* firsts;
    // Per segment and animated sub-track: its rate and six range bytes
    // (minimum and extent per component, or at rate 0 the 16-bit values).
    uint8_t* rates;
    uint8_t* segmentRanges;
} maniCompressPlan;

// The components of a sub-track: 3, or 1 for a float track.
uint32_t maniPlanWidth(const maniSubTrack* t);

// A sub-track's value at a sample; a rotation as its kept components,
// signed so the dropped one is positive.
void maniPlanValue(const maniCompressPlan* p, const maniSubTrack* t, uint32_t sample, float out[3]);

// The segments: one up to MANI_CLIP_SEGMENT_MAX samples, else about
// MANI_CLIP_SEGMENT_IDEAL each; and their first samples.
uint32_t maniPlanSegmentCount(uint32_t samples);
void maniPlanSplit(uint32_t samples, uint32_t segments, uint32_t* firsts);

// A rotation's component to leave out: the one whose smallest magnitude
// over the clip is largest (w on ties), so the one rebuilt stays clear of
// 0, where rebuilding loses precision.
void maniPlanChooseDropped(const maniCompressPlan* p, maniSubTrack* t);

// A sub-track's first value and clip range.
void maniPlanRanges(const maniCompressPlan* p, maniSubTrack* t);

// A component's bits at a rate (1 to 23, or its binary32 at the raw
// rate), against the segment's range bytes (NULL with one segment).
uint32_t maniPlanQuantize(const maniSubTrack* t, uint32_t c, float v, uint32_t rate,
                          const uint8_t* range);

// The value a component plays at a rate: encoded and decoded as the
// sampler decodes it. range holds the segment's bytes (minimum, extent
// per component, or the 16-bit value at rate 0); NULL with one segment.
float maniPlanPlayed(const maniSubTrack* t, uint32_t c, float v, uint32_t rate,
                     const uint8_t* range);

// A segment's range bytes for a sub-track, and the 16-bit values it
// would hold at rate 0 (the middle of the segment's values).
void maniPlanSegmentRange(const maniCompressPlan* p, const maniSubTrack* t, uint32_t s,
                          uint8_t range[6], uint8_t constant[6]);

// A bone channel's value as the sampler sets it into a local: decoded
// at a rate (MANI_CODEC_RAW_RATE for the value itself), or the stored
// constant (form constant), or the rest (form default).
void maniPlanSetChannel(const maniCompressPlan* p, const maniSubTrack* t, uint32_t sample,
                        uint32_t rate, const uint8_t* range, maniTransform* local);

// Float tracks: constant when every sample is within the precision of
// the first; then per segment the fewest bits within it.
void maniPlanFoldFloat(const maniCompressPlan* p, maniSubTrack* t);
void maniPlanFloatRates(maniCompressPlan* p);

// A bone's precision and shell distance, from the options.
float maniPlanPrecision(const maniCompressPlan* p, uint32_t bone);
float maniPlanShell(const maniCompressPlan* p, uint32_t bone);

#endif // MAUL_ANIM_SRC_CLIP_PLAN_H
