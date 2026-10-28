// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the two halves of the clip builder share: channel checks and
// evaluation (clip_build.c) and the named items (clip_build_named.c).

#ifndef MAUL_ANIM_SRC_CLIP_BUILD_H
#define MAUL_ANIM_SRC_CLIP_BUILD_H

#include "root_motion_build.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

// An importer's clip settings in range for a skeleton: a rate finite and
// not negative, a loop and basis known, a root motion source
// valid with its bone in the skeleton.
bool maniImportClipOptionsValid(const maniImportClipOptions* options, const maniSkeleton* skeleton);

// A clip source from an importer's tracks and its clip settings.
static inline void maniApplyImportClipOptions(const maniImportClipOptions* options,
                                              maniClipSource* source)
{
    source->loop = options->loop;
    source->additive = options->additive;
    source->rootMotion = options->rootMotion;
}

// Checks a channel of keys of dimension 1, 3 or 4; a refusal names item
// and has list (0 for bone tracks) or-ed into its reason.
maniBuildResult maniCheckChannel(const maniChannelSource* channel, uint32_t dimension,
                                 uint32_t item, maniBuildReason list);

// A channel's value at a time, as glTF's Appendix C defines it, in
// binary64; rotations normalized.
void maniEvaluateChannel(const maniChannelSource* channel, uint32_t dimension, double time,
                         double out[4]);

// Sample k's time of n on the grid: k / rate, the last at the duration.
double maniClipSampleTime(uint32_t k, uint32_t n, float rate, float duration);

// The named items' sections: float tracks and samples, discrete tracks
// and keys, events, markers, names; laid out once the grid is known.
typedef struct maniClipNamedLayout
{
    uint8_t* block;
    uint64_t blockSize;
    const uint8_t* data[7];
    uint64_t sizes[7];
    uint32_t counts[4];
} maniClipNamedLayout;

// A source's grid: its duration, rate and sample count.
typedef struct maniClipTiming
{
    float duration;
    float rate;
    uint32_t sampleCount;
} maniClipTiming;

// Checks a source as the build does, all but repeated names: seen holds
// a zeroed word per skeleton bone and receives each bone's track index
// + 1.
maniBuildResult maniCheckClipSource(const maniClipSource* source, uint32_t* seen,
                                    maniClipTiming* timingOut);

// Checks every named item, and the latest time any key, event or marker
// names (lastOut), before the duration is known.
maniBuildResult maniCheckClipNamedSource(const maniClipSource* source, float* lastOut);

// Refuses discrete keys, events and markers past the duration.
maniBuildResult maniCheckClipNamedDuration(const maniClipSource* source, float duration);

// Lays out the named sections for a checked source and its grid; refuses
// repeated names. The layout's block is the builder's to release with
// maniReleaseClipNamed.
maniBuildResult maniLayClipNamed(maniBuilder* builder, const maniClipSource* source, float duration,
                                 float rate, uint32_t sampleCount, maniClipNamedLayout* layout);

void maniReleaseClipNamed(maniBuilder* builder, maniClipNamedLayout* layout);

// Builds a clip as maniBuildClip does, with root motion given (NULL for
// none) in place of the source's (which must be NULL): its source and
// moves written as they are, its bone's channels kept as they come. The
// bone must have keyed translation and rotation, and the clip be one of
// poses with a move per sample; else the reason is root motion.
maniBuildResult maniBuildClipMoved(maniBuilder* builder, const maniClipSource* source,
                                   const maniRootMotionWork* given);

#endif // MAUL_ANIM_SRC_CLIP_BUILD_H
