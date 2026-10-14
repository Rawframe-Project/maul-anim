// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The clip file's layout (mani-0006), shared by its view and its builder:
// kind 2 of the container, full precision.

#ifndef MAUL_ANIM_SRC_CLIP_FILE_H
#define MAUL_ANIM_SRC_CLIP_FILE_H

#include <stdbool.h>
#include <stdint.h>

#define MANI_CLIP_FORMAT_VERSION 1u
#define MANI_CLIP_SECTIONS       10u
// The header: skeleton identity (u64), skeleton bone count, sample count
// (u32 each), sample rate, duration (binary32 each), loop policy, track
// count, floats per sample, float track count, discrete track count,
// event count, marker count (u32 each), then zeros.
#define MANI_CLIP_HEADER_SIZE 64u
// A float or discrete track: name offset, name length (u32 each), name
// hash (u64), then for a float track its interpolation (u32: 0 linear,
// 1 step) and zero, for a discrete track its first key and key count.
#define MANI_CLIP_NAMED_TRACK_SIZE 24u
// A discrete key: time (binary32), value (i32).
#define MANI_CLIP_KEY_SIZE 8u
// An event: time (binary32), payload (i32), name offset, name length
// (u32 each), name hash (u64). A marker: the same with a zero payload.
#define MANI_CLIP_EVENT_SIZE 24u
// A track: bone (u16), channel mask (u8), zero (u8).
#define MANI_CLIP_TRACK_SIZE 4u

enum
{
    maniClipTagHeader = 1,
    maniClipTagTracks = 2,
    // Sample-major binary32: for each sample, each track's animated
    // channels (translation 3, rotation 4, scale 3) in that order.
    maniClipTagSamples = 3,
    // Float tracks by increasing name hash, then their samples,
    // sample-major.
    maniClipTagFloatTracks = 4,
    maniClipTagFloatSamples = 5,
    // Discrete tracks by increasing name hash, then their keys, each
    // track's in increasing time.
    maniClipTagDiscreteTracks = 6,
    maniClipTagDiscreteKeys = 7,
    // Events and markers by time, ties in source order.
    maniClipTagEvents = 8,
    maniClipTagMarkers = 9,
    // The names' UTF-8 bytes.
    maniClipTagNames = 10,
};

enum
{
    maniChannelTranslation = 1,
    maniChannelRotation = 2,
    maniChannelScale = 4,
    maniChannelAll = 7,
};

// The floats a track's mask puts in each sample.
static inline uint32_t maniChannelFloats(uint32_t mask)
{
    return ((mask & maniChannelTranslation) ? 3u : 0u) + ((mask & maniChannelRotation) ? 4u : 0u) +
           ((mask & maniChannelScale) ? 3u : 0u);
}

// The sample count for a duration and rate, both finite, the rate above
// 0: 1 for a duration of 0, else ceil(duration * rate) + 1 in binary32.
// False when it would pass MANI_MAX_SAMPLES.
bool maniClipSampleCountFor(float duration, float rate, uint32_t* countOut);

// The identity: maniHash64 over the sections in tag order, each
// continuing the last.
uint64_t maniClipComputeIdentity(const uint8_t* const sections[MANI_CLIP_SECTIONS],
                                 const uint64_t sizes[MANI_CLIP_SECTIONS]);

#endif // MAUL_ANIM_SRC_CLIP_FILE_H
