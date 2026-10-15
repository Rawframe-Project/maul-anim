// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The clip file's layout (mani-0006), shared by its view, its builder and
// its compressor: kind 2 of the container, sections tagged 1 to 10 at
// full precision, 1 to 15 compressed; an empty section is left out of
// the table (absent is empty, and nothing else is), so the identity is
// the same either way.

#ifndef MAUL_ANIM_SRC_CLIP_FILE_H
#define MAUL_ANIM_SRC_CLIP_FILE_H

#include <stdbool.h>
#include <stdint.h>

#define MANI_CLIP_FORMAT_VERSION      2u
#define MANI_CLIP_SECTIONS            10u
#define MANI_CLIP_COMPRESSED_SECTIONS 15u
// The header: skeleton identity (u64), skeleton bone count, sample count
// (u32 each), sample rate, duration (binary32 each), loop policy, track
// count, floats per sample, float track count, discrete track count,
// event count, marker count, segment count (0 for full precision), form
// (0 full precision, 1 compressed), additive basis (0 none, 1 rest, 2
// first frame; u32 each).
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
    // Compressed: a byte per bone track (2 bits per channel: translation,
    // rotation, scale, each a maniClipForm; the top 2 bits 3 less the
    // component a stored rotation leaves out, 0 x to 3 w, so 0 for w),
    // then one per float track, a maniClipForm.
    maniClipTagForms = 11,
    // Compressed: the constant sub-tracks' values in track order, three
    // binary32 for a channel (a rotation's x, y, z with w positive), one
    // for a float track.
    maniClipTagConstants = 12,
    // Compressed: the animated sub-tracks' clip ranges in track order,
    // minimum and extent per component, binary32.
    maniClipTagRanges = 13,
    // Compressed: per segment, its first sample, sample count, offset in
    // the segment data, and bits per sample of the bone and of the float
    // sub-tracks (u32 each).
    maniClipTagSegments = 14,
    // Compressed: per segment, a rate byte per animated sub-track; with
    // several segments its ranges (per component a minimum and an extent
    // byte, or a 16-bit value at rate 0); then its samples' bits, sample-
    // major, least significant bit first. Eight spare bytes end it.
    maniClipTagSegmentData = 15,
};

// A sub-track's form in a compressed clip.
typedef enum maniClipForm
{
    maniClipFormAbsent = 0,
    maniClipFormDefault = 1,
    maniClipFormConstant = 2,
    maniClipFormAnimated = 3,
} maniClipForm;

#define MANI_CLIP_SEGMENT_SIZE 20u
// Segments of about 16 samples; a clip of at most 31 is one segment.
#define MANI_CLIP_SEGMENT_IDEAL 16u
#define MANI_CLIP_SEGMENT_MAX   31u

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
// 0: 1 for a duration of 0, else ceil(duration * rate) + 1 in binary32,
// the product snapped to a whole number within 2^-22 of it.
// False when it would pass MANI_MAX_SAMPLES.
bool maniClipSampleCountFor(float duration, float rate, uint32_t* countOut);

// The identity: maniHash64 over the sections in tag order, each
// continuing the last.
uint64_t maniClipComputeIdentity(const uint8_t* const* sections, const uint64_t* sizes,
                                 uint32_t count);

#endif // MAUL_ANIM_SRC_CLIP_FILE_H
