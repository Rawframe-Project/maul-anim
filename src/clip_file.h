// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The clip file's layout, shared by its view, its builder and
// its compressor: kind 2 of the container, sections tagged 1 to 10 at
// full precision, 1 to 15 compressed, and 16 (root motion) in either; an
// empty section is left out of the table (absent is empty, and nothing
// else is), so the identity is the same either way.

#ifndef MAUL_ANIM_SRC_CLIP_FILE_H
#define MAUL_ANIM_SRC_CLIP_FILE_H

#include <stdbool.h>
#include <stdint.h>

#define MANI_CLIP_FORMAT_VERSION      4u
#define MANI_CLIP_SECTIONS            10u
#define MANI_CLIP_COMPRESSED_SECTIONS 15u
#define MANI_CLIP_ALL_SECTIONS        16u
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
    // Compressed: for each kind of bone channel in kind order (rotation,
    // translation, scale), a bitset of the bone tracks whose channel is
    // constant, then one of those whose channel is animated (track t at
    // bit t % 8 of byte t / 8, maniClipBitsetBytes each); then 2 bits
    // a bone track, 3 less the component its stored rotation leaves out
    // (0 x to 3 w, so 0 for w; track t at bits 2 (t % 4) of byte t / 4);
    // then a byte per float track, a maniClipForm (constant or animated).
    // A channel in its track's mask in neither bitset is the rest.
    maniClipTagForms = 11,
    // Compressed: the constant sub-tracks' values in kind order (each
    // kind's in track order, then the float tracks), three binary32 for a
    // channel (a rotation's kept components, the dropped one positive),
    // one for a float track.
    maniClipTagConstants = 12,
    // Compressed: per kind (rotation, translation, scale, then the float
    // tracks), its animated sub-tracks' clip ranges in blocks of eight in
    // track order, the last block holding the rest: per component the
    // block's minimums, then per component its extents, binary32. A block
    // is one load a component on eight lanes.
    maniClipTagRanges = 13,
    // Compressed: per segment, its first sample, sample count, offset in
    // the segment data, and bits per sample of the bone and of the float
    // sub-tracks (u32 each).
    maniClipTagSegments = 14,
    // Compressed: per segment, a rate byte per animated sub-track in kind
    // order; with several segments their ranges in that order (per
    // component a minimum byte, then per component an extent byte, or a
    // 16-bit value a component at rate 0); then its samples' bits,
    // sample-major, each sample's sub-tracks in kind order, least
    // significant bit first. Eight spare bytes end it.
    maniClipTagSegmentData = 15,
    // Root motion, in either form: its source
    // (MANI_ROOT_MOTION_HEADER_SIZE), the whole-period move, then per
    // sample the motion's translation and rotation.
    maniClipTagRootMotion = 16,
};

// The root motion header: bone, translation axes, rotation, twist axis,
// translation reference, rotation reference, loop (u32 each).
#define MANI_ROOT_MOTION_HEADER_SIZE 28u
// A move: translation x, y, z and rotation x, y, z, w (binary32 each).
#define MANI_ROOT_MOTION_MOVE_FLOATS 7u
#define MANI_ROOT_MOTION_PERIOD_SIZE 28u
// A stored translation component's bound, 2^64: a move across 2^32 wraps
// stays well inside binary32.
#define MANI_ROOT_MOTION_REACH 18446744073709551616.0

// The root motion section's size for a clip of n samples.
static inline uint64_t maniRootMotionSize(uint32_t n)
{
    return MANI_ROOT_MOTION_HEADER_SIZE + MANI_ROOT_MOTION_PERIOD_SIZE +
           4 * (uint64_t)MANI_ROOT_MOTION_MOVE_FLOATS * n;
}

// A sub-track's form in a compressed clip.
typedef enum maniClipForm
{
    maniClipFormAbsent = 0,
    maniClipFormDefault = 1,
    maniClipFormConstant = 2,
    maniClipFormAnimated = 3,
} maniClipForm;

// The kinds of bone channel in the order a compressed clip stores them.
enum
{
    maniClipKindRotation = 0,
    maniClipKindTranslation = 1,
    maniClipKindScale = 2,
    maniClipKinds = 3,
};

// A kind's channel in a track's mask and its forms (0 translation, 1
// rotation, 2 scale).
static inline uint32_t maniClipKindChannel(uint32_t kind)
{
    return kind == maniClipKindRotation ? 1u : (kind == maniClipKindTranslation ? 0u : 2u);
}

// The bytes of one bitset over a clip's bone tracks.
static inline uint64_t maniClipBitsetBytes(uint32_t tracks)
{
    return ((uint64_t)tracks + 7) / 8;
}

// The forms section's size: two bitsets a kind, the dropped components,
// a byte a float track.
static inline uint64_t maniClipFormsSize(uint32_t tracks, uint32_t floatTracks)
{
    return 2 * maniClipKinds * maniClipBitsetBytes(tracks) + ((uint64_t)tracks + 3) / 4 +
           floatTracks;
}

// The byte offset, in a kind's clip ranges, of the minimum (extent 0) or
// the extent (1) of component c of the i-th of the kind's n animated
// sub-tracks of width components: blocks of eight, the last holding the
// rest.
static inline uint64_t maniClipRangeAt(uint32_t n, uint32_t width, uint32_t i, uint32_t c,
                                       uint32_t extent)
{
    uint32_t first = i - i % 8;
    uint32_t lanes = n - first < 8 ? n - first : 8;
    return 4 * ((uint64_t)first * 2 * width + (uint64_t)(extent * width + c) * lanes + (i - first));
}

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
