// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motion database file's layout, shared by its view and
// its builder: kind 8 of the container. Records are little-endian;
// features are binary32, normalized, frame by frame; bounds cover runs of
// MANI_MATCH_SMALL_RUN and MANI_MATCH_LARGE_RUN frames.

#ifndef MAUL_ANIM_SRC_MATCH_FILE_H
#define MAUL_ANIM_SRC_MATCH_FILE_H

#include <stdbool.h>
#include <stdint.h>

#define MANI_MATCH_FORMAT_VERSION 2u
#define MANI_MATCH_SECTIONS       10u

enum
{
    // The skeleton's identity (u64), then u32 each: its bone count, the
    // frame, feature, channel, time, entry and clip counts, the sample
    // rate (binary32), the up and forward axes.
    maniMatchTagHeader = 1,
    // MANI_MATCH_CHANNEL_SIZE each: kind, bone, axis, time count, first
    // time, first feature, feature count (u32 each), weight (binary32),
    // name offset and length (u32 each; a float track's name, else 0).
    maniMatchTagChannels = 2,
    // A binary32 each: the channels' times, channel by channel.
    maniMatchTagTimes = 3,
    // A u64 each: the clips' identities.
    maniMatchTagClips = 4,
    // MANI_MATCH_ENTRY_SIZE each: clip, flags (u32 each), start, end,
    // cost bias (binary32 each), tags, first frame, frame count (u32
    // each).
    maniMatchTagEntries = 5,
    // A binary32 per feature: the offsets, then the scales.
    maniMatchTagNormalization = 6,
    // A binary32 per feature per frame, normalized.
    maniMatchTagFeatures = 7,
    // A binary32 per feature per run: the small runs' minimums, their
    // maximums, the large runs' minimums, their maximums.
    maniMatchTagBounds = 8,
    // The float tracks' names, UTF-8, end to end.
    maniMatchTagNames = 9,
    // The mirror map's identity (u64) when an entry is mirrored, else
    // nothing.
    maniMatchTagMirror = 10,
};

// Every flag an entry may carry.
#define MANI_MATCH_ENTRY_FLAGS (MANI_MATCH_LOOP | MANI_MATCH_NO_JUMP | MANI_MATCH_MIRRORED)

#define MANI_MATCH_HEADER_SIZE  48u
#define MANI_MATCH_CHANNEL_SIZE 40u
#define MANI_MATCH_ENTRY_SIZE   32u

// Holden's runs: 16 frames a small bound, 64 a large one.
#define MANI_MATCH_SMALL_RUN 16u
#define MANI_MATCH_LARGE_RUN 64u

// The runs of n frames covering frames frames.
static inline uint32_t maniMatchRuns(uint32_t frames, uint32_t n)
{
    return frames / n + (frames % n != 0 ? 1u : 0u);
}

// The features a channel of a kind holds at one time.
uint32_t maniMatchChannelWidth(uint32_t kind);

// The frames an entry from start to end holds at rate frames a second:
// the count of k with start + k / rate at most end (before it when
// looping) in binary64, then one less when the last frame's time
// (maniMatchFrameTime, binary32) does not hold, or one more when the
// next one's does.
uint32_t maniMatchEntryFrames(float start, float end, float rate, bool loop);

// An entry's k-th frame's clip time: start + k / rate, in binary64.
float maniMatchFrameTime(float start, uint32_t k, float rate);

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniMatchComputeIdentity(const uint8_t* const data[MANI_MATCH_SECTIONS],
                                  const uint64_t sizes[MANI_MATCH_SECTIONS]);

#endif // MAUL_ANIM_SRC_MATCH_FILE_H
