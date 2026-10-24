// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motion database file's layout, shared by its view and
// its builder: kind 8 of the container. Records are little-endian;
// features are binary32, normalized, frame by frame in the search tree's
// order.
//
// The tree's shape follows from the frame count and the leaf size alone:
// a node holds a range of positions; one of more than the leaf size
// splits into its first half (rounded down) and the rest; nodes are
// numbered depth first, a node before its first half, its first half
// before the rest. The file gives which frame sits at each position;
// each node's box is the exact minimum and maximum of its frames'
// features (a leaf's over its positions in order, any other's from its
// halves'), its lowest frame and the least cost bias of its frames'
// entries.

#ifndef MAUL_ANIM_SRC_MATCH_FILE_H
#define MAUL_ANIM_SRC_MATCH_FILE_H

#include <stdbool.h>
#include <stdint.h>

#define MANI_MATCH_FORMAT_VERSION 3u
#define MANI_MATCH_SECTIONS       13u

enum
{
    // The skeleton's identity (u64), then u32 each: its bone count, the
    // frame, feature, channel, time, entry and clip counts, the sample
    // rate (binary32), the up and forward axes, the leaf size and the
    // node count.
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
    // A binary32 per feature per position, normalized.
    maniMatchTagFeatures = 7,
    // A binary32 per feature per node, node by node: its minimums, then
    // its maximums.
    maniMatchTagBoxes = 8,
    // The float tracks' names, UTF-8, end to end.
    maniMatchTagNames = 9,
    // The mirror map's identity (u64) when an entry is mirrored, else
    // nothing.
    maniMatchTagMirror = 10,
    // MANI_MATCH_NODE_SIZE each: the node's second half (0 for a leaf),
    // its lowest frame (u32 each) and its least bias (binary32).
    maniMatchTagNodes = 11,
    // MANI_MATCH_POSITION_SIZE each: the frame at the position and its
    // entry (u32 each).
    maniMatchTagPositions = 12,
    // A u32 per frame: its position.
    maniMatchTagPlaces = 13,
};

// Every flag an entry may carry.
#define MANI_MATCH_ENTRY_FLAGS (MANI_MATCH_LOOP | MANI_MATCH_NO_JUMP | MANI_MATCH_MIRRORED)

#define MANI_MATCH_HEADER_SIZE   56u
#define MANI_MATCH_CHANNEL_SIZE  40u
#define MANI_MATCH_ENTRY_SIZE    32u
#define MANI_MATCH_NODE_SIZE     12u
#define MANI_MATCH_POSITION_SIZE 8u

// The leaf sizes a database may have, and the builder's default.
#define MANI_MATCH_DEFAULT_LEAF 16u

static inline bool maniMatchLeafValid(uint32_t leaf)
{
    return leaf == 8 || leaf == 16 || leaf == 32;
}

// A walk of the tree's shape, depth first. Halving at least 2^24 frames
// down to leaves of at least 8 takes at most 22 levels, and the walk
// holds at most one pending range a level and the one it visits.
#define MANI_MATCH_WALK_DEPTH 32u

typedef struct maniMatchWalk
{
    uint32_t first[MANI_MATCH_WALK_DEPTH];
    uint32_t count[MANI_MATCH_WALK_DEPTH];
    // The node a pending range is the second half of; UINT32_MAX for a
    // first half or the root.
    uint32_t halfOf[MANI_MATCH_WALK_DEPTH];
    uint32_t top;
    uint32_t next;
    uint32_t leaf;
} maniMatchWalk;

// A node the walk visits: its number, its positions and the node it is
// the second half of (UINT32_MAX when none).
typedef struct maniMatchVisit
{
    uint32_t node;
    uint32_t first;
    uint32_t count;
    uint32_t halfOf;
} maniMatchVisit;

static inline void maniMatchWalkStart(maniMatchWalk* w, uint32_t frames, uint32_t leaf)
{
    w->first[0] = 0;
    w->count[0] = frames;
    w->halfOf[0] = UINT32_MAX;
    w->top = 1;
    w->next = 0;
    w->leaf = leaf;
}

// The next node, false once every one was visited.
static inline bool maniMatchWalkNext(maniMatchWalk* w, maniMatchVisit* v)
{
    if (w->top == 0)
    {
        return false;
    }
    --w->top;
    *v = (maniMatchVisit){w->next++, w->first[w->top], w->count[w->top], w->halfOf[w->top]};
    if (v->count > w->leaf)
    {
        uint32_t half = v->count / 2;
        w->first[w->top] = v->first + half;
        w->count[w->top] = v->count - half;
        w->halfOf[w->top] = v->node;
        w->first[w->top + 1] = v->first;
        w->count[w->top + 1] = half;
        w->halfOf[w->top + 1] = UINT32_MAX;
        w->top += 2;
    }
    return true;
}

// The bounds' combinations, in the order the file defines: the lesser
// and the greater, the earlier kept on a tie (-0 and +0).
static inline float maniMatchLower(float kept, float other)
{
    return other < kept ? other : kept;
}

static inline float maniMatchHigher(float kept, float other)
{
    return other > kept ? other : kept;
}

// The nodes of the tree over frames with leaves of leaf.
uint32_t maniMatchNodeCount(uint32_t frames, uint32_t leaf);

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
