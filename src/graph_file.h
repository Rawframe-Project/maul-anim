// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The graph file's layout (mani-0023), shared by its view and its
// builder: kind 3 of the container. Records are little-endian; names are
// offsets into the names section; nodes are in topological order, each
// after its inputs, the output last.

#ifndef MAUL_ANIM_SRC_GRAPH_FILE_H
#define MAUL_ANIM_SRC_GRAPH_FILE_H

#include <stdint.h>

#define MANI_GRAPH_FORMAT_VERSION 1u
#define MANI_GRAPH_SECTIONS       9u

enum
{
    // The skeleton's identity (u64), then u32 each: its bone count, the
    // parameter, node, input, data word, clip, sync group, mask and slot
    // counts, and 0.
    maniGraphTagHeader = 1,
    // MANI_GRAPH_PARAMETER_SIZE each: type, flags (bit 0 bounded), name
    // offset and length (u32 each), name hash (u64), default (16 bytes),
    // minimum and maximum (4 bytes each, for int and float).
    maniGraphTagParameters = 2,
    // MANI_GRAPH_NODE_SIZE each: kind (u16), flags (u16, bit 0 gives
    // differences), first input, input count, first data word, data word
    // count, slot (u32 each), zero (8 bytes).
    maniGraphTagNodes = 3,
    // u32 each: a node before its reader.
    maniGraphTagInputs = 4,
    // u32 words, per kind (MANI_GRAPH_*_WORDS); a scalar is a parameter
    // index (MANI_NO_PARAMETER for a literal) and a binary32's bits.
    maniGraphTagData = 5,
    // MANI_GRAPH_CLIP_SIZE each: identity, name hash (u64 each), name
    // offset and length, flags (bit 0 differences), zero (u32 each).
    maniGraphTagClips = 6,
    // MANI_GRAPH_SYNC_GROUP_SIZE each: name hash (u64), name offset and
    // length, flags, zero (u32 each).
    maniGraphTagSyncGroups = 7,
    // A binary32 per bone per mask, 0 to 1.
    maniGraphTagMaskWeights = 8,
    // The names' bytes, UTF-8.
    maniGraphTagNames = 9,
};

#define MANI_GRAPH_HEADER_SIZE     48u
#define MANI_GRAPH_PARAMETER_SIZE  48u
#define MANI_GRAPH_NODE_SIZE       32u
#define MANI_GRAPH_CLIP_SIZE       32u
#define MANI_GRAPH_SYNC_GROUP_SIZE 24u

// A parameter's and a node's flags.
#define MANI_GRAPH_BOUNDED    1u
#define MANI_GRAPH_DIFFERENCE 1u

// Node data words. A clip node: slot, sync group, loop, flags, speed (a
// scalar). A blend: direct, then a scalar per input. A mask: its mask. An
// additive node: a scalar per layer. A select: its parameter. A time
// scale: the scale. A seek: the trigger, then the time.
#define MANI_GRAPH_CLIP_WORDS       6u
#define MANI_GRAPH_MASK_WORDS       1u
#define MANI_GRAPH_SELECT_WORDS     1u
#define MANI_GRAPH_TIME_SCALE_WORDS 2u
#define MANI_GRAPH_SEEK_WORDS       3u

// The data words a node of a kind with n inputs holds; 0 for an unknown
// kind.
uint32_t maniGraphDataWords(uint32_t kind, uint32_t inputCount);

// The inputs a kind takes: at least minOut, at most the return (0 with
// minOut 1 for an unknown kind).
uint32_t maniGraphInputRange(uint32_t kind, uint32_t* minOut);

// The identity: maniHash64 over every section in tag order, each
// continuing the last.
uint64_t maniGraphComputeIdentity(const uint8_t* const* sections, const uint64_t* sizes);

#endif // MAUL_ANIM_SRC_GRAPH_FILE_H
