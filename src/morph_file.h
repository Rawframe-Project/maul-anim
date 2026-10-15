// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The morph targets file's layout (mani-0021), shared by its view and its
// builder: kind 7 of the container.

#ifndef MAUL_ANIM_SRC_MORPH_FILE_H
#define MAUL_ANIM_SRC_MORPH_FILE_H

#include "maul-anim/morph.h"

#include <stdint.h>

#define MANI_MORPH_FORMAT_VERSION 1u
#define MANI_MORPH_SECTIONS       7u
// The vertex, channel, target and attribute counts (u32 each).
#define MANI_MORPH_HEADER_SIZE 16u
// An attribute: its semantic and set (u32 each).
#define MANI_MORPH_ATTRIBUTE_SIZE 8u
// A channel: its first target, target count, name offset and name length
// (u32 each).
#define MANI_MORPH_CHANNEL_SIZE 16u
// A target: full weight (binary32), the attributes it holds (a u32 mask
// over the table), vertex count (u32), flags (u32), first index (u64,
// into the indices), first delta (u64, floats into the deltas), name
// offset and name length (u32 each).
#define MANI_MORPH_TARGET_SIZE 40u

// A target's flags: dense (every vertex, no indices).
#define MANI_MORPH_DENSE 1u

enum
{
    maniMorphTagHeader = 1,
    maniMorphTagAttributes = 2,
    maniMorphTagChannels = 3,
    maniMorphTagTargets = 4,
    // u32 vertex indices of the sparse targets, each target's strictly
    // increasing; absent when no sparse target has any.
    maniMorphTagIndices = 5,
    // binary32 deltas, target by target: for each attribute it holds, in
    // table order, its width of floats a vertex; absent when none.
    maniMorphTagDeltas = 6,
    // The channels' names, then the targets' names, UTF-8, end to end.
    maniMorphTagNames = 7,
};

// The floats a delta of the semantic holds: 3 for positions, normals and
// tangents (no w), 2 for texture coordinates, 4 for colors; 0 for none.
static inline uint32_t maniMorphWidth(uint32_t semantic)
{
    return semantic <= maniMorphTangent
               ? 3u
               : (semantic == maniMorphTexcoord ? 2u : (semantic == maniMorphColor ? 4u : 0u));
}

// The floats a target holds a vertex: its attributes' widths, from the
// table's widths.
static inline uint32_t maniMorphFloatsPerVertex(const uint32_t* widths, uint32_t mask)
{
    uint32_t floats = 0;
    for (uint32_t a = 0; a < MANI_MORPH_MAX_ATTRIBUTES; ++a)
    {
        floats += ((mask >> a) & 1u) != 0 ? widths[a] : 0u;
    }
    return floats;
}

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniMorphComputeIdentity(const uint8_t* const data[MANI_MORPH_SECTIONS],
                                  const uint64_t sizes[MANI_MORPH_SECTIONS]);

#endif // MAUL_ANIM_SRC_MORPH_FILE_H
