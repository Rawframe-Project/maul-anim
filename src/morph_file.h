// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The morph targets file's layout (mani-0021), shared by its view and its
// builder: kind 7 of the container.

#ifndef MAUL_ANIM_SRC_MORPH_FILE_H
#define MAUL_ANIM_SRC_MORPH_FILE_H

#include <stdint.h>

#define MANI_MORPH_FORMAT_VERSION 1u
#define MANI_MORPH_SECTIONS       6u
// The vertex, channel and target counts, zero (u32 each).
#define MANI_MORPH_HEADER_SIZE 16u
// A channel: its first target, target count, name offset and name length
// (u32 each).
#define MANI_MORPH_CHANNEL_SIZE 16u
// A target: full weight (binary32), attributes (u32), vertex count (u32),
// zero (u32), first index (u64, into the indices), first delta (u64,
// floats into the deltas).
#define MANI_MORPH_TARGET_SIZE 32u

// A target's attributes: which deltas it holds, and whether it is dense
// (every vertex, no indices).
enum
{
    maniMorphPositions = 1u,
    maniMorphNormals = 2u,
    maniMorphTangents = 4u,
    maniMorphDense = 8u,
};

enum
{
    maniMorphTagHeader = 1,
    maniMorphTagChannels = 2,
    maniMorphTagTargets = 3,
    // u32 vertex indices of the sparse targets, each target's strictly
    // increasing; absent when every target is dense.
    maniMorphTagIndices = 4,
    // binary32 deltas, target by target: its positions, normals and
    // tangents (those it holds), three floats a vertex; absent when no
    // target holds any.
    maniMorphTagDeltas = 5,
    // The channels' names, UTF-8, end to end.
    maniMorphTagNames = 6,
};

// The floats a target holds a vertex: three an attribute.
static inline uint32_t maniMorphFloatsPerVertex(uint32_t attributes)
{
    return 3u * ((attributes & 1u) + ((attributes >> 1) & 1u) + ((attributes >> 2) & 1u));
}

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniMorphComputeIdentity(const uint8_t* const data[MANI_MORPH_SECTIONS],
                                  const uint64_t sizes[MANI_MORPH_SECTIONS]);

#endif // MAUL_ANIM_SRC_MORPH_FILE_H
