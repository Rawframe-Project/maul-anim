// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The skeleton file's layout, shared by its view and its
// builder: kind 1 of the container, its section tags and record sizes.

#ifndef MAUL_ANIM_SRC_SKELETON_FILE_H
#define MAUL_ANIM_SRC_SKELETON_FILE_H

#include <stdint.h>

#define MANI_SKELETON_FORMAT_VERSION 1u

enum
{
    // u16 per bone: the parent, or MANI_NO_BONE.
    maniSkeletonTagParents = 1,
    // u16 per bone: the end of the bone's subtree, exclusive.
    maniSkeletonTagSubtreeEnds = 2,
    // Ten binary32 per bone: translation, rotation (x, y, z, w), scale.
    maniSkeletonTagRests = 3,
    // u64 per bone: maniHash64 of the name from MANI_HASH_INIT.
    maniSkeletonTagNameHashes = 4,
    // u16 per bone: the bones by increasing name hash.
    maniSkeletonTagNameOrder = 5,
    // u32 offset and u32 length per bone, into the name bytes.
    maniSkeletonTagNameSpans = 6,
    // The names' UTF-8 bytes, in bone order, end to end.
    maniSkeletonTagNameBytes = 7,
};

#define MANI_SKELETON_REST_SIZE 40u
#define MANI_SKELETON_SPAN_SIZE 8u
#define MANI_SKELETON_SECTIONS  7u

// The identity: maniHash64 over the bone count (u32), the parents, the
// rests, the name spans and the name bytes, in that order. The other
// sections are derived from these (subtree ends from the parents, name
// hashes and order from the names) and checked against them at open, so
// they add nothing to it; a later format version that adds a section not
// derived from these must add it to the identity.
uint64_t maniSkeletonComputeIdentity(uint32_t boneCount, const uint8_t* parents,
                                     const uint8_t* rests, const uint8_t* spans,
                                     const uint8_t* names, uint64_t nameSize);

#endif // MAUL_ANIM_SRC_SKELETON_FILE_H
