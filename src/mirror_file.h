// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mirror map file's layout, shared by its view and its
// builder: kind 13 of the container. Records are little-endian. With P a
// bone's parent's reference model rotation (the identity for a root), R
// its own, m its partner and mirror(q) the quaternion with its two
// vector components off the plane's normal negated, each bone keeps
// A = P_b^-1 mirror(P_m) and B = mirror(R_m)^-1 R_b, so that its mirrored
// local is A mirror(T_m) for the translation (the normal's component
// negated) and A mirror(Q_m) B for the rotation. Its scale is the
// partner's carried by B: S' = R(B)^T S_m R(B), each axis taking the
// partner's axis the permutation names when B takes axes to axes, the
// diagonal of S' otherwise.

#ifndef MAUL_ANIM_SRC_MIRROR_FILE_H
#define MAUL_ANIM_SRC_MIRROR_FILE_H

#include <stdint.h>

#define MANI_MIRROR_FORMAT_VERSION 2u
#define MANI_MIRROR_SECTIONS       4u

enum
{
    // The header: u64 the skeleton's identity; u32 its bone count, the
    // plane's normal axis (0 x, 1 y, 2 z), the name pair count, the first
    // bone whose parent's partner is not its partner's parent
    // (MANI_NO_BONE for none); two 0 words.
    maniMirrorTagHeader = 1,
    // MANI_MIRROR_BONE_SIZE a bone: u32 its partner, its flags; A and B
    // (4 binary32 each).
    maniMirrorTagBones = 2,
    // MANI_MIRROR_NAME_SIZE a name pair, by increasing hash of the name:
    // u64 the name's hash, the partner's hash; u32 the partner's offset
    // and length in the names.
    maniMirrorTagNames = 3,
    // The partners' names, UTF-8.
    maniMirrorTagStrings = 4,
};

// A bone's flags word: each axis i's scale source p(i) XOR i in bits 2i
// and 2i + 1 (0 the identity); MANI_MIRROR_FLAG_APPROXIMATE when B takes
// no axis to an axis (the permutation bits 0); MANI_MIRROR_FLAG_UNMATCHED
// when the bone's parent's partner is not its partner's parent.
#define MANI_MIRROR_PERMUTATION      0x3Fu
#define MANI_MIRROR_FLAG_APPROXIMATE 0x100u
#define MANI_MIRROR_FLAG_UNMATCHED   0x200u

#define MANI_MIRROR_HEADER_SIZE 32u
#define MANI_MIRROR_BONE_SIZE   40u
#define MANI_MIRROR_NAME_SIZE   24u

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniMirrorComputeIdentity(const uint8_t* const data[MANI_MIRROR_SECTIONS],
                                   const uint64_t sizes[MANI_MIRROR_SECTIONS]);

#endif // MAUL_ANIM_SRC_MIRROR_FILE_H
