// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The skin file's layout, shared by its view and its builder:
// kind 5 of the container.

#ifndef MAUL_ANIM_SRC_SKIN_FILE_H
#define MAUL_ANIM_SRC_SKIN_FILE_H

#include <stdint.h>

#define MANI_SKIN_FORMAT_VERSION 1u
#define MANI_SKIN_SECTIONS       3u
// The skeleton's identity (u64), its bone count (u32), the joint count
// (u32).
#define MANI_SKIN_BINDING_SIZE 16u
#define MANI_SKIN_MATRIX_SIZE  48u

enum
{
    maniSkinTagBinding = 1,
    // u16 per joint: a skeleton bone.
    maniSkinTagJoints = 2,
    // Twelve binary32 per joint: the inverse bind, 3x4 columns.
    maniSkinTagInverseBinds = 3,
};

// The identity: maniHash64 over the binding, then the joints, then the
// inverse binds, each continuing the last; every section is in it.
uint64_t maniSkinComputeIdentity(const uint8_t* binding, const uint8_t* joints,
                                 const uint8_t* inverseBinds, uint32_t jointCount);

#endif // MAUL_ANIM_SRC_SKIN_FILE_H
