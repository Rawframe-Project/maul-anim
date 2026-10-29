// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skins: the bones a mesh is bound to, in the mesh's order,
// each with its inverse bind matrix, read in place from a skin file and
// bound to one skeleton by its identity; and the palettes a renderer
// skins with.

#ifndef MAUL_ANIM_SKIN_H
#define MAUL_ANIM_SKIN_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A skin over a file's bytes, which the caller keeps alive and
    // unchanged while the view is used. Its fields are the library's; read
    // a skin through the functions below. A view is immutable and
    // copyable.
    typedef struct maniSkin
    {
        const uint8_t* joints;
        const uint8_t* inverseBinds;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint32_t skeletonBoneCount;
        uint32_t jointCount;
    } maniSkin;

    /// Opens a skin file in place: checks the container, the sizes, every
    /// joint against the skeleton's bone count the file records, and every
    /// inverse bind finite.
    ///
    /// @param bytes    The file, 16-byte aligned.
    /// @param size     Its size in bytes; bytes past the file are ignored.
    /// @param skinOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for any other format version.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenSkin(const void* bytes, size_t size,
                                                        maniSkin* skinOut);

    /// Returns the number of joints, at least 1.
    ///
    /// @param skin  An opened skin.
    /// @return The joint count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSkin_GetJointCount(const maniSkin* skin);

    /// Returns the identity of the skeleton the skin is bound to.
    ///
    /// @param skin  An opened skin.
    /// @return The skeleton's identity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSkin_GetSkeletonIdentity(const maniSkin* skin);

    /// Returns the skin's own identity, which influences record to bind
    /// to it.
    ///
    /// @param skin  An opened skin.
    /// @return The identity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSkin_GetIdentity(const maniSkin* skin);

    /// Returns the skeleton bone of a joint.
    ///
    /// @param skin   An opened skin.
    /// @param joint  The joint.
    /// @return The bone, or MANI_NO_BONE for a joint past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSkin_GetJoint(const maniSkin* skin, uint32_t joint);

    /// Returns a joint's inverse bind matrix.
    ///
    /// @param skin   An opened skin.
    /// @param joint  The joint.
    /// @return The matrix, or the identity for a joint past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniAffine maniSkin_GetInverseBind(const maniSkin* skin, uint32_t joint);

    /// Computes a skin's palette: entry j is the model matrix of joint j's
    /// bone times the joint's inverse bind matrix.
    ///
    /// @param pose      A pose of the skeleton with its model composed.
    /// @param skeleton  The skeleton the skin is bound to.
    /// @param skin      The skin.
    /// @param palette   Receives the entries; may be NULL when capacity is
    ///                  0.
    /// @param capacity  The array's length.
    /// @param countOut  Receives the joint count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when the array is
    /// shorter than the joint count, with the count written and nothing
    /// else; `mani_errorInvalid` for a NULL argument, a skeleton that is
    /// not the skin's or the pose's, a pose whose model is stale, or a
    /// difference.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_ComputePalette(
        const maniPose* pose, const maniSkeleton* skeleton, const maniSkin* skin,
        maniAffine* palette, uint32_t capacity, uint32_t* countOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SKIN_H
