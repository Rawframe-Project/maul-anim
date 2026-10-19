// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Retargeting maps (mani-0032): how poses of one skeleton (the source)
// become poses of another (the target). A map is built offline for its
// two skeletons (retargetbuild.h) and opened in place; each target bone
// takes its rotation from a source bone, from two neighbors along a
// chain, or keeps its reference, as a model-space turn from each side's
// reference pose.

#ifndef MAUL_ANIM_RETARGET_H
#define MAUL_ANIM_RETARGET_H

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // How a chain's target bones take their rotations: none (each at its
    // reference); interpolated (each source and target bone placed by
    // its distance from the chain's start over the chain's length, both
    // from the reference poses, and each target bone turned by the
    // source turns on either side of its place, slerped); one to one
    // (by order from the start, extra target bones at their reference);
    // one to one reversed (by order from the end).
    typedef uint32_t maniRetargetRotation;
    enum
    {
        maniRetargetRotationNone = 0,
        maniRetargetRotationInterpolated = 1,
        maniRetargetRotationOneToOne = 2,
        maniRetargetRotationOneToOneReversed = 3,
    };

    // A target bone's local translation: its reference's (its length
    // kept); its reference's plus the source's change from its own,
    // carried through the parents' reference frames and scaled by the
    // ratio of the pelvis heights (1 without a pelvis); or the source's
    // local translation itself, carried the same way.
    typedef uint32_t maniRetargetTranslation;
    enum
    {
        maniRetargetTranslationOwn = 0,
        maniRetargetTranslationScaled = 1,
        maniRetargetTranslationAbsolute = 2,
    };

    // An opened map: a view over the caller's bytes, which must outlive
    // it.
    typedef struct maniRetargetMap
    {
        const uint8_t* header;
        const uint8_t* sourceBones;
        const uint8_t* targetBones;
        const uint8_t* plan;
        uint64_t identity;
        uint64_t sourceIdentity;
        uint64_t targetIdentity;
        uint32_t sourceBoneCount;
        uint32_t targetBoneCount;
    } maniRetargetMap;

    /// Opens a retargeting map file in place: checks the container, the
    /// header (the skeletons' identities and bone counts, the pelvis's
    /// bones, axis and numbers), both reference poses (finite, rotations
    /// of length 1) and every target bone's plan (its kind, its source
    /// bones within the source's count, its place and weight in [0, 1],
    /// its translation and scale settings, unused words 0).
    ///
    /// @param bytes  The file, 16-byte aligned.
    /// @param size   Its size in bytes; bytes past the file are ignored.
    /// @param mapOut Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenRetargetMap(const void* bytes, size_t size,
                                                               maniRetargetMap* mapOut);

    /// Returns a map's identity: the hash of its sections.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRetargetMapIdentity(const maniRetargetMap* map);

    /// Returns the identity of the skeleton a map takes poses from.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The source skeleton's identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRetargetMapSourceIdentity(const maniRetargetMap* map);

    /// Returns the identity of the skeleton a map makes poses for.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The target skeleton's identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRetargetMapTargetIdentity(const maniRetargetMap* map);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_RETARGET_H
