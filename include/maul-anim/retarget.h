// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Retargeting maps: how poses of one skeleton (the source)
// become poses of another (the target). A map is built offline for its
// two skeletons (retargetbuild.h) and opened in place; each target bone
// takes its rotation from a source bone, from two neighbors along a
// chain, or keeps its reference, as a model-space turn from each side's
// reference pose.

#ifndef MAUL_ANIM_RETARGET_H
#define MAUL_ANIM_RETARGET_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

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
    /// byte found wrong; `mani_errorVersion` for any other format version.
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
    MANI_API uint64_t maniRetargetMap_GetIdentity(const maniRetargetMap* map);

    /// Returns the identity of the skeleton a map takes poses from.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The source skeleton's identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRetargetMap_GetSourceIdentity(const maniRetargetMap* map);

    /// Returns the identity of the skeleton a map makes poses for.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The target skeleton's identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRetargetMap_GetTargetIdentity(const maniRetargetMap* map);

    /// Returns the scratch maniRetargetMap_RetargetPose takes with a map: a rotation
    /// for every bone of both skeletons.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The bytes; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniRetargetMap_GetScratchSize(const maniRetargetMap* map);

    /// Makes a pose of the target from a pose of the source through a
    /// map. Each bone's model rotation is the product of its root path's
    /// local rotations. A mapped target bone's model rotation is the
    /// source bone's times the inverse of the source's reference model
    /// rotation (two source bones' such turns slerped at the bone's
    /// place), slerped from no turn by its weight, times the target's
    /// reference model rotation; its local rotation is taken against its
    /// parent's as just made. An unmapped bone keeps its reference
    /// local. Translations are the target reference's, or the source's
    /// change from its reference (scaled by the pelvis height ratio) or
    /// the source's own, turned from the source parent's reference
    /// model frame to the target parent's; a bone whose scale carries
    /// takes its reference's times the source's over the source
    /// reference's, per axis. The pelvis's model position is the target
    /// reference's plus the source pelvis's change from its reference,
    /// its horizontal and vertical parts scaled by the map's scales,
    /// blended per model axis toward the unscaled change and then
    /// toward the source pelvis's position, weighted from the reference.
    /// Writes every local of the target pose and leaves its model stale.
    ///
    /// @param map          An opened map.
    /// @param source       The map's source skeleton.
    /// @param sourcePose   A pose of the source (not a difference).
    /// @param target       The map's target skeleton.
    /// @param targetPose   A pose of the target; its locals are written.
    /// @param scratch      Scratch memory, 16-byte aligned.
    /// @param scratchSize  Its size: at least maniRetargetMap_GetScratchSize.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton whose identity is not the map's, a pose of another
    /// skeleton, a source pose of differences, misaligned or too little
    /// scratch.
    /// @par Thread safety
    /// Safe from any thread; the target pose and the scratch are used by
    /// one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniRetargetMap_RetargetPose(
        const maniRetargetMap* map, const maniSkeleton* source, const maniPose* sourcePose,
        const maniSkeleton* target, maniPose* targetPose, void* scratch, size_t scratchSize);

    /// Scales a root motion delta (model space) of the source for the
    /// target: its part along the map's up axis by the vertical scale,
    /// the rest by the horizontal scale (both 1 without a pelvis).
    ///
    /// @param map    An opened map, or NULL (the delta unchanged).
    /// @param delta  The source's root motion translation.
    /// @return The target's.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniVec3 maniRetargetMap_RetargetRootMotion(const maniRetargetMap* map,
                                                         maniVec3 delta);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_RETARGET_H
