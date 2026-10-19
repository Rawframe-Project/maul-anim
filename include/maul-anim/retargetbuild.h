// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building retargeting maps (mani-0032): bone pairs, chains of any
// lengths on each side, the pelvis and a reference pose per skeleton,
// checked against both skeletons and laid out as each target bone's
// plan, so the transfer at runtime only composes rotations. Offline.

#ifndef MAUL_ANIM_RETARGETBUILD_H
#define MAUL_ANIM_RETARGETBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/retarget.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A target bone and the source bone it follows, the share of the
    // source's turn it takes (0 its reference, 1 all), its translation,
    // and whether its scale is its reference's times the source's over
    // the source reference's (per axis), else its reference's.
    typedef struct maniRetargetPairSource
    {
        uint32_t target;
        uint32_t source;
        float rotationWeight;
        maniRetargetTranslation translation;
        bool scale;
    } maniRetargetPairSource;

    // A chain on each side, from a start bone to an end bone below it (at
    // most MANI_IK_CHAIN_MAX_BONES bones each), its rotation mode and
    // weight, and its bones' translation and scale as a pair's.
    typedef struct maniRetargetChainSource
    {
        uint32_t sourceStart;
        uint32_t sourceEnd;
        uint32_t targetStart;
        uint32_t targetEnd;
        maniRetargetRotation rotation;
        float rotationWeight;
        maniRetargetTranslation translation;
        bool scale;
    } maniRetargetChainSource;

    // The pelvis: a bone on each side; the up axis (model space, length
    // 1); horizontal and vertical factors (0 or more) on the ratio of
    // the target reference pelvis's height over the source's, each
    // height its model position along up; per axis of the model, a blend
    // from that scaled change toward the unscaled one (absolute) and a
    // blend toward the source pelvis's own position (toSource), each in
    // [0, 1]; the share of the pelvis's translation change and of its
    // turn it takes. Root motion scales by the same horizontal and
    // vertical ratios.
    typedef struct maniRetargetPelvisSource
    {
        uint32_t source;
        uint32_t target;
        maniVec3 up;
        float horizontal;
        float vertical;
        maniVec3 absolute;
        maniVec3 toSource;
        float translationWeight;
        float rotationWeight;
    } maniRetargetPelvisSource;

    // A map: the two skeletons; each one's reference pose as locals in
    // bone order (NULL for its rest); bone pairs; chains; the pelvis or
    // NULL; and whether each target bone none of these maps follows the
    // source bone of the same name, if there is one (weight 1, its own
    // translation and scale).
    typedef struct maniRetargetMapSource
    {
        const maniSkeleton* source;
        const maniSkeleton* target;
        const maniTransform* sourceReference;
        const maniTransform* targetReference;
        const maniRetargetPairSource* pairs;
        const maniRetargetChainSource* chains;
        const maniRetargetPelvisSource* pelvis;
        uint32_t pairCount;
        uint32_t chainCount;
        bool matchNames;
    } maniRetargetMapSource;

    /// Builds a retargeting map file (kind 4) from a source and keeps it
    /// in the builder. Each target bone is mapped once at most: by a
    /// pair, a chain, the pelvis or a name, in that order.
    ///
    /// @param builder  The builder.
    /// @param source   The map.
    /// @return `mani_success`; `mani_errorInvalid` with
    /// `maniBuildReasonNone` for a NULL builder, source or skeleton, a
    /// count without its list; `maniBuildReasonRetargetReference` (item
    /// the bone, other 0 for the source's pose and 1 for the target's: a
    /// local not finite or its rotation not of length 1 within a
    /// thousandth); `maniBuildReasonRetargetPair` (item the pair, other
    /// the target bone: a bone out of range, a target bone mapped
    /// before, a weight outside [0, 1], an unknown translation);
    /// `maniBuildReasonRetargetChain` (item the chain, other the target
    /// bone mapped before or 0: bones out of range, an end not below its
    /// start or past MANI_IK_CHAIN_MAX_BONES from it, an unknown mode or
    /// translation, a weight outside [0, 1]);
    /// `maniBuildReasonRetargetPelvis` (a bone out of range or the target
    /// mapped before, an up axis not of length 1, a factor negative or
    /// not finite, a blend or weight outside [0, 1], a reference pelvis
    /// at or below height 0); `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult
    maniBuildRetargetMap(maniBuilder* builder, const maniRetargetMapSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_RETARGETBUILD_H
