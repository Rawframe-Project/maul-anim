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
#include "maul-anim/clip.h"
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

    // A clip to retarget: an opened map, its two skeletons and an opened
    // clip of its source; and the target bone that takes the clip's
    // root motion, or MANI_NO_BONE for the first target bone that
    // follows the root motion bone alone.
    typedef struct maniRetargetClipSource
    {
        const maniRetargetMap* map;
        const maniSkeleton* source;
        const maniSkeleton* target;
        const maniClip* clip;
        uint32_t rootMotionBone;
        uint32_t reserved;
    } maniRetargetClipSource;

    /// Builds a full-precision clip of the target from a clip of the
    /// source through a map (bake it once, play it as any clip), and
    /// keeps it in the builder. The clip keeps the source clip's sample
    /// rate, duration, loop policy and additive basis. At each sample
    /// the source's pose is made a pose of the target by
    /// maniRetargetPose; a channel that is the target's rest at every
    /// sample is left out. An additive clip's differences are each added
    /// to the source's rest, carried, and taken as differences from the
    /// source's rest carried. Float tracks keep their values at the
    /// samples and their interpolation (linear or step); discrete
    /// tracks, events and markers are copied. Root motion carries as
    /// moves: each translation's horizontal and vertical parts scaled as
    /// maniRetargetRootMotion scales them, both turned from the root
    /// motion bone's parent's reference frame to the target bone's
    /// parent's; its source's axes follow that turn (all axes, and the
    /// whole rotation, where it takes an axis off the axes).
    ///
    /// @param builder  The builder.
    /// @param source   The clip to retarget.
    /// @return `mani_success`; `mani_errorInvalid` with
    /// `maniBuildReasonNone` for a NULL builder, source or field, or
    /// reserved not 0; `maniBuildReasonRetargetClip` (other 0 the source
    /// skeleton, 1 the target, 2 the clip: a skeleton not the map's, a
    /// clip not of the source); `maniBuildReasonRootMotion` (item the
    /// target bone: past the target's bones, or MANI_NO_BONE where no
    /// bone follows the root motion bone alone);
    /// `maniBuildReasonBasis` for an additive clip whose difference
    /// from its basis carried is not finite (a scale component of 0 in
    /// the basis); `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult
    maniBuildRetargetedClip(maniBuilder* builder, const maniRetargetClipSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_RETARGETBUILD_H
