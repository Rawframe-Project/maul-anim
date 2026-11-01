// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending: poses mixed by weights, per bone through masks,
// in caller memory. Poses and differences never mix: a blend takes
// layers of one kind and writes that kind.

#ifndef MAUL_ANIM_BLEND_H
#define MAUL_ANIM_BLEND_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The threshold graphs blend with: below this total weight the rest
// fills in.
#define MANI_BLEND_THRESHOLD 1e-5f

    // A blend's layer: a pose, its weight, and optional per-bone weights
    // (a float per bone in skeleton order, NULL for 1 everywhere). Weights
    // run from 0 to 1.
    typedef struct maniBlendLayer
    {
        const maniPose* pose;
        float weight;
        const float* boneWeights;
    } maniBlendLayer;

    /// Blends layers into a pose. Each bone takes from each layer its
    /// weight times its bone weight, divided by the bone's total (or by
    /// the threshold where the total is below it, the rest taking the
    /// shortfall; a difference's rest is the identity), so the result
    /// slides into the rest as weights fall. Rotations add on the arc that
    /// agrees with the sum so far (so the layers' order counts) and are
    /// then normalized; translations and scales add linearly and never
    /// pass the largest input's magnitude by more than rounding, so
    /// inputs up to 2^127 stay finite. With no weight on a bone it takes
    /// its rest as it is.
    ///
    /// @param skeleton   The layers' and the output's skeleton.
    /// @param layers     count layers, all poses or all differences.
    /// @param count      The number of layers.
    /// @param threshold  Above 0 and at most 1 (MANI_BLEND_THRESHOLD in
    ///                   graphs).
    /// @param poseOut    Receives the blend, of the layers' kind; not one
    ///                   of the layers' poses.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// pose of another skeleton, layers of both kinds, out among the
    /// layers, a weight outside 0 to 1 or a threshold out of range (out
    /// unchanged).
    /// @par Thread safety
    /// Safe from any thread; out is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSkeleton_BlendPoses(const maniSkeleton* skeleton,
                                                               const maniBlendLayer* layers,
                                                               uint32_t count, float threshold,
                                                               maniPose* poseOut);

    // A mask's chain: a bone's weight, for the bone alone or for its
    // subtree.
    typedef struct maniMaskChain
    {
        uint32_t bone;
        float weight;
        bool subtree;
    } maniMaskChain;

    /// Makes per-bone weights from chains. A bone takes a chain on itself
    /// alone first, else the subtree chain on its nearest ancestor or
    /// itself, else the default; of two chains on one bone and kind the
    /// later wins.
    ///
    /// @param skeleton       The skeleton.
    /// @param chains         count chains.
    /// @param count          The number of chains.
    /// @param defaultWeight  The weight of bones no chain reaches.
    /// @param weightsOut     Receives a weight per bone in skeleton order;
    ///                       may be NULL when capacity is 0.
    /// @param capacity       The floats weightsOut holds.
    /// @param countOut       Receives the bone count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when capacity is below
    /// the bone count, with the count still written and nothing else;
    /// `mani_errorInvalid` for a NULL skeleton or chains, a NULL array with
    /// a positive capacity, a bone past the skeleton or a negative or
    /// non-finite weight.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniSkeleton_BuildMask(const maniSkeleton* skeleton,
                                                              const maniMaskChain* chains,
                                                              uint32_t count, float defaultWeight,
                                                              float* weightsOut, uint32_t capacity,
                                                              uint32_t* countOut);

    /// Writes the difference of a pose from a basis, bone by bone:
    /// `d_t = t - t_b`, `d_r = r_b^-1 r` (normalized), `d_s = s / s_b` per
    /// axis. The output may be the pose or the basis.
    ///
    /// @param skeleton       The poses' skeleton.
    /// @param pose           A pose of the skeleton.
    /// @param basis          A pose of the skeleton.
    /// @param differenceOut  Receives the difference.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// pose of another skeleton, a difference given as pose or basis, or a
    /// difference not finite (a basis scale component of 0 or near it,
    /// translations past binary32's range apart; out unchanged).
    /// @par Thread safety
    /// Safe from any thread; out is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult
    maniSkeleton_ComputePoseDifference(const maniSkeleton* skeleton, const maniPose* pose,
                                       const maniPose* basis, maniPose* differenceOut);

    /// Applies differences to a pose in order, each bone at its layer
    /// weight times its bone weight (both 0 to 1): `t + w d_t`,
    /// `r nlerp(1, d_r, w)` (the difference's turn taken the short way),
    /// `s (1 + w (d_s - 1))` per axis, a factor between 1 and `d_s`.
    ///
    /// @param pose      A pose of the skeleton, changed in place.
    /// @param skeleton  The poses' skeleton.
    /// @param layers    count layers, each a difference.
    /// @param count     The number of layers.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// pose of another skeleton, a difference as the pose, a layer not a
    /// difference, or a weight outside 0 to 1 (the pose unchanged).
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_AddLayers(maniPose* pose,
                                                          const maniSkeleton* skeleton,
                                                          const maniBlendLayer* layers,
                                                          uint32_t count);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_BLEND_H
