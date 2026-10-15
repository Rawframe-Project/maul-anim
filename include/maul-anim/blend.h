// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending (mani-0020): poses mixed by weights, per bone through masks,
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
    // are finite and at least 0; above 1 is allowed.
    typedef struct maniBlendLayer
    {
        const maniPose* pose;
        float weight;
        const float* boneWeights;
    } maniBlendLayer;

    /// Blends layers into a pose. Each bone takes from each layer its
    /// weight times its bone weight; rotations add on the arc that agrees
    /// with the sum so far (so the layers' order counts), translations and
    /// scales linearly. Where a bone's total is below the threshold, its
    /// rest fills the difference (a difference's rest is the identity),
    /// so the result slides into the rest as weights fall. Rotations are
    /// then normalized, translations and scales divided by the total.
    /// With no layers the output is the skeleton's rest.
    ///
    /// @param skeleton   The layers' and the output's skeleton.
    /// @param layers     count layers, all poses or all differences.
    /// @param count      The number of layers.
    /// @param threshold  Finite and above 0 (MANI_BLEND_THRESHOLD in
    ///                   graphs).
    /// @param out        Receives the blend, of the layers' kind; not one
    ///                   of the layers' poses.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// pose of another skeleton, layers of both kinds, out among the
    /// layers, a negative or non-finite weight or a threshold out of range
    /// (out unchanged).
    /// @par Thread safety
    /// Safe from any thread; out is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniBlendPoses(const maniSkeleton* skeleton,
                                                      const maniBlendLayer* layers, uint32_t count,
                                                      float threshold, maniPose* out);

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
    /// @param weights        Receives a weight per bone in skeleton order.
    /// @param capacity       The floats weights holds.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// bone past the skeleton or a negative or non-finite weight;
    /// `mani_errorCapacity` when capacity is below the bone count (weights
    /// unchanged).
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniBuildMask(const maniSkeleton* skeleton,
                                                     const maniMaskChain* chains, uint32_t count,
                                                     float defaultWeight, float* weights,
                                                     uint32_t capacity);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_BLEND_H
