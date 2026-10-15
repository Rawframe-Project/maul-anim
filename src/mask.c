// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Masks from chains (mani-0020): subtree chains carried down the
// depth-first order, then chains on single bones over them.

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <stdbool.h>
#include <stdint.h>

// Finite and at least 0.
static bool Usable(float w)
{
    return w >= 0.0f && w <= FLT_MAX;
}

maniResult maniBuildMask(const maniSkeleton* skeleton, const maniMaskChain* chains, uint32_t count,
                         float defaultWeight, float* weights, uint32_t capacity)
{
    if (skeleton == NULL || weights == NULL || (chains == NULL && count > 0) ||
        !Usable(defaultWeight))
    {
        return mani_errorInvalid;
    }
    uint32_t bones = maniSkeletonBoneCount(skeleton);
    for (uint32_t i = 0; i < count; ++i)
    {
        if (chains[i].bone >= bones || !Usable(chains[i].weight))
        {
            return mani_errorInvalid;
        }
    }
    if (capacity < bones)
    {
        return mani_errorCapacity;
    }
    // -1 marks a bone no subtree chain starts at.
    for (uint32_t b = 0; b < bones; ++b)
    {
        weights[b] = -1.0f;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        weights[chains[i].bone] = chains[i].subtree ? chains[i].weight : weights[chains[i].bone];
    }
    // Parents come first, so each bone finds its parent's weight settled.
    for (uint32_t b = 0; b < bones; ++b)
    {
        uint32_t parent = maniSkeletonParent(skeleton, b);
        float inherited = parent == MANI_NO_BONE ? defaultWeight : weights[parent];
        weights[b] = weights[b] < 0.0f ? inherited : weights[b];
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        weights[chains[i].bone] = chains[i].subtree ? weights[chains[i].bone] : chains[i].weight;
    }
    return mani_success;
}
