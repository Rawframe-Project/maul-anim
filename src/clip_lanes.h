// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The forward cursor's step over one group of MANI_POSE_GROUP_BONES
// bones (mani-0019, mani-0038): two held samples in the pose's group
// layout interpolated at alpha into the pose, with the walk's operations
// in its order (clip_compressed.c's Channel and Nlerp), so a pose is
// random access's bits. A lane whose channel the clip leaves at rest
// takes the held value as it is, as the walk writes a rest.

#ifndef MAUL_ANIM_SRC_CLIP_LANES_H
#define MAUL_ANIM_SRC_CLIP_LANES_H

#include "blend_lanes.h"
#include "pose_memory.h"
#include "simd.h"

// The forward cursor holds its samples in groups on a vector backend
// only: on the scalar one, the walk over the held samples takes fewer
// instructions (mani-0039).
#if !defined(MANI_SIMD_SCALAR)
#define MANI_CLIP_LANES 1

// masks holds three rows of eight a group (translation, rotation,
// scale): all bits set where the bone's channel is interpolated, clear
// where it is a rest.
static inline void maniClipLanesInterpolate(const float* a, const float* b, const float* masks,
                                            float alpha, float* out)
{
    const unsigned lanes = MANI_POSE_GROUP_BONES;
    manif8 t = maniF8Set1(alpha);
    for (unsigned c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        if (c >= 3 && c < 7)
        {
            continue;
        }
        manif8 va = maniF8Load(a + c * lanes);
        manif8 vb = maniF8Load(b + c * lanes);
        manif8 lerp = maniF8Add(va, maniF8Mul(maniF8Sub(vb, va), t));
        manif8 mask = maniF8Load(masks + (c < 3 ? 0 : 2) * lanes);
        maniF8Store(out + c * lanes, maniF8Select(mask, lerp, va));
    }
    manif8 ra[4];
    manif8 rb[4];
    for (unsigned k = 0; k < 4; ++k)
    {
        ra[k] = maniF8Load(a + (3 + k) * lanes);
        rb[k] = maniF8Load(b + (3 + k) * lanes);
    }
    manif8 dot = maniF8Mul(ra[0], rb[0]);
    dot = maniF8Add(dot, maniF8Mul(ra[1], rb[1]));
    dot = maniF8Add(dot, maniF8Mul(ra[2], rb[2]));
    dot = maniF8Add(dot, maniF8Mul(ra[3], rb[3]));
    manif8 s = maniF8Select(maniF8LT(dot, maniF8Zero()), maniF8Set1(-1.0f), maniF8Set1(1.0f));
    // The rotation's rows of a scratch group, normalized as the codec
    // normalizes (the blend's kernel is the same operations).
    float group[MANI_POSE_COMPONENTS * MANI_POSE_GROUP_BONES];
    for (unsigned k = 0; k < 4; ++k)
    {
        manif8 v = maniF8Add(ra[k], maniF8Mul(maniF8Sub(maniF8Mul(s, rb[k]), ra[k]), t));
        maniF8Store(group + (3 + k) * lanes, v);
    }
    maniBlendLanesNormalize(group);
    manif8 mask = maniF8Load(masks + lanes);
    for (unsigned k = 0; k < 4; ++k)
    {
        manif8 q = maniF8Load(group + (3 + k) * lanes);
        maniF8Store(out + (3 + k) * lanes, maniF8Select(mask, q, ra[k]));
    }
}
#endif

#endif // MAUL_ANIM_SRC_CLIP_LANES_H
