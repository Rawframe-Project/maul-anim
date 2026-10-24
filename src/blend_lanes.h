// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The blend's steps over one group of MANI_POSE_GROUP_BONES bones, on
// the family's eight-lane vectors: a group is
// MANI_POSE_COMPONENTS rows of eight floats, row c holding component c
// of each bone, and a group's weights, totals and shares are eight
// floats. Each step is the scalar lane loop's operations in its order,
// separate multiply and add only (never the header's fused one), so
// every backend gives that loop's bits. Inline, so the blend's loop
// keeps its rows in registers between steps.

#ifndef MAUL_ANIM_SRC_BLEND_LANES_H
#define MAUL_ANIM_SRC_BLEND_LANES_H

#include "pose_memory.h"
#include "simd.h"

#include <math.h>
#include <stdbool.h>

_Static_assert(MANI_POSE_GROUP_BONES == 8, "a pose group is one eight-lane vector a component");

#if defined(MANI_SIMD_SCALAR)

// The scalar backend runs the lane loops themselves: the same operations
// in the same order, in less code than the header's eight-lane emulation
// (the core's size budget, tools/size-budget.txt, holds this build).

static inline void maniBlendLanesWeights(float weight, const float* boneWeights, float* w)
{
    for (unsigned k = 0; k < MANI_POSE_GROUP_BONES; ++k)
    {
        w[k] = boneWeights != NULL ? weight * boneWeights[k] : weight;
    }
}

static inline void maniBlendLanesAdd(float* sum, const float* add)
{
    for (unsigned k = 0; k < MANI_POSE_GROUP_BONES; ++k)
    {
        sum[k] += add[k];
    }
}

static inline void maniBlendLanesScale(float* w, const float* scale)
{
    for (unsigned k = 0; k < MANI_POSE_GROUP_BONES; ++k)
    {
        w[k] *= scale[k];
    }
}

static inline void maniBlendLanesShares(const float* total, float threshold, float* inverse,
                                        float* share)
{
    for (unsigned k = 0; k < MANI_POSE_GROUP_BONES; ++k)
    {
        bool under = total[k] < threshold;
        inverse[k] = 1.0f / (under ? threshold : total[k]);
        share[k] = under && total[k] > 0.0f ? (threshold - total[k]) * inverse[k] : 0.0f;
    }
}

static inline void maniBlendLanesAccumulate(float* restrict out, const float* restrict in,
                                            const float* restrict weights, bool first)
{
    const unsigned lanes = MANI_POSE_GROUP_BONES;
    if (first)
    {
        for (unsigned i = 0; i < MANI_POSE_COMPONENTS * lanes; ++i)
        {
            out[i] = 0.0f + in[i] * weights[i % lanes];
        }
        return;
    }
    float rw[MANI_POSE_GROUP_BONES];
    for (unsigned k = 0; k < lanes; ++k)
    {
        float dot =
            ((out[3 * lanes + k] * in[3 * lanes + k] + out[4 * lanes + k] * in[4 * lanes + k]) +
             out[5 * lanes + k] * in[5 * lanes + k]) +
            out[6 * lanes + k] * in[6 * lanes + k];
        rw[k] = dot < 0.0f ? -weights[k] : weights[k];
    }
    for (unsigned c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        const float* w = c >= 3 && c < 7 ? rw : weights;
        for (unsigned k = 0; k < lanes; ++k)
        {
            out[c * lanes + k] += in[c * lanes + k] * w[k];
        }
    }
}

static inline void maniBlendLanesNormalize(float* group)
{
    const unsigned lanes = MANI_POSE_GROUP_BONES;
    for (unsigned k = 0; k < lanes; ++k)
    {
        float x = group[3 * lanes + k];
        float y = group[4 * lanes + k];
        float z = group[5 * lanes + k];
        float w = group[6 * lanes + k];
        float squared = ((x * x + y * y) + z * z) + w * w;
        bool ok = squared > 0.0f && squared < INFINITY;
        float inverse = 1.0f / sqrtf(ok ? squared : 1.0f);
        group[3 * lanes + k] = ok ? x * inverse : 0.0f;
        group[4 * lanes + k] = ok ? y * inverse : 0.0f;
        group[5 * lanes + k] = ok ? z * inverse : 0.0f;
        group[6 * lanes + k] = ok ? w * inverse : 1.0f;
    }
}

#else

// A group's eight weights: weight alone, or times each bone's eight.
static inline void maniBlendLanesWeights(float weight, const float* boneWeights, float* w)
{
    manif8 v = maniF8Set1(weight);
    maniF8Store(w, boneWeights != NULL ? maniF8Mul(v, maniF8Load(boneWeights)) : v);
}

// sum += add, lane by lane.
static inline void maniBlendLanesAdd(float* sum, const float* add)
{
    maniF8Store(sum, maniF8Add(maniF8Load(sum), maniF8Load(add)));
}

// w *= scale, lane by lane.
static inline void maniBlendLanesScale(float* w, const float* scale)
{
    maniF8Store(w, maniF8Mul(maniF8Load(w), maniF8Load(scale)));
}

// From each bone's total: the reciprocal its weights take (of the
// threshold where the total is under it) and the rest's share of the
// shortfall (0 where the total is not under the threshold or not above
// 0).
static inline void maniBlendLanesShares(const float* total, float threshold, float* inverse,
                                        float* share)
{
    manif8 t = maniF8Load(total);
    manif8 limit = maniF8Set1(threshold);
    manif8 under = maniF8LT(t, limit);
    manif8 r = maniF8Div(maniF8Set1(1.0f), maniF8Select(under, limit, t));
    manif8 takes = maniF8And(under, maniF8GT(t, maniF8Zero()));
    maniF8Store(inverse, r);
    maniF8Store(share, maniF8Select(takes, maniF8Mul(maniF8Sub(limit, t), r), maniF8Zero()));
}

// Adds a group of transforms at their weights into the output group:
// the first layer as 0 + in * w; later ones with each rotation's weight
// negated where it points away from the sum so far.
static inline void maniBlendLanesAccumulate(float* restrict out, const float* restrict in,
                                            const float* restrict weights, bool first)
{
    const unsigned lanes = MANI_POSE_GROUP_BONES;
    manif8 w = maniF8Load(weights);
    if (first)
    {
        for (unsigned c = 0; c < MANI_POSE_COMPONENTS; ++c)
        {
            manif8 v = maniF8Mul(maniF8Load(in + c * lanes), w);
            maniF8Store(out + c * lanes, maniF8Add(maniF8Zero(), v));
        }
        return;
    }
    manif8 dot = maniF8Mul(maniF8Load(out + 3 * lanes), maniF8Load(in + 3 * lanes));
    dot = maniF8Add(dot, maniF8Mul(maniF8Load(out + 4 * lanes), maniF8Load(in + 4 * lanes)));
    dot = maniF8Add(dot, maniF8Mul(maniF8Load(out + 5 * lanes), maniF8Load(in + 5 * lanes)));
    dot = maniF8Add(dot, maniF8Mul(maniF8Load(out + 6 * lanes), maniF8Load(in + 6 * lanes)));
    manif8 rw = maniF8Select(maniF8LT(dot, maniF8Zero()), maniF8Neg(w), w);
    // Translation (rows 0 to 2) and scale (7 to 9) at w, the rotation
    // (3 to 6) at rw: three runs with no test a row, which -O2 unrolls.
    for (unsigned c = 0; c < 3; ++c)
    {
        manif8 v = maniF8Mul(maniF8Load(in + c * lanes), w);
        maniF8Store(out + c * lanes, maniF8Add(maniF8Load(out + c * lanes), v));
    }
    for (unsigned c = 3; c < 7; ++c)
    {
        manif8 v = maniF8Mul(maniF8Load(in + c * lanes), rw);
        maniF8Store(out + c * lanes, maniF8Add(maniF8Load(out + c * lanes), v));
    }
    for (unsigned c = 7; c < MANI_POSE_COMPONENTS; ++c)
    {
        manif8 v = maniF8Mul(maniF8Load(in + c * lanes), w);
        maniF8Store(out + c * lanes, maniF8Add(maniF8Load(out + c * lanes), v));
    }
}

// Normalizes the group's rotations in place: to unit length where the
// squared length is positive and finite, the identity elsewhere.
static inline void maniBlendLanesNormalize(float* group)
{
    const unsigned lanes = MANI_POSE_GROUP_BONES;
    manif8 x = maniF8Load(group + 3 * lanes);
    manif8 y = maniF8Load(group + 4 * lanes);
    manif8 z = maniF8Load(group + 5 * lanes);
    manif8 w = maniF8Load(group + 6 * lanes);
    manif8 squared = maniF8Add(
        maniF8Add(maniF8Add(maniF8Mul(x, x), maniF8Mul(y, y)), maniF8Mul(z, z)), maniF8Mul(w, w));
    // Positive and at most FLT_MAX: below infinity is the same set.
    manif8 one = maniF8Set1(1.0f);
    manif8 ok = maniF8And(maniF8GT(squared, maniF8Zero()), maniF8LT(squared, maniF8Set1(INFINITY)));
    manif8 inverse = maniF8Div(one, maniF8Sqrt(maniF8Select(ok, squared, one)));
    maniF8Store(group + 3 * lanes, maniF8Select(ok, maniF8Mul(x, inverse), maniF8Zero()));
    maniF8Store(group + 4 * lanes, maniF8Select(ok, maniF8Mul(y, inverse), maniF8Zero()));
    maniF8Store(group + 5 * lanes, maniF8Select(ok, maniF8Mul(z, inverse), maniF8Zero()));
    maniF8Store(group + 6 * lanes, maniF8Select(ok, maniF8Mul(w, inverse), one));
}

#endif

#endif // MAUL_ANIM_SRC_BLEND_LANES_H
