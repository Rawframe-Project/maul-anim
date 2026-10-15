// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending (mani-0020): each group of eight bones summed over the layers
// in their order, the rest filling in under the threshold, then
// normalized. Binary32 operations in a fixed order, so the same bits on
// every platform.

#include "maul-anim/blend.h"

#include "pose_memory.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LANES      MANI_POSE_GROUP_BONES
#define COMPONENTS MANI_POSE_COMPONENTS

// A group's running sums: each component of its eight bones, their
// total weights, and which bones took their rest unweighted.
typedef struct Sums
{
    float c[COMPONENTS][LANES];
    float total[LANES];
    bool rest[LANES];
} Sums;

// Finite and at least 0.
static bool Usable(float w)
{
    return w >= 0.0f && w <= FLT_MAX;
}

static bool LayerValid(const maniBlendLayer* layer, const maniSkeleton* skeleton,
                       const maniPose* out, bool difference)
{
    if (!maniPoseMatches(layer->pose, skeleton) || layer->pose == out || !Usable(layer->weight) ||
        maniPoseIsDifference(layer->pose) != difference)
    {
        return false;
    }
    for (uint32_t b = 0; layer->boneWeights != NULL && b < skeleton->boneCount; ++b)
    {
        if (!Usable(layer->boneWeights[b]))
        {
            return false;
        }
    }
    return true;
}

// The arguments checked before anything is written; the layers' kind out.
static bool Valid(const maniSkeleton* skeleton, const maniBlendLayer* layers, uint32_t count,
                  float threshold, const maniPose* out, bool* difference)
{
    if (!maniPoseMatches(out, skeleton) || (layers == NULL && count > 0) || !Usable(threshold) ||
        !(threshold > 0.0f))
    {
        return false;
    }
    *difference = count > 0 && layers[0].pose != NULL && maniPoseIsDifference(layers[0].pose);
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!LayerValid(&layers[i], skeleton, out, *difference))
        {
            return false;
        }
    }
    return true;
}

// One layer's eight weights from first: its weight times each bone's,
// 0 past the bones.
static void GroupWeights(const maniBlendLayer* layer, uint32_t first, uint32_t lanes,
                         float w[LANES])
{
    for (uint32_t k = 0; k < LANES; ++k)
    {
        float bone = layer->boneWeights != NULL && k < lanes ? layer->boneWeights[first + k] : 1.0f;
        w[k] = k < lanes ? layer->weight * bone : 0.0f;
    }
}

// Adds a group of eight transforms at their weights: a rotation negated
// where it points away from the sum so far.
static void Accumulate(Sums* s, const float* in, const float w[LANES])
{
    float rw[LANES];
    for (uint32_t k = 0; k < LANES; ++k)
    {
        float dot = ((s->c[3][k] * in[3 * LANES + k] + s->c[4][k] * in[4 * LANES + k]) +
                     s->c[5][k] * in[5 * LANES + k]) +
                    s->c[6][k] * in[6 * LANES + k];
        rw[k] = dot < 0.0f ? -w[k] : w[k];
        s->total[k] += w[k];
    }
    for (uint32_t c = 0; c < COMPONENTS; ++c)
    {
        const float* weights = c >= 3 && c < 7 ? rw : w;
        for (uint32_t k = 0; k < LANES; ++k)
        {
            s->c[c][k] += in[c * LANES + k] * weights[k];
        }
    }
}

// A bone's rest as its ten components: the skeleton's, or for a
// difference the identity.
static void Rest(const maniSkeleton* skeleton, uint32_t bone, bool difference, float v[COMPONENTS])
{
    maniTransform t =
        difference
            ? (maniTransform){{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}
            : maniSkeletonRest(skeleton, bone);
    const float values[COMPONENTS] = {
        t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y,
        t.rotation.z,    t.rotation.w,    t.scale.x,       t.scale.y,    t.scale.z};
    for (uint32_t c = 0; c < COMPONENTS; ++c)
    {
        v[c] = values[c];
    }
}

// The rest into a bone's lane under the threshold: copied where nothing
// weighed, added at the shortfall otherwise.
static void FillRest(Sums* s, const maniSkeleton* skeleton, uint32_t first, uint32_t lanes,
                     float threshold, bool difference)
{
    for (uint32_t k = 0; k < lanes; ++k)
    {
        if (!(s->total[k] < threshold))
        {
            continue;
        }
        float rest[COMPONENTS];
        Rest(skeleton, first + k, difference, rest);
        if (s->total[k] == 0.0f)
        {
            // Nothing weighed: the rest as it is.
            for (uint32_t c = 0; c < COMPONENTS; ++c)
            {
                s->c[c][k] = rest[c];
            }
            s->rest[k] = true;
            continue;
        }
        float f = threshold - s->total[k];
        float dot = ((s->c[3][k] * rest[3] + s->c[4][k] * rest[4]) + s->c[5][k] * rest[5]) +
                    s->c[6][k] * rest[6];
        float rf = dot < 0.0f ? -f : f;
        for (uint32_t c = 0; c < COMPONENTS; ++c)
        {
            s->c[c][k] += rest[c] * (c >= 3 && c < 7 ? rf : f);
        }
        s->total[k] = threshold;
    }
}

// Normalized into the output group: rotations to unit length (the
// identity for a zero sum), the rest divided by the total.
static void Store(const Sums* s, uint32_t lanes, float* out)
{
    for (uint32_t k = 0; k < lanes; ++k)
    {
        if (s->rest[k])
        {
            for (uint32_t c = 0; c < COMPONENTS; ++c)
            {
                out[c * LANES + k] = s->c[c][k];
            }
            continue;
        }
        float x = s->c[3][k];
        float y = s->c[4][k];
        float z = s->c[5][k];
        float w = s->c[6][k];
        float squared = ((x * x + y * y) + z * z) + w * w;
        bool usable = squared > 0.0f && squared <= FLT_MAX;
        float inverse = usable ? 1.0f / sqrtf(squared) : 0.0f;
        out[3 * LANES + k] = usable ? x * inverse : 0.0f;
        out[4 * LANES + k] = usable ? y * inverse : 0.0f;
        out[5 * LANES + k] = usable ? z * inverse : 0.0f;
        out[6 * LANES + k] = usable ? w * inverse : 1.0f;
        float ratio = 1.0f / s->total[k];
        for (uint32_t c = 0; c < COMPONENTS; ++c)
        {
            out[c * LANES + k] = c >= 3 && c < 7 ? out[c * LANES + k] : s->c[c][k] * ratio;
        }
    }
}

maniResult maniBlendPoses(const maniSkeleton* skeleton, const maniBlendLayer* layers,
                          uint32_t count, float threshold, maniPose* out)
{
    bool difference = false;
    if (!Valid(skeleton, layers, count, threshold, out, &difference))
    {
        return mani_errorInvalid;
    }
    float* locals = maniPoseLocals(out);
    uint32_t bones = skeleton->boneCount;
    for (uint32_t first = 0; first < bones; first += LANES)
    {
        uint32_t lanes = bones - first < LANES ? bones - first : LANES;
        size_t group = (size_t)first * COMPONENTS;
        Sums s = {0};
        for (uint32_t i = 0; i < count; ++i)
        {
            if (layers[i].weight == 0.0f)
            {
                continue;
            }
            float w[LANES];
            GroupWeights(&layers[i], first, lanes, w);
            Accumulate(&s, maniPoseLocalsRead(layers[i].pose) + group, w);
        }
        FillRest(&s, skeleton, first, lanes, threshold, difference);
        Store(&s, lanes, locals + group);
    }
    maniPoseSetKind(out, difference);
    maniPoseMarkStale(out);
    return mani_success;
}
