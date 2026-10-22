// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending (mani-0020): each group of eight bones summed over the layers
// in their order at weights divided by the bone's total first (so
// translations and scales never pass the largest input), the rest
// filling in under the threshold, then rotations normalized. Binary32 operations in a fixed order,
// so the same bits on every platform.

#include "maul-anim/blend.h"

#include "blend_groups.h"
#include "blend_lanes.h"
#include "pose_memory.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LANES      MANI_POSE_GROUP_BONES
#define COMPONENTS MANI_POSE_COMPONENTS

// Between 0 and 1 (NaN fails).
static bool Usable(float w)
{
    return w >= 0.0f && w <= 1.0f;
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

// One layer's eight weights from first: its weight times each bone's
// (padding lanes take the layer's; Settle rewrites them).
static void GroupWeights(const maniBlendLayer* layer, uint32_t first, uint32_t lanes,
                         float w[LANES])
{
    if (lanes == LANES)
    {
        maniBlendLanesWeights(layer->weight,
                              layer->boneWeights != NULL ? layer->boneWeights + first : NULL, w);
        return;
    }
    for (uint32_t k = 0; k < LANES; ++k)
    {
        w[k] = layer->weight;
    }
    for (uint32_t k = 0; layer->boneWeights != NULL && k < lanes; ++k)
    {
        w[k] = layer->weight * layer->boneWeights[first + k];
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

// The rest added at its share where a bone's total was under the
// threshold.
static void FillRest(float* o, const float* share, const maniSkeleton* skeleton, uint32_t first,
                     uint32_t lanes, bool difference)
{
    for (uint32_t k = 0; k < lanes; ++k)
    {
        if (!(share[k] > 0.0f))
        {
            continue;
        }
        float rest[COMPONENTS];
        Rest(skeleton, first + k, difference, rest);
        float dot = ((o[3 * LANES + k] * rest[3] + o[4 * LANES + k] * rest[4]) +
                     o[5 * LANES + k] * rest[5]) +
                    o[6 * LANES + k] * rest[6];
        float rf = dot < 0.0f ? -share[k] : share[k];
        for (uint32_t c = 0; c < COMPONENTS; ++c)
        {
            o[c * LANES + k] += rest[c] * (c >= 3 && c < 7 ? rf : share[k]);
        }
    }
}

// The lanes nothing weighed take their rest as it is, and padding bones
// past the last stay the identity.
static void Settle(float* o, const float* total, const maniSkeleton* skeleton, uint32_t first,
                   uint32_t lanes, bool difference)
{
    bool any = lanes < LANES;
    for (uint32_t k = 0; k < LANES; ++k)
    {
        any |= total[k] == 0.0f;
    }
    for (uint32_t k = 0; any && k < LANES; ++k)
    {
        if (k < lanes && total[k] != 0.0f)
        {
            continue;
        }
        float rest[COMPONENTS];
        // Bones past the skeleton's count have the identity for a rest.
        Rest(skeleton, first + k, difference, rest);
        for (uint32_t c = 0; c < COMPONENTS; ++c)
        {
            o[c * LANES + k] = rest[c];
        }
    }
}

// Each bone's total over the layers (from the weights alone, so known
// before any pose is read), the reciprocal the weights' shares take (of
// the threshold where the total is under it), and the rest's share of
// the shortfall; whether any bone takes a share.
static bool Shares(const maniBlendLayer* layers, uint32_t count, uint32_t first, uint32_t lanes,
                   float threshold, float* total, float* inverse, float* share)
{
    float w[LANES];
    for (uint32_t k = 0; k < LANES; ++k)
    {
        total[k] = 0.0f;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        GroupWeights(&layers[i], first, lanes, w);
        maniBlendLanesAdd(total, w);
    }
    maniBlendLanesShares(total, threshold, inverse, share);
    bool any = false;
    for (uint32_t k = 0; k < LANES; ++k)
    {
        any |= share[k] > 0.0f;
    }
    return any;
}

maniResult maniBlendPoses(const maniSkeleton* skeleton, const maniBlendLayer* layers,
                          uint32_t count, float threshold, maniPose* out)
{
    return maniBlendPosesIn(skeleton, layers, count, threshold, out, NULL);
}

maniResult maniBlendPosesIn(const maniSkeleton* skeleton, const maniBlendLayer* layers,
                            uint32_t count, float threshold, maniPose* out, const uint8_t* groups)
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
        if (groups != NULL && groups[first / LANES] == 0)
        {
            continue;
        }
        uint32_t lanes = bones - first < LANES ? bones - first : LANES;
        float* o = locals + (size_t)first * COMPONENTS;
        float w[LANES];
        float total[LANES];
        float inverse[LANES];
        float share[LANES];
        bool fill = Shares(layers, count, first, lanes, threshold, total, inverse, share);
        bool started = false;
        for (uint32_t i = 0; i < count; ++i)
        {
            if (layers[i].weight == 0.0f)
            {
                continue;
            }
            GroupWeights(&layers[i], first, lanes, w);
            maniBlendLanesScale(w, inverse);
            maniBlendLanesAccumulate(
                o, maniPoseLocalsRead(layers[i].pose) + (size_t)first * COMPONENTS, w, !started);
            started = true;
        }
        if (started)
        {
            if (fill)
            {
                FillRest(o, share, skeleton, first, lanes, difference);
            }
            maniBlendLanesNormalize(o);
        }
        Settle(o, total, skeleton, first, lanes, difference);
    }
    maniPoseSetKind(out, difference);
    maniPoseMarkStale(out);
    return mani_success;
}
