// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Differences and additive layers: d_t = t - t_b,
// d_r = r_b^-1 r, d_s = s / s_b, applied as t + w d_t,
// r nlerp(1, d_r, w), s (1 + w (d_s - 1)). Binary32 in a fixed order.

#include "additive.h"

#include "blend_groups.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LANES MANI_POSE_GROUP_BONES

// Between 0 and 1 (NaN fails).
static bool Usable(float w)
{
    return w >= 0.0f && w <= 1.0f;
}

static bool Finite(float v)
{
    return v >= -FLT_MAX && v <= FLT_MAX;
}

// To unit length in binary32, the identity for a zero or overflowing sum.
static maniQuat Normalize(maniQuat q)
{
    float squared = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    if (!(squared > 0.0f && squared <= FLT_MAX))
    {
        return maniQuatIdentity();
    }
    float inverse = 1.0f / sqrtf(squared);
    return (maniQuat){q.x * inverse, q.y * inverse, q.z * inverse, q.w * inverse};
}

static maniTransform Read(const float* locals, uint32_t bone)
{
    const float* s =
        locals + (size_t)(bone / LANES) * (LANES * MANI_POSE_COMPONENTS) + bone % LANES;
    return (maniTransform){{s[0], s[LANES], s[2 * LANES]},
                           {s[3 * LANES], s[4 * LANES], s[5 * LANES], s[6 * LANES]},
                           {s[7 * LANES], s[8 * LANES], s[9 * LANES]}};
}

static void Write(float* locals, uint32_t bone, const maniTransform* t)
{
    const float values[MANI_POSE_COMPONENTS] = {
        t->translation.x, t->translation.y, t->translation.z, t->rotation.x, t->rotation.y,
        t->rotation.z,    t->rotation.w,    t->scale.x,       t->scale.y,    t->scale.z};
    maniPoseSlotPut(maniPoseSlot(locals, bone), 0, values, MANI_POSE_COMPONENTS);
}

static bool IsPose(const maniPose* pose, const maniSkeleton* skeleton)
{
    return maniPoseMatches(pose, skeleton) && !maniPoseIsDifference(pose);
}

// A bone's difference: t - t_b, r_b^-1 r (normalized), s / s_b.
static maniTransform Difference(const float* p, const float* b, uint32_t bone)
{
    maniTransform t = Read(p, bone);
    maniTransform tb = Read(b, bone);
    return (maniTransform){
        maniVec3Sub(t.translation, tb.translation),
        Normalize(maniQuatMul(maniQuatConjugate(tb.rotation), t.rotation)),
        {t.scale.x / tb.scale.x, t.scale.y / tb.scale.y, t.scale.z / tb.scale.z},
    };
}

static bool DifferenceFinite(const maniTransform* d)
{
    return Finite(d->translation.x) && Finite(d->translation.y) && Finite(d->translation.z) &&
           Finite(d->scale.x) && Finite(d->scale.y) && Finite(d->scale.z);
}

maniResult maniPoseDifference(const maniSkeleton* skeleton, const maniPose* pose,
                              const maniPose* basis, maniPose* out)
{
    if (!IsPose(pose, skeleton) || !IsPose(basis, skeleton) || !maniPoseMatches(out, skeleton))
    {
        return mani_errorInvalid;
    }
    const float* p = maniPoseLocalsRead(pose);
    const float* b = maniPoseLocalsRead(basis);
    // Every difference finite (a basis scale of 0 or near it, or
    // translations far apart, are not) before any is written.
    for (uint32_t bone = 0; bone < skeleton->boneCount; ++bone)
    {
        maniTransform d = Difference(p, b, bone);
        if (!DifferenceFinite(&d))
        {
            return mani_errorInvalid;
        }
    }
    float* o = maniPoseLocals(out);
    // Bone by bone, each read before it is written: out may be either.
    for (uint32_t bone = 0; bone < skeleton->boneCount; ++bone)
    {
        maniTransform d = Difference(p, b, bone);
        Write(o, bone, &d);
    }
    maniPoseSetKind(out, true);
    maniPoseMarkStale(out);
    return mani_success;
}

static bool AdditiveValid(const maniSkeleton* skeleton, const maniPose* pose,
                          const maniBlendLayer* layers, uint32_t count)
{
    if (!IsPose(pose, skeleton) || (layers == NULL && count > 0))
    {
        return false;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        const maniBlendLayer* l = &layers[i];
        if (!maniPoseMatches(l->pose, skeleton) || !maniPoseIsDifference(l->pose) ||
            !Usable(l->weight))
        {
            return false;
        }
        for (uint32_t b = 0; l->boneWeights != NULL && b < skeleton->boneCount; ++b)
        {
            if (!Usable(l->boneWeights[b]))
            {
                return false;
            }
        }
    }
    return true;
}

void maniAddTransform(maniTransform* t, const maniTransform* d, float w)
{
    t->translation = maniVec3Add(t->translation, maniVec3Scale(d->translation, w));
    // The difference's turn on its positive-w half, so a weight between
    // 0 and 1 takes the short way from the identity.
    float sign = d->rotation.w < 0.0f ? -1.0f : 1.0f;
    maniQuat q = {sign * d->rotation.x * w, sign * d->rotation.y * w, sign * d->rotation.z * w,
                  (sign * d->rotation.w - 1.0f) * w + 1.0f};
    t->rotation = maniQuatMul(t->rotation, Normalize(q));
    t->scale.x *= 1.0f + w * (d->scale.x - 1.0f);
    t->scale.y *= 1.0f + w * (d->scale.y - 1.0f);
    t->scale.z *= 1.0f + w * (d->scale.z - 1.0f);
}

maniResult maniAddPoses(const maniSkeleton* skeleton, maniPose* pose, const maniBlendLayer* layers,
                        uint32_t count)
{
    return maniAddPosesIn(skeleton, pose, layers, count, NULL);
}

maniResult maniAddPosesIn(const maniSkeleton* skeleton, maniPose* pose,
                          const maniBlendLayer* layers, uint32_t count, const uint8_t* groups)
{
    if (!AdditiveValid(skeleton, pose, layers, count))
    {
        return mani_errorInvalid;
    }
    float* locals = maniPoseLocals(pose);
    for (uint32_t bone = 0; bone < skeleton->boneCount; ++bone)
    {
        if (groups != NULL && groups[bone / MANI_POSE_GROUP_BONES] == 0)
        {
            continue;
        }
        maniTransform t = Read(locals, bone);
        for (uint32_t i = 0; i < count; ++i)
        {
            const maniBlendLayer* l = &layers[i];
            float w = l->boneWeights != NULL ? l->weight * l->boneWeights[bone] : l->weight;
            if (w == 0.0f)
            {
                continue;
            }
            maniTransform d = Read(maniPoseLocalsRead(l->pose), bone);
            maniAddTransform(&t, &d, w);
        }
        Write(locals, bone, &t);
    }
    maniPoseMarkStale(pose);
    return mani_success;
}
