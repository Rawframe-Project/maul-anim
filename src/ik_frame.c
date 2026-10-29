// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the IK solvers share. A model-space turn T of a bone
// whose parent's model rotation is P becomes the local correction P* T P
// (mirrored under a mirrored parent); half angles come from whichever of
// the cosine and sine keeps its digits; the pole's plane blends to a
// chain's own as the pole nears the aim line, so it never flips.

#include "ik_frame.h"

#include "decompose.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Below this sine of the pole's angle to the aim line the pole's plane
// blends toward the chain's own.
#define POLE_SINE 0.05f

static const maniIkFrame kIdentity = {.rotation = {0.0f, 0.0f, 0.0f, 1.0f}};

bool maniIkFiniteVec(maniVec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

bool maniIkPoseUsable(const maniPose* pose, const maniSkeleton* skeleton)
{
    return maniPoseOfSkeleton(pose, skeleton->identity, skeleton->boneCount) &&
           !maniPose_IsDifference(pose);
}

maniIkFrame maniIkLocal(const maniPose* pose, uint32_t bone)
{
    maniTransform t = maniPose_GetLocal(pose, bone);
    if (t.scale.x == 1.0f && t.scale.y == 1.0f && t.scale.z == 1.0f)
    {
        return (maniIkFrame){.rotation = t.rotation, .origin = t.translation};
    }
    return (maniIkFrame){.affine = maniAffineFromTrs(t.translation, t.rotation, t.scale),
                         .scaled = true};
}

static maniAffine Affine(const maniIkFrame* f)
{
    return f->scaled ? f->affine
                     : maniAffineFromTrs(f->origin, f->rotation, maniVec3Make(1.0f, 1.0f, 1.0f));
}

maniIkFrame maniIkMul(const maniIkFrame* a, const maniIkFrame* b)
{
    if (!a->scaled && !b->scaled)
    {
        return (maniIkFrame){.rotation = maniQuatMul(a->rotation, b->rotation),
                             .origin =
                                 maniVec3Add(a->origin, maniQuatRotate(a->rotation, b->origin))};
    }
    maniAffine x = Affine(a);
    maniAffine y = Affine(b);
    return (maniIkFrame){.affine = maniAffineMul(&x, &y), .scaled = true};
}

// A bone's frame in its ancestor top's frame (the model's for
// MANI_NO_BONE): the locals below top composed.
maniIkFrame maniIkBelow(const maniPose* pose, const maniSkeleton* skeleton, uint32_t top,
                        uint32_t bone)
{
    if (bone == top)
    {
        return kIdentity;
    }
    maniIkFrame m = maniIkLocal(pose, bone);
    for (uint32_t b = maniSkeleton_GetParent(skeleton, bone); b != top;
         b = maniSkeleton_GetParent(skeleton, b))
    {
        maniIkFrame local = maniIkLocal(pose, b);
        m = maniIkMul(&local, &m);
    }
    return m;
}

bool maniIkAbove(const maniSkeleton* skeleton, uint32_t ancestor, uint32_t bone)
{
    for (uint32_t b = maniSkeleton_GetParent(skeleton, bone); b != MANI_NO_BONE;
         b = maniSkeleton_GetParent(skeleton, b))
    {
        if (b == ancestor)
        {
            return true;
        }
    }
    return false;
}

maniVec3 maniIkOrigin(const maniIkFrame* f)
{
    return f->scaled ? maniVec3Make(f->affine.m[9], f->affine.m[10], f->affine.m[11]) : f->origin;
}

maniQuat maniIkRotation(const maniIkFrame* f)
{
    if (!f->scaled)
    {
        return maniQuatNormalize(f->rotation);
    }
    maniVec3 t;
    maniQuat q;
    maniDecomposeRigid(&f->affine, &t, &q);
    return q;
}

// Whether a frame mirrors: never without scales.
static bool Mirrored(const maniIkFrame* f)
{
    if (!f->scaled)
    {
        return false;
    }
    const float* a = f->affine.m;
    float det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[3] * (a[1] * a[8] - a[2] * a[7]) +
                a[6] * (a[1] * a[5] - a[2] * a[4]);
    return det < 0.0f;
}

// A model-space turn as a correction of a bone's local rotation under a
// parent frame: P* q P, mirrored through x under a mirrored parent (the
// mirror the decomposition gives).
maniQuat maniIkInParent(maniQuat q, const maniIkFrame* parent)
{
    maniQuat p = maniIkRotation(parent);
    maniQuat r = maniQuatMul(maniQuatMul(maniQuatConjugate(p), q), p);
    return Mirrored(parent) ? (maniQuat){r.x, -r.y, -r.z, r.w} : r;
}

maniQuat maniIkFromParent(maniQuat c, const maniIkFrame* parent)
{
    maniQuat p = maniIkRotation(parent);
    c = Mirrored(parent) ? (maniQuat){c.x, -c.y, -c.z, c.w} : c;
    return maniQuatMul(maniQuatMul(p, c), maniQuatConjugate(p));
}

// The turn by the angle whose cosine and sine are c and s (any common
// scale) about unit axis u: the half angle's sine from the sine where
// the cosine is positive, from the cosine elsewhere, so neither small nor
// near-half turns lose digits.
maniQuat maniIkHalf(maniVec3 u, float c, float s)
{
    float l = sqrtf(c * c + s * s);
    if (!(l > 0.0f))
    {
        return maniQuatIdentity();
    }
    c /= l;
    s /= l;
    float w = 0.0f;
    float x = 0.0f;
    if (c >= 0.0f)
    {
        w = sqrtf(0.5f * (1.0f + c));
        x = s / (2.0f * w);
    }
    else
    {
        x = sqrtf(0.5f * (1.0f - c));
        x = s < 0.0f ? -x : x;
        w = s / (2.0f * x);
    }
    maniVec3 axis = maniVec3Scale(u, x);
    return (maniQuat){axis.x, axis.y, axis.z, w};
}

// The turn about unit axis u taking a onto b, both perpendicular to u.
maniQuat maniIkAbout(maniVec3 u, maniVec3 a, maniVec3 b)
{
    return maniIkHalf(u, maniVec3Dot(a, b), maniVec3Dot(maniVec3Cross(a, b), u));
}

// The turn about the goal line u taking the chain's plane normal n to
// the pole's plane, blended to none as the pole nears the line.
maniQuat maniIkPlane(maniVec3 u, maniVec3 n, maniVec3 pole)
{
    maniVec3 zero = maniVec3Make(0.0f, 0.0f, 0.0f);
    float poleLength = maniVec3Length(pole);
    n = maniVec3Sub(n, maniVec3Scale(u, maniVec3Dot(n, u)));
    if (!(poleLength > 0.0f) || maniVec3Dot(n, n) == 0.0f)
    {
        return maniQuatIdentity();
    }
    maniVec3 p = maniVec3Sub(pole, maniVec3Scale(u, maniVec3Dot(pole, u)));
    float sine = maniVec3Length(p) / poleLength;
    if (!(sine > 0.0f))
    {
        return maniQuatIdentity();
    }
    maniVec3 wanted = maniVec3Normalize(maniVec3Cross(u, p), zero);
    maniQuat full = maniIkAbout(u, maniVec3Normalize(n, zero), wanted);
    float w = sine >= POLE_SINE ? 1.0f : sine / POLE_SINE;
    return w >= 1.0f ? full : maniQuatNlerp(maniQuatIdentity(), full, w);
}

// A correction weighted from the identity, w kept at least 0.
maniQuat maniIkWeighted(maniQuat q, float weight)
{
    q = q.w < 0.0f ? (maniQuat){-q.x, -q.y, -q.z, -q.w} : q;
    return weight >= 1.0f ? q : maniQuatNlerp(maniQuatIdentity(), q, weight);
}

void maniIkCorrect(maniPose* pose, uint32_t bone, maniQuat correction)
{
    maniTransform t = maniPose_GetLocal(pose, bone);
    t.rotation = maniQuatNormalize(maniQuatMul(correction, t.rotation));
    maniPose_SetLocal(pose, bone, t);
}

maniIkFrame maniIkParentModel(const maniPose* pose, const maniSkeleton* skeleton, uint32_t bone)
{
    uint32_t parent = maniSkeleton_GetParent(skeleton, bone);
    return parent != MANI_NO_BONE ? maniIkBelow(pose, skeleton, MANI_NO_BONE, parent) : kIdentity;
}
