// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Two-bone inverse kinematics (mani-0029), after ozz-animation's
// IKTwoBoneJob: the goal softened past a share of the chain's length,
// the middle turned about its hinge by the difference of the wanted and
// current angles (law of cosines, in cosines and sines: no acos), the
// start turned by the shortest arc of the chain onto the goal, then
// about the goal line to the pole's plane and by the twist. The pole's
// plane blends to the chain's own as the pole nears the goal line, so it
// never flips. Model frames come from the locals along the root path;
// a model-space turn T of a bone whose parent's model rotation is P
// becomes the local correction P* T P (mirrored under a mirrored
// parent).

#include "maul-anim/ik.h"

#include "decompose.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Below this sine of the pole's angle to the goal line the pole's plane
// blends toward the chain's own.
#define POLE_SINE 0.05f

static const maniAffine kIdentity = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};

static float Larger(float a, float b)
{
    return a > b ? a : b;
}

static bool FiniteVec(maniVec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static maniAffine Local(const maniPose* pose, uint32_t bone)
{
    maniTransform t = maniPoseLocal(pose, bone);
    return maniAffineFromTrs(t.translation, t.rotation, t.scale);
}

// A bone's frame in its ancestor top's frame (the model's for
// MANI_NO_BONE): the locals below top composed.
static maniAffine Below(const maniPose* pose, const maniSkeleton* skeleton, uint32_t top,
                        uint32_t bone)
{
    maniAffine m = kIdentity;
    for (uint32_t b = bone; b != top; b = maniSkeletonParent(skeleton, b))
    {
        maniAffine local = Local(pose, b);
        m = maniAffineMul(&local, &m);
    }
    return m;
}

static bool Above(const maniSkeleton* skeleton, uint32_t ancestor, uint32_t bone)
{
    for (uint32_t b = maniSkeletonParent(skeleton, bone); b != MANI_NO_BONE;
         b = maniSkeletonParent(skeleton, b))
    {
        if (b == ancestor)
        {
            return true;
        }
    }
    return false;
}

static maniVec3 Origin(const maniAffine* m)
{
    return maniVec3Make(m->m[9], m->m[10], m->m[11]);
}

static maniQuat Rotation(const maniAffine* m)
{
    maniVec3 t;
    maniQuat q;
    maniDecomposeRigid(m, &t, &q);
    return q;
}

static bool Mirrored(const maniAffine* m)
{
    const float* a = m->m;
    float det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[3] * (a[1] * a[8] - a[2] * a[7]) +
                a[6] * (a[1] * a[5] - a[2] * a[4]);
    return det < 0.0f;
}

// A model-space turn as a correction of a bone's local rotation under a
// parent frame: P* q P, mirrored through x under a mirrored parent (the
// mirror the decomposition gives).
static maniQuat InParent(maniQuat q, const maniAffine* parent)
{
    maniQuat p = Rotation(parent);
    maniQuat r = maniQuatMul(maniQuatMul(maniQuatConjugate(p), q), p);
    return Mirrored(parent) ? (maniQuat){r.x, -r.y, -r.z, r.w} : r;
}

// The turn by the angle whose cosine and sine are c and s (any common
// scale) about unit axis u: the half angle's sine from the sine where
// the cosine is positive, from the cosine elsewhere, so neither small nor
// near-half turns lose digits.
static maniQuat Half(maniVec3 u, float c, float s)
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
static maniQuat About(maniVec3 u, maniVec3 a, maniVec3 b)
{
    return Half(u, maniVec3Dot(a, b), maniVec3Dot(maniVec3Cross(a, b), u));
}

// The goal from the start, softened as ozz's; whether it is reached.
static maniVec3 Soften(maniVec3 goal, float lengthA, float lengthB, float soften, bool* reached)
{
    float distance = maniVec3Length(goal);
    float chain = lengthA + lengthB;
    float da = chain * soften;
    float ds = chain - da;
    *reached = distance <= da && distance > fabsf(lengthA - lengthB);
    if (distance > da && distance > 0.0f && ds > 0.0f)
    {
        float alpha = (distance - da) / ds;
        float op = alpha + 3.0f;
        float op2 = op * op;
        float ratio = 81.0f / (op2 * op2);
        float softened = da + ds - ds * ratio;
        return maniVec3Scale(goal, softened / distance);
    }
    return goal;
}

// The middle's opening axis in model space: the hinge given, else the
// chain's bend plane, else (straight) a perpendicular from the middle
// bone's x or z.
static maniVec3 Hinge(const maniTwoBoneSolve* solve, const maniAffine* middle, maniVec3 a,
                      maniVec3 b)
{
    maniVec3 zero = maniVec3Make(0.0f, 0.0f, 0.0f);
    maniQuat r = Rotation(middle);
    if (maniVec3Dot(solve->hinge, solve->hinge) > 0.0f)
    {
        return maniVec3Normalize(maniQuatRotate(r, solve->hinge), zero);
    }
    maniVec3 bend = maniVec3Cross(b, a);
    float scale = maniVec3Dot(a, a) * maniVec3Dot(b, b);
    if (maniVec3Dot(bend, bend) > 1e-12f * scale)
    {
        return maniVec3Normalize(bend, zero);
    }
    maniVec3 u = maniVec3Normalize(a, zero);
    maniVec3 axes[2] = {maniQuatRotate(r, maniVec3Make(1.0f, 0.0f, 0.0f)),
                        maniQuatRotate(r, maniVec3Make(0.0f, 0.0f, 1.0f))};
    for (int k = 0; k < 2; ++k)
    {
        maniVec3 p = maniVec3Sub(axes[k], maniVec3Scale(u, maniVec3Dot(axes[k], u)));
        if (maniVec3Dot(p, p) > 1e-6f)
        {
            return maniVec3Normalize(p, zero);
        }
    }
    return zero;
}

// The middle's turn about axis h opening the angle at m from its current
// cosine to the one spanning the goal's distance.
static maniQuat Middle(maniVec3 a, maniVec3 b, maniVec3 h, float goal)
{
    float la = maniVec3Length(a);
    float lb = maniVec3Length(b);
    float lab = la * lb;
    if (!(lab > 0.0f) || maniVec3Dot(h, h) == 0.0f)
    {
        return maniQuatIdentity();
    }
    // The wanted angle's cosine and sine by the law of cosines, 1 - cos
    // and 1 + cos as products of sums and differences of the lengths so
    // a nearly straight or folded chain keeps its digits.
    float d = la - lb;
    float l = la + lb;
    float below = Larger(0.0f, (goal - d) * (goal + d));
    float above = Larger(0.0f, (l - goal) * (l + goal));
    float cw = 1.0f - below / (2.0f * lab);
    cw = goal >= l ? -1.0f : goal <= fabsf(d) ? 1.0f : cw;
    float sw = goal >= l || goal <= fabsf(d) ? 0.0f : sqrtf(below * above) / (2.0f * lab);
    // The current angle from the chain itself, negative when it bends
    // backward against the hinge.
    float cc = -maniVec3Dot(a, b) / lab;
    float sc = maniVec3Length(maniVec3Cross(a, b)) / lab;
    sc = maniVec3Dot(maniVec3Cross(a, h), b) < 0.0f ? -sc : sc;
    return Half(h, cw * cc + sw * sc, sw * cc - cw * sc);
}

// The turn about the goal line u taking the chain's plane normal n to
// the pole's plane, blended to none as the pole nears the line.
static maniQuat Plane(maniVec3 u, maniVec3 n, maniVec3 pole)
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
    maniQuat full = About(u, maniVec3Normalize(n, zero), wanted);
    float w = sine >= POLE_SINE ? 1.0f : sine / POLE_SINE;
    return w >= 1.0f ? full : maniQuatNlerp(maniQuatIdentity(), full, w);
}

// A correction weighted from the identity, w kept at least 0.
static maniQuat Weighted(maniQuat q, float weight)
{
    q = q.w < 0.0f ? (maniQuat){-q.x, -q.y, -q.z, -q.w} : q;
    return weight >= 1.0f ? q : maniQuatNlerp(maniQuatIdentity(), q, weight);
}

static void Correct(maniPose* pose, uint32_t bone, maniQuat correction)
{
    maniTransform t = maniPoseLocal(pose, bone);
    t.rotation = maniQuatNormalize(maniQuatMul(correction, t.rotation));
    maniPoseSetLocal(pose, bone, t);
}

static bool Valid(const maniPose* pose, const maniSkeleton* skeleton, const maniTwoBoneSolve* s)
{
    uint32_t n = skeleton->boneCount;
    return maniPoseOfSkeleton(pose, skeleton->identity, n) && !maniPoseIsDifference(pose) &&
           s->start < n && s->middle < n && s->end < n && Above(skeleton, s->start, s->middle) &&
           Above(skeleton, s->middle, s->end) && FiniteVec(s->goal) && FiniteVec(s->pole) &&
           FiniteVec(s->hinge) && isfinite(s->twist) && s->soften >= 0.0f && s->soften <= 1.0f &&
           s->weight >= 0.0f && s->weight <= 1.0f;
}

maniResult maniSolveTwoBone(maniPose* pose, const maniSkeleton* skeleton,
                            const maniTwoBoneSolve* solve, bool* reachedOut)
{
    if (pose == NULL || skeleton == NULL || solve == NULL || !Valid(pose, skeleton, solve))
    {
        return mani_errorInvalid;
    }
    uint32_t above = maniSkeletonParent(skeleton, solve->start);
    maniAffine startParent =
        above != MANI_NO_BONE ? Below(pose, skeleton, MANI_NO_BONE, above) : kIdentity;
    maniAffine local = Local(pose, solve->start);
    maniAffine start = maniAffineMul(&startParent, &local);
    maniAffine path =
        Below(pose, skeleton, solve->start, maniSkeletonParent(skeleton, solve->middle));
    maniAffine middleParent = maniAffineMul(&start, &path);
    local = Local(pose, solve->middle);
    maniAffine middle = maniAffineMul(&middleParent, &local);
    path = Below(pose, skeleton, solve->middle, solve->end);
    maniAffine end = maniAffineMul(&middle, &path);

    maniVec3 s = Origin(&start);
    maniVec3 m = Origin(&middle);
    maniVec3 a = maniVec3Sub(m, s);
    maniVec3 b = maniVec3Sub(Origin(&end), m);
    bool reached = false;
    maniVec3 goal = Soften(maniVec3Sub(solve->goal, s), maniVec3Length(a), maniVec3Length(b),
                           solve->soften, &reached);
    maniVec3 h = Hinge(solve, &middle, a, b);
    maniQuat turnMiddle = Middle(a, b, h, maniVec3Length(goal));
    maniVec3 reach = maniVec3Add(a, maniQuatRotate(turnMiddle, b));
    maniQuat turnStart = maniQuatBetween(reach, goal);
    float goalLength = maniVec3Length(goal);
    if (goalLength > 0.0f && maniVec3Dot(reach, reach) > 0.0f)
    {
        maniVec3 u = maniVec3Scale(goal, 1.0f / goalLength);
        maniQuat plane = Plane(u, maniQuatRotate(turnStart, h), solve->pole);
        turnStart = maniQuatMul(plane, turnStart);
        if (solve->twist != 0.0f)
        {
            turnStart = maniQuatMul(maniQuatFromAxisAngle(u, solve->twist), turnStart);
        }
    }
    else
    {
        turnStart = maniQuatIdentity();
    }
    Correct(pose, solve->start, Weighted(InParent(turnStart, &startParent), solve->weight));
    Correct(pose, solve->middle, Weighted(InParent(turnMiddle, &middleParent), solve->weight));
    if (reachedOut != NULL)
    {
        *reachedOut = reached && solve->weight >= 1.0f;
    }
    return mani_success;
}
