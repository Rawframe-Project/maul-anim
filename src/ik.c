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

#include "ik_frame.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static float Larger(float a, float b)
{
    return a > b ? a : b;
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
static maniVec3 Hinge(const maniTwoBoneSolve* solve, const maniIkFrame* middle, maniVec3 a,
                      maniVec3 b)
{
    maniVec3 zero = maniVec3Make(0.0f, 0.0f, 0.0f);
    maniQuat r = maniIkRotation(middle);
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
    return maniIkHalf(h, cw * cc + sw * sc, sw * cc - cw * sc);
}

static bool Valid(const maniPose* pose, const maniSkeleton* skeleton, const maniTwoBoneSolve* s)
{
    uint32_t n = skeleton->boneCount;
    return maniIkPoseUsable(pose, skeleton) && s->start < n && s->middle < n && s->end < n &&
           maniIkAbove(skeleton, s->start, s->middle) && maniIkAbove(skeleton, s->middle, s->end) &&
           maniIkFiniteVec(s->goal) && maniIkFiniteVec(s->pole) && maniIkFiniteVec(s->hinge) &&
           isfinite(s->twist) && s->soften >= 0.0f && s->soften <= 1.0f && s->weight >= 0.0f &&
           s->weight <= 1.0f;
}

maniResult maniSolveTwoBone(maniPose* pose, const maniSkeleton* skeleton,
                            const maniTwoBoneSolve* solve, bool* reachedOut)
{
    if (pose == NULL || skeleton == NULL || solve == NULL || !Valid(pose, skeleton, solve))
    {
        return mani_errorInvalid;
    }
    maniIkFrame startParent = maniIkParentModel(pose, skeleton, solve->start);
    maniIkFrame local = maniIkLocal(pose, solve->start);
    maniIkFrame start = maniIkMul(&startParent, &local);
    maniIkFrame path =
        maniIkBelow(pose, skeleton, solve->start, maniSkeletonParent(skeleton, solve->middle));
    maniIkFrame middleParent = maniIkMul(&start, &path);
    local = maniIkLocal(pose, solve->middle);
    maniIkFrame middle = maniIkMul(&middleParent, &local);
    path = maniIkBelow(pose, skeleton, solve->middle, solve->end);
    maniIkFrame end = maniIkMul(&middle, &path);

    maniVec3 s = maniIkOrigin(&start);
    maniVec3 m = maniIkOrigin(&middle);
    maniVec3 a = maniVec3Sub(m, s);
    maniVec3 b = maniVec3Sub(maniIkOrigin(&end), m);
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
        maniQuat plane = maniIkPlane(u, maniQuatRotate(turnStart, h), solve->pole);
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
    maniIkCorrect(pose, solve->start,
                  maniIkWeighted(maniIkInParent(turnStart, &startParent), solve->weight));
    maniIkCorrect(pose, solve->middle,
                  maniIkWeighted(maniIkInParent(turnMiddle, &middleParent), solve->weight));
    if (reachedOut != NULL)
    {
        *reachedOut = reached && solve->weight >= 1.0f;
    }
    return mani_success;
}
