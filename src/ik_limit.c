// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint limits. A turn r splits into a swing and a twist,
// r = swing twist, the twist r's rotation about the axis (r's vector
// part projected onto it, normalized) and the swing the rest, about an
// axis perpendicular to it. Angles come from the library's own arc
// functions; a turn is rebuilt from a clamped angle only when the
// clamp changed it, so a turn within its limit comes back as it was.

#include "ik_limit.h"

#include "scalar.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/ik.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#define PI_F 3.14159265358979f

static bool Within(float v, float low, float high)
{
    return v >= low && v <= high;
}

bool maniIkLimitValid(const maniJointLimit* limit)
{
    switch (limit->kind)
    {
    case maniJointLimitNone:
        return true;
    case maniJointLimitCone:
        return Within(limit->cone, 0.0f, PI_F) && Within(limit->twistLow, -PI_F, PI_F) &&
               Within(limit->twistHigh, limit->twistLow, PI_F);
    case maniJointLimitHinge:
        return fabsf(maniVec3Length(limit->axis) - 1.0f) <= 1e-3f &&
               Within(limit->low, -PI_F, PI_F) && Within(limit->high, limit->low, PI_F);
    default:
        return false;
    }
}

// The turn by angle about unit axis u.
static maniQuat About(maniVec3 u, float angle)
{
    float s = 0.0f;
    float c = 0.0f;
    maniSinCos(0.5f * angle, &s, &c);
    return (maniQuat){u.x * s, u.y * s, u.z * s, c};
}

// r's twist about unit axis u, w at least 0 as r's is; the identity when
// r is half a turn about an axis perpendicular to u.
static maniQuat Twist(maniQuat r, maniVec3 u)
{
    float d = r.x * u.x + r.y * u.y + r.z * u.z;
    float l = sqrtf(d * d + r.w * r.w);
    if (!(l > 0.0f))
    {
        return maniQuatIdentity();
    }
    float inverse = 1.0f / l;
    return (maniQuat){u.x * d * inverse, u.y * d * inverse, u.z * d * inverse, r.w * inverse};
}

// A twist (w at least 0) about u with its angle clamped from low to high.
static maniQuat ClampTwist(maniQuat t, maniVec3 u, float low, float high)
{
    float angle = 2.0f * maniAtan2(t.x * u.x + t.y * u.y + t.z * u.z, t.w);
    float clamped = angle < low ? low : (angle > high ? high : angle);
    return clamped == angle ? t : About(u, clamped);
}

// A swing (w at least 0) with its angle cut to the cone.
static maniQuat ClampSwing(maniQuat s, float cone)
{
    maniVec3 v = {s.x, s.y, s.z};
    float sine = maniVec3Length(v);
    if (!(2.0f * maniAtan2(sine, s.w) > cone))
    {
        return s;
    }
    return About(maniVec3Scale(v, 1.0f / sine), cone);
}

maniQuat maniIkLimit(const maniJointLimit* limit, maniVec3 axis, maniQuat r)
{
    if (limit == NULL || limit->kind == maniJointLimitNone)
    {
        return r;
    }
    r = r.w < 0.0f ? (maniQuat){-r.x, -r.y, -r.z, -r.w} : r;
    if (limit->kind == maniJointLimitHinge)
    {
        maniVec3 u = maniVec3Normalize(limit->axis, limit->axis);
        return ClampTwist(Twist(r, u), u, limit->low, limit->high);
    }
    bool hasAxis = maniVec3Dot(axis, axis) > 0.0f;
    maniQuat twist = hasAxis ? Twist(r, axis) : maniQuatIdentity();
    // The swing's w is r.w twist.w plus the vector parts' dot, (r.w^2 +
    // d^2) / l = l with d and l Twist's: at least 0 with no flip.
    maniQuat swing = maniQuatMul(r, maniQuatConjugate(twist));
    maniQuat limitedSwing = ClampSwing(swing, limit->cone);
    maniQuat limitedTwist =
        hasAxis ? ClampTwist(twist, axis, limit->twistLow, limit->twistHigh) : twist;
    if (limitedSwing.x == swing.x && limitedSwing.y == swing.y && limitedSwing.z == swing.z &&
        limitedSwing.w == swing.w && limitedTwist.w == twist.w && limitedTwist.x == twist.x &&
        limitedTwist.y == twist.y && limitedTwist.z == twist.z)
    {
        return r;
    }
    return maniQuatNormalize(maniQuatMul(limitedSwing, limitedTwist));
}
