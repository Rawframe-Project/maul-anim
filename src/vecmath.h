// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Vectors, unit quaternions and affine matrices in binary32. The small
// operations are inline; every one is a fixed sequence of exactly rounded
// operations, so results are the same bits everywhere (mani-0001).
// Quaternions are (x, y, z, w); a * b turns by b, then by a. An affine
// matrix holds its three columns, then the translation.

#ifndef MAUL_ANIM_SRC_VECMATH_H
#define MAUL_ANIM_SRC_VECMATH_H

#include "maul-anim/base.h"

#include <stdbool.h>

static inline maniVec3 maniVec3Make(float x, float y, float z)
{
    return (maniVec3){x, y, z};
}

static inline maniVec3 maniVec3Add(maniVec3 a, maniVec3 b)
{
    return (maniVec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static inline maniVec3 maniVec3Sub(maniVec3 a, maniVec3 b)
{
    return (maniVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static inline maniVec3 maniVec3Scale(maniVec3 a, float s)
{
    return (maniVec3){a.x * s, a.y * s, a.z * s};
}

static inline float maniVec3Dot(maniVec3 a, maniVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline maniVec3 maniVec3Cross(maniVec3 a, maniVec3 b)
{
    return (maniVec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline maniQuat maniQuatIdentity(void)
{
    return (maniQuat){0.0f, 0.0f, 0.0f, 1.0f};
}

static inline maniQuat maniQuatMul(maniQuat a, maniQuat b)
{
    return (maniQuat){
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

static inline maniQuat maniQuatConjugate(maniQuat q)
{
    return (maniQuat){-q.x, -q.y, -q.z, q.w};
}

static inline float maniQuatDot(maniQuat a, maniQuat b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// v turned by the unit quaternion q: v + 2w (u x v) + 2 u x (u x v).
static inline maniVec3 maniQuatRotate(maniQuat q, maniVec3 v)
{
    maniVec3 u = {q.x, q.y, q.z};
    maniVec3 t = maniVec3Scale(maniVec3Cross(u, v), 2.0f);
    return maniVec3Add(maniVec3Add(v, maniVec3Scale(t, q.w)), maniVec3Cross(u, t));
}

// The length of v, or 0 for a zero vector.
float maniVec3Length(maniVec3 v);

// v scaled to unit length; a zero, infinite or NaN v gives fallback.
maniVec3 maniVec3Normalize(maniVec3 v, maniVec3 fallback);

// q scaled to unit length; a zero, infinite or NaN q gives the identity.
maniQuat maniQuatNormalize(maniQuat q);

// The normalized lerp from a to b at t, b first turned into a's
// hemisphere, so a negated b gives the same result.
maniQuat maniQuatNlerp(maniQuat a, maniQuat b, float t);

// The spherical lerp from a to b at t along the shorter arc: constant
// angular speed, exact at t = 0 and t = 1 up to b's sign.
maniQuat maniQuatSlerp(maniQuat a, maniQuat b, float t);

// The turn by angle radians about a unit axis.
maniQuat maniQuatFromAxisAngle(maniVec3 axis, float angle);

// A turn's scaled axis-angle on the shorter arc: its axis times its
// angle (0 for no turn).
maniVec3 maniQuatToRotationVector(maniQuat q);

// The turn of a scaled axis-angle (the identity for 0).
maniQuat maniQuatFromRotationVector(maniVec3 v);

// The shortest turn taking the unit direction from onto the unit direction
// to; opposite directions turn half way about an axis perpendicular to
// from. Always a unit quaternion, even for a zero or non-finite input.
maniQuat maniQuatBetween(maniVec3 from, maniVec3 to);

// The affine matrix that scales, then turns, then translates. Inline,
// with the matrix product below, so the model pass composes without
// calls; the expressions are fixed, as their results are pinned.
static inline maniAffine maniAffineFromTrs(maniVec3 translation, maniQuat rotation, maniVec3 scale)
{
    float x = rotation.x;
    float y = rotation.y;
    float z = rotation.z;
    float w = rotation.w;
    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;
    return (maniAffine){{
        (1.0f - 2.0f * (yy + zz)) * scale.x,
        2.0f * (xy + wz) * scale.x,
        2.0f * (xz - wy) * scale.x,
        2.0f * (xy - wz) * scale.y,
        (1.0f - 2.0f * (xx + zz)) * scale.y,
        2.0f * (yz + wx) * scale.y,
        2.0f * (xz + wy) * scale.z,
        2.0f * (yz - wx) * scale.z,
        (1.0f - 2.0f * (xx + yy)) * scale.z,
        translation.x,
        translation.y,
        translation.z,
    }};
}

// The direction v transformed by a, without translation.
static inline maniVec3 maniAffineTransformVector(const maniAffine* a, maniVec3 v)
{
    const float* m = a->m;
    return maniVec3Make(m[0] * v.x + m[3] * v.y + m[6] * v.z, m[1] * v.x + m[4] * v.y + m[7] * v.z,
                        m[2] * v.x + m[5] * v.y + m[8] * v.z);
}

// The point p transformed by a, translation included.
static inline maniVec3 maniAffineTransformPoint(const maniAffine* a, maniVec3 p)
{
    maniVec3 v = maniAffineTransformVector(a, p);
    return maniVec3Make(v.x + a->m[9], v.y + a->m[10], v.z + a->m[11]);
}

// a times b: b's transform, then a's.
static inline maniAffine maniAffineMul(const maniAffine* a, const maniAffine* b)
{
    maniAffine r;
    for (int c = 0; c < 3; ++c)
    {
        maniVec3 column = maniAffineTransformVector(
            a, maniVec3Make(b->m[3 * c], b->m[3 * c + 1], b->m[3 * c + 2]));
        r.m[3 * c] = column.x;
        r.m[3 * c + 1] = column.y;
        r.m[3 * c + 2] = column.z;
    }
    maniVec3 t = maniAffineTransformPoint(a, maniVec3Make(b->m[9], b->m[10], b->m[11]));
    r.m[9] = t.x;
    r.m[10] = t.y;
    r.m[11] = t.z;
    return r;
}

// The inverse of a; false, with inverseOut untouched, when a's linear part
// has a determinant whose magnitude is below 1e-30 or is not finite.
bool maniAffineInverse(const maniAffine* a, maniAffine* inverseOut);

#endif // MAUL_ANIM_SRC_VECMATH_H
