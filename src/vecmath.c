// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Lengths and inverses work in binary64 so that huge and tiny inputs
// neither overflow nor lose their digits; everything else stays in
// binary32. Slerp measures its angle from half-chords,
// theta = 2 atan2(|a - b|, |a + b|), which stays accurate near zero where
// acos(a . b) does not.

#include "vecmath.h"

#include "scalar.h"

#include <math.h>

float maniVec3Length(maniVec3 v)
{
    double x = (double)v.x;
    double y = (double)v.y;
    double z = (double)v.z;
    return (float)sqrt(x * x + y * y + z * z);
}

maniVec3 maniVec3Normalize(maniVec3 v, maniVec3 fallback)
{
    double x = (double)v.x;
    double y = (double)v.y;
    double z = (double)v.z;
    double length = sqrt(x * x + y * y + z * z);
    if (!(length > 0.0 && length <= 1e300))
    {
        return fallback;
    }
    return maniVec3Make((float)(x / length), (float)(y / length), (float)(z / length));
}

maniQuat maniQuatNormalize(maniQuat q)
{
    double x = (double)q.x;
    double y = (double)q.y;
    double z = (double)q.z;
    double w = (double)q.w;
    double length = sqrt(x * x + y * y + z * z + w * w);
    if (!(length > 0.0 && length <= 1e300))
    {
        return maniQuatIdentity();
    }
    return (maniQuat){(float)(x / length), (float)(y / length), (float)(z / length),
                      (float)(w / length)};
}

static maniQuat Negate(maniQuat q)
{
    return (maniQuat){-q.x, -q.y, -q.z, -q.w};
}

maniQuat maniQuatNlerp(maniQuat a, maniQuat b, float t)
{
    if (maniQuatDot(a, b) < 0.0f)
    {
        b = Negate(b);
    }
    maniQuat r = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z),
                  a.w + t * (b.w - a.w)};
    return maniQuatNormalize(r);
}

static float Length4(maniQuat q)
{
    return sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

maniQuat maniQuatSlerp(maniQuat a, maniQuat b, float t)
{
    if (maniQuatDot(a, b) < 0.0f)
    {
        b = Negate(b);
    }
    maniQuat difference = {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
    maniQuat sum = {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
    float theta = 2.0f * maniAtan2(Length4(difference), Length4(sum));
    // Below this angle the two weights are 1 - t and t to within rounding.
    if (theta < 1e-6f)
    {
        return maniQuatNlerp(a, b, t);
    }
    float inverseSin = 1.0f / maniSin(theta);
    float wa = maniSin((1.0f - t) * theta) * inverseSin;
    float wb = maniSin(t * theta) * inverseSin;
    maniQuat r = {wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z,
                  wa * a.w + wb * b.w};
    return maniQuatNormalize(r);
}

maniQuat maniQuatFromAxisAngle(maniVec3 axis, float angle)
{
    float s = 0.0f;
    float c = 0.0f;
    maniSinCos(angle * 0.5f, &s, &c);
    return (maniQuat){axis.x * s, axis.y * s, axis.z * s, c};
}

maniQuat maniQuatBetween(maniVec3 from, maniVec3 to)
{
    // The turn from from onto the unit half-vector h between the two is
    // half the turn onto to, so q = (from x h, from . h). Summing the
    // directions keeps the digits that 1 + from . to loses near opposite
    // directions; binary64 keeps the rest.
    double f[3] = {(double)from.x, (double)from.y, (double)from.z};
    double t[3] = {(double)to.x, (double)to.y, (double)to.z};
    // A binary32 unit vector is unit only to within 6e-8; near opposite
    // directions that difference would tilt the half-vector, so both are
    // made unit in binary64 first.
    double fl = sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    double tl = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    double h[3];
    for (int k = 0; k < 3; ++k)
    {
        f[k] /= fl;
        h[k] = f[k] + t[k] / tl;
    }
    double length = sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    // A zero or non-finite input makes length NaN, which lands here too.
    if (!(length > 1e-6))
    {
        // Half a turn about the axis least aligned with from, made
        // perpendicular to it.
        float ax = fabsf(from.x);
        float ay = fabsf(from.y);
        float az = fabsf(from.z);
        maniVec3 other = ax <= ay && ax <= az ? maniVec3Make(1.0f, 0.0f, 0.0f)
                         : ay <= az           ? maniVec3Make(0.0f, 1.0f, 0.0f)
                                              : maniVec3Make(0.0f, 0.0f, 1.0f);
        maniVec3 axis = maniVec3Normalize(maniVec3Cross(from, other), other);
        return (maniQuat){axis.x, axis.y, axis.z, 0.0f};
    }
    for (int k = 0; k < 3; ++k)
    {
        h[k] /= length;
    }
    double x = f[1] * h[2] - f[2] * h[1];
    double y = f[2] * h[0] - f[0] * h[2];
    double z = f[0] * h[1] - f[1] * h[0];
    double w = f[0] * h[0] + f[1] * h[1] + f[2] * h[2];
    double n = sqrt(x * x + y * y + z * z + w * w);
    return (maniQuat){(float)(x / n), (float)(y / n), (float)(z / n), (float)(w / n)};
}

bool maniAffineInverse(const maniAffine* a, maniAffine* inverseOut)
{
    double m[9];
    for (int i = 0; i < 9; ++i)
    {
        m[i] = (double)a->m[i];
    }
    // Cofactors of the column-major 3x3 linear part.
    double c00 = m[4] * m[8] - m[7] * m[5];
    double c01 = m[7] * m[2] - m[1] * m[8];
    double c02 = m[1] * m[5] - m[4] * m[2];
    double det = m[0] * c00 + m[3] * c01 + m[6] * c02;
    if (!(fabs(det) >= 1e-30 && fabs(det) <= 1e300))
    {
        return false;
    }
    double inv = 1.0 / det;
    double r[9] = {
        c00 * inv,
        c01 * inv,
        c02 * inv,
        (m[6] * m[5] - m[3] * m[8]) * inv,
        (m[0] * m[8] - m[6] * m[2]) * inv,
        (m[3] * m[2] - m[0] * m[5]) * inv,
        (m[3] * m[7] - m[6] * m[4]) * inv,
        (m[6] * m[1] - m[0] * m[7]) * inv,
        (m[0] * m[4] - m[3] * m[1]) * inv,
    };
    double t[3] = {(double)a->m[9], (double)a->m[10], (double)a->m[11]};
    maniAffine out;
    for (int i = 0; i < 9; ++i)
    {
        out.m[i] = (float)r[i];
    }
    for (int k = 0; k < 3; ++k)
    {
        out.m[9 + k] = (float)-(r[k] * t[0] + r[3 + k] * t[1] + r[6 + k] * t[2]);
    }
    *inverseOut = out;
    return true;
}
