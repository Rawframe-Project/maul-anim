// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX's local transform (mani-0014), in binary64 as ufbx evaluates it:
// T Roff Rp Rpre R Rpost^-1 Rp^-1 Soff Sp S Sp^-1 is one TRS, its
// rotation Rpre R Rpost^-1, its scale S, its translation what the
// pivots and offsets give. Angles are reduced in degrees, exactly, so
// right angles give exact sines.

#include "fbx_transform.h"

#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_value.h"
#include "scalar.h"

#include <math.h>
#include <string.h>

// pi / 180 rounded to binary64.
static const double kRadiansPerDegree = 0.017453292519943295;

void maniFbxSinCosDegrees(double degrees, double* sinOut, double* cosOut)
{
    // fmod is exact; a quarter's offset from its start is exact too
    // (Sterbenz), so only the conversion to radians rounds.
    double a = fmod(degrees, 360.0);
    a = a < 0.0 ? a + 360.0 : a;
    a = a >= 360.0 ? 0.0 : a;
    uint32_t quarter = a >= 270.0 ? 3 : a >= 180.0 ? 2 : a >= 90.0 ? 1 : 0;
    double r = (a - 90.0 * (double)quarter) * kRadiansPerDegree;
    double s = 0.0;
    double c = 1.0;
    maniSinCosQuarterDouble(r, &s, &c);
    double sines[4] = {s, c, -s, -c};
    double cosines[4] = {c, -s, -c, s};
    *sinOut = sines[quarter];
    *cosOut = cosines[quarter];
}

void maniFbxQuatMul(const double a[4], const double b[4], double out[4])
{
    double x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    double y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    double z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    double w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = w;
}

// The axes of each order, the first applied first.
static const uint8_t kOrders[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 2, 0},
                                      {1, 0, 2}, {2, 0, 1}, {2, 1, 0}};

void maniFbxEulerQuat(const double degrees[3], uint32_t order, double q[4])
{
    q[0] = q[1] = q[2] = 0.0;
    q[3] = 1.0;
    for (uint32_t k = 0; k < 3; ++k)
    {
        uint32_t axis = kOrders[order][k];
        double about[4] = {0.0, 0.0, 0.0, 1.0};
        maniFbxSinCosDegrees(0.5 * degrees[axis], &about[axis], &about[3]);
        maniFbxQuatMul(about, q, q);
    }
}

void maniFbxQuatRotate(const double q[4], const double v[3], double out[3])
{
    // v + 2w (u x v) + 2 u x (u x v), u the vector part.
    double cx = q[1] * v[2] - q[2] * v[1];
    double cy = q[2] * v[0] - q[0] * v[2];
    double cz = q[0] * v[1] - q[1] * v[0];
    double x = v[0] + 2.0 * (q[3] * cx + q[1] * cz - q[2] * cy);
    double y = v[1] + 2.0 * (q[3] * cy + q[2] * cx - q[0] * cz);
    double z = v[2] + 2.0 * (q[3] * cz + q[0] * cy - q[1] * cx);
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

maniFbxMatrix maniFbxTrsMatrix(const maniFbxTrs* t)
{
    const double* q = t->rotation;
    double x = q[0];
    double y = q[1];
    double z = q[2];
    double w = q[3];
    double r[9] = {
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w),       2.0 * (x * z - y * w),
        2.0 * (x * y - z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w),
        2.0 * (x * z + y * w),       2.0 * (y * z - x * w),       1.0 - 2.0 * (x * x + y * y)};
    maniFbxMatrix m;
    for (uint32_t c = 0; c < 3; ++c)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            m.m[3 * c + i] = r[3 * c + i] * t->scale[c];
        }
        m.m[9 + c] = t->translation[c];
    }
    return m;
}

maniFbxMatrix maniFbxMatrixMul(const maniFbxMatrix* a, const maniFbxMatrix* b)
{
    maniFbxMatrix m;
    for (uint32_t c = 0; c < 4; ++c)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            double v = a->m[i] * b->m[3 * c] + a->m[3 + i] * b->m[3 * c + 1] +
                       a->m[6 + i] * b->m[3 * c + 2];
            m.m[3 * c + i] = c == 3 ? v + a->m[9 + i] : v;
        }
    }
    return m;
}

bool maniFbxMatrixInverse(const maniFbxMatrix* a, maniFbxMatrix* out)
{
    const double* m = a->m;
    // Cofactors of the 3x3 part, transposed: the adjugate.
    double adj[9] = {
        m[4] * m[8] - m[5] * m[7], m[2] * m[7] - m[1] * m[8], m[1] * m[5] - m[2] * m[4],
        m[5] * m[6] - m[3] * m[8], m[0] * m[8] - m[2] * m[6], m[2] * m[3] - m[0] * m[5],
        m[3] * m[7] - m[4] * m[6], m[1] * m[6] - m[0] * m[7], m[0] * m[4] - m[1] * m[3]};
    double det = m[0] * adj[0] + m[3] * adj[1] + m[6] * adj[2];
    if (!(det != 0.0) || !isfinite(1.0 / det))
    {
        return false;
    }
    maniFbxMatrix inv;
    for (uint32_t i = 0; i < 9; ++i)
    {
        inv.m[i] = adj[i] / det;
    }
    for (uint32_t i = 0; i < 3; ++i)
    {
        inv.m[9 + i] = -(inv.m[i] * m[9] + inv.m[3 + i] * m[10] + inv.m[6 + i] * m[11]);
    }
    for (uint32_t i = 0; i < 12; ++i)
    {
        if (!isfinite(inv.m[i]))
        {
            return false;
        }
    }
    *out = inv;
    return true;
}

maniAffine maniFbxMatrixAffine(const maniFbxMatrix* a)
{
    maniAffine out;
    for (uint32_t i = 0; i < 12; ++i)
    {
        out.m[i] = (float)a->m[i];
    }
    return out;
}

maniTransform maniFbxTrsTransform(const maniFbxTrs* t)
{
    return (maniTransform){
        {(float)t->translation[0], (float)t->translation[1], (float)t->translation[2]},
        {(float)t->rotation[0], (float)t->rotation[1], (float)t->rotation[2],
         (float)t->rotation[3]},
        {(float)t->scale[0], (float)t->scale[1], (float)t->scale[2]}};
}

static maniImportResult Refuse(const maniFbx* fbx, uint32_t property, uint32_t object)
{
    uint64_t at = property < fbx->propertyCount ? fbx->properties[property].offset
                                                : fbx->nodes[fbx->objects[object].node].nameOffset;
    return maniFbxFail(maniImportReasonProperty, at, object);
}

// A vector property, its default when the object and its template lack
// it.
static maniImportResult Vector(const maniFbx* fbx, uint32_t object, const char* name,
                               double fallback, double out[3])
{
    uint32_t entry = maniFbxFindProperty(fbx, object, name, strlen(name));
    for (uint32_t k = 0; k < 3; ++k)
    {
        out[k] = fallback;
        uint32_t value = maniFbxPropertyValue(fbx, entry, k);
        if (entry != MANI_FBX_NONE && (!maniFbxNumber(fbx, value, &out[k]) || !isfinite(out[k])))
        {
            return Refuse(fbx, value != MANI_FBX_NONE ? value : maniFbxNodeProperty(fbx, entry, 0),
                          object);
        }
    }
    return maniFbxSuccess();
}

// An integer property in [0, maximum], its default when absent.
static maniImportResult Integer(const maniFbx* fbx, uint32_t object, const char* name,
                                int64_t maximum, int64_t fallback, int64_t* out)
{
    uint32_t entry = maniFbxFindProperty(fbx, object, name, strlen(name));
    *out = fallback;
    uint32_t value = maniFbxPropertyValue(fbx, entry, 0);
    if (entry != MANI_FBX_NONE && !maniFbxInteger(fbx, value, 0, maximum, out))
    {
        return Refuse(fbx, value != MANI_FBX_NONE ? value : maniFbxNodeProperty(fbx, entry, 0),
                      object);
    }
    return maniFbxSuccess();
}

// The vectors of the chain, in its order of application.
enum
{
    kTranslation,
    kRotationOffset,
    kRotationPivot,
    kPreRotation,
    kRotation,
    kPostRotation,
    kScalingOffset,
    kScalingPivot,
    kScaling,
    kVectors
};

static const char* const kVectorNames[kVectors] = {
    "Lcl Translation", "RotationOffset", "RotationPivot", "PreRotation", "Lcl Rotation",
    "PostRotation",    "ScalingOffset",  "ScalingPivot",  "Lcl Scaling"};

// The rotation Rpre R Rpost^-1, or R alone outside the rotation space.
static void Rotation(double v[kVectors][3], uint32_t order, bool space, double q[4])
{
    maniFbxEulerQuat(v[kRotation], space ? order : 0, q);
    if (!space)
    {
        return;
    }
    double pre[4];
    double post[4];
    maniFbxEulerQuat(v[kPreRotation], 0, pre);
    maniFbxEulerQuat(v[kPostRotation], 0, post);
    post[0] = -post[0];
    post[1] = -post[1];
    post[2] = -post[2];
    maniFbxQuatMul(q, post, q);
    maniFbxQuatMul(pre, q, q);
}

static maniImportResult ReadVectors(const maniFbx* fbx, uint32_t object, double v[kVectors][3])
{
    maniImportResult r = maniFbxSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < kVectors; ++k)
    {
        r = Vector(fbx, object, kVectorNames[k], k == kScaling ? 1.0 : 0.0, v[k]);
    }
    return r;
}

maniImportResult maniFbxModelLocal(const maniFbx* fbx, uint32_t object, maniFbxLocal* out)
{
    double v[kVectors][3];
    int64_t order = 0;
    int64_t active = 1;
    int64_t limitOnly = 0;
    int64_t inherit = maniFbxInheritRSrs;
    maniImportResult r = ReadVectors(fbx, object, v);
    if (r.result == mani_success)
    {
        r = Integer(fbx, object, "RotationOrder", 6, 0, &order);
    }
    if (r.result == mani_success)
    {
        r = Integer(fbx, object, "RotationActive", 1, 1, &active);
    }
    if (r.result == mani_success)
    {
        r = Integer(fbx, object, "RotationSpaceForLimitOnly", 1, 0, &limitOnly);
    }
    if (r.result == mani_success)
    {
        r = Integer(fbx, object, "InheritType", 2, maniFbxInheritRSrs, &inherit);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    maniFbxTrs* t = &out->trs;
    // Spheric (6) is XYZ.
    Rotation(v, order == 6 ? 0 : (uint32_t)order, active != 0 && limitOnly == 0, t->rotation);
    memcpy(t->scale, v[kScaling], sizeof t->scale);
    // Applied right to left, as ufbx adds them, for the same rounding:
    // -S Sp + Sp + Soff - Rp, rotated, + Rp + Roff + T.
    double inner[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        inner[i] = -v[kScaling][i] * v[kScalingPivot][i] + v[kScalingPivot][i] +
                   v[kScalingOffset][i] - v[kRotationPivot][i];
    }
    maniFbxQuatRotate(t->rotation, inner, t->translation);
    for (uint32_t i = 0; i < 3; ++i)
    {
        t->translation[i] =
            t->translation[i] + v[kRotationPivot][i] + v[kRotationOffset][i] + v[kTranslation][i];
    }
    out->inherit = (uint32_t)inherit;
    return maniFbxSuccess();
}
