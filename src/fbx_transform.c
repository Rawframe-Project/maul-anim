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

// The arc sine of t in [-1, 1].
static double Asin(double t)
{
    return maniAtan2Double(t, sqrt((1.0 - t) * (1.0 + t)));
}

// ufbx's formulas, order by order: the middle axis b by an arc sine of t,
// the outer axes by arc tangents of the parts p (a's) and r (c's), or at
// gimbal lock the last axis from g alone and the first 0.
typedef struct EulerParts
{
    double t;
    double p[2];
    double r[2];
    double g[2];
} EulerParts;

static EulerParts Parts(const double q[4], uint32_t order)
{
    double x = q[0];
    double y = q[1];
    double z = q[2];
    double w = q[3];
    double t = 0.0;
    switch (order)
    {
    case 0: // XYZ: x, y middle, z.
        t = 2.0 * (w * y - x * z);
        return (EulerParts){t,
                            {2.0 * (w * z + x * y), 2.0 * (w * w + x * x) - 1.0},
                            {-2.0 * (w * x + y * z), 2.0 * (w * w + z * z) - 1.0},
                            {-2.0 * t * (w * x - y * z), t * (2.0 * w * y + 2.0 * x * z)}};
    case 1: // XZY
        t = 2.0 * (w * z + x * y);
        return (EulerParts){t,
                            {2.0 * (w * y - x * z), 2.0 * (w * w + x * x) - 1.0},
                            {-2.0 * (w * x - y * z), 2.0 * (w * w + y * y) - 1.0},
                            {2.0 * t * (w * x + y * z), -t * (2.0 * x * y - 2.0 * w * z)}};
    case 2: // YZX
        t = 2.0 * (w * z - x * y);
        return (EulerParts){t,
                            {2.0 * (w * x + y * z), 2.0 * (w * w + y * y) - 1.0},
                            {-2.0 * (w * y + x * z), 2.0 * (w * w + x * x) - 1.0},
                            {-2.0 * t * (w * y - x * z), t * (2.0 * w * z + 2.0 * x * y)}};
    case 3: // YXZ
        t = 2.0 * (w * x + y * z);
        return (EulerParts){t,
                            {2.0 * (w * z - x * y), 2.0 * (w * w + y * y) - 1.0},
                            {-2.0 * (w * y - x * z), 2.0 * (w * w + z * z) - 1.0},
                            {2.0 * t * (w * y + x * z), -t * (2.0 * y * z - 2.0 * w * x)}};
    case 4: // ZXY
        t = 2.0 * (w * x - y * z);
        return (EulerParts){t,
                            {2.0 * (w * y + x * z), 2.0 * (w * w + z * z) - 1.0},
                            {-2.0 * (w * z + x * y), 2.0 * (w * w + y * y) - 1.0},
                            {-2.0 * t * (w * z - x * y), t * (2.0 * w * x + 2.0 * y * z)}};
    default: // ZYX
        t = 2.0 * (w * y + x * z);
        return (EulerParts){t,
                            {2.0 * (w * x - y * z), 2.0 * (w * w + z * z) - 1.0},
                            {-2.0 * (w * z - x * y), 2.0 * (w * w + x * x) - 1.0},
                            {2.0 * t * (w * z + x * y), -t * (2.0 * x * z - 2.0 * w * y)}};
    }
}

void maniFbxQuatEuler(const double q[4], uint32_t order, double degrees[3])
{
    // The axes each order applies first, second and third.
    uint32_t a = kOrders[order][0];
    uint32_t b = kOrders[order][1];
    uint32_t c = kOrders[order][2];
    EulerParts e = Parts(q, order);
    double radians[3];
    if (fabs(e.t) < 0.999999999)
    {
        radians[b] = Asin(e.t);
        radians[c] = maniAtan2Double(e.p[0], e.p[1]);
        radians[a] = -maniAtan2Double(e.r[0], e.r[1]);
    }
    else
    {
        radians[b] = e.t < 0.0 ? -1.5707963267948966 : 1.5707963267948966;
        radians[c] = maniAtan2Double(e.g[0], e.g[1]);
        radians[a] = 0.0;
    }
    for (uint32_t k = 0; k < 3; ++k)
    {
        degrees[k] = radians[k] * 57.295779513082323;
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

// A vector property's entry read, its default for no entry (the object
// and its template lack it).
static maniImportResult VectorAt(const maniFbx* fbx, uint32_t object, uint32_t entry,
                                 double fallback, double out[3])
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        out[k] = fallback;
        uint32_t value = maniFbxPropertyValue(fbx, entry, k);
        // maniFbxNumber refuses values not finite.
        if (entry != MANI_FBX_NONE && !maniFbxNumber(fbx, value, &out[k]))
        {
            return Refuse(fbx, value != MANI_FBX_NONE ? value : maniFbxNodeProperty(fbx, entry, 0),
                          object);
        }
    }
    return maniFbxSuccess();
}

static maniImportResult Vector(const maniFbx* fbx, uint32_t object, const char* name,
                               double fallback, double out[3])
{
    return VectorAt(fbx, object, maniFbxFindProperty(fbx, object, name, strlen(name)), fallback,
                    out);
}

// An integer property's entry read in [0, maximum], its default for no
// entry.
static maniImportResult IntegerAt(const maniFbx* fbx, uint32_t object, uint32_t entry,
                                  int64_t maximum, int64_t fallback, int64_t* out)
{
    *out = fallback;
    uint32_t value = maniFbxPropertyValue(fbx, entry, 0);
    if (entry != MANI_FBX_NONE && !maniFbxInteger(fbx, value, 0, maximum, out))
    {
        return Refuse(fbx, value != MANI_FBX_NONE ? value : maniFbxNodeProperty(fbx, entry, 0),
                      object);
    }
    return maniFbxSuccess();
}

const char* const maniFbxLocalVectorNames[maniFbxLocalVectors] = {
    "Lcl Translation", "RotationOffset", "RotationPivot", "PreRotation", "Lcl Rotation",
    "PostRotation",    "ScalingOffset",  "ScalingPivot",  "Lcl Scaling"};

const char* const maniFbxLocalIntegerNames[maniFbxLocalIntegers] = {
    "RotationOrder", "RotationActive", "RotationSpaceForLimitOnly", "InheritType"};

const int64_t maniFbxLocalIntegerLimits[maniFbxLocalIntegers] = {6, 1, 1, 2};

// The rotation Rpre R Rpost^-1, or R alone outside the rotation space.
static void Rotation(const double v[maniFbxLocalVectors][3], uint32_t order, bool space,
                     double q[4])
{
    maniFbxEulerQuat(v[maniFbxLocalRotation], space ? order : 0, q);
    if (!space)
    {
        return;
    }
    double pre[4];
    double post[4];
    maniFbxEulerQuat(v[maniFbxLocalPreRotation], 0, pre);
    maniFbxEulerQuat(v[maniFbxLocalPostRotation], 0, post);
    post[0] = -post[0];
    post[1] = -post[1];
    post[2] = -post[2];
    maniFbxQuatMul(q, post, q);
    maniFbxQuatMul(pre, q, q);
}

maniImportResult maniFbxReadLocalInputs(const maniFbx* fbx, uint32_t object,
                                        maniFbxTemplateCache* cache, maniFbxLocalInputs* out)
{
    static const int64_t kDefaults[maniFbxLocalIntegers] = {0, 1, 0, maniFbxInheritRSrs};
    enum
    {
        NAMES = maniFbxLocalVectors + maniFbxLocalIntegers
    };
    const char* names[NAMES];
    for (uint32_t k = 0; k < NAMES; ++k)
    {
        names[k] = k < maniFbxLocalVectors ? maniFbxLocalVectorNames[k]
                                           : maniFbxLocalIntegerNames[k - maniFbxLocalVectors];
    }
    uint32_t entries[NAMES];
    maniFbxFindProperties(fbx, object, names, NAMES, cache, entries);
    maniImportResult r = maniFbxSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < maniFbxLocalVectors; ++k)
    {
        r = VectorAt(fbx, object, entries[k], k == maniFbxLocalScaling ? 1.0 : 0.0,
                     out->vectors[k]);
    }
    for (uint32_t k = 0; r.result == mani_success && k < maniFbxLocalIntegers; ++k)
    {
        r = IntegerAt(fbx, object, entries[maniFbxLocalVectors + k], maniFbxLocalIntegerLimits[k],
                      kDefaults[k], &out->integers[k]);
    }
    // Before 6.0 the Lcl values are the channels' defaults (mani-0017);
    // later files have no such rests.
    static const uint32_t kLcl[3] = {maniFbxLocalTranslation, maniFbxLocalRotation,
                                     maniFbxLocalScaling};
    for (uint32_t k = 0; k < 3; ++k)
    {
        (void)maniFbxLegacyRest(fbx, object, k, out->vectors[kLcl[k]]);
    }
    return r;
}

void maniFbxComposeLocal(const maniFbxLocalInputs* in, maniFbxLocal* out)
{
    const double (*v)[3] = in->vectors;
    int64_t order = in->integers[maniFbxLocalOrder];
    bool space = in->integers[maniFbxLocalActive] != 0 && in->integers[maniFbxLocalLimitOnly] == 0;
    maniFbxTrs* t = &out->trs;
    // Spheric (6) is XYZ.
    Rotation(v, order == 6 ? 0 : (uint32_t)order, space, t->rotation);
    memcpy(t->scale, v[maniFbxLocalScaling], sizeof t->scale);
    // Applied right to left, as ufbx adds them, for the same rounding:
    // -S Sp + Sp + Soff - Rp, rotated, + Rp + Roff + T.
    double inner[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        inner[i] = -v[maniFbxLocalScaling][i] * v[maniFbxLocalScalingPivot][i] +
                   v[maniFbxLocalScalingPivot][i] + v[maniFbxLocalScalingOffset][i] -
                   v[maniFbxLocalRotationPivot][i];
    }
    maniFbxQuatRotate(t->rotation, inner, t->translation);
    for (uint32_t i = 0; i < 3; ++i)
    {
        t->translation[i] = t->translation[i] + v[maniFbxLocalRotationPivot][i] +
                            v[maniFbxLocalRotationOffset][i] + v[maniFbxLocalTranslation][i];
    }
    out->inherit = (uint32_t)in->integers[maniFbxLocalInherit];
}

maniImportResult maniFbxModelLocal(const maniFbx* fbx, uint32_t object, maniFbxTemplateCache* cache,
                                   maniFbxLocal* out)
{
    maniFbxLocalInputs in;
    maniImportResult r = maniFbxReadLocalInputs(fbx, object, cache, &in);
    if (r.result == mani_success)
    {
        maniFbxComposeLocal(&in, out);
    }
    return r;
}

maniImportResult maniFbxGeometricTransform(const maniFbx* fbx, uint32_t object, maniFbxTrs* out)
{
    double rotation[3];
    maniImportResult r = Vector(fbx, object, "GeometricTranslation", 0.0, out->translation);
    if (r.result == mani_success)
    {
        r = Vector(fbx, object, "GeometricRotation", 0.0, rotation);
    }
    if (r.result == mani_success)
    {
        r = Vector(fbx, object, "GeometricScaling", 1.0, out->scale);
    }
    if (r.result == mani_success)
    {
        maniFbxEulerQuat(rotation, 0, out->rotation);
    }
    return r;
}
