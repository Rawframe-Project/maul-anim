// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skinning jobs. Every method makes each vertex's matrix, then
// applies it to the vertex's streams. Linear: the palette matrices summed
// at their weights in the influences' order (the heaviest first, stopping
// at the first weight of 0). Dual quaternion: the scale and shear parts
// summed the same way, the dual quaternions blended in the first's
// hemisphere and normalized, the matrix [R S | t]. Blended: the two at
// FBX's per-vertex weight. Centers of rotation: the dual quaternion's
// rotation and scale, the translation carrying the scaled center where
// the rigid parts' linear blend takes it. Binary32 in a fixed order, so
// the same bits on every platform.

#include "maul-anim/skinning.h"

#include "bytes.h"
#include "decompose.h"
#include "fpenv.h"
#include "influence_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

static bool StreamValid(const maniSkinStream* s)
{
    size_t width = s->kind == maniStreamTangent ? 16 : 12;
    return s->in != NULL && s->out != NULL && s->kind <= maniStreamTangent &&
           s->inStride >= width && s->outStride >= width;
}

// The job's arguments checked before anything is written; the palettes
// the method reads given.
static bool JobValid(const maniSkinJob* job, bool linear, bool dual)
{
    if (job == NULL || job->influences == NULL || (linear && job->palette == NULL) ||
        (dual && job->dualQuaternions == NULL) || (job->streams == NULL && job->streamCount > 0) ||
        job->paletteCount != job->influences->skinJointCount ||
        job->first > job->influences->vertexCount ||
        job->count > job->influences->vertexCount - job->first)
    {
        return false;
    }
    for (uint32_t i = 0; i < job->streamCount; ++i)
    {
        if (!StreamValid(&job->streams[i]))
        {
            return false;
        }
    }
    return true;
}

// A vertex's matrix: its influences' palette matrices at their weights;
// false (m untouched) for a vertex without weight.
static bool Blend(const maniSkinJob* job, uint32_t vertex, float m[12])
{
    const maniInfluences* v = job->influences;
    uint32_t per = v->groupCount * MANI_INFLUENCE_GROUP;
    size_t base = (size_t)vertex * per;
    float w = LoadF32(v->weights + 4 * base);
    if (w == 0.0f)
    {
        return false;
    }
    const float* p = job->palette[maniLoadU16(v->joints + 2 * base)].m;
    for (int k = 0; k < 12; ++k)
    {
        m[k] = p[k] * w;
    }
    for (uint32_t slot = 1; slot < per; ++slot)
    {
        w = LoadF32(v->weights + 4 * (base + slot));
        if (w == 0.0f)
        {
            break;
        }
        p = job->palette[maniLoadU16(v->joints + 2 * (base + slot))].m;
        for (int k = 0; k < 12; ++k)
        {
            m[k] += p[k] * w;
        }
    }
    return true;
}

// R times S into m's 3x3 part, S's rows at scale[0..2], [4..6], [8..10].
static void RotateScale(const maniAffine* r, const float scale[12], float m[12])
{
    for (int c = 0; c < 3; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            m[3 * c + k] =
                (r->m[k] * scale[c] + r->m[3 + k] * scale[4 + c]) + r->m[6 + k] * scale[8 + c];
        }
    }
}

// A dual quaternion palette entry: real, dual, S's rows.
#define DUAL_FLOATS 20

// A vertex's dual quaternion matrix: S summed at the weights, the dual
// quaternions summed in the first's hemisphere, normalized, as [R S | t];
// false (m untouched) for a vertex without weight.
static bool DualMatrix(const maniSkinJob* job, uint32_t vertex, float m[12])
{
    const maniInfluences* v = job->influences;
    uint32_t per = v->groupCount * MANI_INFLUENCE_GROUP;
    size_t base = (size_t)vertex * per;
    float real[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float dual[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float scale[12] = {0.0f};
    const float* lead = NULL;
    for (uint32_t slot = 0; slot < per; ++slot)
    {
        float w = LoadF32(v->weights + 4 * (base + slot));
        if (w == 0.0f)
        {
            break;
        }
        const float* d =
            job->dualQuaternions + DUAL_FLOATS * (size_t)maniLoadU16(v->joints + 2 * (base + slot));
        lead = lead == NULL ? d : lead;
        float dot = ((d[0] * lead[0] + d[1] * lead[1]) + d[2] * lead[2]) + d[3] * lead[3];
        float sw = dot < 0.0f ? -w : w;
        for (int k = 0; k < 4; ++k)
        {
            real[k] += d[k] * sw;
            dual[k] += d[4 + k] * sw;
        }
        for (int k = 0; k < 12; ++k)
        {
            scale[k] += d[8 + k] * w;
        }
    }
    if (lead == NULL)
    {
        return false;
    }
    float squared =
        ((real[0] * real[0] + real[1] * real[1]) + real[2] * real[2]) + real[3] * real[3];
    // The lead's weight is above 0 and every term agrees with it, so the
    // sum's length is at least that weight's.
    float inverse = 1.0f / sqrtf(squared);
    maniQuat q = {real[0] * inverse, real[1] * inverse, real[2] * inverse, real[3] * inverse};
    float e[4] = {dual[0] * inverse, dual[1] * inverse, dual[2] * inverse, dual[3] * inverse};
    // t = 2 (e q*), the vector part.
    float t[3] = {2.0f * (((e[0] * q.w - e[3] * q.x) - e[1] * q.z) + e[2] * q.y),
                  2.0f * (((e[1] * q.w - e[3] * q.y) - e[2] * q.x) + e[0] * q.z),
                  2.0f * (((e[2] * q.w - e[3] * q.z) - e[0] * q.y) + e[1] * q.x)};
    maniAffine r = maniAffineFromTrs((maniVec3){0.0f, 0.0f, 0.0f}, q, (maniVec3){1.0f, 1.0f, 1.0f});
    RotateScale(&r, scale, m);
    for (int c = 0; c < 3; ++c)
    {
        m[9 + c] = t[c];
    }
    return true;
}

// A vertex's matrix about its center of rotation: R S, and the
// translation sum w_j (R_j S c + t_j) - R S c; false for a vertex without
// weight.
static bool CenterMatrix(const maniSkinJob* job, uint32_t vertex, float m[12])
{
    const maniInfluences* v = job->influences;
    uint32_t per = v->groupCount * MANI_INFLUENCE_GROUP;
    size_t base = (size_t)vertex * per;
    float real[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float rigid[12] = {0.0f};
    float scale[12] = {0.0f};
    const float* lead = NULL;
    for (uint32_t slot = 0; slot < per; ++slot)
    {
        float w = LoadF32(v->weights + 4 * (base + slot));
        if (w == 0.0f)
        {
            break;
        }
        uint32_t joint = maniLoadU16(v->joints + 2 * (base + slot));
        const float* d = job->dualQuaternions + DUAL_FLOATS * (size_t)joint;
        lead = lead == NULL ? d : lead;
        float dot = ((d[0] * lead[0] + d[1] * lead[1]) + d[2] * lead[2]) + d[3] * lead[3];
        float sw = dot < 0.0f ? -w : w;
        for (int k = 0; k < 4; ++k)
        {
            real[k] += d[k] * sw;
        }
        // The rigid part [R_j | t_j] summed at the weights.
        maniAffine r =
            maniAffineFromTrs((maniVec3){0.0f, 0.0f, 0.0f}, (maniQuat){d[0], d[1], d[2], d[3]},
                              (maniVec3){1.0f, 1.0f, 1.0f});
        const float* t = job->palette[joint].m + 9;
        for (int k = 0; k < 9; ++k)
        {
            rigid[k] += r.m[k] * w;
        }
        for (int k = 0; k < 3; ++k)
        {
            rigid[9 + k] += t[k] * w;
        }
        for (int k = 0; k < 12; ++k)
        {
            scale[k] += d[8 + k] * w;
        }
    }
    if (lead == NULL)
    {
        return false;
    }
    float squared =
        ((real[0] * real[0] + real[1] * real[1]) + real[2] * real[2]) + real[3] * real[3];
    float inverse = 1.0f / sqrtf(squared);
    maniQuat q = {real[0] * inverse, real[1] * inverse, real[2] * inverse, real[3] * inverse};
    maniAffine r = maniAffineFromTrs((maniVec3){0.0f, 0.0f, 0.0f}, q, (maniVec3){1.0f, 1.0f, 1.0f});
    const uint8_t* cp = v->centers + 12 * (size_t)vertex;
    const float c[3] = {LoadF32(cp), LoadF32(cp + 4), LoadF32(cp + 8)};
    float cs[3];
    for (int i = 0; i < 3; ++i)
    {
        cs[i] = (scale[4 * i] * c[0] + scale[4 * i + 1] * c[1]) + scale[4 * i + 2] * c[2];
    }
    RotateScale(&r, scale, m);
    for (int k = 0; k < 3; ++k)
    {
        float carried =
            ((rigid[k] * cs[0] + rigid[3 + k] * cs[1]) + rigid[6 + k] * cs[2]) + rigid[9 + k];
        float turned = (r.m[k] * cs[0] + r.m[3 + k] * cs[1]) + r.m[6 + k] * cs[2];
        m[9 + k] = carried - turned;
    }
    return true;
}

// v to unit length; zero when its length is 0 or past binary32's range.
static void Unit(float v[3])
{
    float squared = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    bool usable = squared > 0.0f && squared <= FLT_MAX;
    float inverse = usable ? 1.0f / sqrtf(squared) : 0.0f;
    for (int k = 0; k < 3; ++k)
    {
        v[k] = usable ? v[k] * inverse : 0.0f;
    }
}

// The 3x3 part's cofactor matrix times its determinant's sign: the
// inverse transpose's direction, without a division.
static void Cofactor(const float m[12], float c[9])
{
    const float* a = m;
    const float* b = m + 3;
    const float* d = m + 6;
    const float bd[3] = {b[1] * d[2] - b[2] * d[1], b[2] * d[0] - b[0] * d[2],
                         b[0] * d[1] - b[1] * d[0]};
    const float da[3] = {d[1] * a[2] - d[2] * a[1], d[2] * a[0] - d[0] * a[2],
                         d[0] * a[1] - d[1] * a[0]};
    const float ab[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                         a[0] * b[1] - a[1] * b[0]};
    float det = (a[0] * bd[0] + a[1] * bd[1]) + a[2] * bd[2];
    float sign = det < 0.0f ? -1.0f : 1.0f;
    for (int k = 0; k < 3; ++k)
    {
        c[k] = sign * bd[k];
        c[3 + k] = sign * da[k];
        c[6 + k] = sign * ab[k];
    }
}

// x times the 3x3 columns of m (and plus its translation for a point).
static void Apply(const float m[12], const float x[3], bool point, float out[3])
{
    for (int k = 0; k < 3; ++k)
    {
        float r = (m[k] * x[0] + m[3 + k] * x[1]) + m[6 + k] * x[2];
        out[k] = point ? r + m[9 + k] : r;
    }
}

// One stream of one vertex: in to out by the vertex's matrix (or copied
// when it has none).
static void SkinStream(const maniSkinStream* s, uint32_t i, const float* m, const float* c)
{
    const uint8_t* in = (const uint8_t*)s->in + (size_t)i * s->inStride;
    uint8_t* out = (uint8_t*)s->out + (size_t)i * s->outStride;
    float x[4];
    size_t width = s->kind == maniStreamTangent ? 4 : 3;
    memcpy(x, in, width * sizeof(float));
    float y[4] = {x[0], x[1], x[2], width == 4 ? x[3] : 0.0f};
    if (m != NULL)
    {
        Apply(s->kind == maniStreamNormal ? c : m, x, s->kind == maniStreamPoint, y);
        if (s->kind != maniStreamPoint)
        {
            Unit(y);
        }
    }
    memcpy(out, y, width * sizeof(float));
}

typedef enum Method
{
    kLinear,
    kDual,
    kBlended,
    kCenters,
} Method;

// A vertex's matrix by the method; false for a vertex without weight.
static bool VertexMatrix(const maniSkinJob* job, Method method, uint32_t vertex, float m[12])
{
    if (method == kLinear)
    {
        return Blend(job, vertex, m);
    }
    if (method == kDual)
    {
        return DualMatrix(job, vertex, m);
    }
    if (method == kCenters)
    {
        return CenterMatrix(job, vertex, m);
    }
    float b = maniInfluencesDualQuaternionWeight(job->influences, vertex);
    if (b == 0.0f || b == 1.0f)
    {
        return b == 0.0f ? Blend(job, vertex, m) : DualMatrix(job, vertex, m);
    }
    float d[12];
    if (!Blend(job, vertex, m) || !DualMatrix(job, vertex, d))
    {
        return false;
    }
    float a = 1.0f - b;
    for (int k = 0; k < 12; ++k)
    {
        m[k] = m[k] * a + d[k] * b;
    }
    return true;
}

static maniResult Run(const maniSkinJob* job, Method method)
{
    if (!JobValid(job, method != kDual, method != kLinear) ||
        (method == kCenters && job->influences->centers == NULL))
    {
        return mani_errorInvalid;
    }
    bool normals = false;
    for (uint32_t i = 0; i < job->streamCount; ++i)
    {
        normals |= job->streams[i].kind == maniStreamNormal;
    }
    for (uint32_t i = 0; i < job->count; ++i)
    {
        float m[12];
        float c[9];
        bool weighed = VertexMatrix(job, method, job->first + i, m);
        if (weighed && normals)
        {
            Cofactor(m, c);
        }
        for (uint32_t s = 0; s < job->streamCount; ++s)
        {
            SkinStream(&job->streams[s], i, weighed ? m : NULL, c);
        }
    }
    return mani_success;
}

static MANI_NOINLINE maniResult SkinLinearWork(const maniSkinJob* job)
{
    return Run(job, kLinear);
}

maniResult maniSkinLinear(const maniSkinJob* job)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SkinLinearWork(job);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult SkinDualQuaternionWork(const maniSkinJob* job)
{
    return Run(job, kDual);
}

maniResult maniSkinDualQuaternion(const maniSkinJob* job)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SkinDualQuaternionWork(job);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult SkinBlendedWork(const maniSkinJob* job)
{
    return Run(job, kBlended);
}

maniResult maniSkinBlended(const maniSkinJob* job)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SkinBlendedWork(job);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult SkinCentersWork(const maniSkinJob* job)
{
    return Run(job, kCenters);
}

maniResult maniSkinCenters(const maniSkinJob* job)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SkinCentersWork(job);
    maniFloatEnvLeave(&env);
    return result;
}

// One matrix's dual quaternion entry: the polar rotation R, the dual part
// half (t, 0) times R's quaternion, S = R^T A in binary64 then rounded.
static bool DualEntry(const maniAffine* a, float* out)
{
    maniTransform split;
    if (!maniDecompose(a, &split))
    {
        return false;
    }
    maniQuat q = split.rotation;
    maniAffine r = maniAffineFromTrs((maniVec3){0.0f, 0.0f, 0.0f}, q, (maniVec3){1.0f, 1.0f, 1.0f});
    const float* t = a->m + 9;
    const float entry[8] = {q.x,
                            q.y,
                            q.z,
                            q.w,
                            0.5f * ((t[0] * q.w + t[1] * q.z) - t[2] * q.y),
                            0.5f * ((t[1] * q.w + t[2] * q.x) - t[0] * q.z),
                            0.5f * ((t[2] * q.w + t[0] * q.y) - t[1] * q.x),
                            -0.5f * ((t[0] * q.x + t[1] * q.y) + t[2] * q.z)};
    memcpy(out, entry, sizeof entry);
    // S's row i, column c: R's column i dotted with A's column c.
    for (int i = 0; i < 3; ++i)
    {
        for (int c = 0; c < 3; ++c)
        {
            double v = (double)r.m[3 * i] * (double)a->m[3 * c] +
                       (double)r.m[3 * i + 1] * (double)a->m[3 * c + 1] +
                       (double)r.m[3 * i + 2] * (double)a->m[3 * c + 2];
            out[8 + 4 * i + c] = (float)v;
        }
        out[8 + 4 * i + 3] = 0.0f;
    }
    return true;
}

static MANI_NOINLINE maniResult PaletteDualQuaternionsWork(const maniAffine* palette,
                                                           uint32_t count, float* out)
{
    if (count > 0 && (palette == NULL || out == NULL))
    {
        return mani_errorInvalid;
    }
    for (uint32_t j = 0; j < count; ++j)
    {
        maniTransform split;
        if (!maniDecompose(&palette[j], &split))
        {
            return mani_errorInvalid;
        }
    }
    for (uint32_t j = 0; j < count; ++j)
    {
        (void)DualEntry(&palette[j], out + DUAL_FLOATS * (size_t)j);
    }
    return mani_success;
}

maniResult maniPaletteDualQuaternions(const maniAffine* palette, uint32_t count, float* out)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = PaletteDualQuaternionsWork(palette, count, out);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniResult PaletteRowsWork(const maniAffine* palette, uint32_t count,
                                                float* rows)
{
    if (count > 0 && (palette == NULL || rows == NULL))
    {
        return mani_errorInvalid;
    }
    for (uint32_t j = 0; j < count; ++j)
    {
        const float* m = palette[j].m;
        float* r = rows + 12 * (size_t)j;
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                r[4 * row + col] = m[3 * col + row];
            }
        }
    }
    return mani_success;
}

maniResult maniPaletteRows(const maniAffine* palette, uint32_t count, float* rows)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = PaletteRowsWork(palette, count, rows);
    maniFloatEnvLeave(&env);
    return result;
}
