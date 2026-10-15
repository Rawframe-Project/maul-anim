// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skinning jobs (mani-0021). Linear blend: each vertex's palette matrices
// summed at their weights in the influences' order (the heaviest first,
// stopping at the first weight of 0), applied to its streams. Binary32 in
// a fixed order, so the same bits on every platform.

#include "maul-anim/skinning.h"

#include "bytes.h"
#include "influence_file.h"

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

// The job's arguments checked before anything is written.
static bool JobValid(const maniSkinJob* job)
{
    if (job == NULL || job->influences == NULL || job->palette == NULL ||
        (job->streams == NULL && job->streamCount > 0) ||
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

maniResult maniSkinLinear(const maniSkinJob* job)
{
    if (!JobValid(job))
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
        bool weighed = Blend(job, job->first + i, m);
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

maniResult maniPaletteRows(const maniAffine* palette, uint32_t count, float* rows)
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
