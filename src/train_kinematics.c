// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Character space from a frame's Decompressor outputs; see
// train_kinematics.h. Matrices are column-major, three columns of three.

#include "train_kinematics.h"

#include "train_data.h"

#include "maul-anim/skeleton.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// A bone's work: its rotation's columns, the raw second column, the
// first column's and the cross's lengths, its scale and translation, its
// model columns and translation.
enum
{
    kRotation = 0,
    kSecond = 9,
    kFirstLength = 12,
    kCrossLength = 13,
    kScale = 14,
    kTranslation = 17,
    kModel = 20,
    kPosition = 29,
};

// Lengths below this are taken as it, so a degenerate column stays
// finite.
static const float kTiny = 1e-12f;

static void Cross(const float* a, const float* b, float* out)
{
    float x = a[1] * b[2] - a[2] * b[1];
    float y = a[2] * b[0] - a[0] * b[2];
    float z = a[0] * b[1] - a[1] * b[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static float Dot(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float Length(const float* a)
{
    float length = sqrtf(Dot(a, a));
    return length > kTiny ? length : kTiny;
}

// out = m v, m column-major.
static void Apply(const float* m, const float* v, float* out)
{
    for (uint32_t r = 0; r < 3; ++r)
    {
        out[r] = m[r] * v[0] + m[3 + r] * v[1] + m[6 + r] * v[2];
    }
}

// out = m^T v.
static void ApplyTransposed(const float* m, const float* v, float* out)
{
    for (uint32_t c = 0; c < 3; ++c)
    {
        out[c] = Dot(m + 3 * c, v);
    }
}

// Bone b's rotation, scale and translation from the outputs.
static void Local(const maniTrainData* d, const float* y, uint32_t b, uint32_t* scaled, float* w)
{
    const float* x = y + 9 * (size_t)b + 3;
    const float* second = y + 9 * (size_t)b + 6;
    float first = Length(x);
    for (uint32_t i = 0; i < 3; ++i)
    {
        w[kRotation + i] = x[i] / first;
    }
    float u[3];
    Cross(w + kRotation, second, u);
    float cross = Length(u);
    for (uint32_t i = 0; i < 3; ++i)
    {
        w[kRotation + 6 + i] = u[i] / cross;
    }
    Cross(w + kRotation + 6, w + kRotation, w + kRotation + 3);
    memcpy(w + kSecond, second, 3 * sizeof(float));
    w[kFirstLength] = first;
    w[kCrossLength] = cross;
    bool given = *scaled < d->scaledBoneCount && d->scaledBones[*scaled] == b;
    const float* scale =
        given ? y + 9 * (size_t)d->boneCount + 3 * (size_t)(*scaled)++ : d->scales + 3 * (size_t)b;
    memcpy(w + kScale, scale, 3 * sizeof(float));
    memcpy(w + kTranslation, y + 9 * (size_t)b, 3 * sizeof(float));
}

void maniTrainKinematics(const maniTrainData* d, const float* y, float* work, float* character)
{
    uint32_t scaled = 0;
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        float* w = work + (size_t)b * MANI_TRAIN_KINEMATIC_FLOATS;
        memset(w, 0, MANI_TRAIN_KINEMATIC_FLOATS * sizeof(float));
        Local(d, y, b, &scaled, w);
        float local[9];
        for (uint32_t k = 0; k < 9; ++k)
        {
            local[k] = w[kRotation + k] * w[kScale + k / 3];
        }
        uint32_t parent = d->parents[b];
        if (parent == MANI_NO_BONE)
        {
            memcpy(w + kModel, local, sizeof local);
            memcpy(w + kPosition, w + kTranslation, 3 * sizeof(float));
            continue;
        }
        const float* p = work + (size_t)parent * MANI_TRAIN_KINEMATIC_FLOATS;
        for (uint32_t c = 0; c < 3; ++c)
        {
            Apply(p + kModel, local + 3 * c, w + kModel + 3 * c);
        }
        Apply(p + kModel, w + kTranslation, w + kPosition);
        for (uint32_t i = 0; i < 3; ++i)
        {
            w[kPosition + i] += p[kPosition + i];
        }
    }
    const float* root = work;
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        const float* w = work + (size_t)b * MANI_TRAIN_KINEMATIC_FLOATS;
        float* out = character + (size_t)b * MANI_TRAIN_CHARACTER_FLOATS;
        float offset[3];
        for (uint32_t i = 0; i < 3; ++i)
        {
            offset[i] = w[kPosition + i] - root[kPosition + i];
        }
        ApplyTransposed(root + kRotation, offset, out);
        ApplyTransposed(root + kRotation, w + kModel, out + 3);
        ApplyTransposed(root + kRotation, w + kModel + 3, out + 6);
    }
}

// The character values' gradient into each bone's model columns and
// translation (g, 12 a bone) and bone 0's rotation (rootGradient).
static void FromCharacter(const maniTrainData* d, const float* work, const float* gradient,
                          float* g, float* rootGradient)
{
    const float* root = work;
    memset(g, 0, 12 * sizeof(float) * d->boneCount);
    memset(rootGradient, 0, 9 * sizeof(float));
    for (uint32_t b = 0; b < d->boneCount; ++b)
    {
        const float* w = work + (size_t)b * MANI_TRAIN_KINEMATIC_FLOATS;
        const float* q = gradient + (size_t)b * MANI_TRAIN_CHARACTER_FLOATS;
        float* gb = g + 12 * (size_t)b;
        float turned[3];
        Apply(root + kRotation, q, turned);
        for (uint32_t i = 0; i < 3; ++i)
        {
            gb[9 + i] += turned[i];
            g[9 + i] -= turned[i];
        }
        // A value is a column of bone 0's rotation dotted with a vector:
        // that rotation's gradient is the vector times the value's.
        for (uint32_t j = 0; j < 3; ++j)
        {
            for (uint32_t i = 0; i < 3; ++i)
            {
                float offset = w[kPosition + i] - root[kPosition + i];
                rootGradient[3 * j + i] +=
                    offset * q[j] + w[kModel + i] * q[3 + j] + w[kModel + 3 + i] * q[6 + j];
            }
        }
        Apply(root + kRotation, q + 3, gb);
        Apply(root + kRotation, q + 6, gb + 3);
    }
}

// Through the rotation's construction: the gradient by its columns into
// the gradient by the two outputs it was made from.
static void FromRotation(const float* w, float* gr, float* gx, float* gy)
{
    const float* c0 = w + kRotation;
    const float* c2 = w + kRotation + 6;
    float t[3];
    // c1 = c2 x c0.
    Cross(c0, gr + 3, t);
    for (uint32_t i = 0; i < 3; ++i)
    {
        gr[6 + i] += t[i];
    }
    Cross(gr + 3, c2, t);
    for (uint32_t i = 0; i < 3; ++i)
    {
        gr[i] += t[i];
    }
    // c2 = u / |u|, u = c0 x second.
    float along = Dot(c2, gr + 6);
    float gu[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        gu[i] = (gr[6 + i] - c2[i] * along) / w[kCrossLength];
    }
    Cross(w + kSecond, gu, t);
    for (uint32_t i = 0; i < 3; ++i)
    {
        gr[i] += t[i];
    }
    Cross(gu, c0, gy);
    // c0 = x / |x|.
    along = Dot(c0, gr);
    for (uint32_t i = 0; i < 3; ++i)
    {
        gx[i] = (gr[i] - c0[i] * along) / w[kFirstLength];
    }
}

// Through the hierarchy: bone b's model gradient (gb) into its local
// matrix's (gl) and translation's (gt), and into its parent's (gp).
static void ToParent(const float* w, const float* p, const float* gb, float* gp, float* gl,
                     float* gt)
{
    for (uint32_t c = 0; c < 3; ++c)
    {
        ApplyTransposed(p + kModel, gb + 3 * c, gl + 3 * c);
    }
    ApplyTransposed(p + kModel, gb + 9, gt);
    // Model = parent's times local, translation = parent's times t plus
    // the parent's: the parent's gradients.
    for (uint32_t c = 0; c < 3; ++c)
    {
        for (uint32_t r = 0; r < 3; ++r)
        {
            float sum = gb[9 + r] * w[kTranslation + c];
            for (uint32_t k = 0; k < 3; ++k)
            {
                sum += gb[3 * k + r] * (w[kRotation + 3 * k + c] * w[kScale + k]);
            }
            gp[3 * c + r] += sum;
        }
        gp[9 + c] += gb[9 + c];
    }
}

// Bone b's local matrix gradient into its rotation's (plus bone 0's own,
// for bone 0) and scale's, then into the outputs.
static void ToOutputs(const float* w, const float* gl, const float* gt, const float* rootGradient,
                      float* out, float* scaleOut)
{
    float gr[9];
    float gs[3];
    for (uint32_t c = 0; c < 3; ++c)
    {
        for (uint32_t r = 0; r < 3; ++r)
        {
            gr[3 * c + r] = gl[3 * c + r] * w[kScale + c];
        }
        gs[c] = Dot(w + kRotation + 3 * c, gl + 3 * c);
    }
    for (uint32_t k = 0; rootGradient != NULL && k < 9; ++k)
    {
        gr[k] += rootGradient[k];
    }
    float gx[3];
    float gy[3];
    FromRotation(w, gr, gx, gy);
    for (uint32_t i = 0; i < 3; ++i)
    {
        out[i] += gt[i];
        out[3 + i] += gx[i];
        out[6 + i] += gy[i];
    }
    for (uint32_t i = 0; scaleOut != NULL && i < 3; ++i)
    {
        scaleOut[i] += gs[i];
    }
}

void maniTrainKinematicsBackward(const maniTrainData* d, const float* work,
                                 const float* characterGradient, float* yGradient, float* scratch)
{
    float rootGradient[9];
    FromCharacter(d, work, characterGradient, scratch, rootGradient);
    uint32_t scaled = d->scaledBoneCount;
    for (uint32_t b = d->boneCount; b-- > 0;)
    {
        const float* w = work + (size_t)b * MANI_TRAIN_KINEMATIC_FLOATS;
        const float* gb = scratch + 12 * (size_t)b;
        float gl[9];
        float gt[3];
        uint32_t parent = d->parents[b];
        if (parent == MANI_NO_BONE)
        {
            memcpy(gl, gb, sizeof gl);
            memcpy(gt, gb + 9, sizeof gt);
        }
        else
        {
            ToParent(w, work + (size_t)parent * MANI_TRAIN_KINEMATIC_FLOATS, gb,
                     scratch + 12 * (size_t)parent, gl, gt);
        }
        float* scaleOut = NULL;
        if (scaled > 0 && d->scaledBones[scaled - 1] == b)
        {
            scaled -= 1;
            scaleOut = yGradient + 9 * (size_t)d->boneCount + 3 * (size_t)scaled;
        }
        ToOutputs(w, gl, gt, b == 0 ? rootGradient : NULL, yGradient + 9 * (size_t)b, scaleOut);
    }
}
