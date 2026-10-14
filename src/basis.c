// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The basis change (mani-0009, mani-0014). Output axis i is input axis
// p[i] times s[i]; every operation is a copy or a sign flip, so it is
// exact, and only the unit's scaling of translations rounds.

#include "basis.h"

#include <stdint.h>

bool maniBasisAxesValid(const maniAxis axes[3])
{
    bool seen[3] = {false, false, false};
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (axes[i] > maniAxisNegativeZ || seen[axes[i] % 3])
        {
            return false;
        }
        seen[axes[i] % 3] = true;
    }
    return true;
}

maniBasis maniBasisMake(const maniAxis axes[3], float unit)
{
    maniBasis b;
    for (uint32_t i = 0; i < 3; ++i)
    {
        b.p[i] = axes[i] % 3;
        b.s[i] = axes[i] >= 3 ? -1.0f : 1.0f;
    }
    // A cyclic order is even: (0 1 2), (1 2 0), (2 0 1).
    bool even = b.p[1] == (b.p[0] + 1) % 3;
    b.det = b.s[0] * b.s[1] * b.s[2] * (even ? 1.0f : -1.0f);
    b.unit = unit;
    return b;
}

maniAffine maniIdentityAffine(void)
{
    maniAffine a = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    return a;
}

static maniVec3 Permute(const maniBasis* b, maniVec3 v, bool signs)
{
    const float in[3] = {v.x, v.y, v.z};
    float out[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        out[i] = signs ? b->s[i] * in[b->p[i]] : in[b->p[i]];
    }
    return (maniVec3){out[0], out[1], out[2]};
}

static maniVec3 Scale(const maniBasis* b, maniVec3 v)
{
    return (maniVec3){b->unit * v.x, b->unit * v.y, b->unit * v.z};
}

maniTransform maniBasisTransform(const maniBasis* basis, maniTransform t)
{
    maniVec3 axis = Permute(basis, (maniVec3){t.rotation.x, t.rotation.y, t.rotation.z}, true);
    t.translation = Scale(basis, Permute(basis, t.translation, true));
    t.rotation =
        (maniQuat){basis->det * axis.x, basis->det * axis.y, basis->det * axis.z, t.rotation.w};
    t.scale = Permute(basis, t.scale, false);
    return t;
}

void maniBasisVector(const maniBasis* basis, float v[3], bool translation)
{
    maniVec3 out = Permute(basis, (maniVec3){v[0], v[1], v[2]}, translation);
    out = translation ? Scale(basis, out) : out;
    v[0] = out.x;
    v[1] = out.y;
    v[2] = out.z;
}

void maniBasisQuat(const maniBasis* basis, float q[4])
{
    maniVec3 axis = Permute(basis, (maniVec3){q[0], q[1], q[2]}, true);
    q[0] = basis->det * axis.x;
    q[1] = basis->det * axis.y;
    q[2] = basis->det * axis.z;
}

maniAffine maniBasisAffine(const maniBasis* basis, maniAffine a)
{
    maniAffine out;
    for (uint32_t j = 0; j < 3; ++j)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            out.m[j * 3 + i] = basis->s[i] * basis->s[j] * a.m[basis->p[j] * 3 + basis->p[i]];
        }
        out.m[9 + j] = basis->unit * (basis->s[j] * a.m[9 + basis->p[j]]);
    }
    return out;
}
