// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The basis change (mani-0009). Output axis i is input axis p[i]
// times s[i]; every operation is a copy or a sign flip, so it is exact.

#include "gltf_basis.h"

#include <stdint.h>

typedef struct Basis
{
    uint32_t p[3];
    float s[3];
    // det B: the signs' product times the permutation's parity.
    float det;
} Basis;

static Basis Make(const maniGltfOptions* options)
{
    Basis b;
    for (uint32_t i = 0; i < 3; ++i)
    {
        b.p[i] = options->axes[i] % 3;
        b.s[i] = options->axes[i] >= 3 ? -1.0f : 1.0f;
    }
    // A cyclic order is even: (0 1 2), (1 2 0), (2 0 1).
    bool even = b.p[1] == (b.p[0] + 1) % 3;
    b.det = b.s[0] * b.s[1] * b.s[2] * (even ? 1.0f : -1.0f);
    return b;
}

bool maniGltfOptionsValid(const maniGltfOptions* options)
{
    bool seen[3] = {false, false, false};
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (options->axes[i] > maniAxisNegativeZ || seen[options->axes[i] % 3])
        {
            return false;
        }
        seen[options->axes[i] % 3] = true;
    }
    return options->foldPlacement <= 1;
}

maniAffine maniGltfIdentityAffine(void)
{
    maniAffine a = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    return a;
}

static maniVec3 Permute(const Basis* b, maniVec3 v, bool signs)
{
    const float in[3] = {v.x, v.y, v.z};
    float out[3];
    for (uint32_t i = 0; i < 3; ++i)
    {
        out[i] = signs ? b->s[i] * in[b->p[i]] : in[b->p[i]];
    }
    return (maniVec3){out[0], out[1], out[2]};
}

maniTransform maniGltfBasisTransform(const maniGltfOptions* options, maniTransform t)
{
    Basis b = Make(options);
    maniVec3 axis = Permute(&b, (maniVec3){t.rotation.x, t.rotation.y, t.rotation.z}, true);
    t.translation = Permute(&b, t.translation, true);
    t.rotation = (maniQuat){b.det * axis.x, b.det * axis.y, b.det * axis.z, t.rotation.w};
    t.scale = Permute(&b, t.scale, false);
    return t;
}

maniAffine maniGltfBasisAffine(const maniGltfOptions* options, maniAffine a)
{
    Basis b = Make(options);
    maniAffine out;
    for (uint32_t j = 0; j < 3; ++j)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            out.m[j * 3 + i] = b.s[i] * b.s[j] * a.m[b.p[j] * 3 + b.p[i]];
        }
        out.m[9 + j] = b.s[j] * a.m[9 + b.p[j]];
    }
    return out;
}
