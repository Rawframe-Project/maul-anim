// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The glTF options' basis change (mani-0009): basis.c's, glTF's units
// kept.

#include "gltf_basis.h"

#include "basis.h"

static maniBasis Make(const maniGltfOptions* options)
{
    return maniBasisMake(options->axes, 1.0f);
}

static bool Identity(const maniGltfOptions* options)
{
    return options->axes[0] == maniAxisX && options->axes[1] == maniAxisY &&
           options->axes[2] == maniAxisZ;
}

bool maniGltfOptionsValid(const maniGltfOptions* options)
{
    return maniBasisAxesValid(options->axes) && options->foldPlacement <= 1;
}

maniAffine maniGltfIdentityAffine(void)
{
    return maniIdentityAffine();
}

maniTransform maniGltfBasisTransform(const maniGltfOptions* options, maniTransform t)
{
    if (Identity(options))
    {
        return t;
    }
    maniBasis b = Make(options);
    return maniBasisTransform(&b, t);
}

void maniGltfBasisVector(const maniGltfOptions* options, float v[3], bool signs)
{
    maniBasis b = Make(options);
    maniBasisVector(&b, v, signs);
}

void maniGltfBasisQuat(const maniGltfOptions* options, float q[4])
{
    maniBasis b = Make(options);
    maniBasisQuat(&b, q);
}

void maniGltfBasisVectors(const maniGltfOptions* options, float* v, size_t count, bool signs)
{
    if (Identity(options))
    {
        return;
    }
    maniBasis b = Make(options);
    for (size_t i = 0; i < count; ++i)
    {
        maniBasisVector(&b, v + 3 * i, signs);
    }
}

void maniGltfBasisQuats(const maniGltfOptions* options, float* q, size_t count)
{
    if (Identity(options))
    {
        return;
    }
    maniBasis b = Make(options);
    for (size_t i = 0; i < count; ++i)
    {
        maniBasisQuat(&b, q + 4 * i);
    }
}

maniAffine maniGltfBasisAffine(const maniGltfOptions* options, maniAffine a)
{
    if (Identity(options))
    {
        return a;
    }
    maniBasis b = Make(options);
    return maniBasisAffine(&b, a);
}
