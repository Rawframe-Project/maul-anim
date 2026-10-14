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

maniAffine maniGltfBasisAffine(const maniGltfOptions* options, maniAffine a)
{
    maniBasis b = Make(options);
    return maniBasisAffine(&b, a);
}
