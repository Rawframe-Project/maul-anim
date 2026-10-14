// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The import options' basis change (mani-0009): a signed
// permutation B of axes applied as B G B^-1, exactly, to transforms and
// matrices.

#ifndef MAUL_ANIM_SRC_GLTF_BASIS_H
#define MAUL_ANIM_SRC_GLTF_BASIS_H

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdbool.h>

// Axes each below 6 and a permutation of x, y and z; foldPlacement 0 or 1.
bool maniGltfOptionsValid(const maniGltfOptions* options);

maniAffine maniGltfIdentityAffine(void);

// A local transform in the output's basis: translation permuted and
// signed, rotation's vector part as well and times det B, scale permuted.
maniTransform maniGltfBasisTransform(const maniGltfOptions* options, maniTransform t);

// Three floats in the output's basis: a translation (or its tangent),
// permuted and signed; with `signs` false, a scale, permuted.
void maniGltfBasisVector(const maniGltfOptions* options, float v[3], bool signs);

// A quaternion's four floats (or a rotation tangent's) in the output's
// basis: the vector part permuted, signed and times det B.
void maniGltfBasisQuat(const maniGltfOptions* options, float q[4]);

// A matrix in the output's basis: B M B^-1.
maniAffine maniGltfBasisAffine(const maniGltfOptions* options, maniAffine a);

#endif // MAUL_ANIM_SRC_GLTF_BASIS_H
