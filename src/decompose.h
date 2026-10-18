// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A 3x4 affine matrix split into translation, rotation and scale
// (mani-0004): the polar decomposition's rotation, mirrors as a negative
// x scale, degenerate matrices without refusal.

#ifndef MAUL_ANIM_SRC_DECOMPOSE_H
#define MAUL_ANIM_SRC_DECOMPOSE_H

#include "maul-anim/base.h"

#include <stdbool.h>

// Splits a finite matrix; false, with out untouched, when a component is
// not finite.
bool maniDecompose(const maniAffine* matrix, maniTransform* out);

// Splits a matrix when the parts compose back within 1e-4 of its largest
// column (mani-0009): false for a shear TRS cannot hold, or a component
// not finite.
bool maniDecomposeExact(const maniAffine* matrix, maniTransform* out);

// A matrix's rigid part, its translation and maniDecompose's rotation
// (mani-0027): when every two columns are orthogonal within a relative
// 1e-6 the polar factor is the columns over their lengths (one negated
// for a mirror, as maniDecompose's), taken without iterating; otherwise
// maniDecompose's. The identity rotation for a matrix not finite.
void maniDecomposeRigid(const maniAffine* matrix, maniVec3* translationOut, maniQuat* rotationOut);

#endif // MAUL_ANIM_SRC_DECOMPOSE_H
