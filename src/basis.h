// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The importers' basis change (mani-0009, mani-0014): a signed
// permutation B of axes applied as B G B^-1, exactly, to transforms and
// matrices, with translations also scaled by a unit factor.

#ifndef MAUL_ANIM_SRC_BASIS_H
#define MAUL_ANIM_SRC_BASIS_H

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// Output axis i is input axis p[i] times s[i]; det is det B; unit
// scales translations.
typedef struct maniBasis
{
    uint32_t p[3];
    float s[3];
    float det;
    float unit;
} maniBasis;

// Whether each axis is below 6 and x, y and z each appear once.
bool maniBasisAxesValid(const maniAxis axes[3]);

// The basis whose output axis i is input axis axes[i] (or its negative).
maniBasis maniBasisMake(const maniAxis axes[3], float unit);

maniAffine maniIdentityAffine(void);

// A local transform in the output's basis: translation permuted, signed
// and scaled, rotation's vector part permuted, signed and times det B,
// scale permuted.
maniTransform maniBasisTransform(const maniBasis* basis, maniTransform t);

// Three floats in the output's basis: a translation (or its tangent),
// permuted, signed and scaled; with `translation` false, a scale,
// permuted.
void maniBasisVector(const maniBasis* basis, float v[3], bool translation);

// A quaternion's four floats (or a rotation tangent's) in the output's
// basis: the vector part permuted, signed and times det B.
void maniBasisQuat(const maniBasis* basis, float q[4]);

// A matrix in the output's basis: B M B^-1, its translation scaled.
maniAffine maniBasisAffine(const maniBasis* basis, maniAffine a);

#endif // MAUL_ANIM_SRC_BASIS_H
