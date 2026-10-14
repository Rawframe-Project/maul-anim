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

#endif // MAUL_ANIM_SRC_DECOMPOSE_H
