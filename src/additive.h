// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One difference applied to one transform, for the
// compressor's measure of additive clips.

#ifndef MAUL_ANIM_SRC_ADDITIVE_H
#define MAUL_ANIM_SRC_ADDITIVE_H

#include "maul-anim/base.h"

// t plus the difference d at weight w: t + w d_t, r nlerp(1, d_r, w) the
// short way, s (1 + w (d_s - 1)); maniAddPoses's arithmetic.
void maniAddTransform(maniTransform* t, const maniTransform* d, float w);

#endif // MAUL_ANIM_SRC_ADDITIVE_H
