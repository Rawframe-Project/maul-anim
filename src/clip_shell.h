// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The shell metric of compression, in binary64: points at a
// distance along each bone's three axes, carried to the skeleton's space
// through the hierarchy by two poses; the error is the farthest any of
// them moves.

#ifndef MAUL_ANIM_SRC_CLIP_SHELL_H
#define MAUL_ANIM_SRC_CLIP_SHELL_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

// A bone's matrix in the skeleton's space: its parent's times its local
// T R S, column-major 3x4 (x, y, z axes, then the origin).
typedef struct maniShellMatrix
{
    double m[12];
} maniShellMatrix;

// A local T R S as a matrix, and a product of two.
maniShellMatrix maniShellLocal(maniTransform t);

// A bone's local as the shells measure it: itself, or an additive clip's
// difference applied to the bone's rest at weight 1.
maniTransform maniShellAbsolute(const maniSkeleton* skeleton, uint32_t bone, bool additive,
                                maniTransform t);
maniShellMatrix maniShellMul(const maniShellMatrix* a, const maniShellMatrix* b);

// Every bone's matrix of a pose, parents first.
void maniShellModels(const maniSkeleton* skeleton, const maniPose* pose, maniShellMatrix* out);

// The farthest one of the three points at distance d on a bone's axes
// lies apart in two of its matrices.
double maniShellError(const maniShellMatrix* a, const maniShellMatrix* b, double distance);

#endif // MAUL_ANIM_SRC_CLIP_SHELL_H
