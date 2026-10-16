// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Exact geometric predicates on binary32 points, for the builders: every
// coordinate scaled by 2^149 is an integer below 2^277 in magnitude, and
// the orientation and in-circle determinants are computed on those
// integers without rounding, so a triangulation is the same on every
// platform, ties included.

#ifndef MAUL_ANIM_SRC_EXACT_H
#define MAUL_ANIM_SRC_EXACT_H

// The sign (-1, 0 or 1) of the orientation of a, b and c (each x then
// y): 1 when they turn anticlockwise, 0 when on one line.
int maniExactOrient(const float* a, const float* b, const float* c);

// The sign of d against the circle through a, b and c, anticlockwise: 1
// strictly inside, 0 on it, -1 outside.
int maniExactInCircle(const float* a, const float* b, const float* c, const float* d);

#endif // MAUL_ANIM_SRC_EXACT_H
