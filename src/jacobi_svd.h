// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One-sided Jacobi singular value decomposition, for the
// full-body SDLS step.

#ifndef MAUL_ANIM_SRC_JACOBI_SVD_H
#define MAUL_ANIM_SRC_JACOBI_SVD_H

#include <stdint.h>

// The most sweeps, LAPACK's sgesvj's.
#define MANI_JACOBI_MAX_SWEEPS 30u
// The most columns: six rows for each of the full-body solve's at most
// 32 effectors.
#define MANI_JACOBI_MAX_COLUMNS 192u
// Binary32's unit roundoff, LAPACK's SLAMCH('E').
#define MANI_JACOBI_EPSILON 5.9604645e-8f

// Decomposes the n-by-m matrix a (m <= MANI_JACOBI_MAX_COLUMNS columns of
// n each, one after
// another) in place: afterwards a is the input times w, its columns
// orthogonal, each sigma_i times a unit vector; w (m by m, the same
// layout) orthogonal; sigma the columns' lengths, unsorted. With the
// input the transpose of J, J = W diag(sigma) (a's columns / sigma)^T:
// w's columns are J's left singular vectors, a's columns over sigma its
// right ones. Returns the sweeps taken.
uint32_t maniJacobiSvd(float* a, uint32_t n, uint32_t m, float* w, float* sigma);

#endif // MAUL_ANIM_SRC_JACOBI_SVD_H
