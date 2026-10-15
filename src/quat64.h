// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binary64 quaternion operations for the offline side (x, y, z, w).

#ifndef MAUL_ANIM_SRC_QUAT64_H
#define MAUL_ANIM_SRC_QUAT64_H

// glTF's slerp of two unit quaternions by u, the short way: the angle
// from half-angle lengths, well conditioned at every angle; linear below
// 1e-9 radians.
void maniSlerpDouble(const double a[4], const double b[4], double u, double out[4]);

#endif // MAUL_ANIM_SRC_QUAT64_H
