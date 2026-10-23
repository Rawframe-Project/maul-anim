// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint limits: a bone's turn from its rest split into a
// swing and a twist about its axis, each bounded; a hinge keeps only
// the turn about its axis, within its range.

#ifndef MAUL_ANIM_SRC_IK_LIMIT_H
#define MAUL_ANIM_SRC_IK_LIMIT_H

#include "maul-anim/base.h"
#include "maul-anim/ik.h"

#include <stdbool.h>

// Whether a limit is of a known kind with its numbers in range.
bool maniIkLimitValid(const maniJointLimit* limit);

// The turn r (a bone's rotation relative to its rest, unit) within the
// limit, for a bone of unit axis (zero for none); r itself for NULL or
// no limit.
maniQuat maniIkLimit(const maniJointLimit* limit, maniVec3 axis, maniQuat r);

#endif // MAUL_ANIM_SRC_IK_LIMIT_H
