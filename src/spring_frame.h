// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A spring joint's rest direction and its limit's default orientation
// (mani-0026), which the builder and the glTF importer both work out.

#ifndef MAUL_ANIM_SRC_SPRING_FRAME_H
#define MAUL_ANIM_SRC_SPRING_FRAME_H

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

// The head's rest direction to its tail (a bone below it), in the
// head's space: the tail's rest translation carried up through the
// bones between them, normalized (+y for none).
maniVec3 maniSpringRestDirection(const maniSkeleton* skeleton, uint32_t head, uint32_t tail);

// The shortest turn from +y to the unit axis; 180° about x for exactly
// -y.
maniQuat maniSpringTurnFromY(maniVec3 axis);

#endif // MAUL_ANIM_SRC_SPRING_FRAME_H
