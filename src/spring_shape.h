// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring tails against colliders and limits.

#ifndef MAUL_ANIM_SPRING_SHAPE_H
#define MAUL_ANIM_SPRING_SHAPE_H

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stdint.h>

// A collider in the step's model space: a sphere's center (offset), a
// capsule's ends (offset, tail) or a plane's point (offset) and unit
// normal (tail); the radius scaled.
typedef struct maniSpringShape
{
    maniVec3 offset;
    maniVec3 tail;
    float radius;
    uint32_t shape;
    uint32_t inside;
} maniSpringShape;

// A file's binary32 and three of them at p.
float maniSpringFileFloat(const uint8_t* p);
maniVec3 maniSpringFileVec(const uint8_t* p);

// A collider record's shape in model space under its bone's model
// transform t.
maniSpringShape maniSpringShapeOf(const uint8_t* collider, maniTransform t);

// A file's binary32 and three of them at p.
float maniSpringFileFloat(const uint8_t* p);
maniVec3 maniSpringFileVec(const uint8_t* p);

// A collider record's shape in model space under its bone's model
// transform t.
maniSpringShape maniSpringShapeOf(const uint8_t* collider, maniTransform t);

// The largest of a scale's magnitudes.
float maniSpringLargestScale(maniVec3 s);

// Whether a tail of the joint's radius overlaps the shape; if so the
// direction to push it back along (unit) and the overlap (negative).
bool maniSpringCollide(const maniSpringShape* s, maniVec3 tail, float jointRadius,
                       maniVec3* directionOut, float* distanceOut);

// A unit direction in the limit's frame kept within the limit.
maniVec3 maniSpringLimitDirection(uint32_t limit, float angle, float yaw, maniVec3 direction);

#endif // MAUL_ANIM_SPRING_SHAPE_H
