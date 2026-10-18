// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A spring tail against a collider and within a limit (mani-0026), as
// VRMC_springBone 1.0 and its extensions give them: the distance from a
// collider's surface to the tail less the joint's radius (negative when
// they overlap) and the direction to push it, for spheres and capsules
// kept out of or in, and planes; the tail's direction kept in a cone, a
// hinge or a spherical limit, the singular directions as the limit
// extension's table fixes them. The extended collider's capsule
// pseudocode reads its delta before setting it; the delta from the
// capsule's offset is meant.

#include "spring_shape.h"

#include "bytes.h"
#include "scalar.h"
#include "spring_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const maniVec3 kUp = {0.0f, 1.0f, 0.0f};

float maniSpringFileFloat(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

maniVec3 maniSpringFileVec(const uint8_t* p)
{
    return maniVec3Make(maniSpringFileFloat(p), maniSpringFileFloat(p + 4),
                        maniSpringFileFloat(p + 8));
}

float maniSpringLargestScale(maniVec3 s)
{
    float x = fabsf(s.x);
    float y = fabsf(s.y);
    float z = fabsf(s.z);
    float m = x > y ? x : y;
    return m > z ? m : z;
}

maniSpringShape maniSpringShapeOf(const uint8_t* collider, maniTransform t)
{
    const uint8_t* e = collider;
    maniAffine m = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    uint32_t shape = maniLoadU32(e + 4);
    maniVec3 tail = maniSpringFileVec(e + 28);
    // A normal turns with the inverse transpose: the cofactors' signs
    // kept by the determinant's, so no division by a zero scale.
    maniVec3 k = t.scale;
    float sign = k.x * k.y * k.z < 0.0f ? -1.0f : 1.0f;
    maniVec3 normal = maniVec3Scale(
        maniVec3Make(tail.x * k.y * k.z, tail.y * k.x * k.z, tail.z * k.x * k.y), sign);
    return (maniSpringShape){
        maniAffineTransformPoint(&m, maniSpringFileVec(e + 16)),
        shape == maniColliderPlane ? maniVec3Normalize(maniQuatRotate(t.rotation, normal), kUp)
                                   : maniAffineTransformPoint(&m, tail),
        maniSpringFileFloat(e + 40) * maniSpringLargestScale(t.scale), shape, maniLoadU32(e + 8)};
}

// The tail's offset from the shape's nearest axis point: from the offset
// for a sphere, from the segment for a capsule.
static maniVec3 Delta(const maniSpringShape* s, maniVec3 tail)
{
    maniVec3 delta = maniVec3Sub(tail, s->offset);
    if (s->shape != maniColliderCapsule)
    {
        return delta;
    }
    maniVec3 axis = maniVec3Sub(s->tail, s->offset);
    float length = maniVec3Dot(axis, axis);
    float along = maniVec3Dot(axis, delta);
    if (!(length > 0.0f) || along < 0.0f)
    {
        return delta;
    }
    return along > length ? maniVec3Sub(delta, axis)
                          : maniVec3Sub(delta, maniVec3Scale(axis, along / length));
}

bool maniSpringCollide(const maniSpringShape* s, maniVec3 tail, float jointRadius,
                       maniVec3* directionOut, float* distanceOut)
{
    maniVec3 delta = maniVec3Sub(tail, s->offset);
    if (s->shape == maniColliderPlane)
    {
        *directionOut = s->tail;
        *distanceOut = maniVec3Dot(delta, s->tail) - jointRadius;
        return *distanceOut < 0.0f;
    }
    delta = Delta(s, tail);
    float length = maniVec3Length(delta);
    maniVec3 direction = maniVec3Normalize(delta, kUp);
    if (s->inside != 0)
    {
        *directionOut = maniVec3Scale(direction, -1.0f);
        *distanceOut = s->radius - jointRadius - length;
    }
    else
    {
        *directionOut = direction;
        *distanceOut = length - s->radius - jointRadius;
    }
    return *distanceOut < 0.0f;
}

// The square root of a value rounding may have left just below zero.
static float Root(float v)
{
    return v > 0.0f ? sqrtf(v) : 0.0f;
}

static maniVec3 Cone(maniVec3 d, float angle)
{
    float c = maniCos(angle);
    if (!(d.y < c))
    {
        return d;
    }
    float side = Root(1.0f - c * c);
    float horizontal = 1.0f - d.y * d.y;
    if (!(horizontal > 0.0f))
    {
        // Straight down -y: the boundary on the +z side.
        return maniVec3Make(0.0f, c, side);
    }
    float scale = side / sqrtf(horizontal);
    return maniVec3Make(d.x * scale, c, d.z * scale);
}

static maniVec3 Hinge(maniVec3 d, float angle)
{
    float projected = d.y * d.y + d.z * d.z;
    if (!(projected > 0.0f))
    {
        // Along ±x: +y.
        return kUp;
    }
    float length = sqrtf(projected);
    maniVec3 h = maniVec3Make(0.0f, d.y / length, d.z / length);
    float c = maniCos(angle);
    if (!(h.y < c))
    {
        return h;
    }
    float side = Root(1.0f - c * c);
    return maniVec3Make(0.0f, c, h.z < 0.0f ? -side : side);
}

static maniVec3 Spherical(maniVec3 d, float pitchLimit, float yawLimit)
{
    float pitch = 0.0f;
    if (d.y == -1.0f)
    {
        // Straight down: the boundary on the +z side.
        pitch = 3.14159265358979f;
    }
    else if (fabsf(d.x) != 1.0f)
    {
        pitch = maniAtan2(d.z, d.y);
    }
    float yaw = maniAtan2(d.x, Root(1.0f - d.x * d.x));
    pitch = fabsf(pitch) > pitchLimit ? copysignf(pitchLimit, pitch) : pitch;
    yaw = fabsf(yaw) > yawLimit ? copysignf(yawLimit, yaw) : yaw;
    float cy = maniCos(yaw);
    return maniVec3Make(maniSin(yaw), cy * maniCos(pitch), cy * maniSin(pitch));
}

maniVec3 maniSpringLimitDirection(uint32_t limit, float angle, float yaw, maniVec3 direction)
{
    switch (limit)
    {
    case maniSpringLimitCone:
        return Cone(direction, angle);
    case maniSpringLimitHinge:
        return Hinge(direction, angle);
    case maniSpringLimitSpherical:
        return Spherical(direction, angle, yaw);
    default:
        return direction;
    }
}
