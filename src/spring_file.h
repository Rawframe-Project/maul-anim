// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spring set file's layout (mani-0026), shared by its view, its
// runtime and its builder: kind 11 of the container. Records are
// little-endian. A spring's joints are its segments, each a head bone
// and the bone of the joint after it (its tail), so a chain of n joints
// has n - 1; each spring lists the colliders of its groups in group
// order without repeats. Values are per step of the step rate.

#ifndef MAUL_ANIM_SRC_SPRING_FILE_H
#define MAUL_ANIM_SRC_SPRING_FILE_H

#include <stdint.h>

#define MANI_SPRING_FORMAT_VERSION 2u
#define MANI_SPRING_SECTIONS       6u

enum
{
    // The skeleton's identity (u64), then u32 each: the bone, collider,
    // spring, joint and link counts, the name bytes; the reference rate
    // and the step rate (binary32), two zero words.
    maniSpringTagHeader = 1,
    // MANI_SPRING_COLLIDER_SIZE each: bone, shape, inside, 0 (u32 each);
    // offset, tail or unit normal (3 binary32 each), radius (binary32),
    // 0 (u32).
    maniSpringTagColliders = 2,
    // MANI_SPRING_SPRING_SIZE each: first joint, joint count, first link,
    // link count, center bone (MANI_NO_BONE for none), name offset, name
    // length, 0 (u32 each).
    maniSpringTagSprings = 3,
    // MANI_SPRING_JOINT_SIZE each: head bone, tail bone, limit (u32
    // each); then binary32: the forcing factor (a force's share of a
    // step per unit, finite and not negative), hit radius, drag,
    // stiffness, gravity (3), the
    // limit's angle and yaw (clamped to their ranges), the limit's frame
    // in the head's space (a unit quaternion x, y, z, w: the turn from +y
    // to the head's rest direction to its tail, then the limit's
    // rotation; the identity for none).
    maniSpringTagJoints = 4,
    // A collider index (u32) each: the springs' lists end to end.
    maniSpringTagLinks = 5,
    // The springs' names, UTF-8, end to end.
    maniSpringTagNames = 6,
};

#define MANI_SPRING_HEADER_SIZE   48u
#define MANI_SPRING_COLLIDER_SIZE 48u
#define MANI_SPRING_SPRING_SIZE   32u
#define MANI_SPRING_JOINT_SIZE    64u

// Joint record offsets.
enum
{
    maniSpringJointHead = 0,
    maniSpringJointTail = 4,
    maniSpringJointLimit = 8,
    maniSpringJointForcing = 12,
    maniSpringJointHitRadius = 16,
    maniSpringJointDrag = 20,
    maniSpringJointStiffness = 24,
    maniSpringJointGravity = 28,
    maniSpringJointAngle = 40,
    maniSpringJointYaw = 44,
    maniSpringJointFrame = 48,
};

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniSpringComputeIdentity(const uint8_t* const data[MANI_SPRING_SECTIONS],
                                   const uint64_t sizes[MANI_SPRING_SECTIONS]);

#endif // MAUL_ANIM_SRC_SPRING_FILE_H
