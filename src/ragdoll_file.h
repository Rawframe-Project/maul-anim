// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ragdoll profile file's layout (mani-0027), shared by its view and
// its builder: kind 12 of the container. Records are little-endian.
// Bodies are ordered by bone, so a parent body comes before its children
// and the root is the first.

#ifndef MAUL_ANIM_SRC_RAGDOLL_FILE_H
#define MAUL_ANIM_SRC_RAGDOLL_FILE_H

#include <stdint.h>

#define MANI_RAGDOLL_FORMAT_VERSION 1u
#define MANI_RAGDOLL_SECTIONS       3u

enum
{
    // The skeleton's identity (u64); the bone count, body count and name
    // bytes (u32 each); the root body's facing and up axes (3 binary32
    // each); 0 (u32).
    maniRagdollTagHeader = 1,
    // MANI_RAGDOLL_BODY_SIZE each, laid out below.
    maniRagdollTagBodies = 2,
    // The bodies' names, UTF-8, end to end.
    maniRagdollTagNames = 3,
};

#define MANI_RAGDOLL_HEADER_SIZE 48u
#define MANI_RAGDOLL_BODY_SIZE   160u

// Body record offsets: u32 bone, parent (MANI_NO_BONE for the root),
// shape, name offset and length, 0; then binary32: the frame's
// translation (3) and rotation (4), the shape's size (3), the mass, the
// joint's frame in the body (3 and 4) and in the parent (3 and 4), twist
// min and max, swing y and z, drive stiffness, damping and maximum
// torque; two 0 words.
enum
{
    maniRagdollBodyBone = 0,
    maniRagdollBodyParent = 4,
    maniRagdollBodyShape = 8,
    maniRagdollBodyName = 12,
    maniRagdollBodyNameLength = 16,
    maniRagdollBodyFrame = 24,
    maniRagdollBodySize = 52,
    maniRagdollBodyMass = 64,
    maniRagdollBodyJoint = 68,
    maniRagdollBodyParentJoint = 96,
    maniRagdollBodyLimits = 124,
    maniRagdollBodyDrive = 140,
    maniRagdollBodyEnd = 152,
};

// The identity: maniHash64 over the sections in tag order, absent ones
// adding nothing.
uint64_t maniRagdollComputeIdentity(const uint8_t* const data[MANI_RAGDOLL_SECTIONS],
                                    const uint64_t sizes[MANI_RAGDOLL_SECTIONS]);

#endif // MAUL_ANIM_SRC_RAGDOLL_FILE_H
