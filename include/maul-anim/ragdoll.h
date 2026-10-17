// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ragdoll profiles (mani-0027): a skeleton's ragdoll as data a host maps
// onto its own physics engine: bodies on a subset of the bones, each
// with its frame, a primitive shape and a mass, and a swing-twist joint
// to its parent body with limits and an optional drive. A profile is
// built offline for one skeleton and opened in place; the library never
// simulates it.

#ifndef MAUL_ANIM_RAGDOLL_H
#define MAUL_ANIM_RAGDOLL_H

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A body's shape, at its frame's origin: a sphere (size.x the
    // radius), a capsule along the frame's y (size.x the radius, size.y
    // the distance between its caps' centers) or a box (size the half
    // extents).
    typedef uint32_t maniRagdollShape;
    enum
    {
        maniRagdollSphere = 0,
        maniRagdollCapsule = 1,
        maniRagdollBox = 2,
    };

    // An opened profile: a view over the caller's bytes, which must
    // outlive it.
    typedef struct maniRagdoll
    {
        const uint8_t* bodies;
        const uint8_t* names;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint32_t boneCount;
        uint32_t bodyCount;
        uint32_t nameBytes;
        uint32_t reserved;
        maniVec3 facing;
        maniVec3 up;
    } maniRagdoll;

    // A body as a host creates it. Frames are rigid (translation and unit
    // rotation). The root body (index 0) has no parent and no joint: its
    // joint fields are zero and its joint frames the identity.
    typedef struct maniRagdollBody
    {
        // The body's name, its bytes inside the profile (not terminated).
        const char* name;
        uint32_t nameLength;
        // The bone the body follows; its parent body's index, or
        // MANI_NO_BONE for the root. Parents come before children.
        uint32_t bone;
        uint32_t parent;
        maniRagdollShape shape;
        maniVec3 size;
        float mass;
        // The body's frame in its bone's model frame (scale left out).
        maniVec3 frameTranslation;
        maniQuat frameRotation;
        // The joint's frame in this body and in the parent body, equal in
        // the rest pose; twist turns about its x, swing about its y and z.
        maniVec3 jointTranslation;
        maniQuat jointRotation;
        maniVec3 parentJointTranslation;
        maniQuat parentJointRotation;
        // Twist within [twistMin, twistMax] (radians, in [-pi, pi]);
        // swing within an elliptic cone of half angles swingY and swingZ
        // (in [0, pi]).
        float twistMin;
        float twistMax;
        float swingY;
        float swingZ;
        // A drive toward the joint's target: stiffness (torque a radian),
        // damping (torque a radian a second) and the largest torque; all
        // 0 for none.
        float driveStiffness;
        float driveDamping;
        float driveMaxTorque;
    } maniRagdollBody;

    /// Opens a ragdoll profile file in place: checks the container and
    /// every record (bones within the skeleton's count and increasing,
    /// parents before children, one root, shapes, sizes and masses
    /// positive, frames finite with unit rotations, limits and drives in
    /// range, names, the root's facing and up axes unit and
    /// perpendicular).
    ///
    /// @param bytes      The file, 16-byte aligned.
    /// @param size       Its size in bytes; bytes past the file are ignored.
    /// @param ragdollOut Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenRagdoll(const void* bytes, size_t size,
                                                           maniRagdoll* ragdollOut);

    /// Returns a profile's identity: the hash of its sections.
    ///
    /// @param ragdoll  An opened profile, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRagdollIdentity(const maniRagdoll* ragdoll);

    /// Returns the identity of the skeleton a profile was built for.
    ///
    /// @param ragdoll  An opened profile, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniRagdollSkeletonIdentity(const maniRagdoll* ragdoll);

    /// Returns the number of bodies in a profile.
    ///
    /// @param ragdoll  An opened profile, or NULL.
    /// @return The count, at least 1; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniRagdollBodyCount(const maniRagdoll* ragdoll);

    /// Reads a body of a profile, in the order the profile keeps them
    /// (by bone, parents first).
    ///
    /// @param ragdoll  An opened profile.
    /// @param body     The body's index.
    /// @param bodyOut  Receives the body.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// an index past the bodies.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniRagdollReadBody(const maniRagdoll* ragdoll,
                                                           uint32_t body, maniRagdollBody* bodyOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_RAGDOLL_H
