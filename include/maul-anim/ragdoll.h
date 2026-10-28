// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ragdoll profiles: a skeleton's ragdoll as data a host maps
// onto its own physics engine: bodies on a subset of the bones, each
// with its frame, a primitive shape and a mass, and a swing-twist joint
// to its parent body with limits and an optional drive. A profile is
// built offline for one skeleton and opened in place; the library never
// simulates it. Lengths are the skeleton's units, angles radians, times
// seconds; masses and drives are the host engine's units, passed
// through unconverted.

#ifndef MAUL_ANIM_RAGDOLL_H
#define MAUL_ANIM_RAGDOLL_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
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
        size_t nameLength;
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
        // Twist within [twistMin, twistMax] (radians, -pi <= twistMin <=
        // twistMax <= pi; equal, the twist held there); swing within an
        // elliptic cone of half angles swingY and swingZ (each in [0, pi]:
        // 0 locks that axis, pi leaves it free).
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
    /// byte found wrong; `mani_errorVersion` for any other format version.
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

    // A body's place in the world (its frame's origin and rotation) and
    // its motion there: meters a second and radians a second about the
    // world's axes.
    typedef struct maniRagdollBodyState
    {
        maniVec3 position;
        maniQuat rotation;
        maniVec3 linearVelocity;
        maniVec3 angularVelocity;
    } maniRagdollBodyState;

    // A joint's motor target: the joint's frame in the child body turned
    // in its frame in the parent body, and that turn's angular velocity
    // about the parent joint frame's axes.
    typedef struct maniRagdollJointTarget
    {
        maniQuat rotation;
        maniVec3 angularVelocity;
    } maniRagdollJointTarget;

    /// Places a profile's bodies where a pose puts them: each body's
    /// frame on its bone's model frame (its scale left out) under the
    /// character's placement (its scale left out too: a profile is in the
    /// skeleton's units). With the pose before, its placement and the
    /// seconds between, each body's velocities are the differences over
    /// that time, the angular one the shortest turn's (so at most pi over
    /// seconds: a bone turning more than half a turn in one step reads as
    /// the shorter turn the other way); without, they are 0. Kinematic
    /// bodies and world-space drives take these. On a cut pass what the
    /// step means: no previous pose on the first frame or a clip switched
    /// without a blend (the bodies at rest); the current placement as
    /// previousPlacement on a teleport (the animation's motion alone);
    /// pose as previous for the placement's motion alone.
    ///
    /// @param ragdoll    An opened profile.
    /// @param pose       A pose of the profile's skeleton, its model
    ///                   matrices current.
    /// @param placement  The character in the world; NULL for the
    ///                   identity.
    /// @param previous   The pose a moment before, its model current; NULL
    ///                   for no velocities.
    /// @param previousPlacement  Its placement; NULL for the identity.
    /// @param seconds    The time between the two, above 0 and finite when
    ///                   previous is given.
    /// @param bodiesOut  Receives a state per body, in the profile's order.
    /// @param count      The number of states bodiesOut holds.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// pose of another skeleton, a difference or one whose model is not
    /// current, a placement not finite, seconds out of range;
    /// `mani_errorCapacity` when count is under the body count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniRagdollBodiesFromPose(
        const maniRagdoll* ragdoll, const maniPose* pose, const maniTransform* placement,
        const maniPose* previous, const maniTransform* previousPlacement, float seconds,
        maniRagdollBodyState* bodiesOut, uint32_t count);

    /// Works out each joint's motor target from bodies' states: the
    /// child's joint frame in the parent's, and the child's angular
    /// velocity less the parent's about the parent joint frame's axes.
    /// The root's entry is the identity and 0.
    ///
    /// @param ragdoll    An opened profile.
    /// @param bodies     A state per body, in the profile's order.
    /// @param jointsOut  Receives a target per body.
    /// @param count      The number of targets jointsOut holds (and of
    ///                   states bodies holds).
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorCapacity` when count is under the body count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniRagdollJointTargets(const maniRagdoll* ragdoll,
                                                               const maniRagdollBodyState* bodies,
                                                               maniRagdollJointTarget* jointsOut,
                                                               uint32_t count);

    /// Makes a pose from bodies' places: a bone with a body takes its
    /// model frame from the body's, the body's frame taken off, under
    /// the placement, and its local scale from the reference; a bone
    /// without one keeps the reference's local transform below its
    /// parent. The model matrices are composed. Velocities are not read.
    ///
    /// @param ragdoll    An opened profile.
    /// @param skeleton   Its skeleton.
    /// @param bodies     A state per body, in the profile's order: positions
    ///                   finite, rotations of length 1 within a thousandth.
    /// @param count      The number of states bodies holds.
    /// @param placement  The character in the world; NULL for the identity.
    /// @param reference  A pose of the skeleton for the bones without
    ///                   bodies and the scales; may be poseOut.
    /// @param poseOut    Receives the pose.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton or pose of another skeleton, a difference, a placement or
    /// a body's place not finite or a rotation not of length 1;
    /// `mani_errorCapacity` when count is under the body count.
    /// @par Thread safety
    /// Safe from any thread; the poses are used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniRagdollPoseFromBodies(
        const maniRagdoll* ragdoll, const maniSkeleton* skeleton,
        const maniRagdollBodyState* bodies, uint32_t count, const maniTransform* placement,
        const maniPose* reference, maniPose* poseOut);

    // Where a ragdoll gets up: the ground's plane (a point on it and its
    // unit normal, in the world) and the character's model forward and
    // up axes (unit, perpendicular), which name the placement's frame.
    typedef struct maniRagdollGround
    {
        maniVec3 point;
        maniVec3 normal;
        maniVec3 forward;
        maniVec3 up;
    } maniRagdollGround;

    // The placement to get up at and how the body lies.
    typedef struct maniRagdollGetUp
    {
        maniTransform placement;
        bool faceUp;
    } maniRagdollGetUp;

    /// Works out where a ragdoll gets up. The body lies face up when the
    /// root body's facing axis (out of the chest) points along the
    /// normal or across it. The heading is the root body's up axis
    /// (toward the head) on the ground's plane, reversed face up, plus
    /// its facing axis on the plane weighed by how upright the up axis
    /// stands (its component along the normal), so the character stands
    /// looking where its head lay face down and toward its feet face up
    /// (on its back, front or side), and where its chest faces upright;
    /// when that sum is shorter than a thousandth (the two cancel, the
    /// body inverted), the head's way alone. The placement stands on the
    /// plane under the root body, its model up along the normal and its
    /// model forward along the heading, scale 1. A pose from the bodies
    /// under it (maniRagdollPoseFromBodies) is the ragdoll in the
    /// character's new frame, to blend a get-up clip from.
    ///
    /// @param ragdoll  An opened profile.
    /// @param bodies   A state per body, in the profile's order: positions
    ///                 finite, rotations of length 1 within a thousandth.
    /// @param count    The number of states bodies holds.
    /// @param ground   The ground and the character's axes.
    /// @param out      Receives the placement and whether it lies face up.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// body's place not finite or its rotation not of length 1, the
    /// ground not finite, its normal or the axes not of length 1 within a
    /// thousandth or the axes not perpendicular within a thousandth;
    /// `mani_errorCapacity` when count is under the body count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniRagdollGetUpPlacement(const maniRagdoll* ragdoll,
                                                                 const maniRagdollBodyState* bodies,
                                                                 uint32_t count,
                                                                 const maniRagdollGround* ground,
                                                                 maniRagdollGetUp* out);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_RAGDOLL_H
