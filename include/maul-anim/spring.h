// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring bones (mani-0026): chains of bones that sway behind the motion
// as VRMC_springBone 1.0 defines them, with its extended colliders
// (inside spheres and capsules, planes) and angle limits (cones, hinges,
// spherical limits). A set is built offline for one skeleton, its
// per-frame values converted to a fixed step, and opened in place.

#ifndef MAUL_ANIM_SPRING_H
#define MAUL_ANIM_SPRING_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // An opened spring set: a view over the caller's bytes, which must
    // outlive it.
    typedef struct maniSpringSet
    {
        const uint8_t* colliders;
        const uint8_t* springs;
        const uint8_t* joints;
        const uint8_t* links;
        const uint8_t* names;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint32_t boneCount;
        uint32_t colliderCount;
        uint32_t springCount;
        uint32_t jointCount;
        uint32_t linkCount;
        uint32_t nameBytes;
        float referenceRate;
        float stepRate;
    } maniSpringSet;

    /// Opens a spring set file in place: checks the container and every
    /// record (bones within the skeleton's count, shapes, chains each
    /// joint above the next as far as the file can tell, links, names,
    /// every float finite and in its range).
    ///
    /// @param bytes   The file, 16-byte aligned.
    /// @param size    Its size in bytes; bytes past the file are ignored.
    /// @param setOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenSpringSet(const void* bytes, size_t size,
                                                             maniSpringSet* setOut);

    /// Returns a set's identity: the hash of its sections.
    ///
    /// @param set  An opened set, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSpringSetIdentity(const maniSpringSet* set);

    /// Returns the identity of the skeleton a set was built for.
    ///
    /// @param set  An opened set, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSpringSetSkeletonIdentity(const maniSpringSet* set);

    /// Returns the number of springs (chains) in a set.
    ///
    /// @param set  An opened set, or NULL.
    /// @return The count; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSpringSetSpringCount(const maniSpringSet* set);

    /// Returns a spring's name, its bytes inside the set (not terminated).
    ///
    /// @param set        An opened set.
    /// @param spring     The spring's index.
    /// @param lengthOut  Receives the name's length in bytes (0 when the
    ///                   spring has none or the call fails). May be NULL.
    /// @return The name's first byte; NULL for a NULL set or an index
    /// past the springs, and an empty name's position for none.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniSpringSetSpringName(const maniSpringSet* set, uint32_t spring,
                                                 uint32_t* lengthOut);

    /// Returns the rate a set steps at (steps a second), and the rate its
    /// source's values were per frame of.
    ///
    /// @param set               An opened set, or NULL.
    /// @param referenceRateOut  Receives the reference rate. May be NULL.
    /// @return The step rate; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API float maniSpringSetStepRate(const maniSpringSet* set, float* referenceRateOut);

// The most steps a spring state takes in one call: a longer delta plays
// as this many steps' seconds and loses the rest, as fixed-step
// simulations lose time past their limit.
#define MANI_SPRING_MAX_STEPS 20u

// The alignment a spring state's memory needs.
#define MANI_SPRING_STATE_ALIGNMENT 8u

    // The motion of a set's joints for one character: in caller memory, a
    // memcpy copies it (it points only at its set and skeleton).
    typedef struct maniSpringState maniSpringState;

    /// Returns the bytes a set's state needs.
    ///
    /// @param set  An opened set.
    /// @return The size; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniSpringStateSize(const maniSpringSet* set);

    /// Makes a state for a set in caller memory, with no motion yet: the
    /// first step places every tail where the pose puts it.
    ///
    /// @param set       An opened set, which must outlive the state.
    /// @param skeleton  The skeleton the set was built for, which must
    ///                  outlive the state.
    /// @param memory    At least maniSpringStateSize bytes,
    ///                  MANI_SPRING_STATE_ALIGNMENT aligned.
    /// @param size      Its size.
    /// @param stateOut  Receives the state (at memory).
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument,
    /// misaligned memory or a skeleton of another identity;
    /// `mani_errorCapacity` for too little memory.
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniInitSpringState(const maniSpringSet* set,
                                                           const maniSkeleton* skeleton,
                                                           void* memory, size_t size,
                                                           maniSpringState** stateOut);

    /// Moves a state on by delta seconds (its magnitude, at most
    /// MANI_SPRING_MAX_STEPS steps' seconds) toward a pose: runs the steps
    /// due at the set's step rate, each with the pose and placement
    /// interpolated between the last call's and these at its time
    /// (positions linearly, rotations along the shorter arc); time short
    /// of a step is kept for the next call. Tails move by inertia less
    /// drag, toward the pose's direction by stiffness and by gravity in
    /// the world, keep their bone's length, leave colliders (or stay
    /// inside inside ones) and keep within their limits; a spring with a
    /// center bone carries its inertia in the center's space.
    ///
    /// @param state      A state.
    /// @param pose       The animated pose at the end of delta (its model
    ///                   space computed when stale).
    /// @param placement  The character's place in the world at the end of
    ///                   delta (translation and rotation; its scale is not
    ///                   read), or NULL for the identity.
    /// @param delta      The time in seconds, finite, either sign.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL state or
    /// pose, a pose of another bone count or holding a difference, or a
    /// delta or placement not finite.
    /// @par Thread safety
    /// Safe from any thread; the state and pose are used by one thread at
    /// a time.
    MANI_NODISCARD MANI_API maniResult maniStepSprings(maniSpringState* state, maniPose* pose,
                                                       const maniTransform* placement, float delta);

    /// Turns each spring joint of a pose toward its tail: the tails
    /// interpolated between the last two steps at the time short of a
    /// step, carried into the placement of the last call. Joints with no
    /// motion yet keep the pose's rotation.
    ///
    /// @param state  A state.
    /// @param pose   The pose the last step took (its model space computed
    ///               when stale); joints' local rotations are written.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a pose of another bone count or holding a difference.
    /// @par Thread safety
    /// Safe from any thread; the state and pose are used by one thread at
    /// a time.
    MANI_NODISCARD MANI_API maniResult maniApplySprings(const maniSpringState* state,
                                                        maniPose* pose);

    /// Drops a state's motion: the next step places every tail where the
    /// pose puts it (a teleport).
    ///
    /// @param state  A state, or NULL (nothing done).
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_API void maniResetSprings(maniSpringState* state);

    /// Returns the bytes a state's saved form takes.
    ///
    /// @param set  An opened set.
    /// @return The size; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniSpringSavedSize(const maniSpringSet* set);

    /// Saves a state without pointers: its set's identity and its values.
    ///
    /// @param state     A state.
    /// @param out       At least maniSpringSavedSize bytes.
    /// @param capacity  Their size.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorCapacity` for too small a buffer.
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSaveSpringState(const maniSpringState* state, void* out,
                                                           size_t capacity);

    /// Restores a saved state after checking it as external bytes: the
    /// set's identity, the size, every value finite and in its range,
    /// rotations of length 1, no values but zeros without motion.
    ///
    /// @param state  A state of the same set.
    /// @param bytes  The saved bytes.
    /// @param size   Their size.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument;
    /// `mani_errorFormat` with the offset of the first byte found wrong
    /// (the state unchanged).
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_NODISCARD MANI_API maniFileResult maniRestoreSpringState(maniSpringState* state,
                                                                  const void* bytes, size_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SPRING_H
