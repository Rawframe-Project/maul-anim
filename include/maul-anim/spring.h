// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring bones: chains of bones that sway behind the motion
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

    // A spring set over a file's bytes, which the caller keeps alive
    // while the view is used. Its fields are the library's; read a set
    // through the functions below. A view is immutable and copyable. Its
    // base and size are the bytes the open was given; a view whose base or
    // size the caller changed is outside the contract. Every call checks
    // everything else the view holds, its counts and sections, against
    // those bytes, and refuses a view that does not fit them; the bytes are
    // read only within the extents recorded at open, whatever they become.
    // Bytes changed after the open give some motion: a spring's joints or
    // links past the set's are cut at their end, a link's collider past
    // the colliders is skipped, and a name past the names is empty.
    typedef struct maniSpringSet
    {
        const uint8_t* base;
        uint64_t size;
        // The colliders, springs, joints, links and names: each one's
        // offset from base.
        uint64_t sections[5];
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
        uint32_t cookie;
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
    /// byte found wrong; `mani_errorVersion` for any other format version
    /// or a section tag only a later format writes.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenSpringSet(const void* bytes, size_t size,
                                                             maniSpringSet* setOut);

    /// Returns a set's identity: the hash of its sections.
    ///
    /// @param set  An opened set.
    /// @return The identity; 0 for a view no open wrote or whose counts
    /// do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSpringSet_GetIdentity(const maniSpringSet* set);

    /// Returns the identity of the skeleton a set was built for.
    ///
    /// @param set  An opened set.
    /// @return The identity; 0 for a view no open wrote or whose counts
    /// do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSpringSet_GetSkeletonIdentity(const maniSpringSet* set);

    /// Returns the number of springs (chains) in a set.
    ///
    /// @param set  An opened set.
    /// @return The count; 0 for a view no open wrote or whose counts
    /// do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSpringSet_GetSpringCount(const maniSpringSet* set);

    /// Returns a spring's name, its bytes inside the set (not terminated).
    ///
    /// @param set        An opened set.
    /// @param spring     The spring's index.
    /// @param lengthOut  Receives the name's length in bytes (0 when the
    ///                   spring has none or the call fails). May be NULL.
    /// @return The name's first byte; NULL for a view no open wrote or
    /// whose counts do not fit its file, or an index past the springs; an
    /// empty name's position for none.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniSpringSet_GetSpringName(const maniSpringSet* set, uint32_t spring,
                                                     size_t* lengthOut);

    /// Returns the rate a set steps at (steps a second), and the rate its
    /// source's values were per frame of.
    ///
    /// @param set               An opened set.
    /// @param referenceRateOut  Receives the reference rate (0 when the
    ///                          call fails). May be NULL.
    /// @return The step rate; 0 for a view no open wrote or whose counts
    /// do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API float maniSpringSet_GetStepRate(const maniSpringSet* set, float* referenceRateOut);

// The most steps a spring state takes in one call: a longer delta plays
// as this many steps' seconds and loses the rest, as fixed-step
// simulations lose time past their limit.
#define MANI_SPRING_MAX_STEPS 20u

// The alignment a spring state's memory needs.
#define MANI_SPRING_STATE_ALIGNMENT 8u

    // The motion of a set's joints for one character: in caller memory, a
    // memcpy copies it (it points only at its set's and skeleton's
    // bytes). It keeps copies of the set's and the skeleton's views as
    // its init saw them, and every call refuses memory no init wrote.
    typedef struct maniSpringState maniSpringState;

    /// Returns the bytes a set's state needs.
    ///
    /// @param set  An opened set.
    /// @return The size; 0 for a view no open wrote or whose counts
    /// do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniSpringSet_GetStateSize(const maniSpringSet* set);

    /// Makes a state for a set in caller memory, with no motion yet: the
    /// first step places every tail where the pose puts it.
    ///
    /// @param set       An opened set; the state copies the view, and
    ///                  its bytes must outlive the state.
    /// @param skeleton  The skeleton the set was built for; the state
    ///                  copies the view, and its bytes must outlive the
    ///                  state.
    /// @param memory    At least maniSpringSet_GetStateSize bytes,
    ///                  MANI_SPRING_STATE_ALIGNMENT aligned.
    /// @param size      Its size.
    /// @param stateOut  Receives the state (at memory).
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// set no open wrote or whose counts do not fit its file, misaligned
    /// memory or a skeleton of another identity;
    /// `mani_errorCapacity` for too little memory.
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniInitSpringState(const maniSpringSet* set,
                                                           const maniSkeleton* skeleton,
                                                           void* memory, size_t size,
                                                           maniSpringState** stateOut);

    /// Moves a state on by delta seconds (its magnitude, at most
    /// MANI_SPRING_MAX_STEPS steps' seconds) toward a pose: runs the steps
    /// due at the set's step rate, each with the pose, placement and force
    /// interpolated between the last call's and these at its time
    /// (positions and forces linearly, rotations along the shorter arc);
    /// time short of a step is kept for the next call. Tails move by
    /// inertia less drag, toward the pose's direction by stiffness and by
    /// gravity and the external force in the world, keep
    /// their bone's length, leave colliders (or stay
    /// inside inside ones) and keep within their limits; a spring with a
    /// center bone carries its inertia in the center's space.
    ///
    /// @param state      A state.
    /// @param pose       The animated pose at the end of delta (its model
    ///                   space computed when stale).
    /// @param placement  The character's place in the world at the end of
    ///                   delta (translation and rotation; its scale is not
    ///                   read), or NULL for the identity.
    /// @param force      A force in the world on every joint at the end of
    ///                   delta (wind), in gravity's units (a length a
    ///                   second, times the head's scale), or NULL for none
    ///                   (a zero force: after a force it ramps down over
    ///                   the call; a call of no time without one first
    ///                   stops it at once). The first call of a new or
    ///                   reset state takes its force for both ends.
    /// @param delta      The time in seconds, finite, either sign.
    /// @return `mani_success`; `mani_errorInvalid` for a state no init
    /// made, a NULL pose, a pose of another bone count or holding a
    /// difference, or a delta, placement or force not finite.
    /// @par Thread safety
    /// Safe from any thread; the state and pose are used by one thread at
    /// a time.
    MANI_NODISCARD MANI_API maniResult maniSpringState_Step(maniSpringState* state, maniPose* pose,
                                                            const maniTransform* placement,
                                                            const maniVec3* force, float delta);

    /// Turns each spring joint of a pose toward its tail: the last step's
    /// tails moved on at their velocity for the time since that step (the
    /// last step's own at a host rate the step rate divides), carried into
    /// the placement of the last call, then pushed out of their colliders
    /// as the last call left them (where a step left a tail inside one,
    /// the output is pushed further than the step's own). Joints with no
    /// motion yet keep the pose's rotation.
    ///
    /// @param state  A state.
    /// @param pose   The pose the last step took (its model space computed
    ///               when stale); joints' local rotations are written.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// state no init made, or a pose of another bone count or holding a
    /// difference.
    /// @par Thread safety
    /// Safe from any thread; the state and pose are used by one thread at
    /// a time.
    MANI_NODISCARD MANI_API maniResult maniSpringState_Apply(const maniSpringState* state,
                                                             maniPose* pose);

    /// Drops a state's motion: the next step places every tail where the
    /// pose puts it (a teleport) and takes its placement and force as the
    /// last call's.
    ///
    /// @param state  A state; NULL or memory no init made does nothing.
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_API void maniSpringState_Reset(maniSpringState* state);

    /// Saves a state without pointers: its set's identity and its values.
    /// Call it with a NULL buffer and capacity 0 to measure.
    ///
    /// @param state     A state.
    /// @param bytesOut  Receives the saved state: at least capacity bytes;
    ///                  may be NULL when capacity is 0.
    /// @param capacity  The bytes bytesOut holds.
    /// @param sizeOut   Receives the saved state's size in bytes. May be
    ///                  NULL.
    /// @return `mani_success`; `mani_errorCapacity` when capacity is
    /// smaller than the saved state, with the size still written and
    /// nothing else; `mani_errorInvalid` for a state no init made or a
    /// NULL buffer with a positive capacity.
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSpringState_Save(const maniSpringState* state,
                                                            void* bytesOut, size_t capacity,
                                                            size_t* sizeOut);

    /// Restores a saved state after checking it as external bytes: the
    /// set's identity, the size, every value finite and in its range,
    /// rotations of length 1, no values but zeros without motion.
    ///
    /// @param state  A state of the same set.
    /// @param bytes  The saved bytes.
    /// @param size   Their size.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a state no init made; `mani_errorVersion` for a state saved by a release of another ABI
    /// version (MANI_ABI_VERSION) or of another state format version;
    /// `mani_errorFormat` with the offset of the first byte found wrong
    /// (the state unchanged in either case).
    /// @par Thread safety
    /// Safe from any thread; the state is used by one thread at a time.
    MANI_NODISCARD MANI_API maniFileResult maniSpringState_Restore(maniSpringState* state,
                                                                   const void* bytes, size_t size);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SPRING_H
