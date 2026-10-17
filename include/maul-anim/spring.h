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

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SPRING_H
