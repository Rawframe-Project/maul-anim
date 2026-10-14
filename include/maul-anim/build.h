// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The offline side: a builder turns source data into the library's files
// (mani-0003), which it keeps until the next build for a copy into caller
// memory. A builder allocates through the allocator of its def and is
// built only when MAUL_ANIM_OFFLINE is on.

#ifndef MAUL_ANIM_BUILD_H
#define MAUL_ANIM_BUILD_H

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A builder: one owner object for every kind of file.
    typedef struct maniBuilder maniBuilder;

    typedef struct maniBuilderDef
    {
        uint32_t cookie;
        // MANI_ABI_VERSION, set by maniDefaultBuilderDef.
        uint32_t version;
        // The allocator the builder uses; zeroed for the C library's.
        maniAllocator allocator;
    } maniBuilderDef;

// The cookie maniDefaultBuilderDef sets and maniCreateBuilder checks.
#define MANI_BUILDER_DEF_COOKIE 0x4D424C44u

    /// Returns a builder def with every setting at its default.
    ///
    /// @return The def, with a valid cookie.
    /// @par Thread safety
    /// Safe from any thread.
    static inline maniBuilderDef maniDefaultBuilderDef(void)
    {
#ifdef __cplusplus
        maniBuilderDef def = {};
#else
    maniBuilderDef def = {0};
#endif
        def.cookie = MANI_BUILDER_DEF_COOKIE;
        def.version = MANI_ABI_VERSION;
        return def;
    }

    /// Makes a builder from a def.
    ///
    /// @param def        The def.
    /// @param builderOut Receives the builder, or NULL on failure.
    /// @return `mani_success`; `mani_errorVersion` for a def built against
    /// another major or minor version; `mani_errorInvalid` for a NULL
    /// argument, a def without its cookie or an allocator with one
    /// function; `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniCreateBuilder(const maniBuilderDef* def,
                                                         maniBuilder** builderOut);

    /// Destroys a builder and the file it holds.
    ///
    /// @param builder  The builder, or NULL.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_API void maniDestroyBuilder(maniBuilder* builder);

    /// Returns how many calls on the builder were refused for invalid
    /// arguments (conventions section 8).
    ///
    /// @param builder  The builder.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_API uint32_t maniBuilderMisuses(const maniBuilder* builder);

    // Why a build refused its source.
    typedef uint32_t maniBuildReason;
    enum
    {
        maniBuildReasonNone = 0,
        // Too few or too many items: no bones, or more than MANI_MAX_BONES.
        maniBuildReasonCount = 1,
        // A parent index out of range, or a bone its own parent.
        maniBuildReasonParent = 2,
        // A bone no root reaches: on a cycle of parents, or below one.
        maniBuildReasonCycle = 3,
        // An empty name, a NULL one, or one that is not UTF-8.
        maniBuildReasonName = 4,
        // A name a lower item already has; other is that item.
        maniBuildReasonDuplicateName = 5,
        // Two names whose 64-bit hashes are equal; other is the lower item.
        maniBuildReasonNameHash = 6,
        // A translation, rotation or scale component that is not finite.
        maniBuildReasonNotFinite = 7,
        // A rotation of zero length.
        maniBuildReasonRotation = 8,
    };

    // A build's outcome: the status and, when the source was refused
    // (mani_errorInvalid), the reason, the source item and a second item
    // the reason names.
    typedef struct maniBuildResult
    {
        maniResult result;
        maniBuildReason reason;
        uint32_t item;
        uint32_t other;
    } maniBuildResult;

    // A bone as a source gives it.
    typedef struct maniBoneSource
    {
        // The name, UTF-8, not empty, unique in the skeleton.
        const char* name;
        uint32_t nameLength;
        // The parent's index in the source's bones, or -1 for a root.
        int32_t parent;
        // The rest transform relative to the parent; the rotation is
        // normalized by the build.
        maniTransform rest;
    } maniBoneSource;

    // A skeleton's bones in any order, each naming its parent by index.
    typedef struct maniSkeletonSource
    {
        const maniBoneSource* bones;
        uint32_t boneCount;
        uint32_t reserved;
    } maniSkeletonSource;

    /// Builds a skeleton file: checks every bone, orders the bones
    /// depth-first (children and roots in the source's order), normalizes
    /// rotations, and keeps the file in place of any built before.
    ///
    /// @param builder  The builder.
    /// @param source   The bones.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument (a
    /// misuse) or with the reason and source bone for a refused source;
    /// `mani_errorCapacity` when the allocator fails.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildSkeleton(maniBuilder* builder,
                                                              const maniSkeletonSource* source);

    /// Copies the last built file into caller memory.
    ///
    /// @param builder   The builder.
    /// @param bufferOut Receives the bytes: at least capacity bytes; may be
    ///                  NULL when capacity is 0. Open the copy from a
    ///                  16-byte aligned address.
    /// @param capacity  The buffer's size in bytes.
    /// @param sizeOut   Receives the file's size in bytes. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when the buffer is
    /// smaller than the file, with the size still written;
    /// `mani_errorInvalid` for a NULL builder, a NULL buffer with a
    /// positive capacity, or a builder holding no file.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniCopyBuilt(maniBuilder* builder, void* bufferOut,
                                                     size_t capacity, size_t* sizeOut);

    /// Copies where each source bone of the last built skeleton went: the
    /// skeleton's index of source bone i at boneOut[i].
    ///
    /// @param builder   The builder.
    /// @param boneOut   Receives up to capacity indices; may be NULL when
    ///                  capacity is 0.
    /// @param capacity  The array's length.
    /// @param countOut  Receives the source's bone count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when the array is
    /// shorter than the count, with the count still written and the array
    /// filled as far as it goes; `mani_errorInvalid` for a NULL builder, a
    /// NULL array with a positive capacity, or a builder holding no
    /// skeleton.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniCopySkeletonOrder(maniBuilder* builder,
                                                             uint32_t* boneOut, uint32_t capacity,
                                                             uint32_t* countOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_BUILD_H
