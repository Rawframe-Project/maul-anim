// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skeletons: bones in depth-first order, each with a parent,
// a rest transform and a unique name, read in place from a skeleton
// file's bytes. Opening a file checks all of it once; the view then
// reads it without checks and allocates nothing.

#ifndef MAUL_ANIM_SKELETON_H
#define MAUL_ANIM_SKELETON_H

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The parent of a root bone, and what a lookup finds for no bone.
#define MANI_NO_BONE 0xFFFFu
// The most bones a skeleton holds, the only limit on skeletons.
#define MANI_MAX_BONES 65535u

    // A skeleton over a file's bytes, which the caller keeps alive and
    // unchanged while the view is used. Its fields are the library's;
    // read a skeleton through the functions below. A view is immutable:
    // safe to read from any number of threads, and copyable. Every call
    // checks the view it is given, so a view whose fields were changed or
    // mixed from two views reads only within the file it names, and is
    // refused when its counts do not fit that file; the bytes are read
    // only within the extents recorded at open, whatever they become.
    typedef struct maniSkeleton
    {
        const uint8_t* base;
        uint64_t size;
        uint64_t sections[7];
        uint64_t nameSize;
        uint64_t identity;
        uint32_t boneCount;
        uint32_t cookie;
    } maniSkeleton;

    /// Opens a skeleton file in place: checks the container and every
    /// section (counts, depth-first order, rests finite with unit
    /// rotations, names valid UTF-8, unique and matching their hashes, the
    /// identity) and fills the view.
    ///
    /// @param bytes       The file, 16-byte aligned.
    /// @param size        Its size in bytes; bytes past the file are
    ///                    ignored.
    /// @param skeletonOut Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for any other format version
    /// or a section tag only a later format writes.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenSkeleton(const void* bytes, size_t size,
                                                            maniSkeleton* skeletonOut);

    /// Returns the number of bones, 1 to MANI_MAX_BONES.
    ///
    /// @param skeleton  An opened skeleton.
    /// @return The bone count; 0 for a view no open wrote or whose counts
    /// do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSkeleton_GetBoneCount(const maniSkeleton* skeleton);

    /// Returns the skeleton's identity: a hash of its bones' order,
    /// parents, rests and names, which files built against it record.
    ///
    /// @param skeleton  An opened skeleton.
    /// @return The identity; 0 for a view no open wrote or whose counts do
    /// not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniSkeleton_GetIdentity(const maniSkeleton* skeleton);

    /// Returns a bone's parent, always a lower index.
    ///
    /// @param skeleton  An opened skeleton.
    /// @param bone      The bone.
    /// @return The parent, or MANI_NO_BONE for a root, a bone past the
    /// count, a view no open wrote or whose counts do not fit its file, or
    /// a parent in the bytes that is not a lower index.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSkeleton_GetParent(const maniSkeleton* skeleton, uint32_t bone);

    /// Returns the end of a bone's subtree: the bone and its descendants
    /// are the indices from bone up to, not including, the end.
    ///
    /// @param skeleton  An opened skeleton.
    /// @param bone      The bone.
    /// @return The end, or bone itself (an empty range) for a bone past
    /// the count, a view no open wrote or whose counts do not fit its
    /// file, or an end in the bytes past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSkeleton_GetSubtreeEnd(const maniSkeleton* skeleton, uint32_t bone);

    /// Returns a bone's rest transform, relative to its parent.
    ///
    /// @param skeleton  An opened skeleton.
    /// @param bone      The bone.
    /// @return The rest, or the identity transform for a bone past the
    /// count or a view no open wrote or whose counts do not fit its
    /// file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniTransform maniSkeleton_GetRest(const maniSkeleton* skeleton, uint32_t bone);

    /// Returns a bone's name, UTF-8 and not terminated.
    ///
    /// @param skeleton  An opened skeleton.
    /// @param bone      The bone.
    /// @param lengthOut Receives the name's length in bytes, 0 when NULL
    ///                  is returned. May be NULL.
    /// @return The name's bytes, inside the file, or NULL for a bone past
    /// the count, a view no open wrote or whose counts do not fit its
    /// file, or a name span in the bytes past the names.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniSkeleton_GetBoneName(const maniSkeleton* skeleton, uint32_t bone,
                                                  size_t* lengthOut);

    /// Finds a bone by its name.
    ///
    /// @param skeleton  An opened skeleton.
    /// @param name      The name's UTF-8 bytes; may be NULL when length is
    ///                  0.
    /// @param length    The name's length in bytes.
    /// @return The bone, or MANI_NO_BONE when no bone has that name or for
    /// a view no open wrote or whose counts do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniSkeleton_FindBone(const maniSkeleton* skeleton, const char* name,
                                            size_t length);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SKELETON_H
