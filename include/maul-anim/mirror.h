// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mirror maps (mani-0033): how poses of a skeleton are mirrored across a
// plane through the model's origin. A map is built offline for its
// skeleton (mirrorbuild.h) and opened in place; each bone takes its
// partner's local (its own when unpaired) carried through the reference
// pose's model rotations, so rigs whose left and right bones do not have
// mirrored axes mirror to their reference.

#ifndef MAUL_ANIM_MIRROR_H
#define MAUL_ANIM_MIRROR_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // The model axis the mirror plane is normal to.
    typedef uint32_t maniMirrorAxis;
    enum
    {
        maniMirrorAxisX = 0,
        maniMirrorAxisY = 1,
        maniMirrorAxisZ = 2,
    };

    // An opened map: a view over the caller's bytes, which must outlive
    // it.
    typedef struct maniMirrorMap
    {
        const uint8_t* header;
        const uint8_t* bones;
        const uint8_t* names;
        const uint8_t* strings;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint64_t stringsSize;
        uint32_t boneCount;
        uint32_t nameCount;
        maniMirrorAxis axis;
        uint32_t reserved;
    } maniMirrorMap;

    /// Opens a mirror map file in place: checks the container, the header
    /// (the skeleton's identity and bone count, the axis, the name count,
    /// the first unmatched bone), every bone's partner (within the count,
    /// pairing back) and corrections (finite, of length 1), and every
    /// name pair (by increasing hash, its partner's name inside the names,
    /// UTF-8, of its hash).
    ///
    /// @param bytes   The file, 16-byte aligned.
    /// @param size    Its size in bytes; bytes past the file are ignored.
    /// @param mapOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenMirrorMap(const void* bytes, size_t size,
                                                             maniMirrorMap* mapOut);

    /// Returns a map's identity: the hash of its sections.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMirrorMapIdentity(const maniMirrorMap* map);

    /// Returns the identity of the skeleton a map mirrors.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The skeleton's identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMirrorMapSkeletonIdentity(const maniMirrorMap* map);

    /// Returns the axis a map's plane is normal to.
    ///
    /// @param map  An opened map, or NULL.
    /// @return maniMirrorAxisX, Y or Z; maniMirrorAxisX for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniMirrorAxis maniMirrorMapAxis(const maniMirrorMap* map);

    /// Returns a bone's partner: the bone it takes its mirrored local
    /// from (itself when unpaired).
    ///
    /// @param map   An opened map, or NULL.
    /// @param bone  A bone of the map's skeleton.
    /// @return The partner; MANI_NO_BONE for NULL or a bone past the
    /// count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMirrorPartner(const maniMirrorMap* map, uint32_t bone);

    /// Returns the first bone whose parent's partner is not its partner's
    /// parent: where the hierarchy is not mirror-symmetric, so that its
    /// mirrored pose is its partner's local turn mirrored rather than its
    /// partner's model turn.
    ///
    /// @param map  An opened map, or NULL.
    /// @return The bone; MANI_NO_BONE for a symmetric hierarchy or NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMirrorMapUnmatchedBone(const maniMirrorMap* map);

    /// Returns the partner of a float track's, event's or marker's name:
    /// the name the map pairs it with, or the name itself.
    ///
    /// @param map        An opened map.
    /// @param name       The name's UTF-8 bytes; may be NULL when length
    ///                   is 0.
    /// @param length     The name's length in bytes.
    /// @param lengthOut  Receives the partner's length. May be NULL.
    /// @return The partner's bytes inside the map, not terminated, or
    /// name itself when unpaired (or for a NULL map).
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniMirrorName(const maniMirrorMap* map, const char* name, size_t length,
                                        size_t* lengthOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MIRROR_H
