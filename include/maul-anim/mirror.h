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

// A bone's flags (maniMirrorBoneFlags): its parent's partner is not its
// partner's parent; its scale is carried as the nearest axis scale.
#define MANI_MIRROR_BONE_UNMATCHED         1u
#define MANI_MIRROR_BONE_SCALE_APPROXIMATE 2u

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
    /// pairing back), flags (a scale permutation or the approximate flag,
    /// the first unmatched one the header's) and corrections (finite, of
    /// length 1), and every
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

    /// Returns how a bone's mirrored local is made, as flags:
    /// MANI_MIRROR_BONE_UNMATCHED when its parent's partner is not its
    /// partner's parent (its mirrored pose is its partner's local turn
    /// mirrored rather than its partner's model turn), and
    /// MANI_MIRROR_BONE_SCALE_APPROXIMATE when the turn between its
    /// reference frame and its partner's mirrored one takes no axis to an
    /// axis, so a scale that differs between axes is carried as the
    /// nearest axis scale (a uniform scale stays exact). Otherwise a scale
    /// is exact, each axis taking the partner's axis the turn maps to it.
    ///
    /// @param map   An opened map, or NULL.
    /// @param bone  A bone of the map's skeleton.
    /// @return The flags; 0 for NULL or a bone past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMirrorBoneFlags(const maniMirrorMap* map, uint32_t bone);

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

    /// Returns the hash of a name's partner from the name's hash, for
    /// names kept hashed (events, markers): the hash of the name the map
    /// pairs it with, or the hash itself.
    ///
    /// @param map   An opened map, or NULL (the hash itself).
    /// @param hash  The name's hash, as maniClipEvent's nameHash holds it.
    /// @return The partner's hash.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMirrorNameHash(const maniMirrorMap* map, uint64_t hash);

    /// Mirrors a pose: each bone takes its partner's local (its own when
    /// unpaired), the translation's component along the normal negated
    /// and turned by the bone's correction from its partner's parent's
    /// reference frame mirrored to its parent's, the rotation mirrored
    /// between the corrections into its own reference axes, the scale as
    /// it is. On a mirror-symmetric hierarchy each bone's model position
    /// is its partner's reflected and its model rotation its partner's
    /// mirrored, carried to its own reference axes, so the rest's model
    /// rotations stay; mirroring twice gives the pose back. Writes every
    /// local of out and leaves its model stale.
    ///
    /// @param map       An opened map.
    /// @param skeleton  The map's skeleton.
    /// @param in        A pose of it (not a difference).
    /// @param out       A pose of it; may be in.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton whose identity or bone count is not the map's, a pose of
    /// another skeleton, or a difference.
    /// @par Thread safety
    /// Safe from any thread; out is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniMirrorPose(const maniMirrorMap* map,
                                                      const maniSkeleton* skeleton,
                                                      const maniPose* in, maniPose* out);

    /// Mirrors a root motion move: its translation's component along the
    /// normal negated, its rotation mirrored (the turn the reflection makes
    /// of it).
    ///
    /// @param map   An opened map, or NULL (the move unchanged).
    /// @param move  The move.
    /// @return The mirrored move.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniRootMove maniMirrorRootMotion(const maniMirrorMap* map, maniRootMove move);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MIRROR_H
