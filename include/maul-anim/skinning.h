// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skinning (mani-0021): a mesh primitive's influences, bound to a skin,
// over the caller's bytes. Groups of four influences a vertex, weights
// decreasing and summing to 1 (or all 0, a vertex left as it is).

#ifndef MAUL_ANIM_SKINNING_H
#define MAUL_ANIM_SKINNING_H

#include "maul-anim/base.h"
#include "maul-anim/skin.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Influences over a file's bytes, which the caller keeps alive and
    // unchanged while the view is used. Its fields are the library's;
    // read it through the functions below. A view is immutable and
    // copyable.
    typedef struct maniInfluences
    {
        const uint8_t* joints;
        const uint8_t* weights;
        const uint8_t* dualQuaternionWeights;
        const uint8_t* centers;
        uint64_t identity;
        uint64_t skinIdentity;
        uint32_t skinJointCount;
        uint32_t vertexCount;
        uint32_t groupCount;
        uint32_t reserved;
    } maniInfluences;

    /// Opens an influences file in place: checks the container, the sizes,
    /// every joint against the skin's joint count the file records, every
    /// weight finite and from 0 to 1, the dual quaternion weights and the
    /// centers when present, and the identity.
    ///
    /// @param bytes          The file, 16-byte aligned.
    /// @param size           Its size in bytes; bytes past the file are
    ///                       ignored.
    /// @param influencesOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenInfluences(const void* bytes, size_t size,
                                                              maniInfluences* influencesOut);

    /// Returns the number of vertices.
    ///
    /// @param influences  Opened influences.
    /// @return The vertex count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniInfluencesVertexCount(const maniInfluences* influences);

    /// Returns the influences a vertex holds: four times the group count,
    /// the last ones of weight 0 where a vertex has fewer.
    ///
    /// @param influences  Opened influences.
    /// @return The influences a vertex.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniInfluencesPerVertex(const maniInfluences* influences);

    /// Returns whether the influences are a skin's: its identity and joint
    /// count are the ones recorded.
    ///
    /// @param influences  Opened influences.
    /// @param skin        An opened skin.
    /// @return True when bound to the skin.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API bool maniInfluencesFitSkin(const maniInfluences* influences, const maniSkin* skin);

    /// Returns one influence of a vertex: its joint (an index into the
    /// skin's joints) and weight; joint 0 at weight 0 past the vertex
    /// count or the influences.
    ///
    /// @param influences  Opened influences.
    /// @param vertex      The vertex.
    /// @param slot        The influence, 0 the heaviest.
    /// @param weightOut   Receives the weight; may be NULL.
    /// @return The joint.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniInfluence(const maniInfluences* influences, uint32_t vertex,
                                    uint32_t slot, float* weightOut);

    /// Returns a vertex's blend between linear (0) and dual quaternion (1)
    /// skinning, FBX's blended method; 0 for influences without them or
    /// past the vertex count.
    ///
    /// @param influences  Opened influences.
    /// @param vertex      The vertex.
    /// @return The weight, 0 to 1.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API float maniInfluencesDualQuaternionWeight(const maniInfluences* influences,
                                                      uint32_t vertex);

    /// Reads a vertex's center of rotation, built offline.
    ///
    /// @param influences  Opened influences.
    /// @param vertex      The vertex.
    /// @param centerOut   Receives the center; unchanged on false.
    /// @return True when the influences hold centers and the vertex is in
    /// range.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API bool maniInfluencesCenter(const maniInfluences* influences, uint32_t vertex,
                                       maniVec3* centerOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SKINNING_H
