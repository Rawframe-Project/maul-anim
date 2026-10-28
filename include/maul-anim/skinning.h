// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skinning: a mesh primitive's influences, bound to a skin,
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

    // How a skin blends its joints: the job to run on its influences.
    typedef uint32_t maniSkinningMethod;
    enum
    {
        maniSkinningLinear = 0,
        maniSkinningDualQuaternion = 1,
        // Linear and dual quaternion at each vertex's blend weight.
        maniSkinningBlended = 2,
    };

    // What a stream holds: points (positions, moved by the whole
    // transform), directions (turned and renormalized), normals (by the
    // inverse transpose's direction, renormalized) or tangents (four
    // floats, the first three a direction, the fourth copied).
    typedef uint32_t maniStreamKind;
    enum
    {
        maniStreamPoint = 0,
        maniStreamDirection = 1,
        maniStreamNormal = 2,
        maniStreamTangent = 3,
    };

    // A vertex stream: binary32 x, y, z (w for tangents) at in, written
    // at out; strides in bytes between vertices, at least 12 (16 for
    // tangents). out may be in, with the same stride.
    typedef struct maniSkinStream
    {
        const void* in;
        void* out;
        size_t inStride;
        size_t outStride;
        maniStreamKind kind;
    } maniSkinStream;

    // A skinning job over vertices [first, first + count) of the
    // influences, their streams' pointers at vertex first.
    typedef struct maniSkinJob
    {
        const maniInfluences* influences;
        // The skin's palette, as maniComputePalette writes it: a matrix a
        // joint (linear, blended).
        const maniAffine* palette;
        // The same palette as maniPaletteDualQuaternions writes it, 20
        // floats a joint (dual quaternion, blended); NULL otherwise.
        const float* dualQuaternions;
        uint32_t paletteCount;
        uint32_t first;
        uint32_t count;
        uint32_t streamCount;
        const maniSkinStream* streams;
    } maniSkinJob;

    /// Linear blend skinning: each vertex moved by the weighted sum of its
    /// joints' palette matrices, its directions by the sum's 3x3 part, its
    /// normals by that part's cofactor times its determinant's sign (the
    /// inverse transpose's direction), directions renormalized (zero when
    /// they collapse). A vertex without weight is copied.
    ///
    /// @param job  The job.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// palette count other than the influences' joint count, a range past
    /// the vertices, or a stream without pointers, with a stride too
    /// small or of an unknown kind (nothing written).
    /// @par Thread safety
    /// Safe from any thread; the output range is used by one thread at a time.
    /// Jobs over disjoint ranges run at once.
    MANI_NODISCARD MANI_API maniResult maniSkinLinear(const maniSkinJob* job);

    /// Dual quaternion skinning in two phases (Kavan et al. 2008): each
    /// vertex's scale and shear parts summed at their weights and applied
    /// first, then its joints' dual quaternions blended (each turned to
    /// the first's hemisphere) and normalized. Streams as maniSkinLinear
    /// treats them; a vertex with one influence moves exactly as under
    /// linear skinning, up to rounding.
    ///
    /// @param job  The job, its dualQuaternions given.
    /// @return As maniSkinLinear, and `mani_errorInvalid` without dual
    /// quaternions.
    /// @par Thread safety
    /// Safe from any thread; the output range is used by one thread at a time.
    /// Jobs over disjoint ranges run at once.
    MANI_NODISCARD MANI_API maniResult maniSkinDualQuaternion(const maniSkinJob* job);

    /// FBX's blended skinning: each vertex moved by `(1 - b)` times its
    /// linear matrix plus `b` times its dual quaternion one, b its weight
    /// in the influences (0 where they have none). A vertex at 0 or 1
    /// gives the bits of maniSkinLinear or maniSkinDualQuaternion.
    ///
    /// @param job  The job, its palette and dualQuaternions given.
    /// @return As maniSkinDualQuaternion.
    /// @par Thread safety
    /// Safe from any thread; the output range is used by one thread at a time.
    /// Jobs over disjoint ranges run at once.
    MANI_NODISCARD MANI_API maniResult maniSkinBlended(const maniSkinJob* job);

    /// Skinning with optimized centers of rotation (Le and Hodgins 2016)
    /// in two phases: each vertex's scale and shear parts summed at their
    /// weights and applied first (to the vertex and its center c), then
    /// the rotation R of its joints' quaternions blended in the first's
    /// hemisphere, and the translation that carries the scaled center
    /// where linear blending of the rigid parts takes it:
    /// `v' = R (S v - S c) + sum w_j (R_j S c + t_j)`. No collapse and no
    /// bulge; a vertex with one influence moves as under linear skinning.
    ///
    /// @param job  The job, its palette and dualQuaternions given, its
    ///             influences with centers (maniBuildCenters).
    /// @return As maniSkinDualQuaternion, and `mani_errorInvalid` for
    /// influences without centers.
    /// @par Thread safety
    /// Safe from any thread; the output range is used by one thread at a time.
    /// Jobs over disjoint ranges run at once.
    MANI_NODISCARD MANI_API maniResult maniSkinCenters(const maniSkinJob* job);

    /// Writes a palette for dual quaternion skinning, on the CPU or in a
    /// shader: each matrix A split as A = R S (R the rotation of its polar
    /// decomposition, S = R^T A carrying scale, shear and mirror), 20
    /// floats a matrix: R's quaternion (x, y, z, w with w >= 0), the dual
    /// part (half the translation times it), then S as three rows of four
    /// (the fourth 0).
    ///
    /// @param palette  count matrices, as maniComputePalette writes them.
    /// @param count    The number of matrices.
    /// @param out      Receives 20 floats a matrix.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL pointer with
    /// a count or a matrix not finite (out unchanged).
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniPaletteDualQuaternions(const maniAffine* palette,
                                                                  uint32_t count, float* out);

    /// Writes a palette for shaders: each matrix as three rows of four
    /// floats (row-major 3x4), the translation last in each row.
    ///
    /// @param palette  count matrices, as maniComputePalette writes them.
    /// @param count    The number of matrices.
    /// @param rows     Receives 12 floats a matrix.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL pointer with
    /// a count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniPaletteRows(const maniAffine* palette, uint32_t count,
                                                       float* rows);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_SKINNING_H
