// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Morph targets: a mesh primitive's blend shapes over the
// caller's bytes. Named channels, each one or more targets at increasing
// full weights (glTF's targets have one at 1; FBX's in-betweens several),
// each target deltas for some of the file's attributes (positions,
// normals, tangents, texture coordinate and color sets), dense or
// sparse.

#ifndef MAUL_ANIM_MORPH_H
#define MAUL_ANIM_MORPH_H

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// What a channel lookup finds for no channel.
#define MANI_NO_CHANNEL 0xFFFFFFFFu
// The most attributes a morph targets file holds.
#define MANI_MAX_MORPH_ATTRIBUTES 32u

    // A vertex attribute morph targets move: deltas of 3 floats for
    // positions, normals and tangents (no w), 2 for texture coordinates,
    // 4 for colors (RGBA).
    typedef int32_t maniMorphSemantic;
    enum
    {
        maniMorphPosition = 0,
        maniMorphNormal = 1,
        maniMorphTangent = 2,
        maniMorphTexcoord = 3,
        maniMorphColor = 4,
    };

    // Morph targets over a file's bytes, which the caller keeps alive
    // while the view is used. Its fields are the library's; read it
    // through the functions below. A view is immutable and copyable.
    // Its base and size are the bytes the open was given; a view whose
    // base or size the caller changed is outside the contract. Every
    // call checks everything else the view holds, its counts and
    // sections, against those bytes, and refuses a view that does not
    // fit them; the bytes are read only within the extents recorded at
    // open, whatever they become. Bytes changed after the open give
    // some weights, some morphed attributes and some names: what a call
    // reads from them (a channel's targets, a target's indices and
    // deltas, a sparse target's vertices, a name's span) is bounded
    // where it is read, so it reads only within the file and the job
    // writes only the range it was given.
    typedef struct maniMorphTargets
    {
        const uint8_t* base;
        uint64_t size;
        // Sections 2 to 7 of the file (attributes to names): each one's
        // offset from base and size.
        uint64_t offsets[6];
        uint64_t sizes[6];
        uint64_t identity;
        uint32_t vertexCount;
        uint32_t channelCount;
        uint32_t targetCount;
        uint32_t attributeCount;
        uint32_t cookie;
    } maniMorphTargets;

    /// Opens a morph targets file in place: checks the container, the
    /// sizes, the attribute table, the channels covering the targets in
    /// order, every name inside the name bytes, every full
    /// weight finite, not 0 and increasing within its channel, every
    /// sparse target's indices increasing and below the vertex count,
    /// every delta finite, and the identity.
    ///
    /// @param bytes       The file, 16-byte aligned.
    /// @param size        Its size in bytes; bytes past the file are
    ///                    ignored.
    /// @param targetsOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for any other format version
    /// or a section tag only a later format writes.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenMorphTargets(const void* bytes, size_t size,
                                                                maniMorphTargets* targetsOut);

    /// Returns the number of vertices the targets move.
    ///
    /// @param targets  Opened morph targets.
    /// @return The vertex count; 0 for a view no open wrote or whose counts do
    /// not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_GetVertexCount(const maniMorphTargets* targets);

    /// Returns the number of channels, each weighted by its own value.
    ///
    /// @param targets  Opened morph targets.
    /// @return The channel count; 0 for a view no open wrote or whose counts do
    /// not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_GetChannelCount(const maniMorphTargets* targets);

    /// Returns the number of targets over all channels, in channel order.
    ///
    /// @param targets  Opened morph targets.
    /// @return The target count; 0 for a view no open wrote or whose counts do
    /// not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_GetTargetCount(const maniMorphTargets* targets);

    /// Returns the number of attributes the targets move.
    ///
    /// @param targets  Opened morph targets.
    /// @return The attribute count, at most MANI_MAX_MORPH_ATTRIBUTES; 0 for a
    /// view no open wrote or whose counts do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_GetAttributeCount(const maniMorphTargets* targets);

    /// Returns one of the attributes the targets move.
    ///
    /// @param targets    Opened morph targets.
    /// @param attribute  The attribute, in the file's order.
    /// @param setOut     Receives its set (0 for positions, normals and
    ///                   tangents); may be NULL.
    /// @return Its semantic; -1 (and set 0) past the count or for a view no
    /// open wrote or whose counts do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniMorphSemantic maniMorphTargets_GetAttribute(const maniMorphTargets* targets,
                                                             uint32_t attribute, uint32_t* setOut);

    /// Returns a channel's name, UTF-8 and not terminated.
    ///
    /// @param targets    Opened morph targets.
    /// @param channel    The channel.
    /// @param lengthOut  Receives the name's length in bytes, 0 for a
    ///                   channel past the count. May be NULL.
    /// @return The name's bytes, inside the file, or NULL for a channel past
    /// the count, a view no open wrote or whose counts do not fit its file, or
    /// a name span in bytes changed after the open past the names.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniMorphTargets_GetChannelName(const maniMorphTargets* targets,
                                                         uint32_t channel, size_t* lengthOut);

    /// Finds a channel by its name, in time linear in the channels.
    ///
    /// @param targets  Opened morph targets.
    /// @param name     The name's UTF-8 bytes; may be NULL when length is
    ///                 0.
    /// @param length   The name's length in bytes.
    /// @return The channel, or MANI_NO_CHANNEL when none has that name or for a
    /// view no open wrote or whose counts do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_FindChannel(const maniMorphTargets* targets,
                                                   const char* name, size_t length);

    /// Returns a target's name, UTF-8 and not terminated: the one it was
    /// built with, else its channel's, followed by '/' and its index
    /// within the channel past the first. Clip weight tracks (`<node or mesh>/<name>`)
    /// bind to targets by it.
    ///
    /// @param targets    Opened morph targets.
    /// @param target     The target.
    /// @param lengthOut  Receives the name's length in bytes, 0 for a
    ///                   target past the count. May be NULL.
    /// @return The name's bytes, inside the file, or NULL for a target past the
    /// count, a view no open wrote or whose counts do not fit its file, or a
    /// name span in bytes changed after the open past the names.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniMorphTargets_GetTargetName(const maniMorphTargets* targets,
                                                        uint32_t target, size_t* lengthOut);

    /// Finds a target by its name, in time linear in the targets: the
    /// first of that name.
    ///
    /// @param targets  Opened morph targets.
    /// @param name     The name's UTF-8 bytes; may be NULL when length is
    ///                 0.
    /// @param length   The name's length in bytes.
    /// @return The target, or MANI_NO_CHANNEL when none has that name or for a
    /// view no open wrote or whose counts do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_FindTarget(const maniMorphTargets* targets, const char* name,
                                                  size_t length);

    /// Returns a channel's targets: the first's index and their count.
    ///
    /// @param targets   Opened morph targets.
    /// @param channel   The channel.
    /// @param firstOut  Receives the first target's index; may be NULL.
    /// @return The number of targets, 0 for a channel past the count, a view no
    /// open wrote or whose counts do not fit its file, or targets in bytes
    /// changed after the open past the target count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniMorphTargets_GetChannelTargets(const maniMorphTargets* targets,
                                                         uint32_t channel, uint32_t* firstOut);

    /// Returns the channel weight at which a target applies in full.
    ///
    /// @param targets  Opened morph targets.
    /// @param target   The target.
    /// @return The full weight; 0 for a target past the count or a view no open
    /// wrote or whose counts do not fit its file.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API float maniMorphTargets_GetTargetFullWeight(const maniMorphTargets* targets,
                                                        uint32_t target);

    /// Turns channel weights into target weights, for the job or a
    /// shader: within each channel, the weight walks linearly from an
    /// implicit zero shape at 0 through the targets at their full
    /// weights, past the last (or first) extrapolated. A channel of one
    /// target at 1 passes its weight through. Call it with a NULL array
    /// and capacity 0 to measure.
    ///
    /// @param targets           Opened morph targets.
    /// @param channelWeights    A finite weight a channel, any sign.
    /// @param targetWeightsOut  Receives a weight a target; may be NULL
    ///                          when capacity is 0, and may be
    ///                          channelWeights, with the same results.
    /// @param capacity          The floats targetWeightsOut holds.
    /// @param countOut          Receives the target count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when capacity is below the
    /// target count, with the count still written and nothing else;
    /// `mani_errorInvalid` for a morph targets view no open wrote or whose
    /// counts do not fit its file, a NULL targets or weights, a NULL array with
    /// a positive capacity, or a weight not finite (nothing written). A channel
    /// whose targets in bytes changed after the open are past the target count
    /// writes no weight; changed bytes give some weights, read and written only
    /// within the file and the array.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMorphTargets_ComputeTargetWeights(
        const maniMorphTargets* targets, const float* channelWeights, float* targetWeightsOut,
        uint32_t capacity, uint32_t* countOut);

    // A vertex stream a morph job moves: an attribute's floats at in,
    // written at out; strides in bytes between vertices, at least 4 times
    // the components. out may be in, with the same stride.
    typedef struct maniMorphStream
    {
        const void* in;
        void* out;
        size_t inStride;
        size_t outStride;
        maniMorphSemantic semantic;
        // The set: 0 for positions, normals and tangents.
        uint32_t set;
        // 3 for positions and normals, 4 for tangents (w kept), 2 for
        // texture coordinates, 3 or 4 for colors.
        uint32_t components;
    } maniMorphStream;

    // A morph job over vertices [first, first + count) of the targets,
    // their streams' pointers at vertex first.
    typedef struct maniMorphJob
    {
        const maniMorphTargets* targets;
        // A weight a target (maniMorphTargets_ComputeTargetWeights), finite.
        const float* weights;
        uint32_t first;
        uint32_t count;
        uint32_t streamCount;
        // Each takes its attribute's deltas; one whose attribute the file
        // lacks is copied.
        const maniMorphStream* streams;
    } maniMorphJob;

    /// Applies morph targets: each stream's vertices plus its attribute's
    /// weighted deltas, target by target in order (so ranges and sparsity
    /// give the same bits), then every normal and tangent of the range
    /// renormalized (zero kept zero). Runs before skinning, into the
    /// skinning job's input.
    ///
    /// @param job  The job.
    /// @return `mani_success`; `mani_errorInvalid` for a morph targets view no
    /// open wrote or whose counts do not fit its file, a NULL argument, a
    /// weight not finite, a range past the vertices, or a stream without
    /// pointers, with a stride too small, an unknown semantic, a set beyond 0
    /// for positions, normals or tangents, or components other than its
    /// semantic's (nothing written). A target whose deltas or indices in bytes
    /// changed after the open are past their sections, or a dense one without
    /// every vertex, adds nothing, and a sparse vertex outside the range is
    /// skipped: changed bytes morph to some vertices, read and written only
    /// within the file and the streams.
    /// @par Thread safety
    /// Safe from any thread; the output range is used by one thread at a time.
    /// Jobs over disjoint ranges run at once.
    MANI_NODISCARD MANI_API maniResult maniApplyMorphTargets(const maniMorphJob* job);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MORPH_H
