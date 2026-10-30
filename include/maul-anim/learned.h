// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned motion matching: a model of three networks that
// stands in for a motion database and its clips. The Projector maps a
// query to the state of the nearest frame it learned (its features and
// latent values), the Stepper moves a state one step on, and the
// Decompressor gives the pose, the root's motion, the float tracks and
// the events' states of a state. Models are built offline and opened in
// place; every evaluation gives the same bits on every target.

#ifndef MAUL_ANIM_LEARNED_H
#define MAUL_ANIM_LEARNED_H

#include "maul-anim/base.h"
#include "maul-anim/match.h"
#include "maul-anim/pose.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Named limits: layers of one network, values one layer takes or gives,
// weights of one model, latent values of a state, Projectors, float
// tracks and events of one model, and the highest step rate (steps a
// second).
#define MANI_MAX_LEARNED_LAYERS       16u
#define MANI_MAX_LEARNED_WIDTH        1048576u
#define MANI_MAX_LEARNED_WEIGHTS      268435456u
#define MANI_MAX_LEARNED_LATENT       256u
#define MANI_MAX_LEARNED_PROJECTORS   32u
#define MANI_MAX_LEARNED_FLOAT_TRACKS 256u
#define MANI_MAX_LEARNED_EVENTS       256u
#define MANI_MAX_LEARNED_RATE         1000.0f

    // How a model keeps its weights: binary32, or 16-bit integers with a
    // binary32 scale for each of a layer's inputs (half the size).
    typedef uint32_t maniLearnedWeights;
    enum
    {
        maniLearnedWeightsFloat32 = 0,
        maniLearnedWeightsInt16 = 1,
    };

    // An opened model: a view over the caller's bytes, which must outlive
    // it. A state is the model's feature count of normalized features,
    // then its latent count of latent values.
    typedef struct maniLearnedModel
    {
        const uint8_t* channels;
        const uint8_t* times;
        const uint8_t* normalization;
        const uint8_t* bones;
        const uint8_t* networks;
        const uint8_t* layers;
        const uint8_t* values;
        const uint8_t* weights;
        const uint8_t* names;
        const uint8_t* nameBytes;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint64_t databaseIdentity;
        uint64_t nameBytesSize;
        uint32_t boneCount;
        uint32_t featureCount;
        uint32_t channelCount;
        uint32_t timeCount;
        uint32_t latentCount;
        uint32_t scaledBoneCount;
        uint32_t floatTrackCount;
        uint32_t eventCount;
        uint32_t projectorCount;
        uint32_t layerCount;
        uint32_t outputCount;
        uint32_t widest;
        maniLearnedWeights weightFormat;
        float stepRate;
        maniMatchAxis up;
        maniMatchAxis forward;
    } maniLearnedModel;

    /// Opens a model file in place: checks the container, every record,
    /// count and float, that each network's layers chain and fit its role
    /// (the Decompressor from a state to the outputs, the Stepper from a
    /// state to a state's rate of change, each Projector from features to
    /// a state), and the names.
    ///
    /// @param bytes     The file, 16-byte aligned.
    /// @param size      Its size in bytes; bytes past the file are ignored.
    /// @param modelOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for any other format version.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenLearnedModel(const void* bytes, size_t size,
                                                                maniLearnedModel* modelOut);

    /// Returns a model's identity: the hash of its sections.
    ///
    /// @param model  An opened model, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniLearnedModel_GetIdentity(const maniLearnedModel* model);

    /// Returns the identity of the skeleton a model was built for.
    ///
    /// @param model  An opened model, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniLearnedModel_GetSkeletonIdentity(const maniLearnedModel* model);

    /// Returns the identity of the motion database a model learned.
    ///
    /// @param model  An opened model, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniLearnedModel_GetDatabaseIdentity(const maniLearnedModel* model);

    /// Reads a channel of the database a model learned (a query is laid
    /// out as that database's features).
    ///
    /// @param model       An opened model.
    /// @param channel     The channel's index.
    /// @param channelOut  Receives the channel.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// an index past the channels.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniLearnedModel_GetChannel(const maniLearnedModel* model,
                                                                   uint32_t channel,
                                                                   maniMatchChannel* channelOut);

    /// Reads the normalization of the database a model learned: per
    /// feature, the offset taken from a value and the scale it is divided
    /// by. Call it with both arrays NULL and capacity 0 to measure.
    ///
    /// @param model       An opened model.
    /// @param offsetsOut  Receives the feature count of offsets, or NULL.
    /// @param scalesOut   Receives the feature count of scales, or NULL.
    /// @param capacity    The floats each array given holds.
    /// @param countOut    Receives the feature count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when capacity is below
    /// the feature count, with the count still written and nothing else;
    /// `mani_errorInvalid` for a NULL model, or both arrays NULL with a
    /// positive capacity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult
    maniLearnedModel_GetNormalization(const maniLearnedModel* model, float* offsetsOut,
                                      float* scalesOut, uint32_t capacity, uint32_t* countOut);

    /// Reads the tag filter a Projector learned: an entry's tags held all
    /// of required and none of excluded for its frames to be targets.
    ///
    /// @param model        An opened model.
    /// @param projector    The Projector's index.
    /// @param requiredOut  Receives the required tags, or NULL.
    /// @param excludedOut  Receives the excluded tags, or NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL model or an
    /// index past the Projectors.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniLearnedModel_GetProjector(const maniLearnedModel* model,
                                                                     uint32_t projector,
                                                                     uint32_t* requiredOut,
                                                                     uint32_t* excludedOut);

    /// Returns the number of float tracks a model gives.
    ///
    /// @param model  An opened model, or NULL.
    /// @return The count; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniLearnedModel_GetFloatTrackCount(const maniLearnedModel* model);

    /// Returns the number of events a model gives the state of.
    ///
    /// @param model  An opened model, or NULL.
    /// @return The count; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniLearnedModel_GetEventCount(const maniLearnedModel* model);

    /// Returns the name of a float track a model gives.
    ///
    /// @param model      An opened model.
    /// @param track      The track's index.
    /// @param lengthOut  Receives the name's length in bytes, 0 for a NULL
    ///                   name; or NULL.
    /// @return The name, UTF-8, not terminated; NULL for a NULL model or
    /// an index past the tracks.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniLearnedModel_GetFloatTrackName(const maniLearnedModel* model,
                                                            uint32_t track, size_t* lengthOut);

    /// Returns the name of an event a model gives the state of.
    ///
    /// @param model      An opened model.
    /// @param event      The event's index.
    /// @param lengthOut  Receives the name's length in bytes, 0 for a NULL
    ///                   name; or NULL.
    /// @return The name, UTF-8, not terminated; NULL for a NULL model or
    /// an index past the events.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniLearnedModel_GetEventName(const maniLearnedModel* model,
                                                       uint32_t event, size_t* lengthOut);

    /// Returns the bytes of scratch memory an evaluation of the model
    /// needs.
    ///
    /// @param model  An opened model, or NULL.
    /// @return The size; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniLearnedModel_GetScratchSize(const maniLearnedModel* model);

    /// Projects a query onto the state of the nearest frame a Projector
    /// learned.
    ///
    /// Call it with a NULL state and capacity 0 to measure.
    ///
    /// @param model        An opened model.
    /// @param projector    The Projector's index.
    /// @param query        The model's feature count of features,
    ///                     normalized as the database's.
    /// @param stateOut     Receives the state; may be NULL when capacity is
    ///                     0.
    /// @param capacity     The floats stateOut holds.
    /// @param countOut     Receives the state's count of floats: the
    ///                     feature count and the latent count. May be NULL.
    /// @param scratch      Scratch memory, 4-byte aligned.
    /// @param scratchSize  Its size: at least maniLearnedModel_GetScratchSize.
    /// @return `mani_success`; `mani_errorCapacity` when capacity is below
    /// the state's count, with the count still written and nothing else;
    /// `mani_errorFormat` when the Projector gives a value past the float
    /// range, the model's data (stateOut unchanged); `mani_errorInvalid`
    /// for a NULL argument, a NULL state with a positive capacity, an index
    /// past the Projectors, a query value not finite, misaligned or too
    /// little scratch.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniLearnedModel_Project(
        const maniLearnedModel* model, uint32_t projector, const float* query, float* stateOut,
        uint32_t capacity, uint32_t* countOut, void* scratch, size_t scratchSize);

    /// Moves a state one step on: adds the Stepper's rate of change times
    /// one step's seconds (1 / the model's step rate).
    ///
    /// @param model        An opened model.
    /// @param state        The state, moved in place.
    /// @param scratch      Scratch memory, 4-byte aligned.
    /// @param scratchSize  Its size: at least maniLearnedModel_GetScratchSize.
    /// @return `mani_success`; `mani_errorFormat` when the step would take
    /// a value past the float range, the model's data (the state
    /// unchanged); `mani_errorInvalid` for a NULL argument, a state value
    /// not finite, misaligned or too little scratch.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniLearnedModel_Step(const maniLearnedModel* model,
                                                             float* state, void* scratch,
                                                             size_t scratchSize);

    // What the Decompressor gives for a state: the pose (every bone's
    // local transform), the root's velocity and angular velocity (a
    // scaled axis, radians a second) in its own frame, the float tracks'
    // values and the events' states (an event fires as its state rises
    // past 0.5). Any of pose, floatTracks and events may be NULL; each
    // array given holds its capacity of floats, at least the model's
    // count (maniLearnedModel_GetFloatTrackCount, maniLearnedModel_GetEventCount).
    typedef struct maniLearnedOutput
    {
        maniPose* pose;
        float* floatTracks;
        uint32_t floatTrackCapacity;
        float* events;
        uint32_t eventCapacity;
        maniVec3 rootVelocity;
        maniVec3 rootAngularVelocity;
    } maniLearnedOutput;

    /// Decompresses a state: every bone's translation, its rotation from
    /// two matrix columns (made orthonormal, the first kept), its scale
    /// (given, or the constant the model keeps), and the rest of the
    /// outputs.
    ///
    /// @param model        An opened model.
    /// @param state        The state.
    /// @param output       Receives what the Decompressor gives.
    /// @param scratch      Scratch memory, 4-byte aligned.
    /// @param scratchSize  Its size: at least maniLearnedModel_GetScratchSize.
    /// @return `mani_success`; `mani_errorFormat` when the Decompressor
    /// gives a value past the float range, the model's data (nothing
    /// written); `mani_errorCapacity` when an array given holds fewer
    /// floats than the model gives (nothing written); `mani_errorInvalid`
    /// for a NULL argument, a state value not finite, a pose of another
    /// bone count or holding a difference, misaligned or too little
    /// scratch.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniLearnedModel_Decompress(const maniLearnedModel* model,
                                                                   const float* state,
                                                                   maniLearnedOutput* output,
                                                                   void* scratch,
                                                                   size_t scratchSize);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_LEARNED_H
