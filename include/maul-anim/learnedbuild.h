// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building learned motion matching models from networks'
// weights: what a trainer gives, or networks trained elsewhere.

#ifndef MAUL_ANIM_LEARNEDBUILD_H
#define MAUL_ANIM_LEARNEDBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A dense layer: inputCount by outputCount weights, input by input
    // (the weight from input i to output o at i * outputCount + o), and
    // outputCount biases; every value finite.
    typedef struct maniLearnedLayerSource
    {
        const float* weights;
        const float* biases;
        uint32_t inputCount;
        uint32_t outputCount;
    } maniLearnedLayerSource;

    // A network: its layers (each but the last followed by ReLU), the
    // offsets taken from its inputs and the scales they are then divided
    // by (finite, not 0), and the scales its outputs are multiplied by and
    // the offsets then added (finite).
    typedef struct maniLearnedNetworkSource
    {
        const maniLearnedLayerSource* layers;
        const float* inputOffsets;
        const float* inputScales;
        const float* outputOffsets;
        const float* outputScales;
        uint32_t layerCount;
    } maniLearnedNetworkSource;

    // A Projector and the tag filter its targets passed.
    typedef struct maniLearnedProjectorSource
    {
        maniLearnedNetworkSource network;
        uint32_t requiredTags;
        uint32_t excludedTags;
    } maniLearnedProjectorSource;

    // A name: UTF-8, not empty.
    typedef struct maniLearnedNameSource
    {
        const char* name;
        size_t length;
    } maniLearnedNameSource;

    // A model: the skeleton and the motion database it learned (of that
    // skeleton); the latent count (1 to MANI_MAX_LEARNED_LATENT); the step
    // rate (above 0, at most MANI_MAX_LEARNED_RATE); the bones whose scale
    // the Decompressor gives (increasing) and the scale of every bone
    // (three values a bone, finite; NULL for 1), kept for the others; the
    // float tracks and events it gives (names unique within each list);
    // the networks; and how the file keeps the weights.
    //
    // The Decompressor takes a state and gives, bone by bone, the
    // translation and two rotation columns (nine values), then three
    // values for each scaled bone, the root's velocity and angular
    // velocity (six), a value for each float track and one for each
    // event. The Stepper takes a state and gives its rate of change a
    // second. A Projector takes the database's features, normalized, and
    // gives a state.
    typedef struct maniLearnedModelSource
    {
        const maniSkeleton* skeleton;
        const maniMotionDatabase* database;
        const maniLearnedNetworkSource* decompressor;
        const maniLearnedNetworkSource* stepper;
        const maniLearnedProjectorSource* projectors;
        const uint32_t* scaledBones;
        const float* scales;
        const maniLearnedNameSource* floatTracks;
        const maniLearnedNameSource* events;
        uint32_t projectorCount;
        uint32_t scaledBoneCount;
        uint32_t floatTrackCount;
        uint32_t eventCount;
        uint32_t latentCount;
        float stepRate;
        maniLearnedWeights weightFormat;
    } maniLearnedModelSource;

    /// Builds a model: checks the networks' shapes against their roles
    /// and every value, copies the database's channels, normalization and
    /// axes, and writes the weights as asked (16-bit: each input's
    /// weights divided by the largest of their magnitudes over 32767,
    /// rounded to nearest, ties to even, that scale kept).
    ///
    /// @param builder  The builder.
    /// @param source   The model.
    /// @return `mani_success`; `mani_errorLimit` with the reason and item
    /// for a source past a bound (no misuse): `maniBuildReasonTiming` (a
    /// step rate past MANI_MAX_LEARNED_RATE), `maniBuildReasonOption`
    /// (item 0: more than MANI_MAX_LEARNED_LATENT latent values),
    /// `maniBuildReasonCount` (more than MANI_MAX_LEARNED_PROJECTORS
    /// Projectors, MANI_MAX_LEARNED_FLOAT_TRACKS float tracks or
    /// MANI_MAX_LEARNED_EVENTS events), `maniBuildReasonNetwork` (more
    /// than MANI_MAX_LEARNED_LAYERS layers, a layer wider than
    /// MANI_MAX_LEARNED_WIDTH or more than MANI_MAX_LEARNED_WEIGHTS
    /// weights; item and other as below); `mani_errorInvalid` for a NULL
    /// argument (a misuse), or with the reason and item for a refused
    /// source: `maniBuildReasonDatabase` (a database of another
    /// skeleton), `maniBuildReasonTiming` (a step rate not above 0),
    /// `maniBuildReasonOption` (no latent values or the weight format),
    /// `maniBuildReasonCount` (no Projector), `maniBuildReasonJoint` (a
    /// scaled bone past the skeleton or not increasing; item its place),
    /// `maniBuildReasonNotFinite` (a bone's scale; item the bone),
    /// `maniBuildReasonNetwork` (item the network: 0 the Decompressor, 1
    /// the Stepper, 2 + p Projector p; other its layer, or its layer count
    /// for its normalization), `maniBuildReasonFloatTrack` or
    /// `maniBuildReasonEvent` combined with `maniBuildReasonName` or
    /// `maniBuildReasonDuplicateName`; `mani_errorCapacity` when memory
    /// runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult
    maniBuildLearnedModel(maniBuilder* builder, const maniLearnedModelSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_LEARNEDBUILD_H
