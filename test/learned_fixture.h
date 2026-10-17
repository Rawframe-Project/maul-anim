// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model (mani-0025) of random networks over the motion matching
// fixture's database, for the tests of models and of the graphs that
// play them.

#ifndef MAUL_ANIM_TEST_LEARNED_FIXTURE_H
#define MAUL_ANIM_TEST_LEARNED_FIXTURE_H

#include "graph_match_fixture.h"

#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"

#include <stddef.h>
#include <stdint.h>

// The fixture's database: two features (the walk's root half a second
// ahead); three latent values, a state of five. Outputs: nine a bone
// (three), three for the scaled bone 2, six for the root, a float track
// and two events: 39.
#define FEATURES 2u
#define LATENT   3u
#define STATE    5u
#define OUTPUTS  39u

static float s_pool[16384];
static size_t s_used;
static uint64_t s_random = 0x9E3779B97F4A7C15ull;

static inline float Random(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (float)(s_random >> 40) / 16777216.0f - 0.5f;
}

static inline float* Pool(size_t count, float low, float spread)
{
    float* p = s_pool + s_used;
    s_used += count;
    for (size_t k = 0; k < count; ++k)
    {
        p[k] = low + spread * Random();
    }
    return p;
}

// A network of random values through the sizes given (count + 1 of them).
static inline maniLearnedNetworkSource Network(maniLearnedLayerSource* layers,
                                               const uint32_t* sizes, uint32_t count)
{
    for (uint32_t l = 0; l < count; ++l)
    {
        layers[l] =
            (maniLearnedLayerSource){Pool((size_t)sizes[l] * sizes[l + 1], 0.0f, 1.0f),
                                     Pool(sizes[l + 1], 0.0f, 0.4f), sizes[l], sizes[l + 1]};
    }
    return (maniLearnedNetworkSource){layers,
                                      Pool(sizes[0], 0.0f, 1.0f),
                                      Pool(sizes[0], 1.25f, 1.5f),
                                      Pool(sizes[count], 0.0f, 1.0f),
                                      Pool(sizes[count], 1.25f, 1.5f),
                                      count,
                                      0};
}

static maniLearnedLayerSource s_layers[5][3];
static maniLearnedNetworkSource s_decompressor;
static maniLearnedNetworkSource s_stepper;
static maniLearnedProjectorSource s_projectors[2];
static const uint32_t kScaled[1] = {2};
static float s_scales[9] = {1, 1, 1, 2, 2, 2, 1, 1, 1};
static const maniLearnedNameSource kTracks[1] = {{"speed", 5, 0}};
static const maniLearnedNameSource kEvents[2] = {{"left", 4, 0}, {"right", 5, 0}};

static inline void MakeNetworks(void)
{
    static const uint32_t kDecompressor[3] = {STATE, 8, OUTPUTS};
    static const uint32_t kStepper[4] = {STATE, 8, 8, STATE};
    static const uint32_t kFirst[3] = {FEATURES, 7, STATE};
    static const uint32_t kSecond[3] = {FEATURES, 8, STATE};
    s_decompressor = Network(s_layers[0], kDecompressor, 2);
    s_stepper = Network(s_layers[1], kStepper, 3);
    s_projectors[0] = (maniLearnedProjectorSource){Network(s_layers[2], kFirst, 2), 1, 0};
    s_projectors[1] = (maniLearnedProjectorSource){Network(s_layers[3], kSecond, 2), 2, 1};
}

static inline maniLearnedModelSource Source(void)
{
    return (maniLearnedModelSource){.skeleton = &s_skeleton,
                                    .database = &s_database,
                                    .decompressor = &s_decompressor,
                                    .stepper = &s_stepper,
                                    .projectors = s_projectors,
                                    .scaledBones = kScaled,
                                    .scales = s_scales,
                                    .floatTracks = kTracks,
                                    .events = kEvents,
                                    .projectorCount = 2,
                                    .scaledBoneCount = 1,
                                    .floatTrackCount = 1,
                                    .eventCount = 2,
                                    .latentCount = LATENT,
                                    .stepRate = 60.0f,
                                    .weightFormat = maniLearnedWeightsFloat32};
}

static _Alignas(16) uint8_t s_model[65536];
static size_t s_modelSize;

static inline maniBuildResult Build(const maniLearnedModelSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildLearnedModel(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_model, sizeof s_model, &s_modelSize) == mani_success,
              "copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static inline bool Opened(const maniLearnedModelSource* source, maniLearnedModel* model)
{
    return Build(source).result == mani_success &&
           maniOpenLearnedModel(s_model, s_modelSize, model).result == mani_success;
}

#endif // MAUL_ANIM_TEST_LEARNED_FIXTURE_H
