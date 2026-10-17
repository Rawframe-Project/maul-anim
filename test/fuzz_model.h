// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A small learned model (mani-0025) over a fuzz target's database, for
// the hosts of the graph fuzz targets: 2 latent values, hidden layers of
// 4, one Projector of every frame, one event, fixed weights. Included by
// one fuzz target each.

#ifndef MAUL_ANIM_TEST_FUZZ_MODEL_H
#define MAUL_ANIM_TEST_FUZZ_MODEL_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FUZZ_MODEL_WIDTH 4u

// count values from the pool: weights (scale) about 0, or a constant.
static inline float* FuzzModelValues(uint32_t count, float scale, float constant)
{
    static float pool[4096];
    static uint32_t used;
    if (used + count > sizeof pool / sizeof pool[0])
    {
        used = 0;
    }
    float* v = pool + used;
    for (uint32_t k = 0; k < count; ++k)
    {
        v[k] = scale != 0.0f ? scale * sinf(1.7f * (float)(used + k) + 0.3f) : constant;
    }
    used += count;
    return v;
}

static inline maniLearnedNetworkSource FuzzModelNetwork(maniLearnedLayerSource* layers,
                                                        uint32_t inputs, uint32_t outputs)
{
    layers[0] = (maniLearnedLayerSource){FuzzModelValues(inputs * FUZZ_MODEL_WIDTH, 0.5f, 0.0f),
                                         FuzzModelValues(FUZZ_MODEL_WIDTH, 0.1f, 0.0f), inputs,
                                         FUZZ_MODEL_WIDTH};
    layers[1] =
        (maniLearnedLayerSource){FuzzModelValues(FUZZ_MODEL_WIDTH * outputs, 0.5f, 0.0f),
                                 FuzzModelValues(outputs, 0.1f, 0.0f), FUZZ_MODEL_WIDTH, outputs};
    return (maniLearnedNetworkSource){layers,
                                      FuzzModelValues(inputs, 0.0f, 0.0f),
                                      FuzzModelValues(inputs, 0.0f, 1.0f),
                                      FuzzModelValues(outputs, 0.0f, 0.0f),
                                      FuzzModelValues(outputs, 0.0f, 1.0f),
                                      2,
                                      0};
}

// The model over database (of skeleton), built into file and opened.
static inline bool MakeFuzzModel(maniBuilder* builder, const maniSkeleton* skeleton,
                                 const maniMotionDatabase* database, uint8_t* file, size_t capacity,
                                 maniLearnedModel* out)
{
    static maniLearnedLayerSource layers[3][2];
    static const maniLearnedNameSource kEvent = {"step", 4, 0};
    uint32_t bones = maniSkeletonBoneCount(skeleton);
    uint32_t features = database->featureCount;
    uint32_t state = features + 2;
    uint32_t outputs = 9 * bones + 6 + 1;
    maniLearnedNetworkSource decompressor = FuzzModelNetwork(layers[0], state, outputs);
    maniLearnedNetworkSource stepper = FuzzModelNetwork(layers[1], state, state);
    maniLearnedProjectorSource projector = {FuzzModelNetwork(layers[2], features, state), 0, 0};
    maniLearnedModelSource source = {.skeleton = skeleton,
                                     .database = database,
                                     .decompressor = &decompressor,
                                     .stepper = &stepper,
                                     .projectors = &projector,
                                     .scales = FuzzModelValues(3 * bones, 0.0f, 1.0f),
                                     .events = &kEvent,
                                     .projectorCount = 1,
                                     .eventCount = 1,
                                     .latentCount = 2,
                                     .stepRate = 10.0f,
                                     .weightFormat = maniLearnedWeightsFloat32};
    size_t size = 0;
    return maniBuildLearnedModel(builder, &source).result == mani_success &&
           maniCopyBuilt(builder, file, capacity, &size) == mani_success &&
           maniOpenLearnedModel(file, size, out).result == mani_success;
}

#endif // MAUL_ANIM_TEST_FUZZ_MODEL_H
