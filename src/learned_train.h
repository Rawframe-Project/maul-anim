// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's trainer (mani-0025): its stages in order, numbered
// as the model's networks (0 the Compressor and Decompressor, whose end
// gives every frame's latent values, 1 the Stepper, 2 + p Projector p),
// one in training at a time, each trained network kept (parameters and
// normalization) once its stage ends.

#ifndef MAUL_ANIM_SRC_LEARNED_TRAIN_H
#define MAUL_ANIM_SRC_LEARNED_TRAIN_H

#include "host_code.h"
#include "train_data.h"
#include "train_decompress.h"
#include "train_net.h"
#include "train_projector.h"
#include "train_random.h"
#include "train_stage.h"
#include "train_stepper.h"

#include "maul-anim/base.h"
#include "maul-anim/learnedtrain.h"

#include <stdbool.h>
#include <stdint.h>

// A trained network: its shape, parameters and normalization.
typedef struct maniTrainKept
{
    maniTrainNet net;
    float* parameters;
    float* inputOffsets;
    float* inputScales;
    float* outputOffsets;
    float* outputScales;
} maniTrainKept;

struct maniLearnedTrainer
{
    // The allocator and task hooks the training uses: the host's, the
    // trainer marked while they run (mani-0028).
    maniAllocator allocator;
    maniTaskHooks tasks;
    // The host's allocator and hooks, the mark and the misuses.
    maniHostCode host;
    maniLearnedTrainerSource source;
    maniTrainData data;
    uint64_t signature;
    uint32_t stage;
    uint32_t stageCount;
    // Iterations the stage has run; its last batch's loss.
    uint64_t iteration;
    float loss;
    // Whether the stage's state exists (a stage begins when first run).
    bool begun;
    maniTrainRandom random;
    maniTrainDecompress decompress;
    maniTrainStepper stepper;
    maniTrainProjector projector;
    // Frames by the latent count, once the first stage has ended.
    float* latents;
    // The model's networks: 0 the Decompressor, 1 the Stepper, 2 + p
    // Projector p; kept as their stages end.
    maniTrainKept* networks;
};

// Network n's shape: 0 the Compressor, 1 the Decompressor, 2 the
// Stepper, 3 a Projector; false past a limit.
bool maniLearnedTrainerShape(const maniLearnedTrainer* t, uint32_t n, maniTrainNet* net);

// The signature a checkpoint carries: the hash of the source's
// identities, options, filters and names.
uint64_t maniLearnedTrainerSignature(const maniLearnedTrainerSource* source);

// Room for network n's parameters and normalization, of shape net; false
// when memory runs out.
bool maniLearnedTrainerKeep(maniLearnedTrainer* t, uint32_t n, const maniTrainNet* net);

// The current stage begun: its generator seeded by the seed plus the
// stage, its network started, its normalization measured.
maniResult maniLearnedTrainerBegin(maniLearnedTrainer* t);

// The stage in training's parameters, first and second moments and their
// largest; returns the parameter count.
uint64_t maniLearnedTrainerArrays(const maniLearnedTrainer* t, float* arrays[4]);

// The stage in training released, its progress lost.
void maniLearnedTrainerDrop(maniLearnedTrainer* t);

// The kept networks and latent values released.
void maniLearnedTrainerForget(maniLearnedTrainer* t);

#endif // MAUL_ANIM_SRC_LEARNED_TRAIN_H
