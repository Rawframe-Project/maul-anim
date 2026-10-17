// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One network trained alone (mani-0025), as the Stepper and each
// Projector are: its parameters, AdamW's moments, a step's transposed
// weights, each chunk's gradients and their sum, and the normalization
// of its inputs and outputs.

#ifndef MAUL_ANIM_SRC_TRAIN_SOLO_H
#define MAUL_ANIM_SRC_TRAIN_SOLO_H

#include "train_net.h"
#include "train_random.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stdint.h>

// A network's shape and batch: latent values, the hidden layers' width
// and count, samples a batch (a whole number of chunks), and frames a
// window (the Stepper's).
typedef struct maniTrainSoloShape
{
    uint32_t latentCount;
    uint32_t hiddenWidth;
    uint32_t hiddenLayers;
    uint32_t batch;
    uint32_t window;
    uint32_t reserved;
} maniTrainSoloShape;

// The sizes of a network from in through the shape's hidden layers to
// out; false when the layers pass MANI_LEARNED_MAX_LAYERS or the batch
// is not a whole number of chunks.
bool maniTrainSoloSizes(const maniTrainSoloShape* shape, uint32_t in, uint32_t out,
                        uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1]);

typedef struct maniTrainSolo
{
    maniTrainNet net;
    float* parameters;
    float* m;
    float* v;
    float* largest;
    float* transposed;
    float* sum;
    // Chunk c's gradients start at gradients + c * the parameter count.
    float* gradients;
    float* inputOffsets;
    float* inputScales;
    float* outputOffsets;
    float* outputScales;
    uint32_t chunkCount;
    uint32_t reserved;
} maniTrainSolo;

// A ReLU network through sizes[0] to sizes[layerCount] with chunkCount
// chunks, started from the generator. False past a limit or when memory
// runs out, with nothing held.
bool maniTrainSoloInit(const maniAllocator* allocator, const uint32_t* sizes, uint32_t layerCount,
                       uint32_t chunkCount, maniTrainRandom* r, maniTrainSolo* s);

void maniTrainSoloRelease(const maniAllocator* allocator, maniTrainSolo* s);

// Before a step's chunks: the transposed weights, every chunk's
// gradients 0.
void maniTrainSoloPrepare(maniTrainSolo* s);

// After them: the chunks' gradients summed in order, an AdamW step
// (number step, from 1) at rate.
void maniTrainSoloUpdate(maniTrainSolo* s, uint64_t step, float rate);

#endif // MAUL_ANIM_SRC_TRAIN_SOLO_H
