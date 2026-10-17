// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The first training stage's state (mani-0025), shared by its driver
// (train_decompress) and its chunks (train_decompress_loss).

#ifndef MAUL_ANIM_SRC_TRAIN_STAGE_H
#define MAUL_ANIM_SRC_TRAIN_STAGE_H

#include "train_data.h"
#include "train_net.h"

#include <stdint.h>

// Pairs a chunk: two pairs, MANI_TRAIN_CHUNK frames.
#define MANI_TRAIN_PAIRS 2u

// The shape of the stage: latent values, hidden width, the hidden layer
// counts, pairs a batch (a whole number of chunks).
typedef struct maniTrainDecompressShape
{
    uint32_t latentCount;
    uint32_t hiddenWidth;
    uint32_t compressorLayers;
    uint32_t decompressorLayers;
    uint32_t batchPairs;
    uint32_t reserved;
} maniTrainDecompressShape;

// One chunk's room: both networks' values, the gradients by the
// outputs, inputs and latent values, the scratch of the backward passes.
typedef struct maniTrainDecompressChunk
{
    float* compressorValues;
    float* decompressorValues;
    float* outputGradient;
    float* inputGradient;
    float* latentGradient;
    float* scratch;
    // The outputs denormalized; the character pass's work, values and
    // their gradient, and its backward scratch.
    float* outputs;
    float* kinematics;
    float* character;
    float* characterGradient;
    float* kinematicScratch;
    float* gradients;
    float loss;
    uint32_t pairs[MANI_TRAIN_PAIRS];
} maniTrainDecompressChunk;

typedef struct maniTrainDecompress
{
    maniTrainNet compressor;
    maniTrainNet decompressor;
    // The parameters: the Compressor's, then the Decompressor's.
    uint64_t count;
    float* parameters;
    float* m;
    float* v;
    float* largest;
    float* transposed;
    float* sum;
    // The Compressor's inputs' offsets and scales; the Decompressor's
    // outputs' offsets and scales.
    float* inputOffsets;
    float* inputScales;
    float* outputOffsets;
    float* outputScales;
    maniTrainDecompressChunk* chunks;
    uint32_t chunkCount;
    uint32_t widest;
    // Set by a step for its chunks.
    const maniTrainData* data;
} maniTrainDecompress;

#endif // MAUL_ANIM_SRC_TRAIN_STAGE_H
