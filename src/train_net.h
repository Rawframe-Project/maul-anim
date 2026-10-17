// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A dense network in training (mani-0025): its parameters layer by layer
// (weights input by input, then biases), forward and backward over a
// chunk of samples. Every sum runs in a fixed order in binary32: the
// forward pass as a model's evaluation (biases, then each nonzero input
// times its weights, the output innermost), a weight's gradient over the
// chunk's samples in order, an input's gradient over the outputs in
// order. A batch is summed chunk by chunk in chunk order, so the bits do
// not depend on which thread ran a chunk.

#ifndef MAUL_ANIM_SRC_TRAIN_NET_H
#define MAUL_ANIM_SRC_TRAIN_NET_H

#include "train_random.h"

#include "maul-anim/learned.h"

#include <stdbool.h>
#include <stdint.h>

// Samples a chunk: a batch's gradients are summed over chunks of this
// many, whatever the threads.
#define MANI_TRAIN_CHUNK 4u

// The activation after every layer but the last: ReLU, or ELU (x for x
// above 0, else e^x - 1).
typedef uint32_t maniTrainActivation;
enum
{
    maniTrainRelu = 0,
    maniTrainElu = 1,
};

typedef struct maniTrainNet
{
    uint32_t layerCount;
    maniTrainActivation activation;
    uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1];
    // Where each layer's weights start among the parameters; its biases
    // follow them.
    uint64_t offsets[MANI_LEARNED_MAX_LAYERS];
    uint64_t parameterCount;
    // The values a sample holds over every layer, its inputs included.
    uint64_t valueCount;
    uint32_t widest;
    uint32_t reserved;
} maniTrainNet;

// A network through sizes[0] to sizes[layerCount]; false past the
// limits (MANI_LEARNED_MAX_LAYERS, MANI_LEARNED_MAX_WIDTH,
// MANI_LEARNED_MAX_WEIGHTS) or for a size of 0.
bool maniTrainNetInit(maniTrainNet* n, const uint32_t* sizes, uint32_t layerCount,
                      maniTrainActivation activation);

// PyTorch's default start: weights and biases uniform in +-1 / sqrt(the
// layer's inputs), drawn layer by layer, weights before biases.
void maniTrainNetStart(const maniTrainNet* n, float* parameters, maniTrainRandom* r);

// Where layer l's values for count samples start among a chunk's values
// (layer by layer, sample by sample within a layer).
uint64_t maniTrainLayerAt(const maniTrainNet* n, uint32_t l, uint32_t count);

// The forward pass of count samples (at most MANI_TRAIN_CHUNK) whose
// inputs are already in layer 0 of values.
void maniTrainForward(const maniTrainNet* n, const float* parameters, float* values,
                      uint32_t count);

// Each layer's weights, transposed (output by output), for the backward
// passes of a step: the parameter count of floats.
void maniTrainTranspose(const maniTrainNet* n, const float* parameters, float* transposed);

// The backward pass of count samples: adds to gradients the gradient of
// the loss whose gradient by the outputs is outputGradient (count by the
// last size); writes the gradient by the inputs to inputGradient (count
// by sizes[0]) when not NULL. scratch holds 2 * MANI_TRAIN_CHUNK *
// widest floats.
void maniTrainBackward(const maniTrainNet* n, const float* transposed, const float* values,
                       const float* outputGradient, uint32_t count, float* gradients,
                       float* inputGradient, float* scratch);

#endif // MAUL_ANIM_SRC_TRAIN_NET_H
