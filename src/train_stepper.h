// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Stepper's training, as Holden's train_stepper.py: a
// state is a frame's features and latent values; the network's input is
// the state normalized by each value's mean and one deviation over the
// features and one over the latent values, its output the state's rate
// of change by each value's mean and deviation over consecutive frames
// of one entry. A sample is a window of consecutive frames of one entry,
// predicted from its first frame step by step (the state plus the
// step's seconds times the output). The loss is L1: the features (2.0),
// the latent values (7.5), and their velocities across consecutive steps
// (0.2, 0.5), each the mean over its values; the backward pass runs
// through the window's steps in reverse.

#ifndef MAUL_ANIM_SRC_TRAIN_STEPPER_H
#define MAUL_ANIM_SRC_TRAIN_STEPPER_H

#include "train_data.h"
#include "train_net.h"
#include "train_random.h"
#include "train_solo.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stdint.h>

// One chunk's room: each step's network values, each step's states and
// the gradient by them, the gradients by the network's outputs and
// inputs, the backward pass's scratch.
typedef struct maniTrainStepperChunk
{
    float* values;
    float* states;
    float* stateGradient;
    float* outputGradient;
    float* inputGradient;
    float* scratch;
    float loss;
    uint32_t windows[MANI_TRAIN_CHUNK];
} maniTrainStepperChunk;

typedef struct maniTrainStepper
{
    maniTrainSolo solo;
    maniTrainStepperChunk* chunks;
    // The first frames of the windows that fit in one entry.
    uint32_t* windowStarts;
    uint32_t windowCount;
    uint32_t window;
    uint32_t latentCount;
    float seconds;
    // Set by a step for its chunks.
    const maniTrainData* data;
    const float* latents;
} maniTrainStepper;

// The stage over data and every frame's latent values (frames by the
// shape's latent count): the network started from the generator, the
// normalization measured. False when no window fits in an entry, a shape
// is past a limit or memory runs out, with nothing held.
bool maniTrainStepperInit(const maniAllocator* allocator, const maniTrainData* data,
                          const float* latents, const maniTrainSoloShape* shape, maniTrainRandom* r,
                          maniTrainStepper* t);

void maniTrainStepperRelease(const maniAllocator* allocator, maniTrainStepper* t);

// The windows of window frames within one entry: their first frames into
// starts (unless NULL), in frame order; returns their count, 0 for a
// window below 2.
uint32_t maniTrainWindows(const maniTrainData* d, uint32_t window, uint32_t* starts);

// One iteration: the batch's windows drawn from the generator, the
// chunks run on the hooks, an AdamW step (number step, from 1) at rate.
// Returns the batch's loss.
float maniTrainStepperStep(maniTrainStepper* t, maniTrainRandom* r, const maniTaskHooks* hooks,
                           uint64_t step, float rate);

// One chunk's loss and gradients, its windows set (internal; tested).
void maniTrainStepperRun(const maniTrainStepper* t, maniTrainStepperChunk* c);

#endif // MAUL_ANIM_SRC_TRAIN_STEPPER_H
