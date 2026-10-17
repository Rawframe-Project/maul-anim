// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A Projector's training (mani-0025), as Holden's train_projector.py:
// the network's input is a query of features normalized by each value's
// mean and one deviation over all; its output the features and latent
// values of the frame the match node's exact search finds for the query
// (the Projector's tag masks, no continuing frame, history, exclusions
// or transition cost), by each value's mean and deviation. A sample is a
// frame drawn from all frames, its features plus a spread uniform in
// [0, 1) times, per feature, a Gaussian value times the feature's
// deviation plus 1. The loss is L1: the features (1.0), the latent
// values (5.0), each the mean over its values, and the query's distance
// to the output against its distance to the target (0.3, the mean over
// the samples).

#ifndef MAUL_ANIM_SRC_TRAIN_PROJECTOR_H
#define MAUL_ANIM_SRC_TRAIN_PROJECTOR_H

#include "train_data.h"
#include "train_net.h"
#include "train_random.h"
#include "train_solo.h"

#include "maul-anim/base.h"
#include "maul-anim/match.h"

#include <stdbool.h>
#include <stdint.h>

// One chunk's room: the network's values, the queries, the gradient by
// the outputs, the backward pass's scratch; the targets' frames.
typedef struct maniTrainProjectorChunk
{
    float* values;
    float* queries;
    float* outputGradient;
    float* scratch;
    float loss;
    uint32_t targets[MANI_TRAIN_CHUNK];
} maniTrainProjectorChunk;

typedef struct maniTrainProjector
{
    maniTrainSolo solo;
    maniTrainProjectorChunk* chunks;
    // Each feature's deviation plus 1.
    float* noise;
    uint32_t latentCount;
    uint32_t requiredTags;
    uint32_t excludedTags;
    uint32_t reserved;
    const maniMotionDatabase* database;
    const maniTrainData* data;
    const float* latents;
} maniTrainProjector;

// The stage over the database, its data and every frame's latent values
// (frames by the shape's latent count), for the frames of entries with
// all of requiredTags and none of excludedTags: the network started from
// the generator, the normalization measured. False when no frame passes
// the tags, a shape is past a limit or memory runs out, with nothing
// held.
bool maniTrainProjectorInit(const maniAllocator* allocator, const maniMotionDatabase* database,
                            const maniTrainData* data, const float* latents,
                            const maniTrainSoloShape* shape, uint32_t requiredTags,
                            uint32_t excludedTags, maniTrainRandom* r, maniTrainProjector* t);

void maniTrainProjectorRelease(const maniAllocator* allocator, maniTrainProjector* t);

// Whether a frame passes the tags: an entry with all of required, none
// of excluded, frames, and jumps into it allowed.
bool maniTrainAnyFrame(const maniMotionDatabase* database, uint32_t required, uint32_t excluded);

// One iteration: the batch's queries drawn from the generator, the
// chunks run on the hooks, an AdamW step (number step, from 1) at rate.
// Returns the batch's loss.
float maniTrainProjectorStep(maniTrainProjector* t, maniTrainRandom* r, const maniTaskHooks* hooks,
                             uint64_t step, float rate);

// One chunk's targets, loss and gradients, its queries set (internal;
// tested).
void maniTrainProjectorRun(const maniTrainProjector* t, maniTrainProjectorChunk* c);

#endif // MAUL_ANIM_SRC_TRAIN_PROJECTOR_H
