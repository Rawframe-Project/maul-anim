// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The first stage of a learned model's training: the
// Compressor (ELU) and Decompressor (ReLU) trained together, as Holden's
// train_decompressor.py. A batch is pairs of consecutive frames of one
// entry; the Compressor maps each frame's inputs (normalized by each
// value's mean and each group's deviation) to latent values, and the
// Decompressor maps the frame's features and those latent values to its
// outputs (normalized by each value's mean and deviation). The loss is
// L1: the local positions (75), the rotation columns (10), the scales
// (10), the root's velocities (2 each), the float tracks and events (2),
// the local positions' and columns' velocities across a pair (10, 1.75),
// character space's translations and columns (15, 5) and their
// velocities across a pair (2, 0.75), through the character pass
// (train_kinematics), and the latent values' L1 (0.1), L2 (0.1) and
// change a second (0.01); each term the mean over its values, as
// torch.mean.

#ifndef MAUL_ANIM_SRC_TRAIN_DECOMPRESS_H
#define MAUL_ANIM_SRC_TRAIN_DECOMPRESS_H

#include "train_data.h"
#include "train_random.h"
#include "train_stage.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stdint.h>

// The stage over data: networks started from the generator, the
// normalization measured. False when a shape is past a limit or memory
// runs out, with nothing held.
bool maniTrainDecompressInit(const maniAllocator* allocator, const maniTrainData* data,
                             const maniTrainDecompressShape* shape, maniTrainRandom* r,
                             maniTrainDecompress* t);

void maniTrainDecompressRelease(const maniAllocator* allocator, maniTrainDecompress* t);

// One iteration: the batch's pairs drawn from the generator, the chunks
// run on the hooks, their gradients summed in order, an AdamW step
// (number step, from 1) at rate. Returns the batch's loss.
float maniTrainDecompressStep(maniTrainDecompress* t, const maniTrainData* data, maniTrainRandom* r,
                              const maniTaskHooks* hooks, uint64_t step, float rate);

// Every frame's latent values (frames by the latent count), by the
// trained Compressor.
void maniTrainLatents(const maniTrainDecompress* t, const maniTrainData* data, float* latents);

#endif // MAUL_ANIM_SRC_TRAIN_DECOMPRESS_H
