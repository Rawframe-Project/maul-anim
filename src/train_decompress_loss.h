// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One chunk of the first stage (mani-0025): both networks forward over
// its two pairs, the loss and its gradient by the outputs and latent
// values, both networks backward into the chunk's gradients.

#ifndef MAUL_ANIM_SRC_TRAIN_DECOMPRESS_LOSS_H
#define MAUL_ANIM_SRC_TRAIN_DECOMPRESS_LOSS_H

#include "train_stage.h"

// Runs chunk c of a batch of batchPairs pairs; the transposed weights are
// the step's.
void maniTrainDecompressRun(const maniTrainDecompress* t, maniTrainDecompressChunk* c,
                            uint32_t batchPairs);

#endif // MAUL_ANIM_SRC_TRAIN_DECOMPRESS_LOSS_H
