// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The parts of a learned model's evaluation (mani-0025) a graph's node
// uses beside the public calls.

#ifndef MAUL_ANIM_SRC_LEARNED_EVAL_H
#define MAUL_ANIM_SRC_LEARNED_EVAL_H

#include "maul-anim/learned.h"

// The Decompressor's outputs after the bones' (the root's velocity and
// angular velocity, the float tracks, the events' states) for a state,
// denormalized into out: its hidden layers whole and those outputs only,
// each the same bits as a whole evaluation gives. scratch holds
// maniLearnedScratchSize bytes, 4-byte aligned; nothing is checked. With
// hidden (maniLearnedHiddenCount floats), the last layer's inputs are
// kept there.
void maniLearnedDecompressMotion(const maniLearnedModel* model, const float* state, float* out,
                                 void* scratch, float* hidden);

// The inputs of the Decompressor's last layer: the hidden values a
// decompression keeps for another at the same state (0 for a Decompressor
// of one layer, which keeps none).
uint32_t maniLearnedHiddenCount(const maniLearnedModel* model);

// maniLearnedDecompress taking, with hidden, the last layer's inputs as a
// decompression kept them at the same state, in place of every layer
// before it: the same bits. NULL computes them.
maniResult maniLearnedDecompressFrom(const maniLearnedModel* model, const float* state,
                                     maniLearnedOutput* output, void* scratch, size_t scratchSize,
                                     const float* hidden);

#endif // MAUL_ANIM_SRC_LEARNED_EVAL_H
