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
// maniLearnedScratchSize bytes, 4-byte aligned; nothing is checked.
void maniLearnedDecompressMotion(const maniLearnedModel* model, const float* state, float* out,
                                 void* scratch);

#endif // MAUL_ANIM_SRC_LEARNED_EVAL_H
