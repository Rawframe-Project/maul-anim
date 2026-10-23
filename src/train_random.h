// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The trainer's generator: xoshiro256** seeded by
// SplitMix64, as their authors give them, with uniform and Gaussian
// values made by the library's own arithmetic, so a seed gives the same
// values on every target.

#ifndef MAUL_ANIM_SRC_TRAIN_RANDOM_H
#define MAUL_ANIM_SRC_TRAIN_RANDOM_H

#include <stdint.h>

typedef struct maniTrainRandom
{
    uint64_t s[4];
} maniTrainRandom;

// The state from a seed: four SplitMix64 outputs.
void maniTrainSeed(maniTrainRandom* r, uint64_t seed);

// The next 64 bits.
uint64_t maniTrainNext(maniTrainRandom* r);

// A value in [0, 1): the top 24 bits over 2^24.
float maniTrainUniform(maniTrainRandom* r);

// A value in [0, n), n above 0, every one equally likely (Lemire's
// multiplication, its rejection keeping it exact).
uint32_t maniTrainBelow(maniTrainRandom* r, uint32_t n);

// A standard normal value: Box and Muller's, sqrt(-2 ln u) cos(2 pi v)
// for u in (0, 1] and v in [0, 1), in binary32 with the library's
// logarithm and cosine.
float maniTrainGaussian(maniTrainRandom* r);

#endif // MAUL_ANIM_SRC_TRAIN_RANDOM_H
