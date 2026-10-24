// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The trainer's generator; see train_random.h.

#include "train_random.h"

#include "elementary_float.h"

#include <math.h>
#include <stdint.h>

static uint64_t SplitMix(uint64_t* x)
{
    uint64_t z = (*x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

void maniTrainSeed(maniTrainRandom* r, uint64_t seed)
{
    for (uint32_t k = 0; k < 4; ++k)
    {
        r->s[k] = SplitMix(&seed);
    }
}

static uint64_t Rotate(uint64_t x, uint32_t k)
{
    return (x << k) | (x >> (64 - k));
}

uint64_t maniTrainNext(maniTrainRandom* r)
{
    uint64_t* s = r->s;
    uint64_t result = Rotate(s[1] * 5, 7) * 9;
    uint64_t t = s[1] << 17;
    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = Rotate(s[3], 45);
    return result;
}

float maniTrainUniform(maniTrainRandom* r)
{
    return (float)(maniTrainNext(r) >> 40) * 0x1.0p-24f;
}

uint32_t maniTrainBelow(maniTrainRandom* r, uint32_t n)
{
    uint64_t m = (maniTrainNext(r) >> 32) * n;
    uint32_t low = (uint32_t)m;
    if (low < n)
    {
        uint32_t floor = (uint32_t)(-n) % n;
        while (low < floor)
        {
            m = (maniTrainNext(r) >> 32) * n;
            low = (uint32_t)m;
        }
    }
    return (uint32_t)(m >> 32);
}

float maniTrainGaussian(maniTrainRandom* r)
{
    float u = 1.0f - maniTrainUniform(r);
    float v = maniTrainUniform(r);
    float sine = 0.0f;
    float cosine = 0.0f;
    maniSinCosF(6.2831853f * v, &sine, &cosine);
    return sqrtf(-2.0f * maniLogF(u)) * cosine;
}
