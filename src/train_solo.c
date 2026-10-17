// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One network trained alone (mani-0025); see train_solo.h.

#include "train_solo.h"

#include "allocator.h"
#include "train_adam.h"
#include "train_net.h"
#include "train_random.h"
#include "train_tasks.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MANI_TRAIN_SOLO_ARRAYS 11u

static void ArrayList(maniTrainSolo* s, float** list[MANI_TRAIN_SOLO_ARRAYS],
                      size_t sizes[MANI_TRAIN_SOLO_ARRAYS])
{
    size_t count = s->net.parameterCount;
    size_t in = s->net.sizes[0];
    size_t out = s->net.sizes[s->net.layerCount];
    float** fields[MANI_TRAIN_SOLO_ARRAYS] = {&s->parameters,  &s->m,
                                              &s->v,           &s->largest,
                                              &s->transposed,  &s->sum,
                                              &s->gradients,   &s->inputOffsets,
                                              &s->inputScales, &s->outputOffsets,
                                              &s->outputScales};
    const size_t counts[MANI_TRAIN_SOLO_ARRAYS] = {
        count, count, count, count, count, count, count * s->chunkCount, in, in, out, out};
    for (uint32_t k = 0; k < MANI_TRAIN_SOLO_ARRAYS; ++k)
    {
        list[k] = fields[k];
        sizes[k] = counts[k] + 1;
    }
}

void maniTrainSoloRelease(const maniAllocator* allocator, maniTrainSolo* s)
{
    if (s->chunkCount > 0)
    {
        float** arrays[MANI_TRAIN_SOLO_ARRAYS];
        size_t sizes[MANI_TRAIN_SOLO_ARRAYS];
        ArrayList(s, arrays, sizes);
        for (uint32_t k = 0; k < MANI_TRAIN_SOLO_ARRAYS; ++k)
        {
            maniRelease(allocator, *arrays[k], sizes[k], sizeof(float), 8);
        }
    }
    *s = (maniTrainSolo){0};
}

bool maniTrainSoloInit(const maniAllocator* allocator, const uint32_t* sizes, uint32_t layerCount,
                       uint32_t chunkCount, maniTrainRandom* r, maniTrainSolo* s)
{
    *s = (maniTrainSolo){0};
    if (chunkCount == 0 || !maniTrainNetInit(&s->net, sizes, layerCount, maniTrainRelu))
    {
        return false;
    }
    s->chunkCount = chunkCount;
    float** arrays[MANI_TRAIN_SOLO_ARRAYS];
    size_t counts[MANI_TRAIN_SOLO_ARRAYS];
    ArrayList(s, arrays, counts);
    bool ok = true;
    for (uint32_t k = 0; k < MANI_TRAIN_SOLO_ARRAYS; ++k)
    {
        *arrays[k] = maniAllocate(allocator, counts[k], sizeof(float), 8);
        ok = ok && *arrays[k] != NULL;
    }
    if (!ok)
    {
        maniTrainSoloRelease(allocator, s);
        return false;
    }
    size_t count = s->net.parameterCount;
    memset(s->m, 0, sizeof(float) * count);
    memset(s->v, 0, sizeof(float) * count);
    memset(s->largest, 0, sizeof(float) * count);
    maniTrainNetStart(&s->net, s->parameters, r);
    return true;
}

bool maniTrainSoloSizes(const maniTrainSoloShape* shape, uint32_t in, uint32_t out,
                        uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1])
{
    if (shape->hiddenLayers >= MANI_LEARNED_MAX_LAYERS || shape->batch == 0 ||
        shape->batch % MANI_TRAIN_CHUNK != 0)
    {
        return false;
    }
    sizes[0] = in;
    for (uint32_t l = 1; l <= shape->hiddenLayers; ++l)
    {
        sizes[l] = shape->hiddenWidth;
    }
    sizes[shape->hiddenLayers + 1] = out;
    return true;
}

void maniTrainSoloPrepare(maniTrainSolo* s)
{
    maniTrainTranspose(&s->net, s->parameters, s->transposed);
    memset(s->gradients, 0, sizeof(float) * (size_t)s->net.parameterCount * s->chunkCount);
}

void maniTrainSoloUpdate(maniTrainSolo* s, uint64_t step, float rate)
{
    maniTrainSum(s->gradients, s->chunkCount, s->net.parameterCount, s->sum);
    maniTrainAdam adam = maniTrainAdamDefaults();
    maniTrainAdamStep(&adam, step, rate, s->parameters, s->sum, s->m, s->v, s->largest,
                      s->net.parameterCount);
}
