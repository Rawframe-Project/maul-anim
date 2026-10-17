// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Stepper's training (mani-0025); see train_stepper.h.

#include "train_stepper.h"

#include "allocator.h"
#include "train_data.h"
#include "train_net.h"
#include "train_random.h"
#include "train_solo.h"
#include "train_tasks.h"

#include "maul-anim/base.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MANI_TRAIN_STEPPER_ARRAYS 6u

// A chunk's arrays and their sizes in floats.
static void ChunkList(const maniTrainStepper* t, maniTrainStepperChunk* c,
                      float** list[MANI_TRAIN_STEPPER_ARRAYS],
                      size_t sizes[MANI_TRAIN_STEPPER_ARRAYS])
{
    size_t chunk = MANI_TRAIN_CHUNK;
    size_t states = t->solo.net.sizes[0];
    size_t window = t->window;
    float** fields[MANI_TRAIN_STEPPER_ARRAYS] = {&c->values,        &c->states,
                                                 &c->stateGradient, &c->outputGradient,
                                                 &c->inputGradient, &c->scratch};
    const size_t counts[MANI_TRAIN_STEPPER_ARRAYS] = {(window - 1) * chunk *
                                                          (size_t)t->solo.net.valueCount,
                                                      window * chunk * states,
                                                      window * chunk * states,
                                                      chunk * states,
                                                      chunk * states,
                                                      2 * chunk * (size_t)t->solo.net.widest};
    for (uint32_t k = 0; k < MANI_TRAIN_STEPPER_ARRAYS; ++k)
    {
        list[k] = fields[k];
        sizes[k] = counts[k] + 1;
    }
}

void maniTrainStepperRelease(const maniAllocator* allocator, maniTrainStepper* t)
{
    for (uint32_t k = 0; t->chunks != NULL && k < t->solo.chunkCount; ++k)
    {
        float** arrays[MANI_TRAIN_STEPPER_ARRAYS];
        size_t sizes[MANI_TRAIN_STEPPER_ARRAYS];
        ChunkList(t, &t->chunks[k], arrays, sizes);
        for (uint32_t i = 0; i < MANI_TRAIN_STEPPER_ARRAYS; ++i)
        {
            maniRelease(allocator, *arrays[i], sizes[i], sizeof(float), 8);
        }
    }
    maniRelease(allocator, t->chunks, t->solo.chunkCount, sizeof(maniTrainStepperChunk), 8);
    if (t->data != NULL)
    {
        maniRelease(allocator, t->windowStarts, (size_t)t->data->pairCount + 1, sizeof(uint32_t),
                    4);
    }
    maniTrainSoloRelease(allocator, &t->solo);
    *t = (maniTrainStepper){0};
}

// The windows of t->window frames within one entry: a pair's first frame
// whose next window - 2 pairs follow it frame by frame.
static void Windows(maniTrainStepper* t, const maniTrainData* d)
{
    uint32_t span = t->window - 2;
    for (uint32_t p = 0; p + span < d->pairCount; ++p)
    {
        if (d->pairs[p + span] - d->pairs[p] == span)
        {
            t->windowStarts[t->windowCount++] = d->pairs[p];
        }
    }
}

// The input's offsets and group deviations over the frames' states; the
// output's offsets and deviations over the pairs' rates of change.
static bool Normalize(const maniAllocator* allocator, maniTrainStepper* t, const maniTrainData* d)
{
    uint32_t f = d->featureCount;
    uint32_t l = t->latentCount;
    maniTrainSolo* s = &t->solo;
    maniTrainMeasure(d->features, f, d->frameCount, 0, f, s->inputOffsets, s->inputScales, true);
    maniTrainMeasure(t->latents, l, d->frameCount, 0, l, s->inputOffsets + f, s->inputScales + f,
                     true);
    size_t width = (size_t)f + l;
    float* rates = maniAllocate(allocator, (size_t)d->pairCount * width + 1, sizeof(float), 8);
    if (rates == NULL)
    {
        return false;
    }
    for (uint32_t p = 0; p < d->pairCount; ++p)
    {
        size_t a = d->pairs[p];
        float* row = rates + (size_t)p * width;
        for (uint32_t j = 0; j < f; ++j)
        {
            row[j] = (d->features[(a + 1) * f + j] - d->features[a * f + j]) / t->seconds;
        }
        for (uint32_t j = 0; j < l; ++j)
        {
            row[f + j] = (t->latents[(a + 1) * l + j] - t->latents[a * l + j]) / t->seconds;
        }
    }
    maniTrainMeasure(rates, (uint32_t)width, d->pairCount, 0, (uint32_t)width, s->outputOffsets,
                     s->outputScales, false);
    maniRelease(allocator, rates, (size_t)d->pairCount * width + 1, sizeof(float), 8);
    return true;
}

static bool Chunks(const maniAllocator* allocator, maniTrainStepper* t)
{
    t->chunks = maniAllocate(allocator, t->solo.chunkCount, sizeof(maniTrainStepperChunk), 8);
    if (t->chunks == NULL)
    {
        return false;
    }
    memset(t->chunks, 0, sizeof(maniTrainStepperChunk) * t->solo.chunkCount);
    bool ok = true;
    for (uint32_t k = 0; k < t->solo.chunkCount; ++k)
    {
        float** arrays[MANI_TRAIN_STEPPER_ARRAYS];
        size_t sizes[MANI_TRAIN_STEPPER_ARRAYS];
        ChunkList(t, &t->chunks[k], arrays, sizes);
        for (uint32_t i = 0; i < MANI_TRAIN_STEPPER_ARRAYS; ++i)
        {
            *arrays[i] = maniAllocate(allocator, sizes[i], sizeof(float), 8);
            ok = ok && *arrays[i] != NULL;
        }
    }
    return ok;
}

bool maniTrainStepperInit(const maniAllocator* allocator, const maniTrainData* data,
                          const float* latents, const maniTrainSoloShape* shape, maniTrainRandom* r,
                          maniTrainStepper* t)
{
    *t = (maniTrainStepper){.window = shape->window,
                            .latentCount = shape->latentCount,
                            .seconds = data->frameSeconds,
                            .latents = latents};
    uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1];
    uint32_t states = data->featureCount + shape->latentCount;
    if (shape->window < 2 || !maniTrainSoloSizes(shape, states, states, sizes))
    {
        return false;
    }
    t->data = data;
    t->windowStarts = maniAllocate(allocator, (size_t)data->pairCount + 1, sizeof(uint32_t), 4);
    if (t->windowStarts == NULL)
    {
        maniTrainStepperRelease(allocator, t);
        return false;
    }
    Windows(t, data);
    bool ok = t->windowCount > 0 &&
              maniTrainSoloInit(allocator, sizes, shape->hiddenLayers + 1,
                                shape->batch / MANI_TRAIN_CHUNK, r, &t->solo) &&
              Chunks(allocator, t) && Normalize(allocator, t, data);
    if (!ok)
    {
        maniTrainStepperRelease(allocator, t);
    }
    return ok;
}

static void Task(int32_t start, int32_t end, void* context)
{
    maniTrainStepper* t = context;
    for (int32_t k = start; k < end; ++k)
    {
        maniTrainStepperRun(t, &t->chunks[k]);
    }
}

float maniTrainStepperStep(maniTrainStepper* t, maniTrainRandom* r, const maniTaskHooks* hooks,
                           uint64_t step, float rate)
{
    for (uint32_t k = 0; k < t->solo.chunkCount; ++k)
    {
        for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
        {
            t->chunks[k].windows[s] = t->windowStarts[maniTrainBelow(r, t->windowCount)];
        }
    }
    maniTrainSoloPrepare(&t->solo);
    maniTrainRun(hooks, Task, (int32_t)t->solo.chunkCount, t);
    maniTrainSoloUpdate(&t->solo, step, rate);
    float loss = 0.0f;
    for (uint32_t k = 0; k < t->solo.chunkCount; ++k)
    {
        loss += t->chunks[k].loss;
    }
    return loss;
}

static float Sign(float x)
{
    return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f);
}

// State value j of frame.
static float Truth(const maniTrainStepper* t, uint32_t frame, uint32_t j)
{
    uint32_t f = t->data->featureCount;
    return j < f ? t->data->features[(size_t)frame * f + j]
                 : t->latents[(size_t)frame * t->latentCount + (j - f)];
}

// The window's steps forward from the windows' first frames.
static void Forward(const maniTrainStepper* t, maniTrainStepperChunk* c)
{
    const maniTrainSolo* solo = &t->solo;
    uint32_t n = solo->net.sizes[0];
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        for (uint32_t j = 0; j < n; ++j)
        {
            c->states[(size_t)s * n + j] = Truth(t, c->windows[s], j);
        }
    }
    uint64_t out = maniTrainLayerAt(&solo->net, solo->net.layerCount, MANI_TRAIN_CHUNK);
    for (uint32_t k = 1; k < t->window; ++k)
    {
        float* values = c->values + (size_t)(k - 1) * MANI_TRAIN_CHUNK * solo->net.valueCount;
        const float* from = c->states + (size_t)(k - 1) * MANI_TRAIN_CHUNK * n;
        float* to = c->states + (size_t)k * MANI_TRAIN_CHUNK * n;
        for (size_t i = 0; i < (size_t)MANI_TRAIN_CHUNK * n; ++i)
        {
            values[i] = (from[i] - solo->inputOffsets[i % n]) / solo->inputScales[i % n];
        }
        maniTrainForward(&solo->net, solo->parameters, values, MANI_TRAIN_CHUNK);
        for (size_t i = 0; i < (size_t)MANI_TRAIN_CHUNK * n; ++i)
        {
            float y = values[out + i] * solo->outputScales[i % n] + solo->outputOffsets[i % n];
            to[i] = from[i] + t->seconds * y;
        }
    }
}

// The L1 terms of state value j over the chunk's windows, their
// gradient into c->stateGradient.
static float ValueLoss(const maniTrainStepper* t, maniTrainStepperChunk* c, uint32_t j)
{
    uint32_t n = t->solo.net.sizes[0];
    bool feature = j < t->data->featureCount;
    float batch = (float)(t->solo.chunkCount * MANI_TRAIN_CHUNK);
    float width = (float)(feature ? t->data->featureCount : t->latentCount);
    float w = (feature ? 2.0f : 7.5f) / (batch * (float)t->window * width);
    float v = (feature ? 0.2f : 0.5f) / (batch * (float)(t->window - 1) * width);
    float loss = 0.0f;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        float before = 0.0f;
        for (uint32_t k = 0; k < t->window; ++k)
        {
            size_t at = ((size_t)k * MANI_TRAIN_CHUNK + s) * n + j;
            float truth = Truth(t, c->windows[s] + k, j);
            float diff = c->states[at] - truth;
            loss += w * fabsf(diff);
            c->stateGradient[at] = w * Sign(diff);
            if (k > 0)
            {
                size_t last = at - (size_t)MANI_TRAIN_CHUNK * n;
                float rate = ((c->states[at] - c->states[last]) - (truth - before)) / t->seconds;
                float g = v * Sign(rate) / t->seconds;
                loss += v * fabsf(rate);
                c->stateGradient[at] += g;
                c->stateGradient[last] -= g;
            }
            before = truth;
        }
    }
    return loss;
}

void maniTrainStepperRun(const maniTrainStepper* t, maniTrainStepperChunk* c)
{
    const maniTrainSolo* solo = &t->solo;
    uint32_t n = solo->net.sizes[0];
    Forward(t, c);
    c->loss = 0.0f;
    for (uint32_t j = 0; j < n; ++j)
    {
        c->loss += ValueLoss(t, c, j);
    }
    float* gradients = solo->gradients + (size_t)(c - t->chunks) * (size_t)solo->net.parameterCount;
    for (uint32_t k = t->window - 1; k > 0; --k)
    {
        float* after = c->stateGradient + (size_t)k * MANI_TRAIN_CHUNK * n;
        float* before = after - (size_t)MANI_TRAIN_CHUNK * n;
        for (size_t i = 0; i < (size_t)MANI_TRAIN_CHUNK * n; ++i)
        {
            c->outputGradient[i] = after[i] * t->seconds * solo->outputScales[i % n];
        }
        const float* values = c->values + (size_t)(k - 1) * MANI_TRAIN_CHUNK * solo->net.valueCount;
        maniTrainBackward(&solo->net, solo->transposed, values, c->outputGradient, MANI_TRAIN_CHUNK,
                          gradients, c->inputGradient, c->scratch);
        for (size_t i = 0; i < (size_t)MANI_TRAIN_CHUNK * n; ++i)
        {
            before[i] = before[i] + after[i] + c->inputGradient[i] / solo->inputScales[i % n];
        }
    }
}
