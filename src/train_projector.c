// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A Projector's training (mani-0025); see train_projector.h.

#include "train_projector.h"

#include "allocator.h"
#include "match_search.h"
#include "train_data.h"
#include "train_net.h"
#include "train_random.h"
#include "train_solo.h"
#include "train_tasks.h"

#include "maul-anim/base.h"
#include "maul-anim/match.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define MANI_TRAIN_PROJECTOR_ARRAYS 4u

static void ChunkList(const maniTrainProjector* t, maniTrainProjectorChunk* c,
                      float** list[MANI_TRAIN_PROJECTOR_ARRAYS],
                      size_t sizes[MANI_TRAIN_PROJECTOR_ARRAYS])
{
    size_t chunk = MANI_TRAIN_CHUNK;
    const maniTrainNet* n = &t->solo.net;
    float** fields[MANI_TRAIN_PROJECTOR_ARRAYS] = {&c->values, &c->queries, &c->outputGradient,
                                                   &c->scratch};
    const size_t counts[MANI_TRAIN_PROJECTOR_ARRAYS] = {
        chunk * (size_t)n->valueCount, chunk * n->sizes[0], chunk * n->sizes[n->layerCount],
        2 * chunk * (size_t)n->widest};
    for (uint32_t k = 0; k < MANI_TRAIN_PROJECTOR_ARRAYS; ++k)
    {
        list[k] = fields[k];
        sizes[k] = counts[k] + 1;
    }
}

void maniTrainProjectorRelease(const maniAllocator* allocator, maniTrainProjector* t)
{
    for (uint32_t k = 0; t->chunks != NULL && k < t->solo.chunkCount; ++k)
    {
        float** arrays[MANI_TRAIN_PROJECTOR_ARRAYS];
        size_t sizes[MANI_TRAIN_PROJECTOR_ARRAYS];
        ChunkList(t, &t->chunks[k], arrays, sizes);
        for (uint32_t i = 0; i < MANI_TRAIN_PROJECTOR_ARRAYS; ++i)
        {
            maniRelease(allocator, *arrays[i], sizes[i], sizeof(float), 8);
        }
    }
    maniRelease(allocator, t->chunks, t->solo.chunkCount, sizeof(maniTrainProjectorChunk), 8);
    if (t->data != NULL)
    {
        maniRelease(allocator, t->noise, (size_t)t->data->featureCount + 1, sizeof(float), 8);
    }
    maniTrainSoloRelease(allocator, &t->solo);
    *t = (maniTrainProjector){0};
}

// Whether a frame passes the tags: an entry with the tags, frames, and
// jumps into it allowed.
static bool AnyFrame(const maniMotionDatabase* database, uint32_t required, uint32_t excluded)
{
    for (uint32_t e = 0; e < database->entryCount; ++e)
    {
        maniMatchEntry entry;
        if (maniMotionDatabaseEntry(database, e, &entry) == mani_success &&
            (entry.tags & required) == required && (entry.tags & excluded) == 0 &&
            (entry.flags & MANI_MATCH_NO_JUMP) == 0 && entry.frameCount > 0)
        {
            return true;
        }
    }
    return false;
}

static void Normalize(maniTrainProjector* t, const maniTrainData* d)
{
    uint32_t f = d->featureCount;
    uint32_t l = t->latentCount;
    maniTrainSolo* s = &t->solo;
    maniTrainMeasure(d->features, f, d->frameCount, 0, f, s->outputOffsets, t->noise, false);
    for (uint32_t j = 0; j < f; ++j)
    {
        s->outputScales[j] = t->noise[j];
        t->noise[j] += 1.0f;
    }
    maniTrainMeasure(t->latents, l, d->frameCount, 0, l, s->outputOffsets + f, s->outputScales + f,
                     false);
    maniTrainMeasure(d->features, f, d->frameCount, 0, f, s->inputOffsets, s->inputScales, true);
}

static bool Chunks(const maniAllocator* allocator, maniTrainProjector* t)
{
    t->chunks = maniAllocate(allocator, t->solo.chunkCount, sizeof(maniTrainProjectorChunk), 8);
    if (t->chunks == NULL)
    {
        return false;
    }
    memset(t->chunks, 0, sizeof(maniTrainProjectorChunk) * t->solo.chunkCount);
    bool ok = true;
    for (uint32_t k = 0; k < t->solo.chunkCount; ++k)
    {
        float** arrays[MANI_TRAIN_PROJECTOR_ARRAYS];
        size_t sizes[MANI_TRAIN_PROJECTOR_ARRAYS];
        ChunkList(t, &t->chunks[k], arrays, sizes);
        for (uint32_t i = 0; i < MANI_TRAIN_PROJECTOR_ARRAYS; ++i)
        {
            *arrays[i] = maniAllocate(allocator, sizes[i], sizeof(float), 8);
            ok = ok && *arrays[i] != NULL;
        }
    }
    return ok;
}

bool maniTrainProjectorInit(const maniAllocator* allocator, const maniMotionDatabase* database,
                            const maniTrainData* data, const float* latents,
                            const maniTrainSoloShape* shape, uint32_t requiredTags,
                            uint32_t excludedTags, maniTrainRandom* r, maniTrainProjector* t)
{
    *t = (maniTrainProjector){.latentCount = shape->latentCount,
                              .requiredTags = requiredTags,
                              .excludedTags = excludedTags,
                              .database = database,
                              .latents = latents};
    uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1];
    uint32_t f = data->featureCount;
    if (!AnyFrame(database, requiredTags, excludedTags) ||
        !maniTrainSoloSizes(shape, f, f + shape->latentCount, sizes))
    {
        return false;
    }
    t->data = data;
    t->noise = maniAllocate(allocator, (size_t)f + 1, sizeof(float), 8);
    bool ok = t->noise != NULL &&
              maniTrainSoloInit(allocator, sizes, shape->hiddenLayers + 1,
                                shape->batch / MANI_TRAIN_CHUNK, r, &t->solo) &&
              Chunks(allocator, t);
    if (!ok)
    {
        maniTrainProjectorRelease(allocator, t);
        return false;
    }
    Normalize(t, data);
    return true;
}

static void Task(int32_t start, int32_t end, void* context)
{
    maniTrainProjector* t = context;
    for (int32_t k = start; k < end; ++k)
    {
        maniTrainProjectorRun(t, &t->chunks[k]);
    }
}

float maniTrainProjectorStep(maniTrainProjector* t, maniTrainRandom* r, const maniTaskHooks* hooks,
                             uint64_t step, float rate)
{
    uint32_t f = t->data->featureCount;
    for (uint32_t k = 0; k < t->solo.chunkCount; ++k)
    {
        for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
        {
            uint32_t frame = maniTrainBelow(r, t->data->frameCount);
            float spread = maniTrainUniform(r);
            float* q = t->chunks[k].queries + (size_t)s * f;
            for (uint32_t j = 0; j < f; ++j)
            {
                float gaussian = maniTrainGaussian(r);
                q[j] = t->data->features[(size_t)frame * f + j] + t->noise[j] * spread * gaussian;
            }
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

// The distance from a query to f features, in feature order.
static float Distance(const float* query, const float* features, uint32_t f)
{
    float sum = 0.0f;
    for (uint32_t j = 0; j < f; ++j)
    {
        float d = query[j] - features[j];
        sum += d * d;
    }
    return sqrtf(sum);
}

// Sample s's loss; its gradient by the normalized outputs into
// c->outputGradient. y holds its outputs denormalized.
static float SampleLoss(const maniTrainProjector* t, maniTrainProjectorChunk* c, uint32_t s,
                        const float* y)
{
    const maniTrainSolo* solo = &t->solo;
    uint32_t f = t->data->featureCount;
    uint32_t l = t->latentCount;
    float batch = (float)(solo->chunkCount * MANI_TRAIN_CHUNK);
    const float* q = c->queries + (size_t)s * f;
    const float* x = t->data->features + (size_t)c->targets[s] * f;
    const float* z = t->latents + (size_t)c->targets[s] * l;
    float* g = c->outputGradient + (size_t)s * (f + l);
    float wx = 1.0f / (batch * (float)f);
    float wz = 5.0f / (batch * (float)l);
    float wd = 0.3f / batch;
    float apart = Distance(q, y, f);
    float gap = apart - Distance(q, x, f);
    float loss = 0.0f;
    for (uint32_t j = 0; j < f + l; ++j)
    {
        float diff = y[j] - (j < f ? x[j] : z[j - f]);
        float w = j < f ? wx : wz;
        loss += w * fabsf(diff);
        g[j] = w * Sign(diff);
        if (j < f && apart > 0.0f)
        {
            g[j] += wd * Sign(gap) * ((y[j] - q[j]) / apart);
        }
        g[j] *= solo->outputScales[j];
    }
    return loss + wd * fabsf(gap);
}

void maniTrainProjectorRun(const maniTrainProjector* t, maniTrainProjectorChunk* c)
{
    const maniTrainSolo* solo = &t->solo;
    uint32_t f = t->data->featureCount;
    uint32_t out = f + t->latentCount;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        maniMatchQuery query = {.features = c->queries + (size_t)s * f,
                                .current = MANI_MATCH_NO_FRAME,
                                .requiredTags = t->requiredTags,
                                .excludedTags = t->excludedTags};
        maniMatchResult result = {MANI_MATCH_NO_FRAME, 0.0f};
        (void)maniMatchSearch(t->database, &query, true, &result);
        c->targets[s] = result.frame;
        for (uint32_t j = 0; j < f; ++j)
        {
            c->values[(size_t)s * f + j] =
                (c->queries[(size_t)s * f + j] - solo->inputOffsets[j]) / solo->inputScales[j];
        }
    }
    maniTrainForward(&solo->net, solo->parameters, c->values, MANI_TRAIN_CHUNK);
    // The outputs are denormalized in place: the backward pass reads only
    // the layers' inputs.
    float* yn = c->values + maniTrainLayerAt(&solo->net, solo->net.layerCount, MANI_TRAIN_CHUNK);
    c->loss = 0.0f;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        float* y = yn + (size_t)s * out;
        for (uint32_t j = 0; j < out; ++j)
        {
            y[j] = y[j] * solo->outputScales[j] + solo->outputOffsets[j];
        }
        c->loss += SampleLoss(t, c, s, y);
    }
    float* gradients = solo->gradients + (size_t)(c - t->chunks) * (size_t)solo->net.parameterCount;
    maniTrainBackward(&solo->net, solo->transposed, c->values, c->outputGradient, MANI_TRAIN_CHUNK,
                      gradients, NULL, c->scratch);
}
