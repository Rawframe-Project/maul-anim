// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The first stage of a learned model's training (mani-0025); see
// train_decompress.h.

#include "train_decompress.h"

#include "allocator.h"
#include "train_adam.h"
#include "train_data.h"
#include "train_decompress_loss.h"
#include "train_kinematics.h"
#include "train_net.h"
#include "train_random.h"
#include "train_tasks.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static bool Networks(const maniTrainData* d, const maniTrainDecompressShape* s,
                     maniTrainDecompress* t)
{
    uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1];
    if (s->compressorLayers >= MANI_LEARNED_MAX_LAYERS ||
        s->decompressorLayers >= MANI_LEARNED_MAX_LAYERS || s->batchPairs < MANI_TRAIN_PAIRS ||
        s->batchPairs % MANI_TRAIN_PAIRS != 0 || d->pairCount == 0)
    {
        return false;
    }
    sizes[0] = d->inputCount;
    for (uint32_t l = 1; l <= s->compressorLayers; ++l)
    {
        sizes[l] = s->hiddenWidth;
    }
    sizes[s->compressorLayers + 1] = s->latentCount;
    if (!maniTrainNetInit(&t->compressor, sizes, s->compressorLayers + 1, maniTrainElu))
    {
        return false;
    }
    sizes[0] = d->featureCount + s->latentCount;
    for (uint32_t l = 1; l <= s->decompressorLayers; ++l)
    {
        sizes[l] = s->hiddenWidth;
    }
    sizes[s->decompressorLayers + 1] = d->outputCount;
    return maniTrainNetInit(&t->decompressor, sizes, s->decompressorLayers + 1, maniTrainRelu);
}

// The arrays of one chunk, or of every chunk with all set.
static bool ChunkArrays(const maniAllocator* a, maniTrainDecompress* t, bool release)
{
    bool ok = true;
    size_t chunk = MANI_TRAIN_CHUNK;
    size_t bones = t->data->boneCount;
    size_t sizes[11] = {chunk * (size_t)t->compressor.valueCount,
                        chunk * (size_t)t->decompressor.valueCount,
                        chunk * t->decompressor.sizes[t->decompressor.layerCount],
                        chunk * t->decompressor.sizes[0],
                        chunk * t->compressor.sizes[t->compressor.layerCount],
                        2 * chunk * t->widest,
                        chunk * t->decompressor.sizes[t->decompressor.layerCount],
                        chunk * bones * MANI_TRAIN_KINEMATIC_FLOATS,
                        chunk * bones * MANI_TRAIN_CHARACTER_FLOATS,
                        chunk * bones * MANI_TRAIN_CHARACTER_FLOATS,
                        12 * bones};
    for (uint32_t k = 0; k < t->chunkCount; ++k)
    {
        maniTrainDecompressChunk* c = &t->chunks[k];
        float** arrays[11] = {&c->compressorValues,  &c->decompressorValues, &c->outputGradient,
                              &c->inputGradient,     &c->latentGradient,     &c->scratch,
                              &c->outputs,           &c->kinematics,         &c->character,
                              &c->characterGradient, &c->kinematicScratch};
        for (uint32_t i = 0; i < 11; ++i)
        {
            if (release)
            {
                maniRelease(a, *arrays[i], sizes[i] + 1, sizeof(float), 8);
                continue;
            }
            *arrays[i] = maniAllocate(a, sizes[i] + 1, sizeof(float), 8);
            ok = ok && *arrays[i] != NULL;
        }
    }
    return ok;
}

// The stage's arrays (parameters, moments, the transposed weights, the
// chunks' gradient sum, the normalization) and their sizes.
#define MANI_TRAIN_ARRAYS 10u

static void ArrayList(maniTrainDecompress* t, float** list[MANI_TRAIN_ARRAYS],
                      size_t sizes[MANI_TRAIN_ARRAYS])
{
    const maniTrainData* d = t->data;
    float** fields[MANI_TRAIN_ARRAYS] = {&t->parameters,   &t->m,           &t->v,
                                         &t->largest,      &t->transposed,  &t->sum,
                                         &t->inputOffsets, &t->inputScales, &t->outputOffsets,
                                         &t->outputScales};
    const size_t counts[MANI_TRAIN_ARRAYS] = {
        t->count, t->count,      t->count,      t->count,       t->count,
        t->count, d->inputCount, d->inputCount, d->outputCount, d->outputCount};
    for (uint32_t k = 0; k < MANI_TRAIN_ARRAYS; ++k)
    {
        list[k] = fields[k];
        sizes[k] = counts[k] + 1;
    }
}

void maniTrainDecompressRelease(const maniAllocator* allocator, maniTrainDecompress* t)
{
    if (t->chunks != NULL)
    {
        (void)ChunkArrays(allocator, t, true);
        float* block = t->chunks[0].gradients;
        maniRelease(allocator, block, (size_t)t->chunkCount * t->count + 1, sizeof(float), 8);
        maniRelease(allocator, t->chunks, t->chunkCount, sizeof(maniTrainDecompressChunk), 8);
    }
    float** arrays[MANI_TRAIN_ARRAYS];
    size_t sizes[MANI_TRAIN_ARRAYS];
    if (t->data != NULL)
    {
        ArrayList(t, arrays, sizes);
        for (uint32_t k = 0; k < MANI_TRAIN_ARRAYS; ++k)
        {
            maniRelease(allocator, *arrays[k], sizes[k], sizeof(float), 8);
        }
    }
    *t = (maniTrainDecompress){0};
}

static void Normalize(maniTrainDecompress* t, const maniTrainData* d)
{
    maniTrainMeasure(d->outputs, d->outputCount, d->frameCount, 0, d->outputCount, t->outputOffsets,
                     t->outputScales, false);
    for (uint32_t g = 0; g < maniTrainGroupCount; ++g)
    {
        uint32_t first = d->groupStarts[g];
        maniTrainMeasure(d->inputs, d->inputCount, d->frameCount, first,
                         d->groupStarts[g + 1] - first, t->inputOffsets, t->inputScales, true);
    }
}

bool maniTrainDecompressInit(const maniAllocator* allocator, const maniTrainData* data,
                             const maniTrainDecompressShape* shape, maniTrainRandom* r,
                             maniTrainDecompress* t)
{
    *t = (maniTrainDecompress){.data = data};
    if (!Networks(data, shape, t))
    {
        return false;
    }
    t->count = t->compressor.parameterCount + t->decompressor.parameterCount;
    t->widest = t->compressor.widest > t->decompressor.widest ? t->compressor.widest
                                                              : t->decompressor.widest;
    t->chunkCount = shape->batchPairs / MANI_TRAIN_PAIRS;
    float** arrays[MANI_TRAIN_ARRAYS];
    size_t sizes[MANI_TRAIN_ARRAYS];
    ArrayList(t, arrays, sizes);
    bool ok = true;
    for (uint32_t k = 0; k < MANI_TRAIN_ARRAYS; ++k)
    {
        *arrays[k] = maniAllocate(allocator, sizes[k], sizeof(float), 8);
        ok = ok && *arrays[k] != NULL;
    }
    t->chunks = maniAllocate(allocator, t->chunkCount, sizeof(maniTrainDecompressChunk), 8);
    float* block = maniAllocate(allocator, (size_t)t->chunkCount * t->count + 1, sizeof(float), 8);
    if (t->chunks != NULL)
    {
        memset(t->chunks, 0, sizeof(maniTrainDecompressChunk) * t->chunkCount);
        for (uint32_t k = 0; k < t->chunkCount; ++k)
        {
            t->chunks[k].gradients = block != NULL ? block + (size_t)k * t->count : NULL;
        }
    }
    ok = ok && t->chunks != NULL && block != NULL && ChunkArrays(allocator, t, false);
    if (!ok)
    {
        maniRelease(allocator, t->chunks == NULL ? block : NULL,
                    (size_t)t->chunkCount * t->count + 1, sizeof(float), 8);
        maniTrainDecompressRelease(allocator, t);
        return false;
    }
    memset(t->m, 0, sizeof(float) * t->count);
    memset(t->v, 0, sizeof(float) * t->count);
    memset(t->largest, 0, sizeof(float) * t->count);
    maniTrainNetStart(&t->compressor, t->parameters, r);
    maniTrainNetStart(&t->decompressor, t->parameters + t->compressor.parameterCount, r);
    Normalize(t, data);
    return true;
}

static void Task(int32_t start, int32_t end, void* context)
{
    maniTrainDecompress* t = context;
    for (int32_t k = start; k < end; ++k)
    {
        maniTrainDecompressRun(t, &t->chunks[k], t->chunkCount * MANI_TRAIN_PAIRS);
    }
}

float maniTrainDecompressStep(maniTrainDecompress* t, const maniTrainData* data, maniTrainRandom* r,
                              const maniTaskHooks* hooks, uint64_t step, float rate)
{
    t->data = data;
    for (uint32_t k = 0; k < t->chunkCount; ++k)
    {
        for (uint32_t p = 0; p < MANI_TRAIN_PAIRS; ++p)
        {
            t->chunks[k].pairs[p] = maniTrainBelow(r, data->pairCount);
        }
    }
    uint64_t own = t->compressor.parameterCount;
    maniTrainTranspose(&t->compressor, t->parameters, t->transposed);
    maniTrainTranspose(&t->decompressor, t->parameters + own, t->transposed + own);
    maniTrainRun(hooks, Task, (int32_t)t->chunkCount, t);
    maniTrainSum(t->chunks[0].gradients, t->chunkCount, t->count, t->sum);
    maniTrainAdam adam = maniTrainAdamDefaults();
    maniTrainAdamStep(&adam, step, rate, t->parameters, t->sum, t->m, t->v, t->largest, t->count);
    float loss = 0.0f;
    for (uint32_t k = 0; k < t->chunkCount; ++k)
    {
        loss += t->chunks[k].loss;
    }
    return loss;
}

void maniTrainLatents(const maniTrainDecompress* t, const maniTrainData* data, float* latents)
{
    maniTrainDecompressChunk* c = &t->chunks[0];
    uint32_t in = data->inputCount;
    uint32_t latent = t->compressor.sizes[t->compressor.layerCount];
    for (uint32_t first = 0; first < data->frameCount; first += MANI_TRAIN_CHUNK)
    {
        uint32_t count = data->frameCount - first < MANI_TRAIN_CHUNK ? data->frameCount - first
                                                                     : MANI_TRAIN_CHUNK;
        for (uint32_t s = 0; s < count; ++s)
        {
            const float* row = data->inputs + (size_t)(first + s) * in;
            float* x = c->compressorValues + (size_t)s * in;
            for (uint32_t i = 0; i < in; ++i)
            {
                x[i] = (row[i] - t->inputOffsets[i]) / t->inputScales[i];
            }
        }
        maniTrainForward(&t->compressor, t->parameters, c->compressorValues, count);
        const float* z =
            c->compressorValues + maniTrainLayerAt(&t->compressor, t->compressor.layerCount, count);
        memcpy(latents + (size_t)first * latent, z, sizeof(float) * (size_t)count * latent);
    }
}
