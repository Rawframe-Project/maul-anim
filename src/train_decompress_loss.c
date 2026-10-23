// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One chunk of the first stage; train_decompress.h gives
// the loss.

#include "train_decompress_loss.h"

#include "train_data.h"
#include "train_kinematics.h"
#include "train_net.h"
#include "train_stage.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static float Sign(float x)
{
    return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f);
}

// The weight of each output's L1 term over the batch's frames: positions,
// columns, scales, the root's velocities, the extras, as torch.mean of
// each term over its own values.
static float OutputWeight(const maniTrainData* d, uint32_t j, float frames)
{
    uint32_t bones = d->boneCount;
    if (j < 9 * bones)
    {
        return j % 9 < 3 ? 75.0f / (frames * 3.0f * (float)bones)
                         : 10.0f / (frames * 6.0f * (float)bones);
    }
    j -= 9 * bones;
    if (j < 3 * d->scaledBoneCount)
    {
        return 10.0f / (frames * 3.0f * (float)d->scaledBoneCount);
    }
    j -= 3 * d->scaledBoneCount;
    return j < 6 ? 2.0f / (frames * 3.0f)
                 : 2.0f / (frames * (float)(d->floatTrackCount + d->eventCount));
}

// The outputs' L1 terms: frames' values, then the local velocities across
// each pair. Writes the outputs denormalized and the gradient by them.
static float OutputLoss(const maniTrainDecompress* t, const uint32_t* frames, const float* yn,
                        maniTrainDecompressChunk* c, uint32_t batchPairs)
{
    const maniTrainData* d = t->data;
    uint32_t o = d->outputCount;
    float count = 2.0f * (float)batchPairs;
    float y[MANI_TRAIN_CHUNK];
    float loss = 0.0f;
    float* gradient = c->outputGradient;
    for (uint32_t j = 0; j < o; ++j)
    {
        float w = OutputWeight(d, j, count);
        for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
        {
            y[s] = yn[(size_t)s * o + j] * t->outputScales[j] + t->outputOffsets[j];
            c->outputs[(size_t)s * o + j] = y[s];
            float diff = y[s] - d->outputs[(size_t)frames[s] * o + j];
            loss += w * fabsf(diff);
            gradient[(size_t)s * o + j] = w * Sign(diff);
        }
        if (j >= 9 * d->boneCount)
        {
            continue;
        }
        float v = j % 9 < 3 ? 10.0f / ((float)batchPairs * 3.0f * (float)d->boneCount)
                            : 1.75f / ((float)batchPairs * 6.0f * (float)d->boneCount);
        for (uint32_t p = 0; p < MANI_TRAIN_PAIRS; ++p)
        {
            const float* target = d->outputs + j;
            float want = target[(size_t)frames[2 * p + 1] * o] - target[(size_t)frames[2 * p] * o];
            float diff = ((y[2 * p + 1] - y[2 * p]) - want) / d->frameSeconds;
            loss += v * fabsf(diff);
            gradient[(size_t)(2 * p + 1) * o + j] += v * Sign(diff) / d->frameSeconds;
            gradient[(size_t)(2 * p) * o + j] -= v * Sign(diff) / d->frameSeconds;
        }
    }
    return loss;
}

// The weight of a character term over count values of each bone: as
// torch.mean over them.
static float Mean(float weight, float count, uint32_t width, uint32_t bones)
{
    return weight / (count * (float)width * (float)bones);
}

// One character value's terms over the chunk (b's value i): its L1 at
// each frame and its velocity's across each pair, into its gradient.
static float CharacterTerm(const maniTrainData* d, const uint32_t* frames,
                           maniTrainDecompressChunk* c, uint32_t b, uint32_t i, uint32_t batchPairs)
{
    size_t each = (size_t)d->boneCount * MANI_TRAIN_CHARACTER_FLOATS;
    size_t k = (size_t)b * MANI_TRAIN_CHARACTER_FLOATS + i;
    bool position = i < 3;
    uint32_t group = position ? maniTrainGroupCharacterPosition : maniTrainGroupCharacterColumns;
    uint32_t at = d->groupStarts[group] + (position ? 3 * b + i : 6 * b + i - 3);
    float w =
        Mean(position ? 15.0f : 5.0f, 2.0f * (float)batchPairs, position ? 3 : 6, d->boneCount);
    float v = Mean(position ? 2.0f : 0.75f, (float)batchPairs, position ? 3 : 6, d->boneCount);
    float seconds = d->frameSeconds;
    float q[MANI_TRAIN_CHUNK];
    float target[MANI_TRAIN_CHUNK];
    float loss = 0.0f;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        q[s] = c->character[(size_t)s * each + k];
        target[s] = d->inputs[(size_t)frames[s] * d->inputCount + at];
        loss += w * fabsf(q[s] - target[s]);
        c->characterGradient[(size_t)s * each + k] = w * Sign(q[s] - target[s]);
    }
    for (uint32_t p = 0; p < MANI_TRAIN_PAIRS; ++p)
    {
        float diff = ((q[2 * p + 1] - q[2 * p]) - (target[2 * p + 1] - target[2 * p])) / seconds;
        loss += v * fabsf(diff);
        c->characterGradient[(size_t)(2 * p + 1) * each + k] += v * Sign(diff) / seconds;
        c->characterGradient[(size_t)(2 * p) * each + k] -= v * Sign(diff) / seconds;
    }
    return loss;
}

// The character-space terms of each frame (translations 15, columns 5)
// and their velocities across each pair (2, 0.75), into the gradient by
// the outputs through the character pass.
static float CharacterLoss(const maniTrainDecompress* t, const uint32_t* frames,
                           maniTrainDecompressChunk* c, uint32_t batchPairs)
{
    const maniTrainData* d = t->data;
    uint32_t bones = d->boneCount;
    size_t each = (size_t)bones * MANI_TRAIN_CHARACTER_FLOATS;
    size_t work = (size_t)bones * MANI_TRAIN_KINEMATIC_FLOATS;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        maniTrainKinematics(d, c->outputs + (size_t)s * d->outputCount, c->kinematics + s * work,
                            c->character + s * each);
    }
    float loss = 0.0f;
    for (uint32_t b = 0; b < bones; ++b)
    {
        for (uint32_t i = 0; i < MANI_TRAIN_CHARACTER_FLOATS; ++i)
        {
            loss += CharacterTerm(d, frames, c, b, i, batchPairs);
        }
    }
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        maniTrainKinematicsBackward(d, c->kinematics + s * work, c->characterGradient + s * each,
                                    c->outputGradient + (size_t)s * d->outputCount,
                                    c->kinematicScratch);
    }
    return loss;
}

// The latent values' terms, added to their gradient: L1 and L2 over the
// frames, the change a second across each pair.
static float LatentLoss(const maniTrainDecompress* t, const float* z, float* gradient,
                        uint32_t batchPairs)
{
    uint32_t latent = t->compressor.sizes[t->compressor.layerCount];
    float frames = 2.0f * (float)batchPairs * (float)latent;
    float pairs = (float)batchPairs * (float)latent;
    float seconds = t->data->frameSeconds;
    float loss = 0.0f;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        for (uint32_t k = 0; k < latent; ++k)
        {
            float x = z[(size_t)s * latent + k];
            loss += 0.1f / frames * fabsf(x) + 0.1f / frames * x * x;
            gradient[(size_t)s * latent + k] += 0.1f / frames * Sign(x) + 0.2f / frames * x;
        }
    }
    for (uint32_t p = 0; p < MANI_TRAIN_PAIRS; ++p)
    {
        for (uint32_t k = 0; k < latent; ++k)
        {
            float diff =
                (z[(size_t)(2 * p + 1) * latent + k] - z[(size_t)(2 * p) * latent + k]) / seconds;
            float g = 0.01f / pairs * Sign(diff) / seconds;
            loss += 0.01f / pairs * fabsf(diff);
            gradient[(size_t)(2 * p + 1) * latent + k] += g;
            gradient[(size_t)(2 * p) * latent + k] -= g;
        }
    }
    return loss;
}

// The Compressor's normalized inputs and the Decompressor's inputs.
static void Inputs(const maniTrainDecompress* t, const uint32_t* frames,
                   maniTrainDecompressChunk* c)
{
    const maniTrainData* d = t->data;
    uint32_t in = d->inputCount;
    uint32_t n = d->featureCount;
    uint32_t latent = t->compressor.sizes[t->compressor.layerCount];
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        const float* row = d->inputs + (size_t)frames[s] * in;
        float* x = c->compressorValues + (size_t)s * in;
        for (uint32_t i = 0; i < in; ++i)
        {
            x[i] = (row[i] - t->inputOffsets[i]) / t->inputScales[i];
        }
    }
    maniTrainForward(&t->compressor, t->parameters, c->compressorValues, MANI_TRAIN_CHUNK);
    const float* z = c->compressorValues +
                     maniTrainLayerAt(&t->compressor, t->compressor.layerCount, MANI_TRAIN_CHUNK);
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        float* x = c->decompressorValues + (size_t)s * (n + latent);
        memcpy(x, d->features + (size_t)frames[s] * n, sizeof(float) * n);
        memcpy(x + n, z + (size_t)s * latent, sizeof(float) * latent);
    }
}

void maniTrainDecompressRun(const maniTrainDecompress* t, maniTrainDecompressChunk* c,
                            uint32_t batchPairs)
{
    const maniTrainData* d = t->data;
    uint32_t frames[MANI_TRAIN_CHUNK];
    for (uint32_t p = 0; p < MANI_TRAIN_PAIRS; ++p)
    {
        frames[2 * p] = d->pairs[c->pairs[p]];
        frames[2 * p + 1] = frames[2 * p] + 1;
    }
    Inputs(t, frames, c);
    uint64_t own = t->compressor.parameterCount;
    maniTrainForward(&t->decompressor, t->parameters + own, c->decompressorValues,
                     MANI_TRAIN_CHUNK);
    const float* yn =
        c->decompressorValues +
        maniTrainLayerAt(&t->decompressor, t->decompressor.layerCount, MANI_TRAIN_CHUNK);
    c->loss = OutputLoss(t, frames, yn, c, batchPairs);
    c->loss += CharacterLoss(t, frames, c, batchPairs);
    uint32_t o = d->outputCount;
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        for (uint32_t j = 0; j < o; ++j)
        {
            c->outputGradient[(size_t)s * o + j] *= t->outputScales[j];
        }
    }
    memset(c->gradients, 0, sizeof(float) * (size_t)t->count);
    maniTrainBackward(&t->decompressor, t->transposed + own, c->decompressorValues,
                      c->outputGradient, MANI_TRAIN_CHUNK, c->gradients + own, c->inputGradient,
                      c->scratch);
    uint32_t n = d->featureCount;
    uint32_t latent = t->compressor.sizes[t->compressor.layerCount];
    for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
    {
        memcpy(c->latentGradient + (size_t)s * latent,
               c->inputGradient + (size_t)s * (n + latent) + n, sizeof(float) * latent);
    }
    const float* z = c->compressorValues +
                     maniTrainLayerAt(&t->compressor, t->compressor.layerCount, MANI_TRAIN_CHUNK);
    c->loss += LatentLoss(t, z, c->latentGradient, batchPairs);
    maniTrainBackward(&t->compressor, t->transposed, c->compressorValues, c->latentGradient,
                      MANI_TRAIN_CHUNK, c->gradients, NULL, c->scratch);
}
