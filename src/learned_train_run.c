// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's trainer (mani-0025): its stages begun, stepped and
// ended, and the model built from its networks.

#include "allocator.h"
#include "builder.h"
#include "learned_file.h"
#include "learned_train.h"
#include "train_adam.h"
#include "train_data.h"
#include "train_decompress.h"
#include "train_net.h"
#include "train_projector.h"
#include "train_random.h"
#include "train_solo.h"
#include "train_stage.h"
#include "train_stepper.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/learnedtrain.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// A kept network's floats: its parameters, then its input offsets and
// scales, then its output offsets and scales.
static size_t KeptFloats(const maniTrainNet* n)
{
    return (size_t)n->parameterCount + 2 * (size_t)n->sizes[0] +
           2 * (size_t)n->sizes[n->layerCount];
}

bool maniLearnedTrainerKeep(maniLearnedTrainer* t, uint32_t n, const maniTrainNet* net)
{
    maniTrainKept* k = &t->networks[n];
    float* block = maniAllocate(&t->allocator, KeptFloats(net) + 1, sizeof(float), 8);
    if (block == NULL)
    {
        return false;
    }
    k->net = *net;
    k->parameters = block;
    k->inputOffsets = block + net->parameterCount;
    k->inputScales = k->inputOffsets + net->sizes[0];
    k->outputOffsets = k->inputScales + net->sizes[0];
    k->outputScales = k->outputOffsets + net->sizes[net->layerCount];
    return true;
}

static void Unkeep(maniLearnedTrainer* t, uint32_t n)
{
    maniTrainKept* k = &t->networks[n];
    if (k->parameters != NULL)
    {
        maniRelease(&t->allocator, k->parameters, KeptFloats(&k->net) + 1, sizeof(float), 8);
    }
    *k = (maniTrainKept){0};
}

void maniLearnedTrainerForget(maniLearnedTrainer* t)
{
    for (uint32_t n = 0; t->networks != NULL && n < t->stageCount; ++n)
    {
        Unkeep(t, n);
    }
    maniRelease(&t->allocator, t->latents,
                (size_t)t->data.frameCount * t->source.options.latentCount + 1, sizeof(float), 8);
    t->latents = NULL;
}

void maniLearnedTrainerDrop(maniLearnedTrainer* t)
{
    if (!t->begun)
    {
        return;
    }
    if (t->stage == MANI_LEARNED_DECOMPRESSOR)
    {
        maniTrainDecompressRelease(&t->allocator, &t->decompress);
    }
    else if (t->stage == MANI_LEARNED_STEPPER)
    {
        maniTrainStepperRelease(&t->allocator, &t->stepper);
    }
    else
    {
        maniTrainProjectorRelease(&t->allocator, &t->projector);
    }
    t->begun = false;
}

uint64_t maniLearnedTrainerArrays(const maniLearnedTrainer* t, float* arrays[4])
{
    if (t->stage == MANI_LEARNED_DECOMPRESSOR)
    {
        const maniTrainDecompress* d = &t->decompress;
        arrays[0] = d->parameters;
        arrays[1] = d->m;
        arrays[2] = d->v;
        arrays[3] = d->largest;
        return d->count;
    }
    const maniTrainSolo* s =
        t->stage == MANI_LEARNED_STEPPER ? &t->stepper.solo : &t->projector.solo;
    arrays[0] = s->parameters;
    arrays[1] = s->m;
    arrays[2] = s->v;
    arrays[3] = s->largest;
    return s->net.parameterCount;
}

maniResult maniLearnedTrainerBegin(maniLearnedTrainer* t)
{
    const maniLearnedTrainerOptions* o = &t->source.options;
    maniTrainSeed(&t->random, o->seed + t->stage);
    bool ok = false;
    if (t->stage == MANI_LEARNED_DECOMPRESSOR)
    {
        const maniTrainDecompressShape shape = {o->latentCount,      o->hiddenWidth,
                                                o->compressorLayers, o->decompressorLayers,
                                                o->batchSize,        0};
        ok = maniTrainDecompressInit(&t->allocator, &t->data, &shape, &t->random, &t->decompress);
    }
    else if (t->stage == MANI_LEARNED_STEPPER)
    {
        const maniTrainSoloShape shape = {o->latentCount, o->hiddenWidth,   o->stepperLayers,
                                          o->batchSize,   o->stepperWindow, 0};
        ok = maniTrainStepperInit(&t->allocator, &t->data, t->latents, &shape, &t->random,
                                  &t->stepper);
    }
    else
    {
        const maniTrainSoloShape shape = {
            o->latentCount, o->hiddenWidth, o->projectorLayers, o->batchSize, 0, 0};
        const maniLearnedTagFilter* f = &t->source.projectors[t->stage - MANI_LEARNED_PROJECTORS];
        ok = maniTrainProjectorInit(&t->allocator, t->source.database, &t->data, t->latents, &shape,
                                    f->required, f->excluded, &t->random, &t->projector);
    }
    t->begun = ok;
    t->iteration = 0;
    t->loss = 0.0f;
    return ok ? mani_success : mani_errorCapacity;
}

// The Decompressor kept (its inputs as they are) and every frame's latent
// values from the Compressor.
static bool EndFirst(maniLearnedTrainer* t)
{
    const maniTrainDecompress* s = &t->decompress;
    size_t count = (size_t)t->data.frameCount * t->source.options.latentCount + 1;
    t->latents = maniAllocate(&t->allocator, count, sizeof(float), 8);
    if (t->latents == NULL ||
        !maniLearnedTrainerKeep(t, MANI_LEARNED_DECOMPRESSOR, &s->decompressor))
    {
        maniRelease(&t->allocator, t->latents, count, sizeof(float), 8);
        t->latents = NULL;
        return false;
    }
    maniTrainLatents(s, &t->data, t->latents);
    maniTrainKept* k = &t->networks[MANI_LEARNED_DECOMPRESSOR];
    memcpy(k->parameters, s->parameters + s->compressor.parameterCount,
           sizeof(float) * (size_t)k->net.parameterCount);
    for (uint32_t i = 0; i < k->net.sizes[0]; ++i)
    {
        k->inputOffsets[i] = 0.0f;
        k->inputScales[i] = 1.0f;
    }
    uint32_t out = k->net.sizes[k->net.layerCount];
    memcpy(k->outputOffsets, s->outputOffsets, sizeof(float) * out);
    memcpy(k->outputScales, s->outputScales, sizeof(float) * out);
    return true;
}

static bool EndSolo(maniLearnedTrainer* t, const maniTrainSolo* s)
{
    if (!maniLearnedTrainerKeep(t, t->stage, &s->net))
    {
        return false;
    }
    maniTrainKept* k = &t->networks[t->stage];
    uint32_t in = s->net.sizes[0];
    uint32_t out = s->net.sizes[s->net.layerCount];
    memcpy(k->parameters, s->parameters, sizeof(float) * (size_t)s->net.parameterCount);
    memcpy(k->inputOffsets, s->inputOffsets, sizeof(float) * in);
    memcpy(k->inputScales, s->inputScales, sizeof(float) * in);
    memcpy(k->outputOffsets, s->outputOffsets, sizeof(float) * out);
    memcpy(k->outputScales, s->outputScales, sizeof(float) * out);
    return true;
}

// The stage in training ended: its network kept, its state released.
static maniResult End(maniLearnedTrainer* t)
{
    const maniTrainSolo* solo =
        t->stage == MANI_LEARNED_STEPPER ? &t->stepper.solo : &t->projector.solo;
    bool ok = t->stage == MANI_LEARNED_DECOMPRESSOR ? EndFirst(t) : EndSolo(t, solo);
    if (!ok)
    {
        return mani_errorCapacity;
    }
    maniLearnedTrainerDrop(t);
    t->stage += 1;
    t->iteration = 0;
    t->loss = 0.0f;
    return mani_success;
}

// The learning rate before iteration step (from 1): the option times 0.99
// to the whole thousands of the iterations before it.
static float Rate(float rate, uint64_t step)
{
    double decay = 1.0;
    double x = 0.99;
    for (uint64_t k = (step - 1) / 1000; k > 0; k >>= 1)
    {
        decay *= (k & 1u) != 0 ? x : 1.0;
        x *= x;
    }
    return (float)((double)rate * decay);
}

static void Iterate(maniLearnedTrainer* t)
{
    uint64_t step = t->iteration + 1;
    float rate = Rate(t->source.options.learningRate, step);
    if (t->stage == MANI_LEARNED_DECOMPRESSOR)
    {
        t->loss =
            maniTrainDecompressStep(&t->decompress, &t->data, &t->random, &t->tasks, step, rate);
    }
    else if (t->stage == MANI_LEARNED_STEPPER)
    {
        t->loss = maniTrainStepperStep(&t->stepper, &t->random, &t->tasks, step, rate);
    }
    else
    {
        t->loss = maniTrainProjectorStep(&t->projector, &t->random, &t->tasks, step, rate);
    }
    t->iteration = step;
}

maniResult maniTrainLearned(maniLearnedTrainer* trainer, uint64_t iterations,
                            maniLearnedTrainerReport* reportOut)
{
    if (trainer != NULL && maniHostCalledBack(&trainer->host))
    {
        return mani_errorState;
    }
    if (trainer == NULL || reportOut == NULL)
    {
        return mani_errorInvalid;
    }
    maniLearnedTrainer* t = trainer;
    maniResult r = mani_success;
    while (r == mani_success && t->stage < t->stageCount)
    {
        if (t->begun && t->iteration == t->source.options.iterations)
        {
            r = End(t);
            continue;
        }
        if (iterations == 0)
        {
            break;
        }
        r = t->begun ? mani_success : maniLearnedTrainerBegin(t);
        if (r == mani_success)
        {
            Iterate(t);
            iterations -= 1;
        }
    }
    *reportOut = (maniLearnedTrainerReport){t->stage, t->stageCount, t->iteration, t->loss, 0};
    return r;
}

static void Network(const maniTrainKept* k, maniLearnedLayerSource* layers,
                    maniLearnedNetworkSource* out)
{
    for (uint32_t l = 0; l < k->net.layerCount; ++l)
    {
        const float* w = k->parameters + k->net.offsets[l];
        uint32_t in = k->net.sizes[l];
        uint32_t o = k->net.sizes[l + 1];
        layers[l] = (maniLearnedLayerSource){w, w + (size_t)in * o, in, o};
    }
    *out = (maniLearnedNetworkSource){layers,
                                      k->inputOffsets,
                                      k->inputScales,
                                      k->outputOffsets,
                                      k->outputScales,
                                      k->net.layerCount,
                                      0};
}

maniBuildResult maniBuildLearnedModelFromTrainer(maniBuilder* builder,
                                                 const maniLearnedTrainer* trainer,
                                                 maniLearnedWeights weightFormat)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    if (trainer == NULL || trainer->stage < trainer->stageCount)
    {
        maniBuilderMisuse(builder);
        return (maniBuildResult){mani_errorInvalid, maniBuildReasonNone, 0, 0};
    }
    const maniLearnedTrainer* t = trainer;
    uint32_t projectors = t->source.projectorCount;
    maniLearnedLayerSource layers[MANI_LEARNED_MAX_LAYERS * 2];
    maniLearnedNetworkSource decompressor;
    maniLearnedNetworkSource stepper;
    size_t count = (size_t)projectors * MANI_LEARNED_MAX_LAYERS;
    maniLearnedLayerSource* projectorLayers =
        maniAllocate(&t->allocator, count, sizeof(maniLearnedLayerSource), 8);
    maniLearnedProjectorSource* list =
        maniAllocate(&t->allocator, projectors, sizeof(maniLearnedProjectorSource), 8);
    maniBuildResult r = {mani_errorCapacity, maniBuildReasonNone, 0, 0};
    if (projectorLayers != NULL && list != NULL)
    {
        Network(&t->networks[MANI_LEARNED_DECOMPRESSOR], layers, &decompressor);
        Network(&t->networks[MANI_LEARNED_STEPPER], layers + MANI_LEARNED_MAX_LAYERS, &stepper);
        for (uint32_t p = 0; p < projectors; ++p)
        {
            const maniLearnedTagFilter* f = &t->source.projectors[p];
            Network(&t->networks[MANI_LEARNED_PROJECTORS + p],
                    projectorLayers + (size_t)p * MANI_LEARNED_MAX_LAYERS, &list[p].network);
            list[p].requiredTags = f->required;
            list[p].excludedTags = f->excluded;
        }
        const maniLearnedModelSource model = {t->source.skeleton,
                                              t->source.database,
                                              &decompressor,
                                              &stepper,
                                              list,
                                              t->data.scaledBones,
                                              t->data.scales,
                                              t->source.floatTracks,
                                              t->source.events,
                                              projectors,
                                              t->data.scaledBoneCount,
                                              t->source.floatTrackCount,
                                              t->source.eventCount,
                                              t->source.options.latentCount,
                                              t->source.database->sampleRate,
                                              weightFormat,
                                              0};
        r = maniBuildLearnedModel(builder, &model);
    }
    maniRelease(&t->allocator, projectorLayers, count, sizeof(maniLearnedLayerSource), 8);
    maniRelease(&t->allocator, list, projectors, sizeof(maniLearnedProjectorSource), 8);
    return r;
}
