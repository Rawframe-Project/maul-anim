// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's trainer (mani-0025): its def and source checked, its
// data built, its signature, and its release.

#include "learned_train.h"

#include "allocator.h"
#include "learned_build.h"
#include "learned_file.h"
#include "train_data.h"
#include "train_decompress.h"
#include "train_net.h"
#include "train_projector.h"
#include "train_stepper.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/learned.h"
#include "maul-anim/learnedtrain.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorInvalid, reason, item, other);
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

// Each network's sizes: n 0 the Compressor, 1 the Decompressor, 2 the
// Stepper, 3 a Projector.
static bool Shape(const maniLearnedTrainerSource* s, const maniTrainData* d, uint32_t n,
                  maniTrainNet* net)
{
    const maniLearnedTrainerOptions* o = &s->options;
    const uint32_t hidden[4] = {o->compressorLayers, o->decompressorLayers, o->stepperLayers,
                                o->projectorLayers};
    uint32_t states = d->featureCount + o->latentCount;
    const uint32_t in[4] = {d->inputCount, states, states, d->featureCount};
    const uint32_t out[4] = {o->latentCount, d->outputCount, states, states};
    uint32_t sizes[MANI_LEARNED_MAX_LAYERS + 1];
    if (hidden[n] >= MANI_LEARNED_MAX_LAYERS)
    {
        return false;
    }
    sizes[0] = in[n];
    for (uint32_t l = 1; l <= hidden[n]; ++l)
    {
        sizes[l] = o->hiddenWidth;
    }
    sizes[hidden[n] + 1] = out[n];
    return maniTrainNetInit(net, sizes, hidden[n] + 1, n == 0 ? maniTrainElu : maniTrainRelu);
}

bool maniLearnedTrainerShape(const maniLearnedTrainer* t, uint32_t n, maniTrainNet* net)
{
    return Shape(&t->source, &t->data, n, net);
}

// The options alone: item is the option's place in the options.
static maniBuildResult CheckOptions(const maniLearnedTrainerOptions* o)
{
    const bool valid[12] = {o->latentCount >= 1 && o->latentCount <= MANI_LEARNED_MAX_LATENT,
                            o->hiddenWidth >= 1 && o->hiddenWidth <= MANI_LEARNED_MAX_WIDTH,
                            o->compressorLayers < MANI_LEARNED_MAX_LAYERS,
                            o->decompressorLayers < MANI_LEARNED_MAX_LAYERS,
                            o->stepperLayers < MANI_LEARNED_MAX_LAYERS,
                            o->projectorLayers < MANI_LEARNED_MAX_LAYERS,
                            o->batchSize >= MANI_TRAIN_CHUNK &&
                                o->batchSize <= MANI_LEARNED_MAX_BATCH &&
                                o->batchSize % MANI_TRAIN_CHUNK == 0,
                            o->stepperWindow >= 2 && o->stepperWindow <= MANI_LEARNED_MAX_WINDOW,
                            o->iterations >= 1,
                            true,
                            o->learningRate > 0.0f && o->learningRate < HUGE_VALF,
                            o->eventSeconds >= 0.0f && o->eventSeconds < HUGE_VALF};
    for (uint32_t k = 0; k < 12; ++k)
    {
        if (!valid[k])
        {
            return Refuse(maniBuildReasonOption, k, 0);
        }
    }
    return kOk;
}

// The source's objects with each other: the skeleton, the clips, the
// rate, the counts, the names and the filters.
static maniBuildResult CheckSource(const maniLearnedTrainerSource* s)
{
    const maniMotionDatabase* d = s->database;
    if (maniMotionDatabaseSkeletonIdentity(d) != maniSkeletonIdentity(s->skeleton))
    {
        return Refuse(maniBuildReasonDatabase, 0, 0);
    }
    if (s->clipCount != d->clipCount)
    {
        return Refuse(maniBuildReasonClip, s->clipCount, d->clipCount);
    }
    for (uint32_t k = 0; k < s->clipCount; ++k)
    {
        if (s->clips[k] == NULL ||
            maniClipIdentity(s->clips[k]) != maniMotionDatabaseClipIdentity(d, k))
        {
            return Refuse(maniBuildReasonClip, k, 0);
        }
    }
    if (!(d->sampleRate <= MANI_LEARNED_MAX_RATE))
    {
        return Refuse(maniBuildReasonTiming, 0, 0);
    }
    if (s->projectorCount < 1 || s->projectorCount > MANI_LEARNED_MAX_PROJECTORS ||
        s->floatTrackCount > MANI_LEARNED_MAX_FLOAT_TRACKS ||
        s->eventCount > MANI_LEARNED_MAX_EVENTS)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    maniBuildResult r =
        maniLearnedCheckNames(s->floatTracks, s->floatTrackCount, maniBuildReasonFloatTrack);
    r = r.result == mani_success
            ? maniLearnedCheckNames(s->events, s->eventCount, maniBuildReasonEvent)
            : r;
    for (uint32_t p = 0; r.result == mani_success && p < s->projectorCount; ++p)
    {
        const maniLearnedTagFilter* f = &s->projectors[p];
        r = maniTrainAnyFrame(d, f->required, f->excluded) ? kOk
                                                           : Refuse(maniBuildReasonTags, p, 0);
    }
    return r;
}

// The networks' shapes within the limits, the model's weights in all
// too, and a window of the Stepper in an entry.
static maniBuildResult CheckShapes(const maniLearnedTrainerSource* s, const maniTrainData* d)
{
    maniTrainNet net;
    uint64_t weights = 0;
    for (uint32_t n = 0; n < 4; ++n)
    {
        if (!Shape(s, d, n, &net))
        {
            return Refuse(maniBuildReasonOption, 1, 0);
        }
        uint64_t copies = n == 0 ? 0u : (n == 3 ? s->projectorCount : 1u);
        for (uint32_t l = 0; l < net.layerCount; ++l)
        {
            weights += copies * net.sizes[l] * (uint64_t)net.sizes[l + 1];
        }
    }
    if (weights > MANI_LEARNED_MAX_WEIGHTS)
    {
        return Refuse(maniBuildReasonOption, 1, 0);
    }
    if (maniTrainWindows(d, s->options.stepperWindow, NULL) == 0)
    {
        return Refuse(maniBuildReasonOption, 7, 0);
    }
    return kOk;
}

static bool Given(const maniLearnedTrainerSource* s)
{
    return s->skeleton != NULL && s->database != NULL && (s->clips != NULL || s->clipCount == 0) &&
           s->projectors != NULL && (s->floatTracks != NULL || s->floatTrackCount == 0) &&
           (s->events != NULL || s->eventCount == 0);
}

static maniResult CheckDef(const maniLearnedTrainerDef* def)
{
    if (def == NULL || def->cookie != MANI_LEARNED_TRAINER_DEF_COOKIE)
    {
        return mani_errorInvalid;
    }
    if (def->version != MANI_ABI_VERSION)
    {
        return mani_errorVersion;
    }
    bool hooks = (def->tasks.enqueueTask == NULL) == (def->tasks.finishTask == NULL);
    return maniAllocatorValid(&def->allocator) && hooks ? mani_success : mani_errorInvalid;
}

void maniDestroyLearnedTrainer(maniLearnedTrainer* trainer)
{
    if (trainer == NULL || maniHostCalledBack(&trainer->host))
    {
        return;
    }
    maniLearnedTrainerDrop(trainer);
    maniLearnedTrainerForget(trainer);
    const maniAllocator* a = &trainer->allocator;
    maniRelease(a, trainer->networks, (size_t)trainer->stageCount, sizeof(maniTrainKept), 8);
    maniTrainDataRelease(a, &trainer->data);
    // The block itself straight to the host's: the library's allocator
    // marks the trainer after the host's call returns.
    maniAllocator allocator = trainer->host.allocator;
    maniRelease(&allocator, trainer, 1, sizeof(maniLearnedTrainer), alignof(maniLearnedTrainer));
}

uint32_t maniLearnedTrainerMisuses(const maniLearnedTrainer* trainer)
{
    return trainer != NULL ? maniHostMisuses(&trainer->host) : 0;
}

maniBuildResult maniCreateLearnedTrainer(const maniLearnedTrainerDef* def,
                                         const maniLearnedTrainerSource* source,
                                         maniLearnedTrainer** trainerOut)
{
    if (trainerOut == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    *trainerOut = NULL;
    maniResult checked = CheckDef(def);
    if (checked != mani_success || source == NULL || !Given(source))
    {
        return Result(checked != mani_success ? checked : mani_errorInvalid, maniBuildReasonNone, 0,
                      0);
    }
    maniBuildResult r = CheckOptions(&source->options);
    r = r.result == mani_success ? CheckSource(source) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    maniLearnedTrainer* t =
        maniAllocate(&def->allocator, 1, sizeof(maniLearnedTrainer), alignof(maniLearnedTrainer));
    if (t == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    *t = (maniLearnedTrainer){.source = *source,
                              .stageCount = MANI_LEARNED_PROJECTORS + source->projectorCount};
    maniHostCodeInit(&t->host, &def->allocator, &def->tasks);
    t->allocator = maniHostAllocator(&t->host);
    t->tasks = maniHostTasks(&t->host);
    const maniTrainDataSource data = {source->skeleton,
                                      source->database,
                                      source->clips,
                                      source->floatTracks,
                                      source->events,
                                      source->floatTrackCount,
                                      source->eventCount,
                                      source->options.eventSeconds,
                                      0};
    if (!maniTrainDataBuild(&t->allocator, &data, &t->data))
    {
        maniRelease(&def->allocator, t, 1, sizeof(maniLearnedTrainer), alignof(maniLearnedTrainer));
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    r = CheckShapes(source, &t->data);
    t->networks = r.result == mani_success
                      ? maniAllocate(&t->allocator, t->stageCount, sizeof(maniTrainKept), 8)
                      : NULL;
    if (r.result == mani_success && t->networks == NULL)
    {
        r = Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    if (r.result != mani_success)
    {
        maniDestroyLearnedTrainer(t);
        return r;
    }
    memset(t->networks, 0, sizeof(maniTrainKept) * t->stageCount);
    t->signature = maniLearnedTrainerSignature(source);
    *trainerOut = t;
    return kOk;
}
