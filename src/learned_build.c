// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a learned model: the source checked (the
// networks' shapes against their roles, every value, the bones and
// names), the database's channels, times, normalization and axes copied,
// and the file written, 16-bit weights quantized input by input.

#include "learned_build.h"

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "learned_file.h"
#include "match_file.h"
#include "text.h"

#include "maul-anim/build.h"
#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorInvalid, reason, item, other);
}

static maniBuildResult PastLimit(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorLimit, reason, item, other);
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

static bool AllFinite(const float* values, uint64_t count)
{
    for (uint64_t k = 0; k < count; ++k)
    {
        if (!Finite(values[k]))
        {
            return false;
        }
    }
    return true;
}

// Network n of the source: the Decompressor, the Stepper, then the
// Projectors.
static const maniLearnedNetworkSource* NetworkOf(const maniLearnedModelSource* s, uint32_t n)
{
    if (n == MANI_LEARNED_DECOMPRESSOR)
    {
        return s->decompressor;
    }
    return n == MANI_LEARNED_STEPPER ? s->stepper
                                     : &s->projectors[n - MANI_LEARNED_PROJECTORS].network;
}

// What network n takes and gives in its role.
static void Role(const maniLearnedModelSource* s, uint32_t n, uint32_t* in, uint32_t* out)
{
    uint32_t features = s->database->featureCount;
    uint32_t state = features + s->latentCount;
    *in = n < MANI_LEARNED_PROJECTORS ? state : features;
    uint64_t outputs = maniLearnedOutputCount(s->skeleton->boneCount, s->scaledBoneCount,
                                              s->floatTrackCount, s->eventCount);
    *out = n == MANI_LEARNED_DECOMPRESSOR
               ? (outputs > MANI_MAX_LEARNED_WIDTH ? MANI_MAX_LEARNED_WIDTH + 1 : (uint32_t)outputs)
               : state;
}

// The totals the networks give, counted as they are checked.
typedef struct Totals
{
    uint64_t values;
    uint64_t weights;
    uint32_t layers;
    uint32_t names;
} Totals;

static maniBuildResult CheckLayer(const maniLearnedLayerSource* l, uint32_t n, uint32_t k,
                                  uint32_t previous, bool last, uint32_t out, Totals* t)
{
    if (l->outputCount > MANI_MAX_LEARNED_WIDTH)
    {
        return PastLimit(maniBuildReasonNetwork, n, k);
    }
    bool shape =
        l->inputCount == previous && l->outputCount >= 1 && (!last || l->outputCount == out);
    if (!shape || l->weights == NULL || l->biases == NULL)
    {
        return Refuse(maniBuildReasonNetwork, n, k);
    }
    uint64_t weights = (uint64_t)l->inputCount * l->outputCount;
    t->weights += weights;
    if (t->weights > MANI_MAX_LEARNED_WEIGHTS)
    {
        return PastLimit(maniBuildReasonNetwork, n, k);
    }
    if (!AllFinite(l->weights, weights) || !AllFinite(l->biases, l->outputCount))
    {
        return Refuse(maniBuildReasonNetwork, n, k);
    }
    return kOk;
}

// Network n's layers chained from its role's input to its role's output,
// within the limits, every value finite; its normalization finite, the
// input scales not 0.
static maniBuildResult CheckNetwork(const maniLearnedModelSource* s, uint32_t n,
                                    maniLearnedWeights format, Totals* t)
{
    const maniLearnedNetworkSource* w = NetworkOf(s, n);
    if (w->layerCount > MANI_MAX_LEARNED_LAYERS)
    {
        return PastLimit(maniBuildReasonNetwork, n, 0);
    }
    if (w->layers == NULL || w->layerCount < 1)
    {
        return Refuse(maniBuildReasonNetwork, n, 0);
    }
    uint32_t in = 0;
    uint32_t out = 0;
    Role(s, n, &in, &out);
    uint32_t previous = in;
    for (uint32_t k = 0; k < w->layerCount; ++k)
    {
        const maniLearnedLayerSource* l = &w->layers[k];
        maniBuildResult r = CheckLayer(l, n, k, previous, k + 1 == w->layerCount, out, t);
        if (r.result != mani_success)
        {
            return r;
        }
        t->values += l->outputCount + (format == maniLearnedWeightsInt16 ? l->inputCount : 0u);
        previous = l->outputCount;
    }
    bool given = w->inputOffsets != NULL && w->inputScales != NULL && w->outputOffsets != NULL &&
                 w->outputScales != NULL;
    bool valid = given && AllFinite(w->inputOffsets, in) && AllFinite(w->inputScales, in) &&
                 AllFinite(w->outputOffsets, out) && AllFinite(w->outputScales, out);
    for (uint32_t i = 0; valid && i < in; ++i)
    {
        valid = w->inputScales[i] != 0.0f;
    }
    if (!valid)
    {
        return Refuse(maniBuildReasonNetwork, n, w->layerCount);
    }
    t->values += 2 * (uint64_t)in + 2 * (uint64_t)out;
    t->layers += w->layerCount;
    return kOk;
}

// A list's names not empty, UTF-8 and unique.
static maniBuildResult CheckNames(const maniLearnedNameSource* names, uint32_t count,
                                  maniBuildReason list, Totals* t)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        const maniLearnedNameSource* a = &names[k];
        if (maniNamePastLimit(a->length))
        {
            return Result(mani_errorLimit, list | maniBuildReasonName, k, 0);
        }
        if (a->name == NULL || a->length == 0 || !maniValidUtf8((const uint8_t*)a->name, a->length))
        {
            return Refuse(list | maniBuildReasonName, k, 0);
        }
        for (uint32_t j = 0; j < k; ++j)
        {
            if (names[j].length == a->length && memcmp(names[j].name, a->name, a->length) == 0)
            {
                return Refuse(list | maniBuildReasonDuplicateName, k, j);
            }
        }
        t->names += a->length;
        if (t->names > UINT32_MAX / 2)
        {
            return Refuse(list | maniBuildReasonName, k, 0);
        }
    }
    return kOk;
}

// The model's own fields: the database's skeleton, the rate, the latent
// count and weight format, the counts, the scaled bones and scales.
static maniBuildResult CheckModel(const maniLearnedModelSource* s)
{
    uint32_t bones = s->skeleton->boneCount;
    if (maniMotionDatabase_GetSkeletonIdentity(s->database) !=
        maniSkeleton_GetIdentity(s->skeleton))
    {
        return Refuse(maniBuildReasonDatabase, 0, 0);
    }
    if (s->stepRate > MANI_MAX_LEARNED_RATE)
    {
        return PastLimit(maniBuildReasonTiming, 0, 0);
    }
    if (!(s->stepRate > 0.0f))
    {
        return Refuse(maniBuildReasonTiming, 0, 0);
    }
    if (s->latentCount > MANI_MAX_LEARNED_LATENT)
    {
        return PastLimit(maniBuildReasonOption, 0, 0);
    }
    if (s->latentCount < 1)
    {
        return Refuse(maniBuildReasonOption, 0, 0);
    }
    if ((uint32_t)s->weightFormat > maniLearnedWeightsInt16)
    {
        return Refuse(maniBuildReasonOption, 1, 0);
    }
    if (s->projectorCount > MANI_MAX_LEARNED_PROJECTORS ||
        s->floatTrackCount > MANI_MAX_LEARNED_FLOAT_TRACKS ||
        s->eventCount > MANI_MAX_LEARNED_EVENTS)
    {
        return PastLimit(maniBuildReasonCount, 0, 0);
    }
    if (s->projectorCount < 1 || s->scaledBoneCount > bones)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    for (uint32_t k = 0; k < s->scaledBoneCount; ++k)
    {
        if (s->scaledBones[k] >= bones || (k > 0 && s->scaledBones[k] <= s->scaledBones[k - 1]))
        {
            return Refuse(maniBuildReasonJoint, k, 0);
        }
    }
    for (uint32_t b = 0; s->scales != NULL && b < bones; ++b)
    {
        if (!AllFinite(s->scales + 3 * (size_t)b, 3))
        {
            return Refuse(maniBuildReasonNotFinite, b, 0);
        }
    }
    return kOk;
}

maniBuildResult maniLearnedCheckNames(const maniLearnedNameSource* names, uint32_t count,
                                      maniBuildReason list)
{
    Totals totals = {0, 0, 0, 0};
    return CheckNames(names, count, list, &totals);
}

static maniBuildResult Check(const maniLearnedModelSource* s, Totals* t)
{
    maniBuildResult r = CheckModel(s);
    for (uint32_t n = 0;
         r.result == mani_success && n < MANI_LEARNED_PROJECTORS + s->projectorCount; ++n)
    {
        r = CheckNetwork(s, n, s->weightFormat, t);
    }
    r = r.result == mani_success
            ? CheckNames(s->floatTracks, s->floatTrackCount, maniBuildReasonFloatTrack, t)
            : r;
    return r.result == mani_success ? CheckNames(s->events, s->eventCount, maniBuildReasonEvent, t)
                                    : r;
}

static void StoreF32(uint8_t* p, float v)
{
    memcpy(p, &v, sizeof v);
}

static uint8_t* PutFloats(uint8_t* at, const float* values, uint64_t count)
{
    memcpy(at, values, 4 * (size_t)count);
    return at + 4 * count;
}

static void LayHeader(const maniLearnedModelSource* s, const Totals* t, uint8_t* h)
{
    const maniMotionDatabase* d = s->database;
    maniStoreU64(h, maniSkeleton_GetIdentity(s->skeleton));
    maniStoreU64(h + 8, maniMotionDatabase_GetIdentity(d));
    const uint32_t words[11] = {
        s->skeleton->boneCount, d->featureCount,    d->channelCount,    d->timeCount,
        s->latentCount,         s->scaledBoneCount, s->floatTrackCount, s->eventCount,
        s->projectorCount,      t->layers,          s->weightFormat};
    for (uint32_t k = 0; k < 11; ++k)
    {
        maniStoreU32(h + 16 + 4 * k, words[k]);
    }
    StoreF32(h + 60, s->stepRate);
    maniStoreU32(h + 64, d->up);
    maniStoreU32(h + 68, d->forward);
}

// The channels' first 32 bytes are the database's records' own layout.
static void LayChannels(const maniMotionDatabase* d, uint8_t* channels)
{
    for (uint32_t c = 0; c < d->channelCount; ++c)
    {
        memcpy(channels + (size_t)c * MANI_LEARNED_CHANNEL_SIZE,
               d->channels + (size_t)c * MANI_MATCH_CHANNEL_SIZE, MANI_LEARNED_CHANNEL_SIZE);
    }
}

static void LayBones(const maniLearnedModelSource* s, uint8_t* bones)
{
    for (uint32_t k = 0; k < s->scaledBoneCount; ++k)
    {
        maniStoreU32(bones + 4 * (size_t)k, s->scaledBones[k]);
    }
    uint8_t* scales = bones + 4 * (size_t)s->scaledBoneCount;
    for (uint32_t k = 0; k < 3 * s->skeleton->boneCount; ++k)
    {
        StoreF32(scales + 4 * (size_t)k, s->scales != NULL ? s->scales[k] : 1.0f);
    }
}

// x rounded to nearest, ties to even.
static float RoundEven(float x)
{
    float low = floorf(x);
    float part = x - low;
    if (part > 0.5f || (part == 0.5f && fmodf(low, 2.0f) != 0.0f))
    {
        return low + 1.0f;
    }
    return low;
}

// A layer's 16-bit weights, input by input: each input's weights over
// their largest magnitude / 32767, rounded to nearest even; that scale
// kept among the values.
static void Quantize(const maniLearnedLayerSource* l, uint8_t* weights, uint8_t* scales)
{
    for (uint32_t i = 0; i < l->inputCount; ++i)
    {
        const float* row = l->weights + (size_t)i * l->outputCount;
        float largest = 0.0f;
        for (uint32_t o = 0; o < l->outputCount; ++o)
        {
            largest = fabsf(row[o]) > largest ? fabsf(row[o]) : largest;
        }
        float scale = largest / 32767.0f;
        StoreF32(scales + 4 * (size_t)i, scale);
        for (uint32_t o = 0; o < l->outputCount; ++o)
        {
            float q = scale > 0.0f ? RoundEven(row[o] / scale) : 0.0f;
            q = q > 32767.0f ? 32767.0f : (q < -32767.0f ? -32767.0f : q);
            maniStoreU16(weights + 2 * ((size_t)i * l->outputCount + o), (uint16_t)(int16_t)q);
        }
    }
}

// Every network's values and weights, in the file's order.
static void LayNetworks(const maniLearnedModelSource* s, uint8_t* networks, uint8_t* layers,
                        uint8_t* values, uint8_t* weights)
{
    bool narrow = s->weightFormat == maniLearnedWeightsInt16;
    uint32_t layer = 0;
    for (uint32_t n = 0; n < MANI_LEARNED_PROJECTORS + s->projectorCount; ++n)
    {
        const maniLearnedNetworkSource* w = NetworkOf(s, n);
        uint8_t* e = networks + (size_t)n * MANI_LEARNED_NETWORK_SIZE;
        bool projector = n >= MANI_LEARNED_PROJECTORS;
        maniStoreU32(e, w->layerCount);
        maniStoreU32(e + 4,
                     projector ? s->projectors[n - MANI_LEARNED_PROJECTORS].requiredTags : 0);
        maniStoreU32(e + 8,
                     projector ? s->projectors[n - MANI_LEARNED_PROJECTORS].excludedTags : 0);
        uint32_t in = 0;
        uint32_t out = 0;
        Role(s, n, &in, &out);
        values = PutFloats(values, w->inputOffsets, in);
        values = PutFloats(values, w->inputScales, in);
        values = PutFloats(values, w->outputOffsets, out);
        values = PutFloats(values, w->outputScales, out);
        for (uint32_t k = 0; k < w->layerCount; ++k, ++layer)
        {
            const maniLearnedLayerSource* l = &w->layers[k];
            maniStoreU32(layers + (size_t)layer * MANI_LEARNED_LAYER_SIZE, l->inputCount);
            maniStoreU32(layers + (size_t)layer * MANI_LEARNED_LAYER_SIZE + 4, l->outputCount);
            values = PutFloats(values, l->biases, l->outputCount);
            uint64_t count = (uint64_t)l->inputCount * l->outputCount;
            if (narrow)
            {
                Quantize(l, weights, values);
                values += 4 * (size_t)l->inputCount;
                weights += 2 * count;
            }
            else
            {
                weights = PutFloats(weights, l->weights, count);
            }
        }
    }
}

static void LayNames(const maniLearnedModelSource* s, uint8_t* names, uint8_t* bytes)
{
    uint32_t at = 0;
    for (uint32_t k = 0; k < s->floatTrackCount + s->eventCount; ++k)
    {
        const maniLearnedNameSource* n =
            k < s->floatTrackCount ? &s->floatTracks[k] : &s->events[k - s->floatTrackCount];
        maniStoreU32(names + (size_t)k * MANI_LEARNED_NAME_SIZE, at);
        // The check took only names within MANI_MAX_NAME_BYTES.
        maniStoreU32(names + (size_t)k * MANI_LEARNED_NAME_SIZE + 4, (uint32_t)n->length);
        memcpy(bytes + at, n->name, n->length);
        at += n->length;
    }
}

static maniBuildResult Write(maniBuilder* builder, const maniLearnedModelSource* s, const Totals* t)
{
    const maniMotionDatabase* d = s->database;
    uint64_t weightBytes = s->weightFormat == maniLearnedWeightsFloat32
                               ? 4 * t->weights
                               : (2 * t->weights + 3) / 4 * 4;
    const uint64_t sizes[MANI_LEARNED_SECTIONS] = {
        MANI_LEARNED_HEADER_SIZE,
        (uint64_t)d->channelCount * MANI_LEARNED_CHANNEL_SIZE,
        4 * (uint64_t)d->timeCount,
        8 * (uint64_t)d->featureCount,
        4 * (uint64_t)s->scaledBoneCount + 12 * (uint64_t)s->skeleton->boneCount,
        (uint64_t)(MANI_LEARNED_PROJECTORS + s->projectorCount) * MANI_LEARNED_NETWORK_SIZE,
        (uint64_t)t->layers * MANI_LEARNED_LAYER_SIZE,
        4 * t->values,
        weightBytes,
        (uint64_t)(s->floatTrackCount + s->eventCount) * MANI_LEARNED_NAME_SIZE,
        t->names};
    uint64_t own = 0;
    for (uint32_t i = 0; i < MANI_LEARNED_SECTIONS; ++i)
    {
        own += sizes[i];
    }
    uint8_t* staging =
        own < SIZE_MAX ? maniAllocate(&builder->allocator, (size_t)own + 1, 1, 8) : NULL;
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(staging, 0, (size_t)own + 1);
    uint8_t* data[MANI_LEARNED_SECTIONS];
    uint8_t* at = staging;
    for (uint32_t i = 0; i < MANI_LEARNED_SECTIONS; ++i)
    {
        data[i] = at;
        at += sizes[i];
    }
    LayHeader(s, t, data[0]);
    LayChannels(d, data[1]);
    memcpy(data[2], d->times, (size_t)sizes[2]);
    memcpy(data[3], d->normalization, (size_t)sizes[3]);
    LayBones(s, data[4]);
    LayNetworks(s, data[5], data[6], data[7], data[8]);
    LayNames(s, data[9], data[10]);
    maniSectionSource sections[MANI_LEARNED_SECTIONS];
    const uint8_t* views[MANI_LEARNED_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_LEARNED_SECTIONS; ++i)
    {
        views[i] = data[i];
        if (sizes[i] > 0)
        {
            sections[count++] = (maniSectionSource){i + 1, data[i], sizes[i]};
        }
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindLearnedModel, MANI_LEARNED_FORMAT_VERSION,
                           maniLearnedComputeIdentity(views, sizes), sections, count);
        maniBuilderKeep(builder, maniKindLearnedModel, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, (size_t)own + 1, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

// The lists the counts name are given.
static bool Given(const maniLearnedModelSource* s)
{
    return s->skeleton != NULL && s->database != NULL && s->decompressor != NULL &&
           s->stepper != NULL && (s->projectors != NULL || s->projectorCount == 0) &&
           (s->scaledBones != NULL || s->scaledBoneCount == 0) &&
           (s->floatTracks != NULL || s->floatTrackCount == 0) &&
           (s->events != NULL || s->eventCount == 0);
}

static MANI_NOINLINE maniBuildResult BuildLearnedModelWork(maniBuilder* builder,
                                                           const maniLearnedModelSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (source == NULL || !Given(source))
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    Totals totals = {0, 0, 0, 0};
    maniBuildResult r = Check(source, &totals);
    return r.result == mani_success ? Write(builder, source, &totals) : r;
}

maniBuildResult maniBuildLearnedModel(maniBuilder* builder, const maniLearnedModelSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildLearnedModelWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
