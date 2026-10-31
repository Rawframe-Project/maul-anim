// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model file's view: every section checked once as
// hostile input (the networks' shapes against their roles, every value
// and name), then read in place.

#include "maul-anim/learned.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "learned_file.h"
#include "match_file.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint64_t maniLearnedComputeIdentity(const uint8_t* const data[MANI_LEARNED_SECTIONS],
                                    const uint64_t sizes[MANI_LEARNED_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_LEARNED_SECTIONS; ++i)
    {
        hash = maniHashBytes(hash, data[i], sizes[i]);
    }
    return hash;
}

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static const maniFileResult kOk = {mani_success, 0, 0};

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

typedef struct Sections
{
    const uint8_t* data[MANI_LEARNED_SECTIONS];
    uint64_t sizes[MANI_LEARNED_SECTIONS];
    uint64_t at[MANI_LEARNED_SECTIONS];
    // Where each one's size is in the table; 12 (the header's section
    // count) for an absent one.
    uint64_t entry[MANI_LEARNED_SECTIONS];
} Sections;

static const uint8_t kEmpty = 0;

static maniFileResult LoadSections(const maniContainer* c, Sections* s)
{
    for (uint32_t i = 0; i < MANI_LEARNED_SECTIONS; ++i)
    {
        s->data[i] = &kEmpty;
        s->sizes[i] = 0;
        s->at[i] = 0;
        s->entry[i] = 12;
    }
    for (uint32_t i = 0; i < c->sectionCount; ++i)
    {
        uint64_t entryAt = MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        const uint8_t* entry = c->bytes + entryAt;
        uint32_t tag = maniLoadU32(entry);
        if (tag == 0 || tag > MANI_LEARNED_SECTIONS)
        {
            return Fail(entryAt);
        }
        uint64_t size = maniLoadU64(entry + 16);
        if (size == 0)
        {
            return Fail(entryAt + 16);
        }
        uint64_t offset = maniLoadU64(entry + 8);
        s->data[tag - 1] = c->bytes + offset;
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = offset;
        s->entry[tag - 1] = entryAt + 16;
    }
    return kOk;
}

// The header's counts within their limits, the format, the rate, the
// axes on two different lines.
static maniFileResult CheckHeader(const Sections* s, maniLearnedModel* m)
{
    if (s->sizes[0] != MANI_LEARNED_HEADER_SIZE)
    {
        return Fail(s->entry[0]);
    }
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    uint32_t counts[10];
    // Bones, features, channels, times, latent, scaled bones (at most the
    // bones), float tracks, events, Projectors; then the layers, at least
    // one a network.
    const uint32_t lows[9] = {1, 1, 1, 1, 1, 0, 0, 0, 1};
    uint32_t highs[9] = {MANI_MAX_BONES,
                         MANI_MAX_MATCH_FEATURES,
                         MANI_MAX_MATCH_CHANNELS,
                         MANI_MAX_MATCH_CHANNELS * MANI_MAX_MATCH_TIMES,
                         MANI_MAX_LEARNED_LATENT,
                         MANI_MAX_BONES,
                         MANI_MAX_LEARNED_FLOAT_TRACKS,
                         MANI_MAX_LEARNED_EVENTS,
                         MANI_MAX_LEARNED_PROJECTORS};
    for (uint32_t k = 0; k < 10; ++k)
    {
        counts[k] = maniLoadU32(h + 16 + 4 * k);
        uint32_t networks = k == 9 ? MANI_LEARNED_PROJECTORS + counts[8] : 0u;
        uint32_t low = k < 9 ? lows[k] : networks;
        uint32_t high = k < 9 ? highs[k] : networks * MANI_MAX_LEARNED_LAYERS;
        high = k == 5 ? counts[0] : high;
        if (counts[k] < low || counts[k] > high)
        {
            return Fail(at + 16 + 4 * k);
        }
    }
    float rate = LoadF32(h + 60);
    const bool valid[4] = {maniLoadU32(h + 56) <= maniLearnedWeightsInt16,
                           rate > 0.0f && rate <= MANI_MAX_LEARNED_RATE,
                           maniLoadU32(h + 64) <= maniMatchAxisNegativeZ,
                           maniLoadU32(h + 68) <= maniMatchAxisNegativeZ &&
                               maniLoadU32(h + 68) % 3 != maniLoadU32(h + 64) % 3};
    for (uint32_t k = 0; k < 4; ++k)
    {
        if (!valid[k])
        {
            return Fail(at + 56 + 4 * k);
        }
    }
    *m = (maniLearnedModel){.skeletonIdentity = maniLoadU64(h),
                            .databaseIdentity = maniLoadU64(h + 8),
                            .boneCount = counts[0],
                            .featureCount = counts[1],
                            .channelCount = counts[2],
                            .timeCount = counts[3],
                            .latentCount = counts[4],
                            .scaledBoneCount = counts[5],
                            .floatTrackCount = counts[6],
                            .eventCount = counts[7],
                            .projectorCount = counts[8],
                            .layerCount = counts[9],
                            .weightFormat = maniLoadU32(h + 56),
                            .stepRate = rate,
                            .up = maniLoadU32(h + 64),
                            .forward = maniLoadU32(h + 68)};
    m->outputCount = (uint32_t)maniLearnedOutputCount(m->boneCount, m->scaledBoneCount,
                                                      m->floatTrackCount, m->eventCount);
    return kOk;
}

// A section of an exact size: absent when that size is 0.
static maniFileResult CheckSize(const Sections* s, uint32_t tag, uint64_t size)
{
    return s->sizes[tag - 1] == size ? kOk : Fail(s->entry[tag - 1]);
}

// The sections the header's counts size.
static maniFileResult CheckSizes(const Sections* s, maniLearnedModel* m)
{
    const uint64_t sizes[7] = {
        (uint64_t)m->channelCount * MANI_LEARNED_CHANNEL_SIZE,
        4 * (uint64_t)m->timeCount,
        8 * (uint64_t)m->featureCount,
        4 * (uint64_t)m->scaledBoneCount + 12 * (uint64_t)m->boneCount,
        (uint64_t)(MANI_LEARNED_PROJECTORS + m->projectorCount) * MANI_LEARNED_NETWORK_SIZE,
        (uint64_t)m->layerCount * MANI_LEARNED_LAYER_SIZE,
        (uint64_t)(m->floatTrackCount + m->eventCount) * MANI_LEARNED_NAME_SIZE};
    const uint32_t tags[7] = {maniLearnedTagChannels,      maniLearnedTagTimes,
                              maniLearnedTagNormalization, maniLearnedTagBones,
                              maniLearnedTagNetworks,      maniLearnedTagLayers,
                              maniLearnedTagNames};
    for (uint32_t k = 0; k < 7; ++k)
    {
        maniFileResult r = CheckSize(s, tags[k], sizes[k]);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    m->channels = s->data[maniLearnedTagChannels - 1];
    m->times = s->data[maniLearnedTagTimes - 1];
    m->normalization = s->data[maniLearnedTagNormalization - 1];
    m->bones = s->data[maniLearnedTagBones - 1];
    m->networks = s->data[maniLearnedTagNetworks - 1];
    m->layers = s->data[maniLearnedTagLayers - 1];
    m->values = s->data[maniLearnedTagValues - 1];
    m->weights = s->data[maniLearnedTagWeights - 1];
    m->names = s->data[maniLearnedTagNames - 1];
    m->nameBytes = s->data[maniLearnedTagNameBytes - 1];
    m->nameBytesSize = s->sizes[maniLearnedTagNameBytes - 1];
    return kOk;
}

// One channel's words in order (kind, bone, axis, times, first time,
// first feature, features, weight), the first wrong one's place.
static uint32_t ChannelWrong(const maniLearnedModel* m, const uint8_t* e, uint32_t timeAt,
                             uint32_t featureAt)
{
    uint32_t kind = maniLoadU32(e);
    uint32_t times = maniLoadU32(e + 12);
    bool bone = (uint32_t)kind <= maniMatchBoneFacing;
    float weight = LoadF32(e + 28);
    const bool valid[8] = {(uint32_t)kind <= maniMatchFloatTrack,
                           bone ? maniLoadU32(e + 4) < m->boneCount : maniLoadU32(e + 4) == 0,
                           kind == maniMatchBoneFacing
                               ? maniLoadU32(e + 8) <= maniMatchAxisNegativeZ
                               : maniLoadU32(e + 8) == 0,
                           times >= 1 && times <= MANI_MAX_MATCH_TIMES,
                           maniLoadU32(e + 16) == timeAt,
                           maniLoadU32(e + 20) == featureAt,
                           maniLoadU32(e + 24) == maniMatchChannelWidth(kind) * times,
                           weight > 0.0f && weight < HUGE_VALF};
    for (uint32_t k = 0; k < 8; ++k)
    {
        if (!valid[k])
        {
            return 4 * k;
        }
    }
    return UINT32_MAX;
}

static maniFileResult CheckChannels(const maniLearnedModel* m, const Sections* s)
{
    uint32_t timeAt = 0;
    uint32_t featureAt = 0;
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        const uint8_t* e = m->channels + (size_t)c * MANI_LEARNED_CHANNEL_SIZE;
        uint64_t here = s->at[maniLearnedTagChannels - 1] + (uint64_t)c * MANI_LEARNED_CHANNEL_SIZE;
        uint32_t wrong = ChannelWrong(m, e, timeAt, featureAt);
        if (wrong != UINT32_MAX)
        {
            return Fail(here + wrong);
        }
        timeAt += maniLoadU32(e + 12);
        featureAt += maniLoadU32(e + 24);
        if (timeAt > m->timeCount || featureAt > m->featureCount)
        {
            return Fail(here + (timeAt > m->timeCount ? 12 : 24));
        }
    }
    if (timeAt != m->timeCount || featureAt != m->featureCount)
    {
        return Fail(s->at[0] + (timeAt != m->timeCount ? 28 : 20));
    }
    for (uint32_t t = 0; t < m->timeCount; ++t)
    {
        if (!(fabsf(LoadF32(m->times + 4 * (size_t)t)) <= MANI_MAX_MATCH_REACH))
        {
            return Fail(s->at[maniLearnedTagTimes - 1] + 4 * (uint64_t)t);
        }
    }
    return kOk;
}

// The normalization's offsets finite, its scales finite and above 0; the
// scaled bones increasing within the bones, every bone's scale finite.
static maniFileResult CheckFeaturesAndBones(const maniLearnedModel* m, const Sections* s)
{
    for (uint32_t f = 0; f < 2 * m->featureCount; ++f)
    {
        float v = LoadF32(m->normalization + 4 * (size_t)f);
        if (!(f < m->featureCount ? Finite(v) : v > 0.0f && v < HUGE_VALF))
        {
            return Fail(s->at[maniLearnedTagNormalization - 1] + 4 * (uint64_t)f);
        }
    }
    uint64_t at = s->at[maniLearnedTagBones - 1];
    for (uint32_t k = 0; k < m->scaledBoneCount; ++k)
    {
        uint32_t bone = maniLoadU32(m->bones + 4 * (size_t)k);
        if (bone >= m->boneCount || (k > 0 && bone <= maniLoadU32(m->bones + 4 * (size_t)k - 4)))
        {
            return Fail(at + 4 * (uint64_t)k);
        }
    }
    const uint8_t* scales = m->bones + 4 * (size_t)m->scaledBoneCount;
    for (uint32_t k = 0; k < 3 * m->boneCount; ++k)
    {
        if (!Finite(LoadF32(scales + 4 * (size_t)k)))
        {
            return Fail(at + 4 * (uint64_t)m->scaledBoneCount + 4 * (uint64_t)k);
        }
    }
    return kOk;
}

// What network n takes and gives in its role.
static void Role(const maniLearnedModel* m, uint32_t n, uint32_t* in, uint32_t* out)
{
    uint32_t state = m->featureCount + m->latentCount;
    *in = n < MANI_LEARNED_PROJECTORS ? state : m->featureCount;
    *out = n == MANI_LEARNED_DECOMPRESSOR ? m->outputCount : state;
}

// The values and weights the shapes give, counted as the networks are
// checked.
typedef struct Totals
{
    uint64_t values;
    uint64_t weights;
} Totals;

// Network n's record and layers from layer first on: its layer count,
// tags (0 for the Decompressor and Stepper), each layer's sizes within
// the width and chained, the first input and last output its role's.
static maniFileResult CheckNetwork(maniLearnedModel* m, const Sections* s, uint32_t n,
                                   uint32_t first, Totals* t)
{
    const uint8_t* e = m->networks + (size_t)n * MANI_LEARNED_NETWORK_SIZE;
    uint64_t here = s->at[maniLearnedTagNetworks - 1] + (uint64_t)n * MANI_LEARNED_NETWORK_SIZE;
    uint32_t count = maniLoadU32(e);
    bool projector = n >= MANI_LEARNED_PROJECTORS;
    const bool valid[3] = {
        count >= 1 && count <= MANI_MAX_LEARNED_LAYERS && count <= m->layerCount - first,
        projector || maniLoadU32(e + 4) == 0, projector || maniLoadU32(e + 8) == 0};
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (!valid[k])
        {
            return Fail(here + 4 * k);
        }
    }
    uint32_t in = 0;
    uint32_t out = 0;
    Role(m, n, &in, &out);
    t->values += 2 * (uint64_t)in + 2 * (uint64_t)out;
    uint32_t previous = in;
    for (uint32_t l = first; l < first + count; ++l)
    {
        const uint8_t* layer = m->layers + (size_t)l * MANI_LEARNED_LAYER_SIZE;
        uint64_t at = s->at[maniLearnedTagLayers - 1] + (uint64_t)l * MANI_LEARNED_LAYER_SIZE;
        uint32_t inputs = maniLoadU32(layer);
        uint32_t outputs = maniLoadU32(layer + 4);
        if (inputs != previous)
        {
            return Fail(at);
        }
        bool last = l + 1 == first + count;
        if (outputs < 1 || outputs > MANI_MAX_LEARNED_WIDTH || (last && outputs != out))
        {
            return Fail(at + 4);
        }
        t->values += outputs + (m->weightFormat == maniLearnedWeightsInt16 ? inputs : 0u);
        t->weights += (uint64_t)inputs * outputs;
        if (t->weights > MANI_MAX_LEARNED_WEIGHTS)
        {
            return Fail(at + 4);
        }
        m->widest = inputs > m->widest ? inputs : m->widest;
        m->widest = outputs > m->widest ? outputs : m->widest;
        previous = outputs;
    }
    return kOk;
}

static maniFileResult CheckNetworks(maniLearnedModel* m, const Sections* s, Totals* t)
{
    uint32_t first = 0;
    for (uint32_t n = 0; n < MANI_LEARNED_PROJECTORS + m->projectorCount; ++n)
    {
        maniFileResult r = CheckNetwork(m, s, n, first, t);
        if (r.result != mani_success)
        {
            return r;
        }
        first += maniLoadU32(m->networks + (size_t)n * MANI_LEARNED_NETWORK_SIZE);
    }
    return first == m->layerCount ? kOk : Fail(s->at[0] + 52);
}

// Every value finite; each network's input scales not 0, and with 16-bit
// weights each layer's input scales not below 0.
static maniFileResult CheckValues(const maniLearnedModel* m, const Sections* s, uint64_t count)
{
    maniFileResult r = CheckSize(s, maniLearnedTagValues, 4 * count);
    uint64_t base = s->at[maniLearnedTagValues - 1];
    uint64_t v = 0;
    uint32_t layer = 0;
    for (uint32_t n = 0;
         r.result == mani_success && n < MANI_LEARNED_PROJECTORS + m->projectorCount; ++n)
    {
        uint32_t in = 0;
        uint32_t out = 0;
        Role(m, n, &in, &out);
        for (uint64_t k = 0; k < 2 * (uint64_t)in + 2 * (uint64_t)out; ++k, ++v)
        {
            float x = LoadF32(m->values + 4 * v);
            bool scale = k >= in && k < 2 * (uint64_t)in;
            if (!Finite(x) || (scale && x == 0.0f))
            {
                return Fail(base + 4 * v);
            }
        }
        uint32_t layers = maniLoadU32(m->networks + (size_t)n * MANI_LEARNED_NETWORK_SIZE);
        for (uint32_t l = layer; l < layer + layers; ++l)
        {
            const uint8_t* e = m->layers + (size_t)l * MANI_LEARNED_LAYER_SIZE;
            uint64_t biases = maniLoadU32(e + 4);
            uint64_t scales = m->weightFormat == maniLearnedWeightsInt16 ? maniLoadU32(e) : 0u;
            for (uint64_t k = 0; k < biases + scales; ++k, ++v)
            {
                float x = LoadF32(m->values + 4 * v);
                if (!Finite(x) || (k >= biases && x < 0.0f))
                {
                    return Fail(base + 4 * v);
                }
            }
        }
        layer += layers;
    }
    return r;
}

// binary32 weights finite; 16-bit ones any, their padding 0.
static maniFileResult CheckWeights(const maniLearnedModel* m, const Sections* s, uint64_t count)
{
    bool wide = m->weightFormat == maniLearnedWeightsFloat32;
    uint64_t size = wide ? 4 * count : (2 * count + 3) / 4 * 4;
    maniFileResult r = CheckSize(s, maniLearnedTagWeights, size);
    uint64_t base = s->at[maniLearnedTagWeights - 1];
    for (uint64_t k = 0; wide && r.result == mani_success && k < count; ++k)
    {
        if (!Finite(LoadF32(m->weights + 4 * k)))
        {
            return Fail(base + 4 * k);
        }
    }
    if (!wide && r.result == mani_success && count % 2 != 0 &&
        maniLoadU16(m->weights + 2 * count) != 0)
    {
        return Fail(base + 2 * count);
    }
    return r;
}

// The names end to end over the name bytes, not empty, UTF-8, unique
// within their list (float tracks, events).
static maniFileResult CheckNames(const maniLearnedModel* m, const Sections* s)
{
    uint32_t count = m->floatTrackCount + m->eventCount;
    uint64_t base = s->at[maniLearnedTagNames - 1];
    uint64_t at = 0;
    for (uint32_t k = 0; k < count; ++k)
    {
        const uint8_t* e = m->names + (size_t)k * MANI_LEARNED_NAME_SIZE;
        uint32_t length = maniLoadU32(e + 4);
        if (maniLoadU32(e) != at)
        {
            return Fail(base + (uint64_t)k * MANI_LEARNED_NAME_SIZE);
        }
        bool fits = length >= 1 && length <= m->nameBytesSize - at;
        if (!fits || !maniValidUtf8(m->nameBytes + at, length))
        {
            return Fail(base + (uint64_t)k * MANI_LEARNED_NAME_SIZE + 4);
        }
        uint32_t listStart = k < m->floatTrackCount ? 0u : m->floatTrackCount;
        for (uint32_t j = listStart; j < k; ++j)
        {
            const uint8_t* o = m->names + (size_t)j * MANI_LEARNED_NAME_SIZE;
            if (maniLoadU32(o + 4) == length &&
                memcmp(m->nameBytes + maniLoadU32(o), m->nameBytes + at, length) == 0)
            {
                return Fail(base + (uint64_t)k * MANI_LEARNED_NAME_SIZE + 4);
            }
        }
        at += length;
    }
    return at == m->nameBytesSize ? kOk : Fail(s->entry[maniLearnedTagNameBytes - 1]);
}

static MANI_NOINLINE maniFileResult OpenLearnedModelWork(const void* bytes, size_t size, void* out)
{
    maniLearnedModel* modelOut = out;
    if (modelOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindLearnedModel, MANI_LEARNED_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    Sections s;
    maniLearnedModel m;
    Totals t = {0, 0};
    r = LoadSections(&c, &s);
    r = r.result == mani_success ? CheckHeader(&s, &m) : r;
    r = r.result == mani_success ? CheckSizes(&s, &m) : r;
    r = r.result == mani_success ? CheckChannels(&m, &s) : r;
    r = r.result == mani_success ? CheckFeaturesAndBones(&m, &s) : r;
    r = r.result == mani_success ? CheckNetworks(&m, &s, &t) : r;
    r = r.result == mani_success ? CheckValues(&m, &s, t.values) : r;
    r = r.result == mani_success ? CheckWeights(&m, &s, t.weights) : r;
    r = r.result == mani_success ? CheckNames(&m, &s) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniLearnedComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    m.identity = c.identity;
    *modelOut = m;
    return kOk;
}

maniFileResult maniOpenLearnedModel(const void* bytes, size_t size, maniLearnedModel* modelOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = maniContainerOpenTagged(bytes, size, modelOut, OpenLearnedModelWork);
    maniFloatEnvLeave(env);
    return result;
}

uint64_t maniLearnedModel_GetIdentity(const maniLearnedModel* model)
{
    return model != NULL ? model->identity : 0;
}

uint64_t maniLearnedModel_GetSkeletonIdentity(const maniLearnedModel* model)
{
    return model != NULL ? model->skeletonIdentity : 0;
}

uint64_t maniLearnedModel_GetDatabaseIdentity(const maniLearnedModel* model)
{
    return model != NULL ? model->databaseIdentity : 0;
}

maniResult maniLearnedModel_GetChannel(const maniLearnedModel* model, uint32_t channel,
                                       maniMatchChannel* channelOut)
{
    if (model == NULL || channelOut == NULL || channel >= model->channelCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e = model->channels + (size_t)channel * MANI_LEARNED_CHANNEL_SIZE;
    uint32_t count = maniLoadU32(e + 12);
    *channelOut = (maniMatchChannel){.kind = maniLoadU32(e),
                                     .bone = maniLoadU32(e + 4),
                                     .axis = maniLoadU32(e + 8),
                                     .timeCount = count,
                                     .weight = LoadF32(e + 28),
                                     .firstFeature = maniLoadU32(e + 20),
                                     .featureCount = maniLoadU32(e + 24)};
    memcpy(channelOut->times, model->times + 4 * (size_t)maniLoadU32(e + 16), 4 * (size_t)count);
    return mani_success;
}

maniResult maniLearnedModel_GetNormalization(const maniLearnedModel* model, float* offsetsOut,
                                             float* scalesOut, uint32_t capacity,
                                             uint32_t* countOut)
{
    if (model == NULL || (offsetsOut == NULL && scalesOut == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = model->featureCount;
    }
    if (capacity < model->featureCount)
    {
        return mani_errorCapacity;
    }
    size_t bytes = 4 * (size_t)model->featureCount;
    if (offsetsOut != NULL)
    {
        memcpy(offsetsOut, model->normalization, bytes);
    }
    if (scalesOut != NULL)
    {
        memcpy(scalesOut, model->normalization + bytes, bytes);
    }
    return mani_success;
}

maniResult maniLearnedModel_GetProjector(const maniLearnedModel* model, uint32_t projector,
                                         uint32_t* requiredOut, uint32_t* excludedOut)
{
    if (model == NULL || projector >= model->projectorCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e =
        model->networks + (size_t)(MANI_LEARNED_PROJECTORS + projector) * MANI_LEARNED_NETWORK_SIZE;
    if (requiredOut != NULL)
    {
        *requiredOut = maniLoadU32(e + 4);
    }
    if (excludedOut != NULL)
    {
        *excludedOut = maniLoadU32(e + 8);
    }
    return mani_success;
}

static const char* Name(const maniLearnedModel* model, uint32_t k, size_t* lengthOut)
{
    const uint8_t* e = model->names + (size_t)k * MANI_LEARNED_NAME_SIZE;
    if (lengthOut != NULL)
    {
        *lengthOut = maniLoadU32(e + 4);
    }
    return (const char*)(model->nameBytes + maniLoadU32(e));
}

uint32_t maniLearnedModel_GetFloatTrackCount(const maniLearnedModel* model)
{
    return model == NULL ? 0 : model->floatTrackCount;
}

uint32_t maniLearnedModel_GetEventCount(const maniLearnedModel* model)
{
    return model == NULL ? 0 : model->eventCount;
}

const char* maniLearnedModel_GetFloatTrackName(const maniLearnedModel* model, uint32_t track,
                                               size_t* lengthOut)
{
    if (model == NULL || track >= model->floatTrackCount)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return Name(model, track, lengthOut);
}

const char* maniLearnedModel_GetEventName(const maniLearnedModel* model, uint32_t event,
                                          size_t* lengthOut)
{
    if (model == NULL || event >= model->eventCount)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return Name(model, model->floatTrackCount + event, lengthOut);
}
