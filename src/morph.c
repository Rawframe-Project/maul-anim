// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Morph targets (mani-0021): opening the file (every count, offset,
// index and delta checked, so the job reads without checks), the channel
// weights' walk through their targets (ufbx's rule for FBX in-betweens),
// and the job: streams plus weighted deltas in target order, directions
// renormalized.

#include "maul-anim/morph.h"

#include "bytes.h"
#include "container.h"
#include "morph_file.h"

#include "maul-anim/base.h"
#include "maul-anim/skinning.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

uint64_t maniMorphComputeIdentity(const uint8_t* const data[MANI_MORPH_SECTIONS],
                                  const uint64_t sizes[MANI_MORPH_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_MORPH_SECTIONS; ++i)
    {
        hash = maniHashBytes(hash, data[i], sizes[i]);
    }
    return hash;
}

// The file's sections by tag (index tag - 1), an absent one empty.
typedef struct Sections
{
    const uint8_t* data[MANI_MORPH_SECTIONS];
    uint64_t sizes[MANI_MORPH_SECTIONS];
    uint64_t at[MANI_MORPH_SECTIONS];
    // Where each one's size is in the table; 12 for an absent one.
    uint64_t entry[MANI_MORPH_SECTIONS];
} Sections;

static const uint8_t kEmpty = 0;

static maniFileResult LoadSections(const maniContainer* c, Sections* s)
{
    for (uint32_t i = 0; i < MANI_MORPH_SECTIONS; ++i)
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
        if (tag == 0 || tag > MANI_MORPH_SECTIONS)
        {
            return Fail(entryAt);
        }
        uint64_t size = maniLoadU64(entry + 16);
        if (size == 0)
        {
            return Fail(entryAt + 16);
        }
        s->data[tag - 1] = c->bytes + maniLoadU64(entry + 8);
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = maniLoadU64(entry + 8);
        s->entry[tag - 1] = entryAt + 16;
    }
    return kOk;
}

// The header's counts and the fixed-size sections against them.
static maniFileResult CheckHeader(const Sections* s, maniMorphTargets* m)
{
    if (s->sizes[0] != MANI_MORPH_HEADER_SIZE)
    {
        return Fail(s->entry[0]);
    }
    const uint8_t* h = s->data[0];
    m->vertexCount = maniLoadU32(h);
    m->channelCount = maniLoadU32(h + 4);
    m->targetCount = maniLoadU32(h + 8);
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (maniLoadU32(h + 4 * k) == 0)
        {
            return Fail(s->at[0] + 4 * k);
        }
    }
    if (maniLoadU32(h + 12) != 0)
    {
        return Fail(s->at[0] + 12);
    }
    if (s->sizes[1] != (uint64_t)m->channelCount * MANI_MORPH_CHANNEL_SIZE)
    {
        return Fail(s->entry[1]);
    }
    if (s->sizes[2] != (uint64_t)m->targetCount * MANI_MORPH_TARGET_SIZE)
    {
        return Fail(s->entry[2]);
    }
    return kOk;
}

// Channels covering the targets in order, at least one each, names inside
// the name bytes and covering them in order.
static maniFileResult CheckChannels(const Sections* s, const maniMorphTargets* m)
{
    uint64_t target = 0;
    uint64_t name = 0;
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        const uint8_t* e = s->data[1] + (uint64_t)c * MANI_MORPH_CHANNEL_SIZE;
        uint64_t at = s->at[1] + (uint64_t)c * MANI_MORPH_CHANNEL_SIZE;
        if (maniLoadU32(e) != target)
        {
            return Fail(at);
        }
        uint32_t count = maniLoadU32(e + 4);
        if (count == 0 || count > m->targetCount - target)
        {
            return Fail(at + 4);
        }
        target += count;
        if (maniLoadU32(e + 8) != name)
        {
            return Fail(at + 8);
        }
        uint32_t length = maniLoadU32(e + 12);
        if (length == 0 || length > s->sizes[5] - name)
        {
            return Fail(at + 12);
        }
        name += length;
    }
    if (target != m->targetCount)
    {
        return Fail(s->at[1] + (uint64_t)(m->channelCount - 1) * MANI_MORPH_CHANNEL_SIZE + 4);
    }
    return name == s->sizes[5] ? kOk : Fail(s->entry[5]);
}

// Running totals of the indices and deltas the targets claim.
typedef struct Claimed
{
    uint64_t indices;
    uint64_t deltas;
} Claimed;

// One target's record: full weight finite, not 0, above the channel's
// previous; attributes known; a dense one covering every vertex; its
// indices and deltas following the previous target's.
static maniFileResult CheckTarget(const Sections* s, const maniMorphTargets* m, uint32_t t,
                                  float previous, Claimed* claimed)
{
    const uint8_t* e = s->data[2] + (uint64_t)t * MANI_MORPH_TARGET_SIZE;
    uint64_t at = s->at[2] + (uint64_t)t * MANI_MORPH_TARGET_SIZE;
    float full = LoadF32(e);
    if (!Finite(full) || full == 0.0f || !(full > previous))
    {
        return Fail(at);
    }
    uint32_t attributes = maniLoadU32(e + 4);
    if (attributes > 15u)
    {
        return Fail(at + 4);
    }
    uint32_t count = maniLoadU32(e + 8);
    bool dense = (attributes & maniMorphDense) != 0;
    if ((dense && count != m->vertexCount) || count > m->vertexCount)
    {
        return Fail(at + 8);
    }
    if (maniLoadU32(e + 12) != 0)
    {
        return Fail(at + 12);
    }
    if (maniLoadU64(e + 16) != claimed->indices)
    {
        return Fail(at + 16);
    }
    if (maniLoadU64(e + 24) != claimed->deltas)
    {
        return Fail(at + 24);
    }
    claimed->indices += dense ? 0 : count;
    claimed->deltas += (uint64_t)count * maniMorphFloatsPerVertex(attributes);
    return kOk;
}

static maniFileResult CheckTargets(const Sections* s, const maniMorphTargets* m)
{
    Claimed claimed = {0, 0};
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        const uint8_t* e = s->data[1] + (uint64_t)c * MANI_MORPH_CHANNEL_SIZE;
        uint32_t first = maniLoadU32(e);
        uint32_t count = maniLoadU32(e + 4);
        float previous = -INFINITY;
        for (uint32_t t = first; t < first + count; ++t)
        {
            maniFileResult r = CheckTarget(s, m, t, previous, &claimed);
            if (r.result != mani_success)
            {
                return r;
            }
            previous = LoadF32(s->data[2] + (uint64_t)t * MANI_MORPH_TARGET_SIZE);
        }
    }
    if (s->sizes[3] != 4 * claimed.indices)
    {
        return Fail(s->entry[3]);
    }
    return s->sizes[4] == 4 * claimed.deltas ? kOk : Fail(s->entry[4]);
}

// Each sparse target's indices increasing and below the vertex count;
// every delta finite.
static maniFileResult CheckValues(const Sections* s, const maniMorphTargets* m)
{
    for (uint32_t t = 0; t < m->targetCount; ++t)
    {
        const uint8_t* e = s->data[2] + (uint64_t)t * MANI_MORPH_TARGET_SIZE;
        if ((maniLoadU32(e + 4) & maniMorphDense) != 0)
        {
            continue;
        }
        uint32_t count = maniLoadU32(e + 8);
        uint64_t first = maniLoadU64(e + 16);
        uint64_t floor = 0;
        for (uint64_t i = first; i < first + count; ++i)
        {
            uint32_t v = maniLoadU32(s->data[3] + 4 * i);
            if (v < floor || v >= m->vertexCount)
            {
                return Fail(s->at[3] + 4 * i);
            }
            floor = (uint64_t)v + 1;
        }
    }
    for (uint64_t k = 0; k < s->sizes[4] / 4; ++k)
    {
        if (!Finite(LoadF32(s->data[4] + 4 * k)))
        {
            return Fail(s->at[4] + 4 * k);
        }
    }
    return kOk;
}

maniFileResult maniOpenMorphTargets(const void* bytes, size_t size, maniMorphTargets* targetsOut)
{
    if (targetsOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindMorphTargets, MANI_MORPH_FORMAT_VERSION, &c);
    Sections s;
    if (r.result == mani_success)
    {
        r = LoadSections(&c, &s);
    }
    maniMorphTargets m = {0};
    if (r.result == mani_success)
    {
        r = CheckHeader(&s, &m);
    }
    if (r.result == mani_success)
    {
        r = CheckChannels(&s, &m);
    }
    if (r.result == mani_success)
    {
        r = CheckTargets(&s, &m);
    }
    if (r.result == mani_success)
    {
        r = CheckValues(&s, &m);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniMorphComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    m.channels = s.data[1];
    m.targets = s.data[2];
    m.indices = s.data[3];
    m.deltas = s.data[4];
    m.names = s.data[5];
    m.identity = c.identity;
    *targetsOut = m;
    return kOk;
}

uint32_t maniMorphVertexCount(const maniMorphTargets* targets)
{
    return targets->vertexCount;
}

uint32_t maniMorphChannelCount(const maniMorphTargets* targets)
{
    return targets->channelCount;
}

uint32_t maniMorphTargetCount(const maniMorphTargets* targets)
{
    return targets->targetCount;
}

const char* maniMorphChannelName(const maniMorphTargets* targets, uint32_t channel,
                                 uint32_t* lengthOut)
{
    bool inside = channel < targets->channelCount;
    const uint8_t* e = targets->channels + (size_t)channel * MANI_MORPH_CHANNEL_SIZE;
    if (lengthOut != NULL)
    {
        *lengthOut = inside ? maniLoadU32(e + 12) : 0;
    }
    return inside ? (const char*)targets->names + maniLoadU32(e + 8) : NULL;
}

uint32_t maniMorphFindChannel(const maniMorphTargets* targets, const char* name, size_t length)
{
    // No channel has an empty name.
    if (name == NULL || length == 0)
    {
        return MANI_NO_CHANNEL;
    }
    for (uint32_t c = 0; c < targets->channelCount; ++c)
    {
        const uint8_t* e = targets->channels + (size_t)c * MANI_MORPH_CHANNEL_SIZE;
        if (maniLoadU32(e + 12) == length &&
            memcmp(targets->names + maniLoadU32(e + 8), name, length) == 0)
        {
            return c;
        }
    }
    return MANI_NO_CHANNEL;
}

uint32_t maniMorphChannelTargets(const maniMorphTargets* targets, uint32_t channel,
                                 uint32_t* firstOut)
{
    bool inside = channel < targets->channelCount;
    const uint8_t* e = targets->channels + (size_t)channel * MANI_MORPH_CHANNEL_SIZE;
    if (firstOut != NULL)
    {
        *firstOut = inside ? maniLoadU32(e) : 0;
    }
    return inside ? maniLoadU32(e + 4) : 0;
}

float maniMorphTargetFullWeight(const maniMorphTargets* targets, uint32_t target)
{
    if (target >= targets->targetCount)
    {
        return 0.0f;
    }
    return LoadF32(targets->targets + (size_t)target * MANI_MORPH_TARGET_SIZE);
}

// The zero shape at full weight 0, outside the channel's targets.
#define ZERO_KEY UINT32_MAX

static float Full(const maniMorphTargets* m, uint32_t first, uint32_t key)
{
    return key == ZERO_KEY ? 0.0f : maniMorphTargetFullWeight(m, first + key);
}

// The keys around a positive weight: from the zero shape (or the last
// negative key) up through the positive keys, stopping past the weight.
static void AroundPositive(const maniMorphTargets* m, uint32_t first, uint32_t count,
                           int64_t lastNegative, float weight, uint32_t* prev, uint32_t* next)
{
    *prev = lastNegative >= 0 ? (uint32_t)lastNegative : ZERO_KEY;
    *next = ZERO_KEY;
    for (uint32_t i = (uint32_t)(lastNegative + 1); i < count; ++i)
    {
        *prev = *next;
        *next = i;
        if (Full(m, first, i) > weight)
        {
            break;
        }
    }
}

// The keys around a weight of 0 or below, mirrored.
static void AroundNegative(const maniMorphTargets* m, uint32_t first, uint32_t count,
                           int64_t lastNegative, float weight, uint32_t* prev, uint32_t* next)
{
    *prev = (uint64_t)(lastNegative + 1) < count ? (uint32_t)(lastNegative + 1) : ZERO_KEY;
    *next = ZERO_KEY;
    for (int64_t i = lastNegative; i >= 0; --i)
    {
        *prev = *next;
        *next = (uint32_t)i;
        if (Full(m, first, (uint32_t)i) < weight)
        {
            break;
        }
    }
}

// One channel's walk (ufbx's ufbxi_update_blend_channel): the two keys
// around the weight on its side of 0, the zero shape among them, and the
// weight between them. Keys strictly increase, so two real keys differ.
static void Walk(const maniMorphTargets* m, uint32_t first, uint32_t count, float weight,
                 float* out)
{
    int64_t lastNegative = -1;
    for (uint32_t i = 0; i < count; ++i)
    {
        out[i] = 0.0f;
        lastNegative = Full(m, first, i) < 0.0f ? (int64_t)i : lastNegative;
    }
    uint32_t prev = ZERO_KEY;
    uint32_t next = ZERO_KEY;
    if (weight > 0.0f)
    {
        AroundPositive(m, first, count, lastNegative, weight, &prev, &next);
    }
    else
    {
        AroundNegative(m, first, count, lastNegative, weight, &prev, &next);
    }
    // With the zero shape at one end, the other key's weight is the
    // weight over its full weight, ufbx's value without the rounding of
    // 1 - t (so a key at 1 passes any weight through exactly).
    if (prev == ZERO_KEY || next == ZERO_KEY)
    {
        uint32_t key = prev == ZERO_KEY ? next : prev;
        if (key != ZERO_KEY)
        {
            out[key] = weight / Full(m, first, key);
        }
        return;
    }
    float low = Full(m, first, prev);
    float t = (weight - low) / (Full(m, first, next) - low);
    out[prev] = 1.0f - t;
    out[next] = t;
}

maniResult maniMorphTargetWeights(const maniMorphTargets* targets, const float* channelWeights,
                                  float* targetWeights)
{
    if (targets == NULL || channelWeights == NULL || targetWeights == NULL)
    {
        return mani_errorInvalid;
    }
    for (uint32_t c = 0; c < targets->channelCount; ++c)
    {
        if (!Finite(channelWeights[c]))
        {
            return mani_errorInvalid;
        }
    }
    for (uint32_t c = 0; c < targets->channelCount; ++c)
    {
        uint32_t first = 0;
        uint32_t count = maniMorphChannelTargets(targets, c, &first);
        Walk(targets, first, count, channelWeights[c], targetWeights + first);
    }
    return mani_success;
}

// A stream the job takes: pointers, a kind with deltas, strides wide
// enough.
static bool StreamValid(const maniSkinStream* s)
{
    size_t width = s->kind == maniStreamTangent ? 16 : 12;
    bool kind =
        s->kind == maniStreamPoint || s->kind == maniStreamNormal || s->kind == maniStreamTangent;
    return s->in != NULL && s->out != NULL && kind && s->inStride >= width && s->outStride >= width;
}

static bool JobValid(const maniMorphJob* job)
{
    if (job == NULL || job->targets == NULL || job->weights == NULL ||
        (job->streams == NULL && job->streamCount > 0) || job->first > job->targets->vertexCount ||
        job->count > job->targets->vertexCount - job->first)
    {
        return false;
    }
    for (uint32_t t = 0; t < job->targets->targetCount; ++t)
    {
        if (!Finite(job->weights[t]))
        {
            return false;
        }
    }
    for (uint32_t i = 0; i < job->streamCount; ++i)
    {
        if (!StreamValid(&job->streams[i]))
        {
            return false;
        }
    }
    return true;
}

// The attribute a stream kind takes.
static uint32_t Attribute(maniStreamKind kind)
{
    return kind == maniStreamPoint
               ? maniMorphPositions
               : (kind == maniStreamNormal ? maniMorphNormals : maniMorphTangents);
}

// A target's slice of its deltas for one attribute: the floats for its
// vertices, three a vertex.
static const uint8_t* AttributeDeltas(const maniMorphTargets* m, const uint8_t* e,
                                      uint32_t attribute)
{
    uint32_t attributes = maniLoadU32(e + 4);
    uint32_t count = maniLoadU32(e + 8);
    uint64_t at = maniLoadU64(e + 24);
    for (uint32_t a = maniMorphPositions; a < attribute; a <<= 1)
    {
        at += (attributes & a) != 0 ? 3 * (uint64_t)count : 0;
    }
    return m->deltas + 4 * at;
}

// The first of a sparse target's indices at or past vertex.
static uint32_t LowerBound(const uint8_t* indices, uint32_t count, uint32_t vertex)
{
    uint32_t low = 0;
    uint32_t high = count;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (maniLoadU32(indices + 4 * (size_t)mid) < vertex)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    return low;
}

static void AddDelta(const maniSkinStream* s, uint32_t i, const uint8_t* delta, float weight)
{
    uint8_t* out = (uint8_t*)s->out + (size_t)i * s->outStride;
    for (int k = 0; k < 3; ++k)
    {
        float x = LoadF32(out + 4 * k) + LoadF32(delta + 4 * k) * weight;
        memcpy(out + 4 * k, &x, sizeof x);
    }
}

// One target's deltas onto one stream over the job's range.
static void AddTarget(const maniMorphJob* job, uint32_t t, const maniSkinStream* s)
{
    const maniMorphTargets* m = job->targets;
    const uint8_t* e = m->targets + (size_t)t * MANI_MORPH_TARGET_SIZE;
    uint32_t attribute = Attribute(s->kind);
    if ((maniLoadU32(e + 4) & attribute) == 0)
    {
        return;
    }
    const uint8_t* deltas = AttributeDeltas(m, e, attribute);
    float w = job->weights[t];
    uint32_t end = job->first + job->count;
    if ((maniLoadU32(e + 4) & maniMorphDense) != 0)
    {
        for (uint32_t v = job->first; v < end; ++v)
        {
            AddDelta(s, v - job->first, deltas + 12 * (size_t)v, w);
        }
        return;
    }
    const uint8_t* indices = m->indices + 4 * maniLoadU64(e + 16);
    uint32_t count = maniLoadU32(e + 8);
    for (uint32_t i = LowerBound(indices, count, job->first); i < count; ++i)
    {
        uint32_t v = maniLoadU32(indices + 4 * (size_t)i);
        if (v >= end)
        {
            break;
        }
        AddDelta(s, v - job->first, deltas + 12 * (size_t)i, w);
    }
}

// x, y, z to unit length; zero when the length is 0 or past binary32's
// range.
static void Renormalize(uint8_t* p)
{
    float v[3];
    memcpy(v, p, sizeof v);
    float squared = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    bool usable = squared > 0.0f && squared <= FLT_MAX;
    float inverse = usable ? 1.0f / sqrtf(squared) : 0.0f;
    for (int k = 0; k < 3; ++k)
    {
        v[k] = usable ? v[k] * inverse : 0.0f;
    }
    memcpy(p, v, sizeof v);
}

maniResult maniApplyMorphTargets(const maniMorphJob* job)
{
    if (!JobValid(job))
    {
        return mani_errorInvalid;
    }
    for (uint32_t k = 0; k < job->streamCount; ++k)
    {
        const maniSkinStream* s = &job->streams[k];
        size_t width = s->kind == maniStreamTangent ? 16 : 12;
        for (uint32_t i = 0; i < job->count && s->in != s->out; ++i)
        {
            memmove((uint8_t*)s->out + (size_t)i * s->outStride,
                    (const uint8_t*)s->in + (size_t)i * s->inStride, width);
        }
        for (uint32_t t = 0; t < job->targets->targetCount; ++t)
        {
            if (job->weights[t] != 0.0f)
            {
                AddTarget(job, t, s);
            }
        }
        for (uint32_t i = 0; i < job->count && s->kind != maniStreamPoint; ++i)
        {
            Renormalize((uint8_t*)s->out + (size_t)i * s->outStride);
        }
    }
    return mani_success;
}
