// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Morph targets: opening the file (the attribute table and every count,
// offset, index and delta checked), the view's getters, and the channel
// weights' walk through their targets (ufbx's rule for FBX in-betweens).
// The job is in morph_apply.c.

#include "maul-anim/morph.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "invariant.h"
#include "morph_file.h"
#include "morph_view.h"

#include "maul-anim/base.h"

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

MANI_NODISCARD static bool Finite(float value)
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
        // The container keeps tags from 1 to the last.
        MANI_ASSERT(tag >= 1 && tag <= MANI_MORPH_SECTIONS);
        uint64_t size = maniLoadU64(entry + 16);
        // The container refuses an empty entry: an empty section is left out.
        MANI_ASSERT(size > 0);
        s->data[tag - 1] = c->bytes + maniLoadU64(entry + 8);
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = maniLoadU64(entry + 8);
        s->entry[tag - 1] = entryAt + 16;
    }
    return kOk;
}

// Section indices (tag - 1).
enum
{
    kHeader = maniMorphTagHeader - 1,
    kAttributes = maniMorphTagAttributes - 1,
    kChannels = maniMorphTagChannels - 1,
    kTargets = maniMorphTagTargets - 1,
    kIndices = maniMorphTagIndices - 1,
    kDeltas = maniMorphTagDeltas - 1,
    kNames = maniMorphTagNames - 1,
};

// The header's counts and the fixed-size sections against them.
static maniFileResult CheckHeader(const Sections* s, maniMorphTargets* m)
{
    if (s->sizes[kHeader] != MANI_MORPH_HEADER_SIZE)
    {
        return Fail(s->entry[kHeader]);
    }
    const uint8_t* h = s->data[kHeader];
    m->vertexCount = maniLoadU32(h);
    m->channelCount = maniLoadU32(h + 4);
    m->targetCount = maniLoadU32(h + 8);
    m->attributeCount = maniLoadU32(h + 12);
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (maniLoadU32(h + 4 * k) == 0)
        {
            return Fail(s->at[kHeader] + 4 * k);
        }
    }
    if (m->attributeCount > MANI_MAX_MORPH_ATTRIBUTES)
    {
        return Fail(s->at[kHeader] + 12);
    }
    const uint64_t want[3] = {(uint64_t)m->attributeCount * MANI_MORPH_ATTRIBUTE_SIZE,
                              (uint64_t)m->channelCount * MANI_MORPH_CHANNEL_SIZE,
                              (uint64_t)m->targetCount * MANI_MORPH_TARGET_SIZE};
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (s->sizes[kAttributes + k] != want[k])
        {
            return Fail(s->entry[kAttributes + k]);
        }
    }
    return kOk;
}

// The attribute table: known semantics, set 0 for the first three, none
// repeated; their widths out.
static maniFileResult CheckAttributes(const Sections* s, const maniMorphTargets* m,
                                      uint32_t widths[MANI_MAX_MORPH_ATTRIBUTES])
{
    for (uint32_t a = 0; a < MANI_MAX_MORPH_ATTRIBUTES; ++a)
    {
        widths[a] = 0;
    }
    for (uint32_t a = 0; a < m->attributeCount; ++a)
    {
        const uint8_t* e = s->data[kAttributes] + (uint64_t)a * MANI_MORPH_ATTRIBUTE_SIZE;
        uint64_t at = s->at[kAttributes] + (uint64_t)a * MANI_MORPH_ATTRIBUTE_SIZE;
        uint32_t semantic = maniLoadU32(e);
        uint32_t set = maniLoadU32(e + 4);
        if (maniMorphWidth(semantic) == 0)
        {
            return Fail(at);
        }
        if ((uint32_t)semantic <= maniMorphTangent && set != 0)
        {
            return Fail(at + 4);
        }
        for (uint32_t b = 0; b < a; ++b)
        {
            const uint8_t* f = s->data[kAttributes] + (uint64_t)b * MANI_MORPH_ATTRIBUTE_SIZE;
            if (maniLoadU32(f) == semantic && maniLoadU32(f + 4) == set)
            {
                return Fail(at);
            }
        }
        widths[a] = maniMorphWidth(semantic);
    }
    return kOk;
}

// Channels covering the targets in order, at least one each, names inside
// the name bytes in order from the first; the bytes their names take out.
static maniFileResult CheckChannels(const Sections* s, const maniMorphTargets* m,
                                    uint64_t* namesOut)
{
    uint64_t target = 0;
    uint64_t name = 0;
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        const uint8_t* e = s->data[kChannels] + (uint64_t)c * MANI_MORPH_CHANNEL_SIZE;
        uint64_t at = s->at[kChannels] + (uint64_t)c * MANI_MORPH_CHANNEL_SIZE;
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
        if (length == 0 || length > s->sizes[kNames] - name)
        {
            return Fail(at + 12);
        }
        name += length;
    }
    if (target != m->targetCount)
    {
        return Fail(s->at[kChannels] + (uint64_t)(m->channelCount - 1) * MANI_MORPH_CHANNEL_SIZE +
                    4);
    }
    *namesOut = name;
    return kOk;
}

// Running totals of the indices, deltas and name bytes the targets claim.
typedef struct Claimed
{
    uint64_t indices;
    uint64_t deltas;
    uint64_t names;
} Claimed;

// One target's record: full weight finite, not 0, above the channel's
// previous; attributes in the table; a dense one covering every vertex;
// its indices and deltas following the previous target's.
static maniFileResult CheckTarget(const Sections* s, const maniMorphTargets* m, uint32_t t,
                                  float previous, const uint32_t* widths, Claimed* claimed)
{
    const uint8_t* e = s->data[kTargets] + (uint64_t)t * MANI_MORPH_TARGET_SIZE;
    uint64_t at = s->at[kTargets] + (uint64_t)t * MANI_MORPH_TARGET_SIZE;
    float full = LoadF32(e);
    if (!Finite(full) || full == 0.0f || !(full > previous))
    {
        return Fail(at);
    }
    uint32_t mask = maniLoadU32(e + 4);
    if (m->attributeCount < 32 && (mask >> m->attributeCount) != 0)
    {
        return Fail(at + 4);
    }
    uint32_t count = maniLoadU32(e + 8);
    uint32_t flags = maniLoadU32(e + 12);
    bool dense = flags == MANI_MORPH_DENSE;
    if ((dense && count != m->vertexCount) || count > m->vertexCount)
    {
        return Fail(at + 8);
    }
    if (flags > MANI_MORPH_DENSE)
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
    if (maniLoadU32(e + 32) != claimed->names)
    {
        return Fail(at + 32);
    }
    uint32_t length = maniLoadU32(e + 36);
    if (length == 0 || length > s->sizes[kNames] - claimed->names)
    {
        return Fail(at + 36);
    }
    claimed->names += length;
    claimed->indices += dense ? 0 : count;
    claimed->deltas += (uint64_t)count * maniMorphFloatsPerVertex(widths, mask);
    return kOk;
}

static maniFileResult CheckTargets(const Sections* s, const maniMorphTargets* m,
                                   const uint32_t* widths, uint64_t names)
{
    Claimed claimed = {0, 0, names};
    for (uint32_t c = 0; c < m->channelCount; ++c)
    {
        const uint8_t* e = s->data[kChannels] + (uint64_t)c * MANI_MORPH_CHANNEL_SIZE;
        uint32_t first = maniLoadU32(e);
        uint32_t count = maniLoadU32(e + 4);
        float previous = -INFINITY;
        for (uint32_t t = first; t < first + count; ++t)
        {
            maniFileResult r = CheckTarget(s, m, t, previous, widths, &claimed);
            if (r.result != mani_success)
            {
                return r;
            }
            previous = LoadF32(s->data[kTargets] + (uint64_t)t * MANI_MORPH_TARGET_SIZE);
        }
    }
    if (claimed.names != s->sizes[kNames])
    {
        return Fail(s->entry[kNames]);
    }
    if (s->sizes[kIndices] != 4 * claimed.indices)
    {
        return Fail(s->entry[kIndices]);
    }
    return s->sizes[kDeltas] == 4 * claimed.deltas ? kOk : Fail(s->entry[kDeltas]);
}

// Each sparse target's indices increasing and below the vertex count;
// every delta finite.
static maniFileResult CheckValues(const Sections* s, const maniMorphTargets* m)
{
    for (uint32_t t = 0; t < m->targetCount; ++t)
    {
        const uint8_t* e = s->data[kTargets] + (uint64_t)t * MANI_MORPH_TARGET_SIZE;
        if (maniLoadU32(e + 12) == MANI_MORPH_DENSE)
        {
            continue;
        }
        uint32_t count = maniLoadU32(e + 8);
        uint64_t first = maniLoadU64(e + 16);
        uint64_t floor = 0;
        for (uint64_t i = first; i < first + count; ++i)
        {
            uint32_t v = maniLoadU32(s->data[kIndices] + 4 * i);
            if (v < floor || v >= m->vertexCount)
            {
                return Fail(s->at[kIndices] + 4 * i);
            }
            floor = (uint64_t)v + 1;
        }
    }
    for (uint64_t k = 0; k < s->sizes[kDeltas] / 4; ++k)
    {
        if (!Finite(LoadF32(s->data[kDeltas] + 4 * k)))
        {
            return Fail(s->at[kDeltas] + 4 * k);
        }
    }
    return kOk;
}

static maniFileResult CheckAll(const Sections* s, maniMorphTargets* m)
{
    uint32_t widths[MANI_MAX_MORPH_ATTRIBUTES];
    uint64_t names = 0;
    maniFileResult r = CheckHeader(s, m);
    if (r.result == mani_success)
    {
        r = CheckAttributes(s, m, widths);
    }
    if (r.result == mani_success)
    {
        r = CheckChannels(s, m, &names);
    }
    if (r.result == mani_success)
    {
        r = CheckTargets(s, m, widths, names);
    }
    if (r.result == mani_success)
    {
        r = CheckValues(s, m);
    }
    return r;
}

static MANI_NOINLINE maniFileResult OpenMorphTargetsWork(const void* bytes, size_t size, void* out)
{
    maniMorphTargets* targetsOut = out;
    if (targetsOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindMorphTargets,
                                         MANI_MORPH_FORMAT_VERSION, MANI_MORPH_SECTIONS, &c);
    Sections s;
    if (r.result == mani_success)
    {
        r = LoadSections(&c, &s);
    }
    maniMorphTargets m = {0};
    if (r.result == mani_success)
    {
        r = CheckAll(&s, &m);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniMorphComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    m.base = c.bytes;
    m.size = c.size;
    for (uint32_t i = kAttributes; i <= kNames; ++i)
    {
        m.offsets[i - kAttributes] = s.sizes[i] > 0 ? s.at[i] : 0;
        m.sizes[i - kAttributes] = s.sizes[i];
    }
    m.identity = c.identity;
    m.cookie = MANI_VIEW_COOKIE(maniKindMorphTargets);
    *targetsOut = m;
    return kOk;
}

maniFileResult maniOpenMorphTargets(const void* bytes, size_t size, maniMorphTargets* targetsOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result = maniContainerOpenTagged(bytes, size, targetsOut, OpenMorphTargetsWork);
    maniFloatEnvLeave(env);
    return result;
}

MANI_NOINLINE bool maniMorphTargetsValid(const maniMorphTargets* m)
{
    if (m == NULL || m->cookie != MANI_VIEW_COOKIE(maniKindMorphTargets) || m->base == NULL ||
        m->attributeCount > MANI_MAX_MORPH_ATTRIBUTES)
    {
        return false;
    }
    // The fixed sections hold their counts' entries; the indices, deltas
    // and names are read through bounds where they are used.
    const uint64_t need[6] = {(uint64_t)m->attributeCount * MANI_MORPH_ATTRIBUTE_SIZE,
                              (uint64_t)m->channelCount * MANI_MORPH_CHANNEL_SIZE,
                              (uint64_t)m->targetCount * MANI_MORPH_TARGET_SIZE,
                              0,
                              0,
                              0};
    bool fits = true;
    for (uint32_t i = 0; i < 6; ++i)
    {
        fits = fits && maniViewFits(m->offsets[i], m->sizes[i], m->size) && need[i] <= m->sizes[i];
    }
    return fits;
}

uint32_t maniMorphTargets_GetVertexCount(const maniMorphTargets* targets)
{
    return maniMorphTargetsValid(targets) ? targets->vertexCount : 0;
}

uint32_t maniMorphTargets_GetChannelCount(const maniMorphTargets* targets)
{
    return maniMorphTargetsValid(targets) ? targets->channelCount : 0;
}

uint32_t maniMorphTargets_GetTargetCount(const maniMorphTargets* targets)
{
    return maniMorphTargetsValid(targets) ? targets->targetCount : 0;
}

uint32_t maniMorphTargets_GetAttributeCount(const maniMorphTargets* targets)
{
    return maniMorphTargetsValid(targets) ? targets->attributeCount : 0;
}

maniMorphSemantic maniMorphTargets_GetAttribute(const maniMorphTargets* targets, uint32_t attribute,
                                                uint32_t* setOut)
{
    bool valid = maniMorphTargetsValid(targets);
    if (!valid && setOut != NULL)
    {
        *setOut = 0;
    }
    return valid ? maniMorphAttribute(targets, attribute, setOut) : -1;
}

// A name by its span (offset, then length) in the names: NULL with length
// 0 for a span in bytes changed since the open that falls outside them.
static const char* Name(const maniMorphTargets* m, const uint8_t* span, size_t* lengthOut)
{
    uint64_t start = maniLoadU32(span);
    uint64_t length = maniLoadU32(span + 4);
    bool fits = maniViewFits(start, length, maniMorphSectionSize(m, maniMorphTagNames));
    if (lengthOut != NULL)
    {
        *lengthOut = fits ? (size_t)length : 0;
    }
    return fits ? (const char*)maniMorphSection(m, maniMorphTagNames) + start : NULL;
}

static const uint8_t* Channel(const maniMorphTargets* m, uint32_t channel)
{
    return maniMorphSection(m, maniMorphTagChannels) + (size_t)channel * MANI_MORPH_CHANNEL_SIZE;
}

const char* maniMorphTargets_GetChannelName(const maniMorphTargets* targets, uint32_t channel,
                                            size_t* lengthOut)
{
    if (!maniMorphTargetsValid(targets) || channel >= targets->channelCount)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return Name(targets, Channel(targets, channel) + 8, lengthOut);
}

// The first of count entries whose name (its span at `span` bytes in) is
// the one given, or MANI_NO_CHANNEL.
static uint32_t FindName(const maniMorphTargets* m, const uint8_t* entries, size_t entrySize,
                         uint32_t count, size_t span, const char* name, size_t length)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        size_t found = 0;
        const char* bytes = Name(m, entries + i * entrySize + span, &found);
        if (bytes != NULL && found == length && memcmp(bytes, name, length) == 0)
        {
            return i;
        }
    }
    return MANI_NO_CHANNEL;
}

uint32_t maniMorphTargets_FindChannel(const maniMorphTargets* targets, const char* name,
                                      size_t length)
{
    // No channel has an empty name.
    if (!maniMorphTargetsValid(targets) || name == NULL || length == 0)
    {
        return MANI_NO_CHANNEL;
    }
    return FindName(targets, Channel(targets, 0), MANI_MORPH_CHANNEL_SIZE, targets->channelCount, 8,
                    name, length);
}

const char* maniMorphTargets_GetTargetName(const maniMorphTargets* targets, uint32_t target,
                                           size_t* lengthOut)
{
    if (!maniMorphTargetsValid(targets) || target >= targets->targetCount)
    {
        if (lengthOut != NULL)
        {
            *lengthOut = 0;
        }
        return NULL;
    }
    return Name(targets, maniMorphTargetEntry(targets, target) + 32, lengthOut);
}

uint32_t maniMorphTargets_FindTarget(const maniMorphTargets* targets, const char* name,
                                     size_t length)
{
    // No target has an empty name.
    if (!maniMorphTargetsValid(targets) || name == NULL || length == 0)
    {
        return MANI_NO_CHANNEL;
    }
    return FindName(targets, maniMorphTargetEntry(targets, 0), MANI_MORPH_TARGET_SIZE,
                    targets->targetCount, 32, name, length);
}

// A channel's targets, the view checked: none for a range in bytes
// changed since the open that is not within the targets.
static uint32_t ChannelTargets(const maniMorphTargets* m, uint32_t channel, uint32_t* firstOut)
{
    if (channel >= m->channelCount)
    {
        if (firstOut != NULL)
        {
            *firstOut = 0;
        }
        return 0;
    }
    const uint8_t* e = Channel(m, channel);
    uint32_t first = maniLoadU32(e);
    uint32_t count = maniLoadU32(e + 4);
    bool fits = first <= m->targetCount && count <= m->targetCount - first;
    if (firstOut != NULL)
    {
        *firstOut = fits ? first : 0;
    }
    return fits ? count : 0;
}

uint32_t maniMorphTargets_GetChannelTargets(const maniMorphTargets* targets, uint32_t channel,
                                            uint32_t* firstOut)
{
    if (!maniMorphTargetsValid(targets))
    {
        if (firstOut != NULL)
        {
            *firstOut = 0;
        }
        return 0;
    }
    return ChannelTargets(targets, channel, firstOut);
}

// A target's full weight, the view checked.
static float FullWeight(const maniMorphTargets* m, uint32_t target)
{
    return target < m->targetCount ? LoadF32(maniMorphTargetEntry(m, target)) : 0.0f;
}

float maniMorphTargets_GetTargetFullWeight(const maniMorphTargets* targets, uint32_t target)
{
    return maniMorphTargetsValid(targets) ? FullWeight(targets, target) : 0.0f;
}

// The zero shape at full weight 0, outside the channel's targets.
#define ZERO_KEY UINT32_MAX

static float Full(const maniMorphTargets* m, uint32_t first, uint32_t key)
{
    return key == ZERO_KEY ? 0.0f : FullWeight(m, first + key);
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
            // Opening refused a full weight of 0; bytes changed since may
            // hold one, which gives some weight.
            MANI_ASSERT(key < count);
            out[key] = weight / Full(m, first, key);
        }
        return;
    }
    float low = Full(m, first, prev);
    MANI_ASSERT(prev < count && next < count);
    float t = (weight - low) / (Full(m, first, next) - low);
    out[prev] = 1.0f - t;
    out[next] = t;
}

static MANI_NOINLINE maniResult MorphTargetWeightsWork(const maniMorphTargets* targets,
                                                       const float* channelWeights,
                                                       float* targetWeights, uint32_t capacity,
                                                       uint32_t* countOut)
{
    if (!maniMorphTargetsValid(targets) || channelWeights == NULL ||
        (targetWeights == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = targets->targetCount;
    }
    if (targetWeights == NULL || capacity < targets->targetCount)
    {
        return mani_errorCapacity;
    }
    for (uint32_t c = 0; c < targets->channelCount; ++c)
    {
        if (!Finite(channelWeights[c]))
        {
            return mani_errorInvalid;
        }
    }
    // Last channel first: a channel's targets start at or past its own
    // index, so in place (targetWeights the channel weights) each channel
    // writes only weights already read.
    for (uint32_t c = targets->channelCount; c-- > 0;)
    {
        uint32_t first = 0;
        // A channel whose targets in bytes changed since the open are not
        // within the targets has none, and writes nothing.
        uint32_t count = ChannelTargets(targets, c, &first);
        Walk(targets, first, count, channelWeights[c], targetWeights + first);
    }
    return mani_success;
}

maniResult maniMorphTargets_ComputeTargetWeights(const maniMorphTargets* targets,
                                                 const float* channelWeights,
                                                 float* targetWeightsOut, uint32_t capacity,
                                                 uint32_t* countOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result =
        MorphTargetWeightsWork(targets, channelWeights, targetWeightsOut, capacity, countOut);
    maniFloatEnvLeave(env);
    return result;
}
