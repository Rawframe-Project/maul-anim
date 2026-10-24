// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A motion database file's view: every section checked once
// as hostile input, the search tree's order checked to place every frame
// once with its entry and its boxes recomputed and compared bit for bit
// (the search's exactness rests on them), then read in place.

#include "maul-anim/match.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "match_file.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint32_t maniMatchChannelWidth(uint32_t kind)
{
    static const uint32_t kWidths[6] = {3, 3, 3, 2, 2, 1};
    return kind < 6 ? kWidths[kind] : 0u;
}

float maniMatchFrameTime(float start, uint32_t k, float rate)
{
    return (float)((double)start + (double)k / (double)rate);
}

static bool Within(float time, float end, bool loop)
{
    return loop ? time < end : time <= end;
}

uint32_t maniMatchEntryFrames(float start, float end, float rate, bool loop)
{
    double span = ((double)end - (double)start) * (double)rate;
    if (!(span >= 0.0 && span < (double)MANI_MATCH_MAX_FRAMES))
    {
        return UINT32_MAX;
    }
    // The binary64 count, then one frame less when the last's binary32
    // time does not hold, or one more when the next's does: once only, as
    // far past 2^24 seconds a step no longer moves the time.
    uint32_t n = (uint32_t)floor(span) + 1u;
    if (n > 1 && !Within(maniMatchFrameTime(start, n - 1, rate), end, loop))
    {
        return n - 1;
    }
    return Within(maniMatchFrameTime(start, n, rate), end, loop) ? n + 1 : n;
}

uint32_t maniMatchNodeCount(uint32_t frames, uint32_t leaf)
{
    maniMatchWalk walk;
    maniMatchVisit v;
    maniMatchWalkStart(&walk, frames, leaf);
    while (maniMatchWalkNext(&walk, &v))
    {
    }
    return walk.next;
}

uint64_t maniMatchComputeIdentity(const uint8_t* const data[MANI_MATCH_SECTIONS],
                                  const uint64_t sizes[MANI_MATCH_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_MATCH_SECTIONS; ++i)
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

static uint32_t Bits(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

typedef struct Sections
{
    const uint8_t* data[MANI_MATCH_SECTIONS];
    uint64_t sizes[MANI_MATCH_SECTIONS];
    uint64_t at[MANI_MATCH_SECTIONS];
    // Where each one's size is in the table; the section count for an
    // absent one.
    uint64_t entry[MANI_MATCH_SECTIONS];
} Sections;

static const uint8_t kEmpty = 0;

static maniFileResult LoadSections(const maniContainer* c, Sections* s)
{
    for (uint32_t i = 0; i < MANI_MATCH_SECTIONS; ++i)
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
        if (tag == 0 || tag > MANI_MATCH_SECTIONS)
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

// The header's counts within their limits, the rate, the axes on two
// different lines, the leaf size and the tree's node count.
static maniFileResult CheckHeader(const Sections* s, maniMotionDatabase* d)
{
    if (s->sizes[0] != MANI_MATCH_HEADER_SIZE)
    {
        return Fail(s->entry[0]);
    }
    const uint8_t* h = s->data[0];
    uint64_t at = s->at[0];
    uint32_t counts[7];
    // Bones, frames, features, channels, times, entries, clips.
    const uint32_t highs[7] = {MANI_MAX_BONES,
                               MANI_MATCH_MAX_FRAMES,
                               MANI_MATCH_MAX_FEATURES,
                               MANI_MATCH_MAX_CHANNELS,
                               MANI_MATCH_MAX_CHANNELS * MANI_MATCH_MAX_TIMES,
                               MANI_MATCH_MAX_ENTRIES,
                               MANI_MATCH_MAX_CLIPS};
    for (uint32_t k = 0; k < 7; ++k)
    {
        counts[k] = maniLoadU32(h + 8 + 4 * k);
        if (counts[k] < 1 || counts[k] > highs[k])
        {
            return Fail(at + 8 + 4 * k);
        }
    }
    float rate = LoadF32(h + 36);
    uint32_t up = maniLoadU32(h + 40);
    uint32_t forward = maniLoadU32(h + 44);
    if (!(rate > 0.0f && rate <= MANI_MATCH_MAX_RATE))
    {
        return Fail(at + 36);
    }
    if (up > maniMatchAxisNegativeZ)
    {
        return Fail(at + 40);
    }
    if (forward > maniMatchAxisNegativeZ || forward % 3 == up % 3)
    {
        return Fail(at + 44);
    }
    uint32_t leaf = maniLoadU32(h + 48);
    if (!maniMatchLeafValid(leaf))
    {
        return Fail(at + 48);
    }
    uint32_t nodes = maniLoadU32(h + 52);
    if (nodes != maniMatchNodeCount(counts[1], leaf))
    {
        return Fail(at + 52);
    }
    *d = (maniMotionDatabase){.skeletonIdentity = maniLoadU64(h),
                              .skeletonBoneCount = counts[0],
                              .frameCount = counts[1],
                              .featureCount = counts[2],
                              .channelCount = counts[3],
                              .timeCount = counts[4],
                              .entryCount = counts[5],
                              .clipCount = counts[6],
                              .sampleRate = rate,
                              .up = up,
                              .forward = forward,
                              .leafFrames = leaf,
                              .nodeCount = nodes};
    return kOk;
}

// Every section but the names and the mirror of the size the header's
// counts give.
static maniFileResult CheckSizes(const Sections* s, maniMotionDatabase* d)
{
    uint64_t features = d->featureCount;
    uint64_t frames = d->frameCount;
    uint64_t nodes = d->nodeCount;
    // By tag: the header is checked, the names and mirror below.
    const uint64_t sizes[MANI_MATCH_SECTIONS] = {MANI_MATCH_HEADER_SIZE,
                                                 (uint64_t)d->channelCount *
                                                     MANI_MATCH_CHANNEL_SIZE,
                                                 4 * (uint64_t)d->timeCount,
                                                 8 * (uint64_t)d->clipCount,
                                                 (uint64_t)d->entryCount * MANI_MATCH_ENTRY_SIZE,
                                                 8 * features,
                                                 4 * features * frames,
                                                 8 * features * nodes,
                                                 s->sizes[8],
                                                 s->sizes[9],
                                                 nodes * MANI_MATCH_NODE_SIZE,
                                                 frames * MANI_MATCH_POSITION_SIZE,
                                                 4 * frames};
    for (uint32_t k = 1; k < MANI_MATCH_SECTIONS; ++k)
    {
        if (s->sizes[k] != sizes[k])
        {
            return Fail(s->entry[k]);
        }
    }
    d->channels = s->data[1];
    d->times = s->data[2];
    d->clips = s->data[3];
    d->entries = s->data[4];
    d->normalization = s->data[5];
    d->features = s->data[6];
    d->boxes = s->data[7];
    d->names = s->data[8];
    d->nodes = s->data[10];
    d->positions = s->data[11];
    d->places = s->data[12];
    d->namesSize = s->sizes[8];
    // The mirror map's identity, not 0, or nothing.
    if (s->sizes[9] != 0 && s->sizes[9] != 8)
    {
        return Fail(s->entry[9]);
    }
    d->mirrorMapIdentity = s->sizes[9] != 0 ? maniLoadU64(s->data[9]) : 0;
    if (s->sizes[9] != 0 && d->mirrorMapIdentity == 0)
    {
        return Fail(s->at[9]);
    }
    return kOk;
}

// A float track's name in the names section, UTF-8; any other kind's
// offset and length 0.
static bool NameValid(const maniMotionDatabase* d, uint32_t kind, const uint8_t* e)
{
    uint32_t offset = maniLoadU32(e + 32);
    uint32_t length = maniLoadU32(e + 36);
    if (kind != maniMatchFloatTrack)
    {
        return offset == 0 && length == 0;
    }
    return length > 0 && offset <= d->namesSize && length <= d->namesSize - offset &&
           maniValidUtf8(d->names + offset, length);
}

// One channel's words in order (kind, bone, axis, times, first time,
// first feature, features, weight, name), the first wrong one's place.
static uint32_t ChannelWrong(const maniMotionDatabase* d, const uint8_t* e, uint32_t timeAt,
                             uint32_t featureAt)
{
    uint32_t kind = maniLoadU32(e);
    uint32_t times = maniLoadU32(e + 12);
    bool bone = kind <= maniMatchBoneFacing;
    float weight = LoadF32(e + 28);
    const bool valid[9] = {
        kind <= maniMatchFloatTrack,
        bone ? maniLoadU32(e + 4) < d->skeletonBoneCount : maniLoadU32(e + 4) == 0,
        kind == maniMatchBoneFacing ? maniLoadU32(e + 8) <= maniMatchAxisNegativeZ
                                    : maniLoadU32(e + 8) == 0,
        times >= 1 && times <= MANI_MATCH_MAX_TIMES,
        maniLoadU32(e + 16) == timeAt,
        maniLoadU32(e + 20) == featureAt,
        maniLoadU32(e + 24) == maniMatchChannelWidth(kind) * times,
        weight > 0.0f && weight < HUGE_VALF,
        NameValid(d, kind, e)};
    for (uint32_t k = 0; k < 9; ++k)
    {
        if (!valid[k])
        {
            return 4 * k;
        }
    }
    return UINT32_MAX;
}

static maniFileResult CheckChannels(const maniMotionDatabase* d, const Sections* s)
{
    uint32_t timeAt = 0;
    uint32_t featureAt = 0;
    for (uint32_t c = 0; c < d->channelCount; ++c)
    {
        const uint8_t* e = d->channels + (size_t)c * MANI_MATCH_CHANNEL_SIZE;
        uint64_t here = s->at[1] + (uint64_t)c * MANI_MATCH_CHANNEL_SIZE;
        uint32_t wrong = ChannelWrong(d, e, timeAt, featureAt);
        if (wrong != UINT32_MAX)
        {
            return Fail(here + wrong);
        }
        timeAt += maniLoadU32(e + 12);
        featureAt += maniLoadU32(e + 24);
        if (timeAt > d->timeCount || featureAt > d->featureCount)
        {
            return Fail(here + (timeAt > d->timeCount ? 12 : 24));
        }
    }
    if (timeAt != d->timeCount || featureAt != d->featureCount)
    {
        return Fail(s->at[0] + (timeAt != d->timeCount ? 24 : 16));
    }
    for (uint32_t t = 0; t < d->timeCount; ++t)
    {
        float time = LoadF32(d->times + 4 * (size_t)t);
        if (!(fabsf(time) <= MANI_MATCH_MAX_REACH))
        {
            return Fail(s->at[2] + 4 * (uint64_t)t);
        }
    }
    return kOk;
}

// One entry's words in order (clip, flags, start, end, bias, tags, first
// frame, frames), the first wrong one's place.
static uint32_t EntryWrong(const maniMotionDatabase* d, const uint8_t* e, uint32_t frameAt)
{
    uint32_t flags = maniLoadU32(e + 4);
    float start = LoadF32(e + 8);
    float end = LoadF32(e + 12);
    bool loop = (flags & MANI_MATCH_LOOP) != 0;
    bool mirrored = (flags & MANI_MATCH_MIRRORED) != 0;
    bool span = start >= 0.0f && start < HUGE_VALF;
    const bool valid[8] = {
        maniLoadU32(e) < d->clipCount,
        flags <= MANI_MATCH_ENTRY_FLAGS && (!mirrored || d->mirrorMapIdentity != 0),
        span,
        end < HUGE_VALF && (loop ? end > start : end >= start),
        fabsf(LoadF32(e + 16)) < HUGE_VALF,
        true,
        maniLoadU32(e + 24) == frameAt,
        maniLoadU32(e + 28) == maniMatchEntryFrames(start, end, d->sampleRate, loop) &&
            maniLoadU32(e + 28) >= 1};
    for (uint32_t k = 0; k < 8; ++k)
    {
        if (!valid[k])
        {
            return 4 * k;
        }
    }
    return UINT32_MAX;
}

static maniFileResult CheckEntries(const maniMotionDatabase* d, const Sections* s)
{
    uint32_t frameAt = 0;
    for (uint32_t i = 0; i < d->entryCount; ++i)
    {
        const uint8_t* e = d->entries + (size_t)i * MANI_MATCH_ENTRY_SIZE;
        uint64_t here = s->at[4] + (uint64_t)i * MANI_MATCH_ENTRY_SIZE;
        uint32_t wrong = EntryWrong(d, e, frameAt);
        if (wrong != UINT32_MAX)
        {
            return Fail(here + wrong);
        }
        frameAt += maniLoadU32(e + 28);
        if (frameAt > d->frameCount)
        {
            return Fail(here + 28);
        }
    }
    return frameAt == d->frameCount ? kOk : Fail(s->at[0] + 12);
}

// Offsets finite, scales finite and above 0, features finite.
static maniFileResult CheckValues(const maniMotionDatabase* d, const Sections* s)
{
    for (uint32_t k = 0; k < 2 * d->featureCount; ++k)
    {
        float v = LoadF32(d->normalization + 4 * (size_t)k);
        bool scale = k >= d->featureCount;
        if (!(scale ? v > 0.0f && v < HUGE_VALF : fabsf(v) < HUGE_VALF))
        {
            return Fail(s->at[5] + 4 * (uint64_t)k);
        }
    }
    uint64_t count = (uint64_t)d->frameCount * d->featureCount;
    for (uint64_t k = 0; k < count; ++k)
    {
        if (!(fabsf(LoadF32(d->features + 4 * k)) < HUGE_VALF))
        {
            return Fail(s->at[6] + 4 * k);
        }
    }
    return kOk;
}

// Each position's frame within the frames and placed there, so the
// order holds every frame once; then each entry's frames at positions
// naming it.
static maniFileResult CheckOrder(const maniMotionDatabase* d, const Sections* s)
{
    for (uint32_t p = 0; p < d->frameCount; ++p)
    {
        uint64_t at = (uint64_t)p * MANI_MATCH_POSITION_SIZE;
        uint32_t frame = maniLoadU32(d->positions + at);
        if (frame >= d->frameCount)
        {
            return Fail(s->at[11] + at);
        }
        if (maniLoadU32(d->places + 4 * (size_t)frame) != p)
        {
            return Fail(s->at[12] + 4 * (uint64_t)frame);
        }
    }
    for (uint32_t e = 0; e < d->entryCount; ++e)
    {
        const uint8_t* entry = d->entries + (size_t)e * MANI_MATCH_ENTRY_SIZE;
        uint32_t first = maniLoadU32(entry + 24);
        uint32_t count = maniLoadU32(entry + 28);
        for (uint32_t f = first; f < first + count; ++f)
        {
            uint64_t p = maniLoadU32(d->places + 4 * (size_t)f);
            uint64_t at = p * MANI_MATCH_POSITION_SIZE + 4;
            if (maniLoadU32(d->positions + at) != e)
            {
                return Fail(s->at[11] + at);
            }
        }
    }
    return kOk;
}

// A leaf's words: no second half, its lowest frame, its least bias and
// its box, each over its positions in order.
static maniFileResult CheckLeaf(const maniMotionDatabase* d, const Sections* s,
                                const maniMatchVisit* v)
{
    uint64_t nodeAt = (uint64_t)v->node * MANI_MATCH_NODE_SIZE;
    const uint8_t* node = d->nodes + nodeAt;
    uint32_t lowest = UINT32_MAX;
    float bias = HUGE_VALF;
    for (uint32_t p = v->first; p < v->first + v->count; ++p)
    {
        const uint8_t* at = d->positions + (size_t)p * MANI_MATCH_POSITION_SIZE;
        uint32_t frame = maniLoadU32(at);
        lowest = frame < lowest ? frame : lowest;
        const uint8_t* entry = d->entries + (size_t)maniLoadU32(at + 4) * MANI_MATCH_ENTRY_SIZE;
        bias = maniMatchLower(bias, LoadF32(entry + 16));
    }
    const uint32_t words[3] = {0, lowest, Bits(bias)};
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (maniLoadU32(node + 4 * k) != words[k])
        {
            return Fail(s->at[10] + nodeAt + 4 * k);
        }
    }
    uint64_t n = d->featureCount;
    uint64_t boxAt = 4 * 2 * n * v->node;
    for (uint32_t j = 0; j < n; ++j)
    {
        float low = HUGE_VALF;
        float high = -HUGE_VALF;
        for (uint32_t p = v->first; p < v->first + v->count; ++p)
        {
            float x = LoadF32(d->features + 4 * ((uint64_t)p * n + j));
            low = maniMatchLower(low, x);
            high = maniMatchHigher(high, x);
        }
        uint64_t lowAt = boxAt + 4 * (uint64_t)j;
        uint64_t highAt = lowAt + 4 * n;
        if (maniLoadU32(d->boxes + lowAt) != Bits(low))
        {
            return Fail(s->at[7] + lowAt);
        }
        if (maniLoadU32(d->boxes + highAt) != Bits(high))
        {
            return Fail(s->at[7] + highAt);
        }
    }
    return kOk;
}

// Any other node's words from its halves' (node + 1 and its second
// half), checked already: the lower lowest frame and least bias, the
// lower minimums and higher maximums.
static maniFileResult CheckSplit(const maniMotionDatabase* d, const Sections* s, uint32_t k,
                                 uint32_t second)
{
    uint64_t nodeAt = (uint64_t)k * MANI_MATCH_NODE_SIZE;
    const uint8_t* a = d->nodes + nodeAt + MANI_MATCH_NODE_SIZE;
    const uint8_t* b = d->nodes + (size_t)second * MANI_MATCH_NODE_SIZE;
    uint32_t lowest =
        maniLoadU32(a + 4) < maniLoadU32(b + 4) ? maniLoadU32(a + 4) : maniLoadU32(b + 4);
    float bias = maniMatchLower(LoadF32(a + 8), LoadF32(b + 8));
    if (maniLoadU32(d->nodes + nodeAt + 4) != lowest)
    {
        return Fail(s->at[10] + nodeAt + 4);
    }
    if (maniLoadU32(d->nodes + nodeAt + 8) != Bits(bias))
    {
        return Fail(s->at[10] + nodeAt + 8);
    }
    uint64_t width = 2 * (uint64_t)d->featureCount;
    uint64_t boxAt = 4 * width * k;
    const uint8_t* boxA = d->boxes + boxAt + 4 * width;
    const uint8_t* boxB = d->boxes + 4 * width * second;
    for (uint32_t j = 0; j < width; ++j)
    {
        float x = LoadF32(boxA + 4 * (size_t)j);
        float y = LoadF32(boxB + 4 * (size_t)j);
        float v = j < d->featureCount ? maniMatchLower(x, y) : maniMatchHigher(x, y);
        if (maniLoadU32(d->boxes + boxAt + 4 * (uint64_t)j) != Bits(v))
        {
            return Fail(s->at[7] + boxAt + 4 * (uint64_t)j);
        }
    }
    return kOk;
}

// The tree: on the walk, each node that is a second half named by the
// node it halves and each leaf; then the other nodes from the last up,
// so their halves are checked first.
static maniFileResult CheckTree(const maniMotionDatabase* d, const Sections* s)
{
    maniMatchWalk walk;
    maniMatchVisit v;
    maniMatchWalkStart(&walk, d->frameCount, d->leafFrames);
    while (maniMatchWalkNext(&walk, &v))
    {
        uint64_t halfAt = (uint64_t)v.halfOf * MANI_MATCH_NODE_SIZE;
        if (v.halfOf != UINT32_MAX && maniLoadU32(d->nodes + halfAt) != v.node)
        {
            return Fail(s->at[10] + halfAt);
        }
        maniFileResult r = v.count <= d->leafFrames ? CheckLeaf(d, s, &v) : kOk;
        if (r.result != mani_success)
        {
            return r;
        }
    }
    for (uint32_t k = d->nodeCount; k-- > 0;)
    {
        uint32_t second = maniLoadU32(d->nodes + (size_t)k * MANI_MATCH_NODE_SIZE);
        maniFileResult r = second != 0 ? CheckSplit(d, s, k, second) : kOk;
        if (r.result != mani_success)
        {
            return r;
        }
    }
    return kOk;
}

static MANI_NOINLINE maniFileResult OpenMotionDatabaseWork(const void* bytes, size_t size,
                                                           maniMotionDatabase* databaseOut)
{
    if (databaseOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r =
        maniOpenContainer(bytes, size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    // Before 1.0 a reader reads its own format version alone: an older
    // database is built again from its source.
    if (c.formatVersion != MANI_MATCH_FORMAT_VERSION)
    {
        return (maniFileResult){mani_errorVersion, 0, 6};
    }
    Sections s;
    maniMotionDatabase d;
    r = LoadSections(&c, &s);
    r = r.result == mani_success ? CheckHeader(&s, &d) : r;
    r = r.result == mani_success ? CheckSizes(&s, &d) : r;
    r = r.result == mani_success ? CheckChannels(&d, &s) : r;
    r = r.result == mani_success ? CheckEntries(&d, &s) : r;
    r = r.result == mani_success ? CheckValues(&d, &s) : r;
    r = r.result == mani_success ? CheckOrder(&d, &s) : r;
    r = r.result == mani_success ? CheckTree(&d, &s) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniMatchComputeIdentity(s.data, s.sizes) != c.identity)
    {
        return Fail(40);
    }
    d.identity = c.identity;
    *databaseOut = d;
    return kOk;
}

maniFileResult maniOpenMotionDatabase(const void* bytes, size_t size,
                                      maniMotionDatabase* databaseOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniFileResult result = OpenMotionDatabaseWork(bytes, size, databaseOut);
    maniFloatEnvLeave(&env);
    return result;
}

uint64_t maniMotionDatabaseIdentity(const maniMotionDatabase* database)
{
    return database != NULL ? database->identity : 0;
}

uint64_t maniMotionDatabaseMirrorMapIdentity(const maniMotionDatabase* database)
{
    return database != NULL ? database->mirrorMapIdentity : 0;
}

uint64_t maniMotionDatabaseSkeletonIdentity(const maniMotionDatabase* database)
{
    return database != NULL ? database->skeletonIdentity : 0;
}

uint64_t maniMotionDatabaseClipIdentity(const maniMotionDatabase* database, uint32_t clip)
{
    if (database == NULL || clip >= database->clipCount)
    {
        return 0;
    }
    return maniLoadU64(database->clips + 8 * (size_t)clip);
}

maniResult maniMotionDatabaseChannel(const maniMotionDatabase* database, uint32_t channel,
                                     maniMatchChannel* channelOut)
{
    if (database == NULL || channelOut == NULL || channel >= database->channelCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e = database->channels + (size_t)channel * MANI_MATCH_CHANNEL_SIZE;
    uint32_t count = maniLoadU32(e + 12);
    *channelOut = (maniMatchChannel){.kind = maniLoadU32(e),
                                     .bone = maniLoadU32(e + 4),
                                     .axis = maniLoadU32(e + 8),
                                     .timeCount = count,
                                     .weight = LoadF32(e + 28),
                                     .firstFeature = maniLoadU32(e + 20),
                                     .featureCount = maniLoadU32(e + 24)};
    memcpy(channelOut->times, database->times + 4 * (size_t)maniLoadU32(e + 16), 4 * (size_t)count);
    return mani_success;
}

const char* maniMotionDatabaseChannelName(const maniMotionDatabase* database, uint32_t channel,
                                          uint32_t* lengthOut)
{
    if (database == NULL || channel >= database->channelCount)
    {
        return NULL;
    }
    const uint8_t* e = database->channels + (size_t)channel * MANI_MATCH_CHANNEL_SIZE;
    if (maniLoadU32(e) != maniMatchFloatTrack)
    {
        return NULL;
    }
    if (lengthOut != NULL)
    {
        *lengthOut = maniLoadU32(e + 36);
    }
    return (const char*)(database->names + maniLoadU32(e + 32));
}

maniResult maniMotionDatabaseEntry(const maniMotionDatabase* database, uint32_t entry,
                                   maniMatchEntry* entryOut)
{
    if (database == NULL || entryOut == NULL || entry >= database->entryCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e = database->entries + (size_t)entry * MANI_MATCH_ENTRY_SIZE;
    *entryOut = (maniMatchEntry){maniLoadU32(e),      maniLoadU32(e + 4), LoadF32(e + 8),
                                 LoadF32(e + 12),     LoadF32(e + 16),    maniLoadU32(e + 20),
                                 maniLoadU32(e + 24), maniLoadU32(e + 28)};
    return mani_success;
}

static MANI_NOINLINE maniResult MotionDatabaseFrameWork(const maniMotionDatabase* database,
                                                        uint32_t frame, uint32_t* entryOut,
                                                        float* timeOut)
{
    if (database == NULL || entryOut == NULL || timeOut == NULL || frame >= database->frameCount)
    {
        return mani_errorInvalid;
    }
    // The entry whose frames hold it: the last whose first is at most it.
    uint32_t low = 0;
    uint32_t high = database->entryCount - 1;
    while (low < high)
    {
        uint32_t mid = low + (high - low + 1) / 2;
        uint32_t first = maniLoadU32(database->entries + (size_t)mid * MANI_MATCH_ENTRY_SIZE + 24);
        low = first <= frame ? mid : low;
        high = first <= frame ? high : mid - 1;
    }
    const uint8_t* e = database->entries + (size_t)low * MANI_MATCH_ENTRY_SIZE;
    *entryOut = low;
    *timeOut =
        maniMatchFrameTime(LoadF32(e + 8), frame - maniLoadU32(e + 24), database->sampleRate);
    return mani_success;
}

maniResult maniMotionDatabaseFrame(const maniMotionDatabase* database, uint32_t frame,
                                   uint32_t* entryOut, float* timeOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = MotionDatabaseFrameWork(database, frame, entryOut, timeOut);
    maniFloatEnvLeave(&env);
    return result;
}

maniResult maniMotionDatabaseFeatures(const maniMotionDatabase* database, uint32_t frame,
                                      float* featuresOut)
{
    if (database == NULL || featuresOut == NULL || frame >= database->frameCount)
    {
        return mani_errorInvalid;
    }
    size_t position = maniLoadU32(database->places + 4 * (size_t)frame);
    memcpy(featuresOut, database->features + 4 * position * database->featureCount,
           4 * (size_t)database->featureCount);
    return mani_success;
}

maniResult maniMotionDatabaseNormalization(const maniMotionDatabase* database, float* offsetsOut,
                                           float* scalesOut)
{
    if (database == NULL)
    {
        return mani_errorInvalid;
    }
    size_t bytes = 4 * (size_t)database->featureCount;
    if (offsetsOut != NULL)
    {
        memcpy(offsetsOut, database->normalization, bytes);
    }
    if (scalesOut != NULL)
    {
        memcpy(scalesOut, database->normalization + bytes, bytes);
    }
    return mani_success;
}
