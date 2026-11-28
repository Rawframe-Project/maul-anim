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
#include "invariant.h"
#include "match_file.h"
#include "match_view.h"
#include "text.h"
#include "view.h"

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

// The rate is a database's or a source's, each checked positive and
// at most MANI_MAX_MATCH_RATE before any time is computed from it.
float maniMatchFrameTime(float start, uint32_t k, float rate)
{
    MANI_ASSERT(rate > 0.0f && rate <= MANI_MAX_MATCH_RATE);
    return (float)((double)start + (double)k / (double)rate);
}

MANI_NODISCARD static bool Within(float time, float end, bool loop)
{
    return loop ? time < end : time <= end;
}

bool maniMatchEntryFrames(float start, float end, float rate, bool loop, uint32_t* framesOut)
{
    double span = ((double)end - (double)start) * (double)rate;
    if (!(span >= 0.0 && span < (double)MANI_MAX_MATCH_FRAMES))
    {
        return false;
    }
    // The binary64 count, then one frame less when the last's binary32
    // time does not hold, or one more when the next's does: once only, as
    // far past 2^24 seconds a step no longer moves the time.
    uint32_t n = (uint32_t)floor(span) + 1u;
    MANI_ASSERT(n <= MANI_MAX_MATCH_FRAMES);
    if (n > 1 && !Within(maniMatchFrameTime(start, n - 1, rate), end, loop))
    {
        *framesOut = n - 1;
        return true;
    }
    *framesOut = Within(maniMatchFrameTime(start, n, rate), end, loop) ? n + 1 : n;
    return true;
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
        // The container keeps tags from 1 to the last.
        MANI_ASSERT(tag >= 1 && tag <= MANI_MATCH_SECTIONS);
        uint64_t size = maniLoadU64(entry + 16);
        // The container refuses an empty entry: an empty section is left out.
        MANI_ASSERT(size > 0);
        uint64_t offset = maniLoadU64(entry + 8);
        s->data[tag - 1] = c->bytes + offset;
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = offset;
        s->entry[tag - 1] = entryAt + 16;
    }
    return kOk;
}

// The header's counts within their limits, the rate, the axes on two
// different lines, the leaf size (its own section's, absent the default,
// present only off it) and the tree's node count.
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
                               MANI_MAX_MATCH_FRAMES,
                               MANI_MAX_MATCH_FEATURES,
                               MANI_MAX_MATCH_CHANNELS,
                               MANI_MAX_MATCH_CHANNELS * MANI_MAX_MATCH_TIMES,
                               MANI_MAX_MATCH_ENTRIES,
                               MANI_MAX_MATCH_CLIPS};
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
    if (!(rate > 0.0f && rate <= MANI_MAX_MATCH_RATE))
    {
        return Fail(at + 36);
    }
    if ((uint32_t)up > maniMatchAxisNegativeZ)
    {
        return Fail(at + 40);
    }
    if ((uint32_t)forward > maniMatchAxisNegativeZ || forward % 3 == up % 3)
    {
        return Fail(at + 44);
    }
    uint32_t leaf = MANI_MATCH_DEFAULT_LEAF;
    if (s->sizes[13] != 0)
    {
        if (s->sizes[13] != 4)
        {
            return Fail(s->entry[13]);
        }
        // The sizes but the default.
        leaf = maniLoadU32(s->data[13]);
        if (leaf != 8 && leaf != 32)
        {
            return Fail(s->at[13]);
        }
    }
    uint32_t nodes = maniLoadU32(h + 48);
    if (nodes != maniMatchNodeCount(counts[1], leaf))
    {
        return Fail(at + 48);
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

// Every section but the names, the mirror and the leaf size of the size
// the header's counts give.
static maniFileResult CheckSizes(const Sections* s, maniMotionDatabase* d)
{
    uint64_t features = d->featureCount;
    uint64_t frames = d->frameCount;
    uint64_t nodes = d->nodeCount;
    // By tag but the leaf size: the header is checked, the names and
    // mirror below, the leaf size with the header.
    const uint64_t sizes[MANI_MATCH_SECTIONS - 1] = {
        MANI_MATCH_HEADER_SIZE,
        (uint64_t)d->channelCount * MANI_MATCH_CHANNEL_SIZE,
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
    for (uint32_t k = 1; k < MANI_MATCH_SECTIONS - 1; ++k)
    {
        if (s->sizes[k] != sizes[k])
        {
            return Fail(s->entry[k]);
        }
    }
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
MANI_NODISCARD static bool NameValid(const Sections* s, uint32_t kind, const uint8_t* e)
{
    uint32_t offset = maniLoadU32(e + 32);
    uint32_t length = maniLoadU32(e + 36);
    if (kind != maniMatchFloatTrack)
    {
        return offset == 0 && length == 0;
    }
    return length > 0 && offset <= s->sizes[8] && length <= s->sizes[8] - offset &&
           maniValidUtf8(s->data[8] + offset, length);
}

// One channel's words in order (kind, bone, axis, times, first time,
// first feature, features, weight, name), the first wrong one's place.
static uint32_t ChannelWrong(const maniMotionDatabase* d, const Sections* s, const uint8_t* e,
                             uint32_t timeAt, uint32_t featureAt)
{
    uint32_t kind = maniLoadU32(e);
    uint32_t times = maniLoadU32(e + 12);
    bool bone = (uint32_t)kind <= maniMatchBoneFacing;
    float weight = LoadF32(e + 28);
    const bool valid[9] = {
        (uint32_t)kind <= maniMatchFloatTrack,
        bone ? maniLoadU32(e + 4) < d->skeletonBoneCount : maniLoadU32(e + 4) == 0,
        kind == maniMatchBoneFacing ? maniLoadU32(e + 8) <= maniMatchAxisNegativeZ
                                    : maniLoadU32(e + 8) == 0,
        times >= 1 && times <= MANI_MAX_MATCH_TIMES,
        maniLoadU32(e + 16) == timeAt,
        maniLoadU32(e + 20) == featureAt,
        maniLoadU32(e + 24) == maniMatchChannelWidth(kind) * times,
        weight > 0.0f && weight < HUGE_VALF,
        NameValid(s, kind, e)};
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
        const uint8_t* e = s->data[1] + (size_t)c * MANI_MATCH_CHANNEL_SIZE;
        uint64_t here = s->at[1] + (uint64_t)c * MANI_MATCH_CHANNEL_SIZE;
        uint32_t wrong = ChannelWrong(d, s, e, timeAt, featureAt);
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
        float time = LoadF32(s->data[2] + 4 * (size_t)t);
        if (!(fabsf(time) <= MANI_MAX_MATCH_REACH))
        {
            return Fail(s->at[2] + 4 * (uint64_t)t);
        }
    }
    return kOk;
}

// One entry's words in order (clip, flags, start, end, bias, tags, first
// frame, frames), the first wrong one's place.
static uint32_t EntryWrong(const maniMotionDatabase* d, const uint8_t* e, uint64_t frameAt)
{
    uint32_t flags = maniLoadU32(e + 4);
    float start = LoadF32(e + 8);
    float end = LoadF32(e + 12);
    bool loop = (flags & maniMatchEntryLoop) != 0;
    bool mirrored = (flags & maniMatchEntryMirrored) != 0;
    bool span = start >= 0.0f && start < HUGE_VALF;
    uint32_t frames = 0;
    bool held = span && maniMatchEntryFrames(start, end, d->sampleRate, loop, &frames);
    const bool valid[8] = {maniLoadU32(e) < d->clipCount,
                           flags <= MANI_MATCH_ENTRY_FLAGS &&
                               (!mirrored || d->mirrorMapIdentity != 0),
                           span,
                           end < HUGE_VALF && (loop ? end > start : end >= start),
                           fabsf(LoadF32(e + 16)) < HUGE_VALF,
                           true,
                           maniLoadU32(e + 24) == frameAt,
                           held && maniLoadU32(e + 28) == frames && frames >= 1};
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
    uint64_t frameAt = 0;
    for (uint32_t i = 0; i < d->entryCount; ++i)
    {
        const uint8_t* e = s->data[4] + (size_t)i * MANI_MATCH_ENTRY_SIZE;
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
        float v = LoadF32(s->data[5] + 4 * (size_t)k);
        bool scale = k >= d->featureCount;
        if (!(scale ? v > 0.0f && v < HUGE_VALF : fabsf(v) < HUGE_VALF))
        {
            return Fail(s->at[5] + 4 * (uint64_t)k);
        }
    }
    uint64_t count = (uint64_t)d->frameCount * d->featureCount;
    for (uint64_t k = 0; k < count; ++k)
    {
        if (!(fabsf(LoadF32(s->data[6] + 4 * k)) < HUGE_VALF))
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
        uint32_t frame = maniLoadU32(s->data[11] + at);
        if (frame >= d->frameCount)
        {
            return Fail(s->at[11] + at);
        }
        if (maniLoadU32(s->data[12] + 4 * (size_t)frame) != p)
        {
            return Fail(s->at[12] + 4 * (uint64_t)frame);
        }
    }
    for (uint32_t e = 0; e < d->entryCount; ++e)
    {
        const uint8_t* entry = s->data[4] + (size_t)e * MANI_MATCH_ENTRY_SIZE;
        uint32_t first = maniLoadU32(entry + 24);
        uint32_t count = maniLoadU32(entry + 28);
        for (uint32_t f = first; f < first + count; ++f)
        {
            uint64_t p = maniLoadU32(s->data[12] + 4 * (size_t)f);
            uint64_t at = p * MANI_MATCH_POSITION_SIZE + 4;
            if (maniLoadU32(s->data[11] + at) != e)
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
    const uint8_t* node = s->data[10] + nodeAt;
    uint32_t lowest = UINT32_MAX;
    float bias = HUGE_VALF;
    for (uint32_t p = v->first; p < v->first + v->count; ++p)
    {
        const uint8_t* at = s->data[11] + (size_t)p * MANI_MATCH_POSITION_SIZE;
        uint32_t frame = maniLoadU32(at);
        lowest = frame < lowest ? frame : lowest;
        const uint8_t* entry = s->data[4] + (size_t)maniLoadU32(at + 4) * MANI_MATCH_ENTRY_SIZE;
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
            float x = LoadF32(s->data[6] + 4 * ((uint64_t)p * n + j));
            low = maniMatchLower(low, x);
            high = maniMatchHigher(high, x);
        }
        uint64_t lowAt = boxAt + 4 * (uint64_t)j;
        uint64_t highAt = lowAt + 4 * n;
        if (maniLoadU32(s->data[7] + lowAt) != Bits(low))
        {
            return Fail(s->at[7] + lowAt);
        }
        if (maniLoadU32(s->data[7] + highAt) != Bits(high))
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
    const uint8_t* a = s->data[10] + nodeAt + MANI_MATCH_NODE_SIZE;
    const uint8_t* b = s->data[10] + (size_t)second * MANI_MATCH_NODE_SIZE;
    uint32_t lowest =
        maniLoadU32(a + 4) < maniLoadU32(b + 4) ? maniLoadU32(a + 4) : maniLoadU32(b + 4);
    float bias = maniMatchLower(LoadF32(a + 8), LoadF32(b + 8));
    if (maniLoadU32(s->data[10] + nodeAt + 4) != lowest)
    {
        return Fail(s->at[10] + nodeAt + 4);
    }
    if (maniLoadU32(s->data[10] + nodeAt + 8) != Bits(bias))
    {
        return Fail(s->at[10] + nodeAt + 8);
    }
    uint64_t width = 2 * (uint64_t)d->featureCount;
    uint64_t boxAt = 4 * width * k;
    const uint8_t* boxA = s->data[7] + boxAt + 4 * width;
    const uint8_t* boxB = s->data[7] + 4 * width * second;
    for (uint32_t j = 0; j < width; ++j)
    {
        float x = LoadF32(boxA + 4 * (size_t)j);
        float y = LoadF32(boxB + 4 * (size_t)j);
        float v = j < d->featureCount ? maniMatchLower(x, y) : maniMatchHigher(x, y);
        if (maniLoadU32(s->data[7] + boxAt + 4 * (uint64_t)j) != Bits(v))
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
        if (v.halfOf != UINT32_MAX && maniLoadU32(s->data[10] + halfAt) != v.node)
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
        uint32_t second = maniLoadU32(s->data[10] + (size_t)k * MANI_MATCH_NODE_SIZE);
        maniFileResult r = second != 0 ? CheckSplit(d, s, k, second) : kOk;
        if (r.result != mani_success)
        {
            return r;
        }
    }
    return kOk;
}

static MANI_NOINLINE maniFileResult OpenMotionDatabaseWork(const void* bytes, size_t size,
                                                           void* out)
{
    maniMotionDatabase* databaseOut = out;
    if (databaseOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindMotionDatabase,
                                         MANI_MATCH_FORMAT_VERSION, MANI_MATCH_SECTIONS, &c);
    if (r.result != mani_success)
    {
        return r;
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
    d.base = bytes;
    d.size = size;
    // The sections by their tags, the mirror's and the leaf size's left
    // out: the view holds both as values.
    const uint32_t tags[11] = {1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12};
    for (uint32_t k = 0; k < 11; ++k)
    {
        d.sections[k] = s.at[tags[k]];
    }
    d.cookie = MANI_VIEW_COOKIE(maniKindMotionDatabase);
    *databaseOut = d;
    return kOk;
}

maniFileResult maniOpenMotionDatabase(const void* bytes, size_t size,
                                      maniMotionDatabase* databaseOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniFileResult result =
        maniContainerOpenTagged(bytes, size, databaseOut, OpenMotionDatabaseWork);
    maniFloatEnvLeave(env);
    return result;
}

MANI_NOINLINE bool maniMotionDatabaseValid(const maniMotionDatabase* database)
{
    const maniMotionDatabase* d = database;
    if (d == NULL || d->cookie != MANI_VIEW_COOKIE(maniKindMotionDatabase) || d->base == NULL ||
        d->frameCount - 1u >= MANI_MAX_MATCH_FRAMES ||
        d->featureCount - 1u >= MANI_MAX_MATCH_FEATURES || d->entryCount == 0 ||
        d->nodeCount == 0 || !(d->sampleRate > 0.0f && d->sampleRate <= MANI_MAX_MATCH_RATE) ||
        (d->leafFrames != 8 && d->leafFrames != 16 && d->leafFrames != 32))
    {
        return false;
    }
    uint64_t n = d->featureCount;
    uint64_t frames = d->frameCount;
    const uint64_t bytes[11] = {(uint64_t)d->channelCount * MANI_MATCH_CHANNEL_SIZE,
                                4 * (uint64_t)d->timeCount,
                                8 * (uint64_t)d->clipCount,
                                (uint64_t)d->entryCount * MANI_MATCH_ENTRY_SIZE,
                                8 * n,
                                4 * n * frames,
                                8 * n * d->nodeCount,
                                d->namesSize,
                                (uint64_t)d->nodeCount * MANI_MATCH_NODE_SIZE,
                                frames * MANI_MATCH_POSITION_SIZE,
                                4 * frames};
    bool fits = true;
    for (uint32_t k = 0; k < 11; ++k)
    {
        fits = fits && maniViewFits(d->sections[k], bytes[k], d->size);
    }
    return fits;
}

uint64_t maniMotionDatabase_GetIdentity(const maniMotionDatabase* database)
{
    return maniMotionDatabaseValid(database) ? database->identity : 0;
}

uint64_t maniMotionDatabase_GetMirrorMapIdentity(const maniMotionDatabase* database)
{
    return maniMotionDatabaseValid(database) ? database->mirrorMapIdentity : 0;
}

uint64_t maniMotionDatabase_GetSkeletonIdentity(const maniMotionDatabase* database)
{
    return maniMotionDatabaseValid(database) ? database->skeletonIdentity : 0;
}

uint64_t maniMotionDatabase_GetClipIdentity(const maniMotionDatabase* database, uint32_t clip)
{
    if (!maniMotionDatabaseValid(database) || clip >= database->clipCount)
    {
        return 0;
    }
    return maniLoadU64(maniMatchSection(database, maniMatchClips) + 8 * (size_t)clip);
}

maniResult maniMotionDatabase_GetChannel(const maniMotionDatabase* database, uint32_t channel,
                                         maniMatchChannel* channelOut)
{
    if (!maniMotionDatabaseValid(database) || channelOut == NULL ||
        channel >= database->channelCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e =
        maniMatchSection(database, maniMatchChannels) + (size_t)channel * MANI_MATCH_CHANNEL_SIZE;
    // In bytes changed since the open, a kind past the last is a float
    // track, a bone past the skeleton none, an axis past the six the
    // first, and the times and features past the database's are cut at
    // their end, and the times past what the features hold.
    uint32_t kind = maniLoadU32(e);
    kind = kind <= maniMatchFloatTrack ? kind : maniMatchFloatTrack;
    uint32_t bone = maniLoadU32(e + 4);
    uint32_t axis = maniLoadU32(e + 8);
    uint32_t firstTime = maniLoadU32(e + 16);
    firstTime = firstTime < database->timeCount ? firstTime : database->timeCount;
    uint32_t count = maniLoadU32(e + 12);
    uint32_t times = database->timeCount - firstTime;
    times = times < MANI_MAX_MATCH_TIMES ? times : MANI_MAX_MATCH_TIMES;
    count = count < times ? count : times;
    uint32_t firstFeature = maniLoadU32(e + 20);
    firstFeature = firstFeature < database->featureCount ? firstFeature : database->featureCount;
    uint32_t features = maniLoadU32(e + 24);
    uint32_t room = database->featureCount - firstFeature;
    features = features < room ? features : room;
    uint32_t width = maniMatchChannelWidth(kind);
    count = count < features / width ? count : features / width;
    *channelOut =
        (maniMatchChannel){.kind = kind,
                           .bone = bone < database->skeletonBoneCount ? bone : MANI_NO_BONE,
                           .axis = axis <= maniMatchAxisNegativeZ ? axis : maniMatchAxisX,
                           .timeCount = count,
                           .weight = LoadF32(e + 28),
                           .firstFeature = firstFeature,
                           .featureCount = features};
    memcpy(channelOut->times, maniMatchSection(database, maniMatchTimes) + 4 * (size_t)firstTime,
           4 * (size_t)count);
    return mani_success;
}

const char* maniMotionDatabase_GetChannelName(const maniMotionDatabase* database, uint32_t channel,
                                              size_t* lengthOut)
{
    if (lengthOut != NULL)
    {
        *lengthOut = 0;
    }
    if (!maniMotionDatabaseValid(database) || channel >= database->channelCount)
    {
        return NULL;
    }
    const uint8_t* e =
        maniMatchSection(database, maniMatchChannels) + (size_t)channel * MANI_MATCH_CHANNEL_SIZE;
    if (maniLoadU32(e) != maniMatchFloatTrack)
    {
        return NULL;
    }
    // A name in bytes changed since the open past the names is empty, at
    // the names' start.
    uint64_t offset = maniLoadU32(e + 32);
    uint64_t length = maniLoadU32(e + 36);
    bool named = maniViewFits(offset, length, database->namesSize);
    if (lengthOut != NULL)
    {
        *lengthOut = named ? (size_t)length : 0;
    }
    return (const char*)maniMatchSection(database, maniMatchNames) + (named ? offset : 0);
}

maniResult maniMotionDatabase_GetEntry(const maniMotionDatabase* database, uint32_t entry,
                                       maniMatchEntry* entryOut)
{
    if (!maniMotionDatabaseValid(database) || entryOut == NULL || entry >= database->entryCount)
    {
        return mani_errorInvalid;
    }
    const uint8_t* e = maniMatchEntryRecord(database, entry);
    // In bytes changed since the open, a clip past the clips is clip 0
    // and the frames past the frames are cut at their end.
    uint32_t clip = maniLoadU32(e);
    uint32_t first = maniLoadU32(e + 24);
    first = first < database->frameCount ? first : database->frameCount;
    uint32_t frames = maniLoadU32(e + 28);
    uint32_t room = database->frameCount - first;
    *entryOut = (maniMatchEntry){clip < database->clipCount ? clip : 0u,
                                 maniLoadU32(e + 4),
                                 LoadF32(e + 8),
                                 LoadF32(e + 12),
                                 LoadF32(e + 16),
                                 maniLoadU32(e + 20),
                                 first,
                                 frames < room ? frames : room};
    return mani_success;
}

static MANI_NOINLINE maniResult MotionDatabaseFrameWork(const maniMotionDatabase* database,
                                                        uint32_t frame, uint32_t* entryOut,
                                                        float* timeOut)
{
    if (!maniMotionDatabaseValid(database) || entryOut == NULL || timeOut == NULL ||
        frame >= database->frameCount)
    {
        return mani_errorInvalid;
    }
    // The entry whose frames hold it: the last whose first is at most it.
    // The opener checked that the entries' frames follow each other from
    // 0 to the frame count, so one holds every frame; in bytes changed
    // since the open, the search still ends at an entry, and the time is
    // some time.
    uint32_t low = 0;
    uint32_t high = database->entryCount - 1;
    while (low < high)
    {
        uint32_t mid = low + (high - low + 1) / 2;
        uint32_t first = maniLoadU32(maniMatchEntryRecord(database, mid) + 24);
        low = first <= frame ? mid : low;
        high = first <= frame ? high : mid - 1;
    }
    const uint8_t* e = maniMatchEntryRecord(database, low);
    *entryOut = low;
    *timeOut =
        maniMatchFrameTime(LoadF32(e + 8), frame - maniLoadU32(e + 24), database->sampleRate);
    return mani_success;
}

maniResult maniMotionDatabase_GetFrame(const maniMotionDatabase* database, uint32_t frame,
                                       uint32_t* entryOut, float* timeOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = MotionDatabaseFrameWork(database, frame, entryOut, timeOut);
    maniFloatEnvLeave(env);
    return result;
}

maniResult maniMotionDatabase_GetFeatures(const maniMotionDatabase* database, uint32_t frame,
                                          float* featuresOut, uint32_t capacity, uint32_t* countOut)
{
    if (!maniMotionDatabaseValid(database) || (featuresOut == NULL && capacity > 0) ||
        frame >= database->frameCount)
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = database->featureCount;
    }
    if (featuresOut == NULL || capacity < database->featureCount)
    {
        return mani_errorCapacity;
    }
    memcpy(featuresOut, maniMatchFrameFeatures(database, frame),
           4 * (size_t)database->featureCount);
    return mani_success;
}

maniResult maniMotionDatabase_GetNormalization(const maniMotionDatabase* database,
                                               float* offsetsOut, float* scalesOut,
                                               uint32_t capacity, uint32_t* countOut)
{
    if (!maniMotionDatabaseValid(database) ||
        (offsetsOut == NULL && scalesOut == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = database->featureCount;
    }
    if (capacity < database->featureCount)
    {
        return mani_errorCapacity;
    }
    size_t bytes = 4 * (size_t)database->featureCount;
    if (offsetsOut != NULL)
    {
        memcpy(offsetsOut, maniMatchSection(database, maniMatchNormalization), bytes);
    }
    if (scalesOut != NULL)
    {
        memcpy(scalesOut, maniMatchSection(database, maniMatchNormalization) + bytes, bytes);
    }
    return mani_success;
}
