// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a motion database, and on success reading every
// channel, entry and frame; then the same bytes with the payload's hash
// made right, and then the identity too, so that mutations reach the
// checks behind them. Run as fuzz_match <corpus dir>; the seed comes from
// test_match --seed <file>. After each open, up to 16 of the file's bytes
// change and every channel, entry, frame and name is read again, and the
// database searched: within the file, whatever it gives.

#include "container.h"
#include "match_file.h"

#include "maul-anim/match.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

// Up to 16 bytes inside an opened file changed, picked from its own
// bytes, as a host's memory may change after the open. Restore puts them
// back.
typedef struct Changed
{
    uint64_t at[16];
    uint8_t kept[16];
} Changed;

static Changed Change(uint8_t* bytes, uint64_t size)
{
    Changed c;
    uint64_t state = size * 0x9E3779B97F4A7C15u;
    for (uint64_t i = 0; i < size && i < 64; ++i)
    {
        state = (state ^ bytes[i]) * 0x100000001B3u;
    }
    for (int i = 0; i < 16; ++i)
    {
        state ^= state >> 29;
        state *= 0xBF58476D1CE4E5B9u;
        c.at[i] = state % size;
        c.kept[i] = bytes[c.at[i]];
        bytes[c.at[i]] ^= (uint8_t)(state >> 56) | 1u;
    }
    return c;
}

static void Restore(uint8_t* bytes, const Changed* c)
{
    for (int i = 15; i >= 0; --i)
    {
        bytes[c->at[i]] = c->kept[i];
    }
}

// Every channel, name, entry and frame of bytes changed since the open
// read within the database's, and a search's pick one of its frames.
static void Changes(const maniMotionDatabase* d)
{
    const char* names = (const char*)d->base + d->sections[7];
    for (uint32_t c = 0; c < d->channelCount; ++c)
    {
        maniMatchChannel ch;
        size_t length = 0;
        const char* name = maniMotionDatabase_GetChannelName(d, c, &length);
        if (maniMotionDatabase_GetChannel(d, c, &ch) != 0 ||
            ch.firstFeature + (uint64_t)ch.featureCount > d->featureCount ||
            (uint64_t)maniMatchChannelWidth(ch.kind) * ch.timeCount > ch.featureCount ||
            (name != NULL && (name < names || length > d->namesSize ||
                              (uint64_t)(name - names) > d->namesSize - length)))
        {
            abort();
        }
    }
    float features[MANI_MAX_MATCH_FEATURES];
    for (uint32_t f = 0; f < d->frameCount; ++f)
    {
        uint32_t entry = 0;
        float time = 0.0f;
        maniMatchEntry e;
        if (maniMotionDatabase_GetFrame(d, f, &entry, &time) != 0 ||
            maniMotionDatabase_GetEntry(d, entry, &e) != 0 || e.clip >= d->clipCount ||
            e.firstFrame + (uint64_t)e.frameCount > d->frameCount ||
            maniMotionDatabase_GetFeatures(d, f, features, d->featureCount, NULL) != 0)
        {
            abort();
        }
    }
    float query[MANI_MAX_MATCH_FEATURES] = {0.0f};
    maniMatchQuery q = {.features = query,
                        .current = 0,
                        .historyCount = 1,
                        .history = {d->frameCount - 1},
                        .jumpThreshold = 0.1f,
                        .historyTime = 0.1f};
    maniMatchResult r;
    if (maniMotionDatabase_Search(d, &q, &r) == 0 && r.frame != MANI_MATCH_NO_FRAME &&
        r.frame >= d->frameCount)
    {
        abort();
    }
}

static void Exercise(uint8_t* bytes, size_t size)
{
    maniMotionDatabase d;
    if (maniOpenMotionDatabase(bytes, size, &d).result != 0)
    {
        return;
    }
    Changed changed = Change(bytes, d.size);
    Changes(&d);
    Restore(bytes, &changed);
    for (uint32_t c = 0; c < d.channelCount; ++c)
    {
        maniMatchChannel ch;
        if (maniMotionDatabase_GetChannel(&d, c, &ch) != 0 || ch.timeCount == 0 ||
            ch.firstFeature + ch.featureCount > d.featureCount)
        {
            abort();
        }
    }
    float features[MANI_MAX_MATCH_FEATURES];
    for (uint32_t f = 0; f < d.frameCount; ++f)
    {
        uint32_t entry = 0;
        float time = 0.0f;
        maniMatchEntry e;
        if (maniMotionDatabase_GetFrame(&d, f, &entry, &time) != 0 ||
            maniMotionDatabase_GetEntry(&d, entry, &e) != 0 || f < e.firstFrame ||
            f >= e.firstFrame + e.frameCount ||
            maniMotionDatabase_GetFeatures(&d, f, features, d.featureCount, NULL) != 0)
        {
            abort();
        }
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION,
                          MANI_MATCH_SECTIONS, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_MATCH_SECTIONS];
    uint64_t sizes[MANI_MATCH_SECTIONS];
    for (uint32_t t = 1; t <= MANI_MATCH_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniMatchComputeIdentity(sections, sizes);
    memcpy(bytes + 40, &identity, sizeof identity);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    // The file is opened from a 16-byte aligned copy, as a host would;
    // aligned by hand, as aligned_alloc is not everywhere.
    uint8_t* block = malloc(size + 16);
    if (block == NULL)
    {
        return 0;
    }
    uint8_t* copy = block + (16 - (uintptr_t)block % 16) % 16;
    if (size > 0)
    {
        memcpy(copy, data, size);
    }
    Exercise(copy, size);
    uint64_t fileSize = 0;
    if (size >= MANI_CONTAINER_HEADER_SIZE)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= MANI_CONTAINER_HEADER_SIZE && fileSize <= size)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + MANI_CONTAINER_HEADER_SIZE,
                                   (int64_t)(fileSize - MANI_CONTAINER_HEADER_SIZE));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
        FixIdentity(copy, size);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
