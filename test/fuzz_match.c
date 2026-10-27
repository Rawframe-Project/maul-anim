// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a motion database, and on success reading every
// channel, entry and frame; then the same bytes with the payload's hash
// made right, and then the identity too, so that mutations reach the
// checks behind them. Run as fuzz_match <corpus dir>; the seed comes from
// test_match --seed <file>.

#include "container.h"
#include "match_file.h"

#include "maul-anim/match.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniMotionDatabase d;
    if (maniOpenMotionDatabase(bytes, size, &d).result != 0)
    {
        return;
    }
    for (uint32_t c = 0; c < d.channelCount; ++c)
    {
        maniMatchChannel ch;
        if (maniMotionDatabaseChannel(&d, c, &ch) != 0 || ch.timeCount == 0 ||
            ch.firstFeature + ch.featureCount > d.featureCount)
        {
            abort();
        }
    }
    float features[MANI_MATCH_MAX_FEATURES];
    for (uint32_t f = 0; f < d.frameCount; ++f)
    {
        uint32_t entry = 0;
        float time = 0.0f;
        maniMatchEntry e;
        if (maniMotionDatabaseFrame(&d, f, &entry, &time) != 0 ||
            maniMotionDatabaseEntry(&d, entry, &e) != 0 || f < e.firstFrame ||
            f >= e.firstFrame + e.frameCount || maniMotionDatabaseFeatures(&d, f, features) != 0)
        {
            abort();
        }
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION, &c)
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
    if (fileSize >= MANI_CONTAINER_HEADER_SIZE && fileSize <= size &&
        fileSize - MANI_CONTAINER_HEADER_SIZE < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + MANI_CONTAINER_HEADER_SIZE,
                                   (int32_t)(fileSize - MANI_CONTAINER_HEADER_SIZE));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
        FixIdentity(copy, size);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
