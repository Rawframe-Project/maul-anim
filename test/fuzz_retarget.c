// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a retargeting map, and on success reading every
// target bone's plan: its source bones within the source's count (or
// none), its kind known; then the same bytes with the payload's hash
// made right, and then the identity too, so that mutations reach the
// checks behind them. Run as fuzz_retarget <corpus dir>; the seed comes
// from test_retarget_map --seed <file>. After each open, up to 16 of the
// file's bytes change and every call that needs no skeleton runs again:
// it reads only within the file and ends, whatever it gives.

#include "bytes.h"
#include "container.h"
#include "retarget_file.h"

#include "maul-anim/retarget.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
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

// The calls that need no skeleton.
static void Calls(const maniRetargetMap* map)
{
    (void)maniRetargetMap_GetIdentity(map);
    (void)maniRetargetMap_GetScratchSize(map);
    (void)maniRetargetMap_RetargetRootMotion(map, (maniVec3){0.5f, 2.0f, -3.0f});
}

static void Exercise(uint8_t* bytes, size_t size)
{
    maniRetargetMap map;
    if (maniOpenRetargetMap(bytes, size, &map).result != 0)
    {
        return;
    }
    Calls(&map);
    Changed c = Change(bytes, map.size);
    Calls(&map);
    Restore(bytes, &c);
    for (uint32_t b = 0; b < map.targetBoneCount; ++b)
    {
        const uint8_t* e = map.base + map.sections[2] + (size_t)b * MANI_RETARGET_PLAN_SIZE;
        uint32_t kind = maniLoadU32(e + maniRetargetPlanKind);
        uint32_t a = maniLoadU32(e + maniRetargetPlanA);
        uint32_t two = maniLoadU32(e + maniRetargetPlanB);
        bool aOk = kind == maniRetargetPlanReference ? a == MANI_NO_BONE : a < map.sourceBoneCount;
        bool bOk = kind == maniRetargetPlanTwo ? two < map.sourceBoneCount : two == MANI_NO_BONE;
        if (kind > maniRetargetPlanTwo || !aOk || !bOk)
        {
            abort();
        }
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindRetargetMap, MANI_RETARGET_FORMAT_VERSION,
                          MANI_RETARGET_SECTIONS, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_RETARGET_SECTIONS];
    uint64_t sizes[MANI_RETARGET_SECTIONS];
    for (uint32_t t = 1; t <= MANI_RETARGET_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniRetargetComputeIdentity(sections, sizes);
    memcpy(bytes + 40, &identity, sizeof identity);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    // Opened from a 16-byte aligned copy, aligned by hand.
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
        hash = maniHash64(MANI_HASH_INIT, copy + MANI_CONTAINER_HEADER_SIZE,
                          (int64_t)(fileSize - MANI_CONTAINER_HEADER_SIZE));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
