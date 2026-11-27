// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a mirror map, and on success reading every bone's
// partner (within the count, pairing back) and every name pair's
// partner; then the same bytes with the payload's hash made right, and
// then the identity too, so that mutations reach the checks behind them.
// After each open, up to 16 of the file's bytes change and every call
// runs again: it reads only within the file and ends, whatever it gives.
// Run as fuzz_mirror <corpus dir>; the seed comes from test_mirror_map
// --seed <file>.

#include "bytes.h"
#include "container.h"
#include "mirror_file.h"

#include "maul-anim/mirror.h"
#include "maul-anim/rootmotion.h"
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

// Every call on the map; with checked set, what an unchanged file must
// give: partners within the count pairing back, and every partner's name
// inside the names.
static void Calls(const maniMirrorMap* map, bool checked)
{
    uint32_t bones = map->boneCount;
    for (uint32_t b = 0; b < bones; ++b)
    {
        uint32_t partner = maniMirrorMap_GetPartner(map, b);
        (void)maniMirrorMap_GetBoneFlags(map, b);
        if (checked && (partner >= bones || maniMirrorMap_GetPartner(map, partner) != b))
        {
            abort();
        }
    }
    (void)maniMirrorMap_GetUnmatchedBone(map);
    (void)maniMirrorMap_MirrorRootMotion(
        map, (maniRootMove){{1.0f, 2.0f, 3.0f}, {0.0f, 0.6f, 0.0f, 0.8f}});
    const char* strings = (const char*)map->base + map->sections[3];
    for (uint32_t i = 0; i < map->nameCount; ++i)
    {
        const uint8_t* e = map->base + map->sections[2] + (size_t)i * MANI_MIRROR_NAME_SIZE;
        (void)maniMirrorMap_MirrorNameHash(map, maniLoadU64(e));
        uint64_t offset = maniLoadU32(e + 16);
        uint64_t length = maniLoadU32(e + 20);
        if (offset > map->stringsSize || length > map->stringsSize - offset)
        {
            continue;
        }
        size_t out = 0;
        const char* name = strings + offset;
        const char* partner = maniMirrorMap_MirrorName(map, name, (size_t)length, &out);
        if (partner == NULL ||
            (partner != name && (partner < strings || out > map->stringsSize ||
                                 (uint64_t)(partner - strings) > map->stringsSize - out)))
        {
            abort();
        }
    }
}

static void Exercise(uint8_t* bytes, size_t size)
{
    maniMirrorMap map;
    if (maniOpenMirrorMap(bytes, size, &map).result != 0)
    {
        return;
    }
    Calls(&map, true);
    Changed c = Change(bytes, map.size);
    Calls(&map, false);
    Restore(bytes, &c);
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindMirrorMap, MANI_MIRROR_FORMAT_VERSION,
                          MANI_MIRROR_SECTIONS, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_MIRROR_SECTIONS];
    uint64_t sizes[MANI_MIRROR_SECTIONS];
    for (uint32_t t = 1; t <= MANI_MIRROR_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniMirrorComputeIdentity(sections, sizes);
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
