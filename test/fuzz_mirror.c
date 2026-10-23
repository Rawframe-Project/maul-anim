// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a mirror map, and on success reading every bone's
// partner (within the count, pairing back) and every name pair's
// partner; then the same bytes with the payload's hash made right, and
// then the identity too, so that mutations reach the checks behind them.
// Run as fuzz_mirror <corpus dir>; the seed comes from test_mirror_map
// --seed <file>.

#include "bytes.h"
#include "container.h"
#include "mirror_file.h"

#include "maul-anim/mirror.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniMirrorMap map;
    if (maniOpenMirrorMap(bytes, size, &map).result != 0)
    {
        return;
    }
    for (uint32_t b = 0; b < map.boneCount; ++b)
    {
        uint32_t partner = maniMirrorPartner(&map, b);
        if (partner >= map.boneCount || maniMirrorPartner(&map, partner) != b)
        {
            abort();
        }
    }
    for (uint32_t i = 0; i < map.nameCount; ++i)
    {
        const uint8_t* e = map.names + (size_t)i * MANI_MIRROR_NAME_SIZE;
        size_t length = 0;
        const char* name = (const char*)map.strings + maniLoadU32(e + 16);
        const char* partner = maniMirrorName(&map, name, maniLoadU32(e + 20), &length);
        if (partner == NULL ||
            (partner != name && (partner < (const char*)map.strings ||
                                 partner + length > (const char*)map.strings + map.stringsSize)))
        {
            abort();
        }
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindMirrorMap, MANI_MIRROR_FORMAT_VERSION, &c).result !=
        0)
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
    if (size >= 64)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= 64 && fileSize <= size && fileSize - 64 < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + 64, (int32_t)(fileSize - 64));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
        FixIdentity(copy, size);
        hash = maniHash64(MANI_HASH_INIT, copy + 64, (int32_t)(fileSize - 64));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
