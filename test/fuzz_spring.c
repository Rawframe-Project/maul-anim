// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a spring set, and on success reading every
// spring's name and every joint, link and collider record within the
// counts; then the same bytes with the payload's hash made right, and
// then the identity too, so that mutations reach the checks behind
// them. Run as fuzz_spring <corpus dir>; the seed comes from test_spring
// --seed <file>. After each open, up to 16 of the file's bytes change and
// every name is read again: within the names, whatever it gives.

#include "bytes.h"
#include "container.h"
#include "spring_file.h"

#include "maul-anim/spring.h"

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

// Every spring's name within the names.
static void Names(const maniSpringSet* set)
{
    const char* names = (const char*)set->base + set->sections[4];
    for (uint32_t i = 0; i < maniSpringSet_GetSpringCount(set); ++i)
    {
        size_t length = 0;
        const char* name = maniSpringSet_GetSpringName(set, i, &length);
        if (name == NULL || name < names || length > set->nameBytes ||
            (uint64_t)(name - names) > set->nameBytes - length)
        {
            abort();
        }
    }
}

static void Exercise(uint8_t* bytes, size_t size)
{
    maniSpringSet set;
    if (maniOpenSpringSet(bytes, size, &set).result != 0)
    {
        return;
    }
    Changed c = Change(bytes, set.size);
    Names(&set);
    (void)maniSpringSet_GetStateSize(&set);
    Restore(bytes, &c);
    uint64_t joints = 0;
    uint64_t links = 0;
    for (uint32_t i = 0; i < maniSpringSet_GetSpringCount(&set); ++i)
    {
        size_t length = 0;
        const uint8_t* e = set.base + set.sections[1] + (size_t)i * MANI_SPRING_SPRING_SIZE;
        if (maniSpringSet_GetSpringName(&set, i, &length) == NULL || length > set.nameBytes)
        {
            abort();
        }
        joints += maniLoadU32(e + 4);
        links += maniLoadU32(e + 12);
    }
    if (joints != set.jointCount || links != set.linkCount)
    {
        abort();
    }
    for (uint32_t k = 0; k < set.linkCount; ++k)
    {
        if (maniLoadU32(set.base + set.sections[3] + 4 * (size_t)k) >= set.colliderCount)
        {
            abort();
        }
    }
    for (uint32_t j = 0; j < set.jointCount; ++j)
    {
        const uint8_t* e = set.base + set.sections[2] + (size_t)j * MANI_SPRING_JOINT_SIZE;
        if (maniLoadU32(e + maniSpringJointHead) >= set.boneCount ||
            maniLoadU32(e + maniSpringJointTail) >= set.boneCount)
        {
            abort();
        }
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION,
                          MANI_SPRING_SECTIONS, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_SPRING_SECTIONS];
    uint64_t sizes[MANI_SPRING_SECTIONS];
    for (uint32_t t = 1; t <= MANI_SPRING_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniSpringComputeIdentity(sections, sizes);
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
