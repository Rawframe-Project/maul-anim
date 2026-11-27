// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a ragdoll profile, and on success reading every
// body: bones within the count and increasing, parents before children,
// names within the name bytes; then the same bytes with the payload's
// hash made right, and then the identity too, so that mutations reach
// the checks behind them. Run as fuzz_ragdoll <corpus dir>; the seed
// comes from test_ragdoll --seed <file>. After each open, up to 16 of
// the file's bytes change and every body is read again: within the file,
// whatever it gives.

#include "bytes.h"
#include "container.h"
#include "ragdoll_file.h"

#include "maul-anim/ragdoll.h"
#include "maul-anim/skeleton.h"

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

// Every body read, its name's bytes within the names, its parent before
// it, its bone within the skeleton or none.
static void Bodies(const maniRagdoll* ragdoll)
{
    const char* names = (const char*)ragdoll->base + ragdoll->sections[1];
    for (uint32_t i = 0; i < maniRagdoll_GetBodyCount(ragdoll); ++i)
    {
        maniRagdollBody b;
        if (maniRagdoll_GetBody(ragdoll, i, &b) != 0 || b.name < names ||
            b.nameLength > ragdoll->nameBytes ||
            (uint64_t)(b.name - names) > ragdoll->nameBytes - b.nameLength ||
            (i == 0) != (b.parent == MANI_NO_BONE) || (i > 0 && b.parent >= i) ||
            (b.bone != MANI_NO_BONE && b.bone >= ragdoll->boneCount))
        {
            abort();
        }
    }
}

static void Exercise(uint8_t* bytes, size_t size)
{
    maniRagdoll ragdoll;
    if (maniOpenRagdoll(bytes, size, &ragdoll).result != 0)
    {
        return;
    }
    Changed c = Change(bytes, ragdoll.size);
    Bodies(&ragdoll);
    Restore(bytes, &c);
    uint64_t names = 0;
    for (uint32_t i = 0; i < maniRagdoll_GetBodyCount(&ragdoll); ++i)
    {
        maniRagdollBody b;
        if (maniRagdoll_GetBody(&ragdoll, i, &b) != 0 || b.bone >= ragdoll.boneCount ||
            (i == 0) != (b.parent == MANI_NO_BONE) || (i > 0 && b.parent >= i) ||
            b.name != (const char*)ragdoll.base + ragdoll.sections[1] + names || !(b.mass > 0.0f))
        {
            abort();
        }
        names += b.nameLength;
    }
    if (names != ragdoll.nameBytes)
    {
        abort();
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindRagdoll, MANI_RAGDOLL_FORMAT_VERSION,
                          MANI_RAGDOLL_SECTIONS, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_RAGDOLL_SECTIONS];
    uint64_t sizes[MANI_RAGDOLL_SECTIONS];
    for (uint32_t t = 1; t <= MANI_RAGDOLL_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniRagdollComputeIdentity(sections, sizes);
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
