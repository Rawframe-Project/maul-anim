// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a ragdoll profile, and on success reading every
// body: bones within the count and increasing, parents before children,
// names within the name bytes; then the same bytes with the payload's
// hash made right, and then the identity too, so that mutations reach
// the checks behind them. Run as fuzz_ragdoll <corpus dir>; the seed
// comes from test_ragdoll --seed <file>.

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

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniRagdoll ragdoll;
    if (maniOpenRagdoll(bytes, size, &ragdoll).result != 0)
    {
        return;
    }
    uint64_t names = 0;
    for (uint32_t i = 0; i < maniRagdoll_GetBodyCount(&ragdoll); ++i)
    {
        maniRagdollBody b;
        if (maniRagdoll_GetBody(&ragdoll, i, &b) != 0 || b.bone >= ragdoll.boneCount ||
            (i == 0) != (b.parent == MANI_NO_BONE) || (i > 0 && b.parent >= i) ||
            b.name != (const char*)ragdoll.names + names || !(b.mass > 0.0f))
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
    if (maniOpenContainer(bytes, size, maniKindRagdoll, MANI_RAGDOLL_FORMAT_VERSION, &c).result !=
        0)
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
