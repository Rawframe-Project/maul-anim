// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a skin file, and on success reading every joint;
// then the same bytes with the payload's hash made right, so that
// mutations reach the checks behind it. Run as fuzz_skin <corpus dir>;
// the seed comes from test_skin --seed <file>.

#include "container.h"

#include "maul-anim/skin.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

// Up to 16 bytes inside an opened file changed, picked from its own
// bytes, as a host's memory may change after the open: every call after
// reads only within the file and ends, whatever it gives. Restore puts
// them back.
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

static void Exercise(uint8_t* bytes, size_t size)
{
    maniSkin skin;
    if (maniOpenSkin(bytes, size, &skin).result == 0)
    {
        uint32_t count = maniSkin_GetJointCount(&skin);
        for (uint32_t joint = 0; joint < count; ++joint)
        {
            maniAffine bind = maniSkin_GetInverseBind(&skin, joint);
            if (maniSkin_GetJoint(&skin, joint) >= skin.skeletonBoneCount || bind.m[0] != bind.m[0])
            {
                abort();
            }
        }
        Changed c = Change(bytes, skin.size);
        for (uint32_t joint = 0; joint < count; ++joint)
        {
            (void)maniSkin_GetJoint(&skin, joint);
            (void)maniSkin_GetInverseBind(&skin, joint);
        }
        Restore(bytes, &c);
    }
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
    }
    free(block);
    return 0;
}
