// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening an influences file, and on success reading every
// influence, blend weight and center; then the same bytes with the
// payload's hash made right, so that mutations reach the checks behind
// it. Run as fuzz_influences <corpus dir>; the seed comes from
// test_influences --seed <file>.

#include "container.h"

#include "maul-anim/skinning.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniInfluences v;
    if (maniOpenInfluences(bytes, size, &v).result != 0)
    {
        return;
    }
    uint32_t per = maniInfluences_GetPerVertex(&v);
    uint32_t vertices = maniInfluences_GetVertexCount(&v);
    for (uint32_t vertex = 0; vertex < vertices && vertex < 4096; ++vertex)
    {
        for (uint32_t slot = 0; slot < per; ++slot)
        {
            float w = 0.0f;
            if (maniInfluences_GetInfluence(&v, vertex, slot, &w) >= v.skinJointCount ||
                !(w >= 0.0f) || !(w <= 1.0f))
            {
                abort();
            }
        }
        float b = maniInfluences_GetDualQuaternionWeight(&v, vertex);
        maniVec3 c;
        if (!(b >= 0.0f && b <= 1.0f) || (maniInfluences_GetCenter(&v, vertex, &c) && c.x != c.x))
        {
            abort();
        }
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
    if (fileSize >= MANI_CONTAINER_HEADER_SIZE && fileSize <= size &&
        fileSize - MANI_CONTAINER_HEADER_SIZE < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + MANI_CONTAINER_HEADER_SIZE,
                                   (int32_t)(fileSize - MANI_CONTAINER_HEADER_SIZE));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
