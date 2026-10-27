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

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniSkin skin;
    if (maniOpenSkin(bytes, size, &skin).result == 0)
    {
        uint32_t count = maniSkinJointCount(&skin);
        for (uint32_t joint = 0; joint < count; ++joint)
        {
            maniAffine bind = maniSkinInverseBind(&skin, joint);
            if (maniSkinJoint(&skin, joint) >= skin.skeletonBoneCount || bind.m[0] != bind.m[0])
            {
                abort();
            }
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
