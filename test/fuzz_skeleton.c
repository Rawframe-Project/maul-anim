// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a skeleton file, and on success reading every bone
// and finding each by its name; then the same bytes with the payload's
// hash made right, so that mutations reach the checks behind it. Run as fuzz_skeleton <corpus dir>;
// the seed comes from test_skeleton --seed <file>.

#include "container.h"

#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniSkeleton skeleton;
    if (maniOpenSkeleton(bytes, size, &skeleton).result == 0)
    {
        uint32_t count = maniSkeleton_GetBoneCount(&skeleton);
        for (uint32_t bone = 0; bone < count; ++bone)
        {
            uint32_t parent = maniSkeleton_GetParent(&skeleton, bone);
            uint32_t end = maniSkeleton_GetSubtreeEnd(&skeleton, bone);
            maniTransform rest = maniSkeleton_GetRest(&skeleton, bone);
            size_t length = 0;
            const char* name = maniSkeleton_GetBoneName(&skeleton, bone, &length);
            if ((parent != MANI_NO_BONE && parent >= bone) || end <= bone || end > count ||
                rest.rotation.w != rest.rotation.w ||
                maniSkeleton_FindBone(&skeleton, name, length) != bone)
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
