// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a spring set (mani-0026), and on success reading every
// spring's name and every joint, link and collider record within the
// counts; then the same bytes with the payload's hash made right, and
// then the identity too, so that mutations reach the checks behind
// them. Run as fuzz_spring <corpus dir>; the seed comes from test_spring
// --seed <file>.

#include "bytes.h"
#include "container.h"
#include "spring_file.h"

#include "maul-anim/spring.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniSpringSet set;
    if (maniOpenSpringSet(bytes, size, &set).result != 0)
    {
        return;
    }
    uint64_t joints = 0;
    uint64_t links = 0;
    for (uint32_t i = 0; i < maniSpringSetSpringCount(&set); ++i)
    {
        uint32_t length = 0;
        const uint8_t* e = set.springs + (size_t)i * MANI_SPRING_SPRING_SIZE;
        if (maniSpringSetSpringName(&set, i, &length) == NULL || length > set.nameBytes)
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
        if (maniLoadU32(set.links + 4 * (size_t)k) >= set.colliderCount)
        {
            abort();
        }
    }
    for (uint32_t j = 0; j < set.jointCount; ++j)
    {
        const uint8_t* e = set.joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
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
    if (maniOpenContainer(bytes, size, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION, &c).result !=
        0)
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
