// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a retargeting map, and on success reading every
// target bone's plan: its source bones within the source's count (or
// none), its kind known; then the same bytes with the payload's hash
// made right, and then the identity too, so that mutations reach the
// checks behind them. Run as fuzz_retarget <corpus dir>; the seed comes
// from test_retarget_map --seed <file>.

#include "bytes.h"
#include "container.h"
#include "retarget_file.h"

#include "maul-anim/retarget.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniRetargetMap map;
    if (maniOpenRetargetMap(bytes, size, &map).result != 0)
    {
        return;
    }
    for (uint32_t b = 0; b < map.targetBoneCount; ++b)
    {
        const uint8_t* e = map.plan + (size_t)b * MANI_RETARGET_PLAN_SIZE;
        uint32_t kind = maniLoadU32(e + maniRetargetPlanKind);
        uint32_t a = maniLoadU32(e + maniRetargetPlanA);
        uint32_t two = maniLoadU32(e + maniRetargetPlanB);
        bool aOk = kind == maniRetargetPlanReference ? a == MANI_NO_BONE : a < map.sourceBoneCount;
        bool bOk = kind == maniRetargetPlanTwo ? two < map.sourceBoneCount : two == MANI_NO_BONE;
        if (kind > maniRetargetPlanTwo || !aOk || !bOk)
        {
            abort();
        }
    }
}

// The identity the sections give, written where the header keeps it.
static void FixIdentity(uint8_t* bytes, size_t size)
{
    maniContainer c;
    if (maniOpenContainer(bytes, size, maniKindRetargetMap, MANI_RETARGET_FORMAT_VERSION, &c)
            .result != 0)
    {
        return;
    }
    static const uint8_t kEmpty = 0;
    const uint8_t* sections[MANI_RETARGET_SECTIONS];
    uint64_t sizes[MANI_RETARGET_SECTIONS];
    for (uint32_t t = 1; t <= MANI_RETARGET_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    uint64_t identity = maniRetargetComputeIdentity(sections, sizes);
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
