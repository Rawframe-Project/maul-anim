// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a clip file (mani-0006) and reading its named items; then the same bytes with the
// payload's hash made right, so that mutations reach the checks behind it. Run as fuzz_clip <corpus
// dir>; the seed comes from test_clip --seed <file>.

#include "maul-anim/clip.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniClip clip;
    if (maniOpenClip(bytes, size, &clip).result == 0)
    {
        if (maniClipSampleCount(&clip) == 0 || !(maniClipDuration(&clip) >= 0.0f) ||
            !(maniClipSampleRate(&clip) > 0.0f))
        {
            abort();
        }
        for (uint32_t i = 0; i < maniClipFloatTrackCount(&clip); ++i)
        {
            uint32_t length = 0;
            const char* name = maniClipFloatTrackName(&clip, i, &length);
            if (maniClipFindFloatTrack(&clip, name, length) != i)
            {
                abort();
            }
        }
        for (uint32_t i = 0; i < maniClipDiscreteTrackCount(&clip); ++i)
        {
            uint32_t length = 0;
            const char* name = maniClipDiscreteTrackName(&clip, i, &length);
            if (maniClipFindDiscreteTrack(&clip, name, length) != i)
            {
                abort();
            }
            (void)maniClipDiscreteValue(&clip, i, maniClipDuration(&clip) * 0.5f);
        }
        uint32_t first = 0;
        uint32_t count = 0;
        maniClipEventRange(&clip, 0.0f, maniClipDuration(&clip), &first, &count);
        if (first + count > maniClipEventCount(&clip))
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
    if (size >= 64)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= 64 && fileSize <= size && fileSize - 64 < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + 64, (int32_t)(fileSize - 64));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
