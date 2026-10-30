// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Outputs a refused call or a "no" clears: a suite fills an output with a
// sentinel byte, makes the call, and requires every byte zero, padding
// included, so a value left from an earlier call is never read as the
// answer.

#ifndef MAUL_ANIM_TEST_CLEARED_H
#define MAUL_ANIM_TEST_CLEARED_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The byte an output is filled with before the call.
#define CLEARED_SENTINEL 0xA5

static inline void Fill(void* bytes, size_t size)
{
    memset(bytes, CLEARED_SENTINEL, size);
}

static inline bool Cleared(const void* bytes, size_t size)
{
    const uint8_t* b = (const uint8_t*)bytes;
    for (size_t i = 0; i < size; ++i)
    {
        if (b[i] != 0)
        {
            return false;
        }
    }
    return true;
}

#endif
