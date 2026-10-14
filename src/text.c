// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ranges are the Unicode Standard's table of well-formed UTF-8 byte
// sequences (chapter 3): the second byte's range depends on the first,
// which rules out overlong forms, surrogates and code points past
// U+10FFFF.

#include "text.h"

static bool InRange(uint8_t byte, uint8_t low, uint8_t high)
{
    return byte >= low && byte <= high;
}

// For a lead byte: how many continuation bytes follow, and the range of
// the first of them; 0 for an ASCII byte, -1 for a byte that cannot lead.
static int32_t Continuations(uint8_t lead, uint8_t* lowOut, uint8_t* highOut)
{
    *lowOut = 0x80;
    *highOut = 0xBF;
    if (lead < 0x80)
    {
        return 0;
    }
    if (InRange(lead, 0xC2, 0xDF))
    {
        return 1;
    }
    if (InRange(lead, 0xE0, 0xEF))
    {
        *lowOut = lead == 0xE0 ? 0xA0 : 0x80;
        *highOut = lead == 0xED ? 0x9F : 0xBF;
        return 2;
    }
    if (InRange(lead, 0xF0, 0xF4))
    {
        *lowOut = lead == 0xF0 ? 0x90 : 0x80;
        *highOut = lead == 0xF4 ? 0x8F : 0xBF;
        return 3;
    }
    return -1;
}

size_t maniUtf8Sequence(const uint8_t* bytes, size_t length)
{
    if (length == 0)
    {
        return 0;
    }
    uint8_t low = 0;
    uint8_t high = 0;
    int32_t count = Continuations(bytes[0], &low, &high);
    if (count < 0 || length - 1 < (size_t)count)
    {
        return 0;
    }
    if (count > 0 && !InRange(bytes[1], low, high))
    {
        return 0;
    }
    for (int32_t k = 2; k <= count; ++k)
    {
        if (!InRange(bytes[k], 0x80, 0xBF))
        {
            return 0;
        }
    }
    return (size_t)count + 1;
}

bool maniValidUtf8(const uint8_t* bytes, size_t length)
{
    size_t i = 0;
    while (i < length)
    {
        size_t step = maniUtf8Sequence(bytes + i, length - i);
        if (step == 0)
        {
            return false;
        }
        i += step;
    }
    return true;
}
