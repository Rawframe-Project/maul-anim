// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The builders' stable sort: bottom-up merge sort of indices.

#include "sort.h"

#include <stdbool.h>
#include <string.h>

void maniSortByKey(const uint64_t* keys, uint32_t n, uint32_t* sorted, uint32_t* scratch)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        sorted[i] = i;
    }
    uint32_t* from = sorted;
    uint32_t* to = scratch;
    for (uint32_t width = 1; width < n; width *= 2)
    {
        for (uint32_t low = 0; low < n; low += 2 * width)
        {
            uint32_t middle = low + width < n ? low + width : n;
            uint32_t high = middle + width < n ? middle + width : n;
            uint32_t a = low;
            uint32_t b = middle;
            for (uint32_t k = low; k < high; ++k)
            {
                bool takeA = a < middle && (b >= high || keys[from[a]] <= keys[from[b]]);
                to[k] = takeA ? from[a++] : from[b++];
            }
        }
        uint32_t* swap = from;
        from = to;
        to = swap;
    }
    if (from != sorted)
    {
        memcpy(sorted, from, (size_t)n * sizeof(uint32_t));
    }
}

uint64_t maniFloatKey(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x80000000u) != 0 ? (uint64_t)~bits : (uint64_t)(bits | 0x80000000u);
}
