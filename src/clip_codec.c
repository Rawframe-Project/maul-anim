// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The compressed clip's arithmetic (mani-0019). Every operation is one
// binary32 rounding in a fixed order (contraction is off), so the
// compressor and the sampler reach the same bits on every platform.

#include "clip_codec.h"

#include <math.h>

// 1 / (2^bits - 1) for bits 1 to 23, rounded once at translation.
const float maniCodecInverse[24] = {
    0.0f,
    1.0f,
    1.0f / 3.0f,
    1.0f / 7.0f,
    1.0f / 15.0f,
    1.0f / 31.0f,
    1.0f / 63.0f,
    1.0f / 127.0f,
    1.0f / 255.0f,
    1.0f / 511.0f,
    1.0f / 1023.0f,
    1.0f / 2047.0f,
    1.0f / 4095.0f,
    1.0f / 8191.0f,
    1.0f / 16383.0f,
    1.0f / 32767.0f,
    1.0f / 65535.0f,
    1.0f / 131071.0f,
    1.0f / 262143.0f,
    1.0f / 524287.0f,
    1.0f / 1048575.0f,
    1.0f / 2097151.0f,
    1.0f / 4194303.0f,
    1.0f / 8388607.0f,
};

// round(v * max) for v clamped to [0, 1], halves away from zero.
static uint32_t Round(float normalized, uint32_t max)
{
    float v = normalized > 0.0f ? normalized : 0.0f;
    v = v < 1.0f ? v : 1.0f;
    float q = floorf(v * (float)max + 0.5f);
    return q < (float)max ? (uint32_t)q : max;
}

uint32_t maniCodecPack(float normalized, uint32_t bits)
{
    return Round(normalized, (1u << bits) - 1u);
}

void maniCodecSegmentRange(float minimum, float maximum, uint8_t* minimumOut, uint8_t* extentOut)
{
    float low = floorf(minimum * 255.0f);
    low = low > 0.0f ? (low < 255.0f ? low : 255.0f) : 0.0f;
    uint8_t q = (uint8_t)low;
    // One step lower when rounding put the minimum above the values.
    q = maniCodecRangeByte(q) <= minimum || q == 0 ? q : (uint8_t)(q - 1);
    float extent = maximum - maniCodecRangeByte(q);
    float high = ceilf(extent * 255.0f);
    high = high > 0.0f ? (high < 255.0f ? high : 255.0f) : 0.0f;
    uint8_t e = (uint8_t)high;
    e = maniCodecRangeByte(e) >= extent || e == 255 ? e : (uint8_t)(e + 1);
    // Minimum and extent within [0, 1] together; the values are, so this
    // still covers them.
    e = e <= 255 - q ? e : (uint8_t)(255 - q);
    *minimumOut = q;
    *extentOut = e;
}

uint16_t maniCodecPack16(float normalized)
{
    return (uint16_t)Round(normalized, 65535u);
}

float maniCodecUnpack16(uint16_t q)
{
    return (float)q * maniCodecInverse[16];
}

void maniBitWrite(maniBitWriter* w, uint32_t value, uint32_t bits)
{
    for (uint32_t i = 0; i < bits; ++i)
    {
        uint64_t at = w->bit + i;
        w->bytes[at >> 3] |= (uint8_t)(((value >> i) & 1u) << (at & 7u));
    }
    w->bit += bits;
}
