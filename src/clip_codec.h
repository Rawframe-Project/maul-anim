// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The compressed clip's arithmetic: bit rates, quantization,
// segment ranges and the bit stream, shared by the compressor and the
// sampler so that both compute the same binary32 values.

#ifndef MAUL_ANIM_SRC_CLIP_CODEC_H
#define MAUL_ANIM_SRC_CLIP_CODEC_H

#include "bytes.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Bit rates 0 to 24: 0 a value constant in its segment (kept in the
// segment range), 1 to 23 that many bits, 24 a raw binary32.
#define MANI_CODEC_RATES    25u
#define MANI_CODEC_RAW_RATE 24u

// The bits a component takes at a rate.
static inline uint32_t maniCodecBits(uint32_t rate)
{
    return rate == MANI_CODEC_RAW_RATE ? 32u : rate;
}

// A value in [0, 1] on bits (1 to 23): round(v * (2^bits - 1)), halves
// away from zero.
uint32_t maniCodecPack(float normalized, uint32_t bits);

// 1 / (2^bits - 1) for bits 1 to 23 (index 0 unused), rounded once.
extern const float maniCodecInverse[24];

// The value of q on bits (1 to 23): q times 1 / (2^bits - 1), in
// binary32.
static inline float maniCodecUnpack(uint32_t q, uint32_t bits)
{
    return (float)q * maniCodecInverse[bits];
}

// A segment range of clip-normalized values (in [0, 1]) on 8 bits: the
// minimum rounded down and the extent up, a step further when rounding
// alone would not cover the values.
void maniCodecSegmentRange(float minimum, float maximum, uint8_t* minimumOut, uint8_t* extentOut);

// A segment range's byte as a value: q / 255 in binary32.
static inline float maniCodecRangeByte(uint8_t q)
{
    return (float)q * maniCodecInverse[8];
}

// A value constant in its segment, clip-normalized, on 16 bits, and
// back.
uint16_t maniCodecPack16(float normalized);
float maniCodecUnpack16(uint16_t q);

// A segment-normalized value as a clip-normalized one: the segment's
// minimum plus the value times its extent.
static inline float maniCodecSegmentValue(float normalized, uint8_t minimum, uint8_t extent)
{
    return maniCodecRangeByte(minimum) + normalized * maniCodecRangeByte(extent);
}

// A clip-normalized value as the value: times the extent plus the
// minimum.
static inline float maniCodecValue(float normalized, float minimum, float extent)
{
    return normalized * extent + minimum;
}

// A rotation's dropped component from the three kept:
// sqrt(|((1 - a*a) - b*b) - c*c|).
static inline float maniCodecRebuild(float a, float b, float c)
{
    float squared = ((1.0f - a * a) - b * b) - c * c;
    return sqrtf(fabsf(squared));
}

// A rotation's x, y, z and w from its kept components (in increasing
// index order) and the dropped one's index (0 x, 1 y, 2 z, 3 w).
static inline void maniCodecRotation(const float kept[3], uint32_t dropped, float out[4])
{
    float a = kept[0];
    float b = kept[1];
    float c = kept[2];
    float r = maniCodecRebuild(a, b, c);
    out[0] = dropped == 0 ? r : a;
    out[1] = dropped == 0 ? a : (dropped == 1 ? r : b);
    out[2] = dropped <= 1 ? b : (dropped == 2 ? r : c);
    out[3] = dropped == 3 ? r : c;
}

// A rotation made unit: divided by its length, as one reciprocal, in
// binary32; the identity for a length 0 or past binary32's range.
static inline void maniCodecNormalize(float q[4])
{
    float squared = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
    if (!(squared > 0.0f && squared <= FLT_MAX))
    {
        q[0] = 0.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 1.0f;
        return;
    }
    float inverse = 1.0f / sqrtf(squared);
    for (int k = 0; k < 4; ++k)
    {
        q[k] *= inverse;
    }
}

// A bit stream written least significant bit first into bytes the
// caller zeroed.
typedef struct maniBitWriter
{
    uint8_t* bytes;
    uint64_t bit;
} maniBitWriter;

void maniBitWrite(maniBitWriter* w, uint32_t value, uint32_t bits);

// The bits (at most 32) at a bit offset of a stream with eight readable
// bytes past its last bit.
static inline uint32_t maniBitRead(const uint8_t* bytes, uint64_t bit, uint32_t bits)
{
    uint64_t word = maniLoadU64(bytes + (bit >> 3));
    uint64_t mask = bits == 32 ? 0xFFFFFFFFu : ((uint64_t)1 << bits) - 1u;
    return (uint32_t)((word >> (bit & 7u)) & mask);
}

#endif // MAUL_ANIM_SRC_CLIP_CODEC_H
