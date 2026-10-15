// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The compressed clip's arithmetic (mani-0006): bit rates, quantization,
// segment ranges and the bit stream, shared by the compressor and the
// sampler so that both compute the same binary32 values.

#ifndef MAUL_ANIM_SRC_CLIP_CODEC_H
#define MAUL_ANIM_SRC_CLIP_CODEC_H

#include "bytes.h"

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

// The value of q on bits (1 to 23): q times 1 / (2^bits - 1), in
// binary32.
float maniCodecUnpack(uint32_t q, uint32_t bits);

// A segment range of clip-normalized values (in [0, 1]) on 8 bits: the
// minimum rounded down and the extent up, a step further when rounding
// alone would not cover the values.
void maniCodecSegmentRange(float minimum, float maximum, uint8_t* minimumOut, uint8_t* extentOut);

// A segment range's byte as a value: q / 255 in binary32.
float maniCodecRangeByte(uint8_t q);

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

// A rotation's w from x, y and z: sqrt(|((1 - x*x) - y*y) - z*z|).
float maniCodecRebuildW(float x, float y, float z);

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
