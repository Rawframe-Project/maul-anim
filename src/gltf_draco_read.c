// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Values are little-endian. rANS reads its bytes from their end: the
// last byte's top two bits give how many bytes start the state (research
// 59: the specification's tables, its bit order and its symbol layout
// differ from the files Google's encoder writes, which decide). A symbol
// table holds only the symbols of non-zero probability, searched by
// their cumulative probabilities, so its size follows the stream's
// bytes, not the precision or a count of zeros. The tagged coding's
// values are bits read least significant first after the tags; a stream
// that ends before them is invalid.

#include "gltf_draco_read.h"

#include "bytes.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IO_BASE 256u
// The tagged coding's tags: 2^12 precision (Google's 5-bit symbols).
#define TAG_BITS 12u

bool maniDracoU8(maniDracoReader* r, uint8_t* out)
{
    if (r->at >= r->end)
    {
        return false;
    }
    *out = *r->at++;
    return true;
}

bool maniDracoU16(maniDracoReader* r, uint16_t* out)
{
    if (maniDracoLeft(r) < 2)
    {
        return false;
    }
    *out = maniLoadU16(r->at);
    r->at += 2;
    return true;
}

bool maniDracoU32(maniDracoReader* r, uint32_t* out)
{
    if (maniDracoLeft(r) < 4)
    {
        return false;
    }
    *out = maniLoadU32(r->at);
    r->at += 4;
    return true;
}

static bool Leb(maniDracoReader* r, uint32_t bytes, uint64_t* out)
{
    uint64_t v = 0;
    for (uint32_t i = 0; i < bytes; ++i)
    {
        uint8_t b = 0;
        if (!maniDracoU8(r, &b))
        {
            return false;
        }
        v |= (uint64_t)(b & 0x7Fu) << (7u * i);
        if ((b & 0x80u) == 0)
        {
            *out = v;
            return true;
        }
    }
    return false;
}

bool maniDracoLeb32(maniDracoReader* r, uint32_t* out)
{
    uint64_t v = 0;
    if (!Leb(r, 5, &v) || v > UINT32_MAX)
    {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

bool maniDracoLeb64(maniDracoReader* r, uint64_t* out)
{
    return Leb(r, 10, out);
}

bool maniDracoBytes(maniDracoReader* r, size_t size, const uint8_t** out)
{
    if (maniDracoLeft(r) < size)
    {
        return false;
    }
    *out = r->at;
    r->at += size;
    return true;
}

// One table entry: a probability of 6 to 30 bits, or a run of zeros.
static bool Entry(maniDracoReader* r, uint32_t* probability, uint32_t* zeros)
{
    uint8_t b = 0;
    if (!maniDracoU8(r, &b))
    {
        return false;
    }
    uint32_t token = b & 3u;
    *zeros = token == 3u ? (uint32_t)(b >> 2) + 1u : 0u;
    uint32_t p = b >> 2;
    for (uint32_t k = 0; token != 3u && k < token; ++k)
    {
        uint8_t e = 0;
        if (!maniDracoU8(r, &e))
        {
            return false;
        }
        p |= (uint32_t)e << (8u * (k + 1u) - 2u);
    }
    *probability = token == 3u ? 0u : p;
    return true;
}

maniDracoStatus maniDracoRansTable(maniDracoReader* r, maniArena* arena, uint32_t bits,
                                   maniDracoRans* out)
{
    *out = (maniDracoRans){0};
    out->bits = bits;
    out->base = 4u << bits;
    uint32_t symbols = 0;
    if (!maniDracoLeb32(r, &symbols) || symbols / 64u > maniDracoLeft(r))
    {
        return maniDracoInvalid;
    }
    // Every symbol of non-zero probability costs a byte at least.
    size_t most = symbols < maniDracoLeft(r) ? symbols : maniDracoLeft(r);
    uint32_t* values = maniArenaTake(arena, most, sizeof(uint32_t));
    uint32_t* probabilities = maniArenaTake(arena, most, sizeof(uint32_t));
    uint32_t* cumulative = maniArenaTake(arena, most, sizeof(uint32_t));
    if (values == NULL || probabilities == NULL || cumulative == NULL)
    {
        return maniDracoCapacity;
    }
    uint64_t sum = 0;
    uint32_t count = 0;
    for (uint32_t i = 0; i < symbols;)
    {
        uint32_t p = 0;
        uint32_t zeros = 0;
        if (!Entry(r, &p, &zeros) || (zeros > 0 && zeros > symbols - i))
        {
            return maniDracoInvalid;
        }
        if (zeros == 0 && p > 0)
        {
            values[count] = i;
            probabilities[count] = p;
            cumulative[count++] = (uint32_t)sum;
            sum += p;
        }
        if (sum > (1u << bits))
        {
            return maniDracoInvalid;
        }
        i += zeros > 0 ? zeros : 1u;
    }
    if (symbols > 0 && sum != (1u << bits))
    {
        return maniDracoInvalid;
    }
    out->values = values;
    out->probabilities = probabilities;
    out->cumulative = cumulative;
    out->count = count;
    return maniDracoDone;
}

bool maniDracoRansStart(maniDracoReader* r, maniDracoRans* rans)
{
    uint64_t size = 0;
    const uint8_t* bytes = NULL;
    if (!maniDracoLeb64(r, &size) || size > maniDracoLeft(r) || size < 1 ||
        !maniDracoBytes(r, (size_t)size, &bytes))
    {
        return false;
    }
    // The state's first 1 to 4 bytes, less their last byte's two top bits.
    uint32_t n = (uint32_t)(bytes[size - 1] >> 6) + 1u;
    if (size < n || size > UINT32_MAX)
    {
        return false;
    }
    uint32_t state = 0;
    uint32_t mask = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        state |= (uint32_t)bytes[size - n + k] << (8u * k);
        mask = k == 0 ? 0x3Fu : (mask << 8) | 0xFFu;
    }
    state &= mask;
    rans->bytes = bytes;
    rans->offset = (uint32_t)size - n;
    rans->state = state + rans->base;
    return rans->state < rans->base * IO_BASE;
}

uint32_t maniDracoRansRead(maniDracoRans* rans)
{
    while (rans->state < rans->base && rans->offset > 0)
    {
        rans->state = rans->state * IO_BASE + rans->bytes[--rans->offset];
    }
    uint32_t quotient = rans->state >> rans->bits;
    uint32_t remainder = rans->state & ((1u << rans->bits) - 1u);
    // The last symbol whose range starts at or below the remainder.
    uint32_t lo = 0;
    uint32_t hi = rans->count;
    while (hi - lo > 1u)
    {
        uint32_t mid = lo + (hi - lo) / 2u;
        lo = rans->cumulative[mid] <= remainder ? mid : lo;
        hi = rans->cumulative[mid] <= remainder ? hi : mid;
    }
    rans->state = quotient * rans->probabilities[lo] + remainder - rans->cumulative[lo];
    return rans->values[lo];
}

// The tagged coding: a tag (a bit length) per components values, then
// each value in that many bits.
static maniDracoStatus Tagged(maniDracoReader* r, maniArena* arena, uint32_t count,
                              uint32_t components, uint32_t* out)
{
    maniDracoRans tags;
    maniDracoStatus s = maniDracoRansTable(r, arena, TAG_BITS, &tags);
    if (s != maniDracoDone)
    {
        return s;
    }
    if (!maniDracoRansStart(r, &tags) || tags.count == 0 || components == 0)
    {
        return maniDracoInvalid;
    }
    uint64_t bit = 0;
    uint64_t available = (uint64_t)maniDracoLeft(r) * 8u;
    for (uint32_t i = 0; i < count; i += components)
    {
        uint32_t length = maniDracoRansRead(&tags);
        if (length > 32u)
        {
            return maniDracoInvalid;
        }
        for (uint32_t j = 0; j < components && i + j < count; ++j)
        {
            if (available - bit < length)
            {
                return maniDracoInvalid;
            }
            uint32_t v = 0;
            for (uint32_t b = 0; b < length; ++b, ++bit)
            {
                v |= (uint32_t)((r->at[bit >> 3] >> (bit & 7u)) & 1u) << b;
            }
            out[i + j] = v;
        }
    }
    r->at += (bit + 7u) / 8u;
    return maniDracoDone;
}

static maniDracoStatus Raw(maniDracoReader* r, maniArena* arena, uint32_t count, uint32_t* out)
{
    uint8_t length = 0;
    if (!maniDracoU8(r, &length) || length < 1 || length > 18)
    {
        return maniDracoInvalid;
    }
    uint32_t bits = 3u * length / 2u;
    bits = bits < 12u ? 12u : (bits > 20u ? 20u : bits);
    maniDracoRans rans;
    maniDracoStatus s = maniDracoRansTable(r, arena, bits, &rans);
    if (s != maniDracoDone)
    {
        return s;
    }
    if (rans.count == 0 || !maniDracoRansStart(r, &rans))
    {
        return maniDracoInvalid;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        out[i] = maniDracoRansRead(&rans);
    }
    return maniDracoDone;
}

maniDracoStatus maniDracoSymbols(maniDracoReader* r, maniArena* arena, uint32_t count,
                                 uint32_t components, uint32_t* out)
{
    if (count == 0)
    {
        return maniDracoDone;
    }
    uint8_t scheme = 0;
    if (!maniDracoU8(r, &scheme) || scheme > 1)
    {
        return maniDracoInvalid;
    }
    return scheme == 0 ? Tagged(r, arena, count, components, out) : Raw(r, arena, count, out);
}

bool maniDracoBitsRead(maniDracoBits* b, uint32_t n, uint32_t* out)
{
    if (n > 32u || b->size * 8u - b->bit < n)
    {
        return false;
    }
    uint32_t v = 0;
    for (uint32_t k = 0; k < n; ++k, ++b->bit)
    {
        v |= (uint32_t)((b->bytes[b->bit >> 3] >> (b->bit & 7u)) & 1u) << k;
    }
    *out = v;
    return true;
}

// The rABS state's base and its bytes' radix.
#define RABS_BASE      4096u
#define RABS_PRECISION 256u

bool maniDracoRabsStart(maniDracoReader* r, maniDracoRabs* out)
{
    uint32_t size = 0;
    const uint8_t* bytes = NULL;
    *out = (maniDracoRabs){0};
    if (!maniDracoU8(r, &out->zero) || !maniDracoLeb32(r, &size) || size < 1 ||
        !maniDracoBytes(r, size, &bytes))
    {
        return false;
    }
    // The state in the last 1 to 3 bytes, less the last byte's top bits.
    uint32_t n = (uint32_t)(bytes[size - 1] >> 6) + 1u;
    if (n > 3u || size < n)
    {
        return false;
    }
    uint32_t state = 0;
    uint32_t mask = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        state |= (uint32_t)bytes[size - n + k] << (8u * k);
        mask = k == 0 ? 0x3Fu : (mask << 8) | 0xFFu;
    }
    out->bytes = bytes;
    out->offset = size - n;
    out->state = (state & mask) + RABS_BASE;
    return out->state < RABS_BASE * IO_BASE;
}

bool maniDracoRabsRead(maniDracoRabs* rabs)
{
    // The probability of a one in 256ths, wrapping as Google's byte does.
    uint32_t p = (uint8_t)(RABS_PRECISION - rabs->zero);
    if (rabs->state < RABS_BASE && rabs->offset > 0)
    {
        rabs->state = rabs->state * IO_BASE + rabs->bytes[--rabs->offset];
    }
    uint32_t x = rabs->state;
    uint32_t quotient = x / RABS_PRECISION;
    uint32_t remainder = x % RABS_PRECISION;
    uint32_t xn = quotient * p;
    bool one = remainder < p;
    rabs->state = one ? xn + remainder : x - xn - p;
    return one;
}
