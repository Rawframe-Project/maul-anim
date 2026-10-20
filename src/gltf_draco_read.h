// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's stream reading (mani-0021): little-endian values and LEB128
// within a stream's bounds, and its entropy coding, rANS with the tagged
// and raw symbol codings, as Google's decoder reads bitstream 2.2.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_READ_H
#define MAUL_ANIM_SRC_GLTF_DRACO_READ_H

#include "arena.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// What a decoding step gives: done, a stream Draco calls invalid, or an
// allocation the arena refused.
typedef enum maniDracoStatus
{
    maniDracoDone = 0,
    maniDracoInvalid = 1,
    maniDracoCapacity = 2,
} maniDracoStatus;

// Bounds a stream is read within.
typedef struct maniDracoReader
{
    const uint8_t* at;
    const uint8_t* end;
} maniDracoReader;

bool maniDracoU8(maniDracoReader* r, uint8_t* out);
bool maniDracoU16(maniDracoReader* r, uint16_t* out);
bool maniDracoU32(maniDracoReader* r, uint32_t* out);
// A LEB128 value of at most 5 bytes (32 bits) or 10 (64).
bool maniDracoLeb32(maniDracoReader* r, uint32_t* out);
bool maniDracoLeb64(maniDracoReader* r, uint64_t* out);
// size bytes, or false when fewer remain.
bool maniDracoBytes(maniDracoReader* r, size_t size, const uint8_t** out);

static inline size_t maniDracoLeft(const maniDracoReader* r)
{
    return (size_t)(r->end - r->at);
}

// count symbols, components to a tag for the tagged coding, into out:
// a scheme byte, then the tagged or raw coding; nothing is read for a
// count of 0. Tables are taken from the arena.
maniDracoStatus maniDracoSymbols(maniDracoReader* r, maniArena* arena, uint32_t count,
                                 uint32_t components, uint32_t* out);

// rANS over a span of bytes read from its end, at a precision of
// 2^bits (12 to 20), its symbols' cumulative probabilities from a table.
typedef struct maniDracoRans
{
    const uint8_t* bytes;
    uint32_t offset;
    uint32_t state;
    uint32_t base;
    uint32_t bits;
    // The symbols of non-zero probability: their values, probabilities
    // and cumulative probabilities, count of them.
    const uint32_t* values;
    const uint32_t* probabilities;
    const uint32_t* cumulative;
    uint32_t count;
} maniDracoRans;

// Reads a symbol table (count symbols) for a precision of 2^bits.
maniDracoStatus maniDracoRansTable(maniDracoReader* r, maniArena* arena, uint32_t bits,
                                   maniDracoRans* out);
// Starts reading size bytes, which the reader then skips.
bool maniDracoRansStart(maniDracoReader* r, maniDracoRans* rans);
uint32_t maniDracoRansRead(maniDracoRans* rans);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_READ_H
