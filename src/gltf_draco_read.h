// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's stream reading: little-endian values and LEB128
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

// Bits read least significant first from a span of bytes; reading past
// the span is invalid.
typedef struct maniDracoBits
{
    const uint8_t* bytes;
    uint64_t bit;
    uint64_t size;
} maniDracoBits;

// n bits (at most 32) into out; false past the end.
bool maniDracoBitsRead(maniDracoBits* b, uint32_t n, uint32_t* out);

// rABS: binary decisions of one probability (Google's RAnsBitDecoder): a
// probability byte, a LEB128 size and that many bytes read from their end.
typedef struct maniDracoRabs
{
    const uint8_t* bytes;
    uint32_t offset;
    uint32_t state;
    uint8_t zero;
} maniDracoRabs;

bool maniDracoRabsStart(maniDracoReader* r, maniDracoRabs* out);
bool maniDracoRabsRead(maniDracoRabs* rabs);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_READ_H
