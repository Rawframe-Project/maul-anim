// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX property values read on request (mani-0012): numbers, integers,
// strings, and arrays decoded (inflated if compressed) into caller
// memory. Properties are indices into the document's property array.

#ifndef MAUL_ANIM_SRC_FBX_VALUE_H
#define MAUL_ANIM_SRC_FBX_VALUE_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// Whether a string property (binary or ASCII, quoted or a bare word)
// holds exactly these bytes, FBX's entities decoded and made valid UTF-8
// as maniFbxStringDecode makes them.
bool maniFbxStringEquals(const maniFbx* fbx, uint32_t property, const char* bytes, size_t length);

// A string property's bytes when they are its decoded ones (no entity
// and every byte ASCII but NUL): what maniFbxStringEquals compares;
// false for any other string, or no string.
bool maniFbxPlainString(const maniFbx* fbx, uint32_t property, const uint8_t** bytes,
                        size_t* length);

// A name's hash: maniHash64 from MANI_HASH_INIT, a byte at a time.
uint64_t maniFbxNameHash(const void* bytes, size_t length);

// The same hash of a string property's bytes as maniFbxStringEquals
// compares them; MANI_HASH_INIT for no string.
uint64_t maniFbxStringHash(const maniFbx* fbx, uint32_t property);

// Whether two string properties hold the same bytes, so compared.
bool maniFbxSameString(const maniFbx* fbx, uint32_t a, uint32_t b);

// A string property's bytes, entities decoded, then made valid UTF-8 as
// ufbx makes them (mani-0012): each valid sequence kept, each other byte
// and NUL U+FFFD, so a byte can become three. Into out, whole characters
// up to capacity: the count written; with out NULL, the count all take.
size_t maniFbxStringDecode(const maniFbx* fbx, uint32_t property, uint8_t* out, size_t capacity);

// An object name's part before "\x00\x01" (binary) or after "::"
// (ASCII), found before the bytes are made valid, then decoded as
// maniFbxStringDecode decodes.
size_t maniFbxStringNameRange(const maniFbx* fbx, uint32_t property, bool binary, uint8_t* out,
                              size_t capacity);

// A node's k-th property, or MANI_FBX_NONE past its count.
uint32_t maniFbxNodeProperty(const maniFbx* fbx, uint32_t node, uint32_t k);

// A number (any scalar type, finite) as binary64.
bool maniFbxNumber(const maniFbx* fbx, uint32_t property, double* out);

// A whole number in [minimum, maximum]: an integer type, or a float
// type holding a whole value.
bool maniFbxInteger(const maniFbx* fbx, uint32_t property, int64_t minimum, int64_t maximum,
                    int64_t* out);

// A character code as ufbx reads one: a whole number, or an ASCII bare
// word's first byte.
bool maniFbxCharacter(const maniFbx* fbx, uint32_t property, int64_t* out);

// A string's bytes (raw: FBX's names hold "\x00\x01" separators).
bool maniFbxString(const maniFbx* fbx, uint32_t property, const uint8_t** bytesOut,
                   uint32_t* lengthOut);

// An array's element count, or 0 for a property that is no array.
uint32_t maniFbxArrayCount(const maniFbx* fbx, uint32_t property);

// An array's elements as binary64 (float arrays finite) or as integers
// (integer arrays), its count of them into out. Property for the wrong
// kind; Data for a compressed stream that does not inflate to the array
// or a float not finite; Capacity when the allocator fails.
maniImportResult maniFbxReadDoubles(const maniFbx* fbx, uint32_t property, double* out);
maniImportResult maniFbxReadIntegers(const maniFbx* fbx, uint32_t property, int64_t* out);

// A node's values as binary64, as ufbx reads 6.x's Key lists: its one
// array, or its scalars with bare words (ASCII) as their first byte;
// counted with out NULL. Data for a value not finite (a word spelling
// inf or nan reads as such), Property for a string among the values, and
// as maniFbxReadDoubles for an array.
maniImportResult maniFbxReadValues(const maniFbx* fbx, uint32_t node, double* out,
                                   uint32_t* countOut);

// A float or int32 array's binary32 bit patterns, as they are (an ASCII
// array's: from 7.2 integers holding the bits, before float values),
// for KeyAttrDataFloat, whose words are not all floats; Property for
// other kinds.
maniImportResult maniFbxReadFloatBits(const maniFbx* fbx, uint32_t property, uint32_t* out);

#endif // MAUL_ANIM_SRC_FBX_VALUE_H
