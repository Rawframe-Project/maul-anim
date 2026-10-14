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
// holds exactly these bytes, FBX's entities decoded.
bool maniFbxStringEquals(const maniFbx* fbx, uint32_t property, const char* bytes, size_t length);

// A string property's bytes, entities decoded, into out: the count
// written, whole bytes up to capacity.
size_t maniFbxStringDecode(const maniFbx* fbx, uint32_t property, uint8_t* out, size_t capacity);

// A node's k-th property, or MANI_FBX_NONE past its count.
uint32_t maniFbxNodeProperty(const maniFbx* fbx, uint32_t node, uint32_t k);

// A number (any scalar type, finite) as binary64.
bool maniFbxNumber(const maniFbx* fbx, uint32_t property, double* out);

// A whole number in [minimum, maximum]: an integer type, or a float
// type holding a whole value.
bool maniFbxInteger(const maniFbx* fbx, uint32_t property, int64_t minimum, int64_t maximum,
                    int64_t* out);

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

#endif // MAUL_ANIM_SRC_FBX_VALUE_H
