// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The binary FBX container read into the document's tree.

#ifndef MAUL_ANIM_SRC_FBX_BINARY_H
#define MAUL_ANIM_SRC_FBX_BINARY_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The bytes before the version: the magic, 0x1A, the endianness byte.
#define MANI_FBX_MAGIC_SIZE  21u
#define MANI_FBX_HEADER_SIZE 27u

// Whether the bytes start with the binary form's magic.
bool maniFbxIsBinary(const uint8_t* bytes, size_t size);

// Reads the header and every record into fbx's nodes and properties.
maniImportResult maniFbxReadBinary(maniFbx* fbx);

// A word of the file in its byte order.
uint32_t maniFbxLoad32(const maniFbx* fbx, uint64_t at);
uint64_t maniFbxLoad64(const maniFbx* fbx, uint64_t at);

#endif // MAUL_ANIM_SRC_FBX_BINARY_H
