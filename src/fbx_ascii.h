// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ASCII FBX form (mani-0012) read into the document's tree.

#ifndef MAUL_ANIM_SRC_FBX_ASCII_H
#define MAUL_ANIM_SRC_FBX_ASCII_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// Reads every node of a text file into fbx's nodes and properties, and
// its version from the first comment or FBXHeaderExtension.
maniImportResult maniFbxReadAscii(maniFbx* fbx);

// An ASCII number's text, [at, at + length), converted exactly (mani-0007):
// binary64 when finite, or a whole number in [minimum, maximum].
bool maniFbxTextDouble(const maniFbx* fbx, uint64_t at, uint64_t length, double* out);
bool maniFbxTextInteger(const maniFbx* fbx, uint64_t at, uint64_t length, int64_t minimum,
                        int64_t maximum, int64_t* out);

// The next number token of an ASCII array's text in [*at, end): its
// start and length; *at moves past it. False when none is left.
bool maniFbxAsciiNextNumber(const maniFbx* fbx, uint64_t* at, uint64_t end, uint64_t* startOut,
                            uint64_t* lengthOut);

#endif // MAUL_ANIM_SRC_FBX_ASCII_H
