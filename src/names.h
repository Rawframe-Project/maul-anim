// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bone names for imported documents: a source's
// own name, or with `#` and its index appended when it has none or an
// earlier source in the document has the same; fixed per document.

#ifndef MAUL_ANIM_SRC_NAMES_H
#define MAUL_ANIM_SRC_NAMES_H

#include "allocator.h"

#include "maul-anim/import.h"

#include <stddef.h>
#include <stdint.h>

// Every source's bone name, back to back.
typedef struct maniBoneNames
{
    char* text;
    uint32_t* starts;
    size_t textSize;
    uint32_t count;
    // The sources named other than as given, in source order.
    uint32_t* renamed;
    uint32_t renamedCount;
} maniBoneNames;

// Names `count` sources from their decoded names, held back to back in
// `original` (spans[b] to spans[b + 1] the b-th's, spans[count] the
// total); indices[b] is the number a renamed source's suffix shows.
maniImportResult maniNameBones(const maniAllocator* allocator, const char* original,
                               const uint32_t* spans, const uint32_t* indices, uint32_t count,
                               maniBoneNames* namesOut);

void maniReleaseBoneNames(const maniAllocator* allocator, maniBoneNames* names);

// The b-th source's bone name, not terminated.
const char* maniBoneName(const maniBoneNames* names, uint32_t b, size_t* lengthOut);

#endif // MAUL_ANIM_SRC_NAMES_H
