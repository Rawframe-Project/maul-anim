// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An opened FBX document (mani-0012): one tree of named nodes and their
// typed properties, the same from the binary and the ASCII form, every
// length checked at open and every value read on request.

#ifndef MAUL_ANIM_SRC_FBX_DOCUMENT_H
#define MAUL_ANIM_SRC_FBX_DOCUMENT_H

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// No node: the end of a child list.
#define MANI_FBX_NONE UINT32_MAX

// The deepest nesting read.
#define MANI_FBX_MAX_DEPTH 64u

// A property: its FBX type letter (binary `C Y I L F D S R` and the
// arrays `b c i l f d`; from the ASCII form `N` a number's text, `Q`
// text shaped like one that is none ("1.#INF"), `S` a
// string's, `W` a bare word, `A` an array's values), where its payload
// starts in the file, and its count: a string's bytes, an array's
// elements, 1 for a number. An ASCII property's text is encodedSize
// bytes long.
typedef struct maniFbxProperty
{
    uint64_t offset;
    // An array's bytes in the file (compressed or not).
    uint64_t encodedSize;
    uint32_t count;
    uint8_t type;
    // An array's encoding: 0 raw, 1 zlib.
    uint8_t encoding;
    uint8_t reserved[2];
} maniFbxProperty;

// A node: its name in the file, its properties (a run of the property
// array), its first child and its next sibling.
typedef struct maniFbxNode
{
    uint64_t nameOffset;
    uint32_t nameLength;
    uint32_t firstProperty;
    uint32_t propertyCount;
    uint32_t firstChild;
    uint32_t next;
    uint32_t reserved;
} maniFbxNode;

struct maniFbx
{
    maniAllocator allocator;
    const uint8_t* bytes;
    size_t size;
    uint32_t version;
    bool bigEndian;
    // From the ASCII form: numbers are text, strings may hold entities.
    bool ascii;
    // Node 0 has no name and the top-level nodes as its children.
    maniFbxNode* nodes;
    uint32_t nodeCount;
    maniFbxProperty* properties;
    uint32_t propertyCount;
};

static inline maniImportResult maniFbxSuccess(void)
{
    maniImportResult r = {mani_success, maniImportReasonNone, 0, 0, 0};
    return r;
}

static inline maniImportResult maniFbxFail(maniImportReason reason, uint64_t offset, uint32_t item)
{
    maniImportResult r = {mani_errorFormat, reason, offset, item, 0};
    return r;
}

// The first child of a node with the name given, or MANI_FBX_NONE.
uint32_t maniFbxFindChild(const maniFbx* fbx, uint32_t node, const char* name, size_t length);

// Whether a node has the name given.
bool maniFbxNameIs(const maniFbx* fbx, uint32_t node, const char* name, size_t length);

#endif // MAUL_ANIM_SRC_FBX_DOCUMENT_H
