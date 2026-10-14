// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An opened FBX document (mani-0012): one tree of named nodes and their
// typed properties, the same from the binary and the ASCII form, every
// length checked at open and every value read on request.

#ifndef MAUL_ANIM_SRC_FBX_DOCUMENT_H
#define MAUL_ANIM_SRC_FBX_DOCUMENT_H

#include "names.h"

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

// An object (mani-0013): its id (7.x; 6.x: its name's hash), its node,
// and its own and its template's properties nodes (or MANI_FBX_NONE).
typedef struct maniFbxObject
{
    int64_t id;
    uint32_t node;
    uint32_t properties;
    uint32_t templateProperties;
    uint32_t reserved;
} maniFbxObject;

// A connection: child and parent objects (the parent the root when it
// is fbx->objectCount), and the property names it gives (string
// properties, or MANI_FBX_NONE).
typedef struct maniFbxConnection
{
    uint32_t child;
    uint32_t parent;
    uint32_t childProperty;
    uint32_t parentProperty;
} maniFbxConnection;

// GlobalSettings: right (coord), up and front axes (0 x, 1 y, 2 z) with
// their signs; centimetres a unit; the frame rate as numerator over
// denominator, or customRate when the denominator is 0; the time span
// in ticks.
typedef struct maniFbxSettings
{
    uint8_t axes[3];
    int8_t signs[3];
    double unitScale;
    double originalUnitScale;
    uint32_t timeMode;
    uint32_t rateNumerator;
    uint32_t rateDenominator;
    double customRate;
    int64_t spanStart;
    int64_t spanStop;
} maniFbxSettings;

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
    maniFbxObject* objects;
    uint32_t objectCount;
    uint32_t objectCapacity;
    // Objects with no id or name, skipped.
    uint32_t skippedObjects;
    // Object indices sorted by id.
    uint32_t* objectsById;
    maniFbxConnection* connections;
    uint32_t connectionCount;
    uint32_t connectionCapacity;
    // Connection indices sorted by parent and by child, and where each
    // object's run starts (the root's last).
    uint32_t* byParent;
    uint32_t* byChild;
    uint32_t* parentStarts;
    uint32_t* childStarts;
    // Entries of another type or naming no object.
    uint32_t skippedConnections;
    maniFbxSettings settings;
    // Models (mani-0014): their objects in object order, each object's
    // model (or MANI_FBX_NONE), each model's parent (modelCount for the
    // root), children per parent (the root's last) and bone names.
    uint32_t* models;
    uint32_t modelCount;
    uint32_t* modelOfObject;
    uint32_t* modelParents;
    uint32_t* modelChildren;
    uint32_t* childStartsOfModel;
    maniBoneNames modelNames;
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
