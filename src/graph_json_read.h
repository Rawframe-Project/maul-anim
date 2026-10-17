// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Reading the graph JSON form (mani-0023), the parts every list uses:
// objects with their members checked, names decoded and looked up in
// tables sorted by hash, numbers and scalars; the first fault kept with
// its byte offset. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_READ_H
#define MAUL_ANIM_SRC_GRAPH_JSON_READ_H

#include "arena.h"
#include "json.h"

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Names of a list, findable by their bytes: the items sorted by the
// hash of their names.
typedef struct maniNameTable
{
    const uint64_t* keys;
    const uint32_t* items;
    const char* const* names;
    const uint32_t* lengths;
    uint32_t count;
} maniNameTable;

typedef struct maniGraphReader
{
    const maniJson* json;
    maniArena arena;
    const maniSkeleton* skeleton;
    const maniGraphClipSource* hostClips;
    maniNameTable host;
    const maniGraphDatabaseSource* hostDatabases;
    maniNameTable hostDatabaseNames;
    const maniGraphModelSource* hostModels;
    maniNameTable hostModelNames;
    // The first fault: Capacity, or a refusal with its offset.
    maniBuildResult fault;
} maniGraphReader;

// The names one graph's members refer to, and its parameters read.
typedef struct maniGraphScope
{
    maniNameTable parameters;
    maniNameTable clips;
    maniNameTable syncGroups;
    maniNameTable databases;
    maniNameTable models;
    const maniGraphParameterSource* sources;
} maniGraphScope;

// The names of count host slot sources (clip, database or model sources,
// each its asset, then its name and length, stride bytes apart) in a
// table; false when the arena fails.
bool maniGraphJsonHostNames(maniGraphReader* r, const void* items, size_t stride, uint32_t count,
                            maniNameTable* out);

// Records a fault at a value's first byte; returns false.
bool maniGraphJsonFault(maniGraphReader* r, uint32_t value, maniGraphForm form);

// Records the arena failing; returns false.
bool maniGraphJsonNoMemory(maniGraphReader* r);

// An object's members among keys (at most 32): each key's value in
// found, MANI_JSON_NONE when absent; refuses a value that is no object,
// a key not among keys, a repeat, or one of the required (a bit per key)
// missing.
bool maniGraphJsonMembers(maniGraphReader* r, uint32_t object, const char* const* keys,
                          uint32_t count, uint32_t required, uint32_t* found);

// An array's element count; refuses another kind.
bool maniGraphJsonArray(maniGraphReader* r, uint32_t value, uint32_t* countOut);

// The element after one, or an array's first when element is the array.
static inline uint32_t maniGraphJsonNext(const maniJson* json, uint32_t element, bool first)
{
    return first ? element + 1 : json->values[element].next;
}

// A string decoded into the arena.
bool maniGraphJsonName(maniGraphReader* r, uint32_t value, const char** nameOut,
                       uint32_t* lengthOut);

// A table of count names (the arrays kept, not copied).
bool maniGraphJsonTable(maniGraphReader* r, const char* const* names, const uint32_t* lengths,
                        uint32_t count, maniNameTable* tableOut);

// The first item of a name, or MANI_JSON_NONE.
uint32_t maniGraphJsonLookup(const maniNameTable* table, const char* name, uint32_t length);

// A string naming an item of the table; refuses a name it lacks.
bool maniGraphJsonRefer(maniGraphReader* r, const maniNameTable* table, uint32_t value,
                        uint32_t* itemOut);

// The index of a string value among count words, or count.
uint32_t maniGraphJsonWord(const maniGraphReader* r, uint32_t value, const char* const* words,
                           uint32_t count);

bool maniGraphJsonFloat(maniGraphReader* r, uint32_t value, float* out);
bool maniGraphJsonBool(maniGraphReader* r, uint32_t value, bool* out);

// A whole number from 0 to UINT32_MAX - 1.
bool maniGraphJsonIndex(maniGraphReader* r, uint32_t value, uint32_t* out);

// A number, or {"parameter": name} for a parameter of the scope.
bool maniGraphJsonScalar(maniGraphReader* r, const maniGraphScope* scope, uint32_t value,
                         maniGraphScalar* out);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_READ_H
