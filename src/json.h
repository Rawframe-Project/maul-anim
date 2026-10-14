// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// JSON as RFC 8259 defines it, read as hostile input (mani-0007): checked in
// full, then laid out as a flat array of values in document order, each
// a span of the text the caller keeps alive. Strings and numbers are
// decoded when read. Offline.

#ifndef MAUL_ANIM_SRC_JSON_H
#define MAUL_ANIM_SRC_JSON_H

#include "decimal.h"

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The deepest nesting of arrays and objects read.
#define MANI_JSON_MAX_DEPTH 128u
// What a lookup finds for nothing.
#define MANI_JSON_NONE 0xFFFFFFFFu

typedef enum maniJsonKind
{
    maniJsonNull = 0,
    maniJsonFalse = 1,
    maniJsonTrue = 2,
    maniJsonNumber = 3,
    maniJsonString = 4,
    maniJsonArray = 5,
    maniJsonObject = 6,
} maniJsonKind;

// A value: its kind, its bytes (a string's between its quotes, a
// container's from its bracket to the end of the text it spans), the
// elements or members of a container, and the index just past it and
// all it holds. A container's first child, if any, is the next index;
// an object's children are key string, value, key string, value.
typedef struct maniJsonValue
{
    uint32_t kind;
    uint32_t start;
    uint32_t length;
    uint32_t count;
    uint32_t next;
} maniJsonValue;

typedef struct maniJson
{
    const uint8_t* text;
    maniJsonValue* values;
    uint32_t count;
} maniJson;

typedef enum maniJsonReason
{
    maniJsonReasonNone = 0,
    // Not RFC 8259's grammar, or text past the value.
    maniJsonReasonSyntax = 1,
    // Nested deeper than MANI_JSON_MAX_DEPTH.
    maniJsonReasonDepth = 2,
    // More than 4 GiB - 1 bytes.
    maniJsonReasonSize = 3,
    // Bytes that are not UTF-8.
    maniJsonReasonUtf8 = 4,
    // An escape RFC 8259 does not define, or a lone surrogate.
    maniJsonReasonEscape = 5,
    // A control character in a string.
    maniJsonReasonControl = 6,
} maniJsonReason;

typedef struct maniJsonResult
{
    maniResult result;
    maniJsonReason reason;
    // The first byte found wrong.
    uint64_t offset;
} maniJsonResult;

// Reads a JSON text; a leading UTF-8 byte order mark is skipped. On
// success the values are the allocator's until maniReleaseJson.
maniJsonResult maniReadJson(const maniAllocator* allocator, const uint8_t* text, size_t size,
                            maniJson* jsonOut);

void maniReleaseJson(const maniAllocator* allocator, maniJson* json);

// An object's member of a name, the last when the name repeats (glTF
// asks that later values win), or MANI_JSON_NONE; MANI_JSON_NONE also
// for a value that is not an object.
uint32_t maniJsonFind(const maniJson* json, uint32_t object, const char* name, size_t length);

// Whether a string value decodes to these bytes.
bool maniJsonStringEquals(const maniJson* json, uint32_t string, const char* bytes, size_t length);

// Decodes a string value into out, at most capacity bytes; returns the
// decoded length, which is at most the value's length.
size_t maniJsonStringDecode(const maniJson* json, uint32_t string, uint8_t* out, size_t capacity);

// A number value in a binary format; false for another kind or a value
// past the format's range.
bool maniJsonBinary(const maniJson* json, uint32_t number, maniBinaryFormat format,
                    uint64_t* bitsOut);

// A number value as a whole number in [minimum, maximum]; false for
// another kind, a fraction or a value out of range.
bool maniJsonInteger(const maniJson* json, uint32_t number, int64_t minimum, int64_t maximum,
                     int64_t* valueOut);

#endif // MAUL_ANIM_SRC_JSON_H
