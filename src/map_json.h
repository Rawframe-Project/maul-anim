// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The maps' JSON forms, the parts both maps use: the header,
// a reference pose listing the bones whose local is not the rest, the
// text opened for reading and the text kept in the builder. Offline.

#ifndef MAUL_ANIM_SRC_MAP_JSON_H
#define MAUL_ANIM_SRC_MAP_JSON_H

#include "graph_json_read.h"
#include "graph_json_write.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The text read as JSON and r started over it, its arena empty; false
// with the refusal (Json, or Capacity) in *result.
bool maniMapJsonOpen(maniBuilder* builder, const char* text, size_t length, maniJson* json,
                     maniGraphReader* r, maniBuildResult* result);

// The reader's arena and the JSON values released.
void maniMapJsonClose(maniBuilder* builder, maniJson* json, maniGraphReader* r);

// The format string (format) and version (1) at f[0] and f[1].
bool maniMapJsonHeader(maniGraphReader* r, const uint32_t* f, const char* format);

// A reference pose of r's skeleton: a list of {bone, translation,
// rotation, scale} objects, each bone at most once, a bone or member left
// out its rest; NULL for an empty or absent list.
bool maniMapJsonReference(maniGraphReader* r, uint32_t value, const maniTransform** out);

// A list's count, 0 when absent; refuses another kind.
bool maniMapJsonCount(maniGraphReader* r, uint32_t value, uint32_t* countOut);

// A word among count words into *out, one of a kind's values; *out as it
// is when absent.
bool maniMapJsonWord(maniGraphReader* r, uint32_t value, const char* const* words, uint32_t count,
                     int32_t* out);

// The header's lines, after the opening brace.
void maniMapJsonPutHeader(maniGraphWriter* w, const char* format);

// The line "key": [...] of w's skeleton's bones whose local in given is
// not the rest bit for bit (none when given is NULL), one a line.
void maniMapJsonPutReference(maniGraphWriter* w, const char* key, const maniTransform* given);

// ", "key": [count numbers]".
void maniMapJsonPutFloats(maniGraphWriter* w, const char* key, const float* values, uint32_t count);

// A list's opening: a new line, "key": [.
void maniMapJsonPutOpen(maniGraphWriter* w, const char* key);

// Before item i of a list: the comma after the one before, a new line.
void maniMapJsonPutItem(maniGraphWriter* w, uint32_t i);

// A list's closing, on a line of its own when it has items, and the
// comma after it unless last.
void maniMapJsonPutClose(maniGraphWriter* w, uint32_t count, bool last);

// Draws a map's text from its source.
typedef void (*maniMapJsonDraw)(maniGraphWriter* w, const void* source);

// After a build that succeeded (built), the text draw makes of source
// over skeleton, counted then written, kept in the builder; built, or
// Capacity.
maniBuildResult maniMapJsonKeep(maniBuilder* builder, maniBuildResult built,
                                const maniSkeleton* skeleton, maniMapJsonDraw draw,
                                const void* source);

#endif // MAUL_ANIM_SRC_MAP_JSON_H
