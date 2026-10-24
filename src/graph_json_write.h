// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Writing the graph JSON form, the parts every list uses:
// text counted, then written, strings escaped, keys, numbers as the
// shortest binary32 text, parameters by name and scalars. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_WRITE_H
#define MAUL_ANIM_SRC_GRAPH_JSON_WRITE_H

#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The text's bytes so far; out is NULL while counting.
typedef struct maniGraphWriter
{
    char* out;
    size_t at;
    const maniSkeleton* skeleton;
} maniGraphWriter;

void maniGraphJsonPut(maniGraphWriter* w, const char* bytes, size_t length);
void maniGraphJsonPutText(maniGraphWriter* w, const char* text);

// A new line indented by depth steps of two spaces.
void maniGraphJsonLine(maniGraphWriter* w, uint32_t depth);

// A JSON string: quote, backslash and the controls escaped.
void maniGraphJsonPutString(maniGraphWriter* w, const char* bytes, uint32_t length);

// A key and the separator after it.
void maniGraphJsonPutKey(maniGraphWriter* w, const char* key);

void maniGraphJsonPutFloat(maniGraphWriter* w, float value);
void maniGraphJsonPutUnsigned(maniGraphWriter* w, uint64_t value);
void maniGraphJsonPutInt(maniGraphWriter* w, int32_t value);
void maniGraphJsonPutBool(maniGraphWriter* w, bool value);
void maniGraphJsonPutParameterName(maniGraphWriter* w, const maniGraphSource* source,
                                   uint32_t parameter);

// A number a node takes: a literal, or {"parameter": name}.
void maniGraphJsonPutScalar(maniGraphWriter* w, const maniGraphSource* source, maniGraphScalar s);

// count scalars as a list.
void maniGraphJsonPutScalars(maniGraphWriter* w, const maniGraphSource* source,
                             const maniGraphScalar* scalars, uint32_t count);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_WRITE_H
