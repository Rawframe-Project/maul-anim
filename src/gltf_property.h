// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's JSON properties read with their kinds and ranges
// checked (mani-0008): each refusal a maniImportResult with the offset
// of the property's value in the bytes the document was opened from.

#ifndef MAUL_ANIM_SRC_GLTF_PROPERTY_H
#define MAUL_ANIM_SRC_GLTF_PROPERTY_H

#include "json.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// Where the JSON text starts in the bytes, for offsets.
typedef struct maniGltfJson
{
    const maniJson* json;
    uint64_t base;
} maniGltfJson;

maniImportResult maniImportFail(maniImportReason reason, uint64_t offset, uint32_t item);
maniImportResult maniImportSuccess(void);

// The offset of a value in the bytes.
uint64_t maniGltfOffset(const maniGltfJson* g, uint32_t value);

// An object's member, or MANI_JSON_NONE; a refusal (Property) for a value
// of another kind than `kind`, or for a missing member when required.
maniImportResult maniGltfMember(const maniGltfJson* g, uint32_t object, const char* name,
                                maniJsonKind kind, bool required, uint32_t item,
                                uint32_t* valueOut);

// A whole-number member in [minimum, maximum]; *valueOut keeps its
// default when the member is absent and not required.
maniImportResult maniGltfInteger(const maniGltfJson* g, uint32_t object, const char* name,
                                 bool required, int64_t minimum, int64_t maximum, uint32_t item,
                                 int64_t* valueOut);

// An index member below count (Property for a non-index, Index past the
// count); *indexOut is MANI_JSON_NONE when absent and not required.
maniImportResult maniGltfIndex(const maniGltfJson* g, uint32_t object, const char* name,
                               bool required, uint32_t count, uint32_t item, uint32_t* indexOut);

// A boolean member; *valueOut keeps its default when absent.
maniImportResult maniGltfBool(const maniGltfJson* g, uint32_t object, const char* name,
                              uint32_t item, bool* valueOut);

// A top-level array (`nodes`, `accessors`): its value and element count,
// MANI_JSON_NONE and 0 when absent; an element of another kind than
// `kind` is refused.
maniImportResult maniGltfArray(const maniGltfJson* g, const char* name, maniJsonKind kind,
                               uint32_t* arrayOut, uint32_t* countOut);

#endif // MAUL_ANIM_SRC_GLTF_PROPERTY_H
