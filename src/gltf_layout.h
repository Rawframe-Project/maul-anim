// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's buffer views and accessors: checked at
// open, and accessors read into binary32 for the skeleton, skin and clip
// importers.

#ifndef MAUL_ANIM_SRC_GLTF_LAYOUT_H
#define MAUL_ANIM_SRC_GLTF_LAYOUT_H

#include "gltf_document.h"

#include "maul-anim/import.h"

// Checks the buffer views and accessors and fills their arrays.
maniImportResult maniGltfReadLayout(maniGltf* gltf);

// Reads an accessor into binary32, components per element as `rows` and
// `columns` give them (column-major for matrices), normalized integers
// converted by the specification's table, sparse substitution applied;
// out holds count * rows * columns floats. Refuses an external buffer
// not yet supplied (Pending) and NaN or infinite floats (Data).
maniImportResult maniGltfReadFloats(const maniGltf* gltf, uint32_t accessor, float* out);

// An accessor's elements from decoded values, packed one after another in
// its component type: element i is value map[i] (value i without a map).
// Data at `at` for a value past valueCount or one not finite.
maniImportResult maniGltfReadMapped(const maniGltf* gltf, uint32_t accessor, const uint8_t* values,
                                    const uint32_t* map, uint32_t valueCount, uint64_t at,
                                    float* out);

#endif // MAUL_ANIM_SRC_GLTF_LAYOUT_H
