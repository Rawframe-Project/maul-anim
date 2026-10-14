// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's buffers (mani-0008).

#ifndef MAUL_ANIM_SRC_GLTF_BUFFER_H
#define MAUL_ANIM_SRC_GLTF_BUFFER_H

#include "gltf_document.h"

#include "maul-anim/import.h"

// Reads `buffers`: GLB, `data:` and external, after the JSON and any BIN
// chunk are in the document.
maniImportResult maniGltfReadBuffers(maniGltf* gltf);

// Frees what the buffers own, and their array.
void maniGltfReleaseBuffers(maniGltf* gltf);

#endif // MAUL_ANIM_SRC_GLTF_BUFFER_H
