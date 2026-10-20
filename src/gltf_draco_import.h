// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF primitive compressed by KHR_draco_mesh_compression (mani-0021):
// its stream decoded once, and the accessors of the attributes the
// extension names read from it, each one matching the decoded attribute
// as the extension asks.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_IMPORT_H
#define MAUL_ANIM_SRC_GLTF_DRACO_IMPORT_H

#include "arena.h"
#include "gltf_document.h"
#include "gltf_draco.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct maniGltfDraco
{
    // Whether the primitive has the extension; its attributes object and
    // view; the decoded mesh.
    bool present;
    uint32_t attributes;
    uint32_t view;
    maniDracoMesh mesh;
} maniGltfDraco;

// Decodes the primitive's stream, of at most vertices points, into the
// arena, and checks each attribute the extension names against its
// accessor; present is false for a primitive without the extension.
maniImportResult maniGltfDracoDecode(const maniGltf* gltf, maniArena* arena, uint32_t primitive,
                                     uint32_t attributes, uint32_t vertices, maniGltfDraco* out);

// The accessor of the primitive's attribute name as floats: from the
// stream when the extension names it, else from its view.
maniImportResult maniGltfDracoFloats(const maniGltf* gltf, const maniGltfDraco* draco,
                                     const char* name, size_t length, uint32_t accessor,
                                     float* out);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_IMPORT_H
