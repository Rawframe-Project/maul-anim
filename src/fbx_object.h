// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX objects, their connections, templated properties and the scene's
// settings (mani-0013), read once at open from the document's tree, the
// same for 7.x ids and 6.x names.

#ifndef MAUL_ANIM_SRC_FBX_OBJECT_H
#define MAUL_ANIM_SRC_FBX_OBJECT_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Reads Objects, Connections, Definitions and GlobalSettings.
maniImportResult maniFbxReadObjects(maniFbx* fbx);

void maniFbxReleaseObjects(maniFbx* fbx);

// The object with an id (7.x) or MANI_FBX_NONE.
uint32_t maniFbxFindObject(const maniFbx* fbx, int64_t id);

// The connections an object is the parent of, or the child of: a run of
// indices into fbx->connections, in file order; their count.
uint32_t maniFbxChildConnections(const maniFbx* fbx, uint32_t object, const uint32_t** out);
uint32_t maniFbxParentConnections(const maniFbx* fbx, uint32_t object, const uint32_t** out);

// Whether an object is of a kind (its node's name: Model, Deformer).
bool maniFbxObjectKindIs(const maniFbx* fbx, uint32_t object, const char* kind, size_t length);

// Whether an object's subclass (LimbNode, Skin, Cluster) is the one given.
bool maniFbxObjectSubclassIs(const maniFbx* fbx, uint32_t object, const char* name, size_t length);

// An object's name without its class, entities decoded, into out; the
// count of bytes written, whole up to capacity.
size_t maniFbxObjectName(const maniFbx* fbx, uint32_t object, uint8_t* out, size_t capacity);

// The entry (a `P` or `Property` node) holding an object's property of
// that name, its own or its template's; MANI_FBX_NONE when neither has
// it. A name given twice is its later entry.
uint32_t maniFbxFindProperty(const maniFbx* fbx, uint32_t object, const char* name, size_t length);

// An entry's k-th value (after its name, type, label and flags), as an
// index into fbx->properties; MANI_FBX_NONE past its values or for no
// entry (a lookup's MANI_FBX_NONE passes through).
uint32_t maniFbxPropertyValue(const maniFbx* fbx, uint32_t entry, uint32_t k);

#endif // MAUL_ANIM_SRC_FBX_OBJECT_H
