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

// For readers of other layouts (mani-0017): a string property's 6.x key;
// the objects indexed by id, refused (Property) for one given twice; the
// connections indexed by both ends; GlobalSettings read with a frame
// rate to fall back on where it gives none (0 for none); and a Settings
// node's FrameRate (a number or a numeric string, 0 for none).
int64_t maniFbxNameKey(const maniFbx* fbx, uint32_t property, bool* ok);
maniImportResult maniFbxIndexObjects(maniFbx* fbx);
maniImportResult maniFbxIndexConnections(maniFbx* fbx);
maniImportResult maniFbxReadSettingsWith(maniFbx* fbx, double legacyRate);
double maniFbxLegacyFrameRate(const maniFbx* fbx, uint32_t settings);

void maniFbxReleaseObjects(maniFbx* fbx);

// The object with an id (7.x) or MANI_FBX_NONE.
uint32_t maniFbxFindObject(const maniFbx* fbx, int64_t id);

// The object a property names, by id (7.x) or by name (6.x): the root
// (objectCount) for id 0 or "Model::Scene", MANI_FBX_NONE for none.
uint32_t maniFbxReferencedObject(const maniFbx* fbx, uint32_t property);

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

// Before 6.0: the rest value of an object's Lcl Translation (0),
// Rotation (1) or Scaling (2) its channels give (mani-0017); false where
// none does.
bool maniFbxLegacyRest(const maniFbx* fbx, uint32_t object, uint32_t which, double out[3]);

// Whether a property name is one ufbx keeps on a 6.x model rather than
// its attribute (mani-0017): the transform, limits, visibility.
bool maniFbxIsNodeProperty(const uint8_t* name, size_t length);

// Whether an entry is flagged user-defined (U).
bool maniFbxEntryUserDefined(const maniFbx* fbx, uint32_t entry);

// Whether an entry's type is a number or an integer by ufbx's table
// (Number, double, Real, Float, Intensity; Integer, int, enum, Enum,
// Visibility, Visibility Inheritance, KTime).
bool maniFbxEntryNumeric(const maniFbx* fbx, uint32_t entry);

// Whether a 6.x Model has an attribute ufbx splits off: any but a plain
// model (no subclass or Model, unless it holds a mesh's Vertices and
// PolygonVertexIndex).
bool maniFbxModelHasAttribute(const maniFbx* fbx, uint32_t object);

// Whether a 6.x Model is a mesh: subclass Mesh, or none with the arrays.
bool maniFbxModelIsMesh(const maniFbx* fbx, uint32_t object);

// Whether an object is a Deformer of a subclass (Skin, Cluster).
bool maniFbxDeformerIs(const maniFbx* fbx, uint32_t object, const char* subclass, size_t length);

// Calls visit with each object an object is the parent of through an
// `OO` connection, in connection order.
typedef void (*maniFbxVisit)(const maniFbx* fbx, uint32_t child, void* context);
void maniFbxEachChild(const maniFbx* fbx, uint32_t object, maniFbxVisit visit, void* context);

#endif // MAUL_ANIM_SRC_FBX_OBJECT_H
