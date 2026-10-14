// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX models (mani-0014): the objects of kind Model in object order,
// each one's parent model from its `OO` connections, its children in
// connection order, and its bone name; read once at open.

#ifndef MAUL_ANIM_SRC_FBX_MODEL_H
#define MAUL_ANIM_SRC_FBX_MODEL_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

// Reads the models; refused (Hierarchy, item: the model) for a model
// with two parents or in a cycle.
maniImportResult maniFbxReadModels(maniFbx* fbx);

void maniFbxReleaseModels(maniFbx* fbx);

// The model an object is, or MANI_FBX_NONE.
uint32_t maniFbxModelOf(const maniFbx* fbx, uint32_t object);

// A model's children in connection order (the root's for
// fbx->modelCount: models with no parent model), as a run of model
// indices; their count.
uint32_t maniFbxModelChildren(const maniFbx* fbx, uint32_t model, const uint32_t** out);

// A model's bone name, not terminated.
const char* maniFbxModelName(const maniFbx* fbx, uint32_t model, uint32_t* lengthOut);

#endif // MAUL_ANIM_SRC_FBX_MODEL_H
