// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX files before 6.0, read into the same objects and
// connections as later files: top-level models, their Children lists, a
// skin per mesh with Link children (each link a cluster), the legacy
// frame rate, and the rest values channel defaults give.

#ifndef MAUL_ANIM_SRC_FBX_LEGACY_H
#define MAUL_ANIM_SRC_FBX_LEGACY_H

#include "fbx_document.h"

#include "maul-anim/import.h"

// Reads a file before 6.0 in place of maniFbxReadObjects; refused
// (Property) for a Model or Link without its name, or a Children value
// that is not a string; Capacity when the
// allocator fails. A name given twice is kept by its first model.
maniImportResult maniFbxReadLegacy(maniFbx* fbx);

void maniFbxReleaseLegacy(maniFbx* fbx);

#endif // MAUL_ANIM_SRC_FBX_LEGACY_H
