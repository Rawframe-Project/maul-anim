// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skeletons (mani-0014), found at open: skins (their clusters'
// linked models) and limb chains as units, grouped when they share a
// model, each skeleton rooted at its units' common ancestor.

#ifndef MAUL_ANIM_SRC_FBX_UNITS_H
#define MAUL_ANIM_SRC_FBX_UNITS_H

#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

// Reads the skins and limbs and groups them; refused (Hierarchy, item:
// the skin) for a skin whose linked models have no common ancestor.
maniImportResult maniFbxReadUnits(maniFbx* fbx);

void maniFbxReleaseUnits(maniFbx* fbx);

// Marks a skeleton's models (marked: a byte per model, cleared by the
// caller): its root, and every model on the paths from its units' seeds
// up to it.
void maniFbxMarkSkeleton(const maniFbx* fbx, uint32_t skeleton, uint8_t* marked);

#endif // MAUL_ANIM_SRC_FBX_UNITS_H
