// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX models' worlds at rest (mani-0014), as ufbx's ufbxi_update_node
// evaluates them: normal inheritance composes the parent's world; the
// RrSs and Rrs types hang from the parent's unscaled world, with the
// translation scaled by the parent's inherited scale and the scale by
// their scale source's.

#ifndef MAUL_ANIM_SRC_FBX_WORLD_H
#define MAUL_ANIM_SRC_FBX_WORLD_H

#include "fbx_document.h"
#include "fbx_transform.h"

#include "maul-anim/import.h"

#include <stdint.h>

// A model's evaluated transform.
typedef struct maniFbxWorld
{
    // Its world, and its world without its inherited scale.
    maniFbxMatrix world;
    maniFbxMatrix unscaled;
    // Its local as composed: for RrSs and Rrs, the scale and
    // translation scaled as inherited, relative to the parent's
    // unscaled world.
    maniFbxTrs transform;
    double inheritScale[3];
    uint32_t inherit;
    // The model whose inherited scale it takes (RrSs: its parent; Rrs:
    // its parent's source), or MANI_FBX_NONE.
    uint32_t scaleSource;
} maniFbxWorld;

// Evaluates one model from its local, given the evaluations of the
// models so far and its parent model (MANI_FBX_NONE for the root).
void maniFbxEvaluateModel(const maniFbxWorld* worlds, uint32_t parent, const maniFbxLocal* local,
                          maniFbxWorld* out);

// Evaluates models at rest, parents first, into worlds (one per model):
// those needed (a byte per model; a needed model's parent is needed
// too), or every one when needed is NULL; refused as maniFbxModelLocal.
maniImportResult maniFbxEvaluateWorlds(const maniFbx* fbx, const uint8_t* needed,
                                       maniFbxWorld* worlds);

#endif // MAUL_ANIM_SRC_FBX_WORLD_H
