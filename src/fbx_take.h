// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX 6.x takes as stacks (mani-0017), and a stack of either model
// read by its index.

#ifndef MAUL_ANIM_SRC_FBX_TAKE_H
#define MAUL_ANIM_SRC_FBX_TAKE_H

#include "fbx_anim.h"
#include "fbx_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

// Reads stack k: an AnimationStack object (7.x, maniFbxReadAnimation) or
// a Take node (6.x): one base layer, its Model children's channels bound
// to the models their keys name (`Transform`'s T, R and S as the Lcl
// properties; a property not the model's own marked as its
// attribute's). Refused (Property) for a take, model or Transform
// channel without its name, a property driven twice, or as its keys
// are; Capacity when the allocator fails.
maniImportResult maniFbxReadStack(const maniFbx* fbx, uint32_t k, maniFbxAnimation* out);

#endif // MAUL_ANIM_SRC_FBX_TAKE_H
