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

// A take channel's components (mani-0017): the node itself when it holds
// a Key or Default, else its first three named Channel children holding
// one; their count.
uint32_t maniFbxChannelComponents(const maniFbx* fbx, uint32_t node, uint32_t out[3]);

// A component's Default, 0 when absent or not a number.
double maniFbxChannelDefault(const maniFbx* fbx, uint32_t component);

// Reads stack k: an AnimationStack object (7.x, maniFbxReadAnimation) or
// a Take node (6.x): one base layer, its Model children's channels bound
// to the models their keys name (`Transform`'s T, R and S as the Lcl
// properties; a property not the model's own marked as its
// attribute's). Refused (Property) for a take, model or Transform
// channel without its name, a property driven twice, or as its keys
// are; Capacity when the allocator fails.
maniImportResult maniFbxReadStack(const maniFbx* fbx, uint32_t k, maniFbxAnimation* out);
// The same with only the channels of the property named from a 7.x
// stack (a 6.x take is read whole); refused only where the whole read
// is.
maniImportResult maniFbxReadStackOf(const maniFbx* fbx, uint32_t k, const char* name, size_t length,
                                    maniFbxAnimation* out);

#endif // MAUL_ANIM_SRC_FBX_TAKE_H
