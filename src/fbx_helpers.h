// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Helper bones for animation (mani-0014, mani-0016): a skeleton is built
// knowing the file's animation, so a parent whose child's compensation
// stops being exact at some sample of some stack gets its helper bone,
// as one inexact at rest does.

#ifndef MAUL_ANIM_SRC_FBX_HELPERS_H
#define MAUL_ANIM_SRC_FBX_HELPERS_H

#include "fbx_document.h"
#include "fbx_world.h"

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdbool.h>
#include <stdint.h>

// A model's rest under its parent's world, decomposed; false when that
// shears or the parent's world has no inverse.
bool maniFbxCompensate(const maniFbxWorld* worlds, uint32_t child, uint32_t parent,
                       maniTransform* out);

// Marks in helper (a byte per model) the parent of each child given
// whose compensation is not exact at a sample of some stack: each
// stack's default clip grid, the worlds of the models marked in needed
// (a needed model's parent is needed too) sampled. A stack refused, or
// refused at a sample, is skipped: its clips are refused on their own.
// Capacity when the allocator fails.
maniImportResult maniFbxAnimatedHelpers(const maniFbx* fbx, const uint8_t* needed,
                                        const uint32_t* children, uint32_t count, uint8_t* helper);

#endif // MAUL_ANIM_SRC_FBX_HELPERS_H
