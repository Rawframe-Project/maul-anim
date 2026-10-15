// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skins (mani-0015): what other importer parts ask of them.

#ifndef MAUL_ANIM_SRC_FBX_SKIN_H
#define MAUL_ANIM_SRC_FBX_SKIN_H

#include "fbx_document.h"

#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

// Whether a skin links models and every one is a bone of the skeleton.
bool maniFbxSkinWithin(const maniFbx* fbx, uint32_t skin, const maniSkeleton* skeleton);

#endif // MAUL_ANIM_SRC_FBX_SKIN_H
