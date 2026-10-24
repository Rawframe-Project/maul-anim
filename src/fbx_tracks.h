// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An FBX clip's float tracks: a bone's other animated
// numbers and the blend shape weights of meshes in the skeleton.

#ifndef MAUL_ANIM_SRC_FBX_TRACKS_H
#define MAUL_ANIM_SRC_FBX_TRACKS_H

#include "fbx_clip_state.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The float tracks of bone b's model's animated properties outside its
// local transform's inputs (not a 6.x attribute's), into c->floats.
maniImportResult maniFbxBoneFloatTracks(maniFbxClipState* c, uint32_t b);

// The blend shape tracks of each channel the stack drives on a mesh in
// the skeleton: their shapes counted into shapesOut, and, unless
// counting, their tracks made into c->floats; refused (Name, Property)
// as their names and Shape children are.
maniImportResult maniFbxBlendTracks(maniFbxClipState* c, bool counting, size_t* shapesOut);

#endif // MAUL_ANIM_SRC_FBX_TRACKS_H
