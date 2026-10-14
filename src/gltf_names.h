// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bone names for glTF nodes (mani-0009): a node's name, or with
// `#` and its node index appended when it has none or repeats an
// earlier bone's.

#ifndef MAUL_ANIM_SRC_GLTF_NAMES_H
#define MAUL_ANIM_SRC_GLTF_NAMES_H

#include "gltf_document.h"

#include "maul-anim/import.h"

#include <stddef.h>
#include <stdint.h>

// Bone b's name is text[starts[b]] to text[starts[b + 1]].
typedef struct maniGltfNames
{
    char* text;
    uint32_t* starts;
    size_t textSize;
    uint32_t count;
} maniGltfNames;

// Names the bones whose nodes are given, in that order.
maniImportResult maniGltfNameBones(const maniGltf* gltf, const uint32_t* nodes, uint32_t count,
                                   maniGltfNames* namesOut);

void maniGltfReleaseNames(const maniGltf* gltf, maniGltfNames* names);

#endif // MAUL_ANIM_SRC_GLTF_NAMES_H
