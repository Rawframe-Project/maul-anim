// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's node hierarchy and skins: parents,
// depths and common ancestors, each skin's common root, and the
// skeletons skins sharing nodes make.

#ifndef MAUL_ANIM_SRC_GLTF_NODE_H
#define MAUL_ANIM_SRC_GLTF_NODE_H

#include "gltf_document.h"

#include "maul-anim/import.h"

#include <stdint.h>

// Reads `nodes` and `skins`, checks the hierarchy and groups the skins.
maniImportResult maniGltfReadNodes(maniGltf* gltf);

// Frees what maniGltfReadNodes allocated.
void maniGltfReleaseNodes(maniGltf* gltf);

// A skin's joints array value and its count.
uint32_t maniGltfSkinJoints(const maniGltf* gltf, uint32_t skin, uint32_t* countOut);

// A joint's node: the joints array's element as an index (checked at
// open).
uint32_t maniGltfJointNode(const maniGltf* gltf, uint32_t element);

#endif // MAUL_ANIM_SRC_GLTF_NODE_H
