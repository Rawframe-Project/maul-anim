// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bone names for glTF nodes (mani-0009): a node's name, or with `#` and
// its node index appended when it has none or an earlier node in the
// document has the same; fixed per document.

#ifndef MAUL_ANIM_SRC_GLTF_NAMES_H
#define MAUL_ANIM_SRC_GLTF_NAMES_H

#include "gltf_document.h"

#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

// Names every node at open, into the document.
maniImportResult maniGltfReadNames(maniGltf* gltf);

void maniGltfReleaseNames(maniGltf* gltf);

// A node's bone name, not terminated.
const char* maniGltfNodeName(const maniGltf* gltf, uint32_t node, uint32_t* lengthOut);

// The skeleton's bone named as the node, or MANI_NO_BONE.
uint32_t maniGltfNodeBone(const maniGltf* gltf, const maniSkeleton* skeleton, uint32_t node);

#endif // MAUL_ANIM_SRC_GLTF_NAMES_H
