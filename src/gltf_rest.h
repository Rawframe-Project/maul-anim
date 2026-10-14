// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Node rests, matrices and placement for the glTF importer (mani-0009),
// each in the output's basis.

#ifndef MAUL_ANIM_SRC_GLTF_REST_H
#define MAUL_ANIM_SRC_GLTF_REST_H

#include "gltf_document.h"

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <stdint.h>

// A node's local transform: its TRS, or its matrix decomposed (refused
// when projective or sheared).
maniImportResult maniGltfNodeRest(const maniGltf* gltf, uint32_t node,
                                  const maniGltfOptions* options, maniTransform* restOut);

// A skeleton root's rest: its local, with the placement composed in when
// the options fold it.
maniImportResult maniGltfRootRest(const maniGltf* gltf, uint32_t root,
                                  const maniGltfOptions* options, maniTransform* restOut);

// The product of the local transforms of the nodes above root.
maniImportResult maniGltfPlacement(const maniGltf* gltf, uint32_t root,
                                   const maniGltfOptions* options, maniAffine* placementOut);

// A column-major 4x4 as an affine in the output's basis; refused
// (Matrix, at `at`, item) when its last row is not 0 0 0 1 within 1e-5.
maniImportResult maniGltfMatrixAffine(const maniGltf* gltf, const float m[16],
                                      const maniGltfOptions* options, uint64_t at, uint32_t item,
                                      maniAffine* affineOut);

#endif // MAUL_ANIM_SRC_GLTF_REST_H
