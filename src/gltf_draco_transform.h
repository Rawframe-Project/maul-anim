// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's prediction transforms: wrap, which keeps a value
// within the attribute's bounds, and the canonicalized octahedron of a
// normal's two coordinates. A transform gives a value from its
// prediction and its correction.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_TRANSFORM_H
#define MAUL_ANIM_SRC_GLTF_DRACO_TRANSFORM_H

#include "gltf_draco_read.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
    maniDracoWrap = 1,
    maniDracoOctahedron = 3,
};

typedef struct maniDracoTransform
{
    int8_t type;
    uint32_t components;
    int32_t wrapMin;
    int32_t wrapMax;
    // The octahedron's largest quantized value, 2^bits - 1.
    int32_t octahedronMax;
} maniDracoTransform;

// The transform's data: wrap bounds, or the octahedron's largest
// quantized value (odd, of 2 to 30 bits) and an unused center.
bool maniDracoTransformRead(maniDracoReader* r, maniDracoTransform* t);

// The value (the transform's components; out may be corr) from its
// prediction and correction; transform is a maniDracoTransform.
void maniDracoTransformApply(const void* transform, const uint32_t* pred, const uint32_t* corr,
                             uint32_t* out);

// The difference prediction reverted in place over count values of the
// transform's components (16 at most): each from the one before, the
// first from zeros.
void maniDracoTransformDifference(const maniDracoTransform* t, uint32_t* values, uint32_t count);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_TRANSFORM_H
