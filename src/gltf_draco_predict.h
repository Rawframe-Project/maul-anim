// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's mesh prediction schemes (mani-0021), reverted as Google's
// decoder reverts them: parallelogram, multi-parallelogram, constrained
// multi-parallelogram, portable texture coordinates and geometric
// normals, over an attribute's corner table and decoding order, each
// value's prediction then given to the attribute's transform.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_PREDICT_H
#define MAUL_ANIM_SRC_GLTF_DRACO_PREDICT_H

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"
#include "gltf_draco_traverse.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
    maniDracoParallelogram = 1,
    maniDracoMultiParallelogram = 2,
    maniDracoConstrained = 4,
    maniDracoTexCoords = 5,
    maniDracoGeometricNormal = 6,
    // Constrained multi-parallelogram's contexts.
    maniDracoCreaseContexts = 4,
};

// A transform: the value from its prediction and correction (each of the
// attribute's components; out may be corr).
typedef void (*maniDracoTransformFn)(const void* context, const uint32_t* pred,
                                     const uint32_t* corr, uint32_t* out);

typedef struct maniDracoPredictor
{
    uint8_t scheme;
    uint32_t components;
    // The attribute's table and its decoder's order.
    const maniDracoTable* table;
    const maniDracoOrder* order;
    maniDracoTransformFn transform;
    const void* context;
    // Texture coordinates and normals: the positions' portable values (3
    // a value), each point's value (NULL: the point's own), how many.
    const uint32_t* positions;
    const uint32_t* positionOfPoint;
    uint32_t positionCount;
    uint32_t pointCount;
    // Constrained: each context's crease flags. Texture coordinates: the
    // orientations, used from the last.
    uint8_t* crease[maniDracoCreaseContexts];
    uint32_t creaseCount[maniDracoCreaseContexts];
    uint8_t* orientation;
    uint32_t orientationCount;
    // Normals: the flips, and the octahedron's largest quantized value.
    maniDracoRabs flips;
    int32_t octahedronMax;
} maniDracoPredictor;

// The scheme's own data before the transform's (constrained creases,
// texture coordinate orientations); values bounds the orientations.
maniDracoStatus maniDracoPredictData(maniDracoReader* r, maniArena* arena, maniDracoPredictor* p,
                                     uint32_t values);

// The geometric normal's flip bits, after the transform's data.
bool maniDracoPredictFlips(maniDracoReader* r, maniDracoPredictor* p);

// Reverts the prediction over data (entries values, each of the
// predictor's components), corrections in place; false where Google's
// decoder fails or would index outside the tables.
bool maniDracoPredictRevert(maniDracoPredictor* p, uint32_t* data, uint32_t entries);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_PREDICT_H
