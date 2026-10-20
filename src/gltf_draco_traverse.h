// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The order Draco's Edgebreaker attributes are decoded in (mani-0021): a
// traversal of a corner table, depth first or by prediction degree, from
// each face's first corner in turn, as Google's sequencer runs it. Each
// vertex reached first gives the next value, at the point of the corner
// it was reached by.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_TRAVERSE_H
#define MAUL_ANIM_SRC_GLTF_DRACO_TRAVERSE_H

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"

#include <stdint.h>

enum
{
    maniDracoDepthFirst = 0,
    maniDracoPredictionDegree = 1,
};

typedef struct maniDracoOrder
{
    // Per value, its point and the corner it was reached by.
    uint32_t* points;
    uint32_t* valueCorner;
    uint32_t count;
    // Per vertex of the table (of mapSize), its value; 0 where none was
    // reached, as in Google's decoder.
    uint32_t* vertexValue;
    uint32_t vertexCount;
} maniDracoOrder;

// Traverses the table by method, cornerPoint giving each corner's point;
// vertexValue is mapSize long (at least the table's vertices).
maniDracoStatus maniDracoTraverse(maniArena* arena, const maniDracoTable* table,
                                  const uint32_t* cornerPoint, uint32_t mapSize, uint8_t method,
                                  maniDracoOrder* out);

// Each point's value (pointToValue, pointCount long) from the faces:
// false where a corner's vertex is none or a point or value is past the
// points, as Google's UpdatePointToAttributeIndexMapping refuses.
bool maniDracoPointValues(const maniDracoTable* table, const maniDracoOrder* order,
                          const uint32_t* cornerPoint, uint32_t pointCount, uint32_t* pointToValue);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_TRAVERSE_H
