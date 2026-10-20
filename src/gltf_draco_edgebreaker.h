// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's Edgebreaker connectivity (mani-0021), standard and valence, as
// Google's decoder reads bitstream 2.2: the mesh's corner table, each
// attribute data's seams and table, and the points the corners map to.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_EDGEBREAKER_H
#define MAUL_ANIM_SRC_GLTF_DRACO_EDGEBREAKER_H

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"

#include <stdint.h>

typedef struct maniDracoEdgebreaker
{
    maniDracoTable mesh;
    // Vertices of the mesh's table that are holes' (per table vertex).
    uint8_t* hole;
    // The attribute data: each one's table over the mesh's (seams), how
    // many.
    maniDracoTable* data;
    uint32_t dataCount;
    // Each corner's point; the points.
    uint32_t* cornerPoint;
    uint32_t pointCount;
} maniDracoEdgebreaker;

// Reads the connectivity at the reader, of at most maxPoints points and
// maxFaces faces; counts past them are refused before anything is taken.
maniDracoStatus maniDracoReadEdgebreaker(maniDracoReader* r, maniArena* arena, uint32_t maxPoints,
                                         uint32_t maxFaces, maniDracoEdgebreaker* out);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_EDGEBREAKER_H
