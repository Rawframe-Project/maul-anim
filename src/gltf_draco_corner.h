// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's corner tables: a mesh's corners, each one's vertex
// and opposite corner, and each vertex's left-most corner; and an
// attribute's table over the same corners, whose seam edges cut the
// opposites and split vertices. Every walk is bounded and every index
// checked: an invalid corner or vertex reads as none, never past an
// array.

#ifndef MAUL_ANIM_SRC_GLTF_DRACO_CORNER_H
#define MAUL_ANIM_SRC_GLTF_DRACO_CORNER_H

#include "arena.h"

#include <stdbool.h>
#include <stdint.h>

#define MANI_DRACO_NONE UINT32_MAX

// A table. The mesh's has no seams (seam NULL); an attribute's shares its
// mesh's opposites and adds seams, its own vertices and left-most corners.
typedef struct maniDracoTable
{
    uint32_t corners;
    uint32_t vertices;
    uint32_t* opposite;
    uint32_t* vertex;
    uint32_t* leftMost;
    // Edges on a seam (per corner) and base vertices on one (per vertex).
    uint8_t* seam;
    uint8_t* vertexSeam;
} maniDracoTable;

static inline uint32_t maniDracoNext(uint32_t c)
{
    return c == MANI_DRACO_NONE ? c : (c % 3u == 2u ? c - 2u : c + 1u);
}

static inline uint32_t maniDracoPrevious(uint32_t c)
{
    return c == MANI_DRACO_NONE ? c : (c % 3u == 0u ? c + 2u : c - 1u);
}

static inline uint32_t maniDracoOpposite(const maniDracoTable* t, uint32_t c)
{
    return c >= t->corners || (t->seam != NULL && t->seam[c] != 0) ? MANI_DRACO_NONE
                                                                   : t->opposite[c];
}

static inline uint32_t maniDracoVertex(const maniDracoTable* t, uint32_t c)
{
    return c >= t->corners ? MANI_DRACO_NONE : t->vertex[c];
}

static inline uint32_t maniDracoLeftMost(const maniDracoTable* t, uint32_t v)
{
    return v >= t->vertices ? MANI_DRACO_NONE : t->leftMost[v];
}

static inline uint32_t maniDracoSwingRight(const maniDracoTable* t, uint32_t c)
{
    return maniDracoPrevious(maniDracoOpposite(t, maniDracoPrevious(c)));
}

static inline uint32_t maniDracoSwingLeft(const maniDracoTable* t, uint32_t c)
{
    return maniDracoNext(maniDracoOpposite(t, maniDracoNext(c)));
}

// A mesh's table of faces faces and room for vertices vertices, every
// corner without a vertex or an opposite, no vertex yet; false when the
// arena refuses.
bool maniDracoTableInit(maniArena* arena, uint32_t faces, uint32_t vertices, maniDracoTable* out);

// An attribute's table over a mesh's, no seams yet; false when the arena
// refuses.
bool maniDracoSeamsInit(maniArena* arena, const maniDracoTable* mesh, maniDracoTable* out);

// Marks the edge opposite corner c a seam, and its opposite's (Google's
// AddSeamEdge); false for a corner whose vertices are none.
bool maniDracoAddSeam(maniDracoTable* attribute, const maniDracoTable* mesh, uint32_t c);

// The attribute's vertices from its seams (Google's RecomputeVertices);
// false when a seam's walk returns to where it began or the arena refuses.
bool maniDracoRecompute(maniArena* arena, maniDracoTable* attribute, const maniDracoTable* mesh);

#endif // MAUL_ANIM_SRC_GLTF_DRACO_CORNER_H
