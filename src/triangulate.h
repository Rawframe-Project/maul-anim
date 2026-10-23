// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Delaunay triangulations of a blend space's points, for the
// builder: exact, so the same on every platform, ties included.

#ifndef MAUL_ANIM_SRC_TRIANGULATE_H
#define MAUL_ANIM_SRC_TRIANGULATE_H

#include "maul-anim/graph.h"

#include <stdint.h>

#define MANI_TRIANGULATE_MAX_POINTS MANI_GRAPH_MAX_SPACE_POINTS
#define MANI_TRIANGULATE_MAX        (2u * MANI_TRIANGULATE_MAX_POINTS - 5u)

// A triangle: its corners anticlockwise, and per corner the triangle
// across the edge opposite it (MANI_TRIANGULATE_MAX for none).
typedef struct maniTriangle
{
    uint32_t v[3];
    uint32_t n[3];
} maniTriangle;

// The working memory of a triangulation.
typedef struct maniTriangulation
{
    maniTriangle t[MANI_TRIANGULATE_MAX];
    uint32_t count;
    uint32_t hull[MANI_TRIANGULATE_MAX_POINTS];
    uint32_t hullTriangle[MANI_TRIANGULATE_MAX_POINTS];
    uint32_t hullCount;
    uint32_t queue[3 * MANI_TRIANGULATE_MAX];
    uint8_t queued[3 * MANI_TRIANGULATE_MAX];
    uint64_t keys[MANI_TRIANGULATE_MAX_POINTS];
    uint32_t order[MANI_TRIANGULATE_MAX_POINTS];
    uint32_t scratch[MANI_TRIANGULATE_MAX_POINTS];
} maniTriangulation;

// Triangulates n points (x then y each; finite, no two equal, 3 to 256
// of them) in m: the triangle count, 0 when the points are all on one
// line. m->t holds the triangles.
uint32_t maniTriangulate(const float* points, uint32_t n, maniTriangulation* m);

#endif // MAUL_ANIM_SRC_TRIANGULATE_H
