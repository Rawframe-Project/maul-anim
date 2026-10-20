// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's attribute traversals (mani-0021) over hand-built corner
// tables: depth first and by prediction degree on a 4 x 3 grid, on a
// ring of six faces around a vertex and on a 6 x 5 grid of alternating
// diagonals, every vertex reached in the order
// and by the corner Google's traversers give (research's
// draco_predict_oracle.cc); a point's value past the points refused.

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_read.h"
#include "gltf_draco_traverse.h"
#include "test_harness.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_FACES   40u
#define MAX_CORNERS (3u * MAX_FACES)

typedef struct Table
{
    maniDracoTable t;
    uint32_t opposite[MAX_CORNERS];
    uint32_t vertex[MAX_CORNERS];
    uint32_t leftMost[32];
} Table;

// Opposites by shared edges; each vertex's left-most corner as Google's
// CornerTable finds it: swinging left from its first corner to the
// boundary, or round to the corner before it.
static void Build(Table* m, const uint32_t (*faces)[3], uint32_t faceCount, uint32_t vertices)
{
    uint32_t corners = 3u * faceCount;
    for (uint32_t c = 0; c < corners; ++c)
    {
        m->vertex[c] = faces[c / 3u][c % 3u];
        m->opposite[c] = MANI_DRACO_NONE;
    }
    for (uint32_t c = 0; c < corners; ++c)
    {
        for (uint32_t d = 0; d < corners; ++d)
        {
            if (d / 3u != c / 3u &&
                m->vertex[maniDracoNext(d)] == m->vertex[maniDracoPrevious(c)] &&
                m->vertex[maniDracoPrevious(d)] == m->vertex[maniDracoNext(c)])
            {
                m->opposite[c] = d;
            }
        }
    }
    m->t = (maniDracoTable){corners, vertices, m->opposite, m->vertex, m->leftMost, NULL, NULL};
    for (uint32_t v = 0; v < vertices; ++v)
    {
        uint32_t first = 0;
        while (m->vertex[first] != v)
        {
            ++first;
        }
        uint32_t c = first;
        uint32_t left = first;
        while (c != MANI_DRACO_NONE)
        {
            left = c;
            c = maniDracoSwingLeft(&m->t, c);
            c = c == first ? MANI_DRACO_NONE : c;
        }
        m->leftMost[v] = left;
    }
}

static const uint32_t kGrid[12][3] = {
    {0, 1, 5}, {0, 5, 4}, {1, 2, 6},  {1, 6, 5},  {2, 3, 7},  {2, 7, 6},
    {4, 5, 9}, {4, 9, 8}, {5, 6, 10}, {5, 10, 9}, {6, 7, 11}, {6, 11, 10},
};

static const uint32_t kFan[12][3] = {
    {12, 0, 1}, {12, 1, 2}, {12, 2, 3}, {12, 3, 4}, {12, 4, 5}, {12, 5, 0},
    {1, 0, 6},  {2, 1, 7},  {3, 2, 8},  {4, 3, 9},  {5, 4, 10}, {0, 5, 11},
};

// Google's orders: each vertex reached, then the corner reaching it.
static const uint32_t kGridDepth[24] = {1, 1,  5, 2, 0, 0,  4, 5,  9,  20, 10, 28,
                                        6, 25, 2, 7, 7, 16, 3, 13, 11, 32, 8,  23};
static const uint32_t kGridDegree[24] = {1, 1,  5, 2, 0, 0,  4,  5,  9, 20, 10, 28,
                                         6, 25, 2, 7, 7, 16, 11, 32, 3, 13, 8,  23};
static const uint32_t kFanDepth[26] = {0,  1, 1,  2, 12, 0, 2,  5,  7,  23, 3,  8, 8,
                                       26, 4, 11, 9, 29, 5, 14, 10, 32, 11, 35, 6, 20};
static const uint32_t kFanDegree[26] = {0,  1, 1,  2, 12, 0,  2,  5,  7,  23, 3,  8, 8,
                                        26, 4, 11, 5, 14, 11, 35, 10, 32, 9,  29, 6, 20};

static const uint32_t kMixedDepth[60] = {
    1,  1,   7,  2,   0,  0,   6,  5,   12, 32,  13, 34, 14, 40,  8,  37, 2,  9,  9,  16,
    3,  13,  4,  19,  10, 22,  11, 28,  5,  25,  16, 56, 17, 58,  23, 86, 22, 89, 28, 116,
    29, 118, 21, 108, 27, 113, 26, 107, 20, 102, 19, 96, 25, 101, 24, 95, 18, 90, 15, 73};
static const uint32_t kMixedDegree[60] = {
    1,  1,  7,  2,   0,  0,  6,  5,   12, 32,  13, 34,  14, 40,  19, 68, 18, 65, 24, 92,
    25, 94, 26, 100, 20, 97, 21, 76,  27, 106, 28, 112, 22, 109, 16, 81, 15, 78, 9,  45,
    8,  42, 2,  9,   23, 88, 29, 118, 17, 85,  11, 57,  10, 54,  4,  21, 3,  18, 5,  25};

// A 6 x 5 grid whose cells alternate their diagonal, as the oracle's.
static void Mixed(uint32_t faces[40][3])
{
    uint32_t n = 0;
    for (uint32_t r = 0; r < 4; ++r)
    {
        for (uint32_t c = 0; c < 5; ++c)
        {
            uint32_t a = r * 6u + c;
            bool flip = (r + c) % 2u == 1u;
            uint32_t t0[3] = {a, a + 1u, flip ? a + 6u : a + 7u};
            uint32_t t1[3] = {flip ? a + 1u : a, a + 7u, a + 6u};
            memcpy(faces[n++], t0, sizeof t0);
            memcpy(faces[n++], t1, sizeof t1);
        }
    }
}

static bool Order(const uint32_t (*faces)[3], uint32_t faceCount, uint32_t vertices, uint8_t method,
                  const uint32_t* expected)
{
    Table m;
    Build(&m, faces, faceCount, vertices);
    uint32_t cornerPoint[MAX_CORNERS];
    for (uint32_t c = 0; c < m.t.corners; ++c)
    {
        cornerPoint[c] = m.vertex[c];
    }
    maniAllocator allocator = {0};
    maniArena arena = maniArenaMake(&allocator);
    maniDracoOrder o;
    bool same =
        maniDracoTraverse(&arena, &m.t, cornerPoint, vertices, method, &o) == maniDracoDone &&
        o.count == vertices;
    for (uint32_t i = 0; same && i < vertices; ++i)
    {
        same = m.vertex[o.valueCorner[i]] == expected[2 * i] &&
               o.valueCorner[i] == expected[2 * i + 1] && o.points[i] == expected[2 * i] &&
               o.vertexValue[expected[2 * i]] == i;
    }
    maniArenaRelease(&arena);
    return same;
}

static void TestOrders(void)
{
    uint32_t mixed[40][3];
    Mixed(mixed);
    CHECK(Order(kGrid, 12, 12, maniDracoDepthFirst, kGridDepth), "grid, depth first");
    CHECK(Order(kGrid, 12, 12, maniDracoPredictionDegree, kGridDegree), "grid, prediction degree");
    CHECK(Order(kFan, 12, 13, maniDracoDepthFirst, kFanDepth), "fan, depth first");
    CHECK(Order(kFan, 12, 13, maniDracoPredictionDegree, kFanDegree), "fan, prediction degree");
    CHECK(Order((const uint32_t (*)[3])mixed, 40, 30, maniDracoDepthFirst, kMixedDepth),
          "mixed diagonals, depth first");
    CHECK(Order((const uint32_t (*)[3])mixed, 40, 30, maniDracoPredictionDegree, kMixedDegree),
          "mixed diagonals, prediction degree");
}

static void TestPointValues(void)
{
    Table m;
    Build(&m, kGrid, 12, 12);
    uint32_t cornerPoint[MAX_CORNERS];
    uint32_t vertexValue[12];
    uint32_t pointToValue[12];
    for (uint32_t c = 0; c < m.t.corners; ++c)
    {
        cornerPoint[c] = m.vertex[c];
    }
    for (uint32_t v = 0; v < 12; ++v)
    {
        vertexValue[v] = v;
    }
    maniDracoOrder o = {NULL, NULL, 12, vertexValue, 12};
    CHECK(maniDracoPointValues(&m.t, &o, cornerPoint, 12, pointToValue) && pointToValue[7] == 7,
          "each point its vertex's value");
    vertexValue[3] = 12;
    CHECK(!maniDracoPointValues(&m.t, &o, cornerPoint, 12, pointToValue),
          "a value past the points");
    vertexValue[3] = 3;
    cornerPoint[5] = 12;
    CHECK(!maniDracoPointValues(&m.t, &o, cornerPoint, 12, pointToValue),
          "a point past the points");
}

int main(void)
{
    TestOrders();
    TestPointValues();
    return s_failures == 0 ? 0 : 1;
}
