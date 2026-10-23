// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco's mesh predictions (mani-0021) over hand-built corner tables,
// each correction added to its prediction in 32 bits: constrained
// multi-parallelogram and multi-parallelogram at a vertex of six
// parallelograms, the first four taken and creases left out; portable
// texture coordinates and area-weighted normals on a 4 x 3 grid, the
// values Google's predictors give there (tools/
// draco_predict_oracle.cc), normals at three position scales (sums past
// 2^29, and none at all); the orientation and crease counts refused past
// their bounds.

#include "arena.h"
#include "gltf_draco_corner.h"
#include "gltf_draco_predict.h"
#include "gltf_draco_read.h"
#include "gltf_draco_traverse.h"
#include "test_harness.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_FACES    16u
#define MAX_VERTICES 16u

typedef struct Mesh
{
    maniDracoTable table;
    maniDracoOrder order;
    uint32_t opposite[3 * MAX_FACES];
    uint32_t vertex[3 * MAX_FACES];
    uint32_t points[MAX_VERTICES];
    uint32_t valueCorner[MAX_VERTICES];
    uint32_t vertexValue[MAX_VERTICES];
} Mesh;

// A table of these faces, opposites by shared edges; values in vertex
// order, each at its vertex's first corner.
static void Build(Mesh* m, const uint32_t (*faces)[3], uint32_t faceCount, uint32_t vertices)
{
    memset(m, 0, sizeof *m);
    uint32_t corners = 3u * faceCount;
    for (uint32_t c = 0; c < corners; ++c)
    {
        m->vertex[c] = faces[c / 3u][c % 3u];
        m->opposite[c] = MANI_DRACO_NONE;
    }
    for (uint32_t c = 0; c < corners; ++c)
    {
        uint32_t a = m->vertex[maniDracoNext(c)];
        uint32_t b = m->vertex[maniDracoPrevious(c)];
        for (uint32_t d = 0; d < corners; ++d)
        {
            if (d / 3u != c / 3u && m->vertex[maniDracoNext(d)] == b &&
                m->vertex[maniDracoPrevious(d)] == a)
            {
                m->opposite[c] = d;
            }
        }
    }
    m->table = (maniDracoTable){corners, vertices, m->opposite, m->vertex, NULL, NULL, NULL};
    for (uint32_t v = 0; v < vertices; ++v)
    {
        uint32_t c = 0;
        while (m->vertex[c] != v)
        {
            ++c;
        }
        m->points[v] = v;
        m->valueCorner[v] = c;
        m->vertexValue[v] = v;
    }
    m->order = (maniDracoOrder){m->points, m->valueCorner, vertices, m->vertexValue, vertices};
}

// The test's transform: the correction added to the prediction.
static void Add(const void* context, const uint32_t* pred, const uint32_t* corr, uint32_t* out)
{
    const uint32_t* components = context;
    for (uint32_t c = 0; c < *components; ++c)
    {
        out[c] = pred[c] + corr[c];
    }
}

static void Predictor(maniDracoPredictor* p, const Mesh* m, uint8_t scheme,
                      const uint32_t* components)
{
    *p = (maniDracoPredictor){0};
    p->scheme = scheme;
    p->components = *components;
    p->table = &m->table;
    p->order = &m->order;
    p->transform = Add;
    p->context = components;
}

// A vertex (12) inside a ring of six (0 to 5), each ring edge's outer
// face with its own vertex (6 to 11): only the centre has parallelograms,
// one per ring edge.
static const uint32_t kFan[12][3] = {
    {12, 0, 1}, {12, 1, 2}, {12, 2, 3}, {12, 3, 4}, {12, 4, 5}, {12, 5, 0},
    {1, 0, 6},  {2, 1, 7},  {3, 2, 8},  {4, 3, 9},  {5, 4, 10}, {0, 5, 11},
};

static void TestParallelograms(void)
{
    static const uint32_t kOne = 1;
    Mesh m;
    Build(&m, kFan, 12, 13);
    uint32_t corr[13];
    uint32_t data[13];
    int32_t sum[12];
    int32_t running = 0;
    for (uint32_t i = 0; i < 13; ++i)
    {
        corr[i] = (uint32_t)((int32_t)(i * 29u % 41u) - 20);
        running += (int32_t)corr[i];
        if (i < 12)
        {
            sum[i] = running;
        }
    }
    // Swinging left from the centre's corner in face 0 meets faces 0 to 5
    // in order; the parallelogram across ring edge i is that edge's ends
    // less its outer vertex.
    int32_t four = 0;
    int32_t six = 0;
    for (uint32_t i = 0; i < 6; ++i)
    {
        int32_t pred = sum[i] + sum[(i + 1) % 6] - sum[6 + i];
        four += i < 4 ? pred : 0;
        six += pred;
    }
    maniDracoPredictor p;
    uint8_t creases[4] = {0, 0, 0, 0};
    memcpy(data, corr, sizeof data);
    Predictor(&p, &m, maniDracoConstrained, &kOne);
    p.crease[3] = creases;
    p.creaseCount[3] = 4;
    CHECK(maniDracoPredictRevert(&p, data, 13) &&
              (int32_t)data[12] == four / 4 + (int32_t)corr[12] && (int32_t)data[11] == sum[11],
          "constrained: the first four parallelograms");
    creases[1] = 1;
    memcpy(data, corr, sizeof data);
    Predictor(&p, &m, maniDracoConstrained, &kOne);
    p.crease[3] = creases;
    p.creaseCount[3] = 4;
    int32_t pred1 = sum[1] + sum[2] - sum[7];
    CHECK(maniDracoPredictRevert(&p, data, 13) &&
              (int32_t)data[12] == (four - pred1) / 3 + (int32_t)corr[12],
          "constrained: a crease left out");
    p.creaseCount[3] = 3;
    memcpy(data, corr, sizeof data);
    CHECK(!maniDracoPredictRevert(&p, data, 13), "constrained: too few crease flags");
    memcpy(data, corr, sizeof data);
    Predictor(&p, &m, maniDracoMultiParallelogram, &kOne);
    CHECK(maniDracoPredictRevert(&p, data, 13) && (int32_t)data[12] == six / 6 + (int32_t)corr[12],
          "multi-parallelogram: all six");
}

// A 4 x 3 grid, two triangles per cell, and its positions (a bump at
// vertex 5), as the oracle builds them.
static void Grid(Mesh* m, uint32_t* positions, int32_t scale)
{
    uint32_t faces[12][3];
    uint32_t n = 0;
    for (uint32_t r = 0; r + 1 < 3; ++r)
    {
        for (uint32_t c = 0; c + 1 < 4; ++c)
        {
            uint32_t a = r * 4u + c;
            uint32_t e = a + 5u;
            faces[n][0] = a;
            faces[n][1] = a + 1u;
            faces[n++][2] = e;
            faces[n][0] = a;
            faces[n][1] = e;
            faces[n++][2] = a + 4u;
        }
    }
    Build(m, (const uint32_t (*)[3])faces, 12, 12);
    for (int32_t v = 0; v < 12; ++v)
    {
        int32_t p[3] = {(v % 4) * 100 + v * 7, (v / 4) * 90 - v * 3, v == 5 ? 60 : (v * 13) % 17};
        for (uint32_t k = 0; k < 3; ++k)
        {
            positions[3 * v + k] = (uint32_t)(p[k] * scale);
        }
    }
}

static void TestTexCoords(void)
{
    static const uint32_t kTwo = 2;
    static const int32_t kDecoded[24] = {-11, -14, -8, -4, -14, 1,   -6,  1,   -12, -19, -25, -9,
                                         -7,  20,  -9, 5,  -3,  -15, -32, -19, -31, 31,  -15, -9};
    static const uint8_t kOrientations[12] = {1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0};
    Mesh m;
    uint32_t positions[36];
    Grid(&m, positions, 1);
    uint32_t data[24];
    for (uint32_t i = 0; i < 12; ++i)
    {
        data[2 * i] = (uint32_t)((int32_t)(i * 37u % 23u) - 11);
        data[2 * i + 1] = (uint32_t)((int32_t)(i * 53u % 29u) - 14);
    }
    uint8_t orientation[12];
    memcpy(orientation, kOrientations, sizeof orientation);
    maniDracoPredictor p;
    Predictor(&p, &m, maniDracoTexCoords, &kTwo);
    p.positions = positions;
    p.positionCount = 12;
    p.pointCount = 12;
    p.orientation = orientation;
    p.orientationCount = 12;
    CHECK(maniDracoPredictRevert(&p, data, 12) && memcmp(data, kDecoded, sizeof data) == 0 &&
              p.orientationCount == 6,
          "texture coordinates: Google's, orientations from the last");
}

// One triangle whose last coordinate takes the full prediction with
// cx * norm = 8, where floor(sqrt) is 2 and sqrt(9) would be 3: Google's
// prediction (3, -5).
static void TestTexRoot(void)
{
    static const uint32_t kTwo = 2;
    static const uint32_t kFace[1][3] = {{0, 1, 2}};
    static const uint32_t kPositions[9] = {0, 0, 0, 1, 1, 0, 0, 0, 2};
    Mesh m;
    Build(&m, kFace, 1, 3);
    uint32_t data[6] = {0, 0, 5, 3, 0, 0};
    uint8_t orientation[1] = {1};
    maniDracoPredictor p;
    Predictor(&p, &m, maniDracoTexCoords, &kTwo);
    p.positions = kPositions;
    p.positionCount = 3;
    p.pointCount = 3;
    p.orientation = orientation;
    p.orientationCount = 1;
    // Value 1 takes value 0's (0, 0) plus its correction (5, 3).
    CHECK(maniDracoPredictRevert(&p, data, 3) && data[2] == 5u && data[3] == 3u && data[4] == 3u &&
              (int32_t)data[5] == -5,
          "texture coordinates: the square root floored");
}

static void TestNormals(void)
{
    static const uint32_t kTwo = 2;
    static const int32_t kScales[3] = {1, 3001, 0};
    static const int32_t kDecoded[3][24] = {
        {81,  235, 173, 37,  122, 244, 126, 249, 102, 51, 135, 257,
         111, 221, 121, 251, 125, 16,  174, 216, 101, 12, 133, 0},
        {81,  235, 173, 37,  122, 244, 126, 249, 102, 51, 135, 257,
         111, 221, 121, 251, 125, 16,  174, 216, 101, 12, 133, 0},
        {124, 123, 255, 255, 125, 124, 129, 129, 253, 252, 130, 130,
         127, 126, 124, 131, 255, 254, 125, 123, 256, 255, 253, 251},
    };
    // Google's rABS encoding of the flips 010010001011.
    static const uint8_t kFlips[6] = {149, 4, 248, 201, 195, 128};
    for (uint32_t s = 0; s < 3; ++s)
    {
        Mesh m;
        uint32_t positions[36];
        Grid(&m, positions, kScales[s]);
        uint32_t data[24];
        for (uint32_t i = 0; i < 12; ++i)
        {
            data[2 * i] = (uint32_t)((int32_t)(i * 11u % 7u) - 3);
            data[2 * i + 1] = (uint32_t)((int32_t)(i * 5u % 9u) - 4);
        }
        maniDracoPredictor p;
        Predictor(&p, &m, maniDracoGeometricNormal, &kTwo);
        p.positions = positions;
        p.positionCount = 12;
        p.pointCount = 12;
        p.octahedronMax = 255;
        maniDracoReader r = {kFlips, kFlips + sizeof kFlips};
        CHECK(maniDracoPredictFlips(&r, &p) && maniDracoPredictRevert(&p, data, 12) &&
                  memcmp(data, kDecoded[s], sizeof data) == 0,
              s == 0 ? "normals: Google's" : (s == 1 ? "normals: sums scaled" : "normals: none"));
    }
}

// The orientation count past the values and the crease count past the
// corners refused, valid rABS bytes after them.
static void TestDataBounds(void)
{
    static const uint32_t kTwo = 2;
    static const uint8_t kOrientations[10] = {25, 0, 0, 0, 149, 4, 248, 201, 195, 128};
    // 37 flags, then the other three contexts' counts of none.
    static const uint8_t kCreases[10] = {37, 149, 4, 248, 201, 195, 128, 0, 0, 0};
    Mesh m;
    uint32_t positions[36];
    Grid(&m, positions, 1);
    maniAllocator allocator = {0};
    maniArena arena = maniArenaMake(&allocator);
    maniDracoPredictor p;
    Predictor(&p, &m, maniDracoTexCoords, &kTwo);
    maniDracoReader r = {kOrientations, kOrientations + sizeof kOrientations};
    CHECK(maniDracoPredictData(&r, &arena, &p, 24) == maniDracoInvalid,
          "25 orientations for 24 values");
    r = (maniDracoReader){kOrientations, kOrientations + sizeof kOrientations};
    CHECK(maniDracoPredictData(&r, &arena, &p, 25) == maniDracoDone && p.orientationCount == 25,
          "25 orientations for 25 values");
    Predictor(&p, &m, maniDracoConstrained, &kTwo);
    r = (maniDracoReader){kCreases, kCreases + sizeof kCreases};
    CHECK(maniDracoPredictData(&r, &arena, &p, 24) == maniDracoInvalid,
          "37 crease flags for 36 corners");
    maniArenaRelease(&arena);
}

int main(void)
{
    TestParallelograms();
    TestTexCoords();
    TestTexRoot();
    TestNormals();
    TestDataBounds();
    return s_failures == 0 ? 0 : 1;
}
