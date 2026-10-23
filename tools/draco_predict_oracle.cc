// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Google's depth first and prediction degree traversals
// (their visiting orders), and its portable texture coordinate and
// area-weighted normal predictors (d429de8) run in their decoders' loops,
// over small hand-built meshes, a correction added to each prediction in 32 bits (the
// library's unit test uses the same transform), the decoded values
// printed for maul-anim's test/test_gltf_draco_predict.c and
// test_gltf_draco_traverse.c. The meshes: a 4 x 3 grid of vertices with
// a bump, two triangles per cell (predictions: values in vertex order,
// each at its vertex's first corner), and a ring of six faces around a
// vertex with an outer face on each ring edge.
//
// Build: g++-14 -O2 -std=c++17 -I<draco>/src -I<draco build>
//        draco_predict_oracle.cc <draco build>/libdraco.a
// Prints expected values only; never linked into the library.

#include "draco/attributes/point_attribute.h"
#include "draco/compression/attributes/normal_compression_utils.h"
#include "draco/compression/bit_coders/rans_bit_encoder.h"
#include "draco/core/encoder_buffer.h"
#include "draco/compression/attributes/prediction_schemes/mesh_prediction_scheme_data.h"
#include "draco/compression/attributes/prediction_schemes/mesh_prediction_scheme_geometric_normal_predictor_area.h"
#include "draco/compression/attributes/prediction_schemes/mesh_prediction_scheme_tex_coords_portable_predictor.h"
#include "draco/compression/mesh/traverser/depth_first_traverser.h"
#include "draco/compression/mesh/traverser/max_prediction_degree_traverser.h"
#include "draco/mesh/corner_table.h"
#include "draco/mesh/mesh.h"

#include <cstdint>
#include <cstdio>
#include <vector>

namespace
{

constexpr int kColumns = 4;
constexpr int kRows = 3;
constexpr int kVertices = kColumns * kRows;

int32_t Wrap(int64_t x)
{
    return static_cast<int32_t>(static_cast<uint32_t>(x));
}

// Records each vertex a traversal reaches and the corner it reaches it by.
struct Observer
{
    std::vector<int>* order;
    void OnNewFaceVisited(draco::FaceIndex) {}
    void OnNewVertexVisited(draco::VertexIndex v, draco::CornerIndex c)
    {
        order->push_back(static_cast<int>(v.value()));
        order->push_back(static_cast<int>(c.value()));
    }
};

template <class Traverser>
void PrintOrder(const char* name, const draco::CornerTable& table)
{
    std::vector<int> order;
    Traverser t;
    t.Init(&table, Observer{&order});
    t.OnTraversalStart();
    for (int f = 0; f < static_cast<int>(table.num_faces()); ++f)
    {
        t.TraverseFromCorner(draco::CornerIndex(3 * f));
    }
    t.OnTraversalEnd();
    std::printf("static const uint32_t %s[%zu] = {", name, order.size());
    for (size_t i = 0; i < order.size(); ++i)
    {
        std::printf("%s%d", i == 0 ? "" : ", ", order[i]);
    }
    std::printf("};\n");
}

// The traversals' orders, vertex then corner, on a table of these faces.
void PrintOrders(const char* depth, const char* degree, const int (*faces)[3], int count)
{
    draco::IndexTypeVector<draco::FaceIndex, draco::CornerTable::FaceType> f;
    for (int i = 0; i < count; ++i)
    {
        f.push_back({draco::VertexIndex(faces[i][0]), draco::VertexIndex(faces[i][1]),
                     draco::VertexIndex(faces[i][2])});
    }
    draco::CornerTable table;
    table.Init(f);
    PrintOrder<draco::DepthFirstTraverser<draco::CornerTable, Observer>>(depth, table);
    PrintOrder<draco::MaxPredictionDegreeTraverser<draco::CornerTable, Observer>>(degree, table);
}

// One triangle whose third vertex's texture coordinate takes the full
// prediction with cx * norm = 8, where floor(sqrt) is 2 and sqrt(9) 3.
void TexSqrt()
{
    draco::Mesh mesh;
    mesh.AddFace({draco::PointIndex(0), draco::PointIndex(1), draco::PointIndex(2)});
    mesh.set_num_points(3);
    draco::IndexTypeVector<draco::FaceIndex, draco::CornerTable::FaceType> f;
    f.push_back({draco::VertexIndex(0), draco::VertexIndex(1), draco::VertexIndex(2)});
    draco::CornerTable table;
    table.Init(f);
    draco::PointAttribute positions;
    positions.Init(draco::GeometryAttribute::POSITION, 3, draco::DT_INT32, false, 3);
    positions.SetIdentityMapping();
    const int32_t kPositions[3][3] = {{0, 0, 0}, {1, 1, 0}, {0, 0, 2}};
    for (int v = 0; v < 3; ++v)
    {
        positions.SetAttributeValue(draco::AttributeValueIndex(v), kPositions[v]);
    }
    std::vector<draco::CornerIndex> valueCorner = {draco::CornerIndex(0), draco::CornerIndex(1),
                                                   draco::CornerIndex(2)};
    std::vector<int32_t> vertexValue = {0, 1, 2};
    std::vector<draco::PointIndex> points = {draco::PointIndex(0), draco::PointIndex(1),
                                             draco::PointIndex(2)};
    typedef draco::MeshPredictionSchemeData<draco::CornerTable> Data;
    Data md;
    md.Set(&mesh, &table, &valueCorner, &vertexValue);
    draco::MeshPredictionSchemeTexCoordsPortablePredictor<int32_t, Data> tex(md);
    tex.SetPositionAttribute(positions);
    tex.SetEntryToPointIdMap(points.data());
    tex.ResizeOrientations(1);
    tex.set_orientation(0, true);
    std::vector<int32_t> uv = {0, 0, 5, 3, 0, 0};
    tex.ComputePredictedValue<false>(valueCorner[2], uv.data(), 2);
    std::printf("tex sqrt prediction %d %d\n", tex.predicted_value()[0], tex.predicted_value()[1]);
}

const int kFan[12][3] = {
    {12, 0, 1}, {12, 1, 2}, {12, 2, 3}, {12, 3, 4}, {12, 4, 5}, {12, 5, 0},
    {1, 0, 6},  {2, 1, 7},  {3, 2, 8},  {4, 3, 9},  {5, 4, 10}, {0, 5, 11},
};

} // namespace

int main()
{
    draco::Mesh mesh;
    draco::IndexTypeVector<draco::FaceIndex, draco::CornerTable::FaceType> faces;
    for (int r = 0; r + 1 < kRows; ++r)
    {
        for (int c = 0; c + 1 < kColumns; ++c)
        {
            int a = r * kColumns + c;
            int b = a + 1;
            int d = a + kColumns;
            int e = d + 1;
            faces.push_back({draco::VertexIndex(a), draco::VertexIndex(b), draco::VertexIndex(e)});
            faces.push_back({draco::VertexIndex(a), draco::VertexIndex(e), draco::VertexIndex(d)});
            mesh.AddFace({draco::PointIndex(a), draco::PointIndex(b), draco::PointIndex(e)});
            mesh.AddFace({draco::PointIndex(a), draco::PointIndex(e), draco::PointIndex(d)});
        }
    }
    mesh.set_num_points(kVertices);
    draco::CornerTable table;
    table.Init(faces);
    int grid[12][3];
    for (int i = 0; i < 12; ++i)
    {
        for (int k = 0; k < 3; ++k)
        {
            grid[i][k] = static_cast<int>(faces[draco::FaceIndex(i)][k].value());
        }
    }
    PrintOrders("kGridDepth", "kGridDegree", grid, 12);
    PrintOrders("kFanDepth", "kFanDegree", kFan, 12);
    // A 6 x 5 grid whose cells alternate their diagonal.
    int mixed[40][3];
    int m = 0;
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 5; ++c)
        {
            int a = r * 6 + c;
            bool flip = (r + c) % 2 == 1;
            int t0[3] = {a, a + 1, flip ? a + 6 : a + 7};
            int t1[3] = {flip ? a + 1 : a, a + 7, a + 6};
            for (int k = 0; k < 3; ++k)
            {
                mixed[m][k] = t0[k];
                mixed[m + 1][k] = t1[k];
            }
            m += 2;
        }
    }
    PrintOrders("kMixedDepth", "kMixedDegree", mixed, 40);
    TexSqrt();

    // Portable positions, a bump at vertex 5.
    draco::PointAttribute positions;
    positions.Init(draco::GeometryAttribute::POSITION, 3, draco::DT_INT32, false, kVertices);
    positions.SetIdentityMapping();
    for (int v = 0; v < kVertices; ++v)
    {
        int32_t p[3] = {(v % kColumns) * 100 + v * 7, (v / kColumns) * 90 - v * 3,
                        v == 5 ? 60 : (v * 13) % 17};
        positions.SetAttributeValue(draco::AttributeValueIndex(v), p);
    }

    std::vector<draco::CornerIndex> valueCorner(kVertices);
    std::vector<int32_t> vertexValue(kVertices);
    std::vector<draco::PointIndex> points(kVertices);
    for (int v = 0; v < kVertices; ++v)
    {
        for (int c = 0; c < static_cast<int>(table.num_corners()); ++c)
        {
            if (table.Vertex(draco::CornerIndex(c)).value() == static_cast<uint32_t>(v))
            {
                valueCorner[v] = draco::CornerIndex(c);
                break;
            }
        }
        vertexValue[v] = v;
        points[v] = draco::PointIndex(v);
    }
    typedef draco::MeshPredictionSchemeData<draco::CornerTable> Data;
    Data md;
    md.Set(&mesh, &table, &valueCorner, &vertexValue);

    // Texture coordinates: corrections, orientations read from the last.
    draco::MeshPredictionSchemeTexCoordsPortablePredictor<int32_t, Data> tex(md);
    tex.SetPositionAttribute(positions);
    tex.SetEntryToPointIdMap(points.data());
    const bool kOrientations[kVertices] = {true, false, true, true, false, false,
                                           true, false, false, true, true, false};
    tex.ResizeOrientations(kVertices);
    for (int i = 0; i < kVertices; ++i)
    {
        tex.set_orientation(i, kOrientations[i]);
    }
    std::vector<int32_t> uv(2 * kVertices);
    for (int i = 0; i < kVertices; ++i)
    {
        int32_t corr[2] = {(i * 37) % 23 - 11, (i * 53) % 29 - 14};
        if (!tex.ComputePredictedValue<false>(valueCorner[i], uv.data(), i))
        {
            std::printf("tex prediction %d failed\n", i);
            return 1;
        }
        for (int k = 0; k < 2; ++k)
        {
            uv[2 * i + k] = Wrap(static_cast<int64_t>(tex.predicted_value()[k]) + corr[k]);
        }
    }
    std::printf("tex left %zu\n", tex.num_orientations());
    std::printf("static const int32_t kTexDecoded[%d] = {", 2 * kVertices);
    for (int i = 0; i < 2 * kVertices; ++i)
    {
        std::printf("%s%d", i == 0 ? "" : ", ", uv[i]);
    }
    std::printf("};\n");

    // Normals: area-weighted, canonicalized for 8 bits, flipped by a bit;
    // the positions as above, scaled by 3001 (sums past 2^29), and all
    // zero (no normal).
    const bool kFlips[kVertices] = {false, true, false, false, true, false,
                                    false, false, true, false, true, true};
    draco::RAnsBitEncoder encoder;
    encoder.StartEncoding();
    for (int i = 0; i < kVertices; ++i)
    {
        encoder.EncodeBit(kFlips[i]);
    }
    draco::EncoderBuffer flips;
    encoder.EndEncoding(&flips);
    std::printf("static const uint8_t kFlips[%zu] = {", flips.size());
    for (size_t i = 0; i < flips.size(); ++i)
    {
        std::printf("%s%u", i == 0 ? "" : ", ", static_cast<uint8_t>(flips.data()[i]));
    }
    std::printf("};\n");
    const int32_t kScales[3] = {1, 3001, 0};
    for (int scale = 0; scale < 3; ++scale)
    {
        draco::PointAttribute scaled;
        scaled.Init(draco::GeometryAttribute::POSITION, 3, draco::DT_INT32, false, kVertices);
        scaled.SetIdentityMapping();
        for (int v = 0; v < kVertices; ++v)
        {
            int32_t p[3];
            positions.GetValue(draco::AttributeValueIndex(v), p);
            for (int k = 0; k < 3; ++k)
            {
                p[k] *= kScales[scale];
            }
            scaled.SetAttributeValue(draco::AttributeValueIndex(v), p);
        }
        draco::MeshPredictionSchemeGeometricNormalPredictorArea<int32_t, int, Data> normal(md);
        normal.SetPositionAttribute(scaled);
        normal.SetEntryToPointIdMap(points.data());
        draco::OctahedronToolBox box;
        box.SetQuantizationBits(8);
        std::vector<int32_t> oct(2 * kVertices);
        for (int i = 0; i < kVertices; ++i)
        {
            draco::VectorD<int32_t, 3> n;
            normal.ComputePredictedValue(valueCorner[i], n.data());
            box.CanonicalizeIntegerVector(n.data());
            if (kFlips[i])
            {
                n = -n;
            }
            int32_t s = 0;
            int32_t t = 0;
            box.IntegerVectorToQuantizedOctahedralCoords(n.data(), &s, &t);
            int32_t corr[2] = {(i * 11) % 7 - 3, (i * 5) % 9 - 4};
            oct[2 * i] = Wrap(static_cast<int64_t>(s) + corr[0]);
            oct[2 * i + 1] = Wrap(static_cast<int64_t>(t) + corr[1]);
        }
        std::printf("    {");
        for (int i = 0; i < 2 * kVertices; ++i)
        {
            std::printf("%s%d", i == 0 ? "" : ", ", oct[i]);
        }
        std::printf("},\n");
    }
    return 0;
}
