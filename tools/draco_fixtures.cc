// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco streams from Google's encoder (d429de8, Apache-2.0)
// for the library's decoder tests and differential. Builds meshes a
// skin's primitive holds (positions, normals, texture coordinates with
// a seam, joints as four unsigned shorts and weights as four floats,
// both generic), plus one with holes, a second component and a
// non-manifold edge, in two sizes (and with positions alone), a closed
// sphere, and encodes each with every method, speed and
// quantization below. Writes <out>/<name>.drc and prints a line per
// stream: name, size, and the FNV-1a 64 of Google's decoding of it in
// the dump form the library's tests compute (faces as point ids, then
// each attribute in order: type, data type, components, normalized,
// unique id and every point's mapped value bytes).
//
// usage: draco_fixtures <out dir>
// Build: g++-14 -O2 -std=c++17 -I<draco>/src -I<draco build>
//        draco_fixtures.cc <draco build>/libdraco.a
// Test data only; never linked into the library. Writes test/data/draco/.

#include "draco/compression/decode.h"
#include "draco/compression/encode.h"
#include "draco/mesh/triangle_soup_mesh_builder.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct Vertex
{
    float position[3];
    float normal[3];
    float uv[2];
    uint16_t joints[4];
    float weights[4];
};

// A grid of rows x columns quads wrapped around a cylinder, its texture
// coordinates cut along the wrap (a seam), with skin influences; `holes`
// leaves every seventh quad out and adds a fan sharing one edge three
// ways (non-manifold) and a separate triangle.
std::vector<Vertex> Surface(int rows, int columns, bool holes)
{
    std::vector<Vertex> corners;
    auto at = [&](int r, int c, float u) {
        Vertex v{};
        float a = 6.2831853f * (float)c / (float)columns;
        v.position[0] = std::cos(a) * (1.0f + 0.1f * (float)r);
        v.position[1] = (float)r * 0.25f;
        v.position[2] = std::sin(a) * (1.0f + 0.1f * (float)r);
        v.normal[0] = std::cos(a);
        v.normal[2] = std::sin(a);
        v.uv[0] = u;
        v.uv[1] = (float)r / (float)rows;
        int j = r * 3 / (rows + 1);
        v.joints[0] = (uint16_t)j;
        v.joints[1] = (uint16_t)(j + 1);
        float w = (float)(r % 4) / 4.0f;
        v.weights[0] = 1.0f - w;
        v.weights[1] = w;
        return v;
    };
    int quad = 0;
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < columns; ++c, ++quad)
        {
            if (holes && quad % 7 == 3)
            {
                continue;
            }
            float u0 = (float)c / (float)columns;
            float u1 = (float)(c + 1) / (float)columns;
            int c1 = (c + 1) % columns;
            Vertex a = at(r, c, u0), b = at(r, c1, u1), d = at(r + 1, c, u0), e = at(r + 1, c1, u1);
            corners.insert(corners.end(), {a, b, e, a, e, d});
        }
    }
    if (holes)
    {
        Vertex a = at(0, 0, 0.0f), b = at(0, 1, 1.0f / (float)columns);
        Vertex x = at(0, 0, 0.5f);
        x.position[1] = -1.0f;
        Vertex y = x;
        y.position[0] += 0.5f;
        corners.insert(corners.end(), {b, a, x, a, b, y});
        Vertex p = at(0, 0, 0.0f), q = p, s = p;
        p.position[0] = 5.0f;
        q.position[0] = 6.0f;
        s.position[0] = 5.0f;
        s.position[2] = 1.0f;
        corners.insert(corners.end(), {p, q, s});
    }
    return corners;
}

// A closed sphere of rings x segments, poles of a segment's valence, its
// texture coordinates cut along the wrap, influences as the surface's.
std::vector<Vertex> Sphere(int rings, int segments)
{
    std::vector<Vertex> corners;
    auto at = [&](int r, int c) {
        Vertex v{};
        float theta = 3.14159265f * (float)r / (float)rings;
        float phi = 6.2831853f * (float)(c % segments) / (float)segments;
        // The poles exactly, so each is one vertex (sin(pi) is not 0, and
        // 0 * cos(phi) is -0 for half the segments).
        bool pole = r == 0 || r == rings;
        v.position[0] = pole ? 0.0f : std::sin(theta) * std::cos(phi);
        v.position[1] = pole ? (r == 0 ? 1.0f : -1.0f) : std::cos(theta);
        v.position[2] = pole ? 0.0f : std::sin(theta) * std::sin(phi);
        for (int k = 0; k < 3; ++k)
        {
            v.normal[k] = v.position[k];
        }
        v.uv[0] = (float)c / (float)segments;
        v.uv[1] = (float)r / (float)rings;
        int j = r * 3 / (rings + 1);
        v.joints[0] = (uint16_t)j;
        v.joints[1] = (uint16_t)(j + 1);
        float w = (float)(r % 4) / 4.0f;
        v.weights[0] = 1.0f - w;
        v.weights[1] = w;
        return v;
    };
    for (int r = 0; r < rings; ++r)
    {
        for (int c = 0; c < segments; ++c)
        {
            Vertex a = at(r, c), b = at(r, c + 1), d = at(r + 1, c), e = at(r + 1, c + 1);
            if (r > 0)
            {
                corners.insert(corners.end(), {a, b, e});
            }
            if (r + 1 < rings)
            {
                corners.insert(corners.end(), {a, e, d});
            }
        }
    }
    return corners;
}

// Discs of `spokes` triangles around a centre each, side by side: centres
// of that valence for Edgebreaker's valence contexts.
std::vector<Vertex> Fans(int discs, int spokes)
{
    std::vector<Vertex> corners;
    for (int d = 0; d < discs; ++d)
    {
        auto at = [&](int k, bool centre) {
            Vertex v{};
            float a = 6.2831853f * (float)(k % spokes) / (float)spokes;
            v.position[0] = (float)(3 * d) + (centre ? 0.0f : std::cos(a));
            v.position[2] = centre ? 0.0f : std::sin(a);
            v.normal[1] = 1.0f;
            v.uv[0] = centre ? 0.5f : 0.5f + 0.5f * std::cos(a);
            v.uv[1] = centre ? 0.5f : 0.5f + 0.5f * std::sin(a);
            v.joints[0] = (uint16_t)(d % 3);
            v.weights[0] = 1.0f;
            return v;
        };
        for (int k = 0; k < spokes; ++k)
        {
            corners.insert(corners.end(), {at(0, true), at(k, false), at(k + 1, false)});
        }
    }
    return corners;
}

std::unique_ptr<draco::Mesh> Build(const std::vector<Vertex>& corners, bool uv, bool bare)
{
    draco::TriangleSoupMeshBuilder builder;
    int faces = (int)corners.size() / 3;
    builder.Start(faces);
    int position = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    int normal =
        bare ? -1 : builder.AddAttribute(draco::GeometryAttribute::NORMAL, 3, draco::DT_FLOAT32);
    int texcoord = uv && !bare ? builder.AddAttribute(draco::GeometryAttribute::TEX_COORD, 2,
                                                      draco::DT_FLOAT32)
                               : -1;
    int joints =
        bare ? -1 : builder.AddAttribute(draco::GeometryAttribute::GENERIC, 4, draco::DT_UINT16);
    int weights =
        bare ? -1 : builder.AddAttribute(draco::GeometryAttribute::GENERIC, 4, draco::DT_FLOAT32);
    for (int f = 0; f < faces; ++f)
    {
        const Vertex* v = &corners[3 * f];
        draco::FaceIndex face(f);
        builder.SetAttributeValuesForFace(position, face, v[0].position, v[1].position,
                                          v[2].position);
        if (bare)
        {
            continue;
        }
        builder.SetAttributeValuesForFace(normal, face, v[0].normal, v[1].normal, v[2].normal);
        if (uv)
        {
            builder.SetAttributeValuesForFace(texcoord, face, v[0].uv, v[1].uv, v[2].uv);
        }
        builder.SetAttributeValuesForFace(joints, face, v[0].joints, v[1].joints, v[2].joints);
        builder.SetAttributeValuesForFace(weights, face, v[0].weights, v[1].weights,
                                          v[2].weights);
    }
    return builder.Finalize();
}

uint64_t Fnv(uint64_t h, const void* data, size_t size)
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i)
    {
        h = (h ^ p[i]) * 0x100000001b3ull;
    }
    return h;
}

// The dump's hash of a decoded mesh (see the header).
uint64_t Dump(const draco::Mesh& mesh)
{
    uint64_t h = 0xcbf29ce484222325ull;
    uint32_t counts[2] = {mesh.num_faces(), mesh.num_points()};
    h = Fnv(h, counts, sizeof counts);
    for (draco::FaceIndex f(0); f < mesh.num_faces(); ++f)
    {
        uint32_t ids[3] = {mesh.face(f)[0].value(), mesh.face(f)[1].value(),
                           mesh.face(f)[2].value()};
        h = Fnv(h, ids, sizeof ids);
    }
    for (int a = 0; a < mesh.num_attributes(); ++a)
    {
        const draco::PointAttribute* att = mesh.attribute(a);
        uint32_t head[5] = {(uint32_t)att->attribute_type(), (uint32_t)att->data_type(),
                            (uint32_t)att->num_components(), att->normalized() ? 1u : 0u,
                            att->unique_id()};
        h = Fnv(h, head, sizeof head);
        for (draco::PointIndex p(0); p < mesh.num_points(); ++p)
        {
            h = Fnv(h, att->GetAddress(att->mapped_index(p)), (size_t)att->byte_stride());
        }
    }
    return h;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "usage: draco_fixtures <out dir>\n");
        return 2;
    }
    struct Shape
    {
        const char* name;
        int rows;
        int columns;
        bool holes;
        bool uv;
        bool sphere;
        bool bare;
        // Added to every joint index: `rig`'s cross 256 (254 to 257).
        int jointBase;
    };
    // Edgebreaker's valence traversal needs 1,000 faces or more at a
    // speed below 5: the large shapes, the sphere and the fans. The sphere
    // is closed, its poles of valence 32; `fans` are thirty discs of 36
    // spokes (rows discs, columns spokes); `bare` holds positions alone,
    // so a merged vertex is removed; `rig` is the tube with joint indices
    // past a byte.
    const Shape shapes[] = {{"tube", 6, 12, false, true, false, false, 0},
                            {"holes", 6, 12, true, true, false, false, 0},
                            {"small", 2, 5, false, false, false, false, 0},
                            {"large", 25, 24, false, true, false, false, 0},
                            {"largeholes", 25, 24, true, true, false, false, 0},
                            {"sphere", 24, 32, false, true, true, false, 0},
                            {"bare", 6, 12, true, false, false, true, 0},
                            {"largebare", 25, 24, true, false, false, true, 0},
                            {"fans", 30, 36, false, true, false, false, 0},
                            {"rig", 6, 12, false, true, false, false, 254}};
    const int speeds[] = {0, 1, 2, 5, 7, 8, 10};
    for (const Shape& s : shapes)
    {
        bool fans = std::string(s.name) == "fans";
        std::vector<Vertex> corners =
            fans ? Fans(s.rows, s.columns)
                 : (s.sphere ? Sphere(s.rows, s.columns) : Surface(s.rows, s.columns, s.holes));
        for (Vertex& v : corners)
        {
            for (uint16_t& j : v.joints)
            {
                j = (uint16_t)(j + s.jointBase);
            }
        }
        std::unique_ptr<draco::Mesh> mesh = Build(corners, s.uv, s.bare);
        for (int method = 0; method < 2; ++method)
        {
            for (int speed : speeds)
            {
                for (int quantize = 0; quantize < 2; ++quantize)
                {
                    draco::Encoder encoder;
                    encoder.SetEncodingMethod(method == 0 ? draco::MESH_SEQUENTIAL_ENCODING
                                                          : draco::MESH_EDGEBREAKER_ENCODING);
                    encoder.SetSpeedOptions(speed, speed);
                    if (quantize)
                    {
                        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
                        encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, 10);
                        encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, 12);
                        encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 12);
                    }
                    draco::EncoderBuffer buffer;
                    if (!encoder.EncodeMeshToBuffer(*mesh, &buffer).ok())
                    {
                        std::fprintf(stderr, "%s: encoding failed\n", s.name);
                        return 1;
                    }
                    char name[128];
                    std::snprintf(name, sizeof name, "%s-%s-s%d-%s", s.name,
                                  method == 0 ? "seq" : "eb", speed, quantize ? "q" : "raw");
                    std::string path = std::string(argv[1]) + "/" + name + ".drc";
                    FILE* f = std::fopen(path.c_str(), "wb");
                    if (f == nullptr || std::fwrite(buffer.data(), 1, buffer.size(), f) !=
                                            buffer.size())
                    {
                        std::fprintf(stderr, "%s: cannot write\n", path.c_str());
                        return 1;
                    }
                    std::fclose(f);
                    draco::DecoderBuffer in;
                    in.Init(buffer.data(), buffer.size());
                    draco::Decoder decoder;
                    auto decoded = decoder.DecodeMeshFromBuffer(&in);
                    if (!decoded.ok())
                    {
                        std::fprintf(stderr, "%s: decoding failed\n", name);
                        return 1;
                    }
                    std::printf("%s %zu 0x%016llxull method %u\n", name, buffer.size(),
                                (unsigned long long)Dump(*decoded.value()),
                                (unsigned)(uint8_t)buffer.data()[8]);
                }
            }
        }
    }
    return 0;
}
