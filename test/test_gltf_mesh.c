// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF meshes: a primitive's two influence sets (normalized
// bytes and shorts, mixed) read into influences, every rule glTF makes a
// MUST refused; its morph targets read over positions, normals, a
// texture coordinate and a color set, normalized shorts decoded, the
// basis change applied, untouched vertices left out, channels named by
// targetNames or index; node and mesh weights; Khronos sample models
// (test/data/gltf) imported whole.

// MSVC's CRT deprecates fopen, which the fixtures use: CMake defines
// _CRT_SECURE_NO_WARNINGS for the fixture tests.

#include "bytes.h"
#include "cleared.h"
#include "morph_file.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/morph.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MANI_TEST_DATA
#error "MANI_TEST_DATA names test/data"
#endif

static char s_json[65536];
static uint8_t s_buffer[512];
static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_skinFile[16384];
static _Alignas(16) uint8_t s_file[65536];
static size_t s_size;

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void Base64(const uint8_t* bytes, size_t size, char* out)
{
    static const char kDigits[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < size; i += 3)
    {
        uint32_t group = (uint32_t)bytes[i] << 16;
        group |= i + 1 < size ? (uint32_t)bytes[i + 1] << 8 : 0;
        group |= i + 2 < size ? bytes[i + 2] : 0;
        out[o++] = kDigits[group >> 18];
        out[o++] = kDigits[(group >> 12) & 63];
        out[o++] = i + 1 < size ? kDigits[(group >> 6) & 63] : '=';
        out[o++] = i + 2 < size ? kDigits[group & 63] : '=';
    }
    out[o] = '\0';
}

// The buffer, four vertices:
//   0 POSITION, VEC3 float          48 bytes at 0 (also NORMAL, COLOR_0)
//   1 JOINTS_0, VEC4 u8             16 at 48
//   2 WEIGHTS_0, VEC4 unorm8        16 at 64
//   3 JOINTS_1, VEC4 u16            32 at 80
//   4 WEIGHTS_1, VEC4 unorm16       32 at 112
//   5 target 0 POSITION, float      48 at 144
//   6 target 0 NORMAL, float        48 at 192
//   7 target 1 TEXCOORD_0, snorm16  16 at 240
//   8 target 1 COLOR_0, VEC3 float  48 at 256
//   9 TEXCOORD_0, VEC2 float        32 at 304
//  10 JOINTS_0's bytes as VEC4 unorm8
//  11 VEC4 float over 148 to 212: vertex 0 moves only its fourth
//     component, vertex 2 its third
static const uint8_t kJoints0[16] = {0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0};
static const uint8_t kWeights0[16] = {128, 127, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 51, 0, 0, 0};
static const uint16_t kJoints1[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
static const uint16_t kWeights1[16] = {0, 0, 0, 0, 0, 0, 0, 0, 65535, 0, 0, 0, 52428, 0, 0, 0};
static const float kPositionDeltas[12] = {1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, -1};
static const float kNormalDeltas[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5f, 0, 0};
static const int16_t kTexcoordDeltas[8] = {16384, -32767, 0, 0, 0, 0, 0, 0};
static const float kColorDeltas[12] = {0, 0, 0, 0, 0, 0, 0.25f, 0.5f, 0.75f, 0, 0, 0};

static void FillBuffer(void)
{
    memset(s_buffer, 0, sizeof s_buffer);
    for (int v = 0; v < 4; ++v)
    {
        float p[3] = {(float)v, 1.0f, 0.5f};
        memcpy(s_buffer + 12 * v, p, sizeof p);
        float uv[2] = {0.25f * (float)v, 0.5f};
        memcpy(s_buffer + 304 + 8 * v, uv, sizeof uv);
    }
    memcpy(s_buffer + 48, kJoints0, 16);
    memcpy(s_buffer + 64, kWeights0, 16);
    for (int i = 0; i < 16; ++i)
    {
        maniStoreU16(s_buffer + 80 + 2 * i, kJoints1[i]);
        maniStoreU16(s_buffer + 112 + 2 * i, kWeights1[i]);
    }
    memcpy(s_buffer + 144, kPositionDeltas, 48);
    memcpy(s_buffer + 192, kNormalDeltas, 48);
    for (int i = 0; i < 8; ++i)
    {
        maniStoreU16(s_buffer + 240 + 2 * i, (uint16_t)kTexcoordDeltas[i]);
    }
    memcpy(s_buffer + 256, kColorDeltas, 48);
}

static const char* const kTargets =
    "{\"POSITION\":5,\"NORMAL\":6,\"_FOO\":0},{\"TEXCOORD_0\":7,\"COLOR_0\":8}";
static const char* s_targets = kTargets;

// The document: nodes 0 (joint, root) and 1 (joint, child), node 2 the
// skinned mesh with its weights; mesh 0 with targetNames (given by
// `names`), its primitive's attributes and targets (`primitive`), and
// mesh weights; `extra` lands inside the primitive.
static void Document(const char* names, const char* extra, const char* attributes)
{
    char digits[700];
    Base64(s_buffer, 336, digits);
    snprintf(
        s_json, sizeof s_json,
        "{\"asset\":{\"version\":\"2.0\"},"
        "\"buffers\":[{\"byteLength\":336,\"uri\":\"data:application/octet-stream;base64,%s\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteLength\":336}],"
        "\"accessors\":["
        "{\"bufferView\":0,\"byteOffset\":0,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},"
        "{\"bufferView\":0,\"byteOffset\":48,\"componentType\":5121,\"count\":4,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":64,\"componentType\":5121,\"normalized\":true,"
        "\"count\":4,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":80,\"componentType\":5123,\"count\":4,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":112,\"componentType\":5123,\"normalized\":true,"
        "\"count\":4,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":144,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":192,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":240,\"componentType\":5122,\"normalized\":true,"
        "\"count\":4,\"type\":\"VEC2\"},"
        "{\"bufferView\":0,\"byteOffset\":256,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":304,\"componentType\":5126,\"count\":4,\"type\":\"VEC2\"}"
        ","
        "{\"bufferView\":0,\"byteOffset\":48,\"componentType\":5121,\"normalized\":true,"
        "\"count\":4,\"type\":\"VEC4\"},"
        "{\"bufferView\":0,\"byteOffset\":148,\"componentType\":5126,\"count\":4,"
        "\"type\":\"VEC4\"}],"
        "\"nodes\":[{\"name\":\"root\",\"children\":[1]},"
        "{\"name\":\"tip\",\"translation\":[0,1,0]},"
        "{\"name\":\"body\",\"mesh\":0,\"skin\":0,\"weights\":[0.5,0.25]},"
        "{\"name\":\"plain\",\"mesh\":0}],"
        "\"skins\":[{\"joints\":[0,1]}],"
        "\"meshes\":[{%s\"weights\":[1,0],\"primitives\":[{%s\"attributes\":{%s},"
        "\"targets\":[%s]}]}]}",
        digits, names, extra, attributes, s_targets);
}

static const char* kAttributes =
    "\"POSITION\":0,\"NORMAL\":0,\"COLOR_0\":0,\"TEXCOORD_0\":9,\"_FOO\":0,"
    "\"JOINTS_0\":1,\"WEIGHTS_0\":2,\"JOINTS_1\":3,\"WEIGHTS_1\":4";

static const char* kNames = "\"extras\":{\"targetNames\":[\"smile\",\"blink\"]},";

static maniGltf* Open(maniBuilder* builder)
{
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, s_json, strlen(s_json), &gltf);
    CHECK(r.result == mani_success, "opens");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu item %u near: %.40s\n", r.reason,
               (unsigned long long)r.offset, r.item, s_json + r.offset);
    }
    return gltf;
}

// The skeleton and skin of the document, opened into skin.
static void BuildSkin(maniBuilder* builder, const maniGltf* gltf, maniSkin* skin)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    CHECK(maniBuildGltfSkeleton(builder, gltf, 0, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success,
          "skeleton");
    CHECK(maniBuildGltfSkin(builder, gltf, 0, &skeleton, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) ==
                  mani_success &&
              maniOpenSkin(s_skinFile, size, skin).result == mani_success,
          "skin");
}

static void TestQueries(void)
{
    FillBuffer();
    Document(kNames, "", kAttributes);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    CHECK(maniGltf_GetMeshCount(gltf) == 1 && maniGltf_GetPrimitiveCount(gltf, 0) == 1 &&
              maniGltf_GetPrimitiveCount(gltf, 1) == 0 &&
              maniGltf_GetPrimitiveTargetCount(gltf, 0, 0) == 2 &&
              maniGltf_GetPrimitiveTargetCount(gltf, 0, 1) == 0,
          "meshes, primitives, targets");
    CHECK(maniGltf_GetNodeMesh(gltf, 2) == 0 && maniGltf_GetNodeSkin(gltf, 2) == 0 &&
              maniGltf_GetNodeMesh(gltf, 0) == UINT32_MAX &&
              maniGltf_GetNodeSkin(gltf, 3) == UINT32_MAX &&
              maniGltf_GetNodeMesh(gltf, 9) == UINT32_MAX,
          "a node's mesh and skin");
    float w[2] = {9, 9};
    CHECK(maniGltf_GetMorphWeights(gltf, 2, w, 2, NULL).result == mani_success && w[0] == 0.5f &&
              w[1] == 0.25f,
          "the node's weights");
    CHECK(maniGltf_GetMorphWeights(gltf, 3, w, 2, NULL).result == mani_success && w[0] == 1.0f &&
              w[1] == 0.0f,
          "the mesh's weights");
    float wide[3] = {9, 9, 9};
    uint32_t count = 0;
    CHECK(maniGltf_GetMorphWeights(gltf, 3, wide, 3, &count).result == mani_success && count == 2 &&
              wide[0] == 1.0f && wide[1] == 0.0f && wide[2] == 9.0f,
          "a wider array takes the mesh's count");
    w[1] = 9.0f;
    uint32_t measured = 0;
    uint32_t refused = 0;
    CHECK(maniGltf_GetMorphWeights(gltf, 3, NULL, 0, &measured).result == mani_errorCapacity &&
              measured == 2 &&
              maniGltf_GetMorphWeights(gltf, 2, w, 1, &refused).result == mani_errorCapacity &&
              refused == 2 && w[0] == 0.0f && w[1] == 9.0f,
          "the weights measured, and a short array refused and zeroed to its capacity");
    CHECK(maniGltf_GetMorphWeights(gltf, 0, w, 2, NULL).result == mani_errorInvalid &&
              maniGltf_GetMorphWeights(gltf, 2, NULL, 2, NULL).result == mani_errorInvalid &&
              maniGltf_GetMorphWeights(NULL, 2, w, 2, NULL).result == mani_errorInvalid,
          "weights refused: no mesh, no output, no document");
    uint32_t noCount = 7;
    Fill(w, sizeof w);
    bool noMesh = maniGltf_GetMorphWeights(gltf, 0, w, 2, &noCount).result == mani_errorInvalid &&
                  noCount == 0 && Cleared(w, sizeof w);
    noCount = 7;
    Fill(w, sizeof w);
    CHECK(noMesh && maniGltf_GetMorphWeights(NULL, 2, w, 2, &noCount).result == mani_errorInvalid &&
              noCount == 0 && Cleared(w, sizeof w),
          "refused weights clear the count and the array");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestInfluences(void)
{
    FillBuffer();
    Document(kNames, "", kAttributes);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniSkin skin;
    BuildSkin(builder, gltf, &skin);
    maniImportResult r = maniBuildGltfInfluences(builder, gltf, 0, 0, &skin);
    CHECK(r.result == mani_success, "influences import");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu item %u\n", r.reason, (unsigned long long)r.offset, r.item);
    }
    maniInfluences v;
    CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
              maniOpenInfluences(s_file, s_size, &v).result == mani_success,
          "opens");
    CHECK(maniInfluences_GetVertexCount(&v) == 4 && maniInfluences_IsSkinFit(&v, &skin), "bound");
    float w0 = 0.0f;
    float w1 = 0.0f;
    uint32_t j0 = maniInfluences_GetInfluence(&v, 0, 0, &w0);
    uint32_t j1 = maniInfluences_GetInfluence(&v, 0, 1, &w1);
    CHECK(j0 == 0 && j1 == 1 && fabsf(w0 - 128.0f / 255.0f) < 1e-6f &&
              fabsf(w1 - 127.0f / 255.0f) < 1e-6f,
          "vertex 0: both joints from set 0");
    j0 = maniInfluences_GetInfluence(&v, 2, 0, &w0);
    (void)maniInfluences_GetInfluence(&v, 2, 1, &w1);
    CHECK(j0 == 1 && w0 == 1.0f && w1 == 0.0f, "vertex 2: from set 1 alone");
    j0 = maniInfluences_GetInfluence(&v, 3, 0, &w0);
    j1 = maniInfluences_GetInfluence(&v, 3, 1, &w1);
    CHECK(j0 == 1 && j1 == 0 && fabsf(w0 - 0.8f) < 1e-6f && fabsf(w1 - 0.2f) < 1e-6f,
          "vertex 3: bytes and shorts mixed, heaviest first");
    CHECK(maniBuildGltfInfluences(builder, gltf, 0, 1, &skin).result == mani_errorInvalid &&
              maniBuildGltfInfluences(builder, gltf, 1, 0, &skin).result == mani_errorInvalid &&
              maniBuildGltfInfluences(builder, gltf, 0, 0, NULL).result == mani_errorInvalid,
          "past a count, no skin");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// A refusal of the influences import, the reason and item.
static void ExpectInfluences(maniImportReason reason, uint32_t item, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = Open(builder);
    maniSkin skin;
    BuildSkin(builder, gltf, &skin);
    maniImportResult r = maniBuildGltfInfluences(builder, gltf, 0, 0, &skin);
    CHECK(r.result == mani_errorFormat && r.reason == reason && r.item == item, what);
    if (r.reason != reason || r.item != item)
    {
        printf("  %s: reason %u item %u\n", what, r.reason, r.item);
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestInfluenceRefusals(void)
{
    FillBuffer();
    s_buffer[64 + 1] = 126;
    Document(kNames, "", kAttributes);
    ExpectInfluences(maniImportReasonInfluence, 0, "bytes summing to 254");
    FillBuffer();
    s_buffer[48 + 1] = 0;
    Document(kNames, "", kAttributes);
    ExpectInfluences(maniImportReasonInfluence, 0, "a joint with two weights");
    FillBuffer();
    s_buffer[48 + 4] = 2;
    Document(kNames, "", kAttributes);
    ExpectInfluences(maniImportReasonInfluence, 1, "a joint past the skin");
    FillBuffer();
    s_buffer[48 + 14] = 2;
    Document(kNames, "", kAttributes);
    ExpectInfluences(maniImportReasonInfluence, 3, "a joint past the skin at weight 0");
    FillBuffer();
    Document(kNames, "",
             "\"POSITION\":0,\"NORMAL\":0,\"COLOR_0\":0,\"TEXCOORD_0\":9,"
             "\"JOINTS_0\":1,\"WEIGHTS_0\":2,\"JOINTS_1\":3");
    ExpectInfluences(maniImportReasonProperty, 1, "joints without weights");
    Document(kNames, "", "\"POSITION\":0,\"JOINTS_1\":3,\"WEIGHTS_1\":4");
    ExpectInfluences(maniImportReasonProperty, 0, "no JOINTS_0");
    Document(kNames, "",
             "\"POSITION\":0,\"NORMAL\":0,\"COLOR_0\":0,\"TEXCOORD_0\":9,"
             "\"JOINTS_0\":10,\"WEIGHTS_0\":2");
    ExpectInfluences(maniImportReasonProperty, 10, "normalized joints");
    Document(kNames, "",
             "\"POSITION\":0,\"NORMAL\":0,\"COLOR_0\":0,\"TEXCOORD_0\":9,"
             "\"JOINTS_0\":1,\"WEIGHTS_0\":1");
    ExpectInfluences(maniImportReasonProperty, 1, "weights not normalized");
    Document(kNames, "",
             "\"POSITION\":0,\"NORMAL\":0,\"COLOR_0\":0,\"TEXCOORD_0\":9,"
             "\"JOINTS_0\":0,\"WEIGHTS_0\":2");
    ExpectInfluences(maniImportReasonProperty, 0, "joints of VEC3");
    Document(kNames, "\"extensions\":{\"KHR_draco_mesh_compression\":{}},", kAttributes);
    ExpectInfluences(maniImportReasonProperty, 0, "a Draco extension without its view");
    Document(kNames,
             "\"extensions\":{\"KHR_draco_mesh_compression\":{\"bufferView\":0,"
             "\"attributes\":{}}},",
             kAttributes);
    ExpectInfluences(maniImportReasonData, 0, "a Draco view that is no stream");
}

// The morph targets imported from the document, opened into m.
static maniImportResult ImportMorph(maniBuilder* builder, const maniGltfOptions* options,
                                    maniMorphTargets* m)
{
    maniGltf* gltf = Open(builder);
    maniImportResult r = maniBuildGltfMorphTargets(builder, gltf, 0, 0, options);
    if (r.result == mani_success)
    {
        CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
                  maniOpenMorphTargets(s_file, s_size, m).result == mani_success,
              "opens");
    }
    maniCloseGltf(gltf);
    return r;
}

// A target's deltas for a table entry at its i-th kept vertex.
static const float* Delta(const maniMorphTargets* m, uint32_t t, uint32_t a, uint32_t i)
{
    const uint8_t* e = m->targets + (size_t)t * MANI_MORPH_TARGET_SIZE;
    uint32_t mask = maniLoadU32(e + 4);
    uint32_t count = maniLoadU32(e + 8);
    uint64_t at = maniLoadU64(e + 24);
    for (uint32_t b = 0; b < a; ++b)
    {
        at += ((mask >> b) & 1u) != 0
                  ? (uint64_t)count * maniMorphWidth(maniMorphTargets_GetAttribute(m, b, NULL))
                  : 0;
    }
    return (const float*)(const void*)(m->deltas +
                                       4 * (at + (uint64_t)i *
                                                     maniMorphWidth(maniMorphTargets_GetAttribute(
                                                         m, a, NULL))));
}

static uint32_t Index(const maniMorphTargets* m, uint32_t t, uint32_t i)
{
    const uint8_t* e = m->targets + (size_t)t * MANI_MORPH_TARGET_SIZE;
    return maniLoadU32(m->indices + 4 * (maniLoadU64(e + 16) + i));
}

// A basis that only flips z is no identity: position deltas' z negated.
static void TestMorphFlipZ(void)
{
    FillBuffer();
    Document(kNames, "", kAttributes);
    maniBuilder* builder = NewBuilder();
    maniGltfOptions flip = {{maniAxisX, maniAxisY, maniAxisNegativeZ}, 0};
    maniMorphTargets m;
    CHECK(ImportMorph(builder, &flip, &m).result == mani_success, "imports with z flipped");
    const float* d = Delta(&m, 0, 0, 2);
    CHECK(Index(&m, 0, 2) == 3 && d[2] == 1.0f, "vertex 3's z negated");
    maniDestroyBuilder(builder);
}

// A vertex whose only delta is a color's fourth component moves.
static void TestMorphAlpha(void)
{
    FillBuffer();
    s_targets = "{\"COLOR_0\":11}";
    Document("", "", kAttributes);
    s_targets = kTargets;
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniMorphTargets m;
    CHECK(ImportMorph(builder, &options, &m).result == mani_success, "a color target imports");
    const uint8_t* e = m.targets;
    CHECK(maniLoadU32(e + 8) == 2 && Index(&m, 0, 0) == 0 && Index(&m, 0, 1) == 2,
          "the vertex moving only alpha kept");
    const float* alpha = Delta(&m, 0, 0, 0);
    CHECK(alpha[0] == 0.0f && alpha[3] == 2.0f, "its delta");
    maniDestroyBuilder(builder);
}

static void TestMorphTargets(void)
{
    FillBuffer();
    Document(kNames, "", kAttributes);
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniMorphTargets m;
    maniImportResult r = ImportMorph(builder, &options, &m);
    CHECK(r.result == mani_success, "morph import");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu item %u\n", r.reason, (unsigned long long)r.offset, r.item);
        maniDestroyBuilder(builder);
        return;
    }
    uint32_t set = 9;
    CHECK(maniMorphTargets_GetVertexCount(&m) == 4 && maniMorphTargets_GetChannelCount(&m) == 2 &&
              maniMorphTargets_GetAttributeCount(&m) == 4 &&
              maniMorphTargets_GetAttribute(&m, 0, &set) == maniMorphPosition &&
              maniMorphTargets_GetAttribute(&m, 1, &set) == maniMorphNormal &&
              maniMorphTargets_GetAttribute(&m, 2, &set) == maniMorphTexcoord && set == 0 &&
              maniMorphTargets_GetAttribute(&m, 3, &set) == maniMorphColor && set == 0,
          "the table: positions, normals, texture coordinates, colors; _FOO left");
    CHECK(maniMorphTargets_FindChannel(&m, "smile", 5) == 0 &&
              maniMorphTargets_FindChannel(&m, "blink", 5) == 1 &&
              maniMorphTargets_GetTargetFullWeight(&m, 1) == 1.0f,
          "channels named by targetNames, full weight 1");
    CHECK(maniMorphTargets_FindTarget(&m, "blink", 5) == 1, "targets named as their channels");
    const uint8_t* t0 = m.targets;
    const uint8_t* t1 = m.targets + MANI_MORPH_TARGET_SIZE;
    CHECK(maniLoadU32(t0 + 8) == 3 && Index(&m, 0, 0) == 0 && Index(&m, 0, 1) == 1 &&
              Index(&m, 0, 2) == 3 && maniLoadU32(t0 + 4) == 3u,
          "target 0: the vertices it moves");
    CHECK(Delta(&m, 0, 0, 1)[1] == 2.0f && Delta(&m, 0, 0, 2)[2] == -1.0f &&
              Delta(&m, 0, 1, 2)[0] == 0.5f,
          "target 0's deltas");
    CHECK(maniLoadU32(t1 + 8) == 2 && Index(&m, 1, 0) == 0 && Index(&m, 1, 1) == 2 &&
              maniLoadU32(t1 + 4) == 12u,
          "target 1: the vertices it moves");
    const float* uv = Delta(&m, 1, 2, 0);
    const float* color = Delta(&m, 1, 3, 1);
    CHECK(uv[0] == 16384.0f / 32767.0f && uv[1] == -1.0f, "normalized shorts decoded");
    CHECK(color[0] == 0.25f && color[2] == 0.75f && color[3] == 0.0f, "VEC3 colors, alpha 0");
    // Z up: glTF's (x, y, z) becomes (x, -z, y).
    options.axes[1] = maniAxisNegativeZ;
    options.axes[2] = maniAxisY;
    CHECK(ImportMorph(builder, &options, &m).result == mani_success, "morph import, Z up");
    const float* p = Delta(&m, 0, 0, 2);
    const float* n = Delta(&m, 0, 1, 2);
    CHECK(p[0] == 0.0f && p[1] == 1.0f && p[2] == 0.0f && n[0] == 0.5f,
          "positions and normals in the output's basis");
    CHECK(Delta(&m, 1, 2, 0)[1] == -1.0f, "texture coordinates untouched");
    // No targetNames: the indices.
    Document("", "", kAttributes);
    options = maniDefaultGltfOptions();
    CHECK(ImportMorph(builder, &options, &m).result == mani_success &&
              maniMorphTargets_FindChannel(&m, "0", 1) == 0 &&
              maniMorphTargets_FindChannel(&m, "1", 1) == 1,
          "channels named by index");
    maniDestroyBuilder(builder);
}

static void ExpectMorph(maniImportReason reason, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniMorphTargets m;
    maniImportResult r = ImportMorph(builder, &options, &m);
    CHECK(r.result == mani_errorFormat && r.reason == reason, what);
    if (r.reason != reason)
    {
        printf("  %s: reason %u item %u\n", what, r.reason, r.item);
    }
    maniDestroyBuilder(builder);
}

// A Draco primitive's targets are never compressed: its morph file is
// the plain primitive's, byte for byte.
static void TestDracoMorph(void)
{
    static uint8_t plain[sizeof s_file];
    FillBuffer();
    s_targets = kTargets;
    maniBuilder* builder = NewBuilder();
    maniGltfOptions options = maniDefaultGltfOptions();
    maniMorphTargets m;
    Document(kNames, "", kAttributes);
    CHECK(ImportMorph(builder, &options, &m).result == mani_success, "the plain primitive");
    size_t size = s_size;
    memcpy(plain, s_file, size);
    Document(kNames, "\"extensions\":{\"KHR_draco_mesh_compression\":{}},", kAttributes);
    CHECK(ImportMorph(builder, &options, &m).result == mani_success && s_size == size &&
              memcmp(plain, s_file, size) == 0,
          "a Draco primitive's targets, the same file");
    maniDestroyBuilder(builder);
}

static void TestMorphRefusals(void)
{
    FillBuffer();
    Document(kNames, "", "\"POSITION\":0,\"COLOR_0\":0,\"TEXCOORD_0\":9");
    ExpectMorph(maniImportReasonProperty, "a semantic the primitive lacks");
    s_targets = "{\"POSITION\":5},{\"COLOR_0\":9}";
    Document(kNames, "", kAttributes);
    ExpectMorph(maniImportReasonProperty, "a color delta of VEC2");
    s_targets = "{\"POSITION\":1}";
    Document(kNames, "", kAttributes);
    ExpectMorph(maniImportReasonProperty, "a position delta of VEC4");
    s_targets = kTargets;
    FillBuffer();
    float nan = NAN;
    memcpy(s_buffer + 144 + 4, &nan, 4);
    Document(kNames, "", kAttributes);
    ExpectMorph(maniImportReasonData, "a delta not finite");
    FillBuffer();
    Document("\"extras\":{\"targetNames\":[\"same\",\"same\"]},", "", kAttributes);
    ExpectMorph(maniImportReasonBuild, "targetNames repeated");
    maniBuilder* builder = NewBuilder();
    Document(kNames, "", kAttributes);
    maniGltf* gltf = Open(builder);
    maniGltfOptions options = maniDefaultGltfOptions();
    maniGltfOptions bad = options;
    bad.axes[0] = maniAxisY;
    CHECK(maniBuildGltfMorphTargets(builder, gltf, 0, 1, &options).result == mani_errorInvalid &&
              maniBuildGltfMorphTargets(builder, gltf, 0, 0, NULL).result == mani_errorInvalid &&
              maniBuildGltfMorphTargets(builder, gltf, 0, 0, &bad).result == mani_errorInvalid,
          "past a count, no options, options out of range");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// A file of test/data/gltf, or NULL; the caller frees it.
static uint8_t* ReadFile(const char* name, size_t* sizeOut)
{
    char path[512];
    snprintf(path, sizeof path, "%s/gltf/%s", MANI_TEST_DATA, name);
    FILE* f = fopen(path, "rb");
    if (f == NULL)
    {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* bytes = malloc((size_t)size + 1);
    size_t read = bytes != NULL ? fread(bytes, 1, (size_t)size, f) : 0;
    fclose(f);
    *sizeOut = read;
    return bytes;
}

// SimpleSkin's influences against its accessors read straight, and
// AnimatedMorphCube's targets: every primitive imports.
static void TestFixtures(void)
{
    size_t size = 0;
    uint8_t* bytes = ReadFile("SimpleSkinEmbedded.gltf", &size);
    CHECK(bytes != NULL, "SimpleSkin read");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(bytes != NULL && maniOpenGltf(builder, bytes, size, &gltf).result == mani_success,
          "SimpleSkin opens");
    if (gltf != NULL)
    {
        maniSkin skin;
        BuildSkin(builder, gltf, &skin);
        maniInfluences v;
        CHECK(maniBuildGltfInfluences(builder, gltf, 0, 0, &skin).result == mani_success &&
                  maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
                  maniOpenInfluences(s_file, s_size, &v).result == mani_success,
              "SimpleSkin's influences");
        // Its 10 vertices: the bottom row on joint 0, the top on joint 1,
        // the middle shared.
        float w = 0.0f;
        CHECK(maniInfluences_GetVertexCount(&v) == 10 &&
                  maniInfluences_GetInfluence(&v, 0, 0, &w) == 0 && w == 1.0f &&
                  maniInfluences_GetInfluence(&v, 9, 0, &w) == 1 && w == 1.0f,
              "SimpleSkin's ends on one joint each");
        maniCloseGltf(gltf);
    }
    free(bytes);
    bytes = ReadFile("AnimatedMorphCube.glb", &size);
    gltf = NULL;
    CHECK(bytes != NULL && maniOpenGltf(builder, bytes, size, &gltf).result == mani_success,
          "AnimatedMorphCube opens");
    if (gltf != NULL)
    {
        maniGltfOptions options = maniDefaultGltfOptions();
        maniMorphTargets m;
        CHECK(maniBuildGltfMorphTargets(builder, gltf, 0, 0, &options).result == mani_success &&
                  maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
                  maniOpenMorphTargets(s_file, s_size, &m).result == mani_success,
              "AnimatedMorphCube's targets");
        CHECK(maniMorphTargets_GetChannelCount(&m) == 2 &&
                  maniMorphTargets_GetAttributeCount(&m) == 3,
              "two targets of positions, normals and tangents");
        maniCloseGltf(gltf);
    }
    free(bytes);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestQueries();
    TestInfluences();
    TestInfluenceRefusals();
    TestMorphTargets();
    TestMorphAlpha();
    TestMorphFlipZ();
    TestMorphRefusals();
    TestDracoMorph();
    TestFixtures();
    return s_failures == 0 ? 0 : 1;
}
