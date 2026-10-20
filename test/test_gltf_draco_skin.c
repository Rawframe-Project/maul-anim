// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Draco-compressed skins (mani-0021): SimpleSkin compressed by glTF
// Transform, with float weights (which its encoder quantizes to 12 bits)
// and with normalized byte weights, gives the influences of the
// uncompressed SimpleSkin vertex for vertex, in the order Draco gives
// them, to half a quantization step; the extension's rules refused, each by a
// same-length change to the file's JSON; every byte of the stream
// changed decoded without fault.

// MSVC's CRT deprecates fopen, which the fixtures use: CMake defines
// _CRT_SECURE_NO_WARNINGS for the fixture tests.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
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

// SimpleSkin's vertices and the influences each holds at most.
#define VERTICES 10u
#define SLOTS    4u

static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_skinFile[16384];
static _Alignas(16) uint8_t s_file[65536];
static uint8_t s_bytes[8192];
static size_t s_size;

// A vertex's influences, heaviest first.
typedef struct Vertex
{
    uint32_t joints[SLOTS];
    float weights[SLOTS];
} Vertex;

static size_t ReadFile(const char* name)
{
    char path[512];
    snprintf(path, sizeof path, "%s/gltf/%s", MANI_TEST_DATA, name);
    FILE* f = fopen(path, "rb");
    size_t size = f != NULL ? fread(s_bytes, 1, sizeof s_bytes, f) : 0;
    if (f != NULL)
    {
        fclose(f);
    }
    CHECK(size > 0, name);
    return size;
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// The influences of mesh 0's primitive under skin 0, into vertices; the
// import's result.
static maniImportResult Import(const uint8_t* bytes, size_t size, Vertex* vertices)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniGltfOptions options = maniDefaultGltfOptions();
    maniSkeleton skeleton;
    maniSkin skin;
    maniInfluences v;
    size_t built = 0;
    maniImportResult r = maniOpenGltf(builder, bytes, size, &gltf);
    bool ok =
        r.result == mani_success &&
        maniGltfBuildSkeleton(builder, gltf, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &built) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, built, &skeleton).result == mani_success &&
        maniGltfBuildSkin(builder, gltf, 0, &skeleton, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skinFile, sizeof s_skinFile, &built) == mani_success &&
        maniOpenSkin(s_skinFile, built, &skin).result == mani_success;
    CHECK(r.result != mani_success || ok, "skeleton and skin");
    if (ok)
    {
        r = maniGltfBuildInfluences(builder, gltf, 0, 0, &skin);
    }
    if (ok && r.result == mani_success && vertices != NULL)
    {
        CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
                  maniOpenInfluences(s_file, s_size, &v).result == mani_success &&
                  maniInfluencesVertexCount(&v) == VERTICES,
              "influences open");
        for (uint32_t i = 0; i < VERTICES; ++i)
        {
            for (uint32_t k = 0; k < SLOTS; ++k)
            {
                vertices[i].joints[k] = maniInfluence(&v, i, k, &vertices[i].weights[k]);
            }
        }
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    return r;
}

// A joint's weight in a vertex (0 for a joint it does not hold).
static float WeightOf(const Vertex* v, uint32_t joint)
{
    float w = 0.0f;
    for (uint32_t k = 0; k < SLOTS; ++k)
    {
        w += v->weights[k] != 0.0f && v->joints[k] == joint ? v->weights[k] : 0.0f;
    }
    return w;
}

// The same joints' weights, in whatever order (quantization may swap two
// equal weights).
static bool Same(const Vertex* a, const Vertex* b, float tolerance)
{
    for (uint32_t k = 0; k < SLOTS; ++k)
    {
        if (fabsf(WeightOf(a, a->joints[k]) - WeightOf(b, a->joints[k])) > tolerance ||
            fabsf(WeightOf(a, b->joints[k]) - WeightOf(b, b->joints[k])) > tolerance)
        {
            return false;
        }
    }
    return true;
}

// Every vertex of a matched by its own vertex of b.
static bool SameVertices(const Vertex* a, const Vertex* b, float tolerance)
{
    bool used[VERTICES] = {false};
    for (uint32_t i = 0; i < VERTICES; ++i)
    {
        bool found = false;
        for (uint32_t j = 0; j < VERTICES && !found; ++j)
        {
            found = !used[j] && Same(&a[i], &b[j], tolerance);
            used[j] = used[j] || found;
        }
        if (!found)
        {
            return false;
        }
    }
    return true;
}

static void TestInfluences(void)
{
    Vertex plain[VERTICES];
    Vertex packed[VERTICES];
    size_t size = ReadFile("SimpleSkinEmbedded.gltf");
    CHECK(Import(s_bytes, size, plain).result == mani_success, "the plain skin");
    size = ReadFile("SimpleSkin-draco.glb");
    // The encoder quantizes float weights to 12 bits by default.
    CHECK(Import(s_bytes, size, packed).result == mani_success &&
              SameVertices(plain, packed, 0.5f / 4095.0f + 1e-6f),
          "float weights: the plain skin's to a half step");
    size = ReadFile("SimpleSkin-draco-q.glb");
    CHECK(Import(s_bytes, size, packed).result == mani_success &&
              SameVertices(plain, packed, 0.5f / 255.0f + 1e-6f),
          "normalized byte weights: the plain skin's to a half step");
}

// The file with one part of its JSON replaced by text of the same length.
static size_t Changed(const char* name, const char* from, const char* to)
{
    size_t size = ReadFile(name);
    size_t n = strlen(from);
    CHECK(n == strlen(to), "a same-length change");
    for (size_t i = 0; i + n <= size; ++i)
    {
        if (memcmp(s_bytes + i, from, n) == 0)
        {
            memcpy(s_bytes + i, to, n);
            return size;
        }
    }
    CHECK(false, from);
    return size;
}

static void TestRefusals(void)
{
    static const struct
    {
        const char* from;
        const char* to;
        maniImportReason reason;
        uint32_t item;
        const char* what;
    } kChanges[] = {
        {"\"JOINTS_0\":1,\"WEIGHTS_0\":2}}", "\"JOINTS_0\":7,\"WEIGHTS_0\":2}}",
         maniImportReasonProperty, 0, "a unique id not in the stream"},
        {"\"JOINTS_0\":1,\"WEIGHTS_0\":2}}", "\"JOINTS_0\":1,\"WEIGHTS_9\":2}}",
         maniImportReasonProperty, 0, "a name not among the primitive's"},
        {"\"VEC4\",\"componentType\":5123", "\"VEC4\",\"componentType\":5121",
         maniImportReasonProperty, 2, "a component type not the stream's"},
        {"\"bufferView\":2,\"attributes\"", "\"bufferView\":1,\"attributes\"", maniImportReasonData,
         1, "a view that is no stream"},
    };
    for (size_t i = 0; i < sizeof kChanges / sizeof kChanges[0]; ++i)
    {
        size_t size = Changed("SimpleSkin-draco.glb", kChanges[i].from, kChanges[i].to);
        maniImportResult r = Import(s_bytes, size, NULL);
        CHECK(r.result == mani_errorFormat && r.reason == kChanges[i].reason &&
                  r.item == kChanges[i].item,
              kChanges[i].what);
    }
    // Every attribute's count past the stream's ten points.
    size_t size = ReadFile("SimpleSkin-draco.glb");
    for (size_t i = 0; i + 10 <= size; ++i)
    {
        if (memcmp(s_bytes + i, "\"count\":10", 10) == 0)
        {
            s_bytes[i + 9] = '2';
        }
    }
    maniImportResult r = Import(s_bytes, size, NULL);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty && r.item == 1,
          "accessors of more points than the stream's");
}

// Every byte of the stream (view 2's 213 bytes, from its magic) changed
// to its complement, imported without fault.
static void TestDamage(void)
{
    size_t size = ReadFile("SimpleSkin-draco.glb");
    size_t start = 0;
    while (start + 5 <= size && memcmp(s_bytes + start, "DRACO", 5) != 0)
    {
        ++start;
    }
    CHECK(start + 213u <= size, "the stream");
    for (size_t at = start; at < start + 213u && at < size; ++at)
    {
        s_bytes[at] = (uint8_t)~s_bytes[at];
        (void)Import(s_bytes, size, NULL);
        s_bytes[at] = (uint8_t)~s_bytes[at];
    }
}

int main(void)
{
    TestInfluences();
    TestRefusals();
    TestDamage();
    return s_failures == 0 ? 0 : 1;
}
