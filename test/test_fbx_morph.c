// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX morph targets (mani-0021): a BlendShape's channels and shapes as
// targets, in-betweens and negative targets sorted, a shape after an
// equal target and a channel without shapes left out, names as the
// clips' tracks, offsets moved by the options' unit (normals not),
// indices past the control points skipped, Blender's per-vertex weights,
// rest weights; the 6.x form; the refusals and the misuses.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/morph.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Face's geometry has four control points. Its BlendShape has Smile (rest
// 25 percent; Smile_half at 50 with normals, Smile at 100 with an index
// past the points), Wide (rest 50; two unnamed shapes at 1, the second
// left out), Frown (its unnamed shapes at 1 and -1, sorted) and Empty (no
// shapes).
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Geometry: 50, \"Geometry::Face\", \"Mesh\" {\n"
    "  Vertices: *12 {\n   a: 0,0,0,1,0,0,0,1,0,0,0,1\n  }\n }\n"
    " Model: 1, \"Model::Face\", \"Mesh\" {\n }\n"
    " Deformer: 60, \"Deformer::Face\", \"BlendShape\" {\n }\n"
    " Deformer: 61, \"SubDeformer::Smile\", \"BlendShapeChannel\" {\n  Properties70: {\n"
    "   P: \"DeformPercent\", \"Number\", \"\", \"A\",25\n  }\n"
    "  FullWeights: *2 {\n   a: 50,100\n  }\n }\n"
    " Geometry: 70, \"Geometry::Smile_half\", \"Shape\" {\n"
    "  Indexes: *2 {\n   a: 0,1\n  }\n"
    "  Vertices: *6 {\n   a: 0,0,2,0,0,2\n  }\n"
    "  Normals: *6 {\n   a: 0,0,0.5,0,0,0\n  }\n }\n"
    " Geometry: 71, \"Geometry::Smile\", \"Shape\" {\n"
    "  Indexes: *3 {\n   a: 0,1,9\n  }\n"
    "  Vertices: *9 {\n   a: 0,0,4,1,0,0,5,5,5\n  }\n }\n"
    " Deformer: 62, \"SubDeformer::Wide\", \"BlendShapeChannel\" {\n  Properties70: {\n"
    "   P: \"DeformPercent\", \"Number\", \"\", \"A\",50\n  }\n }\n"
    " Geometry: 72, \"Geometry::\", \"Shape\" {\n"
    "  Indexes: *1 {\n   a: 2\n  }\n  Vertices: *3 {\n   a: 0,1,0\n  }\n }\n"
    " Geometry: 73, \"Geometry::\", \"Shape\" {\n"
    "  Indexes: *1 {\n   a: 2\n  }\n  Vertices: *3 {\n   a: 7,7,7\n  }\n }\n"
    " Deformer: 63, \"SubDeformer::Frown\", \"BlendShapeChannel\" {\n"
    "  FullWeights: *2 {\n   a: 100,-100\n  }\n }\n"
    " Geometry: 74, \"Geometry::\", \"Shape\" {\n"
    "  Indexes: *1 {\n   a: 3\n  }\n  Vertices: *3 {\n   a: 0,0,-1\n  }\n }\n"
    " Geometry: 75, \"Geometry::\", \"Shape\" {\n"
    "  Indexes: *1 {\n   a: 3\n  }\n  Vertices: *3 {\n   a: 0,0,1\n  }\n }\n"
    " Deformer: 64, \"SubDeformer::Empty\", \"BlendShapeChannel\" {\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n C: \"OO\",50,1\n C: \"OO\",60,50\n"
    " C: \"OO\",61,60\n C: \"OO\",62,60\n C: \"OO\",63,60\n C: \"OO\",64,60\n"
    " C: \"OO\",70,61\n C: \"OO\",71,61\n C: \"OO\",72,62\n C: \"OO\",73,62\n"
    " C: \"OO\",74,63\n C: \"OO\",75,63\n"
    "}\n";

static char s_text[8192];
static _Alignas(16) uint8_t s_file[16384];

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static maniFbx* Open(maniBuilder* builder, const char* text)
{
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, text, strlen(text), &fbx);
    CHECK(r.result == mani_success, "opens");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu near: %.30s\n", r.reason, (unsigned long long)r.offset,
               text + r.offset);
    }
    return fbx;
}

// The scene's first set, or a refusal.
static maniImportResult Import(const char* text, const maniFbxOptions* options, maniMorphTargets* m)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, text);
    maniImportResult r = {mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    if (fbx != NULL)
    {
        r = maniFbxBuildMorphTargets(builder, fbx, 0, options);
        size_t size = 0;
        if (r.result == mani_success)
        {
            CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
                      maniOpenMorphTargets(s_file, size, m).result == mani_success,
                  "opens");
        }
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return r;
}

// kScene with one piece of text replaced, into s_text.
static const char* Replace(const char* text, const char* from, const char* to)
{
    static char copy[8192];
    snprintf(copy, sizeof copy, "%s", text);
    const char* at = strstr(copy, from);
    CHECK(at != NULL, "the text to replace");
    if (at == NULL)
    {
        return copy;
    }
    size_t before = (size_t)(at - copy);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)before, copy, to, at + strlen(from));
    return s_text;
}

static bool Named(const maniMorphTargets* m, uint32_t target, const char* name)
{
    uint32_t length = 0;
    const char* got = maniMorphTargetName(m, target, &length);
    bool right = got != NULL && length == strlen(name) && memcmp(got, name, length) == 0;
    if (!right)
    {
        printf("  target %u: \"%.*s\", expected \"%s\"\n", target, (int)length,
               got != NULL ? got : "", name);
    }
    return right;
}

// The positions (and normals, from (0,1,0)) of the (up to four) vertices with
// one target at weight 1.
static void Apply(const maniMorphTargets* m, uint32_t target, float* positions, float* normals)
{
    static const float kZero[12] = {0};
    static const float kUp[12] = {0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0};
    float weights[8] = {0};
    weights[target] = 1.0f;
    maniMorphStream streams[2] = {
        {kZero, positions, 12, 12, maniMorphPosition, 0, 3, 0},
        {kUp, normals, 12, 12, maniMorphNormal, 0, 3, 0},
    };
    maniMorphJob job = {m, weights, 0, maniMorphVertexCount(m), 2, 0, streams};
    CHECK(maniApplyMorphTargets(&job) == mani_success, "applies");
}

static bool Near(float a, float b)
{
    return fabsf(a - b) <= 1e-6f;
}

static void TestTargets(void)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    maniMorphTargets m;
    maniImportResult r = Import(kScene, &options, &m);
    CHECK(r.result == mani_success, "imports");
    if (r.result != mani_success)
    {
        printf("  reason %u item %u\n", r.reason, r.item);
        return;
    }
    CHECK(maniMorphVertexCount(&m) == 4 && maniMorphChannelCount(&m) == 3 &&
              maniMorphTargetCount(&m) == 5,
          "three channels (Empty left out), five targets (Wide's second left out)");
    CHECK(maniMorphAttributeCount(&m) == 2 && maniMorphAttribute(&m, 1, NULL) == maniMorphNormal,
          "normals, as one shape has them");
    CHECK(Named(&m, 0, "Smile_half") && Named(&m, 1, "Smile") && Named(&m, 2, "Wide") &&
              Named(&m, 3, "Frown") && Named(&m, 4, "Frown/1"),
          "names as the clips' tracks: a shape's own, else its channel's and its index");
    CHECK(maniMorphTargetFullWeight(&m, 0) == 0.5f && maniMorphTargetFullWeight(&m, 1) == 1.0f &&
              maniMorphTargetFullWeight(&m, 3) == -1.0f && maniMorphTargetFullWeight(&m, 4) == 1.0f,
          "FullWeights / 100, sorted");
}

static void TestOffsets(void)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    maniMorphTargets m;
    if (Import(kScene, &options, &m).result != mani_success)
    {
        return;
    }
    float p[12];
    float n[12];
    Apply(&m, 1, p, n);
    CHECK(p[2] == 4.0f && p[3] == 1.0f && p[5] == 0.0f && p[11] == 0.0f,
          "Smile: offsets, the index past the points skipped");
    Apply(&m, 0, p, n);
    CHECK(p[2] == 2.0f && p[5] == 2.0f && Near(n[1], 0.894427191f) && Near(n[2], 0.447213595f) &&
              n[4] == 1.0f,
          "Smile_half: positions and normals");
    Apply(&m, 2, p, n);
    CHECK(p[7] == 1.0f && p[6] == 0.0f, "Wide: the first shape");
    Apply(&m, 3, p, n);
    CHECK(p[11] == 1.0f, "Frown at -1: the second shape, sorted first");
    Apply(&m, 4, p, n);
    CHECK(p[11] == -1.0f, "Frown at 1");
}

static void TestUnit(void)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    maniMorphTargets m;
    float p[12];
    float n[12];
    options.unitMeters = 1.0;
    bool imported = Import(kScene, &options, &m).result == mani_success;
    CHECK(imported, "imports in metres");
    if (!imported)
    {
        return;
    }
    Apply(&m, 0, p, n);
    CHECK(Near(p[2], 0.02f) && Near(n[1], 0.894427191f) && Near(n[2], 0.447213595f),
          "positions in the options' unit, normals not scaled");
}

static void TestWeights(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    float w[6] = {9, 9, 9, 9, 9, 9};
    CHECK(maniFbxMorphCount(fbx) == 1 && maniFbxMorphTargetCount(fbx, 0) == 5 &&
              maniFbxMorphTargetCount(fbx, 1) == 0,
          "one set of five targets");
    uint32_t model = maniFbxMorphModel(fbx, 0);
    uint32_t length = 0;
    const char* name = maniFbxModelBoneName(fbx, model, &length);
    CHECK(name != NULL && length == 4 && memcmp(name, "Face", 4) == 0 &&
              maniFbxMorphModel(fbx, 1) == UINT32_MAX,
          "the mesh model");
    CHECK(maniFbxMorphWeights(fbx, 0, w, 5).result == mani_success && w[0] == 0.5f &&
              w[1] == 0.0f && w[2] == 0.5f && w[3] == 0.0f && w[4] == 0.0f && w[5] == 9.0f,
          "rest weights, walked as the clips walk them");
    CHECK(maniFbxMorphWeights(fbx, 0, w, 4).result == mani_errorInvalid &&
              maniFbxMorphWeights(fbx, 0, NULL, 5).result == mani_errorInvalid &&
              maniFbxMorphWeights(fbx, 1, w, 0).result == mani_errorInvalid &&
              maniFbxMorphWeights(NULL, 0, w, 5).result == mani_errorInvalid,
          "weight misuses");
    maniFbxOptions options = maniDefaultFbxOptions();
    CHECK(maniFbxBuildMorphTargets(builder, fbx, 1, &options).result == mani_errorInvalid &&
              maniFbxBuildMorphTargets(builder, fbx, 0, NULL).result == mani_errorInvalid &&
              maniFbxBuildMorphTargets(builder, NULL, 0, &options).result == mani_errorInvalid &&
              maniFbxBuildMorphTargets(NULL, fbx, 0, &options).result == mani_errorInvalid &&
              maniFbxMorphCount(NULL) == 0 && maniFbxMorphTargetCount(NULL, 0) == 0,
          "misuses");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Blender's binary exporter: every target 1, so a channel keeps its
// first shape, whose offsets take FullWeights / 100 when as many.
static void TestBlender(void)
{
    char text[8192];
    snprintf(text, sizeof text, "%s",
             Replace(kScene, "Objects: {\n",
                     "FBXHeaderExtension: {\n Creator: \"blender-2.79\"\n}\nObjects: {\n"));
    maniFbxOptions options = maniDefaultFbxOptions();
    maniMorphTargets m;
    maniImportResult r = Import(text, &options, &m);
    CHECK(r.result == mani_success && maniMorphTargetCount(&m) == 3 && Named(&m, 0, "Smile_half") &&
              Named(&m, 2, "Frown"),
          "a target a channel");
    float p[12];
    float n[12];
    Apply(&m, 0, p, n);
    CHECK(p[2] == 1.0f && p[5] == 2.0f, "per-vertex weights 0.5 and 1");
    Apply(&m, 2, p, n);
    CHECK(p[11] == -1.0f, "Frown's first shape, its FullWeights not as many as its offsets");
}

static void ExpectRefused(const char* text, maniImportReason reason, uint32_t item,
                          const char* what)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    maniMorphTargets m;
    maniImportResult r = Import(text, &options, &m);
    CHECK(r.result == mani_errorFormat && r.reason == reason && r.item == item, what);
    if (r.reason != reason || r.item != item)
    {
        printf("  %s: result %d reason %u item %u\n", what, r.result, r.reason, r.item);
    }
}

static void TestRefusals(void)
{
    ExpectRefused(Replace(kScene, "  Vertices: *9 {\n   a: 0,0,4,1,0,0,5,5,5\n  }\n",
                          "  Vertices: *6 {\n   a: 0,0,4,1,0,0\n  }\n"),
                  maniImportReasonProperty, 0, "Vertices not three a index");
    ExpectRefused(Replace(kScene, "  Normals: *6 {\n   a: 0,0,0.5,0,0,0\n  }\n",
                          "  Normals: *3 {\n   a: 0,0,0.5\n  }\n"),
                  maniImportReasonProperty, 0, "Normals not as many as Vertices");
    ExpectRefused(Replace(kScene, "  Vertices: *12 {\n   a: 0,0,0,1,0,0,0,1,0,0,0,1\n  }\n", ""),
                  maniImportReasonProperty, 0, "no control points");
    ExpectRefused(Replace(kScene, "  Vertices: *12 {\n   a: 0,0,0,1,0,0,0,1,0,0,0,1\n  }\n",
                          "  Vertices: *11 {\n   a: 0,0,0,1,0,0,0,1,0,0,0\n  }\n"),
                  maniImportReasonProperty, 0, "control points not in threes");
    ExpectRefused(Replace(kScene, "a: 50,100", "a: 0,100"), maniImportReasonBuild,
                  maniBuildReasonWeight, "a full weight of 0");
}

// 6.x: Shape children of the mesh model, each a channel of one target at
// 1 weighted by the model's property of its name; Hollow (no arrays) is
// no shape.
static void TestSix(void)
{
    static const char kSix[] = "; FBX 6.1.0 project file\n"
                               "Objects: {\n"
                               " Model: \"Model::mesh\", \"Mesh\" {\n"
                               "  Properties60: {\n"
                               "   Property: \"Smile\", \"Number\", \"A+\",10\n"
                               "  }\n"
                               "  Vertices: 0,0,0,1,0,0\n"
                               "  PolygonVertexIndex: 0,-2\n"
                               "  Shape: \"Hollow\" {\n  }\n"
                               "  Shape: \"Smile\" {\n"
                               "   Indexes: 1\n"
                               "   Vertices: 0,3,0\n"
                               "  }\n"
                               " }\n"
                               "}\n"
                               "Connections: {\n"
                               " Connect: \"OO\", \"Model::mesh\", \"Model::Scene\"\n"
                               "}\n";
    maniFbxOptions options = maniDefaultFbxOptions();
    maniMorphTargets m;
    maniImportResult r = Import(kSix, &options, &m);
    CHECK(r.result == mani_success && maniMorphVertexCount(&m) == 2 &&
              maniMorphChannelCount(&m) == 1 && maniMorphTargetCount(&m) == 1 &&
              Named(&m, 0, "Smile") && maniMorphTargetFullWeight(&m, 0) == 1.0f,
          "a 6.x shape: a channel of one target at 1");
    if (r.result != mani_success)
    {
        printf("  reason %u item %u\n", r.reason, r.item);
        return;
    }
    float p[12];
    float n[12];
    Apply(&m, 0, p, n);
    CHECK(p[4] == 3.0f && p[1] == 0.0f, "its offset");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kSix);
    float w = 0.0f;
    CHECK(maniFbxMorphCount(fbx) == 1 && maniFbxMorphTargetCount(fbx, 0) == 1 &&
              maniFbxMorphWeights(fbx, 0, &w, 1).result == mani_success && Near(w, 0.1f),
          "its weight: the property over 100");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestTargets();
    TestOffsets();
    TestUnit();
    TestWeights();
    TestBlender();
    TestRefusals();
    TestSix();
    return s_failures == 0 ? 0 : 1;
}
