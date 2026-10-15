// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX influences (mani-0021): clusters' Indexes and Weights a control
// point at a time, a cluster's repeated index summed, an index past the
// control points skipped, a cluster linking nothing left; the skinning
// method from SkinningType and the blend weights; the 6.x form; the
// refusals and the misuses.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Body's mesh has four control points. Skin 40, SkinningType Blend,
// blend weights 0.25 and 0.75 for points 1 and 3. Cluster 41 links
// Hips: points 0, 1, 1 (summed) and 9 (past the points); cluster 42
// links Spine: points 1 and 2; cluster 43 links nothing (point 3 left).
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Geometry: 50, \"Geometry::\", \"Mesh\" {\n"
    "  Vertices: *12 {\n   a: 0,0,0,1,0,0,0,1,0,0,0,1\n  }\n }\n"
    " Model: 1, \"Model::Body\", \"Mesh\" {\n }\n"
    " Model: 2, \"Model::Hips\", \"LimbNode\" {\n }\n"
    " Model: 3, \"Model::Spine\", \"LimbNode\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    " Deformer: 40, \"Deformer::\", \"Skin\" {\n"
    "  SkinningType: \"Blend\"\n"
    "  Indexes: *2 {\n   a: 1,3\n  }\n"
    "  BlendWeights: *2 {\n   a: 0.25,0.75\n  }\n }\n"
    " Deformer: 41, \"SubDeformer::\", \"Cluster\" {\n"
    "  Indexes: *4 {\n   a: 0,1,1,9\n  }\n"
    "  Weights: *4 {\n   a: 1,0.25,0.25,0.5\n  }\n }\n"
    " Deformer: 42, \"SubDeformer::\", \"Cluster\" {\n"
    "  Indexes: *2 {\n   a: 1,2\n  }\n"
    "  Weights: *2 {\n   a: 0.5,1\n  }\n }\n"
    " Deformer: 43, \"SubDeformer::\", \"Cluster\" {\n"
    "  Indexes: *1 {\n   a: 3\n  }\n"
    "  Weights: *1 {\n   a: 1\n  }\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n C: \"OO\",50,1\n C: \"OO\",40,50\n"
    " C: \"OO\",2,0\n C: \"OO\",3,2\n"
    " C: \"OO\",41,40\n C: \"OO\",42,40\n C: \"OO\",43,40\n"
    " C: \"OO\",2,41\n C: \"OO\",3,42\n"
    "}\n";

static char s_text[8192];
static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_skinFile[16384];
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

// Skeleton 0 and skin 0, opened into skin.
static bool BuildSkin(maniBuilder* builder, const maniFbx* fbx, maniSkin* skin)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    return fbx != NULL && maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success &&
           maniFbxBuildSkin(builder, fbx, 0, &skeleton, &options).result == mani_success &&
           maniCopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) == mani_success &&
           maniOpenSkin(s_skinFile, size, skin).result == mani_success;
}

// The scene's influences, or a refusal.
static maniImportResult Import(const char* text, maniInfluences* v)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, text);
    maniSkin skin;
    maniImportResult r = {mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    if (BuildSkin(builder, fbx, &skin))
    {
        r = maniFbxBuildInfluences(builder, fbx, 0, &skin);
        size_t size = 0;
        if (r.result == mani_success)
        {
            CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
                      maniOpenInfluences(s_file, size, v).result == mani_success,
                  "opens");
        }
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return r;
}

// kScene with one piece of text replaced, into s_text.
static const char* Replace(const char* from, const char* to)
{
    const char* at = strstr(kScene, from);
    CHECK(at != NULL, "the text to replace");
    size_t before = (size_t)(at - kScene);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)before, kScene, to, at + strlen(from));
    return s_text;
}

static void TestInfluences(void)
{
    maniInfluences v;
    maniImportResult r = Import(kScene, &v);
    CHECK(r.result == mani_success, "imports");
    if (r.result != mani_success)
    {
        printf("  reason %u item %u\n", r.reason, r.item);
        return;
    }
    float w0 = 0.0f;
    float w1 = 0.0f;
    CHECK(maniInfluencesVertexCount(&v) == 4, "a vertex a control point");
    CHECK(maniInfluence(&v, 0, 0, &w0) == 0 && w0 == 1.0f, "point 0 on Hips");
    uint32_t j0 = maniInfluence(&v, 1, 0, &w0);
    uint32_t j1 = maniInfluence(&v, 1, 1, &w1);
    CHECK(j0 == 0 && j1 == 1 && w0 == 0.5f && w1 == 0.5f,
          "point 1: Hips's repeat summed, beside Spine");
    CHECK(maniInfluence(&v, 2, 0, &w0) == 1 && w0 == 1.0f, "point 2 on Spine");
    (void)maniInfluence(&v, 3, 0, &w0);
    CHECK(w0 == 0.0f, "point 3: only an unlinked cluster's, so none");
    CHECK(maniInfluencesDualQuaternionWeight(&v, 1) == 0.25f &&
              maniInfluencesDualQuaternionWeight(&v, 3) == 0.75f &&
              maniInfluencesDualQuaternionWeight(&v, 0) == 0.0f,
          "blend weights, 0 elsewhere for a Blend skin");
}

static maniSkinningMethod Method(const char* text)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, text);
    maniSkinningMethod m = fbx != NULL ? maniFbxSkinMethod(fbx, 0) : 99;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return m;
}

static void TestMethods(void)
{
    static const char kBlend[] = "  Indexes: *2 {\n   a: 1,3\n  }\n"
                                 "  BlendWeights: *2 {\n   a: 0.25,0.75\n  }\n";
    CHECK(Method(kScene) == maniSkinningBlended, "Blend");
    char text[8192];
    snprintf(text, sizeof text, "%s", Replace(kBlend, ""));
    CHECK(Method(text) == maniSkinningBlended, "Blend without weights");
    snprintf(text, sizeof text, "%s", Replace("\"Blend\"\n", "\"Linear\"\n"));
    CHECK(Method(text) == maniSkinningBlended, "Linear with blend weights: blended");
    snprintf(text, sizeof text, "%s",
             Replace("  SkinningType: \"Blend\"\n"
                     "  Indexes: *2 {\n   a: 1,3\n  }\n"
                     "  BlendWeights: *2 {\n   a: 0.25,0.75\n  }\n",
                     "  SkinningType: \"Rigid\"\n"));
    CHECK(Method(text) == maniSkinningLinear, "Rigid: linear");
    snprintf(text, sizeof text, "%s",
             Replace("  SkinningType: \"Blend\"\n"
                     "  Indexes: *2 {\n   a: 1,3\n  }\n"
                     "  BlendWeights: *2 {\n   a: 0.25,0.75\n  }\n",
                     "  SkinningType: \"DualQuaternion\"\n"));
    CHECK(Method(text) == maniSkinningDualQuaternion, "DualQuaternion");
    maniInfluences v;
    CHECK(Import(text, &v).result == mani_success &&
              maniInfluencesDualQuaternionWeight(&v, 0) == 1.0f &&
              maniInfluencesDualQuaternionWeight(&v, 2) == 1.0f,
          "a dual quaternion skin's blend weights are 1");
    snprintf(text, sizeof text, "%s",
             Replace("  SkinningType: \"Blend\"\n"
                     "  Indexes: *2 {\n   a: 1,3\n  }\n"
                     "  BlendWeights: *2 {\n   a: 0.25,0.75\n  }\n",
                     ""));
    CHECK(Method(text) == maniSkinningLinear, "no SkinningType: linear");
    CHECK(Import(text, &v).result == mani_success &&
              maniInfluencesDualQuaternionWeight(&v, 1) == 0.0f,
          "a linear skin carries no blend weights");
}

static void ExpectRefused(const char* text, maniImportReason reason, uint32_t item,
                          const char* what)
{
    maniInfluences v;
    maniImportResult r = Import(text, &v);
    CHECK(r.result == mani_errorFormat && r.reason == reason && r.item == item, what);
    if (r.reason != reason || r.item != item)
    {
        printf("  %s: result %d reason %u item %u\n", what, r.result, r.reason, r.item);
    }
}

static void TestRefusals(void)
{
    ExpectRefused(Replace("a: 1,0.25,0.25,0.5", "a: 1,-0.25,0.25,0.5"), maniImportReasonInfluence,
                  1, "a negative weight");
    ExpectRefused(Replace("a: 0.25,0.75", "a: 0.25,1.5"), maniImportReasonInfluence, 3,
                  "a blend weight past 1");
    ExpectRefused(Replace("  Weights: *4 {\n   a: 1,0.25,0.25,0.5\n  }\n",
                          "  Weights: *3 {\n   a: 1,0.25,0.25\n  }\n"),
                  maniImportReasonProperty, 0, "Indexes and Weights of different counts");
    ExpectRefused(Replace("  Vertices: *12 {\n   a: 0,0,0,1,0,0,0,1,0,0,0,1\n  }\n", ""),
                  maniImportReasonProperty, 0, "no control points");
    ExpectRefused(Replace("  Vertices: *12 {\n   a: 0,0,0,1,0,0,0,1,0,0,0,1\n  }\n",
                          "  Vertices: *11 {\n   a: 0,0,0,1,0,0,0,1,0,0,0\n  }\n"),
                  maniImportReasonProperty, 0, "control points not in threes");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniSkin skin;
    CHECK(BuildSkin(builder, fbx, &skin), "skin");
    CHECK(maniFbxBuildInfluences(builder, fbx, 1, &skin).result == mani_errorInvalid &&
              maniFbxBuildInfluences(builder, fbx, 0, NULL).result == mani_errorInvalid &&
              maniFbxBuildInfluences(builder, NULL, 0, &skin).result == mani_errorInvalid &&
              maniFbxBuildInfluences(NULL, fbx, 0, &skin).result == mani_errorInvalid,
          "misuses");
    CHECK(maniFbxSkinMethod(fbx, 1) == maniSkinningLinear && maniFbxSkinMethod(NULL, 0) == 0,
          "a skin past the count: linear");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// 6.x: the skin under the mesh model, which holds the control points.
static void TestSix(void)
{
    static const char kSix[] = "; FBX 6.1.0 project file\n"
                               "Objects: {\n"
                               " Model: \"Model::hip\", \"Limb\" {\n }\n"
                               " Model: \"Model::mesh\", \"Mesh\" {\n"
                               "  Vertices: 0,0,0,1,0,0\n"
                               "  PolygonVertexIndex: 0,-2\n }\n"
                               " Deformer: \"Deformer::skin\", \"Skin\" {\n }\n"
                               " Deformer: \"SubDeformer::c\", \"Cluster\" {\n"
                               "  Indexes: 1\n  Weights: 0.5\n"
                               "  Transform: 1,0,0,0,0,1,0,0,0,0,1,0,4,0,0,1\n"
                               "  TransformLink: 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1\n }\n"
                               "}\n"
                               "Connections: {\n"
                               " Connect: \"OO\", \"Model::hip\", \"Model::Scene\"\n"
                               " Connect: \"OO\", \"Model::mesh\", \"Model::Scene\"\n"
                               " Connect: \"OO\", \"Deformer::skin\", \"Model::mesh\"\n"
                               " Connect: \"OO\", \"SubDeformer::c\", \"Deformer::skin\"\n"
                               " Connect: \"OO\", \"Model::hip\", \"SubDeformer::c\"\n"
                               "}\n";
    maniInfluences v;
    maniImportResult r = Import(kSix, &v);
    float w = 0.0f;
    CHECK(r.result == mani_success && maniInfluencesVertexCount(&v) == 2 &&
              maniInfluence(&v, 1, 0, &w) == 0 && w == 1.0f,
          "a 6.x skin: point 1 renormalized to 1");
    if (r.result != mani_success)
    {
        printf("  reason %u item %u\n", r.reason, r.item);
    }
}

int main(void)
{
    TestInfluences();
    TestMethods();
    TestRefusals();
    TestSix();
    return s_failures == 0 ? 0 : 1;
}
