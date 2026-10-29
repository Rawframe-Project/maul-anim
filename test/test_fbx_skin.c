// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skins: joints from clusters by bone name, a cluster
// linking nothing skipped, inverse binds from the cluster's Transform,
// from TransformLink, from a bind pose and from the rest, each times the
// mesh's geometric transform, in metres on request; the 6.x form with
// plain matrices; the refusals and the misuses.

#include "fbx_document.h"
#include "test_fbx_sweep.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Body (mesh at z 2, geometry moved 1 along z and scaled 2); Hips at y
// 1, Spine above, Arm beside. Skin 0's clusters: 41 Hips with Transform
// (y -1, z -2) and TransformLink; 42 Spine with an all-zero Transform
// beside its TransformLink (y 3); 43 Arm with Transform alone (not used
// without a link), in the bind pose at x 5 (a later bind pose at 7
// ignored); 44 Hips with an all-zero Transform and TransformLink and an
// all-zero bind pose entry (a rest pose at 9 ignored), so at rest; 45
// linking a model by a property connection only, skipped; a blend shape
// channel under the skin with a model child, no cluster. Skin 1 hangs
// from the root (no mesh; its cluster links Arm with a TransformLink
// alone at 1 2 3, unread as ufbx leaves it, so Arm's bind pose at x 5);
// skin 2 from a video under Body (no mesh); skin 3 from geometries in a
// cycle (no mesh).
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Geometry: 50, \"Geometry::\", \"Mesh\" {\n }\n"
    " Model: 1, \"Model::Body\", \"Mesh\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0,2\n"
    "   P: \"GeometricTranslation\", \"Vector3D\", \"Vector\", \"\",0,0,1\n"
    "   P: \"GeometricScaling\", \"Vector3D\", \"Vector\", \"\",2,2,2\n  }\n }\n"
    " Model: 2, \"Model::Hips\", \"LimbNode\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    " Model: 3, \"Model::Spine\", \"LimbNode\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    " Model: 4, \"Model::Arm\", \"LimbNode\" {\n  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n  }\n }\n"
    " Deformer: 40, \"Deformer::\", \"Skin\" {\n }\n"
    " Deformer: 41, \"SubDeformer::\", \"Cluster\" {\n"
    "  Transform: *16 {\n   a: 1,0,0,0,0,1,0,0,0,0,1,0,0,-1,-2,1\n  }\n"
    "  TransformLink: *16 {\n   a: 1,0,0,0,0,1,0,0,0,0,1,0,0,1,0,1\n  }\n }\n"
    " Deformer: 42, \"SubDeformer::\", \"Cluster\" {\n"
    "  Transform: *16 {\n   a: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n  }\n"
    "  TransformLink: *16 {\n   a: 1,0,0,0,0,1,0,0,0,0,1,0,0,3,0,1\n  }\n }\n"
    " Deformer: 43, \"SubDeformer::\", \"Cluster\" {\n"
    "  Transform: *16 {\n   a: 1,0,0,0,0,1,0,0,0,0,1,0,0,0,9,1\n  }\n }\n"
    " Deformer: 44, \"SubDeformer::\", \"Cluster\" {\n"
    "  Transform: *16 {\n   a: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n  }\n"
    "  TransformLink: *16 {\n   a: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n  }\n }\n"
    " Deformer: 45, \"SubDeformer::\", \"Cluster\" {\n }\n"
    " Deformer: 46, \"SubDeformer::\", \"BlendShapeChannel\" {\n }\n"
    " Pose: 61, \"Pose::rest\", \"RestPose\" {\n"
    "  PoseNode: {\n   Node: 2\n   Matrix: *16 {\n    a: 1,0,0,0,0,1,0,0,0,0,1,0,9,0,0,1\n   }\n  "
    "}\n }\n"
    " Pose: 60, \"Pose::BIND_POSES\", \"BindPose\" {\n  Type: \"BindPose\"\n"
    "  PoseNode: {\n   Node: 4\n   Matrix: *16 {\n    a: 1,0,0,0,0,1,0,0,0,0,1,0,5,0,0,1\n   }\n  "
    "}\n"
    "  PoseNode: {\n   Node: 2\n   Matrix: *16 {\n    a: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n   }\n  "
    "}\n }\n"
    " Pose: 62, \"Pose::later\", \"BindPose\" {\n"
    "  PoseNode: {\n   Node: 4\n   Matrix: *16 {\n    a: 1,0,0,0,0,1,0,0,0,0,1,0,7,0,0,1\n   }\n  "
    "}\n }\n"
    " Deformer: 70, \"Deformer::\", \"Skin\" {\n }\n"
    " Deformer: 73, \"SubDeformer::\", \"Cluster\" {\n"
    "  TransformLink: *16 {\n   a: 1,0,0,0,0,1,0,0,0,0,1,0,1,2,3,1\n  }\n }\n"
    " Video: 71, \"Video::\", \"Clip\" {\n }\n"
    " Deformer: 72, \"Deformer::\", \"Skin\" {\n }\n"
    " Geometry: 76, \"Geometry::\", \"Mesh\" {\n }\n"
    " Geometry: 77, \"Geometry::\", \"Mesh\" {\n }\n"
    " Deformer: 75, \"Deformer::\", \"Skin\" {\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n C: \"OO\",50,1\n C: \"OO\",40,50\n"
    " C: \"OO\",2,0\n C: \"OO\",3,2\n C: \"OO\",4,3\n"
    " C: \"OO\",41,40\n C: \"OO\",42,40\n C: \"OO\",43,40\n C: \"OO\",44,40\n"
    " C: \"OO\",45,40\n C: \"OO\",46,40\n"
    " C: \"OO\",2,41\n C: \"OO\",3,42\n C: \"OO\",4,43\n C: \"OO\",2,44\n"
    " C: \"OP\",3,45, \"Visibility\"\n C: \"OO\",3,46\n"
    " C: \"OO\",70,0\n C: \"OO\",73,70\n C: \"OO\",4,73\n"
    " C: \"OO\",72,71\n C: \"OO\",71,1\n"
    " C: \"OO\",75,76\n C: \"OO\",76,77\n C: \"OO\",77,76\n"
    "}\n";

static char s_text[8192];
static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_skinFile[16384];

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

// Builds and opens skeleton 0 with the options.
static bool Skeleton(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options,
                     maniSkeleton* out)
{
    size_t size = 0;
    return maniBuildFbxSkeleton(builder, fbx, 0, options).result == mani_success &&
           maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
               mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, out).result == mani_success;
}

static maniAffine maniIdentityAffineForTest(void)
{
    maniAffine a = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    return a;
}

// A scaled translation's matrix: 2 I and t.
static bool IsBind(const maniAffine* a, float scale, float x, float y, float z)
{
    const float expected[12] = {scale, 0, 0, 0, scale, 0, 0, 0, scale, x, y, z};
    for (int i = 0; i < 12; ++i)
    {
        if (!(fabsf(a->m[i] - expected[i]) <= 1e-6f))
        {
            return false;
        }
    }
    return true;
}

static void TestBinds(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    if (fbx == NULL || !Skeleton(builder, fbx, &options, &skeleton))
    {
        CHECK(false, "a skeleton");
        maniCloseFbx(fbx);
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbx_GetSkinModel(fbx, 0) == 0 && maniFbx_GetSkinModel(fbx, 1) == UINT32_MAX &&
              maniFbx_GetSkinModel(fbx, 2) == UINT32_MAX &&
              maniFbx_GetSkinModel(fbx, 3) == UINT32_MAX &&
              maniFbx_GetSkinModel(fbx, 4) == UINT32_MAX &&
              maniFbx_GetSkinModel(NULL, 0) == UINT32_MAX,
          "each skin's mesh model: through a geometry only, and only one");
    maniImportResult r = maniBuildFbxSkin(builder, fbx, 0, &skeleton, &options);
    size_t size = 0;
    maniSkin skin;
    CHECK(r.result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) ==
                  mani_success &&
              maniOpenSkin(s_skinFile, size, &skin).result == mani_success,
          "builds the skin");
    uint32_t hips = maniSkeleton_FindBone(&skeleton, "Hips", 4);
    uint32_t spine = maniSkeleton_FindBone(&skeleton, "Spine", 5);
    uint32_t arm = maniSkeleton_FindBone(&skeleton, "Arm", 3);
    CHECK(maniSkin_GetJointCount(&skin) == 4 && maniSkin_GetJoint(&skin, 0) == hips &&
              maniSkin_GetJoint(&skin, 1) == spine && maniSkin_GetJoint(&skin, 2) == arm &&
              maniSkin_GetJoint(&skin, 3) == hips,
          "joints in cluster order, the unlinked one skipped, Hips twice");
    // G = T(0 0 1) S(2); the mesh at z 2.
    maniAffine b0 = maniSkin_GetInverseBind(&skin, 0);
    maniAffine b1 = maniSkin_GetInverseBind(&skin, 1);
    maniAffine b2 = maniSkin_GetInverseBind(&skin, 2);
    maniAffine b3 = maniSkin_GetInverseBind(&skin, 3);
    CHECK(IsBind(&b0, 2, 0, -1, -1), "Transform times G");
    CHECK(maniBuildFbxSkin(builder, fbx, 1, &skeleton, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) ==
                  mani_success &&
              maniOpenSkin(s_skinFile, size, &skin).result == mani_success,
          "a skin with no mesh");
    maniAffine lone = maniSkin_GetInverseBind(&skin, 0);
    CHECK(IsBind(&lone, 1, -5, 0, 0), "a lone TransformLink unread, relative to the origin");
    CHECK(IsBind(&b1, 2, 0, -3, 3), "TransformLink inverted, the mesh at rest, G");
    CHECK(IsBind(&b2, 2, -5, 0, 3), "the bind pose's matrix");
    CHECK(IsBind(&b3, 2, 0, -1, 3), "the rest when nothing else is given");
    // In metres: translations a hundredth.
    options.unitMeters = 1.0;
    CHECK(Skeleton(builder, fbx, &options, &skeleton) &&
              maniBuildFbxSkin(builder, fbx, 0, &skeleton, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) ==
                  mani_success &&
              maniOpenSkin(s_skinFile, size, &skin).result == mani_success,
          "builds in metres");
    b0 = maniSkin_GetInverseBind(&skin, 0);
    CHECK(IsBind(&b0, 2, 0, -0.01f, -0.01f), "a bind in metres");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestSix(void)
{
    static const char kSix[] = "; FBX 6.1.0 project file\n"
                               "Objects: {\n"
                               " Model: \"Model::hip\", \"Limb\" {\n }\n"
                               " Model: \"Model::mesh\", \"Mesh\" {\n }\n"
                               " Deformer: \"Deformer::skin\", \"Skin\" {\n }\n"
                               " Deformer: \"SubDeformer::c\", \"Cluster\" {\n"
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
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kSix);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    maniSkin skin;
    CHECK(fbx != NULL && maniFbx_GetSkinModel(fbx, 0) == 1 &&
              Skeleton(builder, fbx, &options, &skeleton) &&
              maniBuildFbxSkin(builder, fbx, 0, &skeleton, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) ==
                  mani_success &&
              maniOpenSkin(s_skinFile, size, &skin).result == mani_success,
          "a 6.x skin on its mesh model");
    maniAffine b = maniSkin_GetInverseBind(&skin, 0);
    CHECK(maniSkin_GetJointCount(&skin) == 1 && IsBind(&b, 1, 4, 0, 0), "plain matrices");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// kScene with one piece of text replaced.
static void Replace(const char* from, const char* to)
{
    const char* at = strstr(kScene, from);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)(at - kScene), kScene, to, at + strlen(from));
}

static void ExpectSkinRefusal(uint32_t skinIndex, maniImportReason reason, const char* at,
                              const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, s_text);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    if (fbx == NULL || !Skeleton(builder, fbx, &options, &skeleton))
    {
        CHECK(false, what);
        maniCloseFbx(fbx);
        maniDestroyBuilder(builder);
        return;
    }
    maniImportResult r = maniBuildFbxSkin(builder, fbx, skinIndex, &skeleton, &options);
    bool right = r.result == mani_errorFormat && r.reason == reason &&
                 r.offset == (uint64_t)(strstr(s_text, at) - s_text) && r.item == skinIndex;
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu item %u\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, r.item);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(maniImportReason reason, const char* at, const char* what)
{
    ExpectSkinRefusal(0, reason, at, what);
}

// Skin 0's first bind after a replacement.
static bool FirstBind(float x, float y, float z)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, s_text);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    maniSkin skin;
    bool right =
        fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
        maniBuildFbxSkin(builder, fbx, 0, &skeleton, &options).result == mani_success &&
        maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) == mani_success &&
        maniOpenSkin(s_skinFile, size, &skin).result == mani_success;
    maniAffine b = right ? maniSkin_GetInverseBind(&skin, 0) : maniIdentityAffineForTest();
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return right && IsBind(&b, 2, x, y, z);
}

static void TestMatrices(void)
{
    // A Transform with no translation is given, not all zero.
    Replace("0,0,1,0,0,-1,-2,1", "0,0,1,0,0,0,0,1");
    CHECK(FirstBind(0, 0, 1), "a Transform of identity");
    // An all-zero Transform is none: TransformLink's then.
    Replace("a: 1,0,0,0,0,1,0,0,0,0,1,0,0,-1,-2,1", "a: 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    CHECK(FirstBind(0, -1, 3), "an all-zero Transform");
}

static void TestRefusals(void)
{
    Replace(" Deformer: 41, \"SubDeformer::\", \"Cluster\" {\n",
            " Deformer: 41, \"SubDeformer::\", \"Cluster\" {\n  Mode: \"Additive\"\n");
    ExpectRefusal(maniImportReasonProperty, "Mode: ", "an additive cluster");
    Replace("0,0,1,0,0,-1,-2,1", "0,0,1,0,0,-1,-2,2");
    ExpectRefusal(maniImportReasonMatrix, "Transform:", "a matrix not affine");
    Replace("0,0,1,0,0,-1,-2,1", "0,0,1,1,0,-1,-2,1");
    ExpectRefusal(maniImportReasonMatrix, "Transform:", "a projective row");
    Replace("0,0,1,0,1,2,3,1", "0,0,1,0,1,2,1e999,1");
    ExpectSkinRefusal(1, maniImportReasonData, "1e999", "a number past binary64 in skin 1");
    Replace("*16 {\n   a: 1,0,0,0,0,1,0,0,0,0,1,0,0,-1,-2,1", "*4 {\n   a: 1,0,0,0");
    // The array's offset is its first value's.
    ExpectRefusal(maniImportReasonProperty, " 1,0,0,0\n", "a matrix too short");
    Replace("a: 1,0,0,0,0,1,0,0,0,0,1,0,0,3,0,1", "a: 0,0,0,0,0,1,0,0,0,0,1,0,0,3,0,1");
    ExpectRefusal(maniImportReasonMatrix, "Deformer: 42", "a bind world with no inverse");
    // A skeleton of Spine's subtree lacks Hips.
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    size_t size = 0;
    maniSkeleton skeleton;
    CHECK(maniBuildFbxModelSkeleton(builder, fbx, 2, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success,
          "Spine's subtree");
    maniImportResult r = maniBuildFbxSkin(builder, fbx, 0, &skeleton, &options);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBone &&
              r.offset == (uint64_t)(strstr(kScene, "Deformer: 41") - kScene) && r.item == 0,
          "a joint the skeleton lacks");
    maniFbxOptions bad = options;
    bad.unitMeters = -1;
    CHECK(maniBuildFbxSkin(builder, fbx, 4, &skeleton, &options).result == mani_errorInvalid &&
              maniBuildFbxSkin(builder, fbx, 0, NULL, &options).result == mani_errorInvalid &&
              maniBuildFbxSkin(builder, NULL, 0, &skeleton, &options).result == mani_errorInvalid &&
              maniBuildFbxSkin(NULL, fbx, 0, &skeleton, &options).result == mani_errorInvalid &&
              maniBuildFbxSkin(builder, fbx, 0, &skeleton, &bad).result == mani_errorInvalid,
          "misuses");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Every allocation failing in turn while opening the scene, building its
// skeleton and its first skin.
static void TestAllocationFailures(void)
{
    FbxSweep sweep = {kScene, UINT32_MAX, 0};
    CHECK(TestSweep(FbxSweepWork, &sweep, 8192), "allocation failures handled, nothing kept");
}

int main(void)
{
    TestAllocationFailures();
    TestBinds();
    TestSix();
    TestMatrices();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
