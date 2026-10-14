// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skeleton builds (mani-0014). For every bone of a model, the
// placement times the bone's model matrix must be the model's FBX world
// (in the output's basis when converting): with normal inheritance,
// exact compensation, helper bones where it is inexact or requested, a
// folded placement and converted axes and units. Then the refusals, the
// misuses, model subtrees, a helper's name taken, and the settings.

#include "basis.h"
#include "fbx_document.h"
#include "fbx_transform.h"
#include "fbx_world.h"
#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static char s_text[8192];
static _Alignas(16) uint8_t s_file[16384];

// Armature (a null scaled 100, the placement) over a limb chain: Hips
// (pivots, pre-rotation, rotated), Spine (scaled as given), Head (Rrs,
// rotated 30 about z) under Spine, Leg (RrSs) under Hips; a mesh under
// the armature.
static void Rig(const char* spineScale, const char* extra)
{
    snprintf(s_text, sizeof s_text,
             "; FBX 7.4.0 project file\n"
             "GlobalSettings: {\n Properties70: {\n%s }\n}\n"
             "Objects: {\n"
             " Model: 1, \"Model::Armature\", \"Null\" {\n  Properties70: {\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0,5\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",-90,0,0\n"
             "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",100,100,100\n  }\n }\n"
             " Model: 2, \"Model::Hips\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n"
             "   P: \"PreRotation\", \"Vector3D\", \"Vector\", \"\",10,0,0\n"
             "   P: \"RotationPivot\", \"Vector3D\", \"Vector\", \"\",0.5,0,0\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,20,0\n  }\n }\n"
             " Model: 3, \"Model::Spine\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,2,0\n"
             "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",%s\n  }\n }\n"
             " Model: 4, \"Model::Head\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"InheritType\", \"enum\", \"\", \"\",2\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,30\n  }\n }\n"
             " Model: 5, \"Model::Leg\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"InheritType\", \"enum\", \"\", \"\",0\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,-1,0\n"
             "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",1,1,2\n  }\n }\n"
             " Model: 6, \"Model::Body\", \"Mesh\" {\n }\n"
             "}\n"
             "Connections: {\n C: \"OO\",1,0\n C: \"OO\",2,1\n C: \"OO\",3,2\n C: \"OO\",4,3\n"
             " C: \"OO\",5,2\n C: \"OO\",6,1\n}\n",
             extra, spineScale);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static maniFbx* Open(maniBuilder* builder)
{
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, s_text, strlen(s_text), &fbx);
    CHECK(r.result == mani_success, "opens");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu near: %.30s\n", r.reason, (unsigned long long)r.offset,
               s_text + r.offset);
    }
    return fbx;
}

static bool Near(const maniAffine* a, const maniAffine* b)
{
    float largest = 1.0f;
    for (int i = 0; i < 12; ++i)
    {
        largest = fmaxf(largest, fabsf(b->m[i]));
    }
    for (int i = 0; i < 12; ++i)
    {
        if (!(fabsf(a->m[i] - b->m[i]) <= 2e-5f * largest))
        {
            return false;
        }
    }
    return true;
}

// The bone's model matrix: its rests composed up its parents.
static maniAffine ModelOf(const maniSkeleton* skeleton, uint32_t bone)
{
    maniTransform t = maniSkeletonRest(skeleton, bone);
    maniAffine m = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    for (uint32_t p = maniSkeletonParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeletonParent(skeleton, p))
    {
        t = maniSkeletonRest(skeleton, p);
        maniAffine parent = maniAffineFromTrs(t.translation, t.rotation, t.scale);
        m = maniAffineMul(&parent, &m);
    }
    return m;
}

// Builds the skeleton and checks every model bone against its world;
// the bone count.
static uint32_t Check(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options,
                      const char* what)
{
    maniImportResult r = maniFbxBuildSkeleton(builder, fbx, 0, options);
    CHECK(r.result == mani_success, what);
    if (r.result != mani_success)
    {
        printf("  result %d reason %u item %u\n", (int)r.result, r.reason, r.item);
        return 0;
    }
    size_t size = 0;
    maniSkeleton skeleton;
    CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenSkeleton(s_file, size, &skeleton).result == mani_success,
          "opens the skeleton");
    maniFbxWorld worlds[8];
    (void)maniFbxEvaluateWorlds(fbx, NULL, worlds);
    maniAffine placement;
    CHECK(maniFbxSkeletonPlacement(fbx, 0, options, &placement).result == mani_success,
          "placement");
    // The converting case's basis, derived by hand: TestAxes' file is X
    // right, Z up, -Y front in centimetres, turned to X, Y, Z in metres.
    maniAxis none[3] = {maniAxisX, maniAxisY, maniAxisZ};
    maniAxis turned[3] = {maniAxisX, maniAxisZ, maniAxisNegativeY};
    bool converts = options->convertAxes != 0 || options->unitMeters > 0;
    maniBasis basis = converts ? maniBasisMake(turned, 0.01f) : maniBasisMake(none, 1.0f);
    bool all = true;
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        uint32_t length = 0;
        const char* name = maniFbxModelBoneName(fbx, m, &length);
        uint32_t bone = maniSkeletonFindBone(&skeleton, name, length);
        if (bone == MANI_NO_BONE)
        {
            continue;
        }
        maniAffine model = ModelOf(&skeleton, bone);
        maniAffine got = maniAffineMul(&placement, &model);
        maniAffine expected = maniBasisAffine(&basis, maniFbxMatrixAffine(&worlds[m].world));
        all = all && Near(&got, &expected);
    }
    CHECK(all, what);
    return maniSkeletonBoneCount(&skeleton);
}

static void TestBuilds(void)
{
    maniBuilder* builder = NewBuilder();
    Rig("2,2,2", "");
    maniFbx* fbx = Open(builder);
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    maniFbxOptions options = maniDefaultFbxOptions();
    CHECK(Check(builder, fbx, &options, "uniform scale: compensated exactly") == 4,
          "Hips, Spine, Head, Leg, no helpers");
    options.inheritHelpers = 1;
    CHECK(Check(builder, fbx, &options, "helpers on request") == 6,
          "a helper under Hips and under Spine");
    options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    CHECK(Check(builder, fbx, &options, "the placement folded") == 4, "four bones folded");
    maniCloseFbx(fbx);
    Rig("1,2,3", "");
    fbx = Open(builder);
    options = maniDefaultFbxOptions();
    CHECK(fbx != NULL && Check(builder, fbx, &options, "non-uniform scale: a helper") == 5,
          "Spine's helper only");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestHelperShape(void)
{
    maniBuilder* builder = NewBuilder();
    Rig("1,2,3", "");
    maniFbx* fbx = Open(builder);
    maniFbxOptions options = maniDefaultFbxOptions();
    size_t size = 0;
    maniSkeleton skeleton;
    CHECK(fbx != NULL && maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenSkeleton(s_file, size, &skeleton).result == mani_success,
          "builds");
    uint32_t spine = maniSkeletonFindBone(&skeleton, "Spine", 5);
    uint32_t helper = maniSkeletonFindBone(&skeleton, "Spine#unscaled", 14);
    uint32_t head = maniSkeletonFindBone(&skeleton, "Head", 4);
    maniTransform h = maniSkeletonRest(&skeleton, helper);
    CHECK(helper != MANI_NO_BONE && maniSkeletonParent(&skeleton, helper) == spine &&
              maniSkeletonParent(&skeleton, head) == helper && h.scale.x == 1.0f &&
              h.scale.y == 0.5f && fabsf(h.scale.z - 1.0f / 3.0f) < 1e-7f && h.translation.x == 0 &&
              h.rotation.w == 1,
          "a helper: the inverse of its parent's scale, Head under it");
    CHECK(maniSkeletonFindBone(&skeleton, "Body", 4) == MANI_NO_BONE &&
              maniSkeletonFindBone(&skeleton, "Armature", 8) == MANI_NO_BONE,
          "the mesh and the armature are no bones");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestAxes(void)
{
    // Z up, -Y front, X right, centimetres, turned to Y up, Z front,
    // metres: output y is file z, output z is minus file y.
    maniBuilder* builder = NewBuilder();
    Rig("2,2,2", "  P: \"UpAxis\", \"int\", \"Integer\", \"\",2\n"
                 "  P: \"FrontAxis\", \"int\", \"Integer\", \"\",1\n"
                 "  P: \"FrontAxisSign\", \"int\", \"Integer\", \"\",-1\n");
    maniFbx* fbx = Open(builder);
    maniFbxOptions options = maniDefaultFbxOptions();
    options.convertAxes = 1;
    options.unitMeters = 1.0;
    CHECK(fbx != NULL && Check(builder, fbx, &options, "axes and units converted") == 4,
          "four bones converted");
    maniFbxSettings settings;
    CHECK(maniFbxReadSettings(fbx, &settings) == mani_success && settings.axes[0] == maniAxisX &&
              settings.axes[1] == maniAxisZ && settings.axes[2] == maniAxisNegativeY &&
              settings.unitMeters == 0.01 && settings.rateNumerator == 30,
          "the settings");
    CHECK(maniFbxReadSettings(NULL, &settings) == mani_errorInvalid &&
              maniFbxReadSettings(fbx, NULL) == mani_errorInvalid,
          "settings misuse");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(maniImportResult r, maniImportReason reason, const char* at,
                          uint32_t item, const char* what)
{
    bool right = r.result == mani_errorFormat && r.reason == reason &&
                 r.offset == (uint64_t)(strstr(s_text, at) - s_text) && r.item == item;
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu item %u\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, r.item);
    }
}

static void TestRefusals(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbxOptions options = maniDefaultFbxOptions();
    Rig("0,1,1", "");
    maniFbx* fbx = Open(builder);
    ExpectRefusal(maniFbxBuildSkeleton(builder, fbx, 0, &options), maniImportReasonMatrix,
                  "Model: 3", 2, "a helper for a zero scale");
    maniCloseFbx(fbx);
    // A non-uniform armature under a rotated root: folding shears.
    Rig("2,2,2", "");
    char* scale = strstr(s_text, "100,100,100");
    memcpy(scale, "100,200,300", 11);
    fbx = Open(builder);
    options.foldPlacement = 1;
    ExpectRefusal(maniFbxBuildSkeleton(builder, fbx, 0, &options), maniImportReasonMatrix,
                  "Model: 2", 1, "a folded placement that shears");
    maniCloseFbx(fbx);
    // A model named as a helper.
    Rig("1,2,3", "");
    char* body = strstr(s_text, "Model::Body");
    char renamed[8192];
    snprintf(renamed, sizeof renamed, "%.*sModel::Spine#unscaled\", \"LimbNode%s",
             (int)(body - s_text), s_text, body + strlen("Model::Body\", \"Mesh"));
    memcpy(s_text, renamed, strlen(renamed) + 1);
    char* link = strstr(s_text, "C: \"OO\",6,1");
    link[10] = '4';
    fbx = Open(builder);
    options = maniDefaultFbxOptions();
    maniImportResult r = maniFbxBuildSkeleton(builder, fbx, 0, &options);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonName,
          "a helper's name taken");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestMisuse(void)
{
    maniBuilder* builder = NewBuilder();
    Rig("2,2,2", "");
    maniFbx* fbx = Open(builder);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniFbxOptions bad[5] = {options, options, options, options, options};
    bad[0].axes[1] = maniAxisX;
    bad[1].unitMeters = -1.0;
    bad[2].unitMeters = (double)NAN;
    bad[3].foldPlacement = 2;
    bad[4].inheritHelpers = 2;
    bool refused = true;
    for (int i = 0; i < 5; ++i)
    {
        refused =
            refused && maniFbxBuildSkeleton(builder, fbx, 0, &bad[i]).result == mani_errorInvalid;
    }
    bad[0] = options;
    bad[0].convertAxes = 2;
    refused = refused && maniFbxBuildSkeleton(builder, fbx, 0, &bad[0]).result == mani_errorInvalid;
    CHECK(refused, "options out of range");
    maniAffine placement;
    CHECK(maniFbxBuildSkeleton(builder, fbx, 1, &options).result == mani_errorInvalid &&
              maniFbxBuildSkeleton(NULL, fbx, 0, &options).result == mani_errorInvalid &&
              maniFbxBuildSkeleton(builder, NULL, 0, &options).result == mani_errorInvalid &&
              maniFbxBuildSkeleton(builder, fbx, 0, NULL).result == mani_errorInvalid &&
              maniFbxBuildModelSkeleton(builder, fbx, 6, &options).result == mani_errorInvalid &&
              maniFbxSkeletonPlacement(fbx, 1, &options, &placement).result == mani_errorInvalid &&
              maniFbxSkeletonPlacement(fbx, 0, &bad[1], &placement).result == mani_errorInvalid &&
              maniFbxSkeletonPlacement(fbx, 0, &options, NULL).result == mani_errorInvalid &&
              maniFbxModelPlacement(fbx, 6, &options, &placement).result == mani_errorInvalid &&
              maniFbxModelPlacement(NULL, 0, &options, &placement).result == mani_errorInvalid,
          "arguments out of range");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestModelSkeleton(void)
{
    maniBuilder* builder = NewBuilder();
    Rig("2,2,2", "");
    maniFbx* fbx = Open(builder);
    maniFbxOptions options = maniDefaultFbxOptions();
    size_t size = 0;
    maniSkeleton skeleton;
    CHECK(maniFbxBuildModelSkeleton(builder, fbx, 0, &options).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenSkeleton(s_file, size, &skeleton).result == mani_success &&
              maniSkeletonBoneCount(&skeleton) == 6 &&
              maniSkeletonFindBone(&skeleton, "Body", 4) != MANI_NO_BONE &&
              maniSkeletonFindBone(&skeleton, "Armature", 8) == 0,
          "a model's whole subtree");
    maniAffine placement;
    maniAffine identity = maniIdentityAffine();
    CHECK(maniFbxModelPlacement(fbx, 0, &options, &placement).result == mani_success &&
              Near(&placement, &identity),
          "a top model hangs from nothing");
    maniFbxWorld worlds[6];
    (void)maniFbxEvaluateWorlds(fbx, NULL, worlds);
    maniAffine armature = maniFbxMatrixAffine(&worlds[0].world);
    CHECK(maniFbxModelPlacement(fbx, 1, &options, &placement).result == mani_success &&
              Near(&placement, &armature),
          "Hips hangs from the armature's world");
    maniAffine unscaled = maniFbxMatrixAffine(&worlds[2].unscaled);
    CHECK(maniFbxModelPlacement(fbx, 3, &options, &placement).result == mani_success &&
              Near(&placement, &unscaled),
          "Rrs Head hangs from Spine's unscaled world");
    options.foldPlacement = 1;
    CHECK(maniFbxSkeletonPlacement(fbx, 0, &options, &placement).result == mani_success &&
              Near(&placement, &identity),
          "folded: identity");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestBuilds();
    TestHelperShape();
    TestAxes();
    TestRefusals();
    TestMisuse();
    TestModelSkeleton();
    return s_failures == 0 ? 0 : 1;
}
