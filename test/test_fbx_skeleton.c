// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skeleton builds. For every bone of a model, the
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
// (pivots, pre-rotation, rotated), Spine (scaled as given), under Spine
// Head (Rrs, rotated 30 about z), Arm (Rrs, not rotated), Chest
// (normal) and Prop (a mesh, no bone), Neck (normal) under Head, Leg
// (RrSs) under Hips; Body, a mesh, under the armature. Models: 0
// Armature, 1 Hips, 2 Spine, 3 Head, 4 Leg, 5 Body, 6 Arm, 7 Chest, 8
// Neck, 9 Prop.
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
             " Model: 7, \"Model::Arm\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"InheritType\", \"enum\", \"\", \"\",2\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n  }\n }\n"
             " Model: 8, \"Model::Chest\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0.5,0\n"
             "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,15\n  }\n }\n"
             " Model: 9, \"Model::Neck\", \"LimbNode\" {\n  Properties70: {\n"
             "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0.5,0\n  }\n }\n"
             " Model: 10, \"Model::Prop\", \"Mesh\" {\n }\n"
             "}\n"
             "Connections: {\n C: \"OO\",1,0\n C: \"OO\",2,1\n C: \"OO\",3,2\n C: \"OO\",4,3\n"
             " C: \"OO\",5,2\n C: \"OO\",6,1\n C: \"OO\",7,3\n C: \"OO\",8,3\n"
             " C: \"OO\",10,3\n C: \"OO\",9,4\n}\n",
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
    maniTransform t = maniSkeleton_GetRest(skeleton, bone);
    maniAffine m = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    for (uint32_t p = maniSkeleton_GetParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeleton_GetParent(skeleton, p))
    {
        t = maniSkeleton_GetRest(skeleton, p);
        maniAffine parent = maniAffineFromTrs(t.translation, t.rotation, t.scale);
        m = maniAffineMul(&parent, &m);
    }
    return m;
}

// Builds the skeleton and checks every model bone against its world;
// the bone count.
static uint32_t Check(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options,
                      const maniBasis* basis, const char* what)
{
    maniImportResult r = maniBuildFbxSkeleton(builder, fbx, 0, options);
    CHECK(r.result == mani_success, what);
    if (r.result != mani_success)
    {
        printf("  result %d reason %u item %u\n", (int)r.result, r.reason, r.item);
        return 0;
    }
    size_t size = 0;
    maniSkeleton skeleton;
    CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenSkeleton(s_file, size, &skeleton).result == mani_success,
          "opens the skeleton");
    maniFbxWorld worlds[16];
    (void)maniFbxEvaluateWorlds(fbx, NULL, worlds);
    maniAffine placement;
    CHECK(maniFbx_GetSkeletonPlacement(fbx, 0, options, &placement).result == mani_success,
          "placement");
    bool all = true;
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        size_t length = 0;
        const char* name = maniFbx_GetModelBoneName(fbx, m, &length);
        uint32_t bone = maniSkeleton_FindBone(&skeleton, name, length);
        if (bone == MANI_NO_BONE)
        {
            continue;
        }
        maniAffine model = ModelOf(&skeleton, bone);
        maniAffine got = maniAffineMul(&placement, &model);
        maniAffine expected = maniBasisAffine(basis, maniFbxMatrixAffine(&worlds[m].world));
        all = all && Near(&got, &expected);
    }
    CHECK(all, what);
    return maniSkeleton_GetBoneCount(&skeleton);
}

static maniBasis Identity(void)
{
    maniAxis none[3] = {maniAxisX, maniAxisY, maniAxisZ};
    return maniBasisMake(none, 1.0f);
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
    maniBasis identity = Identity();
    maniFbxOptions options = maniDefaultFbxOptions();
    CHECK(Check(builder, fbx, &options, &identity, "uniform scale: compensated exactly") == 7,
          "seven bones, no helpers, no mesh");
    options.inheritHelpers = 1;
    CHECK(Check(builder, fbx, &options, &identity, "helpers on request") == 9,
          "a helper under Hips and under Spine, none for normal children");
    options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    CHECK(Check(builder, fbx, &options, &identity, "the placement folded") == 7,
          "seven bones folded");
    maniCloseFbx(fbx);
    Rig("1,2,3", "");
    fbx = Open(builder);
    options = maniDefaultFbxOptions();
    CHECK(fbx != NULL &&
              Check(builder, fbx, &options, &identity, "non-uniform scale: a helper") == 8,
          "Spine's helper only, whatever its later children");
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
    CHECK(fbx != NULL && maniBuildFbxSkeleton(builder, fbx, 0, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenSkeleton(s_file, size, &skeleton).result == mani_success,
          "builds");
    uint32_t spine = maniSkeleton_FindBone(&skeleton, "Spine", 5);
    uint32_t helper = maniSkeleton_FindBone(&skeleton, "Spine#unscaled", 14);
    uint32_t head = maniSkeleton_FindBone(&skeleton, "Head", 4);
    uint32_t arm = maniSkeleton_FindBone(&skeleton, "Arm", 3);
    uint32_t chest = maniSkeleton_FindBone(&skeleton, "Chest", 5);
    maniTransform h = maniSkeleton_GetRest(&skeleton, helper);
    CHECK(maniSkeleton_GetParent(&skeleton, arm) == helper &&
              maniSkeleton_GetParent(&skeleton, chest) == spine,
          "every RrSs or Rrs child under the helper, normal ones under the parent");
    CHECK(helper != MANI_NO_BONE && maniSkeleton_GetParent(&skeleton, helper) == spine &&
              maniSkeleton_GetParent(&skeleton, head) == helper && h.scale.x == 1.0f &&
              h.scale.y == 0.5f && fabsf(h.scale.z - 1.0f / 3.0f) < 1e-7f && h.translation.x == 0 &&
              h.rotation.w == 1,
          "a helper: the inverse of its parent's scale, Head under it");
    CHECK(maniSkeleton_FindBone(&skeleton, "Body", 4) == MANI_NO_BONE &&
              maniSkeleton_FindBone(&skeleton, "Armature", 8) == MANI_NO_BONE,
          "the meshes and the armature are no bones");
    CHECK(maniSkeleton_FindBone(&skeleton, "Prop", 4) == MANI_NO_BONE, "a mesh under a bone");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestAxes(void)
{
    // The file: X right, Z up, -Y front, centimetres.
    maniBuilder* builder = NewBuilder();
    Rig("2,2,2", "  P: \"UpAxis\", \"int\", \"Integer\", \"\",2\n"
                 "  P: \"FrontAxis\", \"int\", \"Integer\", \"\",1\n"
                 "  P: \"FrontAxisSign\", \"int\", \"Integer\", \"\",-1\n");
    maniFbx* fbx = Open(builder);
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    // To X right, Y up, Z front in metres: output y is file z, output z
    // minus file y.
    maniAxis toYUp[3] = {maniAxisX, maniAxisZ, maniAxisNegativeY};
    maniBasis basis = maniBasisMake(toYUp, 0.01f);
    maniFbxOptions options = maniDefaultFbxOptions();
    options.convertAxes = 1;
    options.unitMeters = 1.0;
    CHECK(Check(builder, fbx, &options, &basis, "to Y up in metres") == 7, "seven bones");
    // To -X right, Y up, -Z front: output x is minus file x, output z
    // file y (its front, negated, negated).
    maniAxis toBack[3] = {maniAxisNegativeX, maniAxisZ, maniAxisY};
    basis = maniBasisMake(toBack, 1.0f);
    options = maniDefaultFbxOptions();
    options.convertAxes = 1;
    options.axes[0] = maniAxisNegativeX;
    options.axes[2] = maniAxisNegativeZ;
    CHECK(Check(builder, fbx, &options, &basis, "to negative targets") == 7, "seven bones");
    // To the file's own axes, given out of order: nothing changes.
    basis = Identity();
    options.axes[0] = maniAxisX;
    options.axes[1] = maniAxisZ;
    options.axes[2] = maniAxisNegativeY;
    CHECK(Check(builder, fbx, &options, &basis, "to its own axes") == 7, "seven bones");
    options = maniDefaultFbxOptions();
    CHECK(Check(builder, fbx, &options, &basis, "not converted") == 7, "seven bones");
    maniFbxSettings settings;
    CHECK(maniFbx_ReadSettings(fbx, &settings) == mani_success && settings.axes[0] == maniAxisX &&
              settings.axes[1] == maniAxisZ && settings.axes[2] == maniAxisNegativeY &&
              settings.unitMeters == 0.01 && settings.rateNumerator == 30,
          "the settings");
    CHECK(maniFbx_ReadSettings(NULL, &settings) == mani_errorInvalid &&
              maniFbx_ReadSettings(fbx, NULL) == mani_errorInvalid,
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
    ExpectRefusal(maniBuildFbxSkeleton(builder, fbx, 0, &options), maniImportReasonMatrix,
                  "Model: 3", 2, "a helper for a zero scale");
    maniCloseFbx(fbx);
    Rig("1e-300,1,1", "");
    fbx = Open(builder);
    ExpectRefusal(maniBuildFbxSkeleton(builder, fbx, 0, &options), maniImportReasonMatrix,
                  "Model: 3", 2, "a helper scale past binary32");
    maniCloseFbx(fbx);
    // A non-uniform armature under a rotated root: folding shears.
    Rig("2,2,2", "");
    char* scale = strstr(s_text, "100,100,100");
    memcpy(scale, "100,200,300", 11);
    fbx = Open(builder);
    options.foldPlacement = 1;
    ExpectRefusal(maniBuildFbxSkeleton(builder, fbx, 0, &options), maniImportReasonMatrix,
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
    maniImportResult r = maniBuildFbxSkeleton(builder, fbx, 0, &options);
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
            refused && maniBuildFbxSkeleton(builder, fbx, 0, &bad[i]).result == mani_errorInvalid;
    }
    bad[0] = options;
    bad[0].convertAxes = 2;
    refused = refused && maniBuildFbxSkeleton(builder, fbx, 0, &bad[0]).result == mani_errorInvalid;
    CHECK(refused, "options out of range");
    maniAffine placement;
    CHECK(
        maniBuildFbxSkeleton(builder, fbx, 1, &options).result == mani_errorInvalid &&
            maniBuildFbxSkeleton(NULL, fbx, 0, &options).result == mani_errorInvalid &&
            maniBuildFbxSkeleton(builder, NULL, 0, &options).result == mani_errorInvalid &&
            maniBuildFbxSkeleton(builder, fbx, 0, NULL).result == mani_errorInvalid &&
            maniBuildFbxModelSkeleton(builder, fbx, 10, &options).result == mani_errorInvalid &&
            maniFbx_GetSkeletonPlacement(fbx, 1, &options, &placement).result ==
                mani_errorInvalid &&
            maniFbx_GetSkeletonPlacement(fbx, 0, &bad[1], &placement).result == mani_errorInvalid &&
            maniFbx_GetSkeletonPlacement(fbx, 0, &options, NULL).result == mani_errorInvalid &&
            maniFbx_GetModelPlacement(fbx, 10, &options, &placement).result == mani_errorInvalid &&
            maniFbx_GetModelPlacement(NULL, 0, &options, &placement).result == mani_errorInvalid,
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
    CHECK(maniBuildFbxModelSkeleton(builder, fbx, 0, &options).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenSkeleton(s_file, size, &skeleton).result == mani_success &&
              maniSkeleton_GetBoneCount(&skeleton) == 10 &&
              maniSkeleton_FindBone(&skeleton, "Body", 4) != MANI_NO_BONE &&
              maniSkeleton_FindBone(&skeleton, "Armature", 8) == 0,
          "a model's whole subtree");
    maniAffine placement;
    maniAffine identity = maniIdentityAffine();
    CHECK(maniFbx_GetModelPlacement(fbx, 0, &options, &placement).result == mani_success &&
              Near(&placement, &identity),
          "a top model hangs from nothing");
    maniFbxWorld worlds[16];
    (void)maniFbxEvaluateWorlds(fbx, NULL, worlds);
    maniAffine armature = maniFbxMatrixAffine(&worlds[0].world);
    CHECK(maniFbx_GetModelPlacement(fbx, 1, &options, &placement).result == mani_success &&
              Near(&placement, &armature),
          "Hips hangs from the armature's world");
    maniAffine unscaled = maniFbxMatrixAffine(&worlds[2].unscaled);
    CHECK(maniFbx_GetModelPlacement(fbx, 3, &options, &placement).result == mani_success &&
              Near(&placement, &unscaled),
          "Rrs Head hangs from Spine's unscaled world");
    options.foldPlacement = 1;
    CHECK(maniFbx_GetSkeletonPlacement(fbx, 0, &options, &placement).result == mani_success &&
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
