// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX files before 6.0 (mani-0017): top-level models and their Children,
// rest values from the last channel driving them in file order, the
// legacy frame rate, against what ufbx gives for the same file.

#include "fbx_anim.h"
#include "fbx_take.h"
#include "fbx_value.h"
#include "test_fbx_sweep.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Body, a mesh skinned to Hips and Spine with a Smile shape; Hips (a
// limb, its own channels giving T 1 2 3) above Spine (its own channels
// turning it about z from 10). Walk's channels on Hips come later in the
// file, so Hips' rest is theirs, 5 6 0 (Z has no Default); Walk takes
// Smile from 0 to 100. Settings give 24 frames a second.
static const char kScene[] = "; FBX 5.0.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1002\n"
                             " FBXVersion: 5000\n"
                             "}\n"
                             "Model: \"Model::Body\" {\n"
                             " Vertices: 0,0,0,1,0,0,0,1,0\n"
                             " PolygonVertexIndex: 0,1,-3\n"
                             " MaterialAssignation: \"AllSame\"\n"
                             " Shape: \"Smile\" {\n"
                             "  Indexes: 0\n"
                             "  Vertices: 0,0,1\n"
                             " }\n"
                             " Link: \"Model::Hips\" {\n"
                             "  Indexes: 0,1,2\n"
                             "  Weights: 1,0.5,1\n"
                             "  Transform: 1,0,0,0,0,1,0,0,0,0,1,0,-5,-6,0,1\n"
                             "  TransformLink: 1,0,0,0,0,1,0,0,0,0,1,0,5,6,0,1\n"
                             " }\n"
                             " Link: \"Model::Spine\" {\n"
                             "  Indexes: 1\n"
                             "  Weights: 0.5\n"
                             "  Transform: 1,0,0,0,0,1,0,0,0,0,1,0,-5,-7,0,1\n"
                             "  TransformLink: 1,0,0,0,0,1,0,0,0,0,1,0,5,7,0,1\n"
                             " }\n"
                             "}\n"
                             "Model: \"Model::Hips\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Children: \"Model::Spine\"\n"
                             " Channel: \"Transform\" {\n"
                             "  Channel: \"T\" {\n"
                             "   Channel: \"X\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "   Channel: \"Y\" {\n"
                             "    Default: 2\n"
                             "   }\n"
                             "   Channel: \"Z\" {\n"
                             "    Default: 3\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Model: \"Model::Spine\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Channel: \"Transform\" {\n"
                             "  Channel: \"R\" {\n"
                             "   Channel: \"X\" {\n"
                             "    Default: 0\n"
                             "   }\n"
                             "   Channel: \"Y\" {\n"
                             "    Default: 0\n"
                             "   }\n"
                             "   Channel: \"Z\" {\n"
                             "    Default: 10\n"
                             "    KeyVer: 4004\n"
                             "    KeyCount: 2\n"
                             "    Key: 0,10,L,46186158000,50,L\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"Walk\" {\n"
                             "  LocalTime: 0,46186158000\n"
                             "  Model: \"Model::Hips\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 5\n"
                             "      KeyVer: 4004\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,5,L,46186158000,9,L\n"
                             "     }\n"
                             "     Channel: \"Y\" {\n"
                             "      Default: 6\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Body\" {\n"
                             "   Channel: \"Smile (Shape)\" {\n"
                             "    Default: 0\n"
                             "    KeyVer: 4004\n"
                             "    KeyCount: 2\n"
                             "    Key: 0,0,L,46186158000,100,L\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Settings: {\n"
                             " FrameRate: \"24\"\n"
                             "}\n";

static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_clipFile[65536];
static _Alignas(16) uint8_t s_skinFile[16384];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[16384];

static bool Near(double a, double b)
{
    return fabs(a - b) <= 1e-5;
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void TestSkeleton(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, kScene, strlen(kScene), &fbx);
    CHECK(r.result == mani_success && maniFbxVersion(fbx) == 5000 && maniFbxModelCount(fbx) == 3,
          "opens with three models");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    bool built =
        fbx != NULL && maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success;
    uint32_t hips = built ? maniSkeletonFindBone(&skeleton, "Hips", 4) : MANI_NO_BONE;
    uint32_t spine = built ? maniSkeletonFindBone(&skeleton, "Spine", 5) : MANI_NO_BONE;
    CHECK(hips != MANI_NO_BONE && spine != MANI_NO_BONE &&
              maniSkeletonParent(&skeleton, spine) == hips,
          "Children make the hierarchy");
    maniTransform h = built ? maniSkeletonRest(&skeleton, hips) : (maniTransform){0};
    maniTransform s = built ? maniSkeletonRest(&skeleton, spine) : (maniTransform){0};
    double half = 5.0 * 3.14159265358979323846 / 180.0;
    CHECK(built && Near((double)h.translation.x, 5) && Near((double)h.translation.y, 6) &&
              Near((double)h.translation.z, 0) && Near((double)s.rotation.z, sin(half)) &&
              Near((double)s.rotation.w, cos(half)),
          "rests from the last channel driving them");
    maniFbxSettings settings;
    CHECK(fbx != NULL && maniFbxReadSettings(fbx, &settings) == mani_success &&
              settings.timeMode == 14 && settings.customRate == 24.0,
          "the legacy frame rate");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// What ufbx evaluates: a stack's Hips translation and Spine's turn about
// z (as the quaternion's z and w) at a time.
typedef struct Row
{
    uint32_t stack;
    float time;
    double hips[3];
    double spine[2];
} Row;

static const Row kRows[] = {
    {0, 0.0f, {5, 6, 0}, {0.0871557, 0.996195}}, {0, 0.5f, {7, 6, 0}, {0.0871557, 0.996195}},
    {0, 1.0f, {9, 6, 0}, {0.0871557, 0.996195}}, {1, 0.0f, {1, 2, 3}, {0.0871557, 0.996195}},
    {1, 0.5f, {1, 2, 3}, {0.258819, 0.965926}},  {1, 1.0f, {1, 2, 3}, {0.422618, 0.906308}},
};

static bool RowMatches(const maniClip* clip, const maniSkeleton* skeleton, const Row* row)
{
    maniPose* pose = NULL;
    if (maniInitPose(skeleton, s_poseMemory, sizeof s_poseMemory, &pose) != mani_success ||
        maniSampleClip(clip, skeleton, row->time, pose) != mani_success)
    {
        return false;
    }
    maniTransform h = maniPoseLocal(pose, maniSkeletonFindBone(skeleton, "Hips", 4));
    maniTransform s = maniPoseLocal(pose, maniSkeletonFindBone(skeleton, "Spine", 5));
    double dot = (double)s.rotation.z * row->spine[0] + (double)s.rotation.w * row->spine[1];
    return Near((double)h.translation.x, row->hips[0]) &&
           Near((double)h.translation.y, row->hips[1]) &&
           Near((double)h.translation.z, row->hips[2]) && Near(fabs(dot), 1.0);
}

static const char* const kNames[2] = {"Walk", "(internal)"};

// Whether the clips are these, by name.
static bool NamesAre(const maniFbx* fbx, const char* const* names, uint32_t count)
{
    bool named = maniFbxClipCount(fbx) == count;
    for (uint32_t k = 0; named && k < count; ++k)
    {
        uint32_t length = 0;
        const char* name = maniFbxClipName(fbx, k, &length);
        named = name != NULL && length == strlen(names[k]) && memcmp(name, names[k], length) == 0;
    }
    return named;
}

// The take, then the models' own channels as "(internal)", each over its
// keys' span where it gives no times.
static void TestStacks(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success, "opens");
    bool named = fbx != NULL && NamesAre(fbx, kNames, 2);
    CHECK(named, "the take, then (internal)");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    bool built =
        fbx != NULL && maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success;
    for (uint32_t k = 0; built && k < 2; ++k)
    {
        maniClip clip;
        bool rows =
            maniFbxBuildClip(builder, fbx, k, &skeleton, &options, 0.0f, maniLoopClamp, NULL)
                    .result == mani_success &&
            maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
            maniOpenClip(s_clipFile, size, &clip).result == mani_success &&
            Near((double)maniClipDuration(&clip), 1.0);
        for (size_t r = 0; rows && r < sizeof kRows / sizeof kRows[0]; ++r)
        {
            rows = kRows[r].stack != k || RowMatches(&clip, &skeleton, &kRows[r]);
        }
        CHECK(rows, kNames[k]);
    }
    CHECK(built, "a skeleton");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// A translation-only inverse bind.
static bool IsBind(const maniAffine* a, float x, float y, float z)
{
    const float expected[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, x, y, z};
    for (int i = 0; i < 12; ++i)
    {
        if (!(fabsf(a->m[i] - expected[i]) <= 1e-6f))
        {
            return false;
        }
    }
    return true;
}

// Body's links are its skin's clusters, in order, each bind its
// Transform (as ufbx's geometry to bone); Walk's Smile channel (its
// suffix cut) drives Body's Smile weight from 0 to 1.
static void TestSkinAndShape(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    bool built =
        maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success &&
        maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success;
    maniSkin skin;
    bool skinned = built && maniFbxSkinCount(fbx) == 1 &&
                   maniFbxBuildSkin(builder, fbx, 0, &skeleton, &options).result == mani_success &&
                   maniCopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) == mani_success &&
                   maniOpenSkin(s_skinFile, size, &skin).result == mani_success;
    CHECK(skinned && maniSkinJointCount(&skin) == 2 &&
              maniSkinJoint(&skin, 0) == maniSkeletonFindBone(&skeleton, "Hips", 4) &&
              maniSkinJoint(&skin, 1) == maniSkeletonFindBone(&skeleton, "Spine", 5),
          "a skin of the links, in order");
    maniAffine b0 = skinned ? maniSkinInverseBind(&skin, 0) : (maniAffine){{0}};
    maniAffine b1 = skinned ? maniSkinInverseBind(&skin, 1) : (maniAffine){{0}};
    CHECK(IsBind(&b0, -5, -6, 0) && IsBind(&b1, -5, -7, 0), "binds from Transform");
    maniClip clip;
    bool clipped =
        built &&
        maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, 0.0f, maniLoopClamp, NULL).result ==
            mani_success &&
        maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
        maniOpenClip(s_clipFile, size, &clip).result == mani_success;
    float value = -1.0f;
    CHECK(clipped && maniClipFindFloatTrack(&clip, "Body/Smile", 10) != MANI_NO_TRACK &&
              maniClipFloatTrackCount(&clip) == 1 &&
              maniSampleClipFloats(&clip, 0.5f, &value, 1, NULL) == mani_success &&
              Near((double)value, 0.5),
          "the shape's weight, with no property of its name");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// A take channel's " (Shape)" suffix is cut; a name that only is the
// suffix, or lacks it, stays whole.
static void TestShapeNames(void)
{
    static const char kText[] = "; FBX 5.0.0 project file\nModel: \"Model::Face\" {\n}\n"
                                "Takes: {\n Take: \"Talk\" {\n  Model: \"Model::Face\" {\n"
                                "   Channel: \"Smile (Shape)\" {\n    Default: 1\n   }\n"
                                "   Channel: \" (Shape)\" {\n    Default: 1\n   }\n"
                                "   Channel: \"Frown\" {\n    Default: 1\n   }\n  }\n }\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxAnimation a;
    memset(&a, 0, sizeof a);
    bool read = maniOpenFbx(builder, kText, strlen(kText), &fbx).result == mani_success &&
                maniFbxReadStack(fbx, 0, &a).result == mani_success && a.channelCount == 3;
    bool smile = false;
    bool suffix = false;
    bool frown = false;
    for (uint32_t i = 0; read && i < a.channelCount; ++i)
    {
        const maniFbxChannel* c = &a.channels[i];
        smile = smile || (maniFbxChannelNameIs(fbx, c, "Smile", 5) &&
                          c->nameHash == maniFbxNameHash("Smile", 5));
        suffix = suffix || maniFbxChannelNameIs(fbx, c, " (Shape)", 8);
        frown = frown || maniFbxChannelNameIs(fbx, c, "Frown", 5);
    }
    CHECK(read && smile && suffix && frown, "the suffix cut where it ends a longer name");
    if (fbx != NULL)
    {
        maniFbxReleaseAnimation(fbx, &a);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// The settings of a text.
static bool Settings(const char* text, maniFbxSettings* out)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool read = maniOpenFbx(builder, text, strlen(text), &fbx).result == mani_success &&
                maniFbxReadSettings(fbx, out) == mani_success;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return read;
}

static void TestVersion5Settings(void)
{
    // A 6.x file's Version5 Settings, where GlobalSettings has no rate.
    maniFbxSettings s;
    CHECK(Settings("; FBX 6.1.0 project file\nVersion5: {\n Settings: {\n  FrameRate: \"25\"\n"
                   " }\n}\n",
                   &s) &&
              s.timeMode == 14 && s.customRate == 25.0,
          "Version5's frame rate");
    CHECK(Settings("; FBX 6.1.0 project file\nGlobalSettings: {\n Properties60: {\n"
                   "  Property: \"TimeMode\", \"enum\", \"\",11\n }\n}\nVersion5: {\n"
                   " Settings: {\n  FrameRate: 25\n }\n}\n",
                   &s) &&
              s.timeMode == 11 && s.rateNumerator == 24,
          "GlobalSettings' rate first");
}

// Each text refused at open (Property).
static void Refused(const char* text, const char* label)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, text, strlen(text), &fbx);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty, label);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    Refused("; FBX 5.0.0 project file\nModel: {\n}\n", "a model without a name");
    Refused("; FBX 5.0.0 project file\nModel: \"Model::Mesh\" {\n Vertices: 0,0,0\n"
            " PolygonVertexIndex: -1\n Link: {\n }\n}\n",
            "a link without a name");
    Refused("; FBX 5.0.0 project file\nModel: \"Model::A\" {\n}\nModel: \"Model::A\" {\n}\n",
            "a name given twice");
}

static void TestAllocationFailures(void)
{
    // Walk with the skin, then (internal).
    FbxSweep walk = {kScene, 0, 0};
    FbxSweep internal = {kScene, 1, UINT32_MAX};
    CHECK(TestSweep(FbxSweepWork, &walk, 4096) && TestSweep(FbxSweepWork, &internal, 4096),
          "allocation failures handled, nothing kept");
}

int main(void)
{
    TestSkeleton();
    TestStacks();
    TestSkinAndShape();
    TestShapeNames();
    TestVersion5Settings();
    TestRefusals();
    TestAllocationFailures();
    return s_failures == 0 ? 0 : 1;
}
