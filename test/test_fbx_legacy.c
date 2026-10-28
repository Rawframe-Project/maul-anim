// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX files before 6.0: top-level models and their Children,
// rest values from the last channel driving them in file order, the
// legacy frame rate, against what ufbx gives for the same file.

#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_object.h"
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
#include <stdlib.h>
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
                             "LocalTime: 0,92372316000\n"
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

// Fresh memory filled with a pattern, so nothing read before it is
// written passes for zero.
static void* PoisonAlloc(size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    void* p = malloc(size);
    if (p != NULL)
    {
        memset(p, 0xA5, size);
    }
    return p;
}

static void PoisonFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    free(memory);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    def.allocator = (maniAllocator){PoisonAlloc, PoisonFree, NULL};
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
        size_t length = 0;
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
        bool rows = maniFbxBuildClip(
                        builder, fbx, k, &skeleton, &options,
                        &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, NULL)
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
        maniFbxBuildClip(builder, fbx, 0, &skeleton, &options,
                         &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, NULL)
                .result == mani_success &&
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
    CHECK(fbx != NULL && maniFbxClipCount(fbx) == 1, "no (internal) without model channels");
    if (fbx != NULL)
    {
        maniFbxReleaseAnimation(fbx, &a);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Root (Type LimbNode, Children A and B) turned 30 about z with T 1 2 3
// from its own channels, and a Note child of Transform named T that is
// no channel; A's rests from Lcl channels by name, its scale from one
// Default (2 0 0, as ufbx reads it); B has none. Idle's empty T channel
// on Root leaves Root's rest, and its Comment node, not a Model, sets
// nothing, nor does B's Extra channel with a T child. Settings give 25
// frames a second, as a number.
static const char kRests[] = "; FBX 5.0.0 project file\n"
                             "Model: \"Model::Root\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Children: \"Model::A\", \"Model::B\"\n"
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
                             "  Channel: \"R\" {\n"
                             "   Channel: \"X\" {\n"
                             "    Default: 0\n"
                             "   }\n"
                             "   Channel: \"Y\" {\n"
                             "    Default: 0\n"
                             "   }\n"
                             "   Channel: \"Z\" {\n"
                             "    Default: 30\n"
                             "   }\n"
                             "  }\n"
                             "  Note: \"T\" {\n"
                             "   Default: 9\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Model: \"Model::A\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Channel: \"Lcl Translation\" {\n"
                             "  Channel: \"X\" {\n"
                             "   Default: 4\n"
                             "  }\n"
                             "  Channel: \"Y\" {\n"
                             "   Default: 5\n"
                             "  }\n"
                             "  Channel: \"Z\" {\n"
                             "   Default: 6\n"
                             "  }\n"
                             " }\n"
                             " Channel: \"Lcl Scaling\" {\n"
                             "  Default: 2\n"
                             " }\n"
                             "}\n"
                             "Model: \"Model::B\" {\n"
                             " Type: \"LimbNode\"\n"
                             " Channel: \"Extra\" {\n"
                             "  Channel: \"T\" {\n"
                             "   Channel: \"X\" {\n"
                             "    Default: 8\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"Idle\" {\n"
                             "  Model: \"Model::Root\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             "  Comment: \"Model::B\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 7\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Settings: {\n"
                             " FrameRate: 25\n"
                             "}\n";

// Whether object o's rest which is (x, y, z).
static bool RestIs(const maniFbx* fbx, uint32_t o, uint32_t which, double x, double y, double z)
{
    double v[3];
    return maniFbxLegacyRest(fbx, o, which, v) && v[0] == x && v[1] == y && v[2] == z;
}

static void TestRests(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    double v[3];
    bool open = maniOpenFbx(builder, kRests, strlen(kRests), &fbx).result == mani_success;
    CHECK(open && RestIs(fbx, 0, 0, 1, 2, 3) && RestIs(fbx, 0, 1, 0, 0, 30) &&
              !maniFbxLegacyRest(fbx, 0, 2, v),
          "Transform's T and R on one model, an empty channel and a Note ignored");
    CHECK(open && RestIs(fbx, 1, 0, 4, 5, 6) && RestIs(fbx, 1, 2, 2, 0, 0) &&
              !maniFbxLegacyRest(fbx, 1, 1, v),
          "Lcl channels by name, one Default a single component");
    CHECK(open && !maniFbxLegacyRest(fbx, 2, 0, v), "no rest from a take node not a Model");
    maniFbxSettings settings;
    CHECK(open && maniFbxReadSettings(fbx, &settings) == mani_success &&
              settings.customRate == 25.0,
          "a numeric FrameRate");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    bool built =
        open && maniFbxSkeletonCount(fbx) == 1 &&
        maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success;
    uint32_t root = built ? maniSkeletonFindBone(&skeleton, "Root", 4) : MANI_NO_BONE;
    CHECK(built && root != MANI_NO_BONE &&
              maniSkeletonParent(&skeleton, maniSkeletonFindBone(&skeleton, "A", 1)) == root &&
              maniSkeletonParent(&skeleton, maniSkeletonFindBone(&skeleton, "B", 1)) == root,
          "a limb chain by Type, every child of the list");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// A legacy skin and its clusters answer as a Skin and Cluster deformer,
// by whole names only.
static void TestRoles(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool open = maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success;
    uint32_t skins = 0;
    uint32_t clusters = 0;
    bool right = open;
    for (uint32_t o = 0; right && o < fbx->objectCount; ++o)
    {
        uint32_t role = fbx->objects[o].role;
        bool deformer = maniFbxObjectKindIs(fbx, o, "Deformer", 8);
        right =
            deformer == (role != maniFbxRoleNone) &&
            !(role != maniFbxRoleNone && maniFbxObjectKindIs(fbx, o, "Geometry", 8)) &&
            maniFbxObjectSubclassIs(fbx, o, "Skin", 4) == (role == maniFbxRoleLegacySkin) &&
            maniFbxObjectSubclassIs(fbx, o, "Cluster", 7) == (role == maniFbxRoleLegacyCluster) &&
            !maniFbxObjectSubclassIs(fbx, o, "Sk", 2);
        skins += role == maniFbxRoleLegacySkin ? 1 : 0;
        clusters += role == maniFbxRoleLegacyCluster ? 1 : 0;
    }
    CHECK(right && skins == 1 && clusters == 2, "one skin, two clusters, by role");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Which models' Links make a skin: a mesh's, with Vertices and
// PolygonVertexIndex both, not a model with Vertices alone or none. A
// Link may name the scene (it links nothing), and an empty Link before
// another mesh's skin keeps their ids apart. Which models have an
// attribute: a LimbNode, Light or Camera, or one with Vertices; not a
// Null.
static void TestLinks(void)
{
    static const char kText[] =
        "; FBX 5.0.0 project file\nModel: \"Model::Bone\" {\n Type: \"LimbNode\"\n}\n"
        "Model: \"Model::Points\" {\n Vertices: 0,0,0\n Link: \"Model::Bone\" {\n }\n}\n"
        "Model: \"Model::Group\" {\n Link: \"Model::Bone\" {\n }\n}\n"
        "Model: \"Model::Mesh\" {\n Vertices: 0,0,0\n PolygonVertexIndex: -1\n"
        " MaterialAssignation: \"AllSame\"\n Link: \"Model::Scene\"\n"
        " Link: \"Model::Bone\"\n}\n"
        "Model: \"Model::Next\" {\n Vertices: 0,0,0\n PolygonVertexIndex: -1\n"
        " MaterialAssignation: \"AllSame\"\n Link: \"Model::Bone\" {\n }\n}\n"
        "Model: \"Model::Lamp\" {\n Type: \"Light\"\n}\n"
        "Model: \"Model::Eye\" {\n Type: \"Camera\"\n}\n"
        "Model: \"Model::Empty\" {\n Type: \"Null\"\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool open = maniOpenFbx(builder, kText, strlen(kText), &fbx).result == mani_success;
    CHECK(open && maniFbxSkinCount(fbx) == 2, "skins of meshes only");
    // Objects: Bone, Points, Group, Mesh (a skin, two clusters), Next (a
    // skin, a cluster), Lamp, Eye, Empty.
    CHECK(open && fbx->objectCount == 13, "the objects");
    CHECK(open && maniFbxModelHasAttribute(fbx, 0) && maniFbxModelHasAttribute(fbx, 1) &&
              !maniFbxModelIsMesh(fbx, 1) && !maniFbxModelHasAttribute(fbx, 2) &&
              maniFbxModelIsMesh(fbx, 3) && maniFbxModelHasAttribute(fbx, 10) &&
              maniFbxModelHasAttribute(fbx, 11) && !maniFbxModelHasAttribute(fbx, 12),
          "attributes by Type or Vertices, meshes with PolygonVertexIndex too");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// A 6.x take keeps " (Shape)" and has no (internal) stack from a stray
// top-level model's channels.
static void TestSixKeepsSuffix(void)
{
    static const char kText[] =
        "; FBX 6.1.0 project file\nModel: \"Model::Stray\" {\n Channel: \"Transform\" {\n"
        " }\n}\nObjects: {\n Model: \"Model::Face\", \"Mesh\" {\n }\n}\n"
        "Takes: {\n Take: \"Talk\" {\n  Model: \"Model::Face\" {\n"
        "   Channel: \"Smile (Shape)\" {\n    Default: 1\n   }\n  }\n }\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxAnimation a;
    memset(&a, 0, sizeof a);
    bool read = maniOpenFbx(builder, kText, strlen(kText), &fbx).result == mani_success &&
                maniFbxClipCount(fbx) == 1 && maniFbxReadStack(fbx, 0, &a).result == mani_success &&
                a.channelCount == 1;
    CHECK(read && maniFbxChannelNameIs(fbx, &a.channels[0], "Smile (Shape)", 13),
          "the suffix kept from 6.0 on, one stack");
    if (fbx != NULL)
    {
        maniFbxReleaseAnimation(fbx, &a);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// A name given twice is kept by its first model (as ufbx keeps it);
// Children naming it reach that one only.
static void TestNameTwice(void)
{
    static const char kText[] = "; FBX 5.0.0 project file\nModel: \"Model::A\" {\n"
                                " Children: \"Model::B\"\n}\nModel: \"Model::B\" {\n}\n"
                                "Model: \"Model::B\" {\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    bool open = maniOpenFbx(builder, kText, strlen(kText), &fbx).result == mani_success;
    CHECK(open && fbx->objectCount == 3 && fbx->duplicateObjects == 1 &&
              fbx->modelParents[1] == 0 && fbx->modelParents[2] == MANI_FBX_NONE,
          "a name twice: the first model has the parent, the later none");
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
    CHECK(Settings("; FBX 6.1.0 project file\nGlobalSettings: {\n Properties60: {\n"
                   "  Property: \"CustomFrameRate\", \"double\", \"\",50\n }\n}\nVersion5: {\n"
                   " Settings: {\n  FrameRate: 25\n }\n}\n",
                   &s) &&
              s.timeMode == 14 && s.customRate == 50.0,
          "GlobalSettings' custom rate first");
    CHECK(Settings("; FBX 6.1.0 project file\nGlobalSettings: {\n Properties60: {\n"
                   "  Property: \"TimeMode\", \"enum\", \"\",14\n }\n}\nVersion5: {\n"
                   " Settings: {\n  FrameRate: 25\n }\n}\n",
                   &s) &&
              s.timeMode == 14 && s.customRate == 25.0,
          "Version5's rate for a custom mode with none");
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
    Refused("; FBX 5.0.0 project file\nModel: \"Model::Mesh\" {\n Vertices: 0,0,0\n"
            " PolygonVertexIndex: -1\n Link: {\n }\n}\nModel: \"Model::Other\" {\n}\n",
            "a link without a name, a model after it");
    Refused("; FBX 5.0.0 project file\nModel: \"Model::A\" {\n Children: \"Model::B\", B\n}\n",
            "a bare word among the children");
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
    TestRests();
    TestRoles();
    TestLinks();
    TestSixKeepsSuffix();
    TestNameTwice();
    TestShapeNames();
    TestVersion5Settings();
    TestRefusals();
    TestAllocationFailures();
    return s_failures == 0 ? 0 : 1;
}
