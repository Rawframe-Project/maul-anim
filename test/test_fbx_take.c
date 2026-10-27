// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX 6.x takes as clips: stacks named by takes, their
// ranges, Transform channels on the Lcl properties, components missing
// their Default at 0, a user property as a float track and an
// attribute's property as none, against ufbx's evaluated transforms of
// the same file; which properties are the attribute's; refusals.

#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_take.h"
#include "test_fbx_sweep.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Hips (a limb, y 1) above Spine (y 1, rotation order ZXY). Walk moves
// Hips in x and turns it about z, turns Spine about y from its rest of
// 30 and 10 and scales it in x; it drives Hips' Custom (user-defined)
// and Size (its attribute's) and a model the file lacks. Idle has only
// a range; Still sets Hips' x alone, so its y and z are 0.
static const char kScene[] = "; FBX 6.1.0 project file\n"
                             "FBXHeaderExtension: {\n"
                             " FBXHeaderVersion: 1003\n"
                             " FBXVersion: 6100\n"
                             "}\n"
                             "Objects: {\n"
                             " Model: \"Model::Hips\", \"Limb\" {\n"
                             "  Version: 232\n"
                             "  Properties60: {\n"
                             "   Property: \"Lcl Translation\", \"Lcl Translation\", \"A+\",0,1,0\n"
                             "   Property: \"Custom\", \"Number\", \"A+U\",2\n"
                             "   Property: \"Size\", \"Number\", \"A+\",100\n"
                             "  }\n"
                             " }\n"
                             " Model: \"Model::Spine\", \"Limb\" {\n"
                             "  Version: 232\n"
                             "  Properties60: {\n"
                             "   Property: \"Lcl Translation\", \"Lcl Translation\", \"A+\",0,1,0\n"
                             "   Property: \"RotationOrder\", \"enum\", \"\",4\n"
                             "  }\n"
                             " }\n"
                             "}\n"
                             "Connections: {\n"
                             " Connect: \"OO\", \"Model::Hips\", \"Model::Scene\"\n"
                             " Connect: \"OO\", \"Model::Spine\", \"Model::Hips\"\n"
                             "}\n"
                             "Takes: {\n"
                             " Current: \"Walk\"\n"
                             " Take: \"Walk\" {\n"
                             "  FileName: \"Walk.tak\"\n"
                             "  LocalTime: 0,46186158000\n"
                             "  ReferenceTime: 0,46186158000\n"
                             "  Model: \"Model::Hips\" {\n"
                             "   Version: 1.1\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,0,L,46186158000,3,L\n"
                             "     }\n"
                             "     Channel: \"Y\" {\n"
                             "      Default: 1\n"
                             "     }\n"
                             "     Channel: \"Z\" {\n"
                             "      Default: 0\n"
                             "     }\n"
                             "     LayerType: 1\n"
                             "    }\n"
                             "    Channel: \"R\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 0\n"
                             "     }\n"
                             "     Channel: \"Y\" {\n"
                             "      Default: 0\n"
                             "     }\n"
                             "     Channel: \"Z\" {\n"
                             "      Default: 0\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,0,U,s,0,0,n,46186158000,90,U,s,0,0,n\n"
                             "     }\n"
                             "     LayerType: 2\n"
                             "    }\n"
                             "   }\n"
                             "   Channel: \"Custom\" {\n"
                             "    Default: 2\n"
                             "    KeyVer: 4005\n"
                             "    KeyCount: 2\n"
                             "    Key: 0,2,L,46186158000,4,L\n"
                             "   }\n"
                             "   Channel: \"Size\" {\n"
                             "    Default: 100\n"
                             "    KeyVer: 4005\n"
                             "    KeyCount: 2\n"
                             "    Key: 0,100,L,46186158000,50,L\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Spine\" {\n"
                             "   Version: 1.1\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"R\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 30\n"
                             "     }\n"
                             "     Channel: \"Y\" {\n"
                             "      Default: 0\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,0,L,46186158000,60,L\n"
                             "     }\n"
                             "     Channel: \"Z\" {\n"
                             "      Default: 10\n"
                             "     }\n"
                             "    }\n"
                             "    Channel: \"S\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 1\n"
                             "      KeyVer: 4005\n"
                             "      KeyCount: 2\n"
                             "      Key: 0,1,L,46186158000,2,L\n"
                             "     }\n"
                             "     Channel: \"Y\" {\n"
                             "      Default: 1\n"
                             "     }\n"
                             "     Channel: \"Z\" {\n"
                             "      Default: 0.5\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Missing\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 7\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             " Take: \"Idle\" {\n"
                             "  LocalTime: 0,23093079000\n"
                             " }\n"
                             " Take: \"Still\" {\n"
                             "  Model: \"Model::Hips\" {\n"
                             "   Channel: \"Transform\" {\n"
                             "    Channel: \"T\" {\n"
                             "     Channel: \"X\" {\n"
                             "      Default: 5\n"
                             "     }\n"
                             "    }\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             "}\n";

typedef struct Row
{
    uint32_t stack;
    const char* bone;
    float time;
    double trs[10];
    double custom;
} Row;

static const Row kRows[] = {
    {0, "Hips", 0.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {0,
     "Spine",
     0.0f,
     {0.0, 1.0, 0.0, 0.25783416, -0.0225575661, 0.0841859828, 0.962250187, 1.0, 1.0, 0.5},
     0.0},
    {0, "Hips", 0.25f, {0.75, 1.0, 0.0, 0.0, 0.0, 0.122410761, 0.992479524, 1.0, 1.0, 1.0}, 2.5},
    {0,
     "Spine",
     0.25f,
     {0.0, 1.0, 0.0, 0.266616829, 0.10323427, 0.0498116489, 0.956962356, 1.25, 1.0, 0.5},
     0.0},
    {0, "Hips", 0.5f, {1.5, 1.0, 0.0, 0.0, 0.0, 0.382683432, 0.923879533, 1.0, 1.0, 1.0}, 3.0},
    {0,
     "Spine",
     0.5f,
     {0.0, 1.0, 0.0, 0.27083761, 0.227259739, 0.0145850238, 0.935300635, 1.5, 1.0, 0.5},
     0.0},
    {0, "Hips", 0.75f, {2.25, 1.0, 0.0, 0.0, 0.0, 0.615231522, 0.788346481, 1.0, 1.0, 1.0}, 3.5},
    {0,
     "Spine",
     0.75f,
     {0.0, 1.0, 0.0, 0.270424285, 0.347396731, -0.0208911551, 0.89763566, 1.75, 1.0, 0.5},
     0.0},
    {0, "Hips", 1.0f, {3.0, 1.0, 0.0, 0.0, 0.0, 0.707106781, 0.707106781, 1.0, 1.0, 1.0}, 4.0},
    {0,
     "Spine",
     1.0f,
     {0.0, 1.0, 0.0, 0.265383924, 0.461589668, -0.0560098805, 0.84461189, 2.0, 1.0, 0.5},
     0.0},
    {1, "Hips", 0.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {1, "Spine", 0.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {1, "Hips", 0.25f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {1, "Spine", 0.25f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {1, "Hips", 0.5f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {1, "Spine", 0.5f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {1, "Hips", 0.75f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {1, "Spine", 0.75f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {1, "Hips", 1.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {1, "Spine", 1.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {2, "Hips", 0.0f, {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {2, "Spine", 0.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {2, "Hips", 0.25f, {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {2, "Spine", 0.25f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {2, "Hips", 0.5f, {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {2, "Spine", 0.5f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {2, "Hips", 0.75f, {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {2, "Spine", 0.75f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
    {2, "Hips", 1.0f, {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 2.0},
    {2, "Spine", 1.0f, {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0}, 0.0},
};

static const char* const kStackNames[] = {"Walk", "Idle", "Still"};
static const float kDurations[] = {1.0f, 0.5f, 0.0f};

static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_clipFile[65536];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[16384];

static bool Near(double a, double b)
{
    return fabs(a - b) <= 2e-5;
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static bool Build(maniBuilder* builder, const maniFbx* fbx, uint32_t stack, maniSkeleton* skeleton,
                  maniClip* clip)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    size_t size = 0;
    return maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, skeleton).result == mani_success &&
           maniFbxBuildClip(builder, fbx, stack, skeleton, &options,
                            &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                            NULL)
                   .result == mani_success &&
           maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
           maniOpenClip(s_clipFile, size, clip).result == mani_success;
}

// q and p the same rotation (either sign).
static bool SameTurn(maniQuat q, const double* p)
{
    double dot = (double)q.x * p[0] + (double)q.y * p[1] + (double)q.z * p[2] + (double)q.w * p[3];
    return fabs(fabs(dot) - 1.0) <= 2e-5;
}

static bool RowMatches(const maniClip* clip, const maniSkeleton* skeleton, const Row* row)
{
    maniPose* pose = NULL;
    if (maniInitPose(skeleton, s_poseMemory, sizeof s_poseMemory, &pose) != mani_success ||
        maniSampleClip(clip, skeleton, row->time, pose) != mani_success)
    {
        return false;
    }
    uint32_t bone = maniSkeletonFindBone(skeleton, row->bone, strlen(row->bone));
    maniTransform t = maniPoseLocal(pose, bone);
    const double* v = row->trs;
    bool right = bone != MANI_NO_BONE && Near((double)t.translation.x, v[0]) &&
                 Near((double)t.translation.y, v[1]) && Near((double)t.translation.z, v[2]) &&
                 SameTurn(t.rotation, v + 3) && Near((double)t.scale.x, v[7]) &&
                 Near((double)t.scale.y, v[8]) && Near((double)t.scale.z, v[9]);
    if (!right)
    {
        printf("row %s at %g: t %g %g %g s %g %g %g\n", row->bone, (double)row->time,
               (double)t.translation.x, (double)t.translation.y, (double)t.translation.z,
               (double)t.scale.x, (double)t.scale.y, (double)t.scale.z);
    }
    return right;
}

static void TestStacks(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success, "opens");
    bool right = fbx != NULL && maniFbxClipCount(fbx) == 3;
    for (uint32_t k = 0; right && k < 3; ++k)
    {
        uint32_t length = 0;
        const char* name = maniFbxClipName(fbx, k, &length);
        right = name != NULL && length == strlen(kStackNames[k]) &&
                memcmp(name, kStackNames[k], length) == 0;
    }
    CHECK(right, "takes are clips, by name");
    for (uint32_t k = 0; fbx != NULL && k < 3; ++k)
    {
        maniSkeleton skeleton;
        maniClip clip;
        bool built = Build(builder, fbx, k, &skeleton, &clip);
        bool rows = built && fabs((double)maniClipDuration(&clip) - (double)kDurations[k]) <= 1e-6;
        for (size_t r = 0; rows && r < sizeof kRows / sizeof kRows[0]; ++r)
        {
            bool inside = (double)kRows[r].time <= (double)kDurations[k] + 1e-9;
            rows = kRows[r].stack != k || !inside || RowMatches(&clip, &skeleton, &kRows[r]);
        }
        CHECK(rows, kStackNames[k]);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Walk's float tracks: Hips' user property; not its attribute's Size.
static void TestTracks(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniSkeleton skeleton;
    maniClip clip;
    bool built = maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success &&
                 Build(builder, fbx, 0, &skeleton, &clip);
    // Bound: Hips' T, R and Custom, Spine's R and S; not Size.
    maniFbxOptions options = maniDefaultFbxOptions();
    uint32_t bound = 0;
    CHECK(built &&
              maniFbxBuildClip(builder, fbx, 0, &skeleton, &options,
                               &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                               &bound)
                      .result == mani_success &&
              bound == 5,
          "the model channels bound, not an attribute's");
    float value = 0.0f;
    uint32_t track = built ? maniClipFindFloatTrack(&clip, "Hips/Custom", 11) : MANI_NO_TRACK;
    CHECK(built && maniClipFloatTrackCount(&clip) == 1 && track == 0 &&
              maniSampleClipFloats(&clip, 0.5f, &value, 1, NULL) == mani_success &&
              Near((double)value, 3.0),
          "a user property is a track, an attribute's is not");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Which channels drive the attribute: Limb's Size (neither a node
// property nor user-defined) and Absent (a property it lacks), not its
// Visibility or Custom (U); a plain model (no subclass, or Model) and an
// object not a model keep all; a model with mesh arrays and no subclass
// has one. Limb's Tint takes its components from its named children
// holding a value; an empty channel, a node not a Channel and the scene
// root bind nothing. Ref has only a ReferenceTime.
static const char kSplit[] = "; FBX 6.1.0 project file\n"
                             "Objects: {\n"
                             " Model: \"Model::Limb\", \"Limb\" {\n"
                             "  Properties60: {\n"
                             "   Property: \"Custom\", \"Number\", \"A+U\",1\n"
                             "   Property: \"Size\", \"Number\", \"A+\",100\n"
                             "  }\n"
                             " }\n"
                             " Model: \"Model::Plain\", \"\" {\n"
                             " }\n"
                             " Model: \"Model::Shape\", \"\" {\n"
                             "  Vertices: 0,0,0\n"
                             "  PolygonVertexIndex: 0\n"
                             " }\n"
                             " Model: \"Model::Typed\", \"Model\" {\n"
                             " }\n"
                             " Deformer: \"Model::Odd\", \"Skin\" {\n"
                             " }\n"
                             "}\n"
                             "Takes: {\n"
                             " Take: \"T\" {\n"
                             "  Model: \"Model::Limb\" {\n"
                             "   Channel: \"Size\" {\n"
                             "    Default: 100\n"
                             "    KeyCount: 2\n"
                             "    Key: 0,100,L,46186158000,50,L\n"
                             "   }\n"
                             "   Channel: \"Visibility\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "   Channel: \"Custom\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "   Channel: \"Absent\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Plain\" {\n"
                             "   Channel: \"Size\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Shape\" {\n"
                             "   Channel: \"Size\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Typed\" {\n"
                             "   Channel: \"Size\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Odd\" {\n"
                             "   Channel: \"Size\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Scene\" {\n"
                             "   Channel: \"Visibility\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             "  Model: \"Model::Limb\" {\n"
                             "   Channel: \"Tint\" {\n"
                             "    Channel: \"A\" {\n"
                             "     Color: 1,1,1\n"
                             "    }\n"
                             "    Channel: {\n"
                             "     Default: 9\n"
                             "    }\n"
                             "    Channel: \"X\" {\n"
                             "     Default: 1\n"
                             "    }\n"
                             "    Channel: \"Y\" {\n"
                             "     Default: 2\n"
                             "    }\n"
                             "    Channel: \"Z\" {\n"
                             "     Default: 3\n"
                             "    }\n"
                             "   }\n"
                             "   Channel: \"Empty\" {\n"
                             "   }\n"
                             "   Other: \"Size2\" {\n"
                             "    Default: 1\n"
                             "   }\n"
                             "  }\n"
                             " }\n"
                             " Take: \"Ref\" {\n"
                             "  ReferenceTime: 0,23093079000\n"
                             " }\n"
                             "}\n";

// The channel of a target's property, or NULL.
static const maniFbxChannel* Channel(const maniFbx* fbx, const maniFbxAnimation* a, uint32_t target,
                                     const char* name)
{
    uint32_t first = 0;
    uint32_t n = maniFbxAnimatedRun(a, target, &first);
    for (uint32_t i = first; i < first + n; ++i)
    {
        if (maniFbxChannelNameIs(fbx, &a->channels[i], name, strlen(name)))
        {
            return &a->channels[i];
        }
    }
    return NULL;
}

static void TestSplit(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxAnimation a;
    bool read = maniOpenFbx(builder, kSplit, strlen(kSplit), &fbx).result == mani_success &&
                maniFbxReadStack(fbx, 0, &a).result == mani_success;
    static const struct
    {
        uint32_t model;
        const char* name;
        bool attribute;
    } kWant[8] = {{0, "Size", true},   {0, "Visibility", false}, {0, "Custom", false},
                  {0, "Absent", true}, {1, "Size", false},       {2, "Size", true},
                  {3, "Size", false},  {0, "Tint", true}};
    bool right = read && a.channelCount == 9;
    for (int k = 0; right && k < 8; ++k)
    {
        const maniFbxChannel* c = Channel(fbx, &a, fbx->models[kWant[k].model], kWant[k].name);
        right = c != NULL && c->attribute == kWant[k].attribute;
    }
    // The object keyed Model::Odd, a Skin: its own property.
    for (uint32_t o = 0; right && o < fbx->objectCount; ++o)
    {
        const maniFbxChannel* c = Channel(fbx, &a, o, "Size");
        right = !maniFbxObjectKindIs(fbx, o, "Deformer", 8) || (c != NULL && !c->attribute);
    }
    const maniFbxChannel* tint = read ? Channel(fbx, &a, fbx->models[0], "Tint") : NULL;
    right = right && tint != NULL && tint->values[0] == 1.0 && tint->values[1] == 2.0 &&
            tint->values[2] == 3.0;
    CHECK(right, "a 6.x model's properties split from its attribute's");
    double v[3] = {0, 0, 0};
    uint32_t limb = read ? fbx->models[0] : 0;
    CHECK(read && !maniFbxAnimateProperty(&a, fbx, limb, "Size", 4, 0.5, 0, v) &&
              maniFbxAnimateAttribute(&a, fbx, limb, "Size", 4, 0.5, v) && Near(v[0], 75.0),
          "an attribute's property animates apart from the model's");
    if (read)
    {
        maniFbxReleaseAnimation(fbx, &a);
    }
    CHECK(read && maniFbxReadStack(fbx, 1, &a).result == mani_success && a.start == 0 &&
              a.stop == 23093079000,
          "a range from ReferenceTime alone");
    if (read)
    {
        maniFbxReleaseAnimation(fbx, &a);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Each text refused as a stack; at the node a marker's first byte is in
// (the marker NULL for any).
static void Refused(const char* takes, const char* marker, const char* label)
{
    char text[1024];
    snprintf(text, sizeof text,
             "; FBX 6.1.0 project file\nObjects: {\n Model: \"Model::A\", \"Limb\" {\n }\n}\n"
             "Takes: {\n%s}\n",
             takes);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxAnimation a;
    bool opened = maniOpenFbx(builder, text, strlen(text), &fbx).result == mani_success;
    maniImportResult r =
        opened ? maniFbxReadStack(fbx, 0, &a) : (maniImportResult){mani_success, 0, 0, 0};
    const char* at = marker != NULL ? strstr(text, marker) : NULL;
    CHECK(opened && r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
              (at == NULL || r.offset == (uint64_t)(at - text)),
          label);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    Refused(" Take: {\n }\n", NULL, "a take without a name");
    Refused(" Take: \"T\" {\n  Model: {\n  }\n }\n", NULL, "a model without a name");
    Refused(" Take: \"T\" {\n  Model: \"Model::A\" {\n   Channel: \"Transform\" {\n"
            "    Channel: {\n    }\n   }\n  }\n }\n",
            NULL, "a transform channel without a name");
    Refused(" Take: \"T\" {\n  Model: \"Model::A\" {\n   Channel: \"Transform\" {\n"
            "    Channel: \"T\" {\n     Channel: \"X\" {\n      Default: 1\n     }\n    }\n"
            "   }\n   Channel: \"Transform\" {\n    Channel: \"T\" {\n     Channel: \"X\" {\n"
            "      Default: 2\n     }\n    }\n   }\n  }\n }\n",
            "Channel: \"T\" {\n     Channel: \"X\" {\n      Default: 2", "a property driven twice");
    Refused(" Take: \"T\" {\n  Model: \"Model::A\" {\n   Channel: \"Transform\" {\n"
            "    Channel: \"T\" {\n     Channel: \"X\" {\n      Default: 1\n     }\n    }\n"
            "   }\n   Channel: \"Lcl Translation\" {\n    Default: 2\n   }\n  }\n }\n",
            "Channel: \"Lcl", "T and Lcl Translation both");
    Refused(" Take: \"T\" {\n  Model: \"Model::A\" {\n   Channel: \"Visibility\" {\n"
            "    Key: 0,1,L\n   }\n  }\n }\n",
            NULL, "keys without a count");
}

// Every allocation failing in turn while opening the file and building
// Walk's clip.
static void TestAllocationFailures(void)
{
    FbxSweep sweep = {kScene, 0, UINT32_MAX};
    CHECK(TestSweep(FbxSweepWork, &sweep, 4096), "allocation failures handled, nothing kept");
}

int main(void)
{
    TestAllocationFailures();
    TestStacks();
    TestTracks();
    TestSplit();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
