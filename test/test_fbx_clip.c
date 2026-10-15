// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX clips (mani-0016): a rig sampled at 30 a second, its bones' locals
// against values derived by hand (a translation and rotation by linear
// curves, a uniform scale, and a scale-ignoring child whose local comes
// from the worlds), the placement folded or reported, a float track, the
// channel bindings counted; and refusals.

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

// Rig (Null, z 5, animated to 6) above Hips (y 1, x from 0 to 3), Spine
// (y 1, turning 90 about z and scaling 1 to 2, Glow 0 to 1) and Arm
// (scale not inherited, x 1, turned 45 about z). Prop is no bone. Walk
// runs a second; Idle has nothing; Shear scales Spine in x alone; Inherit
// takes Arm's inherit type past its range.
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Model: 5, \"Model::Rig\", \"Null\" {\n  Properties70: {\n   P: \"Lcl Translation\", \"Lcl "
    "Translation\", \"\", \"A\",0,0,5\n  }\n }\n"
    " Model: 1, \"Model::Hips\", \"LimbNode\" {\n  Properties70: {\n   P: \"Lcl Translation\", "
    "\"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    " Model: 2, \"Model::Spine\", \"LimbNode\" {\n  Properties70: {\n   P: \"Lcl Translation\", "
    "\"Lcl Translation\", \"\", \"A\",0,1,0\n   P: \"Glow\", \"Number\", \"\", \"A\",0.5\n  }\n }\n"
    " Model: 3, \"Model::Arm\", \"LimbNode\" {\n  Properties70: {\n   P: \"InheritType\", "
    "\"enum\", \"\", \"A\",2\n   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n  "
    " P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,45\n  }\n }\n"
    " Model: 4, \"Model::Prop\", \"Null\" {\n  Properties70: {\n   P: \"Lcl Translation\", \"Lcl "
    "Translation\", \"\", \"A\",0,0,0\n  }\n }\n"
    " AnimationStack: 100, \"AnimStack::Walk\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 101, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 110, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 111, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,3\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 112, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 113, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,90\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 114, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",1\n  }\n }\n"
    " AnimationCurve: 115, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 1,2\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 116, \"AnimCurveNode::Glow\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Glow\", \"Number\", \"\", \"A\",0.5\n  }\n }\n"
    " AnimationCurve: 117, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 118, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 119, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,7\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 120, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",5\n  }\n }\n"
    " AnimationCurve: 121, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 5,6\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationStack: 200, \"AnimStack::Idle\", \"\" {\n }\n"
    " AnimationStack: 300, \"AnimStack::Shear\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 301, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 310, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",1\n  }\n }\n"
    " AnimationCurve: 311, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 1,2\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationStack: 400, \"AnimStack::Inherit\", \"\" {\n  Properties70: {\n   P: "
    "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
    "\"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 401, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 410, \"AnimCurveNode::Inherit\", \"\" {\n  Properties70: {\n   P: "
    "\"d|InheritType\", \"Number\", \"\", \"A\",2\n  }\n }\n"
    " AnimationCurve: 411, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 2,7\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",5,0\n C: \"OO\",1,5\n C: \"OO\",2,1\n C: \"OO\",3,2\n C: \"OO\",4,0\n"
    " C: \"OO\",101,100\n"
    " C: \"OO\",110,101\n C: \"OP\",110,1, \"Lcl Translation\"\n C: \"OP\",111,110, \"d|X\"\n"
    " C: \"OO\",112,101\n C: \"OP\",112,2, \"Lcl Rotation\"\n C: \"OP\",113,112, \"d|Z\"\n"
    " C: \"OO\",114,101\n C: \"OP\",114,2, \"Lcl Scaling\"\n C: \"OP\",115,114, \"d|X\"\n C: "
    "\"OP\",115,114, \"d|Y\"\n C: \"OP\",115,114, \"d|Z\"\n"
    " C: \"OO\",116,101\n C: \"OP\",116,2, \"Glow\"\n C: \"OP\",117,116, \"d|Glow\"\n"
    " C: \"OO\",118,101\n C: \"OP\",118,4, \"Lcl Translation\"\n C: \"OP\",119,118, \"d|X\"\n"
    " C: \"OO\",120,101\n C: \"OP\",120,5, \"Lcl Translation\"\n C: \"OP\",121,120, \"d|Z\"\n"
    " C: \"OO\",301,300\n C: \"OO\",310,301\n C: \"OP\",310,2, \"Lcl Scaling\"\n C: "
    "\"OP\",311,310, \"d|X\"\n"
    " C: \"OO\",401,400\n C: \"OO\",410,401\n C: \"OP\",410,3, \"InheritType\"\n C: "
    "\"OP\",411,410, \"d|InheritType\"\n"
    "}\n";

static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_clipFile[65536];
static _Alignas(32) uint8_t s_poseMemory[16384];

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

static bool Skeleton(maniBuilder* builder, const maniFbx* fbx, const maniFbxOptions* options,
                     maniSkeleton* out)
{
    size_t size = 0;
    return maniFbxBuildSkeleton(builder, fbx, 0, options).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, out).result == mani_success;
}

static bool OpenClip(maniBuilder* builder, maniClip* clip)
{
    size_t size = 0;
    return maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
           maniOpenClip(s_clipFile, size, clip).result == mani_success;
}

static bool Near(float a, double b)
{
    return fabs((double)a - b) <= 2e-6;
}

static bool NearVec(maniVec3 v, double x, double y, double z)
{
    return Near(v.x, x) && Near(v.y, y) && Near(v.z, z);
}

// A rotation about z by degrees, either sign.
static bool AboutZ(maniQuat q, double degrees)
{
    double h = degrees * 3.14159265358979323846 / 360.0;
    double s = sin(h);
    double c = cos(h);
    bool plus = Near(q.x, 0) && Near(q.y, 0) && Near(q.z, s) && Near(q.w, c);
    bool minus = Near(q.x, 0) && Near(q.y, 0) && Near(q.z, -s) && Near(q.w, -c);
    return plus || minus;
}

// Walk built for skeleton 0 with the placement folded or not, checked at
// each sample.
static void CheckWalk(int fold)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = (uint8_t)fold;
    maniSkeleton skeleton;
    maniClip clip;
    uint32_t bound = 0;
    bool built = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
                 maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, 0.0f, maniLoopClamp, &bound)
                         .result == mani_success &&
                 OpenClip(builder, &clip);
    CHECK(built, "builds Walk");
    if (!built)
    {
        maniCloseFbx(fbx);
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(bound == 5, "five bindings drive the rig: not Prop's");
    CHECK(maniClipSampleRate(&clip) == 30.0f && maniClipDuration(&clip) == 1.0f &&
              maniClipSampleCount(&clip) == 31,
          "a second at the scene's 30 a second");
    uint32_t glow = maniClipFindFloatTrack(&clip, "Spine/Glow", 10);
    CHECK(glow != MANI_NO_TRACK && maniClipFloatTrackCount(&clip) == 1, "Spine/Glow");
    uint32_t hips = maniSkeletonFindBone(&skeleton, "Hips", 4);
    uint32_t spine = maniSkeletonFindBone(&skeleton, "Spine", 5);
    uint32_t arm = maniSkeletonFindBone(&skeleton, "Arm", 3);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    bool hipsRight = true;
    bool spineRight = true;
    bool armRight = true;
    bool glowRight = true;
    for (int k = 0; k <= 30 && pose != NULL; ++k)
    {
        double t = (double)k / 30.0;
        CHECK(maniSampleClip(&clip, &skeleton, (float)t, pose) == mani_success, "samples");
        maniTransform h = maniPoseLocal(pose, hips);
        maniTransform s = maniPoseLocal(pose, spine);
        maniTransform a = maniPoseLocal(pose, arm);
        hipsRight = hipsRight && NearVec(h.translation, 3 * t, 1, fold ? 5 + t : 0);
        spineRight = spineRight && NearVec(s.translation, 0, 1, 0) && AboutZ(s.rotation, 90 * t) &&
                     NearVec(s.scale, 1 + t, 1 + t, 1 + t);
        // Arm ignores Spine's scale but moves with it: relative to
        // Spine's world, x 1, scale 1 / (1 + t).
        armRight = armRight && NearVec(a.translation, 1, 0, 0) && AboutZ(a.rotation, 45) &&
                   NearVec(a.scale, 1 / (1 + t), 1 / (1 + t), 1 / (1 + t));
        float value = 0.0f;
        glowRight = glowRight &&
                    maniSampleClipFloats(&clip, (float)t, &value, 1, NULL) == mani_success &&
                    Near(value, t);
    }
    CHECK(hipsRight, fold ? "Hips under the folded placement" : "Hips");
    CHECK(spineRight, "Spine turning and scaling");
    CHECK(armRight, "Arm from the worlds");
    CHECK(glowRight, "Glow as a float track");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestNames(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    uint32_t length = 0;
    const char* name = maniFbxClipName(fbx, 2, &length);
    CHECK(maniFbxClipCount(fbx) == 4 && name != NULL && length == 5 &&
              memcmp(name, "Shear", 5) == 0 && maniFbxClipName(fbx, 4, &length) == NULL &&
              length == 0 && maniFbxClipCount(NULL) == 0,
          "clips and their names");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    CHECK(fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
              maniFbxBuildClip(builder, fbx, 1, &skeleton, &options, 0.0f, maniLoopRepeat, NULL)
                      .result == mani_success &&
              OpenClip(builder, &clip) && maniClipDuration(&clip) == 0.0f &&
              maniClipSampleCount(&clip) == 1,
          "an empty stack: one sample");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void ExpectRefusal(uint32_t clipIndex, maniImportReason reason, const char* at,
                          uint32_t item, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniImportResult r = {mani_errorInvalid, 0, 0, 0, 0};
    if (fbx != NULL && Skeleton(builder, fbx, &options, &skeleton))
    {
        r = maniFbxBuildClip(builder, fbx, clipIndex, &skeleton, &options, 0.0f, maniLoopClamp,
                             NULL);
    }
    bool right = r.result == mani_errorFormat && r.reason == reason && r.item == item &&
                 r.offset == (uint64_t)(strstr(kScene, at) - kScene);
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu item %u\n", r.result, r.reason,
               (unsigned long long)r.offset, r.item);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    // Models in file order: Rig 0, Hips 1, Spine 2, Arm 3, Prop 4.
    ExpectRefusal(2, maniImportReasonMatrix, "Model: 3,", 3, "a sheared local");
    ExpectRefusal(3, maniImportReasonProperty, "Model: 3,", 3, "an inherit type past its range");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    bool opened = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton);
    CHECK(
        opened &&
            maniFbxBuildClip(builder, fbx, 4, &skeleton, &options, 0.0f, maniLoopClamp, NULL)
                    .result == mani_errorInvalid &&
            maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, -1.0f, maniLoopClamp, NULL)
                    .result == mani_errorInvalid &&
            maniFbxBuildClip(builder, fbx, 0, NULL, &options, 0.0f, maniLoopClamp, NULL).result ==
                mani_errorInvalid &&
            maniFbxBuildClip(NULL, fbx, 0, &skeleton, &options, 0.0f, maniLoopClamp, NULL).result ==
                mani_errorInvalid,
        "misuses");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    CheckWalk(0);
    CheckWalk(1);
    TestNames();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
