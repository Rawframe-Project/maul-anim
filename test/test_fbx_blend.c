// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX blend shapes (research 25): effective weights at channel weights,
// in-betweens and negative targets, against values worked by hand; a
// skinned face's channels as one float track per shape, their names,
// and a mesh outside the skeleton left out.

#include "fbx_blend.h"
#include "fbx_document.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Head (a limb) skins Face; Face's BlendShape has Smile (an in-between
// at 50 percent, Smile_half, and a shape without vertices, ignored),
// Blink (one unnamed shape) and Wide (two unnamed shapes); Prop's Grow
// belongs to no skeleton's mesh. Talk raises Smile, Blink and Grow from
// 0 to 100 over a second and holds Wide at 50.
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Model: 1, \"Model::Head\", \"LimbNode\" {\n  Properties70: {\n   P: \"Lcl Translation\", "
    "\"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    " Model: 2, \"Model::Face\", \"Mesh\" {\n }\n"
    " Model: 3, \"Model::Prop\", \"Mesh\" {\n }\n"
    " Geometry: 50, \"Geometry::Face\", \"Mesh\" {\n }\n"
    " Geometry: 51, \"Geometry::Prop\", \"Mesh\" {\n }\n"
    " Deformer: 40, \"Deformer::Skin\", \"Skin\" {\n }\n"
    " Deformer: 41, \"SubDeformer::Head\", \"Cluster\" {\n }\n"
    " Deformer: 60, \"Deformer::Face\", \"BlendShape\" {\n }\n"
    " Deformer: 61, \"SubDeformer::Smile\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n  FullWeights: *2 {\n   a: 50,100\n  }\n "
    "}\n"
    " Geometry: 70, \"Geometry::Smile_half\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  "
    "Vertices: *3 {\n   a: 0,0,1\n  }\n }\n"
    " Geometry: 71, \"Geometry::Smile\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: "
    "*3 {\n   a: 0,0,1\n  }\n }\n"
    " Geometry: 72, \"Geometry::Decoy\", \"Shape\" {\n }\n"
    " Deformer: 62, \"SubDeformer::Blink\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " Geometry: 73, \"Geometry::\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: *3 {\n "
    "  a: 0,0,1\n  }\n }\n"
    " Deformer: 63, \"SubDeformer::Wide\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " Geometry: 74, \"Geometry::\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: *3 {\n "
    "  a: 0,0,1\n  }\n }\n"
    " Geometry: 75, \"Geometry::\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: *3 {\n "
    "  a: 0,0,1\n  }\n }\n"
    " Deformer: 64, \"Deformer::Prop\", \"BlendShape\" {\n }\n"
    " Deformer: 65, \"SubDeformer::Grow\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " Geometry: 76, \"Geometry::Grow\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: *3 "
    "{\n   a: 0,0,1\n  }\n }\n"
    " AnimationStack: 100, \"AnimStack::Talk\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 101, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 110, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 111, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,100\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 112, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 113, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,100\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 114, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 115, \"AnimCurve::\", \"\" {\n  KeyTime: *1 {\n   a: 0\n  }\n  "
    "KeyValueFloat: *1 {\n   a: 50\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 1\n  }\n }\n"
    " AnimationCurveNode: 116, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 117, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,100\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n C: \"OO\",2,1\n C: \"OO\",3,0\n C: \"OO\",50,2\n C: \"OO\",51,3\n C: "
    "\"OO\",40,50\n C: \"OO\",41,40\n C: \"OO\",1,41\n"
    " C: \"OO\",60,50\n C: \"OO\",61,60\n C: \"OO\",70,61\n C: \"OO\",72,61\n C: \"OO\",71,61\n C: "
    "\"OO\",62,60\n C: \"OO\",73,62\n C: \"OO\",63,60\n C: \"OO\",74,63\n C: \"OO\",75,63\n"
    " C: \"OO\",64,51\n C: \"OO\",65,64\n C: \"OO\",76,65\n"
    " C: \"OO\",101,100\n"
    " C: \"OO\",110,101\n C: \"OP\",110,61, \"DeformPercent\"\n C: \"OP\",111,110, "
    "\"d|DeformPercent\"\n"
    " C: \"OO\",112,101\n C: \"OP\",112,62, \"DeformPercent\"\n C: \"OP\",113,112, "
    "\"d|DeformPercent\"\n"
    " C: \"OO\",114,101\n C: \"OP\",114,63, \"DeformPercent\"\n C: \"OP\",115,114, "
    "\"d|DeformPercent\"\n"
    " C: \"OO\",116,101\n C: \"OP\",116,65, \"DeformPercent\"\n C: \"OP\",117,116, "
    "\"d|DeformPercent\"\n"
    "}\n";

static _Alignas(16) uint8_t s_skeletonFile[16384];
static _Alignas(16) uint8_t s_clipFile[65536];

static bool Near(double a, double b)
{
    return fabs(a - b) <= 1e-6;
}

static bool Weights(const double* targets, uint32_t n, double w, const double* want)
{
    maniFbxBlendShape shapes[4];
    double effective[4];
    for (uint32_t k = 0; k < n; ++k)
    {
        shapes[k] = (maniFbxBlendShape){k, 0, targets[k]};
    }
    maniFbxBlendWeights(shapes, n, w, effective);
    bool right = true;
    for (uint32_t k = 0; k < n; ++k)
    {
        right = right && Near(effective[k], want[k]);
    }
    return right;
}

static void TestWeights(void)
{
    const double one[1] = {1.0};
    CHECK(Weights(one, 1, 0.3, (const double[]){0.3}) &&
              Weights(one, 1, 1.5, (const double[]){1.5}) &&
              Weights(one, 1, -0.2, (const double[]){-0.2}),
          "one shape follows the weight, past either end");
    const double between[2] = {0.5, 1.0};
    CHECK(Weights(between, 2, 0.25, (const double[]){0.5, 0}) &&
              Weights(between, 2, 0.75, (const double[]){0.5, 0.5}) &&
              Weights(between, 2, 1.0, (const double[]){0, 1}),
          "an in-between hands over");
    const double signs[2] = {-1.0, 1.0};
    CHECK(Weights(signs, 2, -0.5, (const double[]){0.5, 0}) &&
              Weights(signs, 2, 0.5, (const double[]){0, 0.5}),
          "a negative target below zero, a positive one above");
    maniFbxBlendWeights(NULL, 0, 0.5, NULL);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Talk's float tracks at times, by name.
static void TestTracks(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success, "opens");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    size_t size = 0;
    uint32_t bound = 0;
    bool built =
        fbx != NULL && maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success &&
        maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, 0.0f, maniLoopClamp, &bound)
                .result == mani_success &&
        maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
        maniOpenClip(s_clipFile, size, &clip).result == mani_success;
    CHECK(built && maniClipFloatTrackCount(&clip) == 5 && bound == 3,
          "five shapes of the skinned face, three weights bound; not Prop's");
    static const char* const kNames[5] = {"Face/Smile_half", "Face/Smile", "Face/Blink",
                                          "Face/Wide", "Face/Wide/1"};
    // At 0.25 and 0.75 s: Smile's weight there, handed from the half
    // shape to the whole; Blink the weight itself; Wide's first shape
    // 0.5, its second (the same target, after it) 0.
    static const double kAt[2][5] = {{0.5, 0, 0.25, 0.5, 0}, {0.5, 0.5, 0.75, 0.5, 0}};
    bool right = built;
    for (int s = 0; s < 2 && right; ++s)
    {
        float values[5] = {0};
        right =
            maniSampleClipFloats(&clip, s == 0 ? 0.25f : 0.75f, values, 5, NULL) == mani_success;
        for (int k = 0; k < 5 && right; ++k)
        {
            uint32_t track = maniClipFindFloatTrack(&clip, kNames[k], strlen(kNames[k]));
            right = track != MANI_NO_TRACK && Near((double)values[track], kAt[s][k]);
        }
    }
    CHECK(right, "effective weights by shape");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestWeights();
    TestTracks();
    return s_failures == 0 ? 0 : 1;
}
