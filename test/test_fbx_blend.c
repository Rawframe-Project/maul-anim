// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX blend shapes (mani-0017): effective weights at channel weights,
// in-betweens and negative targets, against values worked by hand; a
// skinned face's channels as one float track per shape and their names;
// which meshes count; Blender's FullWeights; rest weights.

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
#include <stdlib.h>
#include <string.h>

// Head and Jaw (limbs, Jaw under Head) skin Face; Face's BlendShape has Smile (an
// in-between at 50 percent, Smile_half; a shape without vertices, one
// without indices and a Mesh geometry, all ignored), Blink (one unnamed
// shape), Wide (two unnamed shapes), Frown (its second shape at -100
// percent, sorted first), Squint (FullWeights for its first shape only)
// and Idle (only its Other animated). Orphan hangs from Face's geometry
// with no BlendShape. Prop's Grow belongs to a mesh with a skin of no
// clusters. Talk raises every channel's weight from 0 to 100 over a
// second but Wide's, held at 50.
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
    " Geometry: 72, \"Geometry::Decoy\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n }\n"
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
    " Deformer: 66, \"SubDeformer::Frown\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n  FullWeights: *2 {\n   a: 100,-100\n  }\n "
    "}\n"
    " Geometry: 77, \"Geometry::Frown_up\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  "
    "Vertices: *3 {\n   a: 0,0,1\n  }\n }\n"
    " Geometry: 78, \"Geometry::Frown_down\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  "
    "Vertices: *3 {\n   a: 0,0,1\n  }\n }\n"
    " AnimationCurveNode: 118, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 119, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,100\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " Model: 4, \"Model::Jaw\", \"LimbNode\" {\n  Properties70: {\n   P: \"Lcl Translation\", "
    "\"Lcl Translation\", \"\", \"A\",0,2,0\n  }\n }\n"
    " Deformer: 42, \"SubDeformer::Jaw\", \"Cluster\" {\n }\n"
    " Deformer: 43, \"Deformer::Bare\", \"Skin\" {\n }\n"
    " Geometry: 79, \"Geometry::Decoy2\", \"Shape\" {\n  Vertices: *3 {\n   a: 0,0,1\n  }\n }\n"
    " Geometry: 80, \"Geometry::Stray\", \"Mesh\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: *3 "
    "{\n   a: 0,0,1\n  }\n }\n"
    " Deformer: 67, \"SubDeformer::Squint\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n  FullWeights: *1 {\n   a: 50\n  }\n }\n"
    " Geometry: 81, \"Geometry::Squint_half\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  "
    "Vertices: *3 {\n   a: 0,0,1\n  }\n }\n"
    " Geometry: 82, \"Geometry::Squint\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: "
    "*3 {\n   a: 0,0,1\n  }\n }\n"
    " Deformer: 68, \"SubDeformer::Orphan\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " Geometry: 83, \"Geometry::Orphan\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: "
    "*3 {\n   a: 0,0,1\n  }\n }\n"
    " Deformer: 69, \"SubDeformer::Idle\", \"BlendShapeChannel\" {\n  Properties70: {\n   P: "
    "\"DeformPercent\", \"Number\", \"\", \"A\",0\n   P: \"Other\", \"Number\", \"\", \"A\",0\n  "
    "}\n }\n"
    " Geometry: 84, \"Geometry::Idle\", \"Shape\" {\n  Indexes: *1 {\n   a: 0\n  }\n  Vertices: *3 "
    "{\n   a: 0,0,1\n  }\n }\n"
    " AnimationCurveNode: 120, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 121, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,100\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 122, \"AnimCurveNode::DeformPercent\", \"\" {\n  Properties70: {\n   P: "
    "\"d|DeformPercent\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 123, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,100\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  "
    "KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 124, \"AnimCurveNode::Other\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Other\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 125, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
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
    " C: \"OO\",66,60\n C: \"OO\",77,66\n C: \"OO\",78,66\n C: \"OO\",118,101\n C: \"OP\",118,66, "
    "\"DeformPercent\"\n C: \"OP\",119,118, \"d|DeformPercent\"\n"
    " C: \"OO\",4,1\n C: \"OO\",42,40\n C: \"OO\",4,42\n C: \"OO\",43,51\n C: \"OO\",79,61\n C: "
    "\"OO\",80,61\n C: \"OO\",67,60\n C: \"OO\",81,67\n C: \"OO\",82,67\n C: \"OO\",68,50\n C: "
    "\"OO\",83,68\n C: \"OO\",69,60\n C: \"OO\",84,69\n C: \"OO\",120,101\n C: \"OP\",120,67, "
    "\"DeformPercent\"\n C: \"OP\",121,120, \"d|DeformPercent\"\n C: \"OO\",122,101\n C: "
    "\"OP\",122,68, \"DeformPercent\"\n C: \"OP\",123,122, \"d|DeformPercent\"\n C: "
    "\"OO\",124,101\n C: \"OP\",124,69, \"Other\"\n C: \"OP\",125,124, \"d|Other\"\n"
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
    const double below[3] = {-1.5, -1.0, -0.5};
    CHECK(Weights(below, 3, -0.75, (const double[]){0, 0.5, 0.5}),
          "the walk down stops at the first target below the weight");
    const double zero[1] = {0.0};
    CHECK(Weights(zero, 1, 0.5, (const double[]){0}), "a target at the rest takes nothing");
    maniFbxBlendWeights(NULL, 0, 0.5, NULL);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// A skeleton of bones by name, at rest.
static bool Bones(maniBuilder* builder, const char* const* names, uint32_t count, maniSkeleton* out)
{
    const maniTransform rest = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    maniBoneSource bones[4];
    for (uint32_t k = 0; k < count; ++k)
    {
        bones[k] = (maniBoneSource){names[k], (uint32_t)strlen(names[k]), -1, rest};
    }
    maniSkeletonSource source = {bones, count, 0};
    size_t size = 0;
    return maniBuildSkeleton(builder, &source).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, out).result == mani_success;
}

// Talk's clip of a text, for the file's skeleton (names NULL) or bones by
// name.
static bool Build(maniBuilder* builder, const char* text, const char* const* names, uint32_t count,
                  maniClip* clip, uint32_t* bound)
{
    maniFbx* fbx = NULL;
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    size_t size = 0;
    bool right = maniOpenFbx(builder, text, strlen(text), &fbx).result == mani_success;
    if (right && names == NULL)
    {
        right =
            maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
            maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
            maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success;
    }
    else if (right)
    {
        right = Bones(builder, names, count, &skeleton);
    }
    right =
        right &&
        maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, 0.0f, maniLoopClamp, bound).result ==
            mani_success &&
        maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
        maniOpenClip(s_clipFile, size, clip).result == mani_success;
    maniCloseFbx(fbx);
    return right;
}

// A float track's value at a time; NaN when there is no such track.
static double ValueAt(const maniClip* clip, const char* name, float time)
{
    float values[16] = {0};
    uint32_t track = maniClipFindFloatTrack(clip, name, strlen(name));
    if (track == MANI_NO_TRACK || maniClipFloatTrackCount(clip) > 16 ||
        maniSampleClipFloats(clip, time, values, 16, NULL) != mani_success)
    {
        return (double)NAN;
    }
    return (double)values[track];
}

static void TestTracks(void)
{
    maniBuilder* builder = NewBuilder();
    maniClip clip;
    uint32_t bound = 0;
    bool built = Build(builder, kScene, NULL, 0, &clip, &bound);
    CHECK(built && maniClipFloatTrackCount(&clip) == 9 && bound == 5,
          "nine shapes of the skinned face, five weights bound");
    static const char* const kNames[9] = {"Face/Smile_half", "Face/Smile",       "Face/Blink",
                                          "Face/Wide",       "Face/Wide/1",      "Face/Frown_up",
                                          "Face/Frown_down", "Face/Squint_half", "Face/Squint"};
    // At 0.25 and 0.75 s: Smile's weight there, handed from the half
    // shape to the whole; Blink the weight itself; Wide's first shape
    // 0.5, its second (the same target, after it) 0; Frown's upper shape
    // the weight, its lower none; Squint as Smile, its second shape's
    // target 1 past FullWeights.
    static const double kAt[2][9] = {{0.5, 0, 0.25, 0.5, 0, 0.25, 0, 0.5, 0},
                                     {0.5, 0.5, 0.75, 0.5, 0, 0.75, 0, 0.5, 0.5}};
    bool right = built;
    for (int s = 0; s < 2 && right; ++s)
    {
        for (int k = 0; k < 9 && right; ++k)
        {
            right = Near(ValueAt(&clip, kNames[k], s == 0 ? 0.25f : 0.75f), kAt[s][k]);
        }
    }
    CHECK(right, "effective weights by shape");
    maniDestroyBuilder(builder);
}

// Which meshes count: a bone itself, or a skin all of whose joints are
// bones (a skin of no joints none).
static void TestMeshes(void)
{
    maniBuilder* builder = NewBuilder();
    maniClip clip;
    uint32_t bound = 0;
    static const char* const kHeadProp[2] = {"Head", "Prop"};
    CHECK(Build(builder, kScene, kHeadProp, 2, &clip, &bound) &&
              maniClipFloatTrackCount(&clip) == 1 && Near(ValueAt(&clip, "Prop/Grow", 0.5f), 0.5),
          "a mesh that is a bone; a skin with a joint outside the skeleton left out");
    maniDestroyBuilder(builder);
    builder = NewBuilder();
    CHECK(Build(builder, kScene, kHeadProp, 1, &clip, &bound) &&
              maniClipFloatTrackCount(&clip) == 0,
          "a skin of no joints is no skin within the skeleton");
    maniDestroyBuilder(builder);
}

// Smile_half's weight at 0.75 s with a Creator.
static double HalfWithCreator(const char* creator)
{
    static char text[sizeof kScene + 256];
    const char* rest = strchr(kScene, '\n') + 1;
    snprintf(text, sizeof text,
             "; FBX 7.4.0 project file\nFBXHeaderExtension: {\n Creator: \"%s\"\n}\n%s", creator,
             rest);
    maniBuilder* builder = NewBuilder();
    maniClip clip;
    uint32_t bound = 0;
    double v = Build(builder, text, NULL, 0, &clip, &bound)
                   ? ValueAt(&clip, "Face/Smile_half", 0.75f)
                   : (double)NAN;
    maniDestroyBuilder(builder);
    return v;
}

static void TestBlender(void)
{
    // Blender's binary exporter: every target 1, so the half shape, first
    // in connection order, takes the whole weight.
    CHECK(Near(HalfWithCreator("Blender (stable FBX IO) - 2.79 (sub 0) - 3.8.1"), 0.75) &&
              Near(HalfWithCreator("blender-2.79"), 0.75),
          "Blender's FullWeights are no targets");
    CHECK(Near(HalfWithCreator("Blender (stable FBX IO)"), 0.5) &&
              Near(HalfWithCreator("Blender version 2.79 (sub 0)"), 0.5) &&
              Near(HalfWithCreator("FBX SDK/FBX Plugins version 2020.2"), 0.5),
          "other creators' FullWeights are targets");
}

// The rest weight: the property, Blender's child node (100 when not a
// number) where the channel has no properties, else 0.
static const char kRests[] =
    "; FBX 7.4.0 project file\nObjects: {\n"
    " Deformer: 1, \"SubDeformer::A\", \"BlendShapeChannel\" {\n  Properties70: {\n"
    "   P: \"DeformPercent\", \"Number\", \"\", \"A\",30\n  }\n  DeformPercent: 70\n }\n"
    " Deformer: 2, \"SubDeformer::B\", \"BlendShapeChannel\" {\n  DeformPercent: 40\n }\n"
    " Deformer: 3, \"SubDeformer::C\", \"BlendShapeChannel\" {\n  DeformPercent: \"x\"\n }\n"
    " Deformer: 4, \"SubDeformer::D\", \"BlendShapeChannel\" {\n }\n"
    " Deformer: 5, \"SubDeformer::E\", \"BlendShapeChannel\" {\n  Properties70: {\n"
    "   P: \"Other\", \"Number\", \"\", \"A\",1\n  }\n  DeformPercent: 40\n }\n"
    "}\n";

static void TestRests(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    static const double kWant[5] = {30, 40, 100, 0, 0};
    uint32_t found = 0;
    bool right = maniOpenFbx(builder, kRests, strlen(kRests), &fbx).result == mani_success;
    for (uint32_t o = 0; right && o < fbx->objectCount; ++o)
    {
        if (maniFbxIsBlendChannel(fbx, o))
        {
            right = found < 5 && Near(maniFbxBlendRestPercent(fbx, o), kWant[found]);
            ++found;
        }
    }
    CHECK(right && found == 5, "rest weights");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Before 7.2, shapes held by meshes (mani-0017), as ufbx weighs them:
// a 6.x Mesh model's Smile (a number) and Wink (an integer) are driven;
// Blink (user-defined, so the model's), Frown (a Bool), Gape (no
// property), Hollow (no arrays), Half (no Vertices) and Part (no
// Indexes) are not. The deformers come first, so the models' indices
// differ from their objects'.
static const char kShapes6[] =
    "; FBX 6.1.0 project file\n"
    "FBXHeaderExtension: {\n"
    " FBXHeaderVersion: 1003\n"
    " FBXVersion: 6100\n"
    "}\n"
    "Objects: {\n"
    " Deformer: \"Deformer::Skin Face\", \"Skin\" {\n"
    "  Version: 100\n"
    " }\n"
    " Deformer: \"SubDeformer::Cluster Head\", \"Cluster\" {\n"
    "  Version: 100\n"
    "  Indexes: 0\n"
    "  Weights: 1\n"
    "  Transform: 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1\n"
    "  TransformLink: 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1\n"
    " }\n"
    " Model: \"Model::Head\", \"Limb\" {\n"
    "  Version: 232\n"
    "  Properties60: {\n"
    "   Property: \"Lcl Translation\", \"Lcl Translation\", \"A+\",0,1,0\n"
    "  }\n"
    " }\n"
    " Model: \"Model::Face\", \"Mesh\" {\n"
    "  Version: 232\n"
    "  Properties60: {\n"
    "   Property: \"Smile\", \"Number\", \"A+\",10\n"
    "   Property: \"Blink\", \"Number\", \"A+U\",0\n"
    "   Property: \"Frown\", \"Bool\", \"A+\",0\n"
    "   Property: \"Wink\", \"Integer\", \"A+\",0\n"
    "   Property: \"Half\", \"Number\", \"A+\",0\n"
    "   Property: \"Part\", \"Number\", \"A+\",0\n"
    "  }\n"
    "  Vertices: 0,0,0,1,0,0,0,1,0\n"
    "  PolygonVertexIndex: 0,1,-3\n"
    "  Shape: \"Smile\" {\n"
    "   Indexes: 0\n"
    "   Vertices: 0,0,1\n"
    "  }\n"
    "  Shape: \"Blink\" {\n"
    "   Indexes: 0\n"
    "   Vertices: 0,0,1\n"
    "  }\n"
    "  Shape: \"Frown\" {\n"
    "   Indexes: 0\n"
    "   Vertices: 0,0,1\n"
    "  }\n"
    "  Shape: \"Wink\" {\n"
    "   Indexes: 0\n"
    "   Vertices: 0,0,1\n"
    "  }\n"
    "  Shape: \"Gape\" {\n"
    "   Indexes: 0\n"
    "   Vertices: 0,0,1\n"
    "  }\n"
    "  Shape: \"Hollow\" {\n"
    "  }\n"
    "  Shape: \"Half\" {\n"
    "   Indexes: 0\n"
    "  }\n"
    "  Shape: \"Part\" {\n"
    "   Vertices: 0,0,1\n"
    "  }\n"
    " }\n"
    "}\n"
    "Connections: {\n"
    " Connect: \"OO\", \"Model::Head\", \"Model::Scene\"\n"
    " Connect: \"OO\", \"Model::Face\", \"Model::Scene\"\n"
    " Connect: \"OO\", \"Deformer::Skin Face\", \"Model::Face\"\n"
    " Connect: \"OO\", \"SubDeformer::Cluster Head\", \"Deformer::Skin Face\"\n"
    " Connect: \"OO\", \"Model::Head\", \"SubDeformer::Cluster Head\"\n"
    "}\n"
    "Takes: {\n"
    " Take: \"Talk\" {\n"
    "  LocalTime: 0,46186158000\n"
    "  Model: \"Model::Face\" {\n"
    "   Channel: \"Smile\" {\n"
    "    Default: 10\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,10,L,46186158000,90,L\n"
    "   }\n"
    "   Channel: \"Blink\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "   Channel: \"Frown\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "   Channel: \"Wink\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "   Channel: \"Gape\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "   Channel: \"Half\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "   Channel: \"Part\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "   Channel: \"Hollow\" {\n"
    "    Default: 0\n"
    "    KeyVer: 4005\n"
    "    KeyCount: 2\n"
    "    Key: 0,0,L,46186158000,100,L\n"
    "   }\n"
    "  }\n"
    " }\n"
    "}\n";

// A 7.1 geometry's Smile and Blink, both driven (no split in 7.x);
// Blink only in a second layer, which adds to its rest.
static const char kShapes71[] =
    "; FBX 7.1.0 project file\n"
    "FBXHeaderExtension: {\n"
    " FBXHeaderVersion: 1003\n"
    " FBXVersion: 7100\n"
    "}\n"
    "Objects: {\n"
    " Model: 1, \"Model::Head\", \"LimbNode\" {\n"
    "  Properties70: {\n"
    "   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,1,0\n"
    "  }\n"
    " }\n"
    " Model: 2, \"Model::Face\", \"Mesh\" {\n"
    " }\n"
    " Geometry: 50, \"Geometry::Face\", \"Mesh\" {\n"
    "  Properties70: {\n"
    "   P: \"Smile\", \"Number\", \"\", \"A\",10\n"
    "   P: \"Blink\", \"Number\", \"\", \"A+U\",20\n"
    "  }\n"
    "  Vertices: *9 {\n"
    "   a: 0,0,0,1,0,0,0,1,0\n"
    "  }\n"
    "  PolygonVertexIndex: *3 {\n"
    "   a: 0,1,-3\n"
    "  }\n"
    "  Shape: \"Smile\" {\n"
    "   Indexes: *1 {\n"
    "    a: 0\n"
    "   }\n"
    "   Vertices: *3 {\n"
    "    a: 0,0,1\n"
    "   }\n"
    "  }\n"
    "  Shape: \"Blink\" {\n"
    "   Indexes: *1 {\n"
    "    a: 0\n"
    "   }\n"
    "   Vertices: *3 {\n"
    "    a: 0,0,1\n"
    "   }\n"
    "  }\n"
    " }\n"
    " Deformer: 40, \"Deformer::Skin\", \"Skin\" {\n"
    " }\n"
    " Deformer: 41, \"SubDeformer::Head\", \"Cluster\" {\n"
    " }\n"
    " AnimationStack: 100, \"AnimStack::Talk\", \"\" {\n"
    "  Properties70: {\n"
    "   P: \"LocalStart\", \"KTime\", \"Time\", \"\",0\n"
    "   P: \"LocalStop\", \"KTime\", \"Time\", \"\",46186158000\n"
    "  }\n"
    " }\n"
    " AnimationLayer: 101, \"AnimLayer::Base\", \"\" {\n"
    " }\n"
    " AnimationLayer: 102, \"AnimLayer::Over\", \"\" {\n"
    " }\n"
    " AnimationCurveNode: 110, \"AnimCurveNode::Smile\", \"\" {\n"
    "  Properties70: {\n"
    "   P: \"d|Smile\", \"Number\", \"\", \"A\",10\n"
    "  }\n"
    " }\n"
    " AnimationCurve: 111, \"AnimCurve::\", \"\" {\n"
    "  KeyTime: *2 {\n"
    "   a: 0,46186158000\n"
    "  }\n"
    "  KeyValueFloat: *2 {\n"
    "   a: 10,90\n"
    "  }\n"
    "  KeyAttrFlags: *1 {\n"
    "   a: 4\n"
    "  }\n"
    "  KeyAttrDataFloat: *4 {\n"
    "   a: 0,0,0,0\n"
    "  }\n"
    "  KeyAttrRefCount: *1 {\n"
    "   a: 2\n"
    "  }\n"
    " }\n"
    " AnimationCurveNode: 112, \"AnimCurveNode::Blink\", \"\" {\n"
    "  Properties70: {\n"
    "   P: \"d|Blink\", \"Number\", \"\", \"A\",20\n"
    "  }\n"
    " }\n"
    " AnimationCurve: 113, \"AnimCurve::\", \"\" {\n"
    "  KeyTime: *2 {\n"
    "   a: 0,46186158000\n"
    "  }\n"
    "  KeyValueFloat: *2 {\n"
    "   a: 20,60\n"
    "  }\n"
    "  KeyAttrFlags: *1 {\n"
    "   a: 4\n"
    "  }\n"
    "  KeyAttrDataFloat: *4 {\n"
    "   a: 0,0,0,0\n"
    "  }\n"
    "  KeyAttrRefCount: *1 {\n"
    "   a: 2\n"
    "  }\n"
    " }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n"
    " C: \"OO\",2,0\n"
    " C: \"OO\",50,2\n"
    " C: \"OO\",40,50\n"
    " C: \"OO\",41,40\n"
    " C: \"OO\",1,41\n"
    " C: \"OO\",101,100\n"
    " C: \"OO\",110,101\n"
    " C: \"OP\",110,50, \"Smile\"\n"
    " C: \"OP\",111,110, \"d|Smile\"\n"
    " C: \"OO\",102,100\n"
    " C: \"OO\",112,102\n"
    " C: \"OP\",112,50, \"Blink\"\n"
    " C: \"OP\",113,112, \"d|Blink\"\n"
    "}\n";

// Talk's float tracks of a text: the count, and each named value at
// times 0, 0.25 ... 1.
static bool ShapeTracks(const char* text, uint32_t count, const char* const* names,
                        const double (*values)[5])
{
    maniBuilder* builder = NewBuilder();
    maniClip clip;
    uint32_t bound = 0;
    bool right = Build(builder, text, NULL, 0, &clip, &bound) &&
                 maniClipFloatTrackCount(&clip) == count && bound == count;
    for (uint32_t k = 0; right && k < count; ++k)
    {
        for (int s = 0; right && s < 5; ++s)
        {
            right = Near(ValueAt(&clip, names[k], (float)s * 0.25f), values[k][s]);
        }
    }
    maniDestroyBuilder(builder);
    return right;
}

static void TestShapeOwners(void)
{
    static const char* const kNames6[2] = {"Face/Smile", "Face/Wink"};
    static const double kValues6[2][5] = {{0.1, 0.3, 0.5, 0.7, 0.9}, {0, 0.25, 0.5, 0.75, 1}};
    CHECK(ShapeTracks(kShapes6, 2, kNames6, kValues6), "a 6.x mesh model's shapes");
    static const char* const kNames71[2] = {"Face/Smile", "Face/Blink"};
    static const double kValues71[2][5] = {{0.1, 0.3, 0.5, 0.7, 0.9}, {0.4, 0.5, 0.6, 0.7, 0.8}};
    CHECK(ShapeTracks(kShapes71, 2, kNames71, kValues71), "a 7.1 geometry's shapes");
    {
        // Face skinned to Head: none for a skeleton without Head.
        static const char* const kOther[1] = {"Other"};
        maniBuilder* builder = NewBuilder();
        maniClip clip;
        uint32_t bound = 0;
        CHECK(Build(builder, kShapes6, kOther, 1, &clip, &bound) &&
                  maniClipFloatTrackCount(&clip) == 0,
              "no shapes of a mesh outside the skeleton");
        maniDestroyBuilder(builder);
    }
    // From 7.2, Shape children are no blend shapes.
    static char text[sizeof kShapes71];
    snprintf(text, sizeof text, "%s", kShapes71);
    memcpy(strstr(text, "7.1.0"), "7.2.0", 5);
    memcpy(strstr(text, "7100"), "7200", 4);
    CHECK(ShapeTracks(text, 0, NULL, NULL), "a 7.2 geometry's Shape children");
    // A Shape without a name is refused, as ufbx refuses the file.
    static char unnamed[sizeof kShapes6];
    snprintf(unnamed, sizeof unnamed, "%s", kShapes6);
    char* hollow = strstr(unnamed, "Shape: \"Hollow\"");
    memcpy(hollow, "Shape:          ", 16);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxOptions options = maniDefaultFbxOptions();
    size_t size = 0;
    maniSkeleton skeleton;
    maniImportResult r = {mani_success, 0, 0, 0, 0};
    if (maniOpenFbx(builder, unnamed, strlen(unnamed), &fbx).result == mani_success &&
        maniFbxBuildSkeleton(builder, fbx, 0, &options).result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success)
    {
        r = maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, 0.0f, maniLoopClamp, NULL);
    }
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
              r.offset == (uint64_t)(hollow - unnamed),
          "a shape without a name");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// An allocator that fails its n-th call and counts what is outstanding.
typedef struct Budget
{
    int32_t callsLeft;
    int64_t outstanding;
} Budget;

static void* BudgetAlloc(size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    (void)alignment;
    if (budget->callsLeft-- <= 0)
    {
        return NULL;
    }
    budget->outstanding += (int64_t)size;
    return malloc(size);
}

static void BudgetFree(void* memory, size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    (void)alignment;
    budget->outstanding -= (int64_t)size;
    free(memory);
}

// Every allocation failing in turn while building Talk's clip of a text:
// each fails cleanly, and everything taken is given back at its size.
static void TestAllocationFailures(const char* text, const char* label)
{
    bool clean = true;
    bool done = false;
    for (int32_t calls = 0; !done && calls < 8192; ++calls)
    {
        Budget budget = {calls, 0};
        maniBuilderDef def = maniDefaultBuilderDef();
        def.allocator = (maniAllocator){BudgetAlloc, BudgetFree, &budget};
        maniBuilder* builder = NULL;
        if (maniCreateBuilder(&def, &builder) == mani_success)
        {
            maniClip clip;
            uint32_t bound = 0;
            done = Build(builder, text, NULL, 0, &clip, &bound);
            maniDestroyBuilder(builder);
        }
        clean = clean && budget.outstanding == 0;
    }
    CHECK(done && clean, label);
}

int main(void)
{
    TestAllocationFailures(kScene, "allocation failures handled, nothing kept");
    TestWeights();
    TestTracks();
    TestMeshes();
    TestBlender();
    TestRests();
    TestShapeOwners();
    TestAllocationFailures(kShapes6, "allocation failures handled for 6.x shapes");
    return s_failures == 0 ? 0 : 1;
}
