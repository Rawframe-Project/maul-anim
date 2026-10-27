// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX stacks: one model's properties through five layers
// (a base, an additive one, a passthrough one with an animated weight,
// an override and an additive one adding components), against values
// derived by hand from ufbx's rules; stack ranges from properties, a
// take and the keys; and a property driven twice in a layer, and a
// channel value not a number, refused.

#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Model A (Glow 5). Stack Run, 0 to 1 s: Base sets translation x from 0
// to 10 over 1 s (y 2, z 3); Add (weight 0.5) adds (0, 4, 0), scales
// by (4, -4, 4) and turns (90, 0, 90); Pass (weight 0.25, animated to
// 0.5 until 0.5 s, then up to 2.5) blends toward translation 10, scale
// 16 and no rotation; Over (override) sets Glow 9; Parts adds rotation z
// 10 and scale 1 by components; None (weight -20, so 0) adds
// translation 1 and scales by (0, 2, 2); Full (weight 150, so 1) adds
// translation z 1. Walk's take has only reference times (after a take
// of another name); Span's curves run 0.1 to 0.5, 0.25 to 0.75 and 0.3
// to 0.6 s; Ref has reference times; Jog's take local and reference
// times; Exact scales 3 then by (8, -8, 8) at full weight.
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Model: 1, \"Model::A\", \"Null\" {\n  Properties70: {\n   P: \"Glow\", \"Number\", \"\", "
    "\"A\",5\n  }\n }\n"
    " AnimationStack: 100, \"AnimStack::Run\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 101, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationLayer: 102, \"AnimLayer::Add\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",50\n   P: \"BlendMode\", \"enum\", \"\", \"A\",0\n  }\n }\n"
    " AnimationLayer: 103, \"AnimLayer::Pass\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",25\n   P: \"BlendMode\", \"enum\", \"\", \"A\",2\n  }\n }\n"
    " AnimationLayer: 104, \"AnimLayer::Over\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",10\n   P: \"BlendMode\", \"enum\", \"\", \"A\",1\n  }\n }\n"
    " AnimationLayer: 105, \"AnimLayer::Parts\", \"\" {\n  Properties70: {\n   P: "
    "\"RotationAccumulationMode\", \"enum\", \"\", \"A\",1\n   P: \"ScaleAccumulationMode\", "
    "\"enum\", \"\", \"A\",1\n  }\n }\n"
    " AnimationLayer: 106, \"AnimLayer::None\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",-20\n  }\n }\n"
    " AnimationLayer: 107, \"AnimLayer::Full\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",150\n  }\n }\n"
    " AnimationCurveNode: 110, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",2\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",3\n  }\n }\n"
    " AnimationCurve: 111, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,10\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 112, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",4\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurveNode: 113, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",4\n   P: \"d|Y\", \"Number\", \"\", \"A\",-4\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",4\n  }\n }\n"
    " AnimationCurveNode: 114, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",90\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",90\n  }\n }\n"
    " AnimationCurveNode: 130, \"AnimCurveNode::Weight\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Weight\", \"Number\", \"\", \"A\",25\n  }\n }\n"
    " AnimationCurve: 131, \"AnimCurve::\", \"\" {\n  KeyTime: *3 {\n   a: "
    "0,23093079000,46186158000\n  }\n  KeyValueFloat: *3 {\n   a: 50,50,250\n  }\n  KeyAttrFlags: "
    "*1 {\n   a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   "
    "a: 3\n  }\n }\n"
    " AnimationCurveNode: 115, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",10\n   P: \"d|Y\", \"Number\", \"\", \"A\",10\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",10\n  }\n }\n"
    " AnimationCurveNode: 118, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",16\n   P: \"d|Y\", \"Number\", \"\", \"A\",16\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",16\n  }\n }\n"
    " AnimationCurveNode: 119, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurveNode: 116, \"AnimCurveNode::Glow\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Glow\", \"Number\", \"\", \"A\",5\n  }\n }\n"
    " AnimationCurve: 117, \"AnimCurve::\", \"\" {\n  KeyTime: *1 {\n   a: 0\n  }\n  "
    "KeyValueFloat: *1 {\n   a: 9\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: *4 "
    "{\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 1\n  }\n }\n"
    " AnimationCurveNode: 120, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",10\n  }\n }\n"
    " AnimationCurveNode: 121, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",1\n  }\n }\n"
    " AnimationCurveNode: 122, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",1\n  }\n }\n"
    " AnimationCurveNode: 123, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",2\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",2\n  }\n }\n"
    " AnimationCurveNode: 124, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",1\n  }\n }\n"
    " AnimationStack: 200, \"AnimStack::Walk\", \"\" {\n }\n"
    " AnimationStack: 300, \"AnimStack::Span\", \"\" {\n }\n"
    " AnimationLayer: 301, \"AnimLayer::Only\", \"\" {\n }\n"
    " AnimationCurveNode: 310, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 311, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: "
    "11546539500,34639618500\n  }\n  KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   "
    "a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  "
    "}\n }\n"
    " AnimationCurve: 312, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: "
    "4618615800,23093079000\n  }\n  KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   "
    "a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  "
    "}\n }\n"
    " AnimationCurve: 313, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: "
    "13855847400,27711694800\n  }\n  KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   "
    "a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  "
    "}\n }\n"
    " AnimationStack: 400, \"AnimStack::Ref\", \"\" {\n  Properties70: {\n   P: "
    "\"ReferenceStart\", \"KTime\", \"\", \"A\",3\n   P: \"ReferenceStop\", \"KTime\", \"\", "
    "\"A\",4\n  }\n }\n"
    " AnimationStack: 500, \"AnimStack::Jog\", \"\" {\n }\n"
    " AnimationStack: 600, \"AnimStack::Exact\", \"\" {\n }\n"
    " AnimationLayer: 601, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationLayer: 602, \"AnimLayer::Add\", \"\" {\n }\n"
    " AnimationCurveNode: 610, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",3\n   P: \"d|Y\", \"Number\", \"\", \"A\",3\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",3\n  }\n }\n"
    " AnimationCurveNode: 611, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",8\n   P: \"d|Y\", \"Number\", \"\", \"A\",-8\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",8\n  }\n }\n"
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n"
    " C: \"OO\",101,100\n C: \"OO\",102,100\n C: \"OO\",103,100\n C: \"OO\",104,100\n C: "
    "\"OO\",105,100\n C: \"OO\",106,100\n C: \"OO\",107,100\n"
    " C: \"OO\",110,101\n C: \"OP\",110,1, \"Lcl Translation\"\n C: \"OP\",111,110, \"d|X\"\n"
    " C: \"OO\",112,102\n C: \"OP\",112,1, \"Lcl Translation\"\n C: \"OP\",112,0, \"Lcl "
    "Translation\"\n"
    " C: \"OO\",113,102\n C: \"OP\",113,1, \"Lcl Scaling\"\n"
    " C: \"OO\",114,102\n C: \"OP\",114,1, \"Lcl Rotation\"\n"
    " C: \"OO\",130,103\n C: \"OP\",130,103, \"Weight\"\n C: \"OP\",131,130, \"d|Weight\"\n"
    " C: \"OO\",115,103\n C: \"OP\",115,1, \"Lcl Translation\"\n"
    " C: \"OO\",118,103\n C: \"OP\",118,1, \"Lcl Scaling\"\n"
    " C: \"OO\",119,103\n C: \"OP\",119,1, \"Lcl Rotation\"\n"
    " C: \"OO\",116,104\n C: \"OP\",116,1, \"Glow\"\n C: \"OP\",117,116, \"d|Glow\"\n"
    " C: \"OO\",120,105\n C: \"OP\",120,1, \"Lcl Rotation\"\n"
    " C: \"OO\",121,105\n C: \"OP\",121,1, \"Lcl Scaling\"\n"
    " C: \"OO\",122,106\n C: \"OP\",122,1, \"Lcl Translation\"\n C: \"OO\",123,106\n C: "
    "\"OP\",123,1, \"Lcl Scaling\"\n"
    " C: \"OO\",124,107\n C: \"OP\",124,1, \"Lcl Translation\"\n"
    " C: \"OO\",301,300\n C: \"OO\",310,301\n C: \"OP\",310,1, \"Lcl Translation\"\n C: "
    "\"OP\",312,310, \"d|X\"\n C: \"OP\",311,310, \"d|Y\"\n C: \"OP\",313,310, \"d|Z\"\n"
    " C: \"OO\",601,600\n C: \"OO\",602,600\n C: \"OO\",610,601\n C: \"OP\",610,1, \"Lcl "
    "Scaling\"\n C: \"OO\",611,602\n C: \"OP\",611,1, \"Lcl Scaling\"\n"
    "}\n"
    "Takes: {\n Current: \"Walk\"\n Take: \"Other\" {\n  LocalTime: 1,2\n }\n Take: \"Walk\" {\n  "
    "ReferenceTime: "
    "5,10\n }\n Take: \"Jog\" {\n  LocalTime: 6,9\n  ReferenceTime: 1,2\n }\n}\n";

static char s_text[16384];

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static bool Near3(const double v[3], double x, double y, double z)
{
    return fabs(v[0] - x) <= 1e-9 && fabs(v[1] - y) <= 1e-9 && fabs(v[2] - z) <= 1e-9;
}

// A property of model A at a time from a static value.
static bool At(const maniFbx* fbx, const maniFbxAnimation* a, const char* name, double time,
               const double from[3], double out[3])
{
    memcpy(out, from, 3 * sizeof *out);
    return maniFbxAnimateProperty(a, fbx, maniFbxFindObject(fbx, 1), name, strlen(name), time, 0,
                                  out);
}

static maniFbxAnimation Stack(const maniFbx* fbx, uint32_t k)
{
    maniFbxAnimation a;
    maniImportResult r = maniFbxReadAnimation(fbx, maniFbxStackSource(fbx, k), &a);
    CHECK(r.result == mani_success, "a stack read");
    return a;
}

static void TestRanges(const maniFbx* fbx)
{
    static const int64_t kRanges[6][2] = {{0, 46186158000}, {5, 10}, {4618615800, 34639618500},
                                          {3, 4},           {6, 9},  {0, 0}};
    bool right = true;
    for (uint32_t k = 0; k < 6; ++k)
    {
        maniFbxAnimation a = Stack(fbx, k);
        right = right && a.start == kRanges[k][0] && a.stop == kRanges[k][1];
        maniFbxReleaseAnimation(fbx, &a);
    }
    CHECK(right, "ranges: local, a take's reference, the keys, reference, a take's local");
}

// A stack's range from its take, as ufbx reads it: a stack with no
// properties of its own takes its take's times ahead of its template's
// (LocalStart 3, LocalStop 4); with properties of its own, never; a
// take's reference times come after the template's local ones.
static bool RangeIs(const char* text, uint32_t k, int64_t start, int64_t stop)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    maniFbxAnimation a;
    bool right = maniOpenFbx(builder, text, strlen(text), &fbx).result == mani_success &&
                 maniFbxReadAnimation(fbx, maniFbxStackSource(fbx, k), &a).result == mani_success;
    if (right)
    {
        right = a.start == start && a.stop == stop;
        maniFbxReleaseAnimation(fbx, &a);
    }
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return right;
}

static void TestTakeRanges(void)
{
    static const char kTemplate[] =
        "; FBX 7.4.0 project file\n"
        "Definitions: {\n ObjectType: \"AnimationStack\" {\n  PropertyTemplate: \"FbxAnimStack\" "
        "{\n"
        "   Properties70: {\n    P: \"LocalStart\", \"KTime\", \"Time\", \"\",3\n"
        "    P: \"LocalStop\", \"KTime\", \"Time\", \"\",4\n   }\n  }\n }\n}\n"
        "Objects: {\n AnimationStack: 100, \"AnimStack::Bare\", \"\" {\n }\n"
        " AnimationStack: 200, \"AnimStack::Ref\", \"\" {\n }\n"
        " AnimationStack: 300, \"AnimStack::Own\", \"\" {\n  Properties70: {\n   P: "
        "\"Description\", \"KString\", \"\", \"\", \"x\"\n  }\n }\n"
        " AnimationStack: 400, \"AnimStack::Empty\", \"\" {\n  Properties70: {\n  }\n }\n}\n"
        "Takes: {\n Take: \"Bare\" {\n  LocalTime: 6,9\n }\n Take: \"Ref\" {\n  ReferenceTime: "
        "1,2\n"
        " }\n Take: \"Own\" {\n  LocalTime: 6,9\n }\n Take: \"Empty\" {\n  LocalTime: 6,9\n }\n}\n";
    static const char kNoTemplate[] =
        "; FBX 7.4.0 project file\n"
        "Objects: {\n AnimationStack: 300, \"AnimStack::Own\", \"\" {\n  Properties70: {\n   P: "
        "\"Description\", \"KString\", \"\", \"\", \"x\"\n  }\n }\n}\n"
        "Takes: {\n Take: \"Own\" {\n  LocalTime: 6,9\n }\n}\n";
    CHECK(RangeIs(kTemplate, 0, 6, 9), "a bare stack's take ahead of its template");
    CHECK(RangeIs(kTemplate, 1, 3, 4), "the template's local times ahead of a take's reference");
    CHECK(RangeIs(kTemplate, 2, 3, 4), "a stack with properties of its own: no take");
    CHECK(RangeIs(kTemplate, 3, 6, 9), "an empty Properties70 is none of its own");
    CHECK(RangeIs(kNoTemplate, 0, 0, 0), "no take there with no template: the keys' span");
}

// A stack that is the file's first object (object 0) is named as any.
static void TestFirstObject(void)
{
    static const char kText[] =
        "; FBX 7.4.0 project file\n"
        "Objects: {\n AnimationStack: 100, \"AnimStack::Walk\", \"\" {\n }\n}\n";
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    uint32_t length = 0;
    const char* name = maniOpenFbx(builder, kText, strlen(kText), &fbx).result == mani_success
                           ? maniFbxClipName(fbx, 0, &length)
                           : NULL;
    CHECK(name != NULL && length == 4 && memcmp(name, "Walk", 4) == 0, "object 0's stack named");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestComponents(const maniFbx* fbx)
{
    maniFbxAnimation a = Stack(fbx, 2);
    double v[3];
    const double zero[3] = {0, 0, 0};
    CHECK(At(fbx, &a, "Lcl Translation", 0.5, zero, v) && Near3(v, 1, 0.5, 2.0 / 3.0),
          "curves on d|X, d|Y and d|Z");
    maniFbxReleaseAnimation(fbx, &a);
    a = Stack(fbx, 5);
    const double three[3] = {3, 3, 3};
    CHECK(At(fbx, &a, "Lcl Scaling", 0.5, three, v) && v[0] == 24.0 && v[1] == -24.0 &&
              v[2] == 24.0,
          "a full-weight scale exact, its sign kept");
    maniFbxReleaseAnimation(fbx, &a);
}

static void TestLayers(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, kScene, strlen(kScene), &fbx).result == mani_success, "opens");
    if (fbx == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(maniFbxStackCount(fbx) == 6 &&
              maniFbxStackSource(fbx, 1) == maniFbxFindObject(fbx, 200) &&
              maniFbxStackSource(fbx, 6) == MANI_FBX_NONE,
          "six stacks in object order");
    maniFbxAnimation a = Stack(fbx, 0);
    CHECK(a.layerCount == 7 && a.channelCount == 14, "seven layers, fourteen channels");
    CHECK(a.layers[1].blend == maniFbxBlendAdditive && a.layers[1].weight == 0.5 &&
              a.layers[2].blend == maniFbxBlendPassthrough &&
              a.layers[3].blend == maniFbxBlendOverride && a.layers[4].composeRotation == false &&
              a.layers[4].composeScale == false && a.layers[1].composeRotation &&
              a.layers[1].composeScale && a.layers[5].weight == 0.0 && a.layers[6].weight == 1.0,
          "layer modes and weights, clamped");
    double v[3];
    const double translation[3] = {1, 2, 3};
    const double scaling[3] = {2, 2, 2};
    const double none[3] = {0, 0, 0};
    // 5 2 3, plus 0.5 (0 4 0), then halfway (weight 0.5, not 0.25) to 10,
    // plus nothing (weight 0), plus z 1.
    CHECK(At(fbx, &a, "Lcl Translation", 0.5, translation, v) && Near3(v, 7.5, 7, 7.5),
          "translation");
    // At 0.9 s the animated weight is 2.1, so 1: all the way to 10.
    CHECK(At(fbx, &a, "Lcl Translation", 0.9, translation, v) && Near3(v, 10, 10, 11),
          "an animated weight clamped");
    // 2 times +-4^0.5, then 4^0.5 16^0.5 with the sign, plus 1, times
    // 0^0 (weight 0).
    CHECK(At(fbx, &a, "Lcl Scaling", 0.5, scaling, v) && Near3(v, 9, -7, 9), "scaling");
    // Half of (90, 0, 90) composed, halfway back to none, then z plus 10:
    // from quaternions worked independently.
    CHECK(At(fbx, &a, "Lcl Rotation", 0.5, none, v) &&
              Near3(v, 20.103909361017106, 14.123745145629, 30.103909361017106),
          "rotation");
    const double glow[3] = {5, 0, 0};
    CHECK(At(fbx, &a, "Glow", 0.5, glow, v) && v[0] == 9.0, "overridden");
    const double visible[3] = {1, 0, 0};
    CHECK(!At(fbx, &a, "Visibility", 0.5, visible, v) && v[0] == 1.0, "not animated");
    uint32_t first = 0;
    CHECK(maniFbxAnimatedRun(&a, maniFbxFindObject(fbx, 1), &first) == 13 &&
              maniFbxAnimatedRun(&a, maniFbxFindObject(fbx, 103), &first) == 1,
          "the runs of channels by target; none on the root");
    maniFbxReleaseAnimation(fbx, &a);
    TestRanges(fbx);
    TestComponents(fbx);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// The scene with a text replaced.
static void Replace(const char* from, const char* to)
{
    const char* at = strstr(kScene, from);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)(at - kScene), kScene, to, at + strlen(from));
}

static void ExpectRefusal(int64_t node, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_text, strlen(s_text), &fbx).result == mani_success, "opens");
    maniFbxAnimation a;
    maniImportResult r = fbx != NULL ? maniFbxReadAnimation(fbx, maniFbxStackSource(fbx, 0), &a)
                                     : (maniImportResult){mani_errorInvalid, 0, 0, 0};
    char at[32];
    snprintf(at, sizeof at, "AnimationCurveNode: %lld", (long long)node);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
              r.item == maniFbxFindObject(fbx, node) &&
              r.offset == (uint64_t)(strstr(s_text, at) - s_text),
          what);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    // Base gets the scale node 113, then the translation node 112: the
    // second translation lies past another property.
    Replace(" C: \"OO\",112,102\n", " C: \"OO\",113,101\n C: \"OO\",112,101\n");
    ExpectRefusal(112, "a property driven twice in a layer");
    Replace("P: \"d|Glow\", \"Number\", \"\", \"A\",5",
            "P: \"d|Glow\", \"Number\", \"\", \"A\",\"x\"");
    ExpectRefusal(116, "a channel's value not a number");
}

int main(void)
{
    TestLayers();
    TestTakeRanges();
    TestFirstObject();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
