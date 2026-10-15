// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX stacks (mani-0016): one model's properties through five layers
// (a base, an additive one, a passthrough one with an animated weight,
// an override and an additive one adding components), against values
// derived by hand from ufbx's rules; stack ranges from properties, a
// take and the keys; and a property driven twice in a layer refused.

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
// by 4 and turns 90 about z; Pass (weight 0.25, animated to 0.5) blends
// toward translation 10, scale 16 and no rotation; Over (override) sets
// Glow 9; Parts adds rotation x 10 and scale 1 by components. Walk has
// only a take (5 to 10 ticks); Span's one key pair runs 0.25 to 0.75 s.
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
    "\"Number\", \"\", \"A\",4\n   P: \"d|Y\", \"Number\", \"\", \"A\",4\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",4\n  }\n }\n"
    " AnimationCurveNode: 114, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",90\n  }\n }\n"
    " AnimationCurveNode: 130, \"AnimCurveNode::Weight\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Weight\", \"Number\", \"\", \"A\",25\n  }\n }\n"
    " AnimationCurve: 131, \"AnimCurve::\", \"\" {\n  KeyTime: *1 {\n   a: 0\n  }\n  "
    "KeyValueFloat: *1 {\n   a: 50\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 1\n  }\n }\n"
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
    "\"Number\", \"\", \"A\",10\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurveNode: 121, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
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
    "}\n"
    "Connections: {\n"
    " C: \"OO\",1,0\n"
    " C: \"OO\",101,100\n C: \"OO\",102,100\n C: \"OO\",103,100\n C: \"OO\",104,100\n C: "
    "\"OO\",105,100\n"
    " C: \"OO\",110,101\n C: \"OP\",110,1, \"Lcl Translation\"\n C: \"OP\",111,110, \"d|X\"\n"
    " C: \"OO\",112,102\n C: \"OP\",112,1, \"Lcl Translation\"\n"
    " C: \"OO\",113,102\n C: \"OP\",113,1, \"Lcl Scaling\"\n"
    " C: \"OO\",114,102\n C: \"OP\",114,1, \"Lcl Rotation\"\n"
    " C: \"OO\",130,103\n C: \"OP\",130,103, \"Weight\"\n C: \"OP\",131,130, \"d|Weight\"\n"
    " C: \"OO\",115,103\n C: \"OP\",115,1, \"Lcl Translation\"\n"
    " C: \"OO\",118,103\n C: \"OP\",118,1, \"Lcl Scaling\"\n"
    " C: \"OO\",119,103\n C: \"OP\",119,1, \"Lcl Rotation\"\n"
    " C: \"OO\",116,104\n C: \"OP\",116,1, \"Glow\"\n C: \"OP\",117,116, \"d|Glow\"\n"
    " C: \"OO\",120,105\n C: \"OP\",120,1, \"Lcl Rotation\"\n"
    " C: \"OO\",121,105\n C: \"OP\",121,1, \"Lcl Scaling\"\n"
    " C: \"OO\",301,300\n C: \"OO\",310,301\n C: \"OP\",310,1, \"Lcl Translation\"\n C: "
    "\"OP\",311,310, \"d|Y\"\n"
    "}\n"
    "Takes: {\n Take: \"Walk\" {\n  LocalTime: 5,10\n }\n}\n";

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

// The property of model A at 0.5 s from a static value.
static bool At(const maniFbx* fbx, const maniFbxAnimation* a, const char* name, double x, double y,
               double z, double out[3])
{
    out[0] = x;
    out[1] = y;
    out[2] = z;
    return maniFbxAnimateProperty(a, fbx, maniFbxFindObject(fbx, 1), name, strlen(name), 0.5, 0,
                                  out);
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
    CHECK(maniFbxStackCount(fbx) == 3 &&
              maniFbxStackObject(fbx, 1) == maniFbxFindObject(fbx, 200) &&
              maniFbxStackObject(fbx, 3) == MANI_FBX_NONE,
          "three stacks in object order");
    maniFbxAnimation a;
    maniImportResult r = maniFbxReadAnimation(fbx, maniFbxStackObject(fbx, 0), &a);
    CHECK(r.result == mani_success && a.layerCount == 5 && a.channelCount == 11,
          "five layers, eleven channels");
    if (r.result != mani_success)
    {
        maniCloseFbx(fbx);
        maniDestroyBuilder(builder);
        return;
    }
    CHECK(a.layers[1].blend == maniFbxBlendAdditive && a.layers[1].weight == 0.5 &&
              a.layers[2].blend == maniFbxBlendPassthrough &&
              a.layers[3].blend == maniFbxBlendOverride && a.layers[4].composeRotation == false &&
              a.layers[4].composeScale == false && a.layers[1].composeRotation &&
              a.layers[1].composeScale,
          "layer modes and weights");
    CHECK(a.start == 0 && a.stop == 46186158000, "the stack's local range");
    double v[3];
    // 5 2 3, plus 0.5 (0 4 0), then halfway (weight 0.5, not 0.25) to 10.
    CHECK(At(fbx, &a, "Lcl Translation", 1, 2, 3, v) && Near3(v, 7.5, 7, 6.5), "translation");
    // 2 times 4^0.5, then 4^0.5 16^0.5, then plus 1.
    CHECK(At(fbx, &a, "Lcl Scaling", 2, 2, 2, v) && Near3(v, 9, 9, 9), "scaling");
    // Half of 90 about z composed, halfway back to none, then x plus 10.
    CHECK(At(fbx, &a, "Lcl Rotation", 0, 0, 0, v) && Near3(v, 10, 0, 22.5), "rotation");
    CHECK(At(fbx, &a, "Glow", 5, 0, 0, v) && v[0] == 9.0, "overridden");
    CHECK(!At(fbx, &a, "Visibility", 1, 0, 0, v) && v[0] == 1.0, "not animated");
    uint32_t first = 0;
    CHECK(maniFbxAnimatedRun(&a, maniFbxFindObject(fbx, 1), &first) == 10 &&
              maniFbxAnimatedRun(&a, maniFbxFindObject(fbx, 103), &first) == 1,
          "the runs of channels by target");
    maniFbxReleaseAnimation(fbx, &a);
    CHECK(maniFbxReadAnimation(fbx, maniFbxStackObject(fbx, 1), &a).result == mani_success &&
              a.start == 5 && a.stop == 10,
          "a take's range");
    maniFbxReleaseAnimation(fbx, &a);
    CHECK(maniFbxReadAnimation(fbx, maniFbxStackObject(fbx, 2), &a).result == mani_success &&
              a.start == 11546539500 && a.stop == 34639618500,
          "the keys' range");
    maniFbxReleaseAnimation(fbx, &a);
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestTwice(void)
{
    const char* from = " C: \"OO\",112,102\n";
    const char* at = strstr(kScene, from);
    snprintf(s_text, sizeof s_text, "%.*s C: \"OO\",112,101\n%s", (int)(at - kScene), kScene,
             at + strlen(from));
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = NULL;
    CHECK(maniOpenFbx(builder, s_text, strlen(s_text), &fbx).result == mani_success, "opens");
    maniFbxAnimation a;
    maniImportResult r = fbx != NULL ? maniFbxReadAnimation(fbx, maniFbxStackObject(fbx, 0), &a)
                                     : (maniImportResult){mani_errorInvalid, 0, 0, 0, 0};
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonProperty &&
              r.item == maniFbxFindObject(fbx, 112) &&
              r.offset == (uint64_t)(strstr(s_text, "AnimationCurveNode: 112") - s_text),
          "a property driven twice in a layer");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

int main(void)
{
    TestLayers();
    TestTwice();
    return s_failures == 0 ? 0 : 1;
}
