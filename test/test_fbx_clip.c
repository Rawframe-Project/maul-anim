// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX clips (mani-0016): a rig sampled at 30 a second, its bones' locals
// against values derived by hand (a translation and rotation by linear
// curves, a uniform scale, and a scale-ignoring child whose local comes
// from the worlds), the placement folded or reported, a float track, the
// channel bindings counted; and refusals.

#include "test_fbx_sweep.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Rig (Null, z 5, animated to 6) above Hips (y 1, x from 0 to 3, turned
// 17 about z), Spine
// (y 1, at rest turned 33 about z, turning from 0 to 90 and scaling 1 to
// 2, Glow 0 to 1; rotation order ZXY) and Arm (scale not inherited, x 1, turned 45 about z). Prop
// is no bone (its rotation order out of range, unread). Stacks: Walk, a
// second; Idle, nothing; Shear scales Spine in x alone; Inherit and Below
// take Arm's inherit type past either end; Back ends before it starts;
// Late ends at 1.01 s on a constant key of Glow, repeated after; Still
// moves only Prop; Active animates Arm's RotationActive; Props animates
// Spine's Tint (a colour), Missing and Lone (no entries) and Glow in two
// layers (the second adding a constant 0.5); Clash makes two float tracks of one name; Broken has a
// curve without times; Layered composes Spine's rotation in two layers; Frames keys Glow at NTSC's
// frame 3.
static const char kScene[] =
    "; FBX 7.4.0 project file\n"
    "Objects: {\n"
    " Model: 5, \"Model::Rig\", \"Null\" {\n  Properties70: {\n   P: \"Lcl Translation\", \"Lcl "
    "Translation\", \"\", \"A\",0,0,5\n  }\n }\n"
    " Model: 1, \"Model::Hips\", \"LimbNode\" {\n  Properties70: {\n   P: \"Lcl Rotation\", \"Lcl "
    "Rotation\", \"\", \"A\",0,0,17\n   P: \"Lcl Translation\", "
    "\"Lcl Translation\", \"\", \"A\",0,1,0\n  }\n }\n"
    " Model: 2, \"Model::Spine\", \"LimbNode\" {\n  Properties70: {\n   P: \"Lcl Rotation\", \"Lcl "
    "Rotation\", \"\", \"A\",0,0,33\n   P: \"Lcl Translation\", "
    "\"Lcl Translation\", \"\", \"A\",0,1,0\n   P: \"RotationOrder\", \"enum\", \"\", \"A\",4\n   "
    "P: \"Glow\", \"Number\", \"\", \"A\",0.5\n   P: \"Tint\", \"ColorRGB\", \"\", "
    "\"A\",0.1,0.2,0.3\n  }\n }\n"
    " Model: 3, \"Model::Arm\", \"LimbNode\" {\n  Properties70: {\n   P: \"InheritType\", "
    "\"enum\", \"\", \"A\",2\n   P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",1,0,0\n  "
    " P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,45\n  }\n }\n"
    " Model: 4, \"Model::Prop\", \"Null\" {\n  Properties70: {\n   P: \"Lcl Translation\", \"Lcl "
    "Translation\", \"\", \"A\",0,0,0\n   P: \"RotationOrder\", \"enum\", \"\", \"A\",9\n  }\n }\n"
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
    " AnimationCurveNode: 410, \"AnimCurveNode::InheritType\", \"\" {\n  Properties70: {\n   P: "
    "\"d|InheritType\", \"Number\", \"\", \"A\",2\n  }\n }\n"
    " AnimationCurve: 411, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 2,7\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationStack: 500, \"AnimStack::Back\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",46186158000\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",0\n  }\n }\n"
    " AnimationStack: 600, \"AnimStack::Late\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46648019580\n  }\n }\n"
    " AnimationLayer: 601, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 610, \"AnimCurveNode::Glow\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Glow\", \"Number\", \"\", \"A\",0.5\n  }\n }\n"
    " AnimationCurve: 611, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46648019580\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,9\n  }\n  KeyAttrFlags: *1 {\n   a: 2\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n  Post-Extrapolation: {\n   "
    "Type: 82\n   Repetition: -1\n  }\n }\n"
    " AnimationStack: 700, \"AnimStack::Still\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 701, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 710, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 711, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,4\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationStack: 800, \"AnimStack::Active\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 801, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 810, \"AnimCurveNode::RotationActive\", \"\" {\n  Properties70: {\n   P: "
    "\"d|RotationActive\", \"Number\", \"\", \"A\",1\n  }\n }\n"
    " AnimationCurve: 811, \"AnimCurve::\", \"\" {\n  KeyTime: *1 {\n   a: 0\n  }\n  "
    "KeyValueFloat: *1 {\n   a: 1\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: *4 "
    "{\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 1\n  }\n }\n"
    " AnimationStack: 900, \"AnimStack::Props\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 901, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationLayer: 902, \"AnimLayer::Add\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",100\n  }\n }\n"
    " AnimationCurveNode: 910, \"AnimCurveNode::Tint\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0.1\n   P: \"d|Y\", \"Number\", \"\", \"A\",0.2\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0.3\n  }\n }\n"
    " AnimationCurve: 912, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 913, \"AnimCurveNode::Missing\", \"\" {\n  Properties70: {\n   P: "
    "\"d|X\", \"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 914, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,2\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 915, \"AnimCurveNode::Lone\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Lone\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 916, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,3\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 917, \"AnimCurveNode::Glow\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Glow\", \"Number\", \"\", \"A\",0.5\n  }\n }\n"
    " AnimationCurve: 918, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationCurveNode: 919, \"AnimCurveNode::Glow\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Glow\", \"Number\", \"\", \"A\",0.5\n  }\n }\n"
    " AnimationCurve: 920, \"AnimCurve::\", \"\" {\n  KeyTime: *1 {\n   a: 0\n  }\n  "
    "KeyValueFloat: *1 {\n   a: 0.5\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 1\n  }\n }\n"
    " AnimationStack: 1000, \"AnimStack::Clash\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 1001, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 1010, \"AnimCurveNode::Tint\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurveNode: 1011, \"AnimCurveNode::Tint/0\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Tint/0\", \"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 1012, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationStack: 1100, \"AnimStack::Broken\", \"\" {\n  Properties70: {\n   P: "
    "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
    "\"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 1101, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 1110, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurve: 1111, \"AnimCurve::\", \"\" {\n  KeyValueFloat: *1 {\n   a: 0\n  }\n }\n"
    " AnimationStack: 1200, \"AnimStack::Below\", \"\" {\n  Properties70: {\n   P: \"LocalStart\", "
    "\"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", \"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 1201, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 1210, \"AnimCurveNode::InheritType\", \"\" {\n  Properties70: {\n   P: "
    "\"d|InheritType\", \"Number\", \"\", \"A\",2\n  }\n }\n"
    " AnimationCurve: 1211, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,46186158000\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 2,-3\n  }\n  KeyAttrFlags: *1 {\n   a: 4\n  }\n  KeyAttrDataFloat: "
    "*4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 {\n   a: 2\n  }\n }\n"
    " AnimationStack: 1300, \"AnimStack::Layered\", \"\" {\n  Properties70: {\n   P: "
    "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
    "\"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 1301, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationLayer: 1302, \"AnimLayer::Add\", \"\" {\n  Properties70: {\n   P: \"Weight\", "
    "\"Number\", \"\", \"A\",100\n  }\n }\n"
    " AnimationCurveNode: 1310, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",90\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationCurveNode: 1311, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: \"d|X\", "
    "\"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",90\n   P: \"d|Z\", "
    "\"Number\", \"\", \"A\",0\n  }\n }\n"
    " AnimationStack: 1400, \"AnimStack::Frames\", \"\" {\n  Properties70: {\n   P: "
    "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
    "\"A\",46186158000\n  }\n }\n"
    " AnimationLayer: 1401, \"AnimLayer::Base\", \"\" {\n }\n"
    " AnimationCurveNode: 1410, \"AnimCurveNode::Glow\", \"\" {\n  Properties70: {\n   P: "
    "\"d|Glow\", \"Number\", \"\", \"A\",0.5\n  }\n }\n"
    " AnimationCurve: 1411, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: 0,4623234416\n  }\n  "
    "KeyValueFloat: *2 {\n   a: 0,5\n  }\n  KeyAttrFlags: *1 {\n   a: 2\n  }\n  KeyAttrDataFloat: "
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
    " C: \"OO\",601,600\n C: \"OO\",610,601\n C: \"OP\",610,2, \"Glow\"\n C: \"OP\",611,610, "
    "\"d|Glow\"\n"
    " C: \"OO\",701,700\n C: \"OO\",710,701\n C: \"OP\",710,4, \"Lcl Translation\"\n C: "
    "\"OP\",711,710, \"d|X\"\n"
    " C: \"OO\",801,800\n C: \"OO\",810,801\n C: \"OP\",810,3, \"RotationActive\"\n C: "
    "\"OP\",811,810, \"d|RotationActive\"\n"
    " C: \"OO\",901,900\n C: \"OO\",902,900\n C: \"OO\",910,901\n C: \"OP\",910,2, \"Tint\"\n C: "
    "\"OP\",912,910, \"d|X\"\n C: \"OO\",913,901\n C: \"OP\",913,2, \"Missing\"\n C: "
    "\"OP\",914,913, \"d|Y\"\n C: \"OO\",915,901\n C: \"OP\",915,2, \"Lone\"\n C: \"OP\",916,915, "
    "\"d|Lone\"\n C: \"OO\",917,901\n C: \"OP\",917,2, \"Glow\"\n C: \"OP\",918,917, \"d|Glow\"\n "
    "C: \"OO\",919,902\n C: \"OP\",919,2, \"Glow\"\n C: \"OP\",920,919, \"d|Glow\"\n"
    " C: \"OO\",1001,1000\n C: \"OO\",1010,1001\n C: \"OP\",1010,2, \"Tint\"\n C: "
    "\"OO\",1011,1001\n C: \"OP\",1011,2, \"Tint/0\"\n C: \"OP\",1012,1011, \"d|Tint/0\"\n"
    " C: \"OO\",1101,1100\n C: \"OO\",1110,1101\n C: \"OP\",1110,1, \"Lcl Translation\"\n C: "
    "\"OP\",1111,1110, \"d|X\"\n"
    " C: \"OO\",1201,1200\n C: \"OO\",1210,1201\n C: \"OP\",1210,3, \"InheritType\"\n C: "
    "\"OP\",1211,1210, \"d|InheritType\"\n"
    " C: \"OO\",1301,1300\n C: \"OO\",1302,1300\n C: \"OO\",1310,1301\n C: \"OP\",1310,2, \"Lcl "
    "Rotation\"\n C: \"OO\",1311,1302\n C: \"OP\",1311,2, \"Lcl Rotation\"\n"
    " C: \"OO\",1401,1400\n C: \"OO\",1410,1401\n C: \"OP\",1410,2, \"Glow\"\n C: "
    "\"OP\",1411,1410, \"d|Glow\"\n"
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

// Skeleton 0 of the scene with the Shear stack hidden, as a skeleton
// from a file without that animation would be: no helper for Spine.
static bool PlainSkeleton(maniBuilder* builder, const maniFbxOptions* options, maniSkeleton* out)
{
    static char text[sizeof kScene];
    memcpy(text, kScene, sizeof kScene);
    char* stack = strstr(text, "AnimationStack: 300");
    if (stack == NULL)
    {
        return false;
    }
    memcpy(stack, "AnimationStacq", 14);
    maniFbx* fbx = Open(builder, text);
    bool built = fbx != NULL && Skeleton(builder, fbx, options, out);
    maniCloseFbx(fbx);
    return built;
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
    bool built = fbx != NULL && PlainSkeleton(builder, &options, &skeleton) &&
                 maniFbxBuildClip(
                     builder, fbx, 0, &skeleton, &options,
                     &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, &bound)
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

// Walk's hips travel 3 along x: taken out as root motion; the settings
// checked.
static void TestRootMotion(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    CHECK(fbx != NULL && PlainSkeleton(builder, &options, &skeleton), "skeleton");
    uint32_t hips = maniSkeletonFindBone(&skeleton, "Hips", 4);
    maniRootMotionSource travel = {hips, maniRootAxisX | maniRootAxisZ, 0, 0, 0, 0, 0, 0};
    maniImportClipOptions settings = maniDefaultImportClipOptions();
    settings.loop = maniLoopRepeat;
    settings.rootMotion = &travel;
    maniClip clip;
    maniRootMove move;
    CHECK(maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, &settings, NULL).result ==
                  mani_success &&
              OpenClip(builder, &clip) && maniClipLoopPolicy(&clip) == maniLoopRepeat &&
              maniClipRootMotion(&clip, 0.0f, 2.0f, &move) == mani_success &&
              Near(move.translation.x, 6.0) && Near(move.translation.z, 0.0),
          "two loops of Walk travel 6 along x");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success &&
              maniSampleClip(&clip, &skeleton, 0.5f, pose) == mani_success &&
              Near(maniPoseLocal(pose, hips).translation.x,
                   (double)maniSkeletonRest(&skeleton, hips).translation.x),
          "the hips in place");
    travel.bone = maniSkeletonBoneCount(&skeleton);
    CHECK(maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, &settings, NULL).result ==
                  mani_errorInvalid &&
              maniFbxBuildClip(builder, fbx, 0, &skeleton, &options, NULL, NULL).result ==
                  mani_errorInvalid,
          "misuses");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static void TestNames(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    uint32_t length = 0;
    const char* name = maniFbxClipName(fbx, 2, &length);
    CHECK(maniFbxClipCount(fbx) == 14 && name != NULL && length == 5 &&
              memcmp(name, "Shear", 5) == 0 && maniFbxClipName(fbx, 14, &length) == NULL &&
              length == 0 && maniFbxClipCount(NULL) == 0,
          "clips and their names");
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    CHECK(fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
              maniFbxBuildClip(builder, fbx, 1, &skeleton, &options,
                               &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopRepeat},
                               NULL)
                      .result == mani_success &&
              OpenClip(builder, &clip) && maniClipDuration(&clip) == 0.0f &&
              maniClipSampleCount(&clip) == 1,
          "an empty stack: one sample");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static char s_text[16384];

static void Replace(const char* from, const char* to)
{
    const char* at = strstr(kScene, from);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)(at - kScene), kScene, to, at + strlen(from));
}

// A clip built for a skeleton and opened.
static bool BuildFor(maniBuilder* builder, const maniFbx* fbx, uint32_t index,
                     const maniSkeleton* skeleton, const maniFbxOptions* options, maniClip* clip)
{
    return fbx != NULL &&
           maniFbxBuildClip(builder, fbx, index, skeleton, options,
                            &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                            NULL)
                   .result == mani_success &&
           OpenClip(builder, clip);
}

// A skeleton of three bones: Root (no model), Hips under it, Arm under
// Hips (in the file, under Spine).
static bool Custom(maniBuilder* builder, maniSkeleton* out)
{
    const maniTransform rest = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    const maniBoneSource bones[3] = {
        {"Root", 4, -1, rest}, {"Hips", 4, 0, rest}, {"Arm", 3, 1, rest}};
    maniSkeletonSource source = {bones, 3, 0};
    size_t size = 0;
    return maniBuildSkeleton(builder, &source).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, out).result == mani_success;
}

// A skeleton not from the file: Hips under a bone with no model takes
// its own transform; Arm under Hips its world relative to Hips'.
static void TestOtherSkeleton(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    maniSkeleton skeleton;
    maniClip clip;
    maniPose* pose = NULL;
    bool built = fbx != NULL && Custom(builder, &skeleton) &&
                 BuildFor(builder, fbx, 0, &skeleton, &options, &clip) &&
                 maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success;
    CHECK(built, "builds for another skeleton");
    bool right = built;
    for (int k = 0; k <= 30 && built; k += 5)
    {
        double t = (double)k / 30.0;
        double turn = 90.0 * t * 3.14159265358979323846 / 180.0;
        right = right && maniSampleClip(&clip, &skeleton, (float)t, pose) == mani_success;
        maniTransform h = maniPoseLocal(pose, 1);
        maniTransform a = maniPoseLocal(pose, 2);
        // Hips' own transform, the placement not folded into a bone that
        // is not the root.
        right = right && NearVec(h.translation, 3 * t, 1, 0);
        // T(0 1 0) R(90t) T(1 + t, 0, 0) R(45), relative to Hips.
        right = right && NearVec(a.translation, (1 + t) * cos(turn), 1 + (1 + t) * sin(turn), 0) &&
                AboutZ(a.rotation, 90 * t + 45) && NearVec(a.scale, 1, 1, 1);
    }
    CHECK(right, "own and relative locals");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// With helper bones: Spine#unscaled undoes Spine's scale; Arm hangs from
// it with its own transform, its translation scaled by Spine's.
static void TestHelpers(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    options.inheritHelpers = 1;
    maniSkeleton skeleton;
    maniClip clip;
    maniPose* pose = NULL;
    bool built = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
                 BuildFor(builder, fbx, 0, &skeleton, &options, &clip) &&
                 maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success;
    uint32_t helper = maniSkeletonFindBone(&skeleton, "Spine#unscaled", 14);
    uint32_t arm = maniSkeletonFindBone(&skeleton, "Arm", 3);
    CHECK(built && helper != MANI_NO_BONE && maniSkeletonParent(&skeleton, arm) == helper,
          "builds with helpers");
    bool right = built;
    for (int k = 0; k <= 30 && built; k += 5)
    {
        double t = (double)k / 30.0;
        right = right && maniSampleClip(&clip, &skeleton, (float)t, pose) == mani_success;
        maniTransform h = maniPoseLocal(pose, helper);
        maniTransform a = maniPoseLocal(pose, arm);
        right = right && NearVec(h.scale, 1 / (1 + t), 1 / (1 + t), 1 / (1 + t)) &&
                NearVec(h.translation, 0, 0, 0) && NearVec(a.translation, 1 + t, 0, 0) &&
                AboutZ(a.rotation, 45) && NearVec(a.scale, 1, 1, 1);
    }
    CHECK(right, "a helper's inverse scale, a child's own transform");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Walk's rate and sample count with a TimeMode given.
static bool RateIs(const char* mode, float rate, uint32_t samples)
{
    char settings[256];
    snprintf(settings, sizeof settings,
             "; FBX 7.4.0 project file\nGlobalSettings: {\n Properties70: {\n"
             "  P: \"TimeMode\", \"enum\", \"\", \"\",%s\n }\n}\n",
             mode);
    Replace("; FBX 7.4.0 project file\n", settings);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, s_text);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    bool right = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
                 BuildFor(builder, fbx, 0, &skeleton, &options, &clip) &&
                 maniClipSampleRate(&clip) == rate && maniClipSampleCount(&clip) == samples;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return right;
}

static void TestRates(void)
{
    CHECK(RateIs("11", 24.0f, 25), "the scene's 24 a second");
    CHECK(RateIs("12", 30.0f, 31), "30 for a scene's 1000 a second");
    CHECK(RateIs("1", 120.0f, 121), "the scene's 120 a second, the most taken");
    CHECK(RateIs("9", (float)(30000.0 / 1001.0), 31), "NTSC's rate");
}

static void TestEnds(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    bool opened = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton);
    CHECK(opened && BuildFor(builder, fbx, 4, &skeleton, &options, &clip) &&
              maniClipDuration(&clip) == 0.0f && maniClipSampleCount(&clip) == 1,
          "a stack ending before it starts: one sample");
    float glow = 0.0f;
    CHECK(opened && BuildFor(builder, fbx, 5, &skeleton, &options, &clip) &&
              maniClipDuration(&clip) == (float)1.01 && maniClipSampleCount(&clip) == 32 &&
              maniSampleClipFloats(&clip, maniClipDuration(&clip), &glow, 1, NULL) ==
                  mani_success &&
              glow == 9.0f,
          "the last sample at the stack's stop, on its key");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Replaces in s_text itself.
static void ReplaceAgain(const char* from, const char* to)
{
    static char copy[16384];
    snprintf(copy, sizeof copy, "%s", s_text);
    const char* at = strstr(copy, from);
    snprintf(s_text, sizeof s_text, "%.*s%s%s", (int)(at - copy), copy, to, at + strlen(from));
}

// A clip of a text for skeleton 0 under options, with a sample rate.
static bool BuildText(const char* text, uint32_t index, const maniFbxOptions* options, float rate,
                      maniBuilder** builderOut, maniSkeleton* skeleton, maniClip* clip)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, text);
    bool right =
        fbx != NULL && Skeleton(builder, fbx, options, skeleton) &&
        maniFbxBuildClip(builder, fbx, index, skeleton, options,
                         &(maniImportClipOptions){.sampleRate = rate, .loop = maniLoopClamp}, NULL)
                .result == mani_success &&
        OpenClip(builder, clip);
    maniCloseFbx(fbx);
    *builderOut = builder;
    return right;
}

static maniTransform LocalAt(const maniClip* clip, const maniSkeleton* skeleton, const char* bone,
                             float time)
{
    maniPose* pose = NULL;
    maniTransform none = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    if (maniInitPose(skeleton, s_poseMemory, sizeof s_poseMemory, &pose) != mani_success ||
        maniSampleClip(clip, skeleton, time, pose) != mani_success)
    {
        return none;
    }
    return maniPoseLocal(pose, maniSkeletonFindBone(skeleton, bone, strlen(bone)));
}

static void TestOptions(void)
{
    maniBuilder* builder = NULL;
    maniSkeleton skeleton;
    maniClip clip;
    maniFbxOptions options = maniDefaultFbxOptions();
    CHECK(BuildText(kScene, 0, &options, 10.0f, &builder, &skeleton, &clip) &&
              maniClipSampleRate(&clip) == 10.0f && maniClipSampleCount(&clip) == 11,
          "a rate given");
    maniDestroyBuilder(builder);
    // Units: a centimetre file in units of 2 cm.
    options.unitMeters = 0.02;
    CHECK(BuildText(kScene, 0, &options, 0.0f, &builder, &skeleton, &clip) &&
              NearVec(LocalAt(&clip, &skeleton, "Hips", 0.5f).translation, 0.75, 0.5, 0),
          "the unit applied to samples");
    maniDestroyBuilder(builder);
    // Hips scale-ignoring (RrSs) under Rig scaled 2: folded, the placement
    // is Rig's unscaled world and Hips' own transform takes Rig's scale.
    Replace("P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0,5\n",
            "P: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0,5\n"
            "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",2,2,2\n");
    ReplaceAgain("Model::Hips\", \"LimbNode\" {\n  Properties70: {\n",
                 "Model::Hips\", \"LimbNode\" {\n  Properties70: {\n"
                 "   P: \"InheritType\", \"enum\", \"\", \"\",1\n");
    options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    bool built = BuildText(s_text, 0, &options, 0.0f, &builder, &skeleton, &clip);
    maniTransform h = LocalAt(&clip, &skeleton, "Hips", 0.5f);
    CHECK(built && NearVec(h.translation, 3, 2, 5.5) && NearVec(h.scale, 2, 2, 2),
          "a scale-ignoring root folded under its parent's unscaled world");
    maniDestroyBuilder(builder);
}

// A 7.0 file's stacks read as 7.4's (not as 6.x takes).
static void TestVersion70(void)
{
    Replace("; FBX 7.4.0 project file\n", "; FBX 7.0.0 project file\n");
    maniBuilder* builder = NULL;
    maniSkeleton skeleton;
    maniClip clip;
    maniFbxOptions options = maniDefaultFbxOptions();
    bool built = BuildText(s_text, 0, &options, 0.0f, &builder, &skeleton, &clip);
    CHECK(built && NearVec(LocalAt(&clip, &skeleton, "Hips", 0.5f).translation, 1.5, 1, 0),
          "a 7.0 file's stacks");
    maniDestroyBuilder(builder);
}

// Another skeleton: Spine, its helper, and Hips under the helper.
static bool Helpered(maniBuilder* builder, maniSkeleton* out)
{
    const maniTransform rest = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
    const maniBoneSource bones[3] = {
        {"Spine", 5, -1, rest}, {"Spine#unscaled", 14, 0, rest}, {"Hips", 4, 1, rest}};
    maniSkeletonSource source = {bones, 3, 0};
    size_t size = 0;
    return maniBuildSkeleton(builder, &source).result == mani_success &&
           maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
           maniOpenSkeleton(s_skeletonFile, size, out).result == mani_success;
}

static void TestHelperFrame(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    bool built = fbx != NULL && Helpered(builder, &skeleton) &&
                 BuildFor(builder, fbx, 0, &skeleton, &options, &clip);
    bool right = built;
    for (int k = 0; k <= 30 && built; k += 10)
    {
        double t = (double)k / 30.0;
        double turn = 90.0 * t * 3.14159265358979323846 / 180.0;
        maniTransform h = LocalAt(&clip, &skeleton, "Hips", (float)t);
        // Relative to Spine's unscaled world: R(-90t) T(0 -1 0), no scale.
        right = right && NearVec(h.translation, -sin(turn), -cos(turn), 0) &&
                AboutZ(h.rotation, -90 * t) && NearVec(h.scale, 1, 1, 1);
    }
    CHECK(right, "a bone relative to a helper's unscaled world");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

static bool StillIsEmpty(uint8_t helpers)
{
    maniBuilder* builder = NULL;
    maniSkeleton skeleton;
    maniClip clip;
    maniFbxOptions options = maniDefaultFbxOptions();
    options.inheritHelpers = helpers;
    bool built = BuildText(kScene, 6, &options, 0.0f, &builder, &skeleton, &clip);
    size_t size = 0;
    (void)maniCopyBuilt(builder, NULL, 0, &size);
    maniClipSource empty = {
        .skeleton = &skeleton, .loop = maniLoopClamp, .sampleRate = 30.0f, .duration = 1.0f};
    size_t reference = 0;
    bool right = built && maniBuildClip(builder, &empty).result == mani_success &&
                 maniCopyBuilt(builder, NULL, 0, &reference) == mani_errorCapacity &&
                 size == reference;
    maniDestroyBuilder(builder);
    return right;
}

static void TestTracks(void)
{
    maniBuilder* builder = NULL;
    maniSkeleton skeleton;
    maniClip clip;
    maniFbxOptions options = maniDefaultFbxOptions();
    // Still moves no bone: its clip is the size of one with no tracks,
    // with helpers or without (the rests formed as the samples are).
    CHECK(StillIsEmpty(0) && StillIsEmpty(1), "no bone moved, no track");
    CHECK(BuildText(kScene, 7, &options, 0.0f, &builder, &skeleton, &clip) &&
              maniClipFloatTrackCount(&clip) == 0,
          "a transform input animated is no float track");
    maniDestroyBuilder(builder);
    bool built = BuildText(kScene, 8, &options, 0.0f, &builder, &skeleton, &clip);
    static const char* const kNames[8] = {"Spine/Tint/0",    "Spine/Tint/1",    "Spine/Tint/2",
                                          "Spine/Missing/0", "Spine/Missing/1", "Spine/Missing/2",
                                          "Spine/Lone",      "Spine/Glow"};
    static const double kAtHalf[8] = {0.5, 0.2, 0.3, 0, 1, 0, 1.5, 1.0};
    bool right = built && maniClipFloatTrackCount(&clip) == 8;
    float values[8] = {0};
    right = right && maniSampleClipFloats(&clip, 0.5f, values, 8, NULL) == mani_success;
    for (int k = 0; k < 8 && right; ++k)
    {
        uint32_t track = maniClipFindFloatTrack(&clip, kNames[k], strlen(kNames[k]));
        right = track != MANI_NO_TRACK && Near(values[track], kAtHalf[k]);
    }
    CHECK(right, "float tracks: a colour's components, entries missing, one name over layers");
    maniDestroyBuilder(builder);
    options.inheritHelpers = 1;
    CHECK(BuildText(kScene, 0, &options, 0.0f, &builder, &skeleton, &clip) &&
              maniClipFloatTrackCount(&clip) == 1,
          "no float tracks for helpers");
    maniDestroyBuilder(builder);
}

// Rx(90) composed with Ry(90), either sign.
static bool Composed(maniQuat q)
{
    double h = sqrt(0.5);
    // (x w)(y w) for halves: x = h h, y = h h, z = h h, w = h h.
    double v = h * h;
    bool plus = Near(q.x, v) && Near(q.y, v) && Near(q.z, v) && Near(q.w, v);
    bool minus = Near(q.x, -v) && Near(q.y, -v) && Near(q.z, -v) && Near(q.w, -v);
    return plus || minus;
}

static void TestTimes(void)
{
    maniBuilder* builder = NULL;
    maniSkeleton skeleton;
    maniClip clip;
    maniFbxOptions options = maniDefaultFbxOptions();
    CHECK(BuildText(kScene, 12, &options, 0.0f, &builder, &skeleton, &clip) &&
              Composed(LocalAt(&clip, &skeleton, "Spine", 0.5f).rotation),
          "layers composed in the model's rotation order");
    maniDestroyBuilder(builder);
    Replace("; FBX 7.4.0 project file\n",
            "; FBX 7.4.0 project file\nGlobalSettings: {\n Properties70: {\n"
            "  P: \"TimeMode\", \"enum\", \"\", \"\",9\n }\n}\n");
    bool built = BuildText(s_text, 13, &options, 0.0f, &builder, &skeleton, &clip);
    float values[1] = {0};
    // Sample 3's time on the clip's grid.
    float third = (float)(3.0 / (double)(float)(30000.0 / 1001.0));
    CHECK(built && maniSampleClipFloats(&clip, third, values, 1, NULL) == mani_success &&
              fabs((double)values[0] - 5.0) <= 1e-4,
          "a key on an NTSC frame lands on its sample");
    maniDestroyBuilder(builder);
}

// The whole scene's skeleton knows Shear's animation: Spine's
// non-uniform scale there gives Spine a helper bone, so Arm hangs from it
// with its own transform in every clip (its translation scaled by
// Spine's), the helper holding the inverse of Spine's scale, and Shear
// builds.
// Whether a scene of Spine (at rest) and Arm (scale not inherited,
// turned 45 about z) under it has a helper for Spine, its stacks Rest
// (nothing) and Bad, a second keying Spine's x scale at its start, middle
// and end and Arm's inherit type at its start.
static bool HelperFrom(const char* scales, const char* inherit)
{
    static const char kCurve[] =
        " AnimationCurve: %d, \"AnimCurve::\", \"\" {\n  KeyTime: *3 {\n   a: "
        "0,23093079000,46186158000\n  }\n  KeyValueFloat: *3 {\n   a: %s\n  }\n  KeyAttrFlags: "
        "*1 {\n   a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: *1 "
        "{\n   a: 3\n  }\n }\n";
    char scale[512];
    char type[512];
    snprintf(scale, sizeof scale, kCurve, 211, scales);
    snprintf(type, sizeof type, kCurve, 221, inherit);
    char text[3072];
    snprintf(
        text, sizeof text,
        "; FBX 7.4.0 project file\nObjects: {\n"
        " Model: 2, \"Model::Spine\", \"LimbNode\" {\n }\n"
        " Model: 3, \"Model::Arm\", \"LimbNode\" {\n  Properties70: {\n   P: \"InheritType\", "
        "\"enum\", \"\", \"\",2\n   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,45\n"
        "  }\n }\n"
        " AnimationStack: 100, \"AnimStack::Rest\", \"\" {\n }\n"
        " AnimationStack: 200, \"AnimStack::Bad\", \"\" {\n  Properties70: {\n   P: "
        "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
        "\"A\",46186158000\n  }\n }\n"
        " AnimationLayer: 201, \"AnimLayer::Base\", \"\" {\n }\n"
        " AnimationCurveNode: 210, \"AnimCurveNode::S\", \"\" {\n  Properties70: {\n   P: "
        "\"d|X\", \"Number\", \"\", \"A\",1\n   P: \"d|Y\", \"Number\", \"\", \"A\",1\n   P: "
        "\"d|Z\", \"Number\", \"\", \"A\",1\n  }\n }\n%s"
        " AnimationCurveNode: 220, \"AnimCurveNode::I\", \"\" {\n  Properties70: {\n   P: "
        "\"d|InheritType\", \"Number\", \"\", \"A\",0\n  }\n }\n%s"
        "}\nConnections: {\n C: \"OO\",2,0\n C: \"OO\",3,2\n C: \"OO\",201,200\n"
        " C: \"OO\",210,201\n C: \"OP\",210,2, \"Lcl Scaling\"\n C: \"OP\",211,210, \"d|X\"\n"
        " C: \"OO\",220,201\n C: \"OP\",220,3, \"InheritType\"\n"
        " C: \"OP\",221,220, \"d|InheritType\"\n}\n",
        scale, type);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, text);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    bool helper = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
                  maniSkeletonFindBone(&skeleton, "Spine#unscaled", 14) != MANI_NO_BONE;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return helper;
}

// A helper where Spine's scale is not uniform at a sample past the
// start and before the end alone; none from a stack the sampler refuses
// partway, whose worlds are partly another sample's.
static void TestRefusedHelper(void)
{
    CHECK(HelperFrom("1,2,1", "2,2,2"), "a helper from the middle of a stack");
    CHECK(!HelperFrom("1,1,1", "2,2,2"), "none where Spine's scale stays uniform");
    CHECK(!HelperFrom("2,2,2", "9,9,9"), "none from a stack refused at Arm, after Spine");
}

// A three-model chain (Top, Spine, and Arm inheriting Rrs) and one
// stack whose curve node gives a model's property the values given,
// with one curve on a component when one is named: whether Spine gets a
// helper.
// More objects for HelperIn's scene, placed before its stack, and their
// connections.
static const char* s_extraObjects = "";
static const char* s_extraLinks = "";

static bool HelperIn(const char* top, const char* spine, int arm, int model, const char* property,
                     double x, double y, double z, const char* component, const char* keys)
{
    char curve[512] = "";
    char link[128] = "";
    if (component[0] != '\0')
    {
        snprintf(curve, sizeof curve,
                 " AnimationCurve: 211, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: "
                 "0,46186158000\n  }\n  KeyValueFloat: *2 {\n   a: %s\n  }\n  KeyAttrFlags: *1 {\n"
                 "   a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: "
                 "*1 {\n   a: 2\n  }\n }\n",
                 keys);
        snprintf(link, sizeof link, " C: \"OP\",211,210, \"d|%s\"\n", component);
    }
    char text[8192];
    snprintf(
        text, sizeof text,
        "; FBX 7.4.0 project file\nObjects: {\n"
        " Model: 1, \"Model::Top\", \"Null\" {\n  Properties70: {\n%s  }\n }\n"
        " Model: 2, \"Model::Spine\", \"LimbNode\" {\n  Properties70: {\n%s  }\n }\n"
        " Model: 3, \"Model::Arm\", \"LimbNode\" {\n  Properties70: {\n   P: \"InheritType\", "
        "\"enum\", \"\", \"\",2\n   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,%d\n"
        "  }\n }\n%s"
        " AnimationStack: 200, \"AnimStack::Move\", \"\" {\n  Properties70: {\n   P: "
        "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
        "\"A\",46186158000\n  }\n }\n"
        " AnimationLayer: 201, \"AnimLayer::Base\", \"\" {\n }\n"
        " AnimationCurveNode: 210, \"AnimCurveNode::N\", \"\" {\n  Properties70: {\n   P: "
        "\"d|X\", \"Number\", \"\", \"A\",%g\n   P: \"d|Y\", \"Number\", \"\", \"A\",%g\n   P: "
        "\"d|Z\", \"Number\", \"\", \"A\",%g\n  }\n }\n%s"
        "}\nConnections: {\n C: \"OO\",1,0\n C: \"OO\",2,1\n C: \"OO\",3,2\n C: "
        "\"OO\",201,200\n C: \"OO\",210,201\n C: \"OP\",210,%d, \"%s\"\n%s%s}\n",
        top, spine, arm, s_extraObjects, x, y, z, curve, model, property, link, s_extraLinks);
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, text);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    bool helper = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
                  maniSkeletonFindBone(&skeleton, "Spine#unscaled", 14) != MANI_NO_BONE;
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
    return helper;
}

// The scales that may shear Arm under Spine in a stack, each found: a
// static one not uniform, a curve on one component, a rest one sheared
// by an animated rotation below it, a curve on the grandparent alone,
// and a curve in a later stack alone. Uniform ones never shear, and a
// stack refused elsewhere is skipped with its scales.
static void TestHelperScales(void)
{
    static const char kScaled[] = "   P: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",1,2,1\n";
    CHECK(HelperIn("", "", 45, 2, "Lcl Scaling", 1, 2, 1, "", ""),
          "a static scale not uniform in a stack: a helper");
    CHECK(!HelperIn("", "", 45, 2, "Lcl Scaling", 2, 2, 2, "", ""), "a static uniform scale: none");
    CHECK(HelperIn("", "", 45, 2, "Lcl Scaling", 1, 1, 1, "Y", "1,2"),
          "a scale curve on one component: a helper");
    CHECK(HelperIn("", kScaled, 0, 3, "Lcl Rotation", 0, 0, 0, "Z", "0,45"),
          "a rest scale sheared by an animated rotation: a helper");
    CHECK(!HelperIn("", kScaled, 0, 3, "Lcl Rotation", 0, 0, 0, "", ""),
          "the same rotation still: none");
    static const char kTurned[] = "   P: \"InheritType\", \"enum\", \"\", \"\",0\n"
                                  "   P: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,30\n";
    CHECK(HelperIn("", kTurned, 45, 1, "Lcl Scaling", 1, 1, 1, "Y", "1,2"),
          "a scale curve on the grandparent alone: a helper");
    // A first stack turning Spine without a scale.
    s_extraObjects =
        " AnimationStack: 100, \"AnimStack::Still\", \"\" {\n  Properties70: {\n   P: "
        "\"LocalStart\", \"KTime\", \"\", \"A\",0\n   P: \"LocalStop\", \"KTime\", \"\", "
        "\"A\",46186158000\n  }\n }\n"
        " AnimationLayer: 101, \"AnimLayer::Base\", \"\" {\n }\n"
        " AnimationCurveNode: 110, \"AnimCurveNode::R\", \"\" {\n  Properties70: {\n   P: "
        "\"d|X\", \"Number\", \"\", \"A\",0\n   P: \"d|Y\", \"Number\", \"\", \"A\",0\n   P: "
        "\"d|Z\", \"Number\", \"\", \"A\",30\n  }\n }\n";
    s_extraLinks = " C: \"OO\",101,100\n C: \"OO\",110,101\n C: \"OP\",110,2, \"Lcl Rotation\"\n";
    CHECK(HelperIn("", "", 45, 2, "Lcl Scaling", 1, 1, 1, "Y", "1,2"),
          "a scale curve in a later stack alone: a helper");
    CHECK(!HelperIn("", "", 45, 2, "Lcl Rotation", 0, 0, 0, "Z", "0,90"),
          "two stacks without a scale: none");
    // A curve on Top whose times run backward refuses the stack.
    s_extraObjects =
        " AnimationCurveNode: 120, \"AnimCurveNode::T\", \"\" {\n  Properties70: {\n   P: "
        "\"d|X\", \"Number\", \"\", \"A\",0\n  }\n }\n"
        " AnimationCurve: 121, \"AnimCurve::\", \"\" {\n  KeyTime: *2 {\n   a: "
        "46186158000,0\n  }\n  KeyValueFloat: *2 {\n   a: 0,1\n  }\n  KeyAttrFlags: *1 {\n"
        "   a: 4\n  }\n  KeyAttrDataFloat: *4 {\n   a: 0,0,0,0\n  }\n  KeyAttrRefCount: "
        "*1 {\n   a: 2\n  }\n }\n";
    s_extraLinks = " C: \"OO\",120,201\n C: \"OP\",120,1, \"Lcl Translation\"\n C: "
                   "\"OP\",121,120, \"d|X\"\n";
    CHECK(!HelperIn("", "", 45, 2, "Lcl Scaling", 1, 1, 1, "Y", "1,2"),
          "a stack refused elsewhere: skipped, its scale curve with it");
    s_extraObjects = "";
    s_extraLinks = "";
}

static void TestAnimatedHelper(void)
{
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    maniClip clip;
    bool built =
        fbx != NULL && Skeleton(builder, fbx, &options, &skeleton) &&
        maniFbxBuildClip(builder, fbx, 0, &skeleton, &options,
                         &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, NULL)
                .result == mani_success &&
        OpenClip(builder, &clip);
    uint32_t spine = built ? maniSkeletonFindBone(&skeleton, "Spine", 5) : MANI_NO_BONE;
    uint32_t helper = built ? maniSkeletonFindBone(&skeleton, "Spine#unscaled", 14) : MANI_NO_BONE;
    uint32_t arm = built ? maniSkeletonFindBone(&skeleton, "Arm", 3) : MANI_NO_BONE;
    CHECK(helper != MANI_NO_BONE && maniSkeletonParent(&skeleton, helper) == spine &&
              maniSkeletonParent(&skeleton, arm) == helper,
          "a helper from the animation");
    maniPose* pose = NULL;
    bool right =
        built && maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success;
    for (int k = 0; right && k <= 30; ++k)
    {
        double t = (double)k / 30.0;
        right = maniSampleClip(&clip, &skeleton, (float)t, pose) == mani_success;
        maniTransform a = maniPoseLocal(pose, arm);
        maniTransform h = maniPoseLocal(pose, helper);
        // Its own transform, the translation scaled by Spine's inherited
        // scale (mani-0014).
        right = right && NearVec(a.translation, 1 + t, 0, 0) && AboutZ(a.rotation, 45) &&
                NearVec(a.scale, 1, 1, 1) &&
                NearVec(h.scale, 1 / (1 + t), 1 / (1 + t), 1 / (1 + t));
    }
    CHECK(right, "Arm its own transform under the helper");
    CHECK(built && maniFbxBuildClip(
                       builder, fbx, 2, &skeleton, &options,
                       &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, NULL)
                           .result == mani_success,
          "Shear builds");
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
    if (fbx != NULL && PlainSkeleton(builder, &options, &skeleton))
    {
        r = maniFbxBuildClip(builder, fbx, clipIndex, &skeleton, &options,
                             &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
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
    ExpectRefusal(9, maniImportReasonName, "Model: 2,", 2, "two float tracks of one name");
    ExpectRefusal(10, maniImportReasonProperty, "AnimationCurve: 1111", 10,
                  "a stack's curve refused, the clip as item");
    ExpectRefusal(11, maniImportReasonProperty, "Model: 3,", 3, "an inherit type below its range");
    Replace("a: 0,3\n", "a: 0,1e300\n");
    {
        maniBuilder* builder = NewBuilder();
        maniFbx* fbx = Open(builder, s_text);
        maniFbxOptions options = maniDefaultFbxOptions();
        maniSkeleton skeleton;
        maniImportResult r = {mani_errorInvalid, 0, 0, 0, 0};
        if (fbx != NULL && Skeleton(builder, fbx, &options, &skeleton))
        {
            r = maniFbxBuildClip(
                builder, fbx, 0, &skeleton, &options,
                &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, NULL);
        }
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonMatrix && r.item == 1,
              "a local past binary32's range");
        maniCloseFbx(fbx);
        maniDestroyBuilder(builder);
    }
    // Models in file order: Rig 0, Hips 1, Spine 2, Arm 3, Prop 4.
    ExpectRefusal(2, maniImportReasonMatrix, "Model: 3,", 3, "a sheared local");
    ExpectRefusal(3, maniImportReasonProperty, "Model: 3,", 3, "an inherit type past its range");
    maniBuilder* builder = NewBuilder();
    maniFbx* fbx = Open(builder, kScene);
    maniFbxOptions options = maniDefaultFbxOptions();
    maniSkeleton skeleton;
    bool opened = fbx != NULL && Skeleton(builder, fbx, &options, &skeleton);
    CHECK(opened &&
              maniFbxBuildClip(builder, fbx, 14, &skeleton, &options,
                               &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                               NULL)
                      .result == mani_errorInvalid &&
              maniFbxBuildClip(builder, fbx, 0, &skeleton, &options,
                               &(maniImportClipOptions){.sampleRate = -1.0f, .loop = maniLoopClamp},
                               NULL)
                      .result == mani_errorInvalid &&
              maniFbxBuildClip(builder, fbx, 0, NULL, &options,
                               &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                               NULL)
                      .result == mani_errorInvalid &&
              maniFbxBuildClip(NULL, fbx, 0, &skeleton, &options,
                               &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                               NULL)
                      .result == mani_errorInvalid,
          "misuses");
    maniCloseFbx(fbx);
    maniDestroyBuilder(builder);
}

// Every allocation failing in turn while opening the scene and building
// the Props clip (stacks, layers, curves, float tracks).
static void TestAllocationFailures(void)
{
    FbxSweep sweep = {kScene, 8, UINT32_MAX};
    CHECK(TestSweep(FbxSweepWork, &sweep, 8192), "allocation failures handled, nothing kept");
}

int main(void)
{
    TestAllocationFailures();
    CheckWalk(0);
    CheckWalk(1);
    TestAnimatedHelper();
    TestHelperScales();
    TestRefusedHelper();
    TestNames();
    TestRootMotion();
    TestOtherSkeleton();
    TestHelpers();
    TestRates();
    TestEnds();
    TestOptions();
    TestVersion70();
    TestHelperFrame();
    TestTracks();
    TestTimes();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
