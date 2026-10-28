// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The IK nodes: a two-bone node's output is its input solved
// as maniSolveTwoBone solves it, its goal and pole read from vec3
// parameters, its twist, soften and weight numbers or float parameters
// (soften and weight clamped to [0, 1], a weight of 0 passing the
// input); an aim node's as maniSolveAim's, its shares equal when left
// out, its cone clamped to [0, pi]; both inside a subgraph through its
// parameter mapping; refused at build (differences, inputs, bones,
// parameters, numbers) and at open at the first wrong word; a crafted
// file whose bones are not each an ancestor of the next passing its
// input. Run as test_graph_ik_node --seed <graph file> <json file> it
// writes a graph of both nodes, its file and text, seeds of the fuzz
// targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_fixture.h"

#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"

#include <stdio.h>

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[2][4096];
static _Alignas(16) uint8_t s_crafted[16384];
static maniPose* s_pose;
static maniPose* s_out;
static maniGraphParameterSource s_parameters[5];
static maniTransform s_walk[3];

static const uint32_t kFirst[1] = {0};
static const uint32_t kChain[3] = {0, 1, 2};

static maniGraphValue Vec(float x, float y, float z)
{
    return (maniGraphValue){.f = {x, y, z, 0.0f}};
}

// Parameters: 0 the goal and 1 the pole (vec3), 2 and 3 floats, 4 a
// bool.
static maniGraphSource IkSource(const maniGraphNodeSource* nodes, uint32_t count)
{
    s_parameters[0] = Parameter("goal", maniGraphParameterVec3, Vec(0.6f, 1.2f, 0.3f));
    s_parameters[1] = Parameter("pole", maniGraphParameterVec3, Vec(0.0f, 0.0f, 1.0f));
    s_parameters[2] = Parameter("a", maniGraphParameterFloat, Float(0.25f));
    s_parameters[3] = Parameter("b", maniGraphParameterFloat, Float(1.0f));
    s_parameters[4] = Parameter("lean", maniGraphParameterBool, Int(0));
    return (maniGraphSource){.parameters = s_parameters,
                             .nodes = nodes,
                             .parameterCount = 5,
                             .nodeCount = count,
                             .output = count - 1};
}

static maniGraphNodeSource TwoBoneNode(const uint32_t* input, maniGraphTwoBoneNodeSource s)
{
    return (maniGraphNodeSource){maniGraphNodeTwoBone, 1, input, .twoBone = s};
}

static maniGraphNodeSource AimNode(const uint32_t* input, maniGraphAimNodeSource s)
{
    return (maniGraphNodeSource){maniGraphNodeAim, 1, input, .aim = s};
}

// The knee: hips, spine and arm toward the goal and pole parameters,
// twisted by a, softened at 0.8, weighted by b.
static const maniGraphTwoBoneNodeSource kKnee = {
    0, 1, 2, 0, 1, {0.0f, 0.0f, 0.0f}, {2, 0.0f}, LITERAL(0.8f), {3, 0.0f}};

// A look-at over the three bones, the arm's x aiming and its y kept
// toward the pole, the cone a, the weight b.
static const maniGraphAimNodeSource kLook = {kChain,
                                             NULL,
                                             3,
                                             0,
                                             1,
                                             {1.0f, 0.0f, 0.0f},
                                             {0.0f, 0.1f, 0.0f},
                                             {0.0f, 1.0f, 0.0f},
                                             LITERAL(0.1f),
                                             {2, 0.0f},
                                             {3, 0.0f}};

static void Evaluate(maniGraphInstance* instance)
{
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_out) == mani_success,
          "evaluated");
}

static bool Same(maniTransform a, maniTransform b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

// Whether the output's bones are bit for bit those of s_pose, or of the
// walk when walk is true.
static bool Matches(bool walk)
{
    bool same = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        same = same && Same(maniPoseLocal(s_out, b), walk ? s_walk[b] : maniPoseLocal(s_pose, b));
    }
    return same;
}

// The walk alone 0.3 seconds in, the input the nodes solve.
static void Walk(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraphInstance* instance = Make(IkSource(nodes, 1), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    for (uint32_t b = 0; b < 3; ++b)
    {
        s_walk[b] = maniPoseLocal(s_out, b);
    }
}

static void Rest(void)
{
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniPoseSetLocal(s_pose, b, s_walk[b]);
    }
}

// s_pose: the walk solved as the knee's node solves it, with the
// parameters' defaults (pole and twist as given).
static void TwoBone(maniVec3 pole, float twist, float soften, float weight)
{
    Rest();
    maniTwoBoneSolve s = {0,     1,      2,     {0.6f, 1.2f, 0.3f}, pole, {0.0f, 0.0f, 0.0f},
                          twist, soften, weight};
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &s, NULL) == mani_success, "the reference");
}

static void Aim(const float* shares, float cone, float weight)
{
    Rest();
    maniAimSolve s = {{0, 1, 2},
                      {shares[0], shares[1], shares[2]},
                      3,
                      {0.6f, 1.2f, 0.3f},
                      {0.0f, 0.0f, 1.0f},
                      {1.0f, 0.0f, 0.0f},
                      {0.0f, 0.1f, 0.0f},
                      {0.0f, 1.0f, 0.0f},
                      0.1f,
                      cone,
                      weight};
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "the reference");
}

// The knee's node: the solve of its input; a weight of 0 the walk, 0.5
// half the correction, 3 clamped to 1; no pole the chain's own plane; a
// softening parameter below 0 clamped to 0.
static void TestTwoBone(void)
{
    static const maniVec3 kPole = {0.0f, 0.0f, 1.0f};
    static const maniVec3 kNone = {0.0f, 0.0f, 0.0f};
    maniGraph g;
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    TwoBoneNode(kFirst, kKnee)};
    maniGraphInstance* instance = Make(IkSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    TwoBone(kPole, 0.25f, 0.8f, 1.0f);
    CHECK(Matches(false) && !Matches(true), "the walk solved");
    CHECK(!maniPoseIsDifference(s_out), "a pose");
    Set(instance, 3, maniGraphParameterFloat, Float(0.0f));
    Evaluate(instance);
    CHECK(Matches(true), "weight 0: the walk");
    Set(instance, 3, maniGraphParameterFloat, Float(0.5f));
    Evaluate(instance);
    TwoBone(kPole, 0.25f, 0.8f, 0.5f);
    CHECK(Matches(false), "weight 0.5");
    Set(instance, 3, maniGraphParameterFloat, Float(3.0f));
    Evaluate(instance);
    TwoBone(kPole, 0.25f, 0.8f, 1.0f);
    CHECK(Matches(false), "weight 3 clamped to 1");
    maniGraphTwoBoneNodeSource free = kKnee;
    free.pole = MANI_NO_PARAMETER;
    free.twist = LITERAL(0.0f);
    free.soften = (maniGraphScalar){2, 0.0f};
    nodes[1] = TwoBoneNode(kFirst, free);
    instance = Make(IkSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Set(instance, 2, maniGraphParameterFloat, Float(-1.0f));
    Evaluate(instance);
    TwoBone(kNone, 0.0f, 0.0f, 1.0f);
    CHECK(Matches(false), "no pole, softening -1 clamped to 0");
    maniGraphTwoBoneNodeSource hinged = kKnee;
    hinged.hinge = (maniVec3){0.6f, 0.0f, 0.8f};
    nodes[1] = TwoBoneNode(kFirst, hinged);
    instance = Make(IkSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Rest();
    maniTwoBoneSolve h = {0, 1, 2, {0.6f, 1.2f, 0.3f}, kPole, hinged.hinge, 0.25f, 0.8f, 1.0f};
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &h, NULL) == mani_success, "the reference");
    bool matched = Matches(false);
    TwoBone(kPole, 0.25f, 0.8f, 1.0f);
    CHECK(matched && !Matches(false), "a hinge, unlike none");
}

// The look-at's node: the solve of its input, its shares equal when
// left out; a cone of 5 clamped to pi, -1 to 0 (no cone); weight 0 the
// walk.
static void TestAim(void)
{
    static const float kEqual[3] = {1.0f, 1.0f, 1.0f};
    static const float kSpine[3] = {0.0f, 2.0f, 1.0f};
    maniGraph g;
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat), AimNode(kFirst, kLook)};
    maniGraphInstance* instance = Make(IkSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Aim(kEqual, 0.25f, 1.0f);
    CHECK(Matches(false) && !Matches(true), "the walk aimed, shares equal");
    Set(instance, 2, maniGraphParameterFloat, Float(5.0f));
    Evaluate(instance);
    Aim(kEqual, MANI_AIM_MAX_CONE, 1.0f);
    CHECK(Matches(false), "cone 5 clamped to pi");
    Set(instance, 2, maniGraphParameterFloat, Float(-1.0f));
    Evaluate(instance);
    Aim(kEqual, 0.0f, 1.0f);
    CHECK(Matches(false), "cone -1 clamped to 0");
    Set(instance, 3, maniGraphParameterFloat, Float(0.0f));
    Evaluate(instance);
    CHECK(Matches(true), "weight 0: the walk");
    maniGraphAimNodeSource shared = kLook;
    shared.shares = kSpine;
    nodes[1] = AimNode(kFirst, shared);
    instance = Make(IkSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Aim(kSpine, 0.25f, 1.0f);
    CHECK(Matches(false), "shares as given");
}

// Both nodes in a subgraph read the outer goal and weight their own
// map to, at other places.
static void TestSubgraph(void)
{
    static maniGraphParameterSource inner[2];
    static maniGraphNodeSource innerNodes[3];
    static const uint32_t kMap[2] = {3, 0};
    static const uint32_t kSecond[1] = {1};
    inner[0] = Parameter("w", maniGraphParameterFloat, Float(0.0f));
    inner[1] = Parameter("target", maniGraphParameterVec3, Vec(0.0f, 0.0f, 0.0f));
    maniGraphTwoBoneNodeSource knee = kKnee;
    knee.goal = 1;
    knee.pole = MANI_NO_PARAMETER;
    knee.twist = LITERAL(0.0f);
    knee.weight = LITERAL(1.0f);
    maniGraphAimNodeSource look = kLook;
    look.bones = &kChain[2];
    look.count = 1;
    look.pole = MANI_NO_PARAMETER;
    look.goal = 1;
    look.cone = LITERAL(0.0f);
    look.weight = (maniGraphScalar){0, 0.0f};
    innerNodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    innerNodes[1] = TwoBoneNode(kFirst, knee);
    innerNodes[2] = AimNode(kSecond, look);
    static maniGraphSource sub;
    sub = (maniGraphSource){
        .parameters = inner, .nodes = innerNodes, .parameterCount = 2, .nodeCount = 3, .output = 2};
    sub.skeleton = &s_skeleton;
    sub.clips = kSlots;
    sub.clipCount = CLIPS;
    maniGraph g;
    maniGraphNodeSource nodes[1] = {(maniGraphNodeSource){(maniGraphNodeKind)maniGraphNodeSubgraph,
                                                          0, NULL, .subgraph = {&sub, kMap}}};
    maniGraphInstance* instance = Make(IkSource(nodes, 1), &g);
    Update(instance, 0.3f);
    Set(instance, 3, maniGraphParameterFloat, Float(0.5f));
    Evaluate(instance);
    Rest();
    maniTwoBoneSolve s = {
        0, 1, 2, {0.6f, 1.2f, 0.3f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 0.0f, 0.8f, 1.0f};
    maniAimSolve a = {{2},
                      {1.0f},
                      1,
                      {0.6f, 1.2f, 0.3f},
                      {0.0f, 0.0f, 0.0f},
                      {1.0f, 0.0f, 0.0f},
                      {0.0f, 0.1f, 0.0f},
                      {0.0f, 1.0f, 0.0f},
                      0.1f,
                      0.0f,
                      0.5f};
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &s, NULL) == mani_success &&
              maniSolveAim(s_pose, &s_skeleton, &a, NULL) == mani_success,
          "the reference");
    CHECK(Matches(false), "the outer goal and weight through the mapping");
}

static maniBuildResult Built(maniGraphSource source)
{
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, &source);
    maniDestroyBuilder(builder);
    return r;
}

static bool RefusedAt(maniBuildResult r, uint32_t node, maniBuildReason reason)
{
    return r.result == mani_errorInvalid && r.item == node && r.reason == reason;
}

static bool KneeRefused(maniGraphTwoBoneNodeSource s, maniBuildReason reason)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat), TwoBoneNode(kFirst, s)};
    return RefusedAt(Built(IkSource(nodes, 2)), 1, reason);
}

static bool LookRefused(maniGraphAimNodeSource s, maniBuildReason reason)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat), AimNode(kFirst, s)};
    return RefusedAt(Built(IkSource(nodes, 2)), 1, reason);
}

// Refused at build: differences in, no input or two; a two-bone node's
// bones past the skeleton or not each an ancestor of the next, a goal
// not a vec3 or none, a pole not a vec3, a hinge not finite, a twist not
// finite, a soften or weight outside [0, 1] or reading a bool.
static void TestTwoBoneRefusals(void)
{
    static const uint32_t kWave[1] = {0};
    static const uint32_t kPair[2] = {0, 1};
    maniGraphNodeSource nodes[3] = {ClipNode(2, 1.0f, maniGraphLoopRepeat),
                                    TwoBoneNode(kWave, kKnee)};
    CHECK(RefusedAt(Built(IkSource(nodes, 2)), 1, maniBuildReasonDifference), "differences in");
    // Not the output: the node itself refuses them.
    static const uint32_t kLayered[2] = {2, 1};
    static const maniGraphScalar kWhole[1] = {LITERAL(1.0f)};
    maniGraphNodeSource layered[4] = {
        ClipNode(2, 1.0f, maniGraphLoopRepeat), TwoBoneNode(kWave, kKnee),
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        (maniGraphNodeSource){maniGraphNodeAdditive, 2, kLayered, .additive = {kWhole}}};
    CHECK(RefusedAt(Built(IkSource(layered, 4)), 1, maniBuildReasonDifference),
          "differences in, as an additive's layer");
    nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    nodes[1] = ClipNode(1, 1.0f, maniGraphLoopRepeat);
    nodes[2] = (maniGraphNodeSource){maniGraphNodeTwoBone, 2, kPair, .twoBone = kKnee};
    CHECK(Built(IkSource(nodes, 3)).result == mani_errorInvalid, "two inputs");
    nodes[0] = (maniGraphNodeSource){maniGraphNodeTwoBone, 0, NULL, .twoBone = kKnee};
    CHECK(Built(IkSource(nodes, 1)).result == mani_errorInvalid, "no input");
    maniGraphTwoBoneNodeSource s = kKnee;
    s.end = 3;
    CHECK(KneeRefused(s, maniBuildReasonNode), "a bone past the skeleton");
    s = kKnee;
    s.start = 2;
    s.end = 0;
    CHECK(KneeRefused(s, maniBuildReasonNode), "bones not each an ancestor of the next");
    s = kKnee;
    s.middle = 0;
    CHECK(KneeRefused(s, maniBuildReasonNode), "a bone twice");
    s = kKnee;
    s.goal = 2;
    CHECK(KneeRefused(s, maniBuildReasonNode), "a goal reading a float");
    s.goal = MANI_NO_PARAMETER;
    CHECK(KneeRefused(s, maniBuildReasonNode), "no goal");
    s = kKnee;
    s.pole = 4;
    CHECK(KneeRefused(s, maniBuildReasonNode), "a pole reading a bool");
    s.pole = 5;
    CHECK(KneeRefused(s, maniBuildReasonNode), "a pole past the parameters");
    s = kKnee;
    s.hinge.y = NAN;
    CHECK(KneeRefused(s, maniBuildReasonNode), "a hinge not finite");
    s = kKnee;
    s.twist = LITERAL(INFINITY);
    CHECK(KneeRefused(s, maniBuildReasonScalar), "a twist not finite");
    s = kKnee;
    s.soften = LITERAL(1.5f);
    CHECK(KneeRefused(s, maniBuildReasonScalar), "a soften past 1");
    s = kKnee;
    s.weight = LITERAL(-0.5f);
    CHECK(KneeRefused(s, maniBuildReasonScalar), "a weight below 0");
    s.weight = (maniGraphScalar){4, 0.0f};
    CHECK(KneeRefused(s, maniBuildReasonScalar), "a weight reading a bool");
}

// Refused at build: an aim node of no bones, past MANI_AIM_MAX_BONES or
// with none given, its bones past the skeleton or not each an ancestor
// of the next, shares negative, not finite or all 0, a forward not of
// length 1, an offset or up not finite, a goal not a vec3, a cone past
// pi.
static void TestAimRefusals(void)
{
    maniGraphNodeSource nodes[4];
    static const uint32_t kNine[9] = {0, 1, 2, 2, 2, 2, 2, 2, 2};
    static const uint32_t kBack[2] = {1, 0};
    static const float kNegative[3] = {1.0f, -1.0f, 1.0f};
    static const float kZero[3] = {0.0f, 0.0f, 0.0f};
    static const float kNan[3] = {1.0f, NAN, 1.0f};
    static const float kHuge[3] = {3e38f, 3e38f, 3e38f};
    maniGraphAimNodeSource s = kLook;
    s.count = 0;
    CHECK(LookRefused(s, maniBuildReasonNode), "no bones");
    s.bones = kNine;
    s.count = 9;
    CHECK(LookRefused(s, maniBuildReasonNode), "nine bones");
    s = kLook;
    s.bones = NULL;
    CHECK(LookRefused(s, maniBuildReasonNode), "none given");
    s = kLook;
    s.bones = kBack;
    s.count = 2;
    CHECK(LookRefused(s, maniBuildReasonNode), "bones not each an ancestor of the next");
    static const uint32_t kPast[1] = {3};
    s = kLook;
    s.bones = kPast;
    s.count = 1;
    CHECK(LookRefused(s, maniBuildReasonNode), "a bone past the skeleton");
    static const uint32_t kLayered[2] = {2, 1};
    static const maniGraphScalar kWhole[1] = {LITERAL(1.0f)};
    nodes[0] = ClipNode(2, 1.0f, maniGraphLoopRepeat);
    nodes[1] = AimNode(kFirst, kLook);
    nodes[2] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    nodes[3] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kLayered, .additive = {kWhole}};
    CHECK(RefusedAt(Built(IkSource(nodes, 4)), 1, maniBuildReasonDifference),
          "differences in, as an additive's layer");
    s = kLook;
    s.bones = kNine + 2;
    s.count = 1;
    CHECK(Built(IkSource((maniGraphNodeSource[]){ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                                 AimNode(kFirst, s)},
                         2))
                  .result == mani_success,
          "one bone");
    const float* const shares[4] = {kNegative, kZero, kNan, kHuge};
    for (int k = 0; k < 4; ++k)
    {
        s = kLook;
        s.shares = shares[k];
        CHECK(LookRefused(s, maniBuildReasonNode), "shares negative, all 0, not finite, huge");
    }
    s = kLook;
    s.forward = (maniVec3){1.01f, 0.0f, 0.0f};
    CHECK(LookRefused(s, maniBuildReasonNode), "a forward not of length 1");
    s.forward = (maniVec3){NAN, 0.0f, 0.0f};
    CHECK(LookRefused(s, maniBuildReasonNode), "a forward not finite");
    s = kLook;
    s.offset.z = INFINITY;
    CHECK(LookRefused(s, maniBuildReasonNode), "an offset not finite");
    s = kLook;
    s.up.x = NAN;
    CHECK(LookRefused(s, maniBuildReasonNode), "an up not finite");
    s = kLook;
    s.goal = 1;
    CHECK(Built(IkSource((maniGraphNodeSource[]){ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                                 AimNode(kFirst, s)},
                         2))
                  .result == mani_success,
          "the goal any vec3");
    s.goal = 3;
    CHECK(LookRefused(s, maniBuildReasonNode), "a goal reading a float");
    s = kLook;
    s.cone = LITERAL(3.2f);
    CHECK(LookRefused(s, maniBuildReasonScalar), "a cone past pi");
}

// The payload hash and identity made right after an edit.
static void Rehash(uint8_t* file, size_t size)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + MANI_CONTAINER_HEADER_SIZE,
                                          size - MANI_CONTAINER_HEADER_SIZE));
    maniContainer c;
    CHECK(maniOpenContainer(file, size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
              mani_success,
          "container");
    const uint8_t* sections[MANI_GRAPH_SECTIONS];
    uint64_t sizes[MANI_GRAPH_SECTIONS];
    for (uint32_t t = 1; t <= MANI_GRAPH_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = file;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    maniStoreU64(file + 40, maniGraphComputeIdentity(sections, sizes));
}

static uint32_t Bits(float f)
{
    uint32_t bits = 0;
    memcpy(&bits, &f, sizeof bits);
    return bits;
}

// The file of node 1 over a walk, built and opened; the byte offset of
// its first data word.
static size_t Built1(maniGraphNodeSource node, size_t* sizeOut)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat), node};
    maniGraphSource source = IkSource(nodes, 2);
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    maniBuilder* builder = NewBuilder();
    maniGraph g;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, sizeOut) == mani_success &&
              maniOpenGraph(s_file, *sizeOut, &g).result == mani_success,
          "a graph of an IK node");
    maniDestroyBuilder(builder);
    uint32_t data = maniLoadU32(g.nodes + MANI_GRAPH_NODE_SIZE + 12);
    return (size_t)(g.data - s_file) + 4 * (size_t)data;
}

// Whether the file with word w of the node's data (from at) set to value
// is refused at word wrong (a scalar's first).
static bool WordRefused(size_t size, size_t at, uint32_t w, uint32_t value, uint32_t wrong)
{
    memcpy(s_crafted, s_file, size);
    maniStoreU32(s_crafted + at + 4 * (size_t)w, value);
    Rehash(s_crafted, size);
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, size, &crafted);
    return r.result == mani_errorFormat && r.offset == at + 4 * (size_t)wrong;
}

// A two-bone node's file refused at open at the first wrong word: a
// bone past the skeleton, a goal reading a float or none, a pole reading
// a bool, a hinge not finite, a twist not finite, a soften past 1, a
// weight reading a float with literal bits.
static void TestTwoBoneFile(void)
{
    size_t size = 0;
    size_t at = Built1(TwoBoneNode(kFirst, kKnee), &size);
    CHECK(maniLoadU32(s_file + at + 12) == 0 && maniLoadU32(s_file + at + 16) == 1,
          "goal and pole at words 3 and 4");
    CHECK(WordRefused(size, at, 0, 3, 0) && WordRefused(size, at, 1, 3, 1) &&
              WordRefused(size, at, 2, 7, 2),
          "a bone past the skeleton");
    CHECK(WordRefused(size, at, 3, 2, 3) && WordRefused(size, at, 3, MANI_NO_PARAMETER, 3) &&
              WordRefused(size, at, 3, 5, 3),
          "a goal reading a float, none or past");
    CHECK(WordRefused(size, at, 4, 4, 4), "a pole reading a bool");
    CHECK(WordRefused(size, at, 6, Bits(NAN), 6), "a hinge not finite");
    CHECK(WordRefused(size, at, 9, Bits(INFINITY), 8) && WordRefused(size, at, 8, 4, 8),
          "a twist not finite or reading a bool");
    CHECK(WordRefused(size, at, 11, Bits(1.5f), 10), "a soften past 1");
    CHECK(WordRefused(size, at, 13, Bits(1.0f), 12), "a weight's parameter with literal bits");
}

// An aim node's file refused at open at the first wrong word: no bones
// or nine, a bone past the skeleton, one past the count not 0, a share
// negative or past the count not 0, shares all 0, a forward not of
// length 1, an up not finite, a cone past pi.
static void TestAimFile(void)
{
    size_t size = 0;
    maniGraphAimNodeSource s = kLook;
    s.cone = LITERAL(0.5f);
    size_t at = Built1(AimNode(kFirst, s), &size);
    CHECK(maniLoadU32(s_file + at) == 3 && maniLoadU32(s_file + at + 4) == 0 &&
              maniLoadU32(s_file + at + 8) == 1,
          "count, goal and pole first");
    CHECK(WordRefused(size, at, 0, 0, 0) && WordRefused(size, at, 0, 9, 0), "no bones or nine");
    CHECK(WordRefused(size, at, 1, 2, 1) && WordRefused(size, at, 2, 4, 2),
          "a goal or pole of a type");
    CHECK(WordRefused(size, at, 3 + 1, 3, 3 + 1), "a bone past the skeleton");
    CHECK(WordRefused(size, at, 3 + 3, 1, 3 + 3), "a bone past the count");
    CHECK(WordRefused(size, at, 11 + 2, Bits(-1.0f), 11 + 2), "a share negative");
    CHECK(WordRefused(size, at, 11 + 7, Bits(1.0f), 11 + 7), "a share past the count");
    CHECK(WordRefused(size, at, 19, Bits(1.01f), 19), "a forward not of length 1");
    CHECK(WordRefused(size, at, 26, Bits(NAN), 26), "an up not finite");
    CHECK(WordRefused(size, at, 31, Bits(3.2f), 30), "a cone past pi");
    CHECK(WordRefused(size, at, 33, Bits(1.0f), 32), "a weight's parameter with literal bits");
    CHECK(maniLoadU32(s_file + at + 4 * 11) == Bits(1.0f) &&
              maniLoadU32(s_file + at + 4 * 13) == Bits(1.0f),
          "shares left out stored as 1");
    memcpy(s_crafted, s_file, size);
    for (uint32_t k = 0; k < 3; ++k)
    {
        maniStoreU32(s_crafted + at + 4 * (11 + (size_t)k), 0);
    }
    Rehash(s_crafted, size);
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, size, &crafted);
    CHECK(r.result == mani_errorFormat && r.offset == at + 4 * 11, "shares all 0");
}

// A crafted file whose IK node reads differences (its input clip made
// the additive wave, both nodes marked as giving differences) refused at
// open, as the build refuses its source.
static void TestDifferenceFile(void)
{
    maniGraphNodeSource nodes[2] = {TwoBoneNode(kFirst, kKnee), AimNode(kFirst, kLook)};
    for (int k = 0; k < 2; ++k)
    {
        size_t size = 0;
        (void)Built1(nodes[k], &size);
        maniGraph g;
        CHECK(maniOpenGraph(s_file, size, &g).result == mani_success, "opened");
        size_t records = (size_t)(g.nodes - s_file);
        size_t clipWord = (size_t)(g.data - s_file) + 4 * (size_t)maniLoadU32(g.nodes + 12);
        memcpy(s_crafted, s_file, size);
        maniStoreU32(s_crafted + clipWord, 2);
        maniStoreU16(s_crafted + records + 2, 1);
        maniStoreU16(s_crafted + records + MANI_GRAPH_NODE_SIZE + 2, 1);
        Rehash(s_crafted, size);
        maniGraph crafted;
        CHECK(maniOpenGraph(s_crafted, size, &crafted).result == mani_errorFormat,
              "differences into an IK node refused");
        maniStoreU16(s_crafted + records + MANI_GRAPH_NODE_SIZE + 2, 0);
        Rehash(s_crafted, size);
        CHECK(maniOpenGraph(s_crafted, size, &crafted).result == mani_errorFormat,
              "a pose node over differences refused");
    }
}

// A crafted two-bone node whose bones are not each an ancestor of the
// next opens (the file holds no parents) and passes its input.
static void TestCraftedChain(void)
{
    size_t size = 0;
    size_t at = Built1(TwoBoneNode(kFirst, kKnee), &size);
    maniStoreU32(s_file + at, 2);
    maniStoreU32(s_file + at + 8, 0);
    Rehash(s_file, size);
    maniGraph g;
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(maniOpenGraph(s_file, size, &g).result == mani_success &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, CLIPS, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "opened and bound");
    Update(instance, 0.3f);
    Evaluate(instance);
    CHECK(Matches(true), "the input passed");
}

static bool WriteFile(const char* path, const void* data, size_t size)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return false;
    }
    size_t written = fwrite(data, 1, size, out);
    return fclose(out) == 0 && written == size;
}

// A walk through the knee's node, then the look-at's, as a file and as
// JSON text.
static bool Seed(const char* graphPath, const char* textPath)
{
    static const uint32_t kSecond[1] = {1};
    static char text[8192];
    maniGraphNodeSource nodes[3] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    TwoBoneNode(kFirst, kKnee), AimNode(kSecond, kLook)};
    maniGraphSource source = IkSource(nodes, 3);
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    size_t size = 0;
    size_t length = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, text, sizeof text, &length) == mani_success &&
              WriteFile(graphPath, s_file, size) && WriteFile(textPath, text, length);
    maniDestroyBuilder(builder);
    return ok;
}

int main(int argc, char** argv)
{
    Open();
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        return Seed(argv[2], argv[3]) && s_failures == 0 ? 0 : 1;
    }
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[0], sizeof s_poseMemory[0], &s_pose) ==
                  mani_success &&
              maniInitPose(&s_skeleton, s_poseMemory[1], sizeof s_poseMemory[1], &s_out) ==
                  mani_success,
          "poses");
    Walk();
    TestTwoBone();
    TestAim();
    TestSubgraph();
    TestTwoBoneRefusals();
    TestAimRefusals();
    TestTwoBoneFile();
    TestAimFile();
    TestDifferenceFile();
    TestCraftedChain();
    return s_failures == 0 ? 0 : 1;
}
