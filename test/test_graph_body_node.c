// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body IK nodes over the hips, spine and arm: an
// XPBD node's output is its input solved as maniPose_SolveBodyXpbd solves it,
// an SDLS node's as maniPose_SolveBodySdls, bit for bit, the effectors' goals
// and rotation read from parameters, weights clamped (0 passing the
// input); settings carried; both inside a subgraph through its mapping;
// the room the instance keeps holding the node's need; the word count
// refusing counts out of range and words past those available; the JSON
// form read back to the same file; refused at build
// (differences, bones, parameters, numbers, settings, stretch for SDLS)
// and at open at the first wrong word; a crafted file whose bones make
// no part passing its input.
// Run as test_graph_body_node --seed <graph file> <json file> it writes
// a graph of both nodes, its file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "graph_body.h"
#include "graph_file.h"
#include "graph_fixture.h"
#include "graph_instance.h"
#include "negative_kinds.h"

#include "maul-anim/bodyik.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"

#include <stdio.h>

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(16) uint8_t s_bodyScratch[16384];
static _Alignas(32) uint8_t s_poseMemory[2][4096];
static _Alignas(16) uint8_t s_crafted[16384];
static _Alignas(16) uint8_t s_read[16384];
static char s_text[32768];
static maniPose* s_pose;
static maniPose* s_out;
static maniGraphParameterSource s_parameters[6];
static maniTransform s_walk[3];

static const uint32_t kFirst[1] = {0};

static maniGraphValue Vec(float x, float y, float z, float w)
{
    return (maniGraphValue){.f = {x, y, z, w}};
}

// Parameters: 0 the hand's goal and 1 the chest's (vec3), 2 and 3
// floats, 4 a bool, 5 the hand's rotation (quat).
static maniGraphSource BodySource(const maniGraphNodeSource* nodes, uint32_t count)
{
    s_parameters[0] = Parameter("hand", maniGraphParameterVec3, Vec(0.5f, 1.3f, 0.3f, 0.0f));
    s_parameters[1] = Parameter("chest", maniGraphParameterVec3, Vec(0.1f, 1.45f, 0.05f, 0.0f));
    s_parameters[2] = Parameter("a", maniGraphParameterFloat, Float(1.0f));
    s_parameters[3] = Parameter("b", maniGraphParameterFloat, Float(1.0f));
    s_parameters[4] = Parameter("lean", maniGraphParameterBool, Int(0));
    s_parameters[5] =
        Parameter("turn", maniGraphParameterQuat, Vec(0.0f, 0.0f, 0.38268343f, 0.92387953f));
    return (maniGraphSource){.parameters = s_parameters,
                             .nodes = nodes,
                             .parameterCount = 6,
                             .nodeCount = count,
                             .output = count - 1};
}

// The arm toward the hand's goal and turned by b toward the rotation;
// the spine half as hard toward the chest's, pulling its chain.
static const maniGraphEffectorSource kTwo[2] = {
    {2, 0, 5, LITERAL(1.0f), {3, 0.0f}, 1.0f, 0.0f},
    {1, 1, MANI_NO_PARAMETER, LITERAL(0.8f), LITERAL(0.0f), 0.5f, 0.25f}};
static const maniGraphEffectorSource kHand[1] = {
    {2, 0, MANI_NO_PARAMETER, LITERAL(1.0f), LITERAL(0.0f), 1.0f, 0.0f}};
static const maniBodyBone kSpine[1] = {
    {.bone = 1,
     .limit = {.kind = maniJointLimitCone, .cone = 0.6f, .twistLow = -0.2f, .twistHigh = 0.2f},
     .preferred = {0.0f, 0.0f, 0.1f},
     .rotationStiffness = 0.2f}};

// The XPBD node: the hips pinned, both effectors, the spine's setting,
// weighted by a. The SDLS node: the hips free, the hand alone.
static const maniGraphBodyNodeSource kXpbd = {.top = 0,
                                              .topMode = maniBodyTopPinned,
                                              .effectors = kTwo,
                                              .bones = kSpine,
                                              .effectorCount = 2,
                                              .boneCount = 1,
                                              .iterations = 16,
                                              .tolerance = 1e-4f,
                                              .weight = {2, 0.0f}};
static const maniGraphBodyNodeSource kSdls = {.top = 0,
                                              .topMode = maniBodyTopFree,
                                              .effectors = kHand,
                                              .effectorCount = 1,
                                              .iterations = 8,
                                              .weight = LITERAL(1.0f)};

static maniGraphNodeSource BodyNode(maniGraphNodeKind kind, const uint32_t* input,
                                    maniGraphBodyNodeSource s)
{
    return (maniGraphNodeSource){kind, 1, input, .body = s};
}

static void Evaluate(maniGraphInstance* instance)
{
    CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_out) == mani_success,
          "evaluated");
}

static bool Matches(bool walk)
{
    bool same = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniTransform x = maniPose_GetLocal(s_out, b);
        maniTransform y = walk ? s_walk[b] : maniPose_GetLocal(s_pose, b);
        same = same && memcmp(&x, &y, sizeof x) == 0;
    }
    return same;
}

static void Walk(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraphInstance* instance = Make(BodySource(nodes, 1), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    for (uint32_t b = 0; b < 3; ++b)
    {
        s_walk[b] = maniPose_GetLocal(s_out, b);
    }
}

// A parameter's default as a goal, as a rotation.
static maniVec3 Goal(uint32_t parameter)
{
    const float* f = s_parameters[parameter].defaultValue.f;
    return (maniVec3){f[0], f[1], f[2]};
}

static maniQuat Turn(uint32_t parameter)
{
    if (parameter == MANI_NO_PARAMETER)
    {
        return (maniQuat){0.0f, 0.0f, 0.0f, 1.0f};
    }
    const float* f = s_parameters[parameter].defaultValue.f;
    return (maniQuat){f[0], f[1], f[2], f[3]};
}

// s_pose, or the pose already there when again: solved by the node's
// solver with the parameters' defaults, rotation weight b and weight as
// given where the node reads a parameter.
static void Reference(bool sdls, const maniGraphBodyNodeSource* n, float b, float weight,
                      bool again)
{
    for (uint32_t k = 0; k < 3 && !again; ++k)
    {
        maniPose_SetLocal(s_pose, k, s_walk[k]);
    }
    maniBodyEffector effectors[MANI_MAX_BODY_IK_EFFECTORS];
    for (uint32_t e = 0; e < n->effectorCount; ++e)
    {
        const maniGraphEffectorSource* f = &n->effectors[e];
        maniGraphScalar w = f->rotationWeight;
        effectors[e] =
            (maniBodyEffector){.bone = f->bone,
                               .position = Goal(f->position),
                               .rotation = Turn(f->rotation),
                               .positionWeight = f->positionWeight.value,
                               .rotationWeight = w.parameter != MANI_NO_PARAMETER ? b : w.value,
                               .strength = f->strength,
                               .pullChain = f->pullChain};
    }
    maniBodySolve s = {.top = n->top,
                       .topMode = n->topMode,
                       .effectors = effectors,
                       .bones = n->bones,
                       .effectorCount = n->effectorCount,
                       .boneCount = n->boneCount,
                       .iterations = n->iterations,
                       .stretch = n->stretch,
                       .tolerance = n->tolerance,
                       .weight =
                           n->weight.parameter != MANI_NO_PARAMETER ? weight : n->weight.value};
    maniResult r = sdls ? maniPose_SolveBodySdls(s_pose, &s_skeleton, &s, s_bodyScratch,
                                                 sizeof s_bodyScratch, NULL)
                        : maniPose_SolveBodyXpbd(s_pose, &s_skeleton, &s, s_bodyScratch,
                                                 sizeof s_bodyScratch, NULL);
    CHECK(r == mani_success, "the reference");
}

static void TestXpbd(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    BodyNode(maniGraphNodeBodyXpbd, kFirst, kXpbd)};
    maniGraphInstance* instance = Make(BodySource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Reference(false, &kXpbd, 1.0f, 1.0f, false);
    CHECK(Matches(false) && !Matches(true), "the walk solved by XPBD");
    uint32_t data = maniLoadU32(g.nodes + MANI_GRAPH_NODE_SIZE + 12);
    size_t need = maniGraphBodyRoom(&g, data, &s_skeleton);
    CHECK(need > 15 + 2 * sizeof(maniBodyEffector) + sizeof(maniBodyBone) &&
              instance->size - instance->roomAt >= need,
          "the instance's room holds the node's arrays and scratch");
    Set(instance, 3, maniGraphParameterFloat, Float(0.5f));
    Set(instance, 2, maniGraphParameterFloat, Float(4.0f));
    Evaluate(instance);
    Reference(false, &kXpbd, 0.5f, 1.0f, false);
    CHECK(Matches(false), "rotation weight 0.5, weight 4 clamped to 1");
    Set(instance, 2, maniGraphParameterFloat, Float(0.5f));
    Set(instance, 0, maniGraphParameterVec3, Vec(0.2f, 1.6f, -0.3f, 0.0f));
    s_parameters[0].defaultValue = Vec(0.2f, 1.6f, -0.3f, 0.0f);
    Evaluate(instance);
    Reference(false, &kXpbd, 0.5f, 0.5f, false);
    s_parameters[0].defaultValue = Vec(0.5f, 1.3f, 0.3f, 0.0f);
    CHECK(Matches(false), "a goal moved, weight 0.5");
    Set(instance, 2, maniGraphParameterFloat, Float(-1.0f));
    Evaluate(instance);
    CHECK(Matches(true), "weight -1 clamped to 0: the walk");
}

static void TestSdls(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    BodyNode(maniGraphNodeBodySdls, kFirst, kSdls)};
    maniGraphInstance* instance = Make(BodySource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Reference(true, &kSdls, 0.0f, 1.0f, false);
    CHECK(Matches(false) && !Matches(true), "the walk solved by SDLS with the top free");
}

// Both nodes in a subgraph read the outer goal and weight their own map
// to, at other places.
static void TestSubgraph(void)
{
    static maniGraphParameterSource inner[4];
    static maniGraphNodeSource innerNodes[3];
    static const uint32_t kMap[4] = {3, 0, 5, 2};
    static const uint32_t kSecond[1] = {1};
    // The XPBD node's effector reads its rotation and both weights from
    // the outer parameters too.
    static const maniGraphEffectorSource kAimed[1] = {{2, 1, 2, {3, 0.0f}, {3, 0.0f}, 1.0f, 0.0f}};
    static const maniGraphEffectorSource kTarget[1] = {
        {2, 1, MANI_NO_PARAMETER, LITERAL(1.0f), LITERAL(0.0f), 1.0f, 0.0f}};
    static const maniGraphEffectorSource kOuter[1] = {
        {2, 0, 5, LITERAL(0.75f), LITERAL(0.75f), 1.0f, 0.0f}};
    inner[0] = Parameter("w", maniGraphParameterFloat, Float(0.0f));
    inner[1] = Parameter("target", maniGraphParameterVec3, Vec(0.0f, 0.0f, 0.0f, 0.0f));
    inner[2] = Parameter("aim", maniGraphParameterQuat, Vec(0.0f, 0.0f, 0.0f, 1.0f));
    inner[3] = Parameter("v", maniGraphParameterFloat, Float(0.0f));
    maniGraphBodyNodeSource xpbd = kXpbd;
    xpbd.effectors = kAimed;
    xpbd.effectorCount = 1;
    xpbd.weight = (maniGraphScalar){0, 0.0f};
    maniGraphBodyNodeSource sdls = kSdls;
    sdls.effectors = kTarget;
    innerNodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    innerNodes[1] = BodyNode(maniGraphNodeBodyXpbd, kFirst, xpbd);
    innerNodes[2] = BodyNode(maniGraphNodeBodySdls, kSecond, sdls);
    static maniGraphSource sub;
    sub = (maniGraphSource){
        .parameters = inner, .nodes = innerNodes, .parameterCount = 4, .nodeCount = 3, .output = 2};
    sub.skeleton = &s_skeleton;
    sub.clips = kSlots;
    sub.clipCount = CLIPS;
    maniGraph g;
    maniGraphNodeSource nodes[1] = {(maniGraphNodeSource){(maniGraphNodeKind)maniGraphNodeSubgraph,
                                                          0, NULL, .subgraph = {&sub, kMap}}};
    maniGraphInstance* instance = Make(BodySource(nodes, 1), &g);
    Update(instance, 0.3f);
    Set(instance, 3, maniGraphParameterFloat, Float(0.5f));
    Set(instance, 2, maniGraphParameterFloat, Float(0.75f));
    Evaluate(instance);
    // The references read the outer parameters where the inner nodes
    // read their own.
    xpbd.effectors = kOuter;
    xpbd.weight = LITERAL(0.5f);
    Reference(false, &xpbd, 0.0f, 0.5f, false);
    Reference(true, &kSdls, 0.0f, 1.0f, true);
    CHECK(Matches(false), "the outer goals and weights through the mapping");
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

static bool Refused(maniGraphNodeKind kind, maniGraphBodyNodeSource s, maniBuildReason reason,
                    uint32_t place)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    BodyNode(kind, kFirst, s)};
    maniBuildResult r = Built(BodySource(nodes, 2));
    return r.result == mani_errorInvalid && r.item == 1 && r.reason == reason &&
           (reason != maniBuildReasonScalar || r.other == place);
}

// A full-body node past a fixed bound: refused at the limit.
static bool PastLimit(maniGraphNodeKind kind, maniGraphBodyNodeSource s)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    BodyNode(kind, kFirst, s)};
    maniBuildResult r = Built(BodySource(nodes, 2));
    return r.result == mani_errorLimit && r.item == 1 && r.reason == maniBuildReasonNode;
}

// Whether the XPBD node with effector 0 changed by change is refused.
static bool EffectorRefused(maniGraphEffectorSource change, maniBuildReason reason, uint32_t place)
{
    maniGraphEffectorSource effectors[2] = {change, kTwo[1]};
    maniGraphBodyNodeSource s = kXpbd;
    s.effectors = effectors;
    return Refused(maniGraphNodeBodyXpbd, s, reason, place);
}

// Whether the XPBD node with its setting changed by change is refused.
static bool SettingRefused(maniBodyBone change)
{
    maniGraphBodyNodeSource s = kXpbd;
    s.bones = &change;
    return Refused(maniGraphNodeBodyXpbd, s, maniBuildReasonNode, 0);
}

static void TestBuildRefusals(void)
{
    static const uint32_t kLayered[2] = {2, 1};
    static const maniGraphScalar kWhole[1] = {LITERAL(1.0f)};
    maniGraphNodeSource nodes[4] = {
        ClipNode(2, 1.0f, maniGraphLoopRepeat), BodyNode(maniGraphNodeBodyXpbd, kFirst, kXpbd),
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        (maniGraphNodeSource){maniGraphNodeAdditive, 2, kLayered, .additive = {kWhole}}};
    maniBuildResult r = Built(BodySource(nodes, 4));
    CHECK(r.result == mani_errorInvalid && r.item == 1 && r.reason == maniBuildReasonDifference,
          "differences in, as an additive's layer");
    maniGraphBodyNodeSource bad[12];
    for (int k = 0; k < 12; ++k)
    {
        bad[k] = kXpbd;
    }
    bad[0].top = 3;
    bad[1].top = 2;
    bad[2].topMode = (maniBodyTopMode)3;
    bad[3].effectors = NULL;
    bad[4].effectorCount = 0;
    bad[5].effectorCount = MANI_MAX_BODY_IK_EFFECTORS + 1;
    bad[6].bones = NULL;
    bad[7].boneCount = MANI_MAX_BODY_IK_BONES + 1;
    bad[8].iterations = 0;
    bad[9].iterations = MANI_MAX_IK_ITERATIONS + 1;
    bad[10].tolerance = -1.0f;
    bad[11].tolerance = NAN;
    int refused = 0;
    for (int k = 0; k < 12; ++k)
    {
        bool past = k == 5 || k == 7 || k == 9;
        refused += past ? PastLimit(maniGraphNodeBodyXpbd, bad[k])
                        : Refused(maniGraphNodeBodyXpbd, bad[k], maniBuildReasonNode, 0);
    }
    CHECK(refused == 12, "the top, mode, counts and numbers refused, past a bound at the limit");
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        maniGraphBodyNodeSource negative = kXpbd;
        negative.topMode = kNegativeKinds[k];
        CHECK(Refused(maniGraphNodeBodyXpbd, negative, maniBuildReasonNode, 0),
              "a negative top mode refused");
    }
    maniGraphBodyNodeSource s = kSdls;
    s.stretch = true;
    CHECK(Refused(maniGraphNodeBodySdls, s, maniBuildReasonNode, 0), "an SDLS node's stretch");
    s.stretch = false;
    s.top = 2;
    s.effectors = kTwo;
    s.effectorCount = 2;
    CHECK(Refused(maniGraphNodeBodySdls, s, maniBuildReasonNode, 0),
          "an effector not below the top");
    maniGraphEffectorSource e[9];
    for (int k = 0; k < 9; ++k)
    {
        e[k] = kTwo[0];
    }
    e[0].bone = 3;
    e[1].bone = 1;
    e[2].position = 2;
    e[3].position = MANI_NO_PARAMETER;
    e[4].rotation = 2;
    e[5].strength = 1.5f;
    e[6].pullChain = -0.1f;
    e[7].strength = NAN;
    e[8].rotation = 0;
    refused = 0;
    for (int k = 0; k < 9; ++k)
    {
        refused += EffectorRefused(e[k], maniBuildReasonNode, 0);
    }
    CHECK(refused == 9, "effectors' bones, parameters and numbers refused");
    maniGraphEffectorSource w = kTwo[0];
    w.positionWeight = LITERAL(1.5f);
    CHECK(EffectorRefused(w, maniBuildReasonScalar, 0), "a position weight past 1");
    w = kTwo[0];
    w.rotationWeight = (maniGraphScalar){4, 0.0f};
    CHECK(EffectorRefused(w, maniBuildReasonScalar, 1), "a rotation weight reading a bool");
    w.rotation = MANI_NO_PARAMETER;
    w.rotationWeight = LITERAL(0.5f);
    CHECK(EffectorRefused(w, maniBuildReasonScalar, 1), "a rotation weight with no rotation");
    w.rotationWeight = (maniGraphScalar){3, 0.0f};
    CHECK(EffectorRefused(w, maniBuildReasonScalar, 1),
          "a rotation weight's parameter with no rotation");
    maniBodyBone b[6];
    for (int k = 0; k < 6; ++k)
    {
        b[k] = kSpine[0];
    }
    b[0].bone = 3;
    b[1].limit.cone = 3.5f;
    b[2].limit = (maniJointLimit){.kind = maniJointLimitHinge,
                                  .axis = {0.83205029f, 0.5547002f, 0.0f},
                                  .low = -1.0f,
                                  .high = 1.0f};
    b[3].preferred.y = INFINITY;
    b[4].rotationStiffness = 1.5f;
    b[5].positionStiffness = -0.5f;
    refused = 0;
    for (int k = 0; k < 6; ++k)
    {
        refused += SettingRefused(b[k]);
    }
    CHECK(refused == 6, "settings' bones, limits, angles and stiffness refused");
    const maniBodyBone twice[2] = {kSpine[0], kSpine[0]};
    s = kXpbd;
    s.bones = twice;
    s.boneCount = 2;
    CHECK(Refused(maniGraphNodeBodyXpbd, s, maniBuildReasonNode, 0), "a setting twice");
    s = kXpbd;
    s.weight = LITERAL(-0.5f);
    CHECK(Refused(maniGraphNodeBodyXpbd, s, maniBuildReasonScalar, 4), "a weight below 0");
}

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
// its first data word and of its record.
static size_t Built1(maniGraphNodeSource node, size_t* sizeOut, size_t* recordOut)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat), node};
    maniGraphSource source = BodySource(nodes, 2);
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    maniBuilder* builder = NewBuilder();
    maniGraph g;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, sizeOut) == mani_success &&
              maniOpenGraph(s_file, *sizeOut, &g).result == mani_success,
          "a graph of a body node");
    maniDestroyBuilder(builder);
    *recordOut = (size_t)(g.nodes - s_file) + MANI_GRAPH_NODE_SIZE;
    return (size_t)(g.data - s_file) + 4 * (size_t)maniLoadU32(g.nodes + MANI_GRAPH_NODE_SIZE + 12);
}

// Whether the file with word w of the node's data set to value is
// refused at word wrong (a scalar's first), or at the record's word
// count when wrong is UINT32_MAX.
static bool WordRefused(size_t size, size_t at, uint32_t w, uint32_t value, uint32_t wrong,
                        size_t record)
{
    memcpy(s_crafted, s_file, size);
    maniStoreU32(s_crafted + at + 4 * (size_t)w, value);
    Rehash(s_crafted, size);
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, size, &crafted);
    size_t offset = wrong == UINT32_MAX ? record + 16 : at + 4 * (size_t)wrong;
    return r.result == mani_errorFormat && r.offset == offset;
}

static void TestFile(void)
{
    size_t size = 0;
    size_t record = 0;
    size_t at = Built1(BodyNode(maniGraphNodeBodyXpbd, kFirst, kXpbd), &size, &record);
    CHECK(maniLoadU32(s_file + at + 4 * 7) == 2 && maniLoadU32(s_file + at + 4 * 8) == 1 &&
              maniLoadU32(s_file + record + 16) == 9 + 2 * 9 + 15,
          "two effectors, one setting, 42 words");
    // The head.
    CHECK(WordRefused(size, at, 0, 3, 0, record), "a top past");
    CHECK(WordRefused(size, at, 1, 3, 1, record), "a mode unknown");
    CHECK(WordRefused(size, at, 2, 2, 2, record), "stretch neither 0 nor 1");
    CHECK(WordRefused(size, at, 3, 0, 3, record) && WordRefused(size, at, 3, 257, 3, record),
          "iterations");
    CHECK(WordRefused(size, at, 4, Bits(-1.0f), 4, record) &&
              WordRefused(size, at, 4, Bits(INFINITY), 4, record),
          "a tolerance below 0 or not finite");
    CHECK(WordRefused(size, at, 6, Bits(1.0f), 5, record), "a weight's parameter with bits");
    CHECK(WordRefused(size, at, 7, 33, UINT32_MAX, record) &&
              WordRefused(size, at, 7, 0, UINT32_MAX, record) &&
              WordRefused(size, at, 8, 129, UINT32_MAX, record) &&
              WordRefused(size, at, 8, 0, UINT32_MAX, record),
          "counts out of range or not the words'");
    // Effector 0 from word 9, effector 1 from 18.
    CHECK(WordRefused(size, at, 9, 3, 9, record), "an effector's bone past");
    CHECK(WordRefused(size, at, 10, 2, 10, record) &&
              WordRefused(size, at, 10, MANI_NO_PARAMETER, 10, record),
          "a position reading a float or none");
    CHECK(WordRefused(size, at, 11, 2, 11, record), "a rotation reading a float");
    CHECK(WordRefused(size, at, 13, Bits(1.5f), 12, record), "a position weight past 1");
    CHECK(WordRefused(size, at, 15, Bits(1.0f), 14, record),
          "a rotation weight's parameter with bits");
    CHECK(WordRefused(size, at, 16, Bits(1.5f), 16, record), "a strength past 1");
    CHECK(WordRefused(size, at, 17, Bits(-0.1f), 17, record), "a pull chain below 0");
    CHECK(WordRefused(size, at, 24, Bits(0.5f), 23, record) &&
              WordRefused(size, at, 23, 3, 23, record),
          "a rotation weight with no rotation");
    // The setting from word 27.
    CHECK(WordRefused(size, at, 27, 3, 27, record), "a setting's bone past");
    CHECK(WordRefused(size, at, 28, 3, 28, record), "a limit of an unknown kind");
    CHECK(WordRefused(size, at, 32, Bits(3.5f), 29, record), "a cone past pi");
    CHECK(WordRefused(size, at, 37, Bits(INFINITY), 37, record), "a preferred angle");
    CHECK(WordRefused(size, at, 40, Bits(1.5f), 40, record), "a rotation stiffness past 1");
    CHECK(WordRefused(size, at, 41, Bits(-1.0f), 41, record), "a position stiffness below 0");
    at = Built1(BodyNode(maniGraphNodeBodySdls, kFirst, kSdls), &size, &record);
    CHECK(maniLoadU32(s_file + record + 16) == 9 + 9, "one effector, no settings, 18 words");
    CHECK(WordRefused(size, at, 2, 1, 2, record), "an SDLS node's stretch");
}

// A rotation weight of -0 with no rotation built as the number 0, so the
// file opens.
static void TestNegativeZero(void)
{
    maniGraphEffectorSource effectors[2] = {kTwo[0], kTwo[1]};
    effectors[1].rotationWeight = LITERAL(-0.0f);
    maniGraphBodyNodeSource s = kXpbd;
    s.effectors = effectors;
    size_t size = 0;
    size_t record = 0;
    size_t at = Built1(BodyNode(maniGraphNodeBodyXpbd, kFirst, s), &size, &record);
    CHECK(maniLoadU32(s_file + at + 4 * 24) == 0, "-0 written as 0");
}

// The word count on its own: the counts' ranges and the words available.
static void TestWords(void)
{
    static _Alignas(8) uint8_t words[4 * MANI_GRAPH_BODY_WORDS];
    maniGraph g = {.data = words};
    const uint32_t counts[5][3] = {
        {2, 1, 42}, {2, 1, 41}, {0, 1, 1000}, {33, 0, 1000}, {2, 129, 4000}};
    const uint32_t want[5] = {42, 0, 0, 0, 0};
    int right = 0;
    for (int k = 0; k < 5; ++k)
    {
        maniStoreU32(words + 4 * maniGraphBodyEffectorCount, counts[k][0]);
        maniStoreU32(words + 4 * maniGraphBodySettingCount, counts[k][1]);
        right += maniGraphBodyDataWords(&g, 0, counts[k][2]) == want[k];
    }
    CHECK(right == 5 && maniGraphBodyDataWords(&g, 0, MANI_GRAPH_BODY_WORDS - 1) == 0,
          "counts out of range and words past those available give 0");
}

// Whether the XPBD node with word w set to value opens, binds and
// passes its input.
static bool CraftedPasses(uint32_t w, uint32_t value)
{
    size_t size = 0;
    size_t record = 0;
    size_t at = Built1(BodyNode(maniGraphNodeBodyXpbd, kFirst, kXpbd), &size, &record);
    maniStoreU32(s_file + at + 4 * (size_t)w, value);
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
    return Matches(true);
}

// Crafted XPBD nodes open and pass their input: the top the arm, the
// spine's effector not below it; both effectors on the arm. The top made
// free still solves.
static void TestCraftedBody(void)
{
    CHECK(CraftedPasses(0, 2), "a top below an effector: the input passed");
    CHECK(CraftedPasses(18, 2), "one bone twice among the effectors: the input passed");
    CHECK(CraftedPasses(1, 1) == false, "the top free: solved");
}

// The source of both nodes in a row.
static maniGraphSource Both(maniGraphNodeSource* nodes)
{
    static const uint32_t kSecond[1] = {1};
    nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    nodes[1] = BodyNode(maniGraphNodeBodyXpbd, kFirst, kXpbd);
    nodes[2] = BodyNode(maniGraphNodeBodySdls, kSecond, kSdls);
    maniGraphSource source = BodySource(nodes, 3);
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    return source;
}

// Whether the text with from replaced by to is refused as a form fault
// of the kind.
static bool TextRefused(maniBuilder* builder, const char* from, const char* to, uint32_t form)
{
    static char edited[32768];
    const char* found = strstr(s_text, from);
    if (found == NULL)
    {
        return false;
    }
    size_t head = (size_t)(found - s_text);
    (void)snprintf(edited, sizeof edited, "%.*s%s%s", (int)head, s_text, to, found + strlen(from));
    maniGraphNamedAssets assets = {&s_skeleton, kSlots, NULL, NULL, NULL, CLIPS, 0, 0, 0, NULL};
    maniBuildResult r = maniBuildGraphJson(builder, edited, strlen(edited), &assets);
    return r.result == mani_errorInvalid && r.reason == maniBuildReasonForm && r.other == form;
}

// Written as JSON, read back to the same file; a rotation weight with no
// rotation, an unknown mode, settings not a list and no effectors
// refused.
static void TestJson(void)
{
    maniGraphNodeSource nodes[3];
    maniGraphSource source = Both(nodes);
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    size_t length = 0;
    size_t read = 0;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success,
          "built and written");
    s_text[length < sizeof s_text ? length : sizeof s_text - 1] = '\0';
    maniGraphNamedAssets assets = {&s_skeleton, kSlots, NULL, NULL, NULL, CLIPS, 0, 0, 0, NULL};
    CHECK(maniBuildGraphJson(builder, s_text, length, &assets).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_read, sizeof s_read, &read) == mani_success &&
              read == size && memcmp(s_read, s_file, size) == 0,
          "read back to the same file");
    CHECK(strstr(s_text, "\"bones\": [") != NULL && strstr(s_text, "\"topMode\": \"free\"") != NULL,
          "settings and modes written");
    CHECK(TextRefused(builder, "\"positionWeight\": 0.8", "\"rotationWeight\": 0.5",
                      maniGraphFormMember),
          "a rotation weight with no rotation refused");
    CHECK(
        TextRefused(builder, "\"topMode\": \"free\"", "\"topMode\": \"loose\"", maniGraphFormValue),
        "an unknown mode refused");
    CHECK(TextRefused(builder, "\"iterations\": 8", "\"iterations\": 8, \"bones\": 3",
                      maniGraphFormValue),
          "settings not a list refused");
    // The SDLS node's one effector cut out: an empty list.
    const char* last = NULL;
    for (const char* p = strstr(s_text, "\"effectors\": ["); p != NULL;
         p = strstr(p + 1, "\"effectors\": ["))
    {
        last = p;
    }
    const char* end = last != NULL ? strchr(last, ']') : NULL;
    static char list[1024];
    size_t span = end != NULL ? (size_t)(end - last) + 1 : 0;
    if (span > 0 && span < sizeof list)
    {
        memcpy(list, last, span);
        list[span] = '\0';
    }
    CHECK(span > 0 && span < sizeof list &&
              TextRefused(builder, list, "\"effectors\": []", maniGraphFormValue),
          "no effectors refused");
    maniDestroyBuilder(builder);
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

static bool Seed(const char* graphPath, const char* textPath)
{
    maniGraphNodeSource nodes[3];
    maniGraphSource source = Both(nodes);
    size_t size = 0;
    size_t length = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success &&
              WriteFile(graphPath, s_file, size) && WriteFile(textPath, s_text, length);
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
    TestXpbd();
    TestSdls();
    TestSubgraph();
    TestBuildRefusals();
    TestFile();
    TestNegativeZero();
    TestWords();
    TestCraftedBody();
    TestJson();
    return s_failures == 0 ? 0 : 1;
}
