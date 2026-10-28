// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The chain IK nodes over the hips, spine and arm: a CCD
// node's output is its input solved as maniSolveCcd solves it, a FABRIK
// node's as maniSolveFabrik, bit for bit, their goal, pole and the end's
// rotation read from parameters, weights clamped (0 passing the input);
// limits carried; both inside a subgraph through its mapping; the JSON
// form read back to the same file; refused at build (differences,
// bones, parameters, numbers, limits) and at open at the first wrong
// word; a crafted file whose bones are not a chain, or whose limits are
// not one per turning bone, passing its input.
// Run as test_graph_chain_node --seed <graph file> <json file> it writes
// a graph of both nodes, its file and text, seeds of the fuzz targets.

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
static _Alignas(16) uint8_t s_read[16384];
static char s_text[16384];
static maniPose* s_pose;
static maniPose* s_out;
static maniGraphParameterSource s_parameters[6];
static maniTransform s_walk[3];

static const uint32_t kFirst[1] = {0};

static maniGraphValue Vec(float x, float y, float z, float w)
{
    return (maniGraphValue){.f = {x, y, z, w}};
}

// Parameters: 0 the goal and 1 the pole (vec3), 2 and 3 floats, 4 a
// bool, 5 the end's rotation (quat).
static maniGraphSource ChainSource(const maniGraphNodeSource* nodes, uint32_t count)
{
    s_parameters[0] = Parameter("goal", maniGraphParameterVec3, Vec(0.5f, 1.3f, 0.3f, 0.0f));
    s_parameters[1] = Parameter("pole", maniGraphParameterVec3, Vec(0.0f, 0.0f, 1.0f, 0.0f));
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

static const maniJointLimit kCones[2] = {
    {.kind = maniJointLimitCone, .cone = 0.6f, .twistLow = -0.2f, .twistHigh = 0.2f},
    {.kind = maniJointLimitHinge, .axis = {0.0f, 0.0f, 1.0f}, .low = -1.0f, .high = 1.0f}};

// The CCD node: hips to arm toward the goal, the end turned by b toward
// the rotation, weighted by a, with limits.
static const maniGraphChainNodeSource kCcd = {0,      2,     0,    MANI_NO_PARAMETER, 5,        16,
                                              kCones, 1e-4f, 0.3f, {3, 0.0f},         {2, 0.0f}};
static const maniGraphChainNodeSource kFabrik = {
    0, 2, 0, 1, MANI_NO_PARAMETER, 8, NULL, 0.0f, 0.0f, LITERAL(1.0f), {2, 0.0f}};

static maniGraphNodeSource ChainNode(maniGraphNodeKind kind, const uint32_t* input,
                                     maniGraphChainNodeSource s)
{
    return (maniGraphNodeSource){kind, 1, input, .chain = s};
}

static void Evaluate(maniGraphInstance* instance)
{
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_out) == mani_success,
          "evaluated");
}

static bool Matches(bool walk)
{
    bool same = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniTransform x = maniPoseLocal(s_out, b);
        maniTransform y = walk ? s_walk[b] : maniPoseLocal(s_pose, b);
        same = same && memcmp(&x, &y, sizeof x) == 0;
    }
    return same;
}

static void Walk(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraphInstance* instance = Make(ChainSource(nodes, 1), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    for (uint32_t b = 0; b < 3; ++b)
    {
        s_walk[b] = maniPoseLocal(s_out, b);
    }
}

// s_pose: the walk solved by the node's solver with the parameters'
// defaults, its weight and rotation weight as given.
static void Reference(bool fabrik, const maniGraphChainNodeSource* n, float rotationWeight,
                      float weight)
{
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniPoseSetLocal(s_pose, b, s_walk[b]);
    }
    maniChainSolve s = {.root = n->root,
                        .end = n->end,
                        .limits = n->limits,
                        .goal = {0.5f, 1.3f, 0.3f},
                        .pole = n->pole != MANI_NO_PARAMETER ? (maniVec3){0.0f, 0.0f, 1.0f}
                                                             : (maniVec3){0.0f, 0.0f, 0.0f},
                        .rotation = n->rotation != MANI_NO_PARAMETER
                                        ? (maniQuat){0.0f, 0.0f, 0.38268343f, 0.92387953f}
                                        : (maniQuat){0.0f, 0.0f, 0.0f, 1.0f},
                        .iterations = n->iterations,
                        .tolerance = n->tolerance,
                        .step = n->step,
                        .rotationWeight = n->rotation != MANI_NO_PARAMETER ? rotationWeight : 0.0f,
                        .weight = weight};
    maniResult r = fabrik ? maniSolveFabrik(s_pose, &s_skeleton, &s, NULL)
                          : maniSolveCcd(s_pose, &s_skeleton, &s, NULL);
    CHECK(r == mani_success, "the reference");
}

static void TestCcd(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ChainNode(maniGraphNodeCcd, kFirst, kCcd)};
    maniGraphInstance* instance = Make(ChainSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Reference(false, &kCcd, 1.0f, 1.0f);
    CHECK(Matches(false) && !Matches(true), "the walk solved by CCD");
    Set(instance, 3, maniGraphParameterFloat, Float(0.5f));
    Set(instance, 2, maniGraphParameterFloat, Float(4.0f));
    Evaluate(instance);
    Reference(false, &kCcd, 0.5f, 1.0f);
    CHECK(Matches(false), "rotation weight 0.5, weight 4 clamped to 1");
    Set(instance, 2, maniGraphParameterFloat, Float(-1.0f));
    Evaluate(instance);
    CHECK(Matches(true), "weight -1 clamped to 0: the walk");
}

static void TestFabrik(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ChainNode(maniGraphNodeFabrik, kFirst, kFabrik)};
    maniGraphInstance* instance = Make(ChainSource(nodes, 2), &g);
    Update(instance, 0.3f);
    Evaluate(instance);
    Reference(true, &kFabrik, 0.0f, 1.0f);
    CHECK(Matches(false) && !Matches(true), "the walk solved by FABRIK with a pole");
    Set(instance, 2, maniGraphParameterFloat, Float(0.5f));
    Evaluate(instance);
    Reference(true, &kFabrik, 0.0f, 0.5f);
    CHECK(Matches(false), "weight 0.5");
}

// Both nodes in a subgraph read the outer goal and weight their own map
// to, at other places.
static void TestSubgraph(void)
{
    static maniGraphParameterSource inner[2];
    static maniGraphNodeSource innerNodes[3];
    static const uint32_t kMap[2] = {3, 0};
    static const uint32_t kSecond[1] = {1};
    inner[0] = Parameter("w", maniGraphParameterFloat, Float(0.0f));
    inner[1] = Parameter("target", maniGraphParameterVec3, Vec(0.0f, 0.0f, 0.0f, 0.0f));
    maniGraphChainNodeSource ccd = kCcd;
    ccd.goal = 1;
    ccd.rotation = MANI_NO_PARAMETER;
    ccd.rotationWeight = LITERAL(1.0f);
    ccd.weight = (maniGraphScalar){0, 0.0f};
    maniGraphChainNodeSource fabrik = kFabrik;
    fabrik.goal = 1;
    fabrik.pole = MANI_NO_PARAMETER;
    fabrik.weight = LITERAL(1.0f);
    innerNodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    innerNodes[1] = ChainNode(maniGraphNodeCcd, kFirst, ccd);
    innerNodes[2] = ChainNode(maniGraphNodeFabrik, kSecond, fabrik);
    static maniGraphSource sub;
    sub = (maniGraphSource){
        .parameters = inner, .nodes = innerNodes, .parameterCount = 2, .nodeCount = 3, .output = 2};
    sub.skeleton = &s_skeleton;
    sub.clips = kSlots;
    sub.clipCount = CLIPS;
    maniGraph g;
    maniGraphNodeSource nodes[1] = {(maniGraphNodeSource){(maniGraphNodeKind)maniGraphNodeSubgraph,
                                                          0, NULL, .subgraph = {&sub, kMap}}};
    maniGraphInstance* instance = Make(ChainSource(nodes, 1), &g);
    Update(instance, 0.3f);
    Set(instance, 3, maniGraphParameterFloat, Float(0.5f));
    Evaluate(instance);
    Reference(false, &ccd, 0.0f, 0.5f);
    maniChainSolve s = {.root = 0,
                        .end = 2,
                        .goal = {0.5f, 1.3f, 0.3f},
                        .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                        .iterations = 8,
                        .weight = 1.0f};
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, NULL) == mani_success, "the reference");
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

static bool Refused(maniGraphNodeKind kind, maniGraphChainNodeSource s, maniBuildReason reason)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ChainNode(kind, kFirst, s)};
    maniBuildResult r = Built(ChainSource(nodes, 2));
    return r.result == mani_errorInvalid && r.item == 1 && r.reason == reason;
}

// A chain node past a fixed bound: refused at the limit.
static bool PastLimit(maniGraphNodeKind kind, maniGraphChainNodeSource s)
{
    maniGraphNodeSource nodes[2] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ChainNode(kind, kFirst, s)};
    maniBuildResult r = Built(ChainSource(nodes, 2));
    return r.result == mani_errorLimit && r.item == 1 && r.reason == maniBuildReasonNode;
}

static void TestBuildRefusals(void)
{
    static const uint32_t kLayered[2] = {2, 1};
    static const maniGraphScalar kWhole[1] = {LITERAL(1.0f)};
    maniGraphNodeSource nodes[4] = {
        ClipNode(2, 1.0f, maniGraphLoopRepeat), ChainNode(maniGraphNodeCcd, kFirst, kCcd),
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        (maniGraphNodeSource){maniGraphNodeAdditive, 2, kLayered, .additive = {kWhole}}};
    maniBuildResult r = Built(ChainSource(nodes, 4));
    CHECK(r.result == mani_errorInvalid && r.item == 1 && r.reason == maniBuildReasonDifference,
          "differences in, as an additive's layer");
    const maniJointLimit kAlong[2] = {
        {.kind = maniJointLimitHinge, .axis = {0.0f, 1.0f, 0.0f}, .low = -1.0f, .high = 1.0f}, {0}};
    const maniJointLimit kWide[2] = {{.kind = maniJointLimitCone, .cone = 3.5f}, {0}};
    maniGraphChainNodeSource bad[16];
    for (int k = 0; k < 16; ++k)
    {
        bad[k] = kCcd;
    }
    bad[0].end = 3;
    bad[1].root = 2;
    bad[1].end = 0;
    bad[2].end = 0;
    bad[3].limits = kAlong;
    bad[4].limits = kWide;
    bad[5].goal = 2;
    bad[6].pole = 1;
    bad[7].rotation = 2;
    bad[8].rotation = 0;
    bad[9].iterations = 0;
    bad[10].iterations = MANI_MAX_IK_ITERATIONS + 1;
    bad[11].tolerance = -1.0f;
    bad[12].tolerance = NAN;
    bad[13].step = 3.5f;
    bad[14].goal = MANI_NO_PARAMETER;
    bad[15].step = -0.1f;
    int refused = 0;
    for (int k = 0; k < 16; ++k)
    {
        refused += k == 10 ? PastLimit(maniGraphNodeCcd, bad[k])
                           : Refused(maniGraphNodeCcd, bad[k], maniBuildReasonNode);
    }
    CHECK(refused == 16, "bones, limits, parameters and numbers refused, iterations at the limit");
    maniGraphChainNodeSource s = kFabrik;
    s.step = 0.1f;
    CHECK(Refused(maniGraphNodeFabrik, s, maniBuildReasonNode), "a FABRIK node's step");
    s = kFabrik;
    s.pole = 2;
    CHECK(Refused(maniGraphNodeFabrik, s, maniBuildReasonNode), "a pole reading a float");
    s = kCcd;
    s.rotationWeight = LITERAL(1.5f);
    CHECK(Refused(maniGraphNodeCcd, s, maniBuildReasonScalar), "a rotation weight past 1");
    s.rotationWeight = (maniGraphScalar){4, 0.0f};
    CHECK(Refused(maniGraphNodeCcd, s, maniBuildReasonScalar), "a rotation weight reading a bool");
    s = kCcd;
    s.weight = LITERAL(-0.5f);
    CHECK(Refused(maniGraphNodeCcd, s, maniBuildReasonScalar), "a weight below 0");
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
    maniGraphSource source = ChainSource(nodes, 2);
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    maniBuilder* builder = NewBuilder();
    maniGraph g;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, sizeOut) == mani_success &&
              maniOpenGraph(s_file, *sizeOut, &g).result == mani_success,
          "a graph of a chain node");
    maniDestroyBuilder(builder);
    *recordOut = (size_t)(g.nodes - s_file) + MANI_GRAPH_NODE_SIZE;
    return (size_t)(g.data - s_file) + 4 * (size_t)maniLoadU32(g.nodes + MANI_GRAPH_NODE_SIZE + 12);
}

// Whether the file with word w of the node's data set to value is
// refused at word wrong (a scalar's first).
static bool WordRefused(size_t size, size_t at, uint32_t w, uint32_t value, uint32_t wrong)
{
    memcpy(s_crafted, s_file, size);
    maniStoreU32(s_crafted + at + 4 * (size_t)w, value);
    Rehash(s_crafted, size);
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, size, &crafted);
    return r.result == mani_errorFormat && r.offset == at + 4 * (size_t)wrong;
}

static void TestFile(void)
{
    size_t size = 0;
    size_t record = 0;
    size_t at = Built1(ChainNode(maniGraphNodeCcd, kFirst, kCcd), &size, &record);
    CHECK(maniLoadU32(s_file + at + 4 * 12) == 2 && maniLoadU32(s_file + record + 16) == 13 + 18,
          "two limits, 31 words");
    CHECK(WordRefused(size, at, 0, 3, 0) && WordRefused(size, at, 1, 3, 1), "a bone past");
    CHECK(WordRefused(size, at, 2, 2, 2) && WordRefused(size, at, 2, MANI_NO_PARAMETER, 2),
          "a goal reading a float or none");
    CHECK(WordRefused(size, at, 3, 1, 3), "a CCD node's pole");
    CHECK(WordRefused(size, at, 4, 2, 4), "a rotation reading a float");
    CHECK(WordRefused(size, at, 5, 0, 5) && WordRefused(size, at, 5, 257, 5), "iterations");
    CHECK(WordRefused(size, at, 6, Bits(-1.0f), 6) && WordRefused(size, at, 6, Bits(INFINITY), 6),
          "a tolerance below 0 or not finite");
    CHECK(WordRefused(size, at, 7, Bits(3.5f), 7), "a step past pi");
    CHECK(WordRefused(size, at, 9, Bits(1.0f), 8) && WordRefused(size, at, 8, 4, 8),
          "a rotation weight's parameter with bits or of a bool");
    CHECK(WordRefused(size, at, 11, Bits(1.0f), 10), "a weight's parameter with bits");
    CHECK(WordRefused(size, at, 13, 3, 13), "a limit of an unknown kind");
    CHECK(WordRefused(size, at, 14, Bits(1.0f), 14), "a cone's axis not 0");
    CHECK(WordRefused(size, at, 17, Bits(3.5f), 14), "a cone past pi");
    CHECK(WordRefused(size, at, 22 + 4, Bits(0.5f), 22 + 4), "a hinge's cone not 0");
    CHECK(WordRefused(size, at, 22 + 1, Bits(2.0f), 22 + 1), "a hinge's axis not unit");
    memcpy(s_crafted, s_file, size);
    maniStoreU32(s_crafted + at + 4 * 12, 32);
    Rehash(s_crafted, size);
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, size, &crafted);
    CHECK(r.result == mani_errorFormat && r.offset == record + 16, "32 limits: the words wrong");
    at = Built1(ChainNode(maniGraphNodeFabrik, kFirst, kFabrik), &size, &record);
    CHECK(WordRefused(size, at, 7, Bits(0.1f), 7), "a FABRIK node's step");
    CHECK(WordRefused(size, at, 3, 2, 3), "a pole reading a float");
    CHECK(WordRefused(size, at, 9, Bits(1.5f), 8), "a rotation weight past 1");
}

// Whether a CCD node of s, crafted to root and end, opens, binds and
// passes its input.
static bool CraftedPasses(maniGraphChainNodeSource s, uint32_t root, uint32_t end)
{
    size_t size = 0;
    size_t record = 0;
    size_t at = Built1(ChainNode(maniGraphNodeCcd, kFirst, s), &size, &record);
    maniStoreU32(s_file + at, root);
    maniStoreU32(s_file + at + 4, end);
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

// Crafted CCD nodes open and pass their input: a root below the end; one
// limit on a path made of three bones, which turns two.
static void TestCraftedChain(void)
{
    CHECK(CraftedPasses(kCcd, 2, 0), "a root below its end: the input passed");
    maniGraphChainNodeSource s = kCcd;
    s.root = 1;
    CHECK(CraftedPasses(s, 0, 2), "one limit for two turning bones: the input passed");
}

// The source of both nodes in a row.
static maniGraphSource Both(maniGraphNodeSource* nodes)
{
    static const uint32_t kSecond[1] = {1};
    nodes[0] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    nodes[1] = ChainNode(maniGraphNodeCcd, kFirst, kCcd);
    nodes[2] = ChainNode(maniGraphNodeFabrik, kSecond, kFabrik);
    maniGraphSource source = ChainSource(nodes, 3);
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    return source;
}

// Written as JSON, read back to the same file; a limit list of the wrong
// length refused.
static void TestJson(void)
{
    maniGraphNodeSource nodes[3];
    maniGraphSource source = Both(nodes);
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    size_t length = 0;
    size_t read = 0;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success,
          "built and written");
    maniGraphNamedAssets assets = {&s_skeleton, kSlots, NULL, NULL, NULL, CLIPS, 0, 0, 0, NULL};
    CHECK(maniBuildGraphJson(builder, s_text, length, &assets).result == mani_success &&
              maniCopyBuilt(builder, s_read, sizeof s_read, &read) == mani_success &&
              read == size && memcmp(s_read, s_file, size) == 0,
          "read back to the same file");
    char* limits = strstr(s_text, "\"limits\": [");
    CHECK(limits != NULL, "limits written");
    char* second = limits != NULL ? strstr(limits, "}, {") : NULL;
    if (second != NULL)
    {
        // The second limit cut out: one limit for two turning bones.
        char* close = strstr(second + 3, "}]");
        memmove(second + 1, close + 1, strlen(close + 1) + 1);
        maniBuildResult r = maniBuildGraphJson(builder, s_text, strlen(s_text), &assets);
        CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonForm &&
                  r.other == maniGraphFormValue,
              "a limit list of the wrong length refused");
    }
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
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success &&
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
    TestCcd();
    TestFabrik();
    TestSubgraph();
    TestBuildRefusals();
    TestFile();
    TestCraftedChain();
    TestJson();
    return s_failures == 0 ? 0 : 1;
}
