// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blend spaces: a 1D space's weights between its points,
// held at its ends and wrapped across its seam, one point alone; gradient
// bands, cartesian and polar, against weights worked by hand from
// Johansen's equations or (speed and direction together) from a binary64
// model of them; a vec2 parameter for the position; a space evaluated as
// the blending functions blend; the builder's refusals and opening's
// checks; a space in a subgraph; the JSON form round trip. Run as
// test_graph_space --seed <graph file> <json file> it writes the round
// trip's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_space_fixture.h"

#include "maul-anim/pose.h"

#include <stdio.h>

// The walk 0, the run 1 and the short step 3 at -1, 2 and 0.5 on a line
// read from parameter x, with a period.
static maniGraphInstance* Line(float period, maniGraph* g)
{
    static const uint32_t kInputs[3] = {1, 2, 3};
    static const float kPoints[3] = {-1.0f, 2.0f, 0.5f};
    static maniGraphParameterSource x;
    x = Parameter("x", maniGraphParameterFloat, Float(0.0f));
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeBlendSpace1D, 3, kInputs,
         .blendSpace1D = {{MANI_NO_PARAMETER, {0, 0.0f}, LITERAL(0.0f)}, kPoints, period}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip)};
    // The walk 0, the run 1, the step 2, the space 3.
    return Make(
        (maniGraphSource){.parameters = &x, .nodes = nodes, .parameterCount = 1, .nodeCount = 4},
        g);
}

static bool At(maniGraphInstance* instance, const maniGraph* g, float x, const float* want,
               uint32_t count)
{
    Set(instance, 0, maniGraphParameterFloat, Float(x));
    Update(instance, 0.0f);
    return Weights(instance, g, want, count);
}

static void TestLine(void)
{
    maniGraph g;
    maniGraphInstance* instance = Line(0.0f, &g);
    static const float kWalk[3] = {1, 0, 0};
    static const float kStep[3] = {0, 0, 1};
    static const float kBetween[3] = {0, 0.5f, 0.5f};
    static const float kRun[3] = {0, 1, 0};
    static const float kQuarter[3] = {0.25f, 0, 0.75f};
    CHECK(At(instance, &g, -5.0f, kWalk, 3), "held at the low end");
    CHECK(At(instance, &g, 0.5f, kStep, 3), "on a point, that point");
    CHECK(At(instance, &g, 1.25f, kBetween, 3), "halfway between the step and the run");
    CHECK(At(instance, &g, 0.125f, kQuarter, 3), "a quarter of the way to the walk");
    CHECK(At(instance, &g, 9.0f, kRun, 3), "held at the high end");
    instance = Line(4.0f, &g);
    static const float kSeam[3] = {0.5f, 0.5f, 0};
    CHECK(At(instance, &g, 2.5f, kSeam, 3), "across the seam: the run and the walk");
    CHECK(At(instance, &g, -1.5f, kSeam, 3), "wrapped from below");
    CHECK(At(instance, &g, 6.5f, kSeam, 3), "wrapped from above");
    CHECK(At(instance, &g, 1.25f, kBetween, 3), "inside, as without the period");
    static const uint32_t kOne[1] = {1};
    static const float kPoint[1] = {0.5f};
    maniGraphParameterSource x = Parameter("x", maniGraphParameterFloat, Float(0.0f));
    maniGraphNodeSource alone[2] = {
        {maniGraphNodeBlendSpace1D, 1, kOne,
         .blendSpace1D = {{MANI_NO_PARAMETER, {0, 0.0f}, {0, 0.0f}}, kPoint, 2.0f}},
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    instance = Make(
        (maniGraphSource){.parameters = &x, .nodes = alone, .parameterCount = 1, .nodeCount = 2},
        &g);
    static const float kAll[1] = {1};
    // Its y names x, unused: stored as a literal 0, so the file opens.
    CHECK(At(instance, &g, 1.75f, kAll, 1), "one point, wrapped: all of it");
}

// Points (0, 0), (1, 0) and (0, 1), or an idle at the origin, a walk
// forward (1, 0), a walk left (0, 1) and a run forward (2, 0), polar.
static maniGraphInstance* Bands(bool polar, bool vector, maniGraph* g)
{
    static const uint32_t kInputs[4] = {1, 2, 3, 4};
    static const float kCartesian[6] = {0, 0, 1, 0, 0, 1};
    static const float kPolar[8] = {0, 0, 1, 0, 0, 1, 2, 0};
    static maniGraphParameterSource parameters[3];
    parameters[0] = Parameter("x", maniGraphParameterFloat, Float(0.0f));
    parameters[1] = Parameter("y", maniGraphParameterFloat, Float(0.0f));
    parameters[2] = Parameter("at", maniGraphParameterVec2, Float(0.0f));
    maniGraphPosition position = {MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}};
    if (vector)
    {
        position = (maniGraphPosition){2, LITERAL(0.0f), LITERAL(0.0f)};
    }
    uint32_t n = polar ? 4u : 3u;
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeBlendSpaceBands, n, kInputs,
         .bands = {position, polar ? kPolar : kCartesian, polar ? 1u : 0u}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip),
        ClipNode(4, 1.0f, maniGraphLoopClip)};
    return Make(
        (maniGraphSource){
            .parameters = parameters, .nodes = nodes, .parameterCount = 3, .nodeCount = n + 1},
        g);
}

static void TestBands(void)
{
    maniGraph g;
    maniGraphInstance* instance = Bands(false, false, &g);
    static const float kSecond[3] = {0, 1, 0};
    static const float kHalfway[3] = {0.5f, 0.5f, 0};
    static const float kCentre[3] = {0.5f, 0.25f, 0.25f};
    CHECK(AtPoint(instance, &g, 1.0f, 0.0f, kSecond, 3), "exact at a point");
    CHECK(AtPoint(instance, &g, 0.5f, 0.0f, kHalfway, 3), "halfway along an edge");
    // At (1/3, 1/3) the origin's influence is 2/3, the others' 1/3 (their
    // bands to each other end at 1/2, to the origin at 1/3): not
    // barycentric.
    CHECK(AtPoint(instance, &g, 1.0f / 3.0f, 1.0f / 3.0f, kCentre, 3), "the centre");
    // Polar: at (1.5, 0) the walk and the run forward share by speed; the
    // idle's and the walk left's bands from the walk forward reach 0.
    instance = Bands(true, false, &g);
    static const float kSpeeds[4] = {0, 0.5f, 0, 0.5f};
    CHECK(AtPoint(instance, &g, 1.5f, 0.0f, kSpeeds, 4), "polar: between speeds");
    static const float kIdle[4] = {1, 0, 0, 0};
    CHECK(AtPoint(instance, &g, 0.0f, 0.0f, kIdle, 4), "polar: the idle at the origin");
    // Halfway in angle between the walks (magnitude 1 at 45 degrees): the
    // walks' bands to each other end at 1/2; the idle's and the run's
    // toward the walk forward at 0.
    static const float kTurn[4] = {0, 0.5f, 0.5f, 0};
    CHECK(AtPoint(instance, &g, 0.70710678f, 0.70710678f, kTurn, 4), "polar: between directions");
    instance = Bands(false, true, &g);
    maniGraphValue at = {{0.5f, 0.0f, 0.0f, 0.0f}};
    Set(instance, 2, maniGraphParameterVec2, at);
    Update(instance, 0.0f);
    CHECK(Weights(instance, &g, kHalfway, 3), "the position from a vec2 parameter");
}

// Polar bands over an idle, walks forward and left, a run forward and a
// step back to the right, at positions where both the speed and the
// direction part of the bands count: weights from a binary64 model of
// Johansen's equations.
static void TestPolarMixed(void)
{
    static const uint32_t kInputs[5] = {1, 2, 3, 4, 5};
    static const float kPoints[10] = {0, 0, 1, 0, 0, 1, 2, 0, 0.5f, -0.75f};
    maniGraphParameterSource parameters[2] = {Parameter("x", maniGraphParameterFloat, Float(0)),
                                              Parameter("y", maniGraphParameterFloat, Float(0))};
    maniGraphNodeSource nodes[6] = {
        {maniGraphNodeBlendSpaceBands, 5, kInputs,
         .bands = {{MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}}, kPoints, 1}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip),
        ClipNode(4, 1.0f, maniGraphLoopClip),
        ClipNode(5, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .parameters = parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = 6},
        &g);
    static const float kLeft[5] = {0, 0.52623480f, 0.19500120f, 0.27876400f, 0};
    CHECK(AtPoint(instance, &g, 1.25f, 0.5f, kLeft, 5), "polar: ahead and to the left");
    static const float kRight[5] = {0.27527111f, 0.37477646f, 0, 0, 0.34995243f};
    CHECK(AtPoint(instance, &g, 0.5f, -0.25f, kRight, 5), "polar: slow and to the right");
}

// Polar bands across the seam at half a turn, where a turn must wrap
// (points at 0 and either side of 180 degrees), and at the origin, whose
// direction no turn uses: weights from the binary64 model.
static void TestPolarTurns(void)
{
    static const uint32_t kInputs[3] = {1, 2, 3};
    static const float kSeam[6] = {1, 0, -1, 0.25f, -1, -0.25f};
    static const float kRing[6] = {1, 0, 0, 2, -1, 0.5f};
    maniGraphParameterSource parameters[2] = {Parameter("x", maniGraphParameterFloat, Float(0)),
                                              Parameter("y", maniGraphParameterFloat, Float(0))};
    for (int k = 0; k < 2; ++k)
    {
        maniGraphNodeSource nodes[4] = {
            {maniGraphNodeBlendSpaceBands, 3, kInputs,
             .bands = {{MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}}, k == 0 ? kSeam : kRing, 1}},
            ClipNode(0, 1.0f, maniGraphLoopClip),
            ClipNode(1, 1.0f, maniGraphLoopClip),
            ClipNode(3, 1.0f, maniGraphLoopClip)};
        maniGraph g;
        maniGraphInstance* instance = Make(
            (maniGraphSource){
                .parameters = parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = 4},
            &g);
        static const float kAcross[3] = {0, 0.62739642f, 0.37260358f};
        static const float kOrigin[3] = {0.35002322f, 0.30138873f, 0.34858805f};
        CHECK(k == 0 ? AtPoint(instance, &g, -1.0f, 0.0625f, kAcross, 3)
                     : AtPoint(instance, &g, 0.0f, 0.0f, kOrigin, 3),
              k == 0 ? "polar: across the seam" : "polar: at the origin");
    }
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[4][4096];

static maniPose* PoseAt(int i)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], &pose) == mani_success,
          "a pose");
    return pose;
}

// The line at 1.25: the run and the step at half each, as the blending
// functions make it.
static void TestEvaluated(void)
{
    maniGraph g;
    maniGraphInstance* instance = Line(0.0f, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(1.25f));
    Update(instance, 0.25f);
    maniPose* out = PoseAt(0);
    maniPose* run = PoseAt(1);
    maniPose* step = PoseAt(2);
    maniPose* want = PoseAt(3);
    const maniBlendLayer layers[2] = {{run, 0.5f, NULL}, {step, 0.5f, NULL}};
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              maniSampleClip(&s_clips[1], &s_skeleton, 0.25f, run) == mani_success &&
              maniSampleClip(&s_clips[3], &s_skeleton, 0.25f, step) == mani_success &&
              maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, want) == mani_success,
          "evaluated");
    bool same = true;
    for (uint32_t bone = 0; bone < 3; ++bone)
    {
        maniTransform a = maniPoseLocal(out, bone);
        maniTransform b = maniPoseLocal(want, bone);
        same = same && memcmp(&a, &b, sizeof a) == 0;
    }
    CHECK(same, "the space's pose, bit for bit");
}

static void TestRefusals(void)
{
    static const uint32_t kInputs[3] = {1, 2, 3};
    float points[6] = {-1.0f, 2.0f, -1.0f, 0, 0, 0};
    maniGraphParameterSource n = Parameter("n", maniGraphParameterInt, Int(0));
    maniGraphPosition literal = {MANI_NO_PARAMETER, LITERAL(0.0f), LITERAL(0.0f)};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeBlendSpace1D, 3, kInputs, .blendSpace1D = {literal, points, 0.0f}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip)};
    // One point more than a space may have, each input a clip node.
    static uint32_t kMany[MANI_GRAPH_MAX_SPACE_POINTS + 1];
    static float many[MANI_GRAPH_MAX_SPACE_POINTS + 1];
    for (uint32_t k = 0; k <= MANI_GRAPH_MAX_SPACE_POINTS; ++k)
    {
        kMany[k] = 1 + k % 3;
        many[k] = (float)k;
    }
    maniGraphNodeSource wide[4] = {nodes[0], nodes[1], nodes[2], nodes[3]};
    wide[0].inputCount = MANI_GRAPH_MAX_SPACE_POINTS + 1;
    wide[0].inputs = kMany;
    wide[0].blendSpace1D.points = many;
    CHECK(Refused(BuildOnly(wide, 4, NULL, 0), maniBuildReasonNode, 0), "a point past the most");
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonPoint, 2), "two equal points");
    points[2] = NAN;
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonPoint, 2), "a point not finite");
    points[2] = 0.5f;
    nodes[0].blendSpace1D.period = 3.0f;
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonPoint, 1),
          "points spanning the period");
    nodes[0].blendSpace1D.period = -1.0f;
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonNode, 0), "a negative period");
    nodes[0].blendSpace1D.period = 0.0f;
    nodes[0].blendSpace1D.points = NULL;
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonNode, 0), "no points");
    nodes[0].blendSpace1D.points = points;
    nodes[0].blendSpace1D.position.y = (maniGraphScalar){7, 0.0f};
    CHECK(BuildOnly(nodes, 4, NULL, 0).result == mani_success, "a 1D space's y unused");
    nodes[0].blendSpace1D.position.x = (maniGraphScalar){0, 0.0f};
    CHECK(Refused(BuildOnly(nodes, 4, &n, 1), maniBuildReasonScalar, 0), "x from an int");
    nodes[0].blendSpace1D.position = (maniGraphPosition){0, LITERAL(0.0f), LITERAL(0.0f)};
    CHECK(Refused(BuildOnly(nodes, 4, &n, 1), maniBuildReasonScalar, 0), "a vector not vec2");
    float pairs[6] = {0, 0, 1, 0, 0, 0};
    nodes[0] = (maniGraphNodeSource){maniGraphNodeBlendSpaceBands, 3, kInputs,
                                     .bands = {literal, pairs, 0}};
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonPoint, 2), "two equal pairs");
    pairs[5] = 1.0f;
    nodes[0].bands.polar = 2;
    CHECK(Refused(BuildOnly(nodes, 4, NULL, 0), maniBuildReasonNode, 0), "a polar flag past 1");
    nodes[0].bands.polar = 0;
    CHECK(BuildOnly(nodes, 4, NULL, 0).result == mani_success, "builds when right");
    nodes[0].bands.position.y = (maniGraphScalar){0, 0.0f};
    CHECK(Refused(BuildOnly(nodes, 4, &n, 1), maniBuildReasonScalar, 1), "y from an int");
}

static void TestOpenRefusals(void)
{
    maniGraph g;
    (void)Line(4.0f, &g);
    // Words: the vector at 0, x at 1, y at 3, the period at 5, the points
    // from 6, the order from 9.
    CHECK(RefusedAtWord(&g, 3, 5, Bits(-1.0f)), "a negative period");
    CHECK(RefusedAtWord(&g, 3, 5, Bits(2.0f)), "a period the points span");
    CHECK(RefusedAtWord(&g, 3, 7, 0x7FC00000u), "a point not finite");
    CHECK(RefusedAtWord(&g, 3, 10, 0), "an order not ascending");
    CHECK(RefusedAtWord(&g, 3, 11, 7), "an order past the inputs");
    CHECK(RefusedAtWord(&g, 3, 0, 0), "a vector from a float");
    CHECK(RefusedAtWord(&g, 3, 3, 0), "a 1D space's y from a parameter");
    CHECK(RefusedAtWord(&g, 3, 4, Bits(1.0f)), "a 1D space's y not zero");
    (void)Bands(false, true, &g);
    // Words: the vector at 0, x at 1, y at 3, the flag at 5, the points
    // from 6.
    CHECK(RefusedAtWord(&g, 3, 1, 0), "a vector with a scalar after it");
    CHECK(RefusedAtWord(&g, 3, 5, 2), "a polar flag past 1");
    CHECK(RefusedAtWord(&g, 3, 8, Bits(0.0f)), "two equal points");
}

// A 1D space in a subgraph reading the outer graph's parameter: built as
// the same graph written flat.
static void TestSubgraph(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const float kPoints[2] = {0.0f, 1.0f};
    maniGraphParameterSource s = Parameter("s", maniGraphParameterFloat, Float(0.0f));
    maniGraphPosition position = {MANI_NO_PARAMETER, {0, 0.0f}, LITERAL(0.0f)};
    maniGraphNodeSource inner[3] = {
        {maniGraphNodeBlendSpace1D, 2, kInputs, .blendSpace1D = {position, kPoints, 0.0f}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraphSource sub = {&s_skeleton, &s, kSlots, NULL, inner, 1,    CLIPS, 0,   3,
                           0,           0,  NULL,   0,    0,     NULL, NULL,  NULL};
    maniGraphParameterSource outer[2] = {Parameter("a", maniGraphParameterFloat, Float(0.0f)),
                                         Parameter("speed", maniGraphParameterFloat, Float(0.0f))};
    static const uint32_t kMap[1] = {1};
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&sub, kMap}};
    maniGraphNodeSource flat[3] = {inner[0], inner[1], inner[2]};
    flat[0].blendSpace1D.position.x = (maniGraphScalar){1, 0.0f};
    static _Alignas(16) uint8_t files[2][16384];
    size_t sizes[2] = {0, 0};
    const maniGraphSource sources[2] = {{&s_skeleton, outer, kSlots, NULL, &node, 2, CLIPS, 0, 1, 0,
                                         0, NULL, 0, 0, NULL, NULL, NULL},
                                        {&s_skeleton, outer, kSlots, NULL, flat, 2, CLIPS, 0, 3, 0,
                                         0, NULL, 0, 0, NULL, NULL, NULL}};
    for (int k = 0; k < 2; ++k)
    {
        maniBuilder* builder = NewBuilder();
        CHECK(maniBuildGraph(builder, &sources[k]).result == mani_success &&
                  maniCopyBuilt(builder, files[k], sizeof files[k], &sizes[k]) == mani_success,
              "builds");
        maniDestroyBuilder(builder);
    }
    CHECK(sizes[0] == sizes[1] && memcmp(files[0], files[1], sizes[0]) == 0,
          "a space inlined as written flat");
}

static char s_text[16384];
static _Alignas(16) uint8_t s_built[16384];
static size_t s_textSize;
static size_t s_builtSize;

// Both kinds written as JSON and read back to the same bytes; the text
// and the file kept as the fuzz targets' seeds.
static void TestJson(void)
{
    static const uint32_t kLineInputs[2] = {1, 2};
    static const uint32_t kBandInputs[3] = {3, 4, 5};
    static const uint32_t kOutput[2] = {0, 6};
    static const float kLine[2] = {-1.0f, 0.5f};
    static const float kPairs[6] = {0, 0, 1, 0.25f, -0.5f, 1};
    static const maniGraphScalar kHalves[2] = {LITERAL(0.5f), LITERAL(0.5f)};
    maniGraphParameterSource parameters[2] = {Parameter("x", maniGraphParameterFloat, Float(0.0f)),
                                              Parameter("at", maniGraphParameterVec2, Float(0.0f))};
    maniGraphNodeSource nodes[7] = {
        {maniGraphNodeBlendSpace1D, 2, kLineInputs,
         .blendSpace1D = {{MANI_NO_PARAMETER, {0, 0.0f}, LITERAL(0.0f)}, kLine, 2.0f}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip),
        ClipNode(4, 1.0f, maniGraphLoopClip),
        ClipNode(5, 1.0f, maniGraphLoopClip),
        {maniGraphNodeBlendSpaceBands, 3, kBandInputs,
         .bands = {{1, LITERAL(0.0f), LITERAL(0.0f)}, kPairs, 1}}};
    maniGraphNodeSource all[8];
    memcpy(all, nodes, sizeof nodes);
    all[7] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kOutput, .blend = {kHalves, 0}};
    maniGraphSource source = {&s_skeleton, parameters, kSlots, NULL, all, 2,    CLIPS, 0,   8,
                              7,           0,          NULL,   0,    0,   NULL, NULL,  NULL};
    static _Alignas(16) uint8_t read[16384];
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text, &s_textSize) == mani_success &&
              maniBuildGraphJson(builder, s_text, s_textSize,
                                 &(maniGraphNamedAssets){&s_skeleton, kSlots, NULL, NULL, NULL,
                                                         CLIPS, 0, 0, 0, NULL})
                      .result == mani_success &&
              maniCopyBuilt(builder, read, sizeof read, &readSize) == mani_success,
          "written and read");
    maniDestroyBuilder(builder);
    CHECK(readSize == s_builtSize && memcmp(read, s_built, s_builtSize) == 0, "the same bytes");
    CHECK(strstr(s_text, "\"kind\": \"blendSpace1D\"") != NULL &&
              strstr(s_text, "\"position\": {\"vector\": \"at\"}") != NULL &&
              strstr(s_text, "\"points\": [[0, 0], [1, 0.25], [-0.5, 1]], \"polar\": true") != NULL,
          "the form's members");
}

// A 1D space's period, a banded space's polar flag and a position's y
// left out of the text: 0, false and 0.
static void TestJsonDefaults(void)
{
    static const char kText[] =
        "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": \"x\", "
        "\"type\": \"float\"}], \"clips\": [\"walk\", \"run\"], \"nodes\": [{\"kind\": "
        "\"blendSpaceBands\", \"inputs\": [1, 2], \"position\": {\"x\": 0.5}, \"points\": [[0, "
        "0], [1, 0]]}, {\"kind\": \"blendSpace1D\", \"inputs\": [2, 3], \"position\": {\"x\": "
        "{\"parameter\": \"x\"}}, \"points\": [0, 1]}, {\"kind\": \"clip\", \"clip\": "
        "\"walk\"}, {\"kind\": \"clip\", \"clip\": \"run\"}], \"output\": 0}";
    static const uint32_t kBands[2] = {1, 2};
    static const uint32_t kLine[2] = {2, 3};
    static const float kPairs[4] = {0, 0, 1, 0};
    static const float kPoints[2] = {0, 1};
    maniGraphParameterSource x = Parameter("x", maniGraphParameterFloat, Float(0.0f));
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeBlendSpaceBands, 2, kBands,
         .bands = {{MANI_NO_PARAMETER, LITERAL(0.5f), LITERAL(0.0f)}, kPairs, 0}},
        {maniGraphNodeBlendSpace1D, 2, kLine,
         .blendSpace1D = {{MANI_NO_PARAMETER, {0, 0.0f}, LITERAL(0.0f)}, kPoints, 0.0f}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraphSource source = {&s_skeleton, &x, kSlots, NULL, nodes, 1,    2,    0,   4,
                              0,           0,  NULL,   0,    0,     NULL, NULL, NULL};
    static _Alignas(16) uint8_t built[16384];
    static _Alignas(16) uint8_t read[16384];
    size_t builtSize = 0;
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, built, sizeof built, &builtSize) == mani_success &&
              maniBuildGraphJson(builder, kText, sizeof kText - 1,
                                 &(maniGraphNamedAssets){&s_skeleton, kSlots, NULL, NULL, NULL,
                                                         CLIPS, 0, 0, 0, NULL})
                      .result == mani_success &&
              maniCopyBuilt(builder, read, sizeof read, &readSize) == mani_success,
          "built and read");
    maniDestroyBuilder(builder);
    CHECK(readSize == builtSize && memcmp(read, built, builtSize) == 0, "members left out");
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

int main(int argc, char** argv)
{
    Open();
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestLine();
    TestBands();
    TestPolarMixed();
    TestPolarTurns();
    TestEvaluated();
    TestRefusals();
    TestOpenRefusals();
    TestSubgraph();
    TestJson();
    TestJsonDefaults();
    return s_failures == 0 ? 0 : 1;
}
