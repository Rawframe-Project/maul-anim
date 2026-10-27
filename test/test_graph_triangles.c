// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blend spaces by triangles: a square and its centre
// triangulated into four, weighed barycentrically inside and by the
// nearest edge or corner outside; a square's cocircular tie kept as the
// sweep made it; a grid's triangulation pinned; authored triangles,
// their union not convex; a space evaluated as the blending functions
// blend; the builder's refusals (points on one line, triangles
// clockwise, overlapping, contained, too many, a point in none) and
// opening's checks; a space in a subgraph; the JSON form round trip. Run
// as test_graph_triangles --seed <graph file> <json file> it writes the
// round trip's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_space_fixture.h"

#include "maul-anim/pose.h"

#include <stdio.h>

// The clips the inputs play: the walk, the run, the short and long
// steps, the double and triple steps.
static const uint32_t kClipOf[6] = {0, 1, 3, 4, 5, 6};

static const float kSquare[10] = {0, 0, 1, 0, 1, 1, 0, 1, 0.5f, 0.5f};

// A space over n points at (x, y) from parameters 0 and 1 (the source's
// triangles, or none), its inputs clip nodes 1 to n.
static maniGraphInstance* Space(const float* points, uint32_t n, const uint32_t* triangles,
                                uint32_t count, maniGraph* g)
{
    static const uint32_t kInputs[6] = {1, 2, 3, 4, 5, 6};
    static maniGraphParameterSource parameters[2];
    parameters[0] = Parameter("x", maniGraphParameterFloat, Float(0.0f));
    parameters[1] = Parameter("y", maniGraphParameterFloat, Float(0.0f));
    maniGraphNodeSource nodes[7];
    nodes[0] = (maniGraphNodeSource){
        maniGraphNodeBlendSpaceTriangles, n, kInputs,
        .triangulated = {{MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}}, points, triangles, count}};
    for (uint32_t k = 0; k < n; ++k)
    {
        nodes[k + 1] = ClipNode(kClipOf[k], 1.0f, maniGraphLoopClip);
    }
    return Make(
        (maniGraphSource){
            .parameters = parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = n + 1},
        g);
}

// The square's corners 0 to 3 anticlockwise from the origin, its centre
// 4: four triangles about the centre.
static void TestSquare(void)
{
    maniGraph g;
    maniGraphInstance* instance = Space(kSquare, 5, NULL, 0, &g);
    CHECK(DataWord(&g, 5, MANI_GRAPH_POSITION_WORDS) == 4, "four triangles");
    static const float kInside[5] = {0.25f, 0.25f, 0, 0, 0.5f};
    CHECK(AtPoint(instance, &g, 0.5f, 0.25f, kInside, 5), "barycentric inside");
    static const float kCentre[5] = {0, 0, 0, 0, 1};
    CHECK(AtPoint(instance, &g, 0.5f, 0.5f, kCentre, 5), "on a point, that point");
    static const float kBelow[5] = {0.5f, 0.5f, 0, 0, 0};
    CHECK(AtPoint(instance, &g, 0.5f, -1.0f, kBelow, 5), "below: the nearest edge's middle");
    static const float kRight[5] = {0, 0.75f, 0.25f, 0, 0};
    CHECK(AtPoint(instance, &g, 3.0f, 0.25f, kRight, 5), "right: along the nearest edge");
    static const float kCorner[5] = {0, 0, 1, 0, 0};
    CHECK(AtPoint(instance, &g, 2.0f, 2.0f, kCorner, 5), "past a corner: the corner");
}

// The square alone: its corners all on one circle, no flip; the sweep
// (the points by x, then y) joins (1, 0) to (0, 1), so the centre lies on
// that diagonal.
static void TestTie(void)
{
    maniGraph g;
    maniGraphInstance* instance = Space(kSquare, 4, NULL, 0, &g);
    CHECK(DataWord(&g, 4, MANI_GRAPH_POSITION_WORDS) == 2, "two triangles");
    static const float kDiagonal[4] = {0, 0.5f, 0, 0.5f};
    CHECK(AtPoint(instance, &g, 0.5f, 0.5f, kDiagonal, 4), "the sweep's diagonal");
}

// The square's two triangles under a time scale: the third reserved
// triangle zero and the time scale's words after it, as opening checks.
static void TestReserved(void)
{
    static const uint32_t kInputs[4] = {2, 3, 4, 5};
    static const uint32_t kScaled[1] = {1};
    maniGraphParameterSource parameters[2] = {Parameter("x", maniGraphParameterFloat, Float(0)),
                                              Parameter("y", maniGraphParameterFloat, Float(0))};
    maniGraphNodeSource nodes[6] = {
        {maniGraphNodeTimeScale, 1, kScaled, .timeScale = {LITERAL(2.0f)}},
        {maniGraphNodeBlendSpaceTriangles, 4, kInputs,
         .triangulated = {{MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}}, kSquare, NULL, 0}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip),
        ClipNode(4, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .parameters = parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = 6},
        &g);
    static const float kDiagonal[4] = {0, 0.5f, 0, 0.5f};
    CHECK(AtPoint(instance, &g, 0.5f, 0.5f, kDiagonal, 4), "a space not the output");
}

// A 3 by 3 grid: every cell a tie, its eight triangles pinned.
static void TestGrid(void)
{
    static const float kGrid[18] = {0, 0, 1, 0, 2, 0, 0, 1, 1, 1, 2, 1, 0, 2, 1, 2, 2, 2};
    static const uint32_t kWant[24] = {3, 0, 1, 3, 1, 4, 3, 4, 6, 6, 4, 7,
                                       4, 1, 2, 4, 2, 5, 4, 5, 7, 7, 5, 8};
    maniGraphParameterSource parameters[2] = {Parameter("x", maniGraphParameterFloat, Float(0)),
                                              Parameter("y", maniGraphParameterFloat, Float(0))};
    static const uint32_t kInputs[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    maniGraphNodeSource nodes[2] = {
        {maniGraphNodeBlendSpaceTriangles, 9, kInputs,
         .triangulated = {{MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}}, kGrid, NULL, 0}},
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    (void)Make(
        (maniGraphSource){
            .parameters = parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = 2},
        &g);
    bool same = DataWord(&g, 1, MANI_GRAPH_POSITION_WORDS) == 8;
    for (uint32_t k = 0; k < 24; ++k)
    {
        uint32_t word = DataWord(&g, 1, MANI_GRAPH_SPACE_WORDS + 18 + k);
        if (word != kWant[k])
        {
            printf("  corner %u: %u, want %u\n", k, word, kWant[k]);
            same = false;
        }
    }
    CHECK(same, "the grid's triangles");
}

// An L of six points in four authored triangles: inside as authored, in
// the notch the nearest edge.
static void TestAuthored(void)
{
    static const float kL[12] = {0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2};
    static const uint32_t kFan[12] = {0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5};
    maniGraph g;
    maniGraphInstance* instance = Space(kL, 6, kFan, 4, &g);
    CHECK(DataWord(&g, 6, MANI_GRAPH_POSITION_WORDS) == 4, "the authored count");
    static const float kNotch[6] = {0, 0, 0.5f, 0.5f, 0, 0};
    CHECK(AtPoint(instance, &g, 1.5f, 1.2f, kNotch, 6), "the notch: the nearest edge");
    static const float kFirst[6] = {0.25f, 0.5f, 0.25f, 0, 0, 0};
    CHECK(AtPoint(instance, &g, 1.5f, 0.25f, kFirst, 6), "inside the first triangle");
    static const uint32_t kHalves[6] = {0, 1, 2, 0, 2, 3};
    instance = Space(kSquare, 4, kHalves, 2, &g);
    static const float kShared[4] = {0.5f, 0, 0.5f, 0};
    CHECK(AtPoint(instance, &g, 0.5f, 0.5f, kShared, 4), "on a shared edge: the first triangle");
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[5][4096];

// The square and centre at (0.5, 0.25): the walk, the run and the double
// step blended as the blending functions blend them.
static void TestEvaluated(void)
{
    maniGraph g;
    maniGraphInstance* instance = Space(kSquare, 5, NULL, 0, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(0.5f));
    Set(instance, 1, maniGraphParameterFloat, Float(0.25f));
    Update(instance, 0.25f);
    maniPose* poses[5];
    for (int i = 0; i < 5; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], &poses[i]) ==
                  mani_success,
              "a pose");
    }
    const maniBlendLayer layers[3] = {
        {poses[1], 0.25f, NULL}, {poses[2], 0.25f, NULL}, {poses[3], 0.5f, NULL}};
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, poses[0]) == mani_success &&
              maniSampleClip(&s_clips[0], &s_skeleton, 0.25f, poses[1]) == mani_success &&
              maniSampleClip(&s_clips[1], &s_skeleton, 0.25f, poses[2]) == mani_success &&
              maniSampleClip(&s_clips[5], &s_skeleton, 0.25f, poses[3]) == mani_success &&
              maniBlendPoses(&s_skeleton, layers, 3, MANI_BLEND_THRESHOLD, poses[4]) ==
                  mani_success,
          "evaluated");
    bool same = true;
    for (uint32_t bone = 0; bone < 3; ++bone)
    {
        maniTransform a = maniPoseLocal(poses[0], bone);
        maniTransform b = maniPoseLocal(poses[4], bone);
        same = same && memcmp(&a, &b, sizeof a) == 0;
    }
    CHECK(same, "the space's pose, bit for bit");
}

static maniBuildResult Triangles(const float* points, uint32_t n, const uint32_t* triangles,
                                 uint32_t count)
{
    static const uint32_t kInputs[6] = {1, 2, 3, 4, 5, 6};
    maniGraphNodeSource nodes[7];
    nodes[0] = (maniGraphNodeSource){
        maniGraphNodeBlendSpaceTriangles, n, kInputs,
        .triangulated = {
            {MANI_NO_PARAMETER, LITERAL(0.0f), LITERAL(0.0f)}, points, triangles, count}};
    for (uint32_t k = 0; k < n; ++k)
    {
        nodes[k + 1] = ClipNode(kClipOf[k], 1.0f, maniGraphLoopClip);
    }
    return BuildOnly(nodes, n + 1, NULL, 0);
}

static void TestRefusals(void)
{
    static const float kLine[8] = {0, 0, 1, 1, 3, 3, -2, -2};
    CHECK(Refused(Triangles(kLine, 4, NULL, 0), maniBuildReasonNode, 0), "points on one line");
    CHECK(Refused(Triangles(kSquare, 2, NULL, 0), maniBuildReasonNode, 0), "two points");
    static const float kRepeated[8] = {0, 0, 1, 0, 1, 1, 1, 0};
    CHECK(Refused(Triangles(kRepeated, 4, NULL, 0), maniBuildReasonPoint, 3), "a point twice");
    static const uint32_t kClockwise[6] = {0, 1, 2, 0, 3, 2};
    CHECK(Refused(Triangles(kSquare, 4, kClockwise, 2), maniBuildReasonSpaceTriangle, 1),
          "a triangle clockwise");
    static const uint32_t kPast[6] = {0, 2, 3, 0, 1, 4};
    CHECK(Refused(Triangles(kSquare, 4, kPast, 2), maniBuildReasonSpaceTriangle, 1),
          "a corner past the points");
    static const uint32_t kDegenerate[6] = {0, 1, 2, 0, 2, 2};
    CHECK(Refused(Triangles(kSquare, 4, kDegenerate, 2), maniBuildReasonSpaceTriangle, 1),
          "a corner twice");
    static const uint32_t kOverlap[6] = {0, 1, 2, 0, 1, 3};
    CHECK(Refused(Triangles(kSquare, 4, kOverlap, 2), maniBuildReasonSpaceTriangle, 1),
          "two triangles overlapping");
    static const float kNested[12] = {0, 0, 4, 0, 0, 4, 1, 1, 2, 1, 1, 2};
    static const uint32_t kInside[6] = {0, 1, 2, 3, 4, 5};
    CHECK(Refused(Triangles(kNested, 6, kInside, 2), maniBuildReasonSpaceTriangle, 1),
          "a triangle inside another");
    static const uint32_t kOne[3] = {0, 1, 2};
    CHECK(Refused(Triangles(kSquare, 4, kOne, 1), maniBuildReasonPoint, 3), "a point in none");
    static const uint32_t kFour[12] = {0, 1, 2, 0, 2, 3, 0, 1, 3, 1, 2, 3};
    CHECK(Refused(Triangles(kSquare, 4, kFour, 4), maniBuildReasonNode, 0), "too many");
    CHECK(Refused(Triangles(kSquare, 4, NULL, 1), maniBuildReasonNode, 0), "no triangles");
    // Apart, their boxes overlapping, across an edge of the second only.
    static const float kApart[12] = {0, 0, 4, 0, 0, 4, 3.5f, -1, 6, 0, 4.375f, 0.5f};
    CHECK(Triangles(kApart, 6, kInside, 2).result == mani_success,
          "apart across the second's edge");
    static const uint32_t kTouching[6] = {0, 1, 4, 1, 2, 4};
    static const float kKite[10] = {0, 0, 1, 0, 1, 1, 9, 9, 0.5f, 0.5f};
    CHECK(Refused(Triangles(kKite, 5, kTouching, 2), maniBuildReasonPoint, 3),
          "triangles sharing an edge: past the overlap test");
}

// The square and centre: count, then points, then triangles from word
// 16, the fifth (reserved) from 28.
static void TestOpenRefusals(void)
{
    maniGraph g;
    (void)Space(kSquare, 5, NULL, 0, &g);
    CHECK(RefusedAtWord(&g, 5, 5, 0), "no triangles");
    CHECK(RefusedAtWord(&g, 5, 5, 6), "more than reserved");
    CHECK(RefusedAtWordOf(&g, 5, 7, 0x7FC00000u, 6), "a point not finite");
    CHECK(RefusedAtWord(&g, 5, 8, Bits(0.0f)), "two equal points");
    CHECK(RefusedAtWord(&g, 5, 16, 5), "a corner past the points");
    CHECK(RefusedAtWordOf(&g, 5, 17, DataWord(&g, 5, 16), 16), "a corner twice");
    CHECK(RefusedAtWord(&g, 5, 28, 1), "a reserved triangle not zero");
    CHECK(RefusedAtWord(&g, 5, 3, 7), "y from no parameter");
}

// A space of authored triangles in a subgraph reading the outer graph's
// parameters: built as the same graph written flat.
static void TestSubgraph(void)
{
    static const uint32_t kInputs[3] = {1, 2, 3};
    static const float kPoints[6] = {0, 0, 1, 0, 0, 1};
    static const uint32_t kOne[3] = {0, 1, 2};
    maniGraphParameterSource inner[2] = {Parameter("u", maniGraphParameterFloat, Float(0)),
                                         Parameter("v", maniGraphParameterFloat, Float(0))};
    maniGraphPosition position = {MANI_NO_PARAMETER, {0, 0.0f}, {1, 0.0f}};
    maniGraphNodeSource nodes[4] = {{maniGraphNodeBlendSpaceTriangles, 3, kInputs,
                                     .triangulated = {position, kPoints, kOne, 1}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip),
                                    ClipNode(3, 1.0f, maniGraphLoopClip)};
    maniGraphSource sub = {&s_skeleton, inner, kSlots, NULL, nodes, 2,    CLIPS, 0,   4,
                           0,           0,     NULL,   0,    0,     NULL, NULL,  NULL};
    maniGraphParameterSource outer[3] = {Parameter("a", maniGraphParameterFloat, Float(0)),
                                         Parameter("x", maniGraphParameterFloat, Float(0)),
                                         Parameter("y", maniGraphParameterFloat, Float(0))};
    static const uint32_t kMap[2] = {1, 2};
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&sub, kMap}};
    maniGraphNodeSource flat[4] = {nodes[0], nodes[1], nodes[2], nodes[3]};
    flat[0].triangulated.position = (maniGraphPosition){MANI_NO_PARAMETER, {1, 0.0f}, {2, 0.0f}};
    static _Alignas(16) uint8_t files[2][16384];
    size_t sizes[2] = {0, 0};
    const maniGraphSource sources[2] = {{&s_skeleton, outer, kSlots, NULL, &node, 3, CLIPS, 0, 1, 0,
                                         0, NULL, 0, 0, NULL, NULL, NULL},
                                        {&s_skeleton, outer, kSlots, NULL, flat, 3, CLIPS, 0, 4, 0,
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

// A Delaunay space and an authored one written as JSON and read back to
// the same bytes; the text and the file kept as the fuzz targets' seeds.
static void TestJson(void)
{
    static const uint32_t kFirst[3] = {1, 2, 3};
    static const uint32_t kSecond[4] = {1, 2, 3, 4};
    static const uint32_t kOutput[2] = {0, 5};
    static const float kPoints[6] = {0, 0, 1, 0, 0.25f, -1};
    static const uint32_t kHalves[6] = {0, 1, 2, 0, 2, 3};
    static const maniGraphScalar kHalf[2] = {LITERAL(0.5f), LITERAL(0.5f)};
    maniGraphParameterSource parameters[2] = {Parameter("x", maniGraphParameterFloat, Float(0.0f)),
                                              Parameter("at", maniGraphParameterVec2, Float(0.0f))};
    maniGraphNodeSource nodes[7] = {
        {maniGraphNodeBlendSpaceTriangles, 3, kFirst,
         .triangulated = {{MANI_NO_PARAMETER, {0, 0.0f}, LITERAL(0.5f)}, kPoints, NULL, 0}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip),
        ClipNode(4, 1.0f, maniGraphLoopClip),
        {maniGraphNodeBlendSpaceTriangles, 4, kSecond,
         .triangulated = {{1, LITERAL(0.0f), LITERAL(0.0f)}, kSquare, kHalves, 2}},
        {maniGraphNodeBlend, 2, kOutput, .blend = {kHalf, 0}}};
    maniGraphSource source = {&s_skeleton, parameters, kSlots, NULL, nodes, 2,    CLIPS, 0,   7,
                              6,           0,          NULL,   0,    0,     NULL, NULL,  NULL};
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
    CHECK(strstr(s_text, "\"points\": [[0, 0], [1, 0], [0.25, -1]]}") != NULL &&
              strstr(s_text, "\"triangles\": [[0, 1, 2], [0, 2, 3]]}") != NULL,
          "triangles written when authored alone");
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
    TestSquare();
    TestTie();
    TestReserved();
    TestGrid();
    TestAuthored();
    TestEvaluated();
    TestRefusals();
    TestOpenRefusals();
    TestSubgraph();
    TestJson();
    return s_failures == 0 ? 0 : 1;
}
