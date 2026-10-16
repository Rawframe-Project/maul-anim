// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inertialization (mani-0023): the quintic against Bollo's published
// form, its ends and its overshoot clamp; an inertial transition eased
// out from the last output, the speed clamped when moving away and kept
// when moving toward, the duration cut when the speed would carry past
// the input, the input exact once the time is up; no speed after a
// skipped evaluation; a second evaluation in one update not taken as
// motion; a request before any output dropped; the least of one
// update's requests, a machine chain's included; the node's trigger;
// its clock under a time scale; a rotation eased by its axis and angle;
// each channel's speed from outputs set by hand, the turn applied before
// the input's rotation; a seek above clearing it; saved and restored
// mid-ease; the builder's refusals (no node above, a path around it,
// half-lives against the method), opening's checks and the saved
// block's; the JSON form, round trip and refusals. Dead blending has its
// own suite. Run as test_graph_inertia --seed <graph file> <json file>
// it writes the round trip's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_inertia.h"
#include "graph_inertia_apply.h"
#include "graph_inertia_fixture.h"

#include "maul-anim/pose.h"

#include <stdio.h>

// Bollo's form as published (SIGGRAPH 2017 supplemental, equations 2 to
// 7), in binary64, x0 positive.
static double Published(double x0, double v0, double t1, double t)
{
    v0 = v0 > 0.0 ? 0.0 : v0;
    t1 = v0 < 0.0 && -5.0 * x0 / v0 < t1 ? -5.0 * x0 / v0 : t1;
    if (t >= t1)
    {
        return 0.0;
    }
    double t2 = t1 * t1;
    double a0 = (-8.0 * t1 * v0 - 20.0 * x0) / t2;
    a0 = a0 > 0.0 ? a0 : 0.0;
    double a = (a0 * t2 + 6.0 * t1 * v0 + 12.0 * x0) / (t2 * t2 * t1);
    double b = (3.0 * a0 * t2 + 16.0 * t1 * v0 + 30.0 * x0) / (t2 * t2);
    double c = (3.0 * a0 * t2 + 12.0 * t1 * v0 + 20.0 * x0) / (t2 * t1);
    double s2 = t * t;
    return (-a * s2 * s2 * t + b * s2 * s2 - c * s2 * t + a0 * s2) / 2.0 + v0 * t + x0;
}

// The library's quintic takes the clamped speed and cut duration.
static float Library(float x0, float v0, float t1, float t)
{
    v0 = v0 > 0.0f ? 0.0f : v0;
    t1 = v0 < 0.0f && -5.0f * x0 / v0 < t1 ? -5.0f * x0 / v0 : t1;
    return maniGraphQuintic(x0, v0, t1, t);
}

static void TestQuintic(void)
{
    static const float kCases[6][3] = {{0.3f, 0.0f, 0.5f},  {0.7f, -1.0f, 0.5f},
                                       {1.0f, -8.0f, 0.5f}, {1.0f, -40.0f, 1.0f},
                                       {0.2f, 3.0f, 0.25f}, {2.0f, -2.0f, 0.4f}};
    bool same = true;
    for (int k = 0; k < 6; ++k)
    {
        for (int i = 0; i <= 10; ++i)
        {
            float t = kCases[k][2] * (float)i / 10.0f;
            double want = Published((double)kCases[k][0], (double)kCases[k][1],
                                    (double)kCases[k][2], (double)t);
            float got = Library(kCases[k][0], kCases[k][1], kCases[k][2], t);
            same = same && fabs((double)got - want) <= 1e-5 * (1.0 + fabs(want));
        }
    }
    CHECK(same, "the published quintic");
    CHECK(maniGraphQuintic(0.5f, -1.0f, 0.4f, 0.0f) == 0.5f, "x0 at the start");
    CHECK(maniGraphQuintic(0.5f, -1.0f, 0.4f, 0.4f) == 0.0f, "0 at its end");
    CHECK(maniGraphQuintic(0.0f, -1.0f, 0.4f, 0.1f) == 0.0f, "nothing to ease");
    bool above = true;
    for (int i = 0; i < 100; ++i)
    {
        above = above && Library(1.0f, -40.0f, 1.0f, (float)i / 100.0f) >= 0.0f;
    }
    CHECK(above, "a fast approach cut short: no overshoot");
}

// The walk at twice its speed, moving away from the run (entered at its
// start): its speed clamped to 0; the output the run plus the quintic
// from the offset at 0.1 into the half second, closing in; the run alone
// from 0.5 on.
static void TestTransition(void)
{
    maniGraph g;
    maniGraphInstance* instance = Walked(2.0f, &g);
    Steps(instance, 3);
    CHECK(NearF(maniPoseLocal(s_pose, 0).translation.x, 0.6), "the walk as it is");
    Pull(instance, 0);
    maniTransform x = Step(instance, 0.1f, 0);
    float x0 = Captured(instance, &g, 0, 0, 3);
    CHECK(Captured(instance, &g, 0, 0, 0) == 1.0f && Captured(instance, &g, 0, 0, 4) == 0.0f &&
              Captured(instance, &g, 0, 0, 5) == 0.5f,
          "moving away: no speed, the whole duration");
    CHECK(x.translation.x == maniGraphQuintic(x0, 0.0f, 0.5f, 0.1f) && NearF(x0, 0.6) &&
              x.translation.y == 1.0f,
          "eased from the last output");
    double before = (double)x.translation.x;
    for (int k = 1; k <= 3; ++k)
    {
        x = Step(instance, 0.1f, 0);
        double offset = (double)x.translation.x - 0.3 * k;
        CHECK(offset < before && offset > 0.0, "closing in");
        before = offset;
    }
    x = Step(instance, 0.1f, 0);
    CHECK(Run(x.translation.x, 0.4) && (Block(instance, &g, 3)[0] & maniGraphInertiaActive) == 0,
          "the run alone at the half second");

    // The walk backward, toward the run's x: its speed kept.
    instance = Walked(-1.0f, &g);
    Steps(instance, 3);
    Pull(instance, 0);
    x = Step(instance, 0.1f, 0);
    float v0 = Captured(instance, &g, 0, 0, 4);
    x0 = Captured(instance, &g, 0, 0, 3);
    CHECK(fabs((double)v0 + 1.0) < 1e-4 && Captured(instance, &g, 0, 0, 5) == 0.5f &&
              x.translation.x == maniGraphQuintic(x0, v0, 0.5f, 0.1f),
          "moving toward: its speed");
}

// A skipped evaluation leaves no speed; a second evaluation in one update
// is not motion.
static void TestHistory(void)
{
    maniGraph g;
    maniGraphInstance* instance = Walked(-1.0f, &g);
    Steps(instance, 1);
    Update(instance, 0.1f);
    Steps(instance, 1);
    Pull(instance, 0);
    Steps(instance, 1);
    CHECK(Captured(instance, &g, 0, 0, 4) == 0.0f && Captured(instance, &g, 0, 0, 3) > 0.0f,
          "a skipped evaluation: no speed");

    instance = Walked(-1.0f, &g);
    Steps(instance, 2);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "evaluated again");
    Pull(instance, 0);
    Steps(instance, 1);
    CHECK(fabs((double)Captured(instance, &g, 0, 0, 4) + 1.0) < 1e-4,
          "evaluated twice: the update before still the motion");
}

// The trigger's request and the transition's in one update: the least.
static void TestRequests(void)
{
    maniGraph g;
    maniGraphInstance* instance = Walked(2.0f, &g);
    Steps(instance, 2);
    Pull(instance, 0);
    Pull(instance, 1);
    maniTransform x = Step(instance, 0.1f, 0);
    CHECK(maniGraphInertiaGet(Block(instance, &g, 3), maniGraphInertiaDuration) == 0.2f &&
              Captured(instance, &g, 0, 0, 5) == 0.2f && instance->cells[1].value.b == 0 &&
              x.translation.x > 0.0f,
          "the least duration, the trigger consumed");
    x = Step(instance, 0.1f, 0);
    CHECK(Run(x.translation.x, 0.1), "done at 0.2: the run alone");

    // The trigger alone: an offset of one step's motion and its speed,
    // the output near the walk.
    instance = Walked(2.0f, &g);
    Steps(instance, 2);
    Pull(instance, 1);
    x = Step(instance, 0.1f, 0);
    CHECK(fabs((double)x.translation.x - 0.6) < 0.05 && x.translation.x != 0.6f &&
              NearF(Captured(instance, &g, 0, 0, 3), 0.2) &&
              fabs((double)Captured(instance, &g, 0, 0, 4) + 2.0) < 1e-4,
          "a request with no jump: the walk's own motion eased");
}

// The spine half turned about z (turn A) to the other way (turn B):
// eased by axis and angle about z, turn B alone at the end.
static void TestRotation(void)
{
    maniGraph g;
    maniGraphInstance* instance =
        Eased(Node(MANI_NO_PARAMETER, (maniGraphScalar){MANI_NO_PARAMETER, 0.0f}), 7, 1.0f, 8, &g);
    Steps(instance, 2);
    Pull(instance, 0);
    maniTransform x = Step(instance, 0.1f, 1);
    float angle = Captured(instance, &g, 1, 1, 3);
    CHECK(fabs((double)Captured(instance, &g, 1, 1, 2)) == 1.0 && angle > 0.0f &&
              Captured(instance, &g, 1, 1, 4) == 0.0f && Captured(instance, &g, 0, 0, 3) == 0.0f,
          "a turn about z captured, still");
    double half = 0.5 * (double)maniGraphQuintic(angle, 0.0f, 0.5f, 0.1f);
    double dot = (double)x.rotation.z * -0.9 + (double)x.rotation.w * 0.43588989;
    CHECK(fabs(fabs(dot) - cos(half)) < 1e-5 && x.rotation.x == 0.0f && x.rotation.y == 0.0f,
          "eased by the angle left");
    for (int k = 0; k < 4; ++k)
    {
        x = Step(instance, 0.1f, 1);
    }
    CHECK(NearF(x.rotation.z, -0.9) && NearF(x.rotation.w, 0.43588989), "turn B alone at the end");
}

// A seek above the node clears it: the input as it is.
static void TestSeek(void)
{
    static const uint32_t kSeek[1] = {1};
    static const uint32_t kNode[1] = {2};
    static const uint32_t kStates[2] = {3, 4};
    maniGraphTransitionSource t[1] = {Inertial(0.5f)};
    Parameters();
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeSeek, 1, kSeek, .seek = {1, {MANI_NO_PARAMETER, 0.0f}}},
        {maniGraphNodeInertialize, 1, kNode,
         .inertialize = {MANI_NO_PARAMETER, 0, {2, 0.0f}, 0.0f, 0.0f, 0.0f}},
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(0, 2.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    // The walk 0, the run 1, the machine 2, the node 3, the seek 4.
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 3, .nodeCount = 5},
        &g);
    Steps(instance, 3);
    Pull(instance, 0);
    Steps(instance, 1);
    CHECK((Block(instance, &g, 3)[0] & maniGraphInertiaActive) != 0, "easing");
    Pull(instance, 1);
    Steps(instance, 1);
    const uint32_t* block = Block(instance, &g, 3);
    CHECK((block[0] & maniGraphInertiaActive) == 0 &&
              maniGraphInertiaGet(block, maniGraphInertiaElapsed) == 0.1f,
          "sought: cleared, its clock begun again");
}

static _Alignas(32) uint32_t s_block[MANI_GRAPH_INERTIA_BLOCK(3, 0)];
static _Alignas(32) uint8_t s_inputMemory[4096];

// Bone 0's last two outputs (consecutive, 0.1 s apart) and the input,
// set by hand; one evaluation captures and applies 0.1 s in.
static void Captures(maniGraphTrs last, maniGraphTrs before, maniTransform input, maniPose* in)
{
    maniGraphInertia node = {3, maniGraphInertialization, 0.0f, 0.0f, 0.0f};
    maniGraphResetInertia(s_block, &node);
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniGraphTrs rest = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
        maniGraphTrsHold(s_block, maniGraphInertiaHistory(s_block, 3, true), b,
                         b == 0 ? last : rest);
        maniGraphTrsHold(s_block, maniGraphInertiaHistory(s_block, 3, false), b,
                         b == 0 ? before : rest);
        maniPoseSetLocal(in, b, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}});
    }
    s_block[maniGraphInertiaFlags] = maniGraphInertiaLastKept | maniGraphInertiaPrevKept;
    s_block[maniGraphInertiaSerial] = 6;
    s_block[maniGraphInertiaPrevSerial] = 4;
    s_block[maniGraphInertiaLastSerial] = 5;
    maniGraphInertiaSet(s_block, maniGraphInertiaStep, 0.1f);
    maniGraphInertiaSet(s_block, maniGraphInertiaLastStep, 0.1f);
    maniGraphInertiaSet(s_block, maniGraphInertiaPending, 0.5f);
    maniPoseSetLocal(in, 0, input);
    maniGraphInertiaEvaluate(s_block, &node, in, s_pose);
}

static float Word(uint32_t k)
{
    return maniGraphInertiaGet(s_block, CaptureAt(maniGraphInertialization, 0) + k);
}

// Each channel's speed toward the input from the last two outputs: the
// rotation's by its twist about the turn's axis, the scale's along its
// offset; a turn applied before the input's own rotation.
static void TestChannels(void)
{
    maniPose* in = NULL;
    CHECK(maniInitPose(&s_skeleton, s_inputMemory, sizeof s_inputMemory, &in) == mani_success,
          "a pose");
    maniGraphTrs last = {{0, 0, 0}, AboutZ(0.4f), {2.0f, 2.0f, 2.0f}};
    maniGraphTrs before = {{0, 0, 0}, AboutZ(0.5f), {2.1f, 2.1f, 2.1f}};
    Captures(last, before, (maniTransform){{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}}, in);
    CHECK(NearF(Word(8), 1.0) && NearF(Word(9), 0.4) && fabs((double)Word(10) + 1.0) < 1e-3 &&
              NearF(Word(11), 0.5),
          "a turn about z, closing at 1 per second");
    double grown = (double)maniGraphQuintic(Word(15), Word(16), Word(17), 0.1f) / 1.7320508;
    CHECK(NearF(Word(15), 1.7320508) && fabs((double)Word(16) + 1.7320508) < 1e-3 &&
              fabs((double)maniPoseLocal(s_pose, 0).scale.x - 1.0 - grown) < 1e-5,
          "a scale closing along its offset, eased");
    maniQuat x = maniQuatFromAxisAngle((maniVec3){1.0f, 0.0f, 0.0f}, 0.5f);
    Captures(last, before, (maniTransform){{0, 0, 0}, x, {1, 1, 1}}, in);
    maniVec3 axis = {Word(6), Word(7), Word(8)};
    maniQuat back = maniQuatMul(maniQuatFromAxisAngle(axis, Word(9)), x);
    float eased = maniGraphQuintic(Word(9), Word(10), Word(11), 0.1f);
    maniQuat want = maniQuatMul(maniQuatFromAxisAngle(axis, eased), x);
    maniTransform out = maniPoseLocal(s_pose, 0);
    CHECK(fabs(fabs((double)maniQuatDot(back, last.r)) - 1.0) < 1e-5 &&
              fabs(fabs((double)maniQuatDot(want, out.rotation)) - 1.0) < 1e-6,
          "the turn taken from the input's rotation and applied before it");
}

// The walk backward at 3 per second toward the run: the duration cut to
// -5 x0 / v0, so the curve never passes the input; the input from then
// on, the node still active.
static void TestCut(void)
{
    maniGraph g;
    maniGraphInstance* instance = Walked(-3.0f, &g);
    Steps(instance, 3);
    Pull(instance, 0);
    (void)Step(instance, 0.1f, 0);
    float x0 = Captured(instance, &g, 0, 0, 3);
    float v0 = Captured(instance, &g, 0, 0, 4);
    CHECK(NearF(x0, 0.1) && fabs((double)v0 + 3.0) < 1e-3 &&
              Captured(instance, &g, 0, 0, 5) == -5.0f * x0 / v0,
          "the duration cut");
    maniTransform x = Step(instance, 0.1f, 0);
    CHECK(Run(x.translation.x, 0.1) && (Block(instance, &g, 3)[0] & maniGraphInertiaActive) != 0,
          "the input once the cut is past");
}

// A request before the node has any output: nothing to ease from.
static void TestFirst(void)
{
    maniGraph g;
    maniGraphInstance* instance = Walked(2.0f, &g);
    Pull(instance, 0);
    maniTransform x = Step(instance, 0.1f, 0);
    CHECK(x.translation.x == 0.0f && (Block(instance, &g, 3)[0] & maniGraphInertiaActive) == 0,
          "a request with no output kept dropped");
}

// A time scale of 2 above the node: its clock runs at twice the update.
static void TestScaled(void)
{
    static const uint32_t kScaled[1] = {1};
    static const uint32_t kNode[1] = {2};
    static const uint32_t kStates[2] = {3, 4};
    maniGraphTransitionSource t[1] = {Inertial(0.5f)};
    Parameters();
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeTimeScale, 1, kScaled, .timeScale = {LITERAL(2.0f)}},
        {maniGraphNodeInertialize, 1, kNode,
         .inertialize = {MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.5f}, 0.0f, 0.0f, 0.0f}},
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(3, 1.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    // The walk 0, the run 1, the machine 2, the node 3, the time scale 4.
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 3, .nodeCount = 5},
        &g);
    Steps(instance, 1);
    CHECK(maniGraphInertiaGet(Block(instance, &g, 3), maniGraphInertiaStep) == 0.2f,
          "its step the scaled one");
}

// Two inertial transitions chained in one update: the least duration,
// though the later asks more.
static void TestChain(void)
{
    static const uint32_t kStates[3] = {2, 3, 4};
    maniGraphTransitionSource t[2] = {Inertial(0.3f), Inertial(0.4f)};
    t[1].from = 1;
    t[1].to = 2;
    t[1].conditions = NULL;
    t[1].conditionCount = 0;
    Parameters();
    maniGraphNodeSource nodes[5] = {
        Node(MANI_NO_PARAMETER, (maniGraphScalar){MANI_NO_PARAMETER, 0.0f}),
        {maniGraphNodeMachine, 3, kStates, .machine = {t, 2, 0, 0, 0, NULL}},
        ClipNode(3, 2.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat),
        ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    // The walk 0, the run 1, the third 2, the machine 3, the node 4.
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 3, .nodeCount = 5},
        &g);
    Steps(instance, 2);
    Pull(instance, 0);
    Steps(instance, 1);
    CHECK(maniGraphInertiaGet(Block(instance, &g, 4), maniGraphInertiaDuration) == 0.3f,
          "the chain's least duration");
}

static _Alignas(8) uint8_t s_saved[2][8192];
static _Alignas(8) uint8_t s_other[16384];

// Saved mid-ease and restored: both go on alike; a saved block refused
// at its word.
static void TestSaved(void)
{
    maniGraph g;
    maniGraphInstance* instance = Walked(-1.0f, &g);
    Steps(instance, 2);
    Pull(instance, 0);
    Steps(instance, 1);
    size_t size = maniGraphStateSize(instance->binding);
    maniGraphInstance* other = NULL;
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniRestoreGraphInstance(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    maniTransform a = Step(instance, 0.1f, 0);
    maniTransform b = Step(other, 0.1f, 0);
    CHECK(memcmp(&a, &b, sizeof a) == 0 && a.translation.x != 0.0f, "eased alike");
    // The node's block is the last: after the machine's.
    size_t block = size - 4 * (size_t)MANI_GRAPH_INERTIA_BLOCK(3, 0);
    uint32_t capture = CaptureAt(maniGraphInertialization, 2) + 6;
    const uint32_t kBad[7][2] = {{maniGraphInertiaFlags, 16},
                                 {maniGraphInertiaStep, 0x7F800000u},
                                 {maniGraphInertiaElapsed, 0xBF800000u},
                                 {maniGraphInertiaPending, 0x7FC00000u},
                                 {9, 1},
                                 {MANI_GRAPH_INERTIA_HEADER + 13, 0xFF800000u},
                                 {capture, 0x40000000u}};
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success,
          "saved again");
    for (int k = 0; k < 7; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kBad[k][0], kBad[k][1]);
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kBad[k][0],
              "a saved block refused at its word");
    }
}

static maniBuildResult Built(const maniGraphNodeSource* nodes, uint32_t count)
{
    Parameters();
    return BuildOnly(nodes, count, s_parameters, 3);
}

// No node above an inertial transition, or a path around it; the node's
// trigger and duration.
static void TestRefusals(void)
{
    static const uint32_t kStates[2] = {1, 2};
    maniGraphTransitionSource t[2] = {Inertial(0.5f), Inertial(0.5f)};
    t[0].fade = maniGraphFadeLinear;
    t[1].from = 1;
    t[1].to = 0;
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 2, 0, 0, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    CHECK(Refused(Built(nodes, 3), maniBuildReasonTransition, 1), "no node above");

    // A blend of the node and the machine: the machine reached around it.
    static const uint32_t kBoth[2] = {1, 2};
    static const uint32_t kNode[1] = {2};
    static const uint32_t kMachine[2] = {3, 4};
    static const uint32_t kUnder[2] = {2, 3};
    static const maniGraphScalar kHalves[2] = {{MANI_NO_PARAMETER, 0.5f},
                                               {MANI_NO_PARAMETER, 0.5f}};
    maniGraphNodeSource around[5] = {
        {maniGraphNodeBlend, 2, kBoth, .blend = {kHalves, 0, 0}},
        {maniGraphNodeInertialize, 1, kNode,
         .inertialize = {MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.2f}, 0.0f, 0.0f, 0.0f}},
        {maniGraphNodeMachine, 2, kMachine, .machine = {t, 2, 0, 0, 0, NULL}},
        nodes[1],
        nodes[2]};
    maniBuildResult r = Built(around, 5);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonTransition && r.item == 2 &&
              r.other == 1,
          "a path around the node");

    maniGraphNodeSource under[4] = {
        Node(MANI_NO_PARAMETER, (maniGraphScalar){MANI_NO_PARAMETER, 0.2f}),
        {maniGraphNodeMachine, 2, kUnder, .machine = {t, 2, 0, 0, 0, NULL}},
        nodes[1],
        nodes[2]};
    CHECK(Built(under, 4).result == mani_success, "beneath the node");
    under[0].inertialize.trigger = 2;
    CHECK(Refused(Built(under, 4), maniBuildReasonNode, 0), "a trigger of another type");
    under[0].inertialize =
        (maniGraphInertializeNodeSource){0, 0, {MANI_NO_PARAMETER, -1.0f}, 0.0f, 0.0f, 0.0f};
    CHECK(Refused(Built(under, 4), maniBuildReasonScalar, 0), "a negative duration");
    under[0] = Dead();
    under[0].inertialize.trigger = MANI_NO_PARAMETER;
    CHECK(Built(under, 4).result == mani_success, "dead blending");
    const float kLives[4][3] = {
        {-0.1f, 0.1f, 1.0f}, {0.3f, 0.0f, 1.0f}, {0.3f, 0.5f, 0.4f}, {0.3f, 0.1f, HUGE_VALF}};
    for (int k = 0; k < 4; ++k)
    {
        under[0].inertialize.halfLifeScale = kLives[k][0];
        under[0].inertialize.halfLifeMin = kLives[k][1];
        under[0].inertialize.halfLifeMax = kLives[k][2];
        CHECK(Refused(Built(under, 4), maniBuildReasonNode, 0), "half-lives refused");
    }
    under[0].inertialize.method = 2;
    CHECK(Refused(Built(under, 4), maniBuildReasonNode, 0), "an unknown method");
    under[0].inertialize =
        Node(MANI_NO_PARAMETER, (maniGraphScalar){MANI_NO_PARAMETER, 0.2f}).inertialize;
    under[0].inertialize.halfLifeMax = 1.0f;
    CHECK(Refused(Built(under, 4), maniBuildReasonNode, 0), "inertialization's half-lives not 0");
}

// The node of Walked: its words checked on open (trigger 0, method 1, the
// duration 2 and 3, the half-lives 4 to 6, the block 7).
static void TestOpenRefusals(void)
{
    maniGraph g;
    (void)Walked(1.0f, &g);
    uint64_t at = 0;
    CHECK(Crafted(&g, 3, 0, MANI_NO_PARAMETER, &at).result == mani_success, "no trigger");
    CHECK(RefusedAtWordOf(&g, 3, 7, DataWord(&g, 3, 7) + 1, 7), "a block past its place");
    const uint32_t kBad[8][3] = {{0, 2, 0}, {0, 3, 0}, {1, 2, 1},          {2, 0, 2},
                                 {2, 5, 2}, {3, 1, 2}, {4, Bits(0.3f), 4}, {7, 1, 7}};
    for (int k = 0; k < 8; ++k)
    {
        CHECK(RefusedAtWordOf(&g, 3, kBad[k][0], kBad[k][1], kBad[k][2]), "a node's word refused");
    }
}

static uint8_t s_built[16384];
static uint8_t s_read[16384];
static char s_text[16384];
static size_t s_builtSize;
static size_t s_textSize;

static bool RoundTrip(const maniGraphSource* source)
{
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
              maniWriteGraphJson(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &s_textSize) == mani_success &&
              maniBuildGraphJson(builder, s_text, s_textSize, &s_skeleton, kSlots, CLIPS).result ==
                  mani_success &&
              maniCopyBuilt(builder, s_read, sizeof s_read, &readSize) == mani_success;
    maniDestroyBuilder(builder);
    s_text[ok ? s_textSize : 0] = 0;
    return ok && readSize == s_builtSize && memcmp(s_read, s_built, s_builtSize) == 0;
}

static maniBuildResult Read(const char* text)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraphJson(builder, text, strlen(text), &s_skeleton, kSlots, CLIPS);
    maniDestroyBuilder(builder);
    return r;
}

static void TestJson(void)
{
    static const uint32_t kNode[1] = {1};
    static const uint32_t kStates[2] = {2, 3};
    maniGraphTransitionSource t[1] = {{0, 1, 0, maniGraphInterruptionNone, maniGraphEntryRestart,
                                       maniGraphFadeInertial, 0.5f, 0.0f, kGo, 1, 0}};
    Parameters();
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeInertialize, 1, kNode, .inertialize = {1, 0, {2, 0.0f}, 0.0f, 0.0f, 0.0f}},
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    maniGraphSource source = {&s_skeleton, s_parameters, kSlots, NULL, nodes, 3, CLIPS, 0, 4, 0, 0};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "{\"kind\": \"inertialize\", \"inputs\": [1], \"trigger\": \"push\", "
                         "\"duration\": {\"parameter\": \"d\"}}") != NULL &&
              strstr(s_text, "\"fade\": \"inertial\"") != NULL,
          "the form");
    nodes[0].inertialize = (maniGraphInertializeNodeSource){
        MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.25f}, 0.0f, 0.0f, 0.0f};
    CHECK(RoundTrip(&source) && strstr(s_text, "\"inputs\": [1], \"duration\": 0.25}") != NULL,
          "no trigger");
    nodes[0] = Dead();
    nodes[0].inertialize.halfLifeMax = 0.75f;
    CHECK(RoundTrip(&source) &&
              strstr(s_text, "\"trigger\": \"push\", \"method\": \"deadBlending\", \"duration\": "
                             "{\"parameter\": \"d\"}, \"halfLifeScale\": 0.3, \"halfLifeMin\": "
                             "0.1, \"halfLifeMax\": 0.75}") != NULL,
          "dead blending");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"push\", \"type\": \"trigger\"}], \"clips\": [\"walk\"], \"nodes\": [{\"kind\": "            \
    "\"inertialize\", \"inputs\": [1]"
#define TAIL "}, {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}"

static void TestJsonForms(void)
{
    CHECK(Read(HEAD TAIL).result == mani_success, "members left out");
    CHECK(Read(HEAD ", \"method\": \"deadBlending\"" TAIL).result == mani_success,
          "dead blending's half-lives by default");
    maniBuildResult lives = Read(HEAD ", \"halfLifeMin\": 0.1" TAIL);
    CHECK(lives.result == mani_errorInvalid && lives.reason == maniBuildReasonNode,
          "inertialization's half-lives not 0");
    static const struct
    {
        const char* text;
        uint32_t form;
        const char* marker;
    } kRefusals[4] = {{HEAD ", \"trigger\": \"pull\"" TAIL, maniGraphFormName, "pull"},
                      {HEAD ", \"method\": \"springs\"" TAIL, maniGraphFormValue, "springs"},
                      {HEAD ", \"duration\": \"long\"" TAIL, maniGraphFormValue, "long"},
                      {HEAD ", \"time\": 1" TAIL, maniGraphFormMember, "time"}};
    for (int k = 0; k < 4; ++k)
    {
        maniBuildResult r = Read(kRefusals[k].text);
        const char* at = strstr(kRefusals[k].text, kRefusals[k].marker);
        bool ok = r.result == mani_errorInvalid && r.reason == maniBuildReasonForm &&
                  r.other == kRefusals[k].form && at != NULL &&
                  r.item == (uint32_t)(at - kRefusals[k].text);
        if (!ok)
        {
            printf("  form %d: reason %u other %u at %u, want %u at %d\n", k, r.reason, r.other,
                   r.item, kRefusals[k].form, at != NULL ? (int)(at - kRefusals[k].text) : -1);
        }
        CHECK(ok, "a form refused at its bytes");
    }
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
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success,
          "a pose");
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestQuintic();
    TestTransition();
    TestHistory();
    TestRequests();
    TestRotation();
    TestSeek();
    TestChannels();
    TestCut();
    TestFirst();
    TestScaled();
    TestChain();
    TestSaved();
    TestRefusals();
    TestOpenRefusals();
    TestJson();
    TestJsonForms();
    return s_failures == 0 ? 0 : 1;
}
