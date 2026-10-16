// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inertialization (mani-0023): the quintic against Bollo's published
// form, its ends and its overshoot clamp; an inertial transition eased
// out from the last output, the speed clamped when moving away and kept
// when moving toward, the input exact once the time is up; no speed after
// a skipped evaluation; a second evaluation in one update not taken as
// motion; the least of one update's requests; the node's trigger; a
// rotation eased by its axis and angle; a seek above clearing it; saved
// and restored mid-ease; the builder's refusals (no node above, a path
// around it), opening's checks and the saved block's; the JSON form,
// round trip and refusals. Run as test_graph_inertia --seed <graph file>
// <json file> it writes the round trip's file and text, seeds of the
// fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_inertia.h"
#include "graph_inertia_apply.h"
#include "graph_space_fixture.h"

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

// Parameters: go and push (triggers), d (float).
static maniGraphParameterSource s_parameters[3];

static void Parameters(void)
{
    s_parameters[0] = Parameter("go", maniGraphParameterTrigger, Int(0));
    s_parameters[1] = Parameter("push", maniGraphParameterTrigger, Int(0));
    s_parameters[2] = Parameter("d", maniGraphParameterFloat, Float(0.2f));
}

static const maniGraphConditionSource kGo[1] = {{maniGraphConditionTrigger, 0, 0, 0, {{0}}, NULL}};

static maniGraphTransitionSource Inertial(float duration)
{
    return (maniGraphTransitionSource){0,
                                       1,
                                       0,
                                       maniGraphInterruptionNone,
                                       maniGraphEntryRestart,
                                       maniGraphFadeInertial,
                                       duration,
                                       0.0f,
                                       kGo,
                                       1,
                                       0};
}

static maniGraphNodeSource Node(uint32_t trigger, maniGraphScalar duration)
{
    static const uint32_t kNode[1] = {1};
    return (maniGraphNodeSource){maniGraphNodeInertialize, 1, kNode,
                                 .inertialize = {trigger, 0, duration}};
}

// An inertialize node (its trigger and duration given) over a machine of
// clips from and to (repeating, from at speed), go taking it inertially
// over half a second: from 0, to 1, the machine 2, the node 3.
static maniGraphInstance* Eased(maniGraphNodeSource node, uint32_t from, float speed, uint32_t to,
                                maniGraph* g)
{
    static const uint32_t kStates[2] = {2, 3};
    static maniGraphTransitionSource t[1];
    t[0] = Inertial(0.5f);
    Parameters();
    maniGraphNodeSource nodes[4] = {
        node,
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(from, speed, maniGraphLoopRepeat),
        ClipNode(to, 1.0f, maniGraphLoopRepeat)};
    return Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 3, .nodeCount = 4},
        g);
}

// The walk (at speed) to the run, the node's trigger push and duration d.
static maniGraphInstance* Walked(float speed, maniGraph* g)
{
    return Eased(Node(1, (maniGraphScalar){2, 0.0f}), 3, speed, 1, g);
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[4096];
static maniPose* s_pose;

// One update of delta and its evaluation; the output's bone b.
static maniTransform Step(maniGraphInstance* instance, float delta, uint32_t b)
{
    Update(instance, delta);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_pose) == mani_success,
          "evaluated");
    return maniPoseLocal(s_pose, b);
}

static void Steps(maniGraphInstance* instance, int count)
{
    for (int k = 0; k < count; ++k)
    {
        (void)Step(instance, 0.1f, 0);
    }
}

static uint32_t* Block(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    uint32_t data = maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    return (uint32_t*)((uint8_t*)instance + instance->machinesAt) +
           maniLoadU32(g->data + 4 * (size_t)(data + 3));
}

// Bone b's channel c (translation 0, rotation 1, scale 2) as captured:
// direction, magnitude, speed, duration.
static float Captured(maniGraphInstance* instance, const maniGraph* g, uint32_t b, uint32_t c,
                      uint32_t k)
{
    return maniGraphInertiaGet(Block(instance, g, 3),
                               maniGraphInertiaBone(b) + MANI_GRAPH_INERTIA_CAPTURE + 6 * c + k);
}

static void Pull(maniGraphInstance* instance, uint32_t trigger)
{
    Set(instance, trigger, maniGraphParameterTrigger, Int(1));
}

// The run's x at its time t: 3 t.
static bool Run(float x, double t)
{
    return fabs((double)x - 3.0 * t) <= 1e-5;
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
        {maniGraphNodeInertialize, 1, kNode, .inertialize = {MANI_NO_PARAMETER, 0, {2, 0.0f}}},
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
    size_t block = size - 4 * (size_t)MANI_GRAPH_INERTIA_BLOCK(3);
    uint32_t capture = maniGraphInertiaBone(2) + MANI_GRAPH_INERTIA_CAPTURE + 6;
    const uint32_t kBad[7][2] = {{maniGraphInertiaFlags, 8},
                                 {maniGraphInertiaStep, 0x7F800000u},
                                 {maniGraphInertiaElapsed, 0xBF800000u},
                                 {maniGraphInertiaPending, 0x7FC00000u},
                                 {9, 1},
                                 {maniGraphInertiaBone(1) + 4, 0xFF800000u},
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
         .inertialize = {MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.2f}}},
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
    under[0].inertialize = (maniGraphInertializeNodeSource){0, 0, {MANI_NO_PARAMETER, -1.0f}};
    CHECK(Refused(Built(under, 4), maniBuildReasonScalar, 0), "a negative duration");
}

// The node of Walked: its words checked on open (trigger 0, the duration
// 1 and 2, the block 3).
static void TestOpenRefusals(void)
{
    maniGraph g;
    (void)Walked(1.0f, &g);
    uint64_t at = 0;
    CHECK(Crafted(&g, 3, 0, MANI_NO_PARAMETER, &at).result == mani_success, "no trigger");
    const uint32_t kBad[6][3] = {{0, 2, 0}, {0, 3, 0}, {1, 0, 1}, {1, 3, 1}, {2, 1, 1}, {3, 1, 3}};
    for (int k = 0; k < 6; ++k)
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
        {maniGraphNodeInertialize, 1, kNode, .inertialize = {1, 0, {2, 0.0f}}},
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    maniGraphSource source = {&s_skeleton, s_parameters, kSlots, NULL, nodes, 3, CLIPS, 0, 4, 0, 0};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "{\"kind\": \"inertialize\", \"inputs\": [1], \"trigger\": \"push\", "
                         "\"duration\": {\"parameter\": \"d\"}}") != NULL &&
              strstr(s_text, "\"fade\": \"inertial\"") != NULL,
          "the form");
    nodes[0].inertialize =
        (maniGraphInertializeNodeSource){MANI_NO_PARAMETER, 0, {MANI_NO_PARAMETER, 0.25f}};
    CHECK(RoundTrip(&source) && strstr(s_text, "\"inputs\": [1], \"duration\": 0.25}") != NULL,
          "no trigger");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"push\", \"type\": \"trigger\"}], \"clips\": [\"walk\"], \"nodes\": [{\"kind\": "            \
    "\"inertialize\", \"inputs\": [1]"
#define TAIL "}, {\"kind\": \"clip\", \"clip\": \"walk\"}], \"output\": 0}"

static void TestJsonForms(void)
{
    CHECK(Read(HEAD TAIL).result == mani_success, "members left out");
    static const struct
    {
        const char* text;
        uint32_t form;
        const char* marker;
    } kRefusals[3] = {{HEAD ", \"trigger\": \"pull\"" TAIL, maniGraphFormName, "pull"},
                      {HEAD ", \"duration\": \"long\"" TAIL, maniGraphFormValue, "long"},
                      {HEAD ", \"time\": 1" TAIL, maniGraphFormMember, "time"}};
    for (int k = 0; k < 3; ++k)
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
    TestSaved();
    TestRefusals();
    TestOpenRefusals();
    TestJson();
    TestJsonForms();
    return s_failures == 0 ? 0 : 1;
}
