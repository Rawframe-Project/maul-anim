// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// State machines: an entry state; a fade's weights, linear
// and smoothstep, from the update after its transition fires; an
// interrupted fade going on from its weights; interruption none, by a
// lower priority value and by any; the lower priority value winning;
// cooldowns; chains of instant transitions to the limit; time in state;
// triggers consumed; bool, int and float comparisons; restart and keep
// entries, and a state still weighted keeping its time; a time scale
// above the machine; a nested machine started over when entered; saved
// and restored mid-fade; evaluation bit for bit with the blending
// functions; the builder's refusals and opening's checks; a machine in a
// subgraph; the JSON form, round trip, members left out and refusals.
// Run as test_graph_machine --seed <graph file> <json file> it writes the
// round trip's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_space_fixture.h"

#include "maul-anim/pose.h"

#include <stdio.h>

// Parameters: speed (float), jump (trigger), crouch (bool), gait (int).
static maniGraphParameterSource s_parameters[4];

static void Parameters(void)
{
    s_parameters[0] = Parameter("speed", maniGraphParameterFloat, Float(0.0f));
    s_parameters[1] = Parameter("jump", maniGraphParameterTrigger, Int(0));
    s_parameters[2] = Parameter("crouch", maniGraphParameterBool, Int(0));
    s_parameters[3] = Parameter("gait", maniGraphParameterInt, Int(0));
}

// A machine over the walk 0, the run 1 and the short step 2 (states and
// node indices alike, the machine node 3), its transitions given.
static maniGraphInstance* Machine(const maniGraphTransitionSource* transitions, uint32_t count,
                                  uint32_t chain, maniGraph* g)
{
    static const uint32_t kStates[3] = {1, 2, 3};
    Parameters();
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 3, kStates, .machine = {transitions, count, 0, chain, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip)};
    return Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 4, .nodeCount = 4},
        g);
}

static maniGraphConditionSource Greater(uint32_t parameter, float value)
{
    return (maniGraphConditionSource){
        maniGraphConditionParameter, parameter, maniGraphCompareGreater, 0, Float(value), NULL};
}

static maniGraphConditionSource Trigger(uint32_t parameter)
{
    return (maniGraphConditionSource){maniGraphConditionTrigger, parameter, 0, 0, Int(0), NULL};
}

static maniGraphTransitionSource Go(uint32_t from, uint32_t to, uint32_t priority, float duration,
                                    const maniGraphConditionSource* conditions, uint32_t count)
{
    return (maniGraphTransitionSource){from,
                                       to,
                                       priority,
                                       maniGraphInterruptionNone,
                                       maniGraphEntryRestart,
                                       maniGraphFadeLinear,
                                       duration,
                                       0.0f,
                                       conditions,
                                       count,
                                       0};
}

static bool Weights3(maniGraphInstance* instance, const maniGraph* g, float a, float b, float c)
{
    const float want[3] = {a, b, c};
    return Weights(instance, g, want, 3);
}

// Faster than 1: the walk to the run over half a second.
static void TestFade(void)
{
    static const maniGraphConditionSource kFast[1] = {
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, {{1.0f}}, NULL}};
    maniGraphTransitionSource run[1] = {Go(0, 1, 0, 0.5f, kFast, 1)};
    maniGraph g;
    maniGraphInstance* instance = Machine(run, 1, 0, &g);
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0) && Near(State(instance, &g, 0)->time, 0.25),
          "the entry state");
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0) && Near(State(instance, &g, 1)->time, 0.0),
          "fired: the run started over at no weight yet");
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 0.5f, 0.5f, 0) && Near(State(instance, &g, 1)->time, 0.25),
          "halfway");
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "done");
    float walk = (float)State(instance, &g, 0)->time;
    Update(instance, 0.25f);
    CHECK(NearF((float)State(instance, &g, 0)->time, (double)walk), "a state at no weight frozen");
    run[0].fade = maniGraphFadeSmoothstep;
    run[0].duration = 1.0f;
    instance = Machine(run, 1, 0, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.25f);
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 0.84375f, 0.15625f, 0), "smoothstep at a quarter");
}

// The walk to the run over a second, interrupted halfway by a jump to the
// short step from any state over a second.
static void TestInterruption(void)
{
    static const maniGraphConditionSource kFast[1] = {
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, {{1.0f}}, NULL}};
    static const maniGraphConditionSource kJump[1] = {
        {maniGraphConditionTrigger, 1, 0, 0, {{0}}, NULL}};
    maniGraphTransitionSource t[2] = {Go(0, 1, 5, 1.0f, kFast, 1),
                                      Go(MANI_GRAPH_ANY_STATE, 2, 1, 1.0f, kJump, 1)};
    const maniGraphInterruption kRules[3] = {maniGraphInterruptionAny, maniGraphInterruptionHigher,
                                             maniGraphInterruptionNone};
    for (int k = 0; k < 3; ++k)
    {
        t[0].interruption = kRules[k];
        maniGraph g;
        maniGraphInstance* instance = Machine(t, 2, 0, &g);
        Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
        Update(instance, 0.5f);
        Update(instance, 0.5f);
        Set(instance, 1, maniGraphParameterTrigger, Int(1));
        // The fade first goes on to three quarters, then the jump may fire
        // from those weights.
        Update(instance, 0.25f);
        maniGraphValue jump = {{0}};
        CHECK(maniGetGraphParameter(instance, 1, &jump) == mani_success &&
                  Weights3(instance, &g, 0.25f, 0.75f, 0) && jump.b == (k < 2 ? 0u : 1u),
              k < 2 ? "interrupted: the trigger consumed" : "not interrupted: the trigger kept");
        Update(instance, 0.25f);
        if (k < 2)
        {
            CHECK(Weights3(instance, &g, 0.1875f, 0.5625f, 0.25f),
                  "on from the weights at the interruption");
        }
        else
        {
            CHECK(Weights3(instance, &g, 0, 1, 0), "the fade done, then the jump fired");
        }
    }
    // By a lower priority value only: the jump's 9 is higher than 5.
    t[0].interruption = maniGraphInterruptionHigher;
    t[1].priority = 9;
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 2, 0, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.5f);
    Update(instance, 0.5f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.25f);
    maniGraphValue jump = {{0}};
    CHECK(maniGetGraphParameter(instance, 1, &jump) == mani_success &&
              Weights3(instance, &g, 0.25f, 0.75f, 0) && jump.b == 1,
          "a higher value waits");
}

// Two transitions from the walk both holding: the lower value fires,
// though listed second.
static void TestPriority(void)
{
    maniGraphTransitionSource t[2] = {Go(0, 1, 7, 0.0f, NULL, 0), Go(0, 2, 3, 0.0f, NULL, 0)};
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 2, 1, &g);
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 0, 0, 1), "the lower priority value");
}

// Instant transitions round the three states: one an update at limit 1,
// four at the default.
static void TestChains(void)
{
    maniGraphTransitionSource t[3] = {Go(0, 1, 0, 0.0f, NULL, 0), Go(1, 2, 1, 0.0f, NULL, 0),
                                      Go(2, 0, 2, 0.0f, NULL, 0)};
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 3, 1, &g);
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "limit 1: one");
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 0, 0, 1), "limit 1: the next");
    instance = Machine(t, 3, 0, &g);
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "the default 4: round once and on");
    t[1] = Go(1, 0, 1, 0.0f, NULL, 0);
    instance = Machine(t, 2, 0, &g);
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "the default 4: there and back twice");
}

// An any-state transition never fires into the current state: the state
// keeps its time and leaves when its half second is up.
static void TestAnyToCurrent(void)
{
    static const maniGraphConditionSource kHalf[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.5f}}, NULL}};
    maniGraphTransitionSource t[2] = {Go(MANI_GRAPH_ANY_STATE, 2, 0, 0.0f, NULL, 0),
                                      Go(2, 0, 1, 0.0f, kHalf, 1)};
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 2, 1, &g);
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 0, 0, 1), "entered from any state");
    Update(instance, 0.25f);
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "not entered again: its half second up");
}

// The walk to the run and straight back; the walk to the run again only
// once a second has passed since it fired.
static void TestCooldown(void)
{
    maniGraphTransitionSource t[2] = {Go(0, 1, 0, 0.0f, NULL, 0), Go(1, 0, 1, 0.0f, NULL, 0)};
    t[0].cooldown = 1.0f;
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 2, 1, &g);
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "fired");
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "back");
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "cooling down");
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "still cooling down");
    Update(instance, 0.5f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "a second on");
}

// Half a second in the walk, then the run; comparisons of each type.
static void TestConditions(void)
{
    static const maniGraphConditionSource kHalf[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.5f}}, NULL}};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.0f, kHalf, 1)};
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 1, 0, &g);
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "a quarter in: not yet");
    Update(instance, 0.25f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "half a second in");
    const maniGraphConditionSource kCompares[4] = {
        {maniGraphConditionParameter, 2, maniGraphCompareEqual, 0, Int(1), NULL},
        {maniGraphConditionParameter, 3, maniGraphCompareAtLeast, 0, Int(-2), NULL},
        {maniGraphConditionParameter, 3, maniGraphCompareNotEqual, 0, Int(3), NULL},
        {maniGraphConditionParameter, 0, maniGraphCompareAtMost, 0, Float(0.5f), NULL}};
    t[0] = Go(0, 1, 0, 0.0f, kCompares, 4);
    instance = Machine(t, 1, 0, &g);
    Set(instance, 3, maniGraphParameterInt, Int(-2));
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "crouch false");
    Set(instance, 2, maniGraphParameterBool, Int(1));
    Set(instance, 3, maniGraphParameterInt, Int(3));
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "gait 3 not wanted");
    Set(instance, 3, maniGraphParameterInt, Int(-2));
    Set(instance, 0, maniGraphParameterFloat, Float(0.75f));
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 1, 0, 0), "speed past 0.5");
    Set(instance, 0, maniGraphParameterFloat, Float(0.5f));
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "all four hold");
    // At the value: less and greater do not hold; crouch not true.
    const maniGraphConditionSource kEdges[3] = {
        {maniGraphConditionParameter, 0, maniGraphCompareLess, 0, Float(1.0f), NULL},
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, Float(1.0f), NULL},
        {maniGraphConditionParameter, 2, maniGraphCompareNotEqual, 0, Int(1), NULL}};
    for (int k = 0; k < 3; ++k)
    {
        t[0] = Go(0, 1, 0, 0.0f, &kEdges[k], 1);
        instance = Machine(t, 1, 0, &g);
        Set(instance, 0, maniGraphParameterFloat, Float(1.0f));
        Set(instance, 2, maniGraphParameterBool, Int(1));
        Update(instance, 0.1f);
        CHECK(Weights3(instance, &g, 1, 0, 0), "not at the value");
    }
    Set(instance, 2, maniGraphParameterBool, Int(0));
    Update(instance, 0.1f);
    CHECK(Weights3(instance, &g, 0, 1, 0), "crouch false: not equal holds");
}

// The walk to the run after a quarter in it, back after a half in the
// run, then to the run again: restarted, the run starts at 0 each time;
// kept, where it was left. A state entered while it still fades out keeps
// its time even under restart.
static void TestEntries(void)
{
    static const maniGraphConditionSource kQuarter[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.25f}}, NULL}};
    static const maniGraphConditionSource kHalf[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.5f}}, NULL}};
    maniGraphTransitionSource t[2] = {Go(0, 1, 0, 0.0f, kQuarter, 1), Go(1, 0, 1, 0.0f, kHalf, 1)};
    for (int keep = 0; keep < 2; ++keep)
    {
        t[0].entry = keep ? maniGraphEntryKeep : maniGraphEntryRestart;
        maniGraph g;
        maniGraphInstance* instance = Machine(t, 2, 1, &g);
        // Into the run at the 1st update (played there when kept, from 0
        // the next when restarted), left at the 3rd, entered again at the
        // 4th: restarted, it is at 0 again; kept, it played a quarter on
        // from the half it had reached.
        for (int k = 0; k < 4; ++k)
        {
            Update(instance, 0.25f);
        }
        double want = keep ? 0.75 : 0.0;
        CHECK(Weights3(instance, &g, 0, 1, 0) && Near(State(instance, &g, 1)->time, want),
              keep ? "kept" : "restarted");
    }
    maniGraphTransitionSource slow[2] = {Go(0, 1, 0, 1.0f, kQuarter, 1),
                                         Go(1, 0, 1, 1.0f, kQuarter, 1)};
    slow[1].interruption = maniGraphInterruptionAny;
    slow[0].interruption = maniGraphInterruptionAny;
    maniGraph g;
    maniGraphInstance* instance = Machine(slow, 2, 1, &g);
    for (int k = 0; k < 3; ++k)
    {
        Update(instance, 0.25f);
    }
    // The walk, still weighted when entered back at the 2nd update, played
    // on throughout.
    CHECK(Near(State(instance, &g, 0)->time, 0.75), "a weighted state kept its time");
}

// A time scale of 2 above the machine: the half-second fade done in two
// updates of an eighth.
static void TestTimeScale(void)
{
    static const uint32_t kStates[2] = {2, 3};
    static const uint32_t kScaled[1] = {1};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.5f, NULL, 0)};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeTimeScale, 1, kScaled, .timeScale = {LITERAL(2.0f)}},
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 4}, &g);
    Update(instance, 0.125f);
    Update(instance, 0.125f);
    const float kHalf[2] = {0.5f, 0.5f};
    CHECK(Weights(instance, &g, kHalf, 2), "a quarter of machine time");
}

// A machine whose second state is another machine (the walk, then the
// run after half a second): entered again from its start, the inner one
// begins at its entry, not on the run it had reached.
static void TestNested(void)
{
    static const maniGraphConditionSource kQuarter[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.25f}}, NULL}};
    static const maniGraphConditionSource kHalf[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.5f}}, NULL}};
    static const maniGraphConditionSource kLong[1] = {
        {maniGraphConditionTimeInState, 0, 0, 0, {{0.75f}}, NULL}};
    static const uint32_t kOuter[2] = {1, 2};
    static const uint32_t kInner[2] = {3, 4};
    maniGraphTransitionSource outer[2] = {Go(0, 1, 0, 0.0f, kQuarter, 1),
                                          Go(1, 0, 1, 0.0f, kLong, 1)};
    maniGraphTransitionSource inner[1] = {Go(0, 1, 0, 0.0f, kHalf, 1)};
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeMachine, 2, kOuter, .machine = {outer, 2, 0, 1, 0, NULL}},
        ClipNode(3, 1.0f, maniGraphLoopClip),
        {maniGraphNodeMachine, 2, kInner, .machine = {inner, 1, 0, 1, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 5}, &g);
    // Nodes: the short step 0, the walk 1, the run 2, the inner 3, the
    // outer 4. The outer enters the inner at the 1st update, leaves it at
    // the 4th, enters it again at the 5th.
    for (int k = 0; k < 3; ++k)
    {
        Update(instance, 0.25f);
    }
    CHECK(NearF(State(instance, &g, 2)->weight, 1.0), "inside: the run");
    Update(instance, 0.25f);
    Update(instance, 0.25f);
    CHECK(NearF(State(instance, &g, 1)->weight, 1.0) && NearF(State(instance, &g, 2)->weight, 0.0),
          "entered again: the inner machine at its entry");
}

static _Alignas(8) uint8_t s_saved[2][4096];
static _Alignas(8) uint8_t s_other[8192];

// Saved halfway through a fade and restored into another instance: both
// go on alike.
static void TestSaved(void)
{
    static const maniGraphConditionSource kFast[1] = {
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, {{1.0f}}, NULL}};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 1.0f, kFast, 1)};
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 1, 0, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.5f);
    Update(instance, 0.5f);
    size_t size = maniGraphStateSize(instance->binding);
    maniGraphInstance* other = NULL;
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniRestoreGraphInstance(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    Update(instance, 0.25f);
    Update(other, 0.25f);
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveGraphInstance(other, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], size) == 0 && Weights3(other, &g, 0.25f, 0.75f, 0),
          "the same after");
    // The block follows the 4 parameters' and 4 nodes' 16 bytes each.
    size_t block = MANI_GRAPH_STATE_HEADER_SIZE + 8 * MANI_GRAPH_VALUE_SIZE;
    // The word changed, its value, the word refused: the header, the
    // record (binary64 high words), a weight, a timer, a heard bit past
    // the one condition.
    const uint32_t kBad[13][3] = {{0, 2, 1},
                                  {0, 3, 0},
                                  {1, 5, 1},
                                  {2, 0xFF800000u, 2},
                                  {4, 0xFFF00000u, 3},
                                  {6, 0x3FF00001u, 5},
                                  {8, 0xBFF00000u, 7},
                                  {9, 1, 9},
                                  {12, 0xBFF00000u, 11},
                                  {13, 2, 13},
                                  {14, 0x3F800001u, 14},
                                  {20, 0xBF800000u, 20},
                                  {21, 2, 21}};
    for (int k = 0; k < 13; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kBad[k][0], kBad[k][1]);
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kBad[k][2],
              "a saved block refused at its word");
    }
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[4][4096];

// Halfway through the walk to the run: the blend of the two at a half.
static void TestEvaluated(void)
{
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 1.0f, NULL, 0)};
    maniGraph g;
    maniGraphInstance* instance = Machine(t, 1, 0, &g);
    Update(instance, 0.25f);
    Update(instance, 0.25f);
    maniPose* poses[4];
    for (int i = 0; i < 4; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], &poses[i]) ==
                  mani_success,
              "a pose");
    }
    // A quarter into the fade: the walk at 0.5 weighs 0.75, the run at
    // 0.25 weighs 0.25.
    const maniBlendLayer layers[2] = {{poses[1], 0.75f, NULL}, {poses[2], 0.25f, NULL}};
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, poses[0]) == mani_success &&
              maniSampleClip(&s_clips[0], &s_skeleton, 0.5f, poses[1]) == mani_success &&
              maniSampleClip(&s_clips[1], &s_skeleton, 0.25f, poses[2]) == mani_success &&
              maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, poses[3]) ==
                  mani_success,
          "evaluated");
    bool same = true;
    for (uint32_t bone = 0; bone < 3; ++bone)
    {
        maniTransform a = maniPoseLocal(poses[0], bone);
        maniTransform b = maniPoseLocal(poses[3], bone);
        same = same && memcmp(&a, &b, sizeof a) == 0;
    }
    CHECK(same, "the machine's pose, bit for bit");
}

static maniBuildResult Built(const maniGraphTransitionSource* t, uint32_t count, uint32_t entry,
                             uint32_t chain)
{
    static const uint32_t kStates[3] = {1, 2, 3};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 3, kStates, .machine = {t, count, entry, chain, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip)};
    return BuildOnly(nodes, 4, s_parameters, 4);
}

static bool Transition(maniBuildResult r, uint32_t t)
{
    return Refused(r, maniBuildReasonTransition, t);
}

static void TestRefusals(void)
{
    maniGraphTransitionSource t[2] = {Go(0, 1, 0, 0.0f, NULL, 0), Go(1, 2, 1, 0.0f, NULL, 0)};
    CHECK(Built(t, 2, 0, 0).result == mani_success, "builds when right");
    CHECK(Refused(Built(t, 2, 3, 0), maniBuildReasonNode, 0), "an entry past the states");
    CHECK(Refused(Built(t, 2, 0, 17), maniBuildReasonNode, 0), "a chain past 16");
    CHECK(Refused(Built(NULL, 2, 0, 0), maniBuildReasonNode, 0), "no transitions");
    t[1].to = 1;
    CHECK(Transition(Built(t, 2, 0, 0), 1), "to its own source");
    t[1] = Go(MANI_GRAPH_ANY_STATE, 2, 0, 0.0f, NULL, 0);
    CHECK(Transition(Built(t, 2, 0, 0), 1), "an any-state priority shared");
    t[1] = Go(2, 0, 0, 0.0f, NULL, 0);
    CHECK(Built(t, 2, 0, 0).result == mani_success, "a priority shared with no rival");
    t[1] = Go(3, 0, 1, 0.0f, NULL, 0);
    CHECK(Transition(Built(t, 2, 0, 0), 1), "from past the states");
    t[1] = Go(1, 0, 1, NAN, NULL, 0);
    CHECK(Transition(Built(t, 2, 0, 0), 1), "a duration not finite");
    t[1] = Go(1, 0, 1, 0.0f, NULL, 0);
    t[1].cooldown = -1.0f;
    CHECK(Transition(Built(t, 2, 0, 0), 1), "a negative cooldown");
    t[1].cooldown = 0.0f;
    t[1].interruption = 3;
    CHECK(Transition(Built(t, 2, 0, 0), 1), "an interruption past any");
    t[1].interruption = 0;
    t[1].conditionCount = 1;
    CHECK(Transition(Built(t, 2, 0, 0), 1), "conditions counted but none");
    maniGraphConditionSource c[17];
    for (int k = 0; k < 17; ++k)
    {
        c[k] = Greater(0, 1.0f);
    }
    t[1].conditions = c;
    t[1].conditionCount = 17;
    CHECK(Transition(Built(t, 2, 0, 0), 1), "17 conditions");
    t[1].conditionCount = 1;
    const maniGraphConditionSource kWrong[6] = {
        {maniGraphConditionParameter, 0, maniGraphCompareEqual, 0, Float(1.0f), NULL},
        {maniGraphConditionParameter, 2, maniGraphCompareLess, 0, Int(1), NULL},
        {maniGraphConditionParameter, 2, maniGraphCompareEqual, 0, Int(2), NULL},
        {maniGraphConditionTrigger, 2, 0, 0, Int(0), NULL},
        {maniGraphConditionTimeInState, 0, 0, 0, Float(-1.0f), NULL},
        {6, 0, 0, 0, Int(0), NULL}};
    for (int k = 0; k < 6; ++k)
    {
        c[0] = kWrong[k];
        CHECK(Transition(Built(t, 2, 0, 0), 1), "a condition wrong");
    }
    c[0] = Trigger(1);
    CHECK(Built(t, 2, 0, 0).result == mani_success, "a trigger");
}

// The machine of TestFade's graph: its words checked on open.
static void TestOpenRefusals(void)
{
    static const maniGraphConditionSource kFast[1] = {
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, {{1.0f}}, NULL}};
    maniGraphTransitionSource t[2] = {Go(0, 1, 0, 0.5f, kFast, 1), Go(1, 2, 4, 0.0f, NULL, 0)};
    maniGraph g;
    (void)Machine(t, 2, 0, &g);
    // Words: entry 0, chain 1, count 2, block 3, conditions 4, listens
    // 5; transitions from 6 and 14; the condition from 22; the states
    // from 26.
    CHECK(RefusedAtWord(&g, 3, 0, 3), "an entry past the states");
    CHECK(RefusedAtWord(&g, 3, 1, 0), "a chain of 0");
    CHECK(RefusedAtWord(&g, 3, 1, 17), "a chain past 16");
    CHECK(RefusedAtWord(&g, 3, 3, 1), "a block not where the count puts it");
    CHECK(RefusedAtWord(&g, 3, 5, 1), "listening with nothing to listen for");
    CHECK(RefusedAtWord(&g, 3, 6, 3), "from past the states");
    CHECK(RefusedAtWord(&g, 3, 7, 0), "to its own source");
    uint64_t at = 0;
    CHECK(Crafted(&g, 3, 16, 0, &at).result == mani_success, "an equal priority (a tie) opens");
    CHECK(RefusedAtWordOf(&g, 3, 8, 5, 16), "priorities out of order");
    CHECK(RefusedAtWord(&g, 3, 9, 3), "an interruption past any");
    CHECK(RefusedAtWord(&g, 3, 9, 0x30u), "an entry past match");
    CHECK(RefusedAtWord(&g, 3, 9, 0x300u), "a fade past inertial");
    CHECK(RefusedAtWord(&g, 3, 10, Bits(-1.0f)), "a negative duration");
    CHECK(RefusedAtWord(&g, 3, 11, 0x7FC00000u), "a cooldown not finite");
    CHECK(RefusedAtWord(&g, 3, 12, 1), "a first condition not in turn");
    CHECK(RefusedAtWordOf(&g, 3, 22, 6, 22), "a condition of no kind");
    CHECK(RefusedAtWordOf(&g, 3, 23, 1, 22), "a float compare on a trigger");
    CHECK(RefusedAtWordOf(&g, 3, 24, maniGraphCompareEqual, 22), "a float compared for equality");
    CHECK(RefusedAtWord(&g, 3, 26, 2), "a final flag past 1");
    CHECK(RefusedAtWord(&g, 3, 27, 0), "a nearest machine where it does not listen");
    CHECK(RefusedAtWord(&g, 3, 28, 1), "a list's first not in turn");
}

// A machine in a subgraph, its condition reading the outer graph's
// parameter through the map: built as the same graph written flat.
static void TestSubgraph(void)
{
    static const uint32_t kStates[2] = {1, 2};
    maniGraphParameterSource inner = Parameter("v", maniGraphParameterFloat, Float(0));
    static const maniGraphConditionSource kFast[1] = {
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, {{1.0f}}, NULL}};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.5f, kFast, 1)};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraphSource sub = {&s_skeleton, &inner, kSlots, NULL, nodes, 1,    CLIPS, 0,   3,
                           0,           0,      NULL,   0,    0,     NULL, NULL,  NULL};
    maniGraphParameterSource outer[2] = {Parameter("a", maniGraphParameterFloat, Float(0)),
                                         Parameter("speed", maniGraphParameterFloat, Float(0))};
    static const uint32_t kMap[1] = {1};
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&sub, kMap}};
    static const maniGraphConditionSource kFlatFast[1] = {
        {maniGraphConditionParameter, 1, maniGraphCompareGreater, 0, {{1.0f}}, NULL}};
    maniGraphTransitionSource flatT[1] = {Go(0, 1, 0, 0.5f, kFlatFast, 1)};
    maniGraphNodeSource flat[3] = {nodes[0], nodes[1], nodes[2]};
    flat[0].machine.transitions = flatT;
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
          "a machine inlined as written flat");
}

static char s_text[16384];
static _Alignas(16) uint8_t s_built[16384];
static _Alignas(16) uint8_t s_read[16384];
static size_t s_textSize;
static size_t s_builtSize;

// The source written and read back to the same bytes; the text kept.
static bool RoundTrip(const maniGraphSource* source)
{
    size_t readSize = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
              maniWriteGraphJson(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &s_textSize) == mani_success &&
              maniBuildGraphJson(builder, s_text, s_textSize,
                                 &(maniGraphNamedAssets){&s_skeleton, kSlots, NULL, NULL, NULL,
                                                         CLIPS, 0, 0, 0, NULL})
                      .result == mani_success &&
              maniCopyBuilt(builder, s_read, sizeof s_read, &readSize) == mani_success;
    maniDestroyBuilder(builder);
    s_text[ok ? s_textSize : 0] = 0;
    return ok && readSize == s_builtSize && memcmp(s_read, s_built, s_builtSize) == 0;
}

static maniBuildResult Read(const char* text)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraphJson(
        builder, text, strlen(text),
        &(maniGraphNamedAssets){&s_skeleton, kSlots, NULL, NULL, NULL, CLIPS, 0, 0, 0, NULL});
    maniDestroyBuilder(builder);
    return r;
}

// Every option and every form of condition, written and read back.
static void TestJson(void)
{
    static const uint32_t kStates[3] = {1, 2, 3};
    const maniGraphConditionSource kRun[3] = {
        {maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, Float(1.5f), NULL},
        {maniGraphConditionParameter, 2, maniGraphCompareEqual, 0, Int(1), NULL},
        {maniGraphConditionParameter, 3, maniGraphCompareAtMost, 0, Int(-3), NULL}};
    const maniGraphConditionSource kJump[2] = {
        {maniGraphConditionTrigger, 1, 0, 0, Int(0), NULL},
        {maniGraphConditionTimeInState, MANI_NO_PARAMETER, 0, 0, Float(0.25f), NULL}};
    maniGraphTransitionSource t[2] = {Go(0, 1, 2, 0.5f, kRun, 3),
                                      Go(MANI_GRAPH_ANY_STATE, 2, 1, 0.0f, kJump, 2)};
    t[0].interruption = maniGraphInterruptionHigher;
    t[0].entry = maniGraphEntryKeep;
    t[0].fade = maniGraphFadeSmoothstep;
    t[0].cooldown = 1.0f;
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 3, kStates, .machine = {t, 2, 1, 6, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip)};
    maniGraphSource source = {&s_skeleton, s_parameters, kSlots, NULL, nodes, 4,    CLIPS, 0, 4, 0,
                              0,           NULL,         0,      0,    NULL,  NULL, NULL};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "\"entry\": 1, \"chainLimit\": 6, \"transitions\": [{\"from\": 0, "
                         "\"to\": 1, \"priority\": 2, \"interruption\": \"higher\", "
                         "\"entry\": \"keep\", \"fade\": \"smoothstep\", \"duration\": "
                         "0.5, \"cooldown\": 1, \"conditions\": [{\"parameter\": \"speed\", "
                         "\"compare\": \">\", \"value\": 1.5}, {\"parameter\": \"crouch\", "
                         "\"compare\": \"==\", \"value\": true}, {\"parameter\": \"gait\", "
                         "\"compare\": \"<=\", \"value\": -3}]}, {\"from\": \"any\"") != NULL &&
              strstr(s_text, "\"conditions\": [{\"trigger\": \"jump\"}, {\"timeInState\": "
                             "0.25}]}], \"finals\": []}") != NULL,
          "the form");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"speed\", \"type\": \"float\"}, {\"name\": \"jump\", \"type\": \"trigger\"}], "              \
    "\"clips\": [\"walk\", \"run\"], \"nodes\": [{\"kind\": \"machine\", \"inputs\": [1, "         \
    "2]"
#define TAIL                                                                                       \
    "}, {\"kind\": \"clip\", \"clip\": \"walk\"}, {\"kind\": \"clip\", \"clip\": "                 \
    "\"run\"}], \"output\": 0}"

// Members left out, and forms refused at their bytes.
static void TestJsonForms(void)
{
    static const char kLeast[] = HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1}]" TAIL;
    CHECK(Read(kLeast).result == mani_success, "a transition of from and to alone");
    static const char kNone[] = HEAD TAIL;
    CHECK(Read(kNone).result == mani_success, "a machine of no transitions");
    static const struct
    {
        const char* text;
        uint32_t form;
        const char* marker;
    } kRefusals[7] = {
        {HEAD ", \"transitions\": [{\"from\": \"all\", \"to\": 1}]" TAIL, maniGraphFormValue,
         "all\""},
        {HEAD ", \"transitions\": [{\"from\": 0, \"to\": \"any\"}]" TAIL, maniGraphFormValue,
         "any\""},
        {HEAD ", \"transitions\": [{\"from\": 0}]" TAIL, maniGraphFormMissing, "{\"from"},
        {HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1, \"conditions\": [{\"parameter\": "
              "\"speed\", \"compare\": \"=~\", \"value\": 1}]}]" TAIL,
         maniGraphFormValue, "=~"},
        {HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1, \"conditions\": [{\"parameter\": "
              "\"speed\", \"value\": 1}]}]" TAIL,
         maniGraphFormMissing, "{\"parameter"},
        {HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1, \"conditions\": [{\"trigger\": "
              "\"jump\", \"timeInState\": 1}]}]" TAIL,
         maniGraphFormMember, "timeInState"},
        {HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1, \"fade\": \"cubic\"}]" TAIL,
         maniGraphFormValue, "cubic"}};
    for (int k = 0; k < 7; ++k)
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
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        Parameters();
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestFade();
    TestInterruption();
    TestPriority();
    TestChains();
    TestAnyToCurrent();
    TestCooldown();
    TestConditions();
    TestEntries();
    TestTimeScale();
    TestNested();
    TestSaved();
    TestEvaluated();
    TestRefusals();
    TestOpenRefusals();
    TestSubgraph();
    TestJson();
    TestJsonForms();
    return s_failures == 0 ? 0 : 1;
}
