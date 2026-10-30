// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// State machines listening to their leading clips: phase
// below 1 at every loop (reaching it counted, leaving it not), across a
// loop's end both ways, phase of 1 or more once, backward play counted
// as loops; the record cleared when a transition fires; the leading clip
// the one weighed most; finished by a clamped clip at the end it moves
// toward, never by a repeating one, and kept while unreached; events heard from any clip
// beneath the state that listed them, not from one below the threshold;
// a match entry seeking at the leading clip's phase; a nested machine's
// final state finishing its state once its fade is done; a machine no
// parent reached keeping its triggers; the record saved and restored;
// the builder's refusals and opening's checks of lists; the JSON form,
// round trip and refusals. Run as test_graph_listen --seed <graph file>
// <json file> it writes the round trip's file and text, seeds of the
// fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_machine.h"
#include "graph_space_fixture.h"
#include "negative_kinds.h"

#include <stdio.h>

// Parameters: speed (float), jump (trigger).
static maniGraphParameterSource s_parameters[2];

static void Parameters(void)
{
    s_parameters[0] = Parameter("speed", maniGraphParameterFloat, Float(0.0f));
    s_parameters[1] = Parameter("jump", maniGraphParameterTrigger, Int(0));
}

static maniGraphInstance* MakeNodes(const maniGraphNodeSource* nodes, uint32_t count, maniGraph* g)
{
    Parameters();
    return Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = count},
        g);
}

// The current state of the machine node p.
static uint32_t Current(const maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    const uint8_t* e = g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
    maniGraphMachine m = {g, maniLoadU32(e + 12), maniLoadU32(e + 8)};
    const uint32_t* blocks = (const uint32_t*)((const uint8_t*)instance + instance->machinesAt);
    return blocks[maniGraphMachineBlockAt(&m) + maniGraphBlockCurrent];
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
                                       count};
}

static maniGraphConditionSource Phase(float value)
{
    return (maniGraphConditionSource){
        maniGraphConditionPhase, MANI_NO_PARAMETER, 0, 0, Float(value), NULL};
}

static maniGraphConditionSource Finished(void)
{
    return (maniGraphConditionSource){
        maniGraphConditionFinished, MANI_NO_PARAMETER, 0, 0, Int(0), NULL};
}

static maniGraphConditionSource Event(const char* name)
{
    return (maniGraphConditionSource){
        maniGraphConditionEvent, MANI_NO_PARAMETER, 0, strlen(name), Int(0), name};
}

static maniGraphConditionSource Fast(void)
{
    return (maniGraphConditionSource){
        maniGraphConditionParameter, 0, maniGraphCompareGreater, 0, Float(1.0f), NULL};
}

static maniGraphConditionSource Jump(void)
{
    return (maniGraphConditionSource){maniGraphConditionTrigger, 1, 0, 0, Int(0), NULL};
}

// A machine over the walk (a second, its loop as given, at speed), the
// long step (two seconds, clamped) and the run: states and nodes 0, 1
// and 2, the machine node 3.
static maniGraphInstance* Three(const maniGraphTransitionSource* t, uint32_t count, uint32_t entry,
                                float speed, maniGraph* g)
{
    static const uint32_t kStates[3] = {1, 2, 3};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 3, kStates, .machine = {t, count, entry, 0, 0, NULL}},
        ClipNode(0, speed, maniGraphLoopRepeat),
        ClipNode(4, speed, maniGraphLoopClamp),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    return MakeNodes(nodes, 4, g);
}

static void Steps(maniGraphInstance* instance, int count, float delta)
{
    for (int k = 0; k < count; ++k)
    {
        Update(instance, delta);
    }
}

// Phase below 1: crossed by the update before, at every loop, reaching
// it counted and leaving it not.
static void TestPhase(void)
{
    maniGraphConditionSource c[2] = {Phase(0.5f), Fast()};
    maniGraphTransitionSource t[1] = {Go(0, 2, 0, 0.0f, c, 2)};
    maniGraph g;
    maniGraphInstance* instance = Three(t, 1, 0, 1.0f, &g);
    // Quarters: the second reaches 0.5 while the gate is shut.
    Steps(instance, 3, 0.25f);
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    // Deciding on (0.5, 0.75], (0.75, 1], (0, 0.25]: not passed.
    Steps(instance, 3, 0.25f);
    CHECK(Current(instance, &g, 3) == 0, "phase left, not passed");
    // Deciding on (0.25, 0.5] of the second loop.
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 3) == 2, "phase passed again a loop later");

    c[1] = c[0];
    instance = Three(t, 1, 0, 1.0f, &g);
    Update(instance, 0.5f);
    Update(instance, 0.5f);
    CHECK(Current(instance, &g, 3) == 2, "a move reaching the phase passes it");

    c[0] = Phase(0.75f);
    c[1] = c[0];
    instance = Three(t, 1, 0, -1.0f, &g);
    Update(instance, 0.125f);
    Update(instance, 0.0f);
    CHECK(Current(instance, &g, 3) == 0, "backward from 0 to 0.875: not passed");
    Update(instance, 0.25f);
    Update(instance, 0.0f);
    CHECK(Current(instance, &g, 3) == 2, "backward past 0.75: passed");

    // Across a loop's end: 0.875 to 0.375 passes 0.25.
    c[0] = Phase(0.25f);
    c[1] = Fast();
    instance = Three(t, 1, 0, 1.0f, &g);
    Update(instance, 0.5f);
    Update(instance, 0.375f);
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.5f);
    CHECK(Current(instance, &g, 3) == 0, "0.5 to 0.875: not passed");
    Update(instance, 0.0f);
    CHECK(Current(instance, &g, 3) == 2, "passed across the loop's end");

    // Backward across it: 0.125 to 0.875 passes 0.0625.
    c[0] = Phase(0.0625f);
    c[1] = c[0];
    instance = Three(t, 1, 0, -1.0f, &g);
    Update(instance, 0.875f);
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 3) == 0, "backward from 1 to 0.125: not passed");
    Update(instance, 0.0f);
    CHECK(Current(instance, &g, 3) == 2, "backward across the loop's start: passed");
}

// A phase of one or more: the loops since the state was entered, once.
static void TestLoops(void)
{
    maniGraphConditionSource c[1] = {Phase(1.5f)};
    maniGraphTransitionSource t[1] = {Go(0, 2, 0, 0.0f, c, 1)};
    maniGraph g;
    maniGraphInstance* instance = Three(t, 1, 0, 1.0f, &g);
    Steps(instance, 3, 0.5f);
    CHECK(Current(instance, &g, 3) == 0, "a loop: not passed");
    Update(instance, 0.5f);
    CHECK(Current(instance, &g, 3) == 2, "a loop and a half: passed");

    maniGraphConditionSource gated[2] = {Phase(1.5f), Fast()};
    t[0].conditions = gated;
    t[0].conditionCount = 2;
    instance = Three(t, 1, 0, 1.0f, &g);
    Steps(instance, 4, 0.5f);
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Steps(instance, 6, 0.5f);
    CHECK(Current(instance, &g, 3) == 0, "passed once, not again");

    t[0].conditions = c;
    t[0].conditionCount = 1;
    instance = Three(t, 1, 0, -1.0f, &g);
    Steps(instance, 4, 0.5f);
    CHECK(Current(instance, &g, 3) == 2, "backward loops count as loops");
}

// A transition firing clears the record: the next state's phase has not
// passed, so an instant chain stops there.
static void TestCleared(void)
{
    maniGraphConditionSource c[1] = {Phase(0.5f)};
    maniGraphTransitionSource t[2] = {Go(0, 1, 0, 0.0f, c, 1), Go(1, 2, 1, 0.0f, c, 1)};
    maniGraph g;
    maniGraphInstance* instance = Three(t, 2, 0, 1.0f, &g);
    Update(instance, 0.75f);
    Update(instance, 0.0f);
    CHECK(Current(instance, &g, 3) == 1, "passed once: one transition, not a chain");
}

// The leading clip is the one weighed most: the clamped long step at 0.8
// over the walk at 0.2 finishes the state.
static void TestLeading(void)
{
    static const uint32_t kStates[2] = {1, 4};
    static const uint32_t kBlend[2] = {2, 3};
    static const maniGraphScalar kWeights[2] = {{MANI_NO_PARAMETER, 0.25f},
                                                {MANI_NO_PARAMETER, 1.0f}};
    maniGraphConditionSource c[1] = {Finished()};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.0f, c, 1)};
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        {maniGraphNodeBlend, 2, kBlend, .blend = {kWeights, 0}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(4, 1.0f, maniGraphLoopClamp),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    maniGraphInstance* instance = MakeNodes(nodes, 5, &g);
    Steps(instance, 4, 0.5f);
    CHECK(Current(instance, &g, 4) == 0, "the long step at its end after the update");
    Update(instance, 0.5f);
    CHECK(Current(instance, &g, 4) == 1, "finished by the clip weighed most");
}

// Finished: a clamped clip at the end it moves toward; never repeating.
static void TestFinished(void)
{
    maniGraphConditionSource c[1] = {Finished()};
    maniGraphTransitionSource t[1] = {Go(1, 0, 0, 0.0f, c, 1)};
    maniGraph g;
    maniGraphInstance* instance = Three(t, 1, 1, 1.0f, &g);
    Steps(instance, 3, 0.75f);
    CHECK(Current(instance, &g, 3) == 1, "at its end after the update");
    Update(instance, 0.75f);
    CHECK(Current(instance, &g, 3) == 0, "finished, taken the update after");

    instance = Three(t, 1, 1, -1.0f, &g);
    Update(instance, 0.25f);
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 3) == 0, "played backward: finished at its start");

    t[0] = Go(0, 2, 0, 0.0f, c, 1);
    instance = Three(t, 1, 0, 1.0f, &g);
    Steps(instance, 10, 0.5f);
    CHECK(Current(instance, &g, 3) == 0, "a repeating clip never finishes");
}

// A machine whose first state blends the walk (weight speed) with the
// run, its second state the run alone (the node shared): walk 0, run 1,
// blend 2, the machine 3.
static maniGraphInstance* Shared(const maniGraphTransitionSource* t, maniGraph* g)
{
    static const uint32_t kStates[2] = {1, 3};
    static const uint32_t kBlend[2] = {2, 3};
    static const maniGraphScalar kWeights[2] = {{0, 0.0f}, {MANI_NO_PARAMETER, 1.0f}};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 2, kStates, .machine = {t, 1, 0, 0, 0, NULL}},
        {maniGraphNodeBlend, 2, kBlend, .blend = {kWeights, 0}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    return MakeNodes(nodes, 4, g);
}

// Events: heard from a clip beneath the state that listed them, by the
// update before; not from one weighed at 0; a whole loop hears all.
static void TestEvents(void)
{
    maniGraphConditionSource c[1] = {Event("right")};
    maniGraphTransitionSource t[1] = {Go(0, 2, 0, 0.0f, c, 1)};
    maniGraph g;
    maniGraphInstance* instance = Three(t, 1, 0, 1.0f, &g);
    Steps(instance, 3, 0.25f);
    CHECK(Current(instance, &g, 3) == 0, "right listed by the third quarter");
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 3) == 2, "heard, taken the update after");

    instance = Three(t, 1, 0, 1.0f, &g);
    Update(instance, 0.25f);
    Update(instance, 2.0f);
    Update(instance, 0.0f);
    CHECK(Current(instance, &g, 3) == 2, "a whole loop hears every event");

    t[0] = Go(0, 1, 0, 0.0f, c, 1);
    instance = Shared(t, &g);
    Steps(instance, 4, 0.25f);
    CHECK(Current(instance, &g, 3) == 0, "a walk weighed at 0 lists nothing");
    Set(instance, 0, maniGraphParameterFloat, Float(0.5f));
    Steps(instance, 3, 0.25f);
    CHECK(Current(instance, &g, 3) == 0, "the walk's right not yet listed");
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 3) == 1, "heard from the walk, the run leading");
}

// A match entry: the state entered at weight 0 seeks its clips to the
// leading clip's phase times their durations.
static void TestMatch(void)
{
    maniGraphConditionSource c[1] = {Jump()};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.5f, c, 1)};
    t[0].entry = maniGraphEntryMatch;
    maniGraph g;
    maniGraphInstance* instance = Three(t, 1, 0, 1.0f, &g);
    Update(instance, 0.25f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 3) == 1 && NearF((float)State(instance, &g, 1)->time, 0.5),
          "the long step at a quarter of its two seconds");
    CHECK(NearF((float)State(instance, &g, 0)->time, 0.5), "the walk going on as it fades");
}

// An outer machine over an inner one (walk to the short step on jump,
// the short step final) and the run: walk 0, short 1, inner 2, run 3,
// outer 4. The outer leaves when its state finishes.
static void TestNested(void)
{
    static const uint32_t kOuter[2] = {1, 4};
    static const uint32_t kInner[2] = {2, 3};
    static const uint32_t kFinals[1] = {1};
    maniGraphConditionSource jump[1] = {Jump()};
    maniGraphConditionSource done[1] = {Finished()};
    maniGraphTransitionSource inner[1] = {Go(0, 1, 0, 0.5f, jump, 1)};
    maniGraphTransitionSource outer[1] = {Go(0, 1, 0, 0.0f, done, 1)};
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeMachine, 2, kOuter, .machine = {outer, 1, 0, 0, 0, NULL}},
        {maniGraphNodeMachine, 2, kInner, .machine = {inner, 1, 0, 0, 1, kFinals}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(3, 1.0f, maniGraphLoopRepeat),
        ClipNode(1, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    maniGraphInstance* instance = MakeNodes(nodes, 5, &g);
    Update(instance, 0.25f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Steps(instance, 3, 0.25f);
    CHECK(Current(instance, &g, 2) == 1 && Current(instance, &g, 4) == 0,
          "the inner fading into its final state: not finished");
    Update(instance, 0.25f);
    CHECK(Current(instance, &g, 4) == 1, "the fade done: finished");
}

// A machine no parent reached does not step: its trigger waits.
static void TestUnreached(void)
{
    static const uint32_t kOuter[2] = {1, 2};
    static const uint32_t kInner[2] = {3, 4};
    maniGraphConditionSource fast[1] = {Fast()};
    maniGraphConditionSource jump[1] = {Jump()};
    maniGraphTransitionSource outer[1] = {Go(0, 1, 0, 0.0f, fast, 1)};
    maniGraphTransitionSource inner[1] = {Go(0, 1, 0, 0.0f, jump, 1)};
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeMachine, 2, kOuter, .machine = {outer, 1, 0, 0, 0, NULL}},
        ClipNode(1, 1.0f, maniGraphLoopRepeat),
        {maniGraphNodeMachine, 2, kInner, .machine = {inner, 1, 0, 0, 0, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(3, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    maniGraphInstance* instance = MakeNodes(nodes, 5, &g);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Steps(instance, 2, 0.25f);
    CHECK(instance->cells[1].value.b == 1 && Current(instance, &g, 3) == 0,
          "unreached: the trigger kept");
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.25f);
    CHECK(instance->cells[1].value.b == 0 && Current(instance, &g, 4) == 1 &&
              Current(instance, &g, 3) == 1,
          "entered: the inner started over and took the trigger");
}

// A finished inner machine left unreached and entered again where it was
// left keeps its finish: it leaves at once. Run 0, long 1, walk 2, inner
// 3, outer 4.
static void TestFinishKept(void)
{
    static const uint32_t kOuter[2] = {1, 2};
    static const uint32_t kInner[2] = {3, 4};
    maniGraphConditionSource jump[1] = {Jump()};
    maniGraphConditionSource fast[1] = {Fast()};
    maniGraphConditionSource done[2] = {Finished(), Fast()};
    maniGraphTransitionSource outer[2] = {Go(1, 0, 0, 0.0f, jump, 1), Go(0, 1, 1, 0.0f, fast, 1)};
    outer[1].entry = maniGraphEntryKeep;
    maniGraphTransitionSource inner[1] = {Go(0, 1, 0, 0.0f, done, 2)};
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeMachine, 2, kOuter, .machine = {outer, 2, 1, 0, 0, NULL}},
        ClipNode(1, 1.0f, maniGraphLoopRepeat),
        {maniGraphNodeMachine, 2, kInner, .machine = {inner, 1, 0, 0, 0, NULL}},
        ClipNode(4, 1.0f, maniGraphLoopClamp),
        ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraph g;
    maniGraphInstance* instance = MakeNodes(nodes, 5, &g);
    Steps(instance, 5, 0.5f);
    Set(instance, 1, maniGraphParameterTrigger, Int(1));
    Steps(instance, 2, 0.5f);
    CHECK(Current(instance, &g, 4) == 0 && Current(instance, &g, 3) == 0,
          "the inner finished, left unreached");
    Set(instance, 0, maniGraphParameterFloat, Float(2.0f));
    Update(instance, 0.5f);
    CHECK(Current(instance, &g, 4) == 1 && Current(instance, &g, 3) == 1,
          "entered again where it was: its finish kept, taken at once");
}

static _Alignas(8) uint8_t s_saved[2][4096];
static _Alignas(8) uint8_t s_other[8192];

// Saved part way to a phase of 1.5 and restored: both decide alike.
static void TestSaved(void)
{
    maniGraphConditionSource c[1] = {Phase(1.5f)};
    maniGraphTransitionSource t[1] = {Go(0, 2, 0, 0.0f, c, 1)};
    maniGraph g;
    maniGraphInstance* instance = Three(t, 1, 0, 1.0f, &g);
    Steps(instance, 3, 0.5f);
    size_t size = SavedGraphSize(instance);
    maniGraphInstance* other = NULL;
    CHECK(size <= sizeof s_saved[0] &&
              maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) ==
                  mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniGraphInstance_Restore(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    Update(instance, 0.5f);
    Update(other, 0.5f);
    CHECK(Current(other, &g, 3) == 2 &&
              maniGraphInstance_Save(instance, s_saved[0], sizeof s_saved[0], NULL) ==
                  mani_success &&
              maniGraphInstance_Save(other, s_saved[1], sizeof s_saved[1], NULL) == mani_success &&
              memcmp(s_saved[0], s_saved[1], size) == 0,
          "the same after");
}

static maniBuildResult Built(const maniGraphTransitionSource* t, const uint32_t* finals,
                             uint32_t finalCount)
{
    static const uint32_t kStates[3] = {1, 2, 3};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 3, kStates, .machine = {t, 1, 0, 0, finalCount, finals}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(4, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    Parameters();
    return BuildOnly(nodes, 4, s_parameters, 2);
}

static void TestRefusals(void)
{
    maniGraphConditionSource c[1] = {Phase(0.5f)};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.0f, c, 1)};
    static const uint32_t kPast[1] = {3};
    CHECK(Built(t, NULL, 0).result == mani_success, "a phase");
    CHECK(Refused(Built(t, kPast, 1), maniBuildReasonNode, 0), "a final past the states");
    CHECK(Refused(Built(t, NULL, 1), maniBuildReasonNode, 0), "finals counted, none given");
    const maniGraphConditionSource kBad[4] = {
        Phase(-0.5f), Phase(NAN), Event(""), {maniGraphConditionEvent, 0, 0, 3, Int(0), NULL}};
    for (int k = 0; k < 4; ++k)
    {
        c[0] = kBad[k];
        CHECK(Refused(Built(t, NULL, 0), maniBuildReasonTransition, 0), "a condition refused");
    }
    c[0] = Event("a");
    c[0].eventLength = (size_t)MANI_MAX_NAME_BYTES + 1;
    maniBuildResult longEvent = Built(t, NULL, 0);
    CHECK(longEvent.result == mani_errorLimit && longEvent.reason == maniBuildReasonTransition,
          "an event past MANI_MAX_NAME_BYTES");
    c[0] = Finished();
    t[0].entry = maniGraphEntryMatch + 1;
    CHECK(Refused(Built(t, NULL, 0), maniBuildReasonTransition, 0), "an entry past match");
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        t[0].entry = kNegativeKinds[k];
        CHECK(Refused(Built(t, NULL, 0), maniBuildReasonTransition, 0), "a negative entry");
        t[0].entry = maniGraphEntryRestart;
        t[0].fade = kNegativeKinds[k];
        CHECK(Refused(Built(t, NULL, 0), maniBuildReasonTransition, 0), "a negative fade");
        t[0].fade = maniGraphFadeLinear;
        t[0].interruption = kNegativeKinds[k];
        CHECK(Refused(Built(t, NULL, 0), maniBuildReasonTransition, 0), "a negative interruption");
        t[0].interruption = maniGraphInterruptionNone;
        c[0].kind = kNegativeKinds[k];
        CHECK(Refused(Built(t, NULL, 0), maniBuildReasonTransition, 0),
              "a negative condition kind");
        c[0] = Finished();
    }
}

// The machine of Shared: its words checked on open. Words: header 0 to
// 5, the transition 6, the condition 14, the states 18 and 22 (final,
// nearest, first, count), the lists 26 (walk, run) and 28 (run).
static void TestOpenRefusals(void)
{
    maniGraphConditionSource c[1] = {Event("right")};
    maniGraphTransitionSource t[1] = {Go(0, 1, 0, 0.0f, c, 1)};
    maniGraph g;
    (void)Shared(t, &g);
    uint64_t at = 0;
    CHECK(Crafted(&g, 3, 15, 0, &at).result == mani_success, "an event's hash is any");
    CHECK(RefusedAtWord(&g, 3, 5, 0), "not listening with an event condition");
    CHECK(RefusedAtWordOf(&g, 3, 13, 0, 4), "conditions not adding up to the count");
    CHECK(RefusedAtWordOf(&g, 3, 17, 1, 14), "an event condition's value not 0");
    CHECK(RefusedAtWord(&g, 3, 19, 2), "a nearest machine of another kind");
    CHECK(RefusedAtWord(&g, 3, 19, 3), "the machine as its own nearest");
    CHECK(RefusedAtWord(&g, 3, 24, 1), "a list's first not in turn");
    CHECK(RefusedAtWord(&g, 3, 26, 2), "a list naming a blend");
    CHECK(RefusedAtWord(&g, 3, 27, 0), "a list out of node order");
    CHECK(RefusedAtWord(&g, 3, 28, 3), "a list naming a node not before it");
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
    bool ok =
        maniBuildGraph(builder, source).result == mani_success &&
        maniBuilder_CopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
        maniWriteGraphJson(builder, source).result == mani_success &&
        maniBuilder_CopyBuilt(builder, s_text, sizeof s_text - 1, &s_textSize) == mani_success &&
        maniBuildGraphJson(
            builder, s_text, s_textSize,
            &(maniGraphNamedAssets){&s_skeleton, kSlots, NULL, NULL, NULL, CLIPS, 0, 0, 0, NULL})
                .result == mani_success &&
        maniBuilder_CopyBuilt(builder, s_read, sizeof s_read, &readSize) == mani_success;
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

// The new conditions, a match entry and finals, written and read back.
static void TestJson(void)
{
    static const uint32_t kStates[3] = {1, 2, 3};
    static const uint32_t kFinals[2] = {2, 1};
    const maniGraphConditionSource kEnd[3] = {Phase(0.5f), Finished(), Event("right")};
    maniGraphTransitionSource t[2] = {Go(0, 1, 0, 0.25f, kEnd, 3), Go(1, 2, 1, 0.0f, kEnd + 1, 1)};
    t[0].entry = maniGraphEntryMatch;
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeMachine, 3, kStates, .machine = {t, 2, 0, 0, 2, kFinals}},
        ClipNode(0, 1.0f, maniGraphLoopRepeat),
        ClipNode(4, 1.0f, maniGraphLoopClamp),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    Parameters();
    maniGraphSource source = {&s_skeleton, s_parameters, kSlots, NULL, nodes, 2,    CLIPS, 0, 4, 0,
                              0,           NULL,         0,      0,    NULL,  NULL, NULL};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "\"entry\": \"match\"") != NULL &&
              strstr(s_text, "\"conditions\": [{\"phase\": 0.5}, {\"finished\": true}, "
                             "{\"event\": \"right\"}]}") != NULL &&
              strstr(s_text, "\"finals\": [2, 1]}") != NULL,
          "the form");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"speed\", \"type\": \"float\"}], \"clips\": [\"walk\", \"run\"], \"nodes\": [{\"kind\": "    \
    "\"machine\", \"inputs\": [1, 2]"
#define TAIL                                                                                       \
    "}, {\"kind\": \"clip\", \"clip\": \"walk\"}, {\"kind\": \"clip\", \"clip\": "                 \
    "\"run\"}], \"output\": 0}"
#define WHEN(c) HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1, \"conditions\": [" c "]}]" TAIL

// Forms refused at their bytes.
static void TestJsonForms(void)
{
    CHECK(Read(WHEN("{\"event\": \"right\"}")).result == mani_success, "an event");
    CHECK(Read(HEAD ", \"finals\": [1]" TAIL).result == mani_success, "finals alone");
    static const struct
    {
        const char* text;
        uint32_t form;
        const char* marker;
    } kRefusals[6] = {
        {WHEN("{\"phase\": 0.5, \"finished\": true}"), maniGraphFormMember, "finished"},
        {WHEN("{\"phase\": 0.5, \"compare\": \">\"}"), maniGraphFormMember, "compare"},
        {WHEN("{\"finished\": false}"), maniGraphFormValue, "false"},
        {WHEN("{}"), maniGraphFormMissing, "{}"},
        {HEAD ", \"transitions\": [{\"from\": 0, \"to\": 1, \"entry\": \"matched\"}]" TAIL,
         maniGraphFormValue, "matched"},
        {HEAD ", \"finals\": [\"one\"]" TAIL, maniGraphFormValue, "one"}};
    for (int k = 0; k < 6; ++k)
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
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestPhase();
    TestLoops();
    TestCleared();
    TestLeading();
    TestFinished();
    TestFinishKept();
    TestEvents();
    TestMatch();
    TestNested();
    TestUnreached();
    TestSaved();
    TestRefusals();
    TestOpenRefusals();
    TestJson();
    TestJsonForms();
    return s_failures == 0 ? 0 : 1;
}
