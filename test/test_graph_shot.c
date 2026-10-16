// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One-shots (mani-0023): fired, faded in and played; faded out when the
// shot's clip has the fade-out's seconds left (backward, toward its
// start), never when it repeats; a smoothstep fade; fired again after an
// abort, from its start;
// fired again while fading out, rising from its weight and starting the
// shot over; aborted at once, abort winning over fire; the fade-out
// trigger, and a trigger read while idle consumed; the base and the shot
// shared by the weight; a masked shot evaluated bit for bit as the
// blending functions do; triggers waiting while no parent reaches it;
// saved and restored mid-fade; a one-shot in a subgraph; the builder's
// refusals and opening's checks; the JSON form, round trip and
// refusals. Run as test_graph_shot --seed <graph file> <json file> it
// writes the round trip's file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "graph_shot.h"
#include "graph_space_fixture.h"

#include "maul-anim/pose.h"

#include <stdio.h>

// Parameters: speed (float), fire, out and abort (triggers).
static maniGraphParameterSource s_parameters[4];

static void Parameters(void)
{
    s_parameters[0] = Parameter("speed", maniGraphParameterFloat, Float(0.0f));
    s_parameters[1] = Parameter("fire", maniGraphParameterTrigger, Int(0));
    s_parameters[2] = Parameter("out", maniGraphParameterTrigger, Int(0));
    s_parameters[3] = Parameter("abort", maniGraphParameterTrigger, Int(0));
}

static maniGraphOneShotNodeSource Shot(float fadeIn, float fadeOut)
{
    return (maniGraphOneShotNodeSource){1, 2, 3, maniGraphFadeLinear, fadeIn, fadeOut, NULL};
}

// A one-shot of the walk (repeating) as its base and the shot clip given
// (the long step, two seconds, by default clamped) at speed: base 0,
// shot 1, the one-shot 2.
static maniGraphInstance* Played(maniGraphOneShotNodeSource shot, uint32_t clip, maniGraphLoop loop,
                                 float speed, maniGraph* g)
{
    static const uint32_t kInputs[2] = {1, 2};
    Parameters();
    maniGraphNodeSource nodes[3] = {{maniGraphNodeOneShot, 2, kInputs, .oneShot = shot},
                                    ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ClipNode(clip, speed, loop)};
    return Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 4, .nodeCount = 3},
        g);
}

static maniGraphInstance* Make1(maniGraphOneShotNodeSource shot, uint32_t clip, maniGraphLoop loop,
                                maniGraph* g)
{
    return Played(shot, clip, loop, 1.0f, g);
}

static const uint32_t* Block(const maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    maniGraphShot shot = {g, maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12), 0};
    return (const uint32_t*)((const uint8_t*)instance + instance->machinesAt) +
           maniGraphShotWord(&shot, maniGraphShotBlock);
}

static float Weight(const maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    float w = 0.0f;
    memcpy(&w, &Block(instance, g, p)[maniGraphShotWeight], sizeof w);
    return w;
}

static uint32_t Phase(const maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    return Block(instance, g, p)[maniGraphShotPhase];
}

static void Pull(maniGraphInstance* instance, uint32_t trigger)
{
    Set(instance, trigger, maniGraphParameterTrigger, Int(1));
}

static void Steps(maniGraphInstance* instance, int count, float delta)
{
    for (int k = 0; k < count; ++k)
    {
        Update(instance, delta);
    }
}

// Fired, faded in over half a second, played, faded out over the last
// half second of the shot's two.
static void TestFire(void)
{
    maniGraph g;
    maniGraphInstance* instance = Make1(Shot(0.5f, 0.5f), 4, maniGraphLoopClamp, &g);
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotIdle && State(instance, &g, 1)->weight == 0.0f &&
              NearF(State(instance, &g, 0)->weight, 1.0),
          "idle: the base alone");
    Pull(instance, 1);
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotIn && Weight(instance, &g, 2) == 0.0f &&
              State(instance, &g, 1)->time == 0.0 && instance->cells[1].value.b == 0,
          "fired: the shot from its start, the trigger consumed");
    Update(instance, 0.25f);
    CHECK(NearF(Weight(instance, &g, 2), 0.5) && NearF(State(instance, &g, 1)->weight, 0.5) &&
              NearF(State(instance, &g, 0)->weight, 0.5),
          "halfway in: base and shot halved");
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotPlaying && Weight(instance, &g, 2) == 1.0f &&
              State(instance, &g, 0)->weight == 0.0f,
          "faded in: playing over the base");
    // The shot at 0.5; at 1.5 it has the fade-out's half second left.
    Steps(instance, 4, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotPlaying && Near(State(instance, &g, 1)->time, 1.5),
          "half a second left after the update");
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotOut && Weight(instance, &g, 2) == 1.0f,
          "fading out from the update after");
    Steps(instance, 2, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotIdle && Weight(instance, &g, 2) == 0.0f,
          "faded out: idle");

    instance = Make1(Shot(0.0f, 0.5f), 4, maniGraphLoopRepeat, &g);
    Pull(instance, 1);
    Steps(instance, 20, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotPlaying, "a repeating shot plays on");

    // Played backward from its start, a clamped shot has nothing left.
    instance = Played(Shot(0.0f, 0.5f), 4, maniGraphLoopClamp, -1.0f, &g);
    Pull(instance, 1);
    Steps(instance, 2, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotPlaying, "started over: not moved yet");
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotOut, "backward at its start: fading out");

    maniGraphOneShotNodeSource smooth = Shot(1.0f, 0.5f);
    smooth.curve = maniGraphFadeSmoothstep;
    instance = Make1(smooth, 4, maniGraphLoopClamp, &g);
    Pull(instance, 1);
    Update(instance, 0.25f);
    Update(instance, 0.25f);
    CHECK(Weight(instance, &g, 2) == 0.15625f, "a smoothstep fade a quarter in");
}

// Fired again while fading out: rising from its weight, from its start.
static void TestRefire(void)
{
    maniGraph g;
    maniGraphInstance* instance = Make1(Shot(0.5f, 0.5f), 4, maniGraphLoopRepeat, &g);
    Pull(instance, 1);
    Steps(instance, 3, 0.25f);
    Pull(instance, 2);
    Update(instance, 0.25f);
    Update(instance, 0.125f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotOut && NearF(Weight(instance, &g, 2), 0.75),
          "a quarter out");
    // The fade moves first, to 0.5, then fire reads.
    Pull(instance, 1);
    Update(instance, 0.125f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotIn && NearF(Weight(instance, &g, 2), 0.5) &&
              State(instance, &g, 1)->time == 0.0,
          "fired again: from its weight, the shot from its start");
    Update(instance, 0.25f);
    CHECK(NearF(Weight(instance, &g, 2), 0.75), "rising from its weight");

    // Aborted mid-shot, then fired again at weight 0: from its start.
    instance = Make1(Shot(0.5f, 0.5f), 4, maniGraphLoopClamp, &g);
    Pull(instance, 1);
    Steps(instance, 3, 0.25f);
    Pull(instance, 3);
    Update(instance, 0.25f);
    CHECK(Weight(instance, &g, 2) == 0.0f && NearF((float)State(instance, &g, 1)->time, 0.5),
          "aborted with the shot at 0.5");
    Pull(instance, 1);
    Update(instance, 0.25f);
    CHECK(State(instance, &g, 1)->time == 0.0, "fired at weight 0: the shot from its start");
}

// Abort stops at once and wins over fire; a trigger read while idle is
// consumed and does nothing.
static void TestTriggers(void)
{
    maniGraph g;
    maniGraphInstance* instance = Make1(Shot(0.0f, 0.5f), 4, maniGraphLoopRepeat, &g);
    Pull(instance, 2);
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotIdle && instance->cells[2].value.b == 0,
          "a fade-out read while idle: consumed");
    Pull(instance, 1);
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotPlaying && Weight(instance, &g, 2) == 1.0f,
          "a fade-in of 0 plays at once");
    Pull(instance, 1);
    Pull(instance, 3);
    Update(instance, 0.25f);
    CHECK(Phase(instance, &g, 2) == maniGraphShotIdle && Weight(instance, &g, 2) == 0.0f &&
              instance->cells[1].value.b == 0 && instance->cells[3].value.b == 0,
          "abort wins over fire, both consumed");
}

// No parent reaches a one-shot under a select's other input: it keeps
// its trigger.
static void TestUnreached(void)
{
    static const uint32_t kChoice[2] = {1, 2};
    static const uint32_t kInputs[2] = {3, 4};
    Parameters();
    s_parameters[0] = Parameter("speed", maniGraphParameterInt, Int(0));
    maniGraphNodeSource nodes[5] = {{maniGraphNodeSelect, 2, kChoice, .select = {0}},
                                    ClipNode(1, 1.0f, maniGraphLoopRepeat),
                                    {maniGraphNodeOneShot, 2, kInputs, .oneShot = Shot(0.0f, 0.0f)},
                                    ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ClipNode(4, 1.0f, maniGraphLoopClamp)};
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 4, .nodeCount = 5},
        &g);
    Pull(instance, 1);
    Update(instance, 0.25f);
    CHECK(instance->cells[1].value.b == 1 && Phase(instance, &g, 3) == maniGraphShotIdle,
          "unreached: the trigger kept");
    Set(instance, 0, maniGraphParameterInt, Int(1));
    Update(instance, 0.25f);
    CHECK(instance->cells[1].value.b == 0 && Phase(instance, &g, 3) == maniGraphShotPlaying,
          "reached: fired");
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[3][4096];

// Masked to bone 0, half faded in: per bone the shot at half on bone 0
// and nothing elsewhere, as the blending functions give it; a mask of
// half at most hands the shot half its weight.
static void TestMask(void)
{
    static const maniMaskChain kChain = {0, 1.0f, false};
    static const maniGraphMaskNodeSource kMask = {&kChain, 1, 0.0f};
    maniGraphOneShotNodeSource shot = Shot(1.0f, 0.0f);
    shot.mask = &kMask;
    maniGraph g;
    maniGraphInstance* instance = Make1(shot, 4, maniGraphLoopClamp, &g);
    Pull(instance, 1);
    Update(instance, 0.25f);
    Update(instance, 0.5f);
    CHECK(NearF(Weight(instance, &g, 2), 0.5) && NearF(State(instance, &g, 0)->weight, 1.0) &&
              NearF(State(instance, &g, 1)->weight, 0.5),
          "the base whole where the mask is 0, the shot at its weight");
    maniPose* poses[3];
    for (int i = 0; i < 3; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], &poses[i]) ==
                  mani_success,
              "a pose");
    }
    const float inside[3] = {0.5f, 0.0f, 0.0f};
    const float outside[3] = {0.5f, 1.0f, 1.0f};
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, poses[0]) == mani_success &&
              maniSampleClip(&s_clips[4], &s_skeleton, 0.5f, poses[1]) == mani_success &&
              maniSampleClip(&s_clips[0], &s_skeleton, 0.75f, poses[2]) == mani_success,
          "evaluated");
    maniPose* expected = NULL;
    const maniBlendLayer layers[2] = {{poses[1], 1.0f, inside}, {poses[2], 1.0f, outside}};
    static _Alignas(32) uint8_t memory[4096];
    CHECK(maniInitPose(&s_skeleton, memory, sizeof memory, &expected) == mani_success &&
              maniBlendPoses(&s_skeleton, layers, 2, MANI_BLEND_THRESHOLD, expected) ==
                  mani_success,
          "blended");
    bool same = true;
    for (uint32_t bone = 0; bone < 3; ++bone)
    {
        maniTransform a = maniPoseLocal(poses[0], bone);
        maniTransform b = maniPoseLocal(expected, bone);
        same = same && memcmp(&a, &b, sizeof a) == 0;
    }
    CHECK(same, "the one-shot's pose, bit for bit");

    static const maniMaskChain kHalf = {0, 0.5f, false};
    static const maniGraphMaskNodeSource kHalfMask = {&kHalf, 1, 0.0f};
    shot = Shot(0.0f, 0.0f);
    shot.mask = &kHalfMask;
    instance = Make1(shot, 4, maniGraphLoopClamp, &g);
    Pull(instance, 1);
    Update(instance, 0.25f);
    CHECK(Weight(instance, &g, 2) == 1.0f && NearF(State(instance, &g, 1)->weight, 0.5),
          "the shot at its weight times the mask's most");
}

static _Alignas(8) uint8_t s_saved[2][4096];
static _Alignas(8) uint8_t s_other[8192];

// Saved mid-fade and restored: both go on alike; a saved block refused
// at its word.
static void TestSaved(void)
{
    maniGraph g;
    maniGraphInstance* instance = Make1(Shot(1.0f, 0.5f), 4, maniGraphLoopClamp, &g);
    Pull(instance, 1);
    Steps(instance, 2, 0.25f);
    size_t size = maniGraphStateSize(instance->binding);
    maniGraphInstance* other = NULL;
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniInitGraphInstance(instance->binding, s_other, sizeof s_other, &other) ==
                  mani_success &&
              maniRestoreGraphInstance(other, s_saved[0], size).result == mani_success,
          "saved and restored");
    Steps(instance, 2, 0.25f);
    Steps(other, 2, 0.25f);
    CHECK(maniSaveGraphInstance(instance, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveGraphInstance(other, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], size) == 0 && NearF(Weight(other, &g, 2), 0.75),
          "the same after");
    // The block follows the 4 parameters' and 3 nodes' 16 bytes each:
    // phase, elapsed, start, weight (in a fade), remaining (high word).
    size_t block = MANI_GRAPH_STATE_HEADER_SIZE + 7 * MANI_GRAPH_VALUE_SIZE;
    const uint32_t kBad[5][3] = {{0, 4, 0},
                                 {1, 0xFF800000u, 1},
                                 {2, 0x40000000u, 2},
                                 {3, 0x3F800001u, 3},
                                 {5, 0xBFF00000u, 4}};
    for (int k = 0; k < 5; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + block + 4 * kBad[k][0], kBad[k][1]);
        maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == block + 4 * kBad[k][2],
              "a saved block refused at its word");
    }
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + block, maniGraphShotIdle);
    maniFileResult r = maniRestoreGraphInstance(other, s_saved[1], size);
    CHECK(r.result == mani_errorFormat && r.offset == block + 4 * maniGraphShotWeight,
          "idle at a weight above 0 refused");
}

// A one-shot in a subgraph, its fire trigger the outer graph's through
// the map: built as the same graph written flat.
static void TestSubgraph(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    maniGraphParameterSource inner = Parameter("go", maniGraphParameterTrigger, Int(0));
    maniGraphOneShotNodeSource shot = {
        0, MANI_NO_PARAMETER, MANI_NO_PARAMETER, maniGraphFadeSmoothstep, 0.25f, 0.5f, NULL};
    maniGraphNodeSource nodes[3] = {{maniGraphNodeOneShot, 2, kInputs, .oneShot = shot},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(4, 1.0f, maniGraphLoopClip)};
    maniGraphSource sub = {&s_skeleton, &inner, kSlots, NULL, nodes, 1, CLIPS, 0, 3, 0, 0};
    Parameters();
    static const uint32_t kMap[1] = {3};
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&sub, kMap}};
    maniGraphNodeSource flat[3] = {nodes[0], nodes[1], nodes[2]};
    flat[0].oneShot.fire = 3;
    static _Alignas(16) uint8_t files[2][16384];
    size_t sizes[2] = {0, 0};
    const maniGraphSource sources[2] = {
        {&s_skeleton, s_parameters, kSlots, NULL, &node, 4, CLIPS, 0, 1, 0, 0},
        {&s_skeleton, s_parameters, kSlots, NULL, flat, 4, CLIPS, 0, 3, 0, 0}};
    for (int k = 0; k < 2; ++k)
    {
        maniBuilder* builder = NewBuilder();
        CHECK(maniBuildGraph(builder, &sources[k]).result == mani_success &&
                  maniCopyBuilt(builder, files[k], sizeof files[k], &sizes[k]) == mani_success,
              "builds");
        maniDestroyBuilder(builder);
    }
    CHECK(sizes[0] == sizes[1] && memcmp(files[0], files[1], sizes[0]) == 0,
          "a one-shot inlined as written flat");
}

static maniBuildResult Built(maniGraphOneShotNodeSource shot)
{
    static const uint32_t kInputs[2] = {1, 2};
    maniGraphNodeSource nodes[3] = {{maniGraphNodeOneShot, 2, kInputs, .oneShot = shot},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(4, 1.0f, maniGraphLoopClip)};
    Parameters();
    return BuildOnly(nodes, 3, s_parameters, 4);
}

static void TestRefusals(void)
{
    CHECK(Built(Shot(0.5f, 0.5f)).result == mani_success, "a one-shot");
    maniGraphOneShotNodeSource bad[7] = {Shot(0.0f, 0.0f), Shot(0.0f, 0.0f), Shot(0.0f, 0.0f),
                                         Shot(0.0f, 0.0f), Shot(0.0f, 0.0f), Shot(-1.0f, 0.0f),
                                         Shot(0.0f, NAN)};
    bad[0].fire = 0;
    bad[1].fire = MANI_NO_PARAMETER;
    bad[2].fadeOutTrigger = 0;
    bad[3].abort = 4;
    bad[4].curve = maniGraphFadeSmoothstep + 1;
    for (int k = 0; k < 7; ++k)
    {
        CHECK(Refused(Built(bad[k]), maniBuildReasonNode, 0), "a one-shot refused");
    }
    static const maniMaskChain kChain = {1, 2.0f, false};
    static const maniGraphMaskNodeSource kMask = {&kChain, 1, 0.0f};
    maniGraphOneShotNodeSource masked = Shot(0.0f, 0.0f);
    masked.mask = &kMask;
    CHECK(Refused(Built(masked), maniBuildReasonWeight, 0), "a mask's weight past 1");
}

// The one-shot of Make1: its words checked on open. Words: fire 0, out
// 1, abort 2, curve 3, fade-in 4, fade-out 5, mask 6, block 7, count 8,
// the shot's clip 9.
static void TestOpenRefusals(void)
{
    maniGraph g;
    (void)Make1(Shot(0.5f, 0.5f), 4, maniGraphLoopClamp, &g);
    uint64_t at = 0;
    CHECK(Crafted(&g, 2, 9, 0, &at).result == mani_success, "the base's clip listed opens");
    CHECK(Crafted(&g, 2, 1, MANI_NO_PARAMETER, &at).result == mani_success, "no fade-out trigger");
    const uint32_t kBad[9][2] = {{0, MANI_NO_PARAMETER}, {0, 0}, {1, 0}, {2, 4}, {3, 2},
                                 {4, 0xBF800000u},       {6, 0}, {7, 1}, {9, 2}};
    for (int k = 0; k < 9; ++k)
    {
        CHECK(RefusedAtWord(&g, 2, kBad[k][0], kBad[k][1]), "a one-shot's word refused");
    }
    // A shot blending the long step and the run: walk 0, long 1, run 2,
    // blend 3, the one-shot 4; its list 1 and 2 at words 9 and 10.
    static const uint32_t kInputs[2] = {1, 2};
    static const uint32_t kBlend[2] = {3, 4};
    static const maniGraphScalar kWeights[2] = {{MANI_NO_PARAMETER, 1.0f},
                                                {MANI_NO_PARAMETER, 1.0f}};
    Parameters();
    maniGraphNodeSource nodes[5] = {{maniGraphNodeOneShot, 2, kInputs, .oneShot = Shot(0.0f, 0.0f)},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    {maniGraphNodeBlend, 2, kBlend, .blend = {kWeights, 0, 0}},
                                    ClipNode(4, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    (void)Make(
        (maniGraphSource){
            .parameters = s_parameters, .nodes = nodes, .parameterCount = 4, .nodeCount = 5},
        &g);
    CHECK(RefusedAtWord(&g, 4, 10, 1), "a list out of node order");
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

// Every member written and read back.
static void TestJson(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniMaskChain kChain = {1, 0.75f, true};
    static const maniGraphMaskNodeSource kMask = {&kChain, 1, 0.25f};
    maniGraphOneShotNodeSource shot = Shot(0.25f, 0.5f);
    shot.curve = maniGraphFadeSmoothstep;
    shot.mask = &kMask;
    maniGraphNodeSource nodes[3] = {{maniGraphNodeOneShot, 2, kInputs, .oneShot = shot},
                                    ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    ClipNode(4, 1.0f, maniGraphLoopClamp)};
    Parameters();
    maniGraphSource source = {&s_skeleton, s_parameters, kSlots, NULL, nodes, 4, CLIPS, 0, 3, 0, 0};
    CHECK(RoundTrip(&source), "written and read to the same bytes");
    CHECK(strstr(s_text, "\"kind\": \"oneShot\", \"inputs\": [1, 2], \"fire\": \"fire\", "
                         "\"fadeOutTrigger\": \"out\", \"abort\": \"abort\", \"curve\": "
                         "\"smoothstep\", \"fadeIn\": 0.25, \"fadeOut\": 0.5, \"mask\": "
                         "{\"chains\": [{\"bone\": ") != NULL &&
              strstr(s_text, "\"defaultWeight\": 0.25}}") != NULL,
          "the form");
    nodes[0].oneShot = (maniGraphOneShotNodeSource){
        1, MANI_NO_PARAMETER, MANI_NO_PARAMETER, maniGraphFadeLinear, 0.0f, 0.0f, NULL};
    CHECK(RoundTrip(&source) && strstr(s_text, "\"fire\": \"fire\", \"curve\": \"linear\", "
                                               "\"fadeIn\": 0, \"fadeOut\": 0}") != NULL,
          "triggers and mask left out");
}

#define HEAD                                                                                       \
    "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": "               \
    "\"go\", \"type\": \"trigger\"}], \"clips\": [\"walk\", \"run\"], \"nodes\": [{\"kind\": "     \
    "\"oneShot\", \"inputs\": [1, 2]"
#define TAIL                                                                                       \
    "}, {\"kind\": \"clip\", \"clip\": \"walk\"}, {\"kind\": \"clip\", \"clip\": "                 \
    "\"run\"}], \"output\": 0}"

// Members left out, and forms refused at their bytes.
static void TestJsonForms(void)
{
    CHECK(Read(HEAD ", \"fire\": \"go\"" TAIL).result == mani_success, "fire alone");
    static const struct
    {
        const char* text;
        uint32_t form;
        const char* marker;
    } kRefusals[4] = {
        {HEAD TAIL, maniGraphFormMissing, "{\"kind\": \"oneShot"},
        {HEAD ", \"fire\": \"go\", \"curve\": \"cubic\"" TAIL, maniGraphFormValue, "cubic"},
        {HEAD ", \"fire\": \"stop\"" TAIL, maniGraphFormName, "stop"},
        {HEAD ", \"fire\": \"go\", \"mask\": {\"chain\": []}" TAIL, maniGraphFormMember,
         "chain\": []"}};
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
    if (argc == 4 && strcmp(argv[1], "--seed") == 0)
    {
        TestJson();
        bool ok = s_failures == 0 && WriteFile(argv[2], s_built, s_builtSize) &&
                  WriteFile(argv[3], s_text, s_textSize);
        return ok ? 0 : 1;
    }
    TestFire();
    TestRefire();
    TestTriggers();
    TestUnreached();
    TestMask();
    TestSaved();
    TestSubgraph();
    TestRefusals();
    TestOpenRefusals();
    TestJson();
    TestJsonForms();
    return s_failures == 0 ? 0 : 1;
}
