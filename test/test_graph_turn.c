// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Turn limit nodes: over a pose input, the first evaluation
// takes the input as it is; a jump is turned toward by the speed times
// the seconds, about the shorter arc's axis (checked in binary64 against
// the angles), a turn within the budget given bit for bit, a negated
// quaternion taken as no turn, translations and scales passed; a bone at
// mask weight 0 passes and one at 0.5 turns twice as far, a speed
// parameter of 0 or below holds and the largest passes; the reset
// trigger, a seek above and an update that did not reach the node take
// the input again; two updates to an evaluation, one backward, add their
// times; a saved state restores to the same motion and its block is
// checked; the builder's refusals and opening's checks of the words; the
// JSON form written and read back; a subgraph's parameters carried; a
// file of format 4 still opens; inspection gives its share. Run as
// test_graph_turn --seed <graph file> <json file> it writes the graph's
// file and text, seeds of the fuzz targets.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_fixture.h"
#include "graph_instance.h"
#include "graph_turn.h"
#include "vecmath.h"

#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/pose.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct Made
{
    _Alignas(16) uint8_t file[16384];
    _Alignas(8) uint8_t binding[1024];
    _Alignas(8) uint8_t instance[16384];
    maniGraph graph;
    maniGraphBinding* bound;
    maniGraphInstance* live;
    size_t size;
} Made;

static Made s_made[2];
static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poses[3][4096];
static uint8_t s_saved[2][16384];

// Parameters: the body pose, the speed, a reset trigger, a seek's
// trigger, the select's pick (true for the raw pose).
static maniGraphParameterSource s_parameters[5];
static maniGraphNodeSource s_nodes[4];
static const uint32_t kPoseInput[1] = {0};
static const uint32_t kPick[2] = {1, 0};
static const uint32_t kSelect[1] = {2};
// The hips at weight 0.5, the spine at 0, every other bone at 1.
static const maniMaskChain kFreeSpine[2] = {{0, 0.5f, false}, {1, 0.0f, false}};
static const maniGraphMaskNodeSource kMask = {kFreeSpine, 2, 1.0f};

// A pose input, a turn limit over it, a select between them, a seek over
// the select.
static maniGraphSource TurnGraph(maniGraphTurnLimitNodeSource turn)
{
    s_parameters[0] = Parameter("body", maniGraphParameterPose, Int(0));
    s_parameters[1] = Parameter("speed", maniGraphParameterFloat, Float(1.0f));
    s_parameters[2] = Parameter("drop", maniGraphParameterTrigger, Int(0));
    s_parameters[3] = Parameter("jump", maniGraphParameterTrigger, Int(0));
    s_parameters[4] = Parameter("raw", maniGraphParameterBool, Int(0));
    s_nodes[0] = (maniGraphNodeSource){maniGraphNodePose, 0, NULL, .pose = {0}};
    s_nodes[1] = (maniGraphNodeSource){maniGraphNodeTurnLimit, 1, kPoseInput, .turnLimit = turn};
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeSelect, 2, kPick, .select = {4}};
    s_nodes[3] = (maniGraphNodeSource){maniGraphNodeSeek, 1, kSelect, .seek = {3, LITERAL(0.0f)}};
    return (maniGraphSource){.skeleton = &s_skeleton,
                             .parameters = s_parameters,
                             .clips = kSlots,
                             .nodes = s_nodes,
                             .parameterCount = 5,
                             .clipCount = CLIPS,
                             .nodeCount = 4,
                             .output = 3};
}

static const maniGraphTurnLimitNodeSource kTurn = {{1, 0.0f}, 2, 0, NULL};

static maniBuildResult Build(const maniGraphSource* source, Made* m)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, m->file, sizeof m->file, &m->size) == mani_success, "copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static bool Instanced(const maniGraphSource* source, Made* m)
{
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    maniGraphAssets assets = {.skeleton = &s_skeleton, .clips = clips, .clipCount = CLIPS};
    return Build(source, m).result == mani_success &&
           maniOpenGraph(m->file, m->size, &m->graph).result == mani_success &&
           maniBindGraph(&m->graph, &assets, m->binding, sizeof m->binding, &m->bound, NULL) ==
               mani_success &&
           maniInitGraphInstance(m->bound, m->instance, sizeof m->instance, &m->live) ==
               mani_success &&
           maniGraphScratchSize(m->bound) <= sizeof s_scratch;
}

static maniPose* PoseAt(int k)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poses[k], sizeof s_poses[k], &pose) == mani_success,
          "a pose");
    return pose;
}

static maniQuat About(float x, float y, float z, float angle)
{
    float s = sinf(0.5f * angle);
    return (maniQuat){x * s, y * s, z * s, cosf(0.5f * angle)};
}

// The body: the rest pose with the hips and spine turned, the spine moved.
static void Body(Made* m, maniQuat hips, maniQuat spine, float lift)
{
    maniPose* body = PoseAt(2);
    CHECK(maniPoseSetRest(body, &s_skeleton) == mani_success, "the rest");
    maniTransform h = maniPoseLocal(body, 0);
    maniTransform s = maniPoseLocal(body, 1);
    h.rotation = hips;
    s.rotation = spine;
    s.translation.y += lift;
    maniPoseSetLocal(body, 0, h);
    maniPoseSetLocal(body, 1, s);
    CHECK(maniSetGraphPose(m->live, 0, body) == mani_success, "the body set");
}

static bool Evaluated(Made* m, float delta, maniPose* pose)
{
    return maniUpdateGraph(m->live, delta, NULL, 0, NULL, NULL) == mani_success &&
           maniEvaluateGraph(m->live, s_scratch, sizeof s_scratch, pose) == mani_success;
}

static double Angle(maniQuat a, maniQuat b)
{
    double d = (double)a.x * (double)b.x + (double)a.y * (double)b.y + (double)a.z * (double)b.z +
               (double)a.w * (double)b.w;
    d = fabs(d) > 1.0 ? 1.0 : fabs(d);
    return 2.0 * acos(d);
}

static bool Same(maniQuat a, maniQuat b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

static maniQuat RotationOf(const maniPose* pose, uint32_t bone)
{
    return maniPoseLocal(pose, bone).rotation;
}

static void Fire(Made* m, uint32_t trigger)
{
    maniGraphValue on = {{0}};
    on.b = 1;
    CHECK(maniSetGraphParameter(m->live, trigger, maniGraphParameterTrigger, on) == mani_success,
          "fired");
}

static void TestStep(void)
{
    maniGraphSource source = TurnGraph(kTurn);
    CHECK(Instanced(&source, &s_made[0]), "a graph");
    Made* m = &s_made[0];
    maniPose* out = PoseAt(0);
    maniQuat start = About(0.0f, 1.0f, 0.0f, 0.3f);
    Body(m, start, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), start) &&
              Same(RotationOf(out, 1), start),
          "the first evaluation takes its input as it is");
    // A jump of 1.5 radians about x at 1 radian a second over 0.1 s.
    maniQuat target = maniPoseLocal(out, 0).rotation;
    maniQuat jump = About(1.0f, 0.0f, 0.0f, 1.5f);
    maniQuat goal = {jump.w * target.x + jump.x * target.w + jump.y * target.z - jump.z * target.y,
                     jump.w * target.y - jump.x * target.z + jump.y * target.w + jump.z * target.x,
                     jump.w * target.z + jump.x * target.y - jump.y * target.x + jump.z * target.w,
                     jump.w * target.w - jump.x * target.x - jump.y * target.y - jump.z * target.z};
    Body(m, goal, start, 0.25f);
    double total = Angle(start, goal);
    CHECK(Evaluated(m, 0.1f, out), "a frame");
    maniQuat first = RotationOf(out, 0);
    CHECK(fabs(Angle(start, first) - 0.1) < 2e-6 && fabs(Angle(first, goal) - (total - 0.1)) < 2e-6,
          "a jump turned toward by the speed times the seconds, on the arc");
    CHECK(maniPoseLocal(out, 1).translation.y == 0.75f && Same(RotationOf(out, 1), start),
          "a translation passes, an unchanged rotation is given as it is");
    double turned = 0.1;
    bool steady = true;
    for (int i = 0; i < 16; ++i)
    {
        maniQuat before = RotationOf(out, 0);
        CHECK(Evaluated(m, 0.1f, out), "a frame");
        maniQuat after = RotationOf(out, 0);
        turned += 0.1;
        double expect = turned < total ? 0.1 : Angle(before, goal);
        steady = steady && fabs(Angle(before, after) - expect) < 4e-6;
    }
    CHECK(steady && Same(RotationOf(out, 0), goal), "turned on, then the goal given as it is");
    // A turn of 0.05 within the budget of 0.1: the input bit for bit.
    maniQuat near = About(0.0f, 0.0f, 1.0f, 0.05f);
    maniQuat small = {near.w * goal.x + near.x * goal.w + near.y * goal.z - near.z * goal.y,
                      near.w * goal.y - near.x * goal.z + near.y * goal.w + near.z * goal.x,
                      near.w * goal.z + near.x * goal.y - near.y * goal.x + near.z * goal.w,
                      near.w * goal.w - near.x * goal.x - near.y * goal.y - near.z * goal.z};
    Body(m, small, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), small), "within the budget");
    maniQuat negated = {-small.x, -small.y, -small.z, -small.w};
    Body(m, negated, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), negated),
          "the same rotation negated is no turn, given as it is");
    // Two updates to an evaluation, one backward: 0.04 + 0.06 seconds.
    Body(m, start, start, 0.0f);
    CHECK(maniUpdateGraph(m->live, 0.04f, NULL, 0, NULL, NULL) == mani_success &&
              maniUpdateGraph(m->live, -0.06f, NULL, 0, NULL, NULL) == mani_success &&
              maniEvaluateGraph(m->live, s_scratch, sizeof s_scratch, out) == mani_success &&
              fabs(Angle(negated, RotationOf(out, 0)) - 0.1) < 2e-6,
          "the times of two updates added, made positive");
    // A speed of 0, then below 0: held.
    maniQuat held = RotationOf(out, 0);
    Set(m->live, 1, maniGraphParameterFloat, Float(0.0f));
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), held), "a speed of 0 holds");
    Set(m->live, 1, maniGraphParameterFloat, Float(-3.0f));
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), held), "a speed below 0 holds");
    Set(m->live, 1, maniGraphParameterFloat, Float(FLT_MAX));
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), start),
          "the largest speed passes the input");
    Set(m->live, 1, maniGraphParameterFloat, Float(-3.0f));
    // The reset trigger, a seek above, an update that did not reach it;
    // the spine, held at the start, given the goal.
    Body(m, start, goal, 0.0f);
    Fire(m, 2);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), start) &&
              Same(RotationOf(out, 1), goal),
          "the reset takes the input, every bone's");
    Body(m, goal, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && Angle(RotationOf(out, 0), start) < 1e-6,
          "held again (the input kept normalized)");
    Fire(m, 3);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), goal), "a seek takes the input");
    // Held at the goal; the raw pose picked for a frame, the body moved to
    // the start; back over the node, the start as it is, not turned from
    // the goal.
    Body(m, start, start, 0.0f);
    Set(m->live, 4, maniGraphParameterBool, Int(1));
    maniGraphNodeView view;
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), start) &&
              maniInspectGraphNode(m->live, 1, &view) == mani_success &&
              view.kind == maniGraphNodeTurnLimit && view.weight == 0.0f,
          "the raw pose picked, the node of no share");
    Set(m->live, 4, maniGraphParameterBool, Int(0));
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 0), start) &&
              maniInspectGraphNode(m->live, 1, &view) == mani_success && view.weight == 1.0f,
          "back from an update that did not reach it: the input as it is");
    // A turn of 0.15 at 1 radian a second over 0.1 s: turned by 0.1, the
    // whole angle against the budget (its half is within it).
    Set(m->live, 1, maniGraphParameterFloat, Float(1.0f));
    Body(m, maniQuatMul(About(0.0f, 0.0f, 1.0f, 0.15f), start), start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && fabs(Angle(start, RotationOf(out, 0)) - 0.1) < 2e-6,
          "a turn past the budget, not by half");
    // The identity held (its x, y and z all 0) is held all the same.
    Body(m, About(1.0f, 0.0f, 0.0f, 0.0f), start, 0.0f);
    Fire(m, 2);
    CHECK(Evaluated(m, 0.1f, out), "the identity taken");
    Body(m, About(1.0f, 0.0f, 0.0f, 1.0f), start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) &&
              fabs(Angle(About(1.0f, 0.0f, 0.0f, 0.0f), RotationOf(out, 0)) - 0.1) < 2e-6,
          "turned from the identity it held");
    // An input of length 2, given as it is, held normalized.
    Body(m, (maniQuat){2.0f * start.x, 2.0f * start.y, 2.0f * start.z, 2.0f * start.w}, start,
         0.0f);
    Fire(m, 2);
    CHECK(Evaluated(m, 0.1f, out), "a long input taken");
    Set(m->live, 1, maniGraphParameterFloat, Float(0.0f));
    Body(m, goal, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out), "held at a speed of 0");
    maniQuat kept = RotationOf(out, 0);
    double length = sqrt((double)kept.x * (double)kept.x + (double)kept.y * (double)kept.y +
                         (double)kept.z * (double)kept.z + (double)kept.w * (double)kept.w);
    CHECK(fabs(length - 1.0) < 1e-6 && Angle(kept, start) < 1e-6, "a long input held normalized");
}

static void TestMask(void)
{
    maniGraphTurnLimitNodeSource turn = kTurn;
    turn.mask = &kMask;
    maniGraphSource source = TurnGraph(turn);
    CHECK(Instanced(&source, &s_made[0]), "a graph");
    Made* m = &s_made[0];
    maniPose* out = PoseAt(0);
    maniQuat start = About(0.0f, 1.0f, 0.0f, 0.3f);
    maniQuat goal = About(0.0f, 0.0f, 1.0f, 1.0f);
    Body(m, start, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out), "a frame");
    Body(m, goal, goal, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 1), goal) &&
              fabs(Angle(start, RotationOf(out, 0)) - 0.2) < 2e-6,
          "the spine at weight 0 passes, the hips at 0.5 turn twice as far");
    Set(m->live, 1, maniGraphParameterFloat, Float(0.0f));
    Body(m, start, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out) && Same(RotationOf(out, 1), start),
          "the spine at weight 0 passes at a speed of 0");
}

static _Alignas(8) uint8_t s_plan[4096];

// A subset of the hips: the spine's held rotation waits for the next full
// evaluation, which turns it by that update's budget alone.
static void TestSubset(void)
{
    maniGraphSource source = TurnGraph(kTurn);
    CHECK(Instanced(&source, &s_made[0]), "a graph");
    Made* m = &s_made[0];
    maniPose* out = PoseAt(0);
    maniQuat start = About(0.0f, 1.0f, 0.0f, 0.3f);
    maniQuat goal = About(0.0f, 0.0f, 1.0f, 1.2f);
    Body(m, start, start, 0.0f);
    CHECK(Evaluated(m, 0.1f, out), "a frame");
    const uint32_t hips[1] = {0};
    maniGraphSubset* plan = NULL;
    CHECK(maniGraphSubsetSize(m->bound) <= sizeof s_plan &&
              maniPlanGraphSubset(m->bound, hips, 1, s_plan, sizeof s_plan, &plan) == mani_success,
          "a plan of the hips");
    Body(m, goal, goal, 0.0f);
    CHECK(maniUpdateGraph(m->live, 0.1f, NULL, 0, NULL, NULL) == mani_success &&
              maniEvaluateGraphSubset(m->live, plan, s_scratch, sizeof s_scratch, out) ==
                  mani_success &&
              fabs(Angle(start, RotationOf(out, 0)) - 0.1) < 2e-6,
          "the hips turned under the plan");
    CHECK(Evaluated(m, 0.1f, out) && fabs(Angle(start, RotationOf(out, 0)) - 0.2) < 4e-6 &&
              fabs(Angle(start, RotationOf(out, 1)) - 0.1) < 2e-6,
          "the spine turns from what it last had by this update's budget");
}

static void TestSaved(void)
{
    maniGraphSource source = TurnGraph(kTurn);
    CHECK(Instanced(&source, &s_made[0]) && Instanced(&source, &s_made[1]), "two instances");
    maniPose* a = PoseAt(0);
    maniPose* b = PoseAt(1);
    for (uint32_t i = 0; i < 6; ++i)
    {
        Body(&s_made[0], About(0.0f, 1.0f, 0.0f, 0.4f * (float)i), About(1.0f, 0.0f, 0.0f, 0.1f),
             0.0f);
        CHECK(Evaluated(&s_made[0], 0.05f, a), "a frame");
    }
    size_t size = maniGraphStateSize(s_made[0].bound);
    CHECK(size <= sizeof s_saved[0] &&
              maniSaveGraphInstance(s_made[0].live, s_saved[0], size) == mani_success &&
              maniRestoreGraphInstance(s_made[1].live, s_saved[0], size).result == mani_success,
          "saved and restored");
    bool same = true;
    for (uint32_t i = 6; i < 12; ++i)
    {
        maniQuat hips = About(0.0f, 1.0f, 0.0f, 0.4f * (float)i);
        maniQuat spine = About(1.0f, 0.0f, 0.0f, 0.1f);
        Body(&s_made[0], hips, spine, 0.0f);
        Body(&s_made[1], hips, spine, 0.0f);
        CHECK(Evaluated(&s_made[0], 0.05f, a) && Evaluated(&s_made[1], 0.05f, b), "a frame");
        same = same && Same(RotationOf(a, 0), RotationOf(b, 0));
    }
    CHECK(same, "a restored instance turns as the saved one");
    const maniGraph* g = &s_made[0].graph;
    uint32_t data = maniLoadU32(g->nodes + 1 * (size_t)MANI_GRAPH_NODE_SIZE + 12);
    uint32_t first = maniLoadU32(g->data + 4 * (size_t)(data + maniGraphTurnBlock));
    uint64_t block = MANI_GRAPH_STATE_HEADER_SIZE +
                     (uint64_t)(g->parameterCount + g->nodeCount) * MANI_GRAPH_VALUE_SIZE +
                     4 * (uint64_t)first;
    // A time below 0, a rotation of length 2, one not finite.
    const uint64_t at[3] = {block + 4, block + 8, block + 12};
    const uint32_t words[3] = {0xBFF00000u, 0x3FB504F3u, 0x7FC00000u};
    for (int k = 0; k < 3; ++k)
    {
        memcpy(s_saved[1], s_saved[0], size);
        maniStoreU32(s_saved[1] + at[k], words[k]);
        if (k == 1)
        {
            for (int c = 0; c < 4; ++c)
            {
                maniStoreU32(s_saved[1] + at[k] + 4 * (uint64_t)c, 0x3FB504F3u);
            }
        }
        maniFileResult r = maniRestoreGraphInstance(s_made[1].live, s_saved[1], size);
        CHECK(r.result == mani_errorFormat && r.offset == (k == 0 ? block : at[k]),
              "a turn block checked");
    }
    memcpy(s_saved[1], s_saved[0], size);
    for (int c = 0; c < 4; ++c)
    {
        maniStoreU32(s_saved[1] + block + 8 + 4 * (uint64_t)c, 0);
    }
    CHECK(maniRestoreGraphInstance(s_made[1].live, s_saved[1], size).result == mani_success,
          "a bone held nothing for: all zero");
}

static bool RefusedFor(const maniGraphSource* source, maniBuildReason reason, uint32_t item)
{
    maniBuildResult r = Build(source, &s_made[0]);
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item;
}

static void TestRefusals(void)
{
    maniGraphTurnLimitNodeSource t = kTurn;
    t.speed = LITERAL(-1.0f);
    maniGraphSource source = TurnGraph(t);
    CHECK(RefusedFor(&source, maniBuildReasonScalar, 1), "a speed below 0");
    t.speed = LITERAL(INFINITY);
    source = TurnGraph(t);
    CHECK(RefusedFor(&source, maniBuildReasonScalar, 1), "a speed not finite");
    t = kTurn;
    t.speed.parameter = 4;
    source = TurnGraph(t);
    CHECK(RefusedFor(&source, maniBuildReasonScalar, 1), "a speed parameter not float");
    t = kTurn;
    t.reset = 1;
    source = TurnGraph(t);
    CHECK(RefusedFor(&source, maniBuildReasonNode, 1), "a reset not a trigger");
    static const maniMaskChain heavy[1] = {{1, 2.0f, false}};
    static const maniGraphMaskNodeSource heavyMask = {heavy, 1, 1.0f};
    t = kTurn;
    t.mask = &heavyMask;
    source = TurnGraph(t);
    CHECK(RefusedFor(&source, maniBuildReasonWeight, 1), "a mask weight past 1");
    source = TurnGraph(kTurn);
    s_nodes[1].inputCount = 2;
    s_nodes[1].inputs = kPick;
    CHECK(Build(&source, &s_made[0]).result == mani_errorInvalid, "two inputs refused");
}

static void TestOpen(void)
{
    maniGraphTurnLimitNodeSource turn = kTurn;
    turn.mask = &kMask;
    maniGraphSource source = TurnGraph(turn);
    CHECK(Build(&source, &s_made[0]).result == mani_success, "built");
    maniGraph g;
    CHECK(maniOpenGraph(s_made[0].file, s_made[0].size, &g).result == mani_success, "opened");
    uint32_t data = maniLoadU32(g.nodes + 1 * (size_t)MANI_GRAPH_NODE_SIZE + 12);
    uint64_t at = (uint64_t)(g.data - s_made[0].file) + 4 * (uint64_t)data;
    // A mask past the masks, a speed parameter with a literal, a literal
    // below 0, a reset not a trigger, a block not its own, a literal
    // infinite, a speed parameter past the parameters.
    const uint32_t place[7] = {0, 2, 2, 3, 4, 2, 1};
    const uint32_t value[7] = {1, 1, 0xBF800000u, 1, 7, 0x7F800000u, 1000};
    for (int k = 0; k < 7; ++k)
    {
        static _Alignas(16) uint8_t copy[16384];
        memcpy(copy, s_made[0].file, s_made[0].size);
        if (k == 2 || k == 5)
        {
            maniStoreU32(copy + at + 4, MANI_NO_PARAMETER);
        }
        maniStoreU32(copy + at + 4 * (uint64_t)place[k], value[k]);
        maniStoreU64(copy + 32, maniHashBytes(MANI_HASH_INIT, copy + 64, s_made[0].size - 64));
        maniFileResult r = maniOpenGraph(copy, s_made[0].size, &g);
        uint64_t expect = at + 4 * (uint64_t)(place[k] == 2 ? 1 : place[k]);
        CHECK(r.result == mani_errorFormat && r.offset == expect, "a turn node's words checked");
    }
}

static char s_text[16384];

static void TestJson(void)
{
    maniGraphTurnLimitNodeSource turn = kTurn;
    turn.mask = &kMask;
    maniGraphSource source = TurnGraph(turn);
    maniGraph g;
    CHECK(Build(&source, &s_made[0]).result == mani_success &&
              maniOpenGraph(s_made[0].file, s_made[0].size, &g).result == mani_success,
          "opened");
    size_t size = 0;
    maniBuilder* builder = NewBuilder();
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text, "{\"kind\": \"turnLimit\", \"inputs\": [0], \"speed\": {\"parameter\": "
                         "\"speed\"}, \"reset\": \"drop\", \"mask\": {\"chains\": [{\"bone\": "
                         "\"hips\", \"weight\": 0.5") != NULL,
          "a turn limit node by its parameters' names, its mask");
    maniGraphNamedAssets assets = {.skeleton = &s_skeleton, .clips = kSlots, .clipCount = CLIPS};
    maniGraph read;
    CHECK(maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success &&
              maniCopyBuilt(builder, s_made[1].file, sizeof s_made[1].file, &size) ==
                  mani_success &&
              maniOpenGraph(s_made[1].file, size, &read).result == mani_success &&
              read.identity == g.identity,
          "read back to the same graph");
    source = TurnGraph((maniGraphTurnLimitNodeSource){LITERAL(2.5f), MANI_NO_PARAMETER, 0, NULL});
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text - 1, &size) == mani_success,
          "written");
    s_text[size] = 0;
    CHECK(strstr(s_text, "\"inputs\": [0], \"speed\": 2.5}") != NULL &&
              maniBuildGraphJson(builder, s_text, size, &assets).result == mani_success,
          "a literal speed, no reset or mask");
    maniDestroyBuilder(builder);
}

// The turn graph as a subgraph whose parameters map to the outer's in
// reverse: its speed and reset the outer's.
static void TestSubgraph(void)
{
    maniGraphSource inner = TurnGraph(kTurn);
    static const uint32_t kMap[5] = {4, 3, 2, 1, 0};
    maniGraphParameterSource outerParameters[5];
    for (uint32_t k = 0; k < 5; ++k)
    {
        outerParameters[k] = s_parameters[4 - k];
    }
    maniGraphNodeSource node = {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                .subgraph = {&inner, kMap}};
    maniGraphSource outer = {.skeleton = &s_skeleton,
                             .parameters = outerParameters,
                             .nodes = &node,
                             .parameterCount = 5,
                             .nodeCount = 1};
    maniGraph g;
    CHECK(Build(&outer, &s_made[0]).result == mani_success &&
              maniOpenGraph(s_made[0].file, s_made[0].size, &g).result == mani_success,
          "built");
    uint32_t p = 0;
    while (maniLoadU16(g.nodes + (size_t)p * MANI_GRAPH_NODE_SIZE) != maniGraphNodeTurnLimit)
    {
        ++p;
    }
    uint32_t data = maniLoadU32(g.nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 12);
    CHECK(maniLoadU32(g.data + 4 * (size_t)(data + maniGraphTurnSpeed)) == 3 &&
              maniLoadU32(g.data + 4 * (size_t)(data + maniGraphTurnReset)) == 2,
          "its speed and reset the outer's");
}

// A file of the format before the turn limit node opens as it did.
static void TestOlder(void)
{
    maniGraphSource source = TurnGraph(kTurn);
    source.nodeCount = 1;
    source.output = 0;
    CHECK(Build(&source, &s_made[0]).result == mani_success, "a graph of a pose input alone");
    maniStoreU16(s_made[0].file + 6, 4);
    maniGraph g;
    CHECK(maniOpenGraph(s_made[0].file, s_made[0].size, &g).result == mani_success,
          "a format 4 file opens");
    maniStoreU16(s_made[0].file + 6, (uint16_t)(MANI_GRAPH_FORMAT_VERSION + 1));
    maniFileResult r = maniOpenGraph(s_made[0].file, s_made[0].size, &g);
    CHECK(r.result == mani_errorVersion && r.offset == 6, "a newer format refused by version");
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

// The masked turn graph's file and JSON text.
static bool Seed(const char* graphPath, const char* textPath)
{
    maniGraphTurnLimitNodeSource turn = kTurn;
    turn.mask = &kMask;
    maniGraphSource source = TurnGraph(turn);
    size_t length = 0;
    maniBuilder* builder = NewBuilder();
    bool ok = Build(&source, &s_made[0]).result == mani_success &&
              maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success &&
              WriteFile(graphPath, s_made[0].file, s_made[0].size) &&
              WriteFile(textPath, s_text, length);
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
    TestStep();
    TestMask();
    TestSubset();
    TestSaved();
    TestRefusals();
    TestOpen();
    TestJson();
    TestSubgraph();
    TestOlder();
    return s_failures == 0 ? 0 : 1;
}
