// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph updates in detail: a node reached twice takes its
// time factor from its first parent; blend weights clamped, normalized
// unless direct, capped when direct, zero when all are; additive weights
// capped; mask shares from the bones' largest weights and the root
// motion bone's; a select clamped to its inputs; time scales multiplied;
// loops overridden, clamped and wrapped both ways; a sync group's first
// equal member leading, a declared leader kept; an unreached follower in
// step, an unreached clip in no group not moved; events and root motion by
// their own weights; marker sync's edges (a marker at the step's end,
// the nearest match, names walked to); evaluated blends normalized,
// capped when direct, a layer of no weight choosing nothing; evaluation
// after a restore that left nodes unweighed; a select's choice
// evaluated; a bounded float clamped up.

#include "graph_fixture.h"

// A walk read by a blend and by a time scale of 2 under the same blend:
// its weights summed, its time factor the blend's (its first parent, the
// later in node order), not the time scale's.
static void TestFirstParent(void)
{
    static const uint32_t kBlend[2] = {1, 2};
    static const uint32_t kScale[1] = {2};
    static const maniGraphScalar kWeights[2] = {LITERAL(1.0f), LITERAL(1.0f)};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeBlend, 2, kBlend, .blend = {kWeights, 0}},
        {maniGraphNodeTimeScale, 1, kScale, .timeScale = {LITERAL(2.0f)}},
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    // The walk 0, the time scale 1, the blend 2.
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 3}, &g);
    Update(instance, 0.25f);
    CHECK(NearF(State(instance, &g, 0)->weight, 1.0) && Near(State(instance, &g, 0)->time, 0.25),
          "weights summed, the first parent's factor");
}

// A blend of the walk and the run by parameters a and b.
static maniGraphInstance* BlendOf(bool direct, maniGraph* g)
{
    static const uint32_t kInputs[2] = {1, 2};
    static maniGraphScalar weights[2];
    static maniGraphParameterSource parameters[2];
    parameters[0] = Parameter("a", maniGraphParameterFloat, Float(1.0f));
    parameters[1] = Parameter("b", maniGraphParameterFloat, Float(1.0f));
    weights[0] = (maniGraphScalar){0, 0.0f};
    weights[1] = direct ? LITERAL(0.5f) : (maniGraphScalar){1, 0.0f};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeBlend, 2, kInputs, .blend = {weights, direct ? 1u : 0u}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    return Make(
        (maniGraphSource){
            .parameters = parameters, .nodes = nodes, .parameterCount = 2, .nodeCount = 3},
        g);
}

static void TestBlendWeights(void)
{
    maniGraph g;
    // The walk 0, the run 1, the blend 2.
    maniGraphInstance* instance = BlendOf(false, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(-1.0f));
    Update(instance, 0.0f);
    CHECK(State(instance, &g, 0)->weight == 0.0f && NearF(State(instance, &g, 1)->weight, 1.0),
          "a negative weight taken as 0, the rest normalized");
    Set(instance, 0, maniGraphParameterFloat, Float(0.0f));
    Set(instance, 1, maniGraphParameterFloat, Float(0.0f));
    Update(instance, 0.0f);
    CHECK(State(instance, &g, 0)->weight == 0.0f && State(instance, &g, 1)->weight == 0.0f,
          "all weights 0: every input 0, none NaN");
    instance = BlendOf(true, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(3.0f));
    Update(instance, 0.0f);
    CHECK(NearF(State(instance, &g, 0)->weight, 1.0) && NearF(State(instance, &g, 1)->weight, 0.5),
          "direct: capped at 1, not normalized");
}

static void TestAdditiveCap(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniGraphScalar kWeights[1] = {{0, 0.0f}};
    maniGraphParameterSource a = Parameter("a", maniGraphParameterFloat, Float(2.0f));
    maniGraphNodeSource nodes[3] = {{maniGraphNodeAdditive, 2, kInputs, .additive = {kWeights}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(2, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    // The walk 0, the wave 1, the additive node 2.
    maniGraphInstance* instance = Make(
        (maniGraphSource){.parameters = &a, .nodes = nodes, .parameterCount = 1, .nodeCount = 3},
        &g);
    Update(instance, 0.0f);
    CHECK(NearF(State(instance, &g, 1)->weight, 1.0) && State(instance, &g, 1)->rootWeight == 0.0f,
          "a layer's weight capped at 1, no root motion");
}

// A mask of the walk on one bone, the run elsewhere.
static maniGraphInstance* MaskOn(uint32_t bone, maniGraph* g)
{
    static const uint32_t kInputs[2] = {1, 2};
    static maniMaskChain chain;
    chain = (maniMaskChain){bone, 1.0f, false};
    maniGraphNodeSource nodes[3] = {{maniGraphNodeMask, 2, kInputs, .mask = {&chain, 1, 0.0f}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    return Make((maniGraphSource){.nodes = nodes, .nodeCount = 3}, g);
}

static void TestMaskShares(void)
{
    maniGraph g;
    // The walk 0, the run 1, the mask 2; the spine alone inside.
    maniGraphInstance* instance = MaskOn(1, &g);
    Update(instance, 0.0f);
    CHECK(NearF(State(instance, &g, 0)->weight, 1.0) && NearF(State(instance, &g, 1)->weight, 1.0),
          "each side weighed by its largest bone weight");
    CHECK(State(instance, &g, 0)->rootWeight == 0.0f && State(instance, &g, 1)->rootWeight == 1.0f,
          "the hips outside: root motion from the run's side");
    instance = MaskOn(0, &g);
    Update(instance, 0.0f);
    CHECK(State(instance, &g, 0)->rootWeight == 1.0f && State(instance, &g, 1)->rootWeight == 0.0f,
          "the hips inside: root motion from the walk's side");
}

static void TestSelectClamp(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    maniGraphParameterSource n = Parameter("n", maniGraphParameterInt, Int(7));
    maniGraphNodeSource nodes[3] = {{maniGraphNodeSelect, 2, kInputs, .select = {0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    // The walk 0, the run 1, the select 2.
    maniGraphInstance* instance = Make(
        (maniGraphSource){.parameters = &n, .nodes = nodes, .parameterCount = 1, .nodeCount = 3},
        &g);
    Update(instance, 0.0f);
    CHECK(State(instance, &g, 2)->word == 1 && State(instance, &g, 1)->weight == 1.0f,
          "past the last input: the last");
    Set(instance, 0, maniGraphParameterInt, Int(-4));
    Update(instance, 0.0f);
    CHECK(State(instance, &g, 2)->word == 0 && State(instance, &g, 0)->weight == 1.0f,
          "below 0: the first");
}

static void TestNestedScales(void)
{
    static const uint32_t kOuter[1] = {1};
    static const uint32_t kInner[1] = {2};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeTimeScale, 1, kOuter, .timeScale = {LITERAL(2.0f)}},
        {maniGraphNodeTimeScale, 1, kInner, .timeScale = {LITERAL(3.0f)}},
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    // The walk 0, the inner scale 1, the outer 2.
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 3}, &g);
    Update(instance, 0.1f);
    CHECK(Near(State(instance, &g, 0)->time, 0.6), "time scales multiplied");
}

// The walk clamped forward and backward, the run repeating backward.
static void TestLoops(void)
{
    static const uint32_t kInputs[3] = {1, 2, 3};
    static const maniGraphScalar kWeights[3] = {LITERAL(1.0f), LITERAL(1.0f), LITERAL(1.0f)};
    maniGraphNodeSource nodes[4] = {{maniGraphNodeBlend, 3, kInputs, .blend = {kWeights, 0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClamp),
                                    ClipNode(1, -1.0f, maniGraphLoopClip),
                                    ClipNode(0, -1.0f, maniGraphLoopClamp)};
    maniGraph g;
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 4}, &g);
    Update(instance, 1.5f);
    CHECK(Near(State(instance, &g, 0)->time, 1.0),
          "clamped at the end, the clip's repeat overridden");
    CHECK(Near(State(instance, &g, 1)->time, 0.5), "repeating backward: wrapped below 0");
    CHECK(State(instance, &g, 2)->time == 0.0, "clamped backward at 0");
}

// The walk and the run in one group at equal weights: the first leads.
static void TestLeaderTie(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniGraphScalar kWeights[2] = {LITERAL(1.0f), LITERAL(1.0f)};
    static const maniGraphSyncGroupSource kGroup = {"feet", 4, 0};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0}},
        {maniGraphNodeClip, 0, NULL, .clip = {0, 0, LITERAL(1.0f), maniGraphLoopClip, 0}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, 0, LITERAL(1.0f), maniGraphLoopClip, 0}}};
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .syncGroups = &kGroup, .nodes = nodes, .syncGroupCount = 1, .nodeCount = 3},
        &g);
    Update(instance, 0.1f);
    CHECK(instance->cells[g.parameterCount + g.nodeCount].group.leader == 0,
          "the first of equal members");
}

// A declared leader stays the leader though a later member weighs more.
static void TestDeclaredKept(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniGraphScalar kWeights[2] = {LITERAL(0.2f), LITERAL(0.8f)};
    static const maniGraphSyncGroupSource kGroup = {"feet", 4, 0};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0}},
        {maniGraphNodeClip, 0, NULL,
         .clip = {0, 0, LITERAL(1.0f), maniGraphLoopClip, maniGraphClipLeader}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, 0, LITERAL(1.0f), maniGraphLoopClip, 0}}};
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){
            .syncGroups = &kGroup, .nodes = nodes, .syncGroupCount = 1, .nodeCount = 3},
        &g);
    Update(instance, 0.1f);
    CHECK(instance->cells[g.parameterCount + g.nodeCount].group.leader == 0,
          "the declared leader kept");
}

// A sync group's follower that no parent reaches (the select's other
// input) still keeps pace with its leader; one in no group stays put.
static void TestUnreachedFollower(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniGraphSyncGroupSource kGroup = {"feet", 4, 0};
    maniGraphParameterSource n = Parameter("n", maniGraphParameterInt, Int(0));
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeSelect, 2, kInputs, .select = {0}},
        {maniGraphNodeClip, 0, NULL,
         .clip = {0, 0, LITERAL(1.0f), maniGraphLoopClip, maniGraphClipLeader}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, 0, LITERAL(1.0f), maniGraphLoopClip, 0}}};
    maniGraph g;
    // The walk 0 (leading), the run 1, the select 2.
    maniGraphInstance* instance = Make((maniGraphSource){.parameters = &n,
                                                         .syncGroups = &kGroup,
                                                         .nodes = nodes,
                                                         .parameterCount = 1,
                                                         .syncGroupCount = 1,
                                                         .nodeCount = 3},
                                       &g);
    Update(instance, 0.25f);
    CHECK(State(instance, &g, 1)->weight == 0.0f && Near(State(instance, &g, 1)->time, 0.25),
          "unreached, in step with its leader");
    nodes[2].clip.syncGroup = MANI_NO_SYNC_GROUP;
    instance = Make((maniGraphSource){.parameters = &n,
                                      .syncGroups = &kGroup,
                                      .nodes = nodes,
                                      .parameterCount = 1,
                                      .syncGroupCount = 1,
                                      .nodeCount = 3},
                    &g);
    Update(instance, 0.25f);
    CHECK(State(instance, &g, 1)->time == 0.0, "unreached in no group: not moved");
}

// A walk of no weight lists no events; root motion follows the root
// weight, not the weight (the hips outside a mask on the spine).
static void TestOwnWeights(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniGraphScalar kWeights[2] = {LITERAL(0.0f), LITERAL(1.0f)};
    maniGraphNodeSource nodes[3] = {{maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 3}, &g);
    maniGraphEvent events[4];
    uint32_t count = 0;
    CHECK(maniUpdateGraph(instance, 0.6f, events, 4, &count, NULL) == mani_success && count == 0,
          "no events from a node of no weight");
    // The walk inside a mask of the hips at 0.25 and the spine at 1: its
    // weight 1, its root weight 0.25 (the run's 0.75, without motion).
    static const uint32_t kMaskInputs[2] = {1, 2};
    static const maniMaskChain kChains[2] = {{0, 0.25f, false}, {1, 1.0f, false}};
    nodes[0] = (maniGraphNodeSource){maniGraphNodeMask, 2, kMaskInputs, .mask = {kChains, 2, 0.0f}};
    instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 3}, &g);
    maniRootMove move;
    CHECK(maniUpdateGraph(instance, 0.5f, NULL, 0, NULL, &move) == mani_success &&
              NearF(State(instance, &g, 0)->weight, 1.0) && Near((double)move.translation.x, 0.125),
          "root motion by the root weight");
}

// The short clip (3) leading a member of one group, every member's
// events firing or not, the member's loop as given.
static maniGraphInstance* Synced(uint32_t member, uint32_t groupFlags, maniGraphLoop loop,
                                 maniGraph* g)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniGraphScalar kWeights[2] = {LITERAL(0.5f), LITERAL(0.5f)};
    static maniGraphSyncGroupSource group;
    group = (maniGraphSyncGroupSource){"feet", 4, groupFlags};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0}},
        {maniGraphNodeClip, 0, NULL,
         .clip = {3, 0, LITERAL(1.0f), maniGraphLoopClip, maniGraphClipLeader}},
        {maniGraphNodeClip, 0, NULL, .clip = {member, 0, LITERAL(1.0f), loop, 0}}};
    // The leader 0, the member 1, the blend 2.
    return Make(
        (maniGraphSource){
            .syncGroups = &group, .nodes = nodes, .syncGroupCount = 1, .nodeCount = 3},
        g);
}

// The leader lands on its right foot: the left passed, the right still
// ahead; the long clip moves a step, its middle not passed.
static void TestMarkerStepEnd(void)
{
    maniGraph g;
    maniGraphInstance* instance = Synced(4, maniGraphSyncAllEvents, maniGraphLoopClip, &g);
    uint32_t count = 0;
    CHECK(maniUpdateGraph(instance, 0.5f, NULL, 0, &count, NULL) == mani_success &&
              Near(State(instance, &g, 1)->time, 0.5) && count == 0,
          "a marker at the step's end is not passed");
}

// A member that does not repeat: phase sync, not markers.
static void TestMarkerPhaseFallback(void)
{
    maniGraph g;
    maniGraphInstance* instance = Synced(4, 0, maniGraphLoopClamp, &g);
    Update(instance, 0.25f);
    CHECK(Near(State(instance, &g, 1)->time, 0.5), "by phase when a member does not repeat");
}

// Without allEvents the follower's middle passes silently.
static void TestMarkerFollowerSilent(void)
{
    maniGraph g;
    maniGraphInstance* instance = Synced(4, 0, maniGraphLoopClip, &g);
    uint32_t total = 0;
    static const float kSteps[3] = {0.25f, 0.5f, 0.25f};
    for (int k = 0; k < 3; ++k)
    {
        uint32_t count = 0;
        CHECK(maniUpdateGraph(instance, kSteps[k], NULL, 0, &count, NULL) == mani_success, "step");
        total += count;
    }
    CHECK(Near(State(instance, &g, 1)->time, 1.5) && total == 0, "a follower's events silent");
}

// A seek above the leader alone: the follower set to its phase, not
// matched by markers.
static void TestLeaderSeek(void)
{
    static const uint32_t kInputs[2] = {1, 3};
    static const uint32_t kSeek[1] = {2};
    static const maniGraphScalar kWeights[2] = {LITERAL(0.5f), LITERAL(0.5f)};
    static const maniGraphSyncGroupSource kGroup = {"feet", 4, 0};
    maniGraphParameterSource t = Parameter("t", maniGraphParameterTrigger, Int(1));
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0}},
        {maniGraphNodeSeek, 1, kSeek, .seek = {0, LITERAL(0.5f)}},
        {maniGraphNodeClip, 0, NULL,
         .clip = {3, 0, LITERAL(1.0f), maniGraphLoopClip, maniGraphClipLeader}},
        {maniGraphNodeClip, 0, NULL, .clip = {4, 0, LITERAL(1.0f), maniGraphLoopClip, 0}}};
    maniGraph g;
    // The leader 0, the seek 1, the follower 2, the blend 3.
    maniGraphInstance* instance = Make((maniGraphSource){.parameters = &t,
                                                         .syncGroups = &kGroup,
                                                         .nodes = nodes,
                                                         .parameterCount = 1,
                                                         .syncGroupCount = 1,
                                                         .nodeCount = 4},
                                       &g);
    Update(instance, 0.1f);
    CHECK(Near(State(instance, &g, 0)->time, 0.5) && Near(State(instance, &g, 2)->time, 1.0),
          "the follower at the leader's phase");
}

// Two steps in two seconds: the follower placed at the match nearest its
// playhead, not the first.
static void TestMarkerNearest(void)
{
    maniGraph g;
    maniGraphInstance* instance = Synced(5, 0, maniGraphLoopClip, &g);
    Update(instance, 0.5f);
    CHECK(Near(State(instance, &g, 1)->time, 0.5), "on the first right foot");
    Update(instance, 0.5f);
    CHECK(Near(State(instance, &g, 1)->time, 1.0), "on to the nearer left foot");
}

// Two lefts before a right: past the leader's one marker, the follower
// walks on to the marker pair that carries the leader's names.
static void TestMarkerNames(void)
{
    maniGraph g;
    maniGraphInstance* instance = Synced(6, 0, maniGraphLoopClip, &g);
    Update(instance, 0.25f);
    CHECK(Near(State(instance, &g, 1)->time, 0.75), "between the second left and the right");
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

static bool SameBits(const maniPose* a, const maniPose* b)
{
    bool same = true;
    for (uint32_t bone = 0; bone < 3; ++bone)
    {
        maniTransform x = maniPoseLocal(a, bone);
        maniTransform y = maniPoseLocal(b, bone);
        same = same && memcmp(&x, &y, sizeof x) == 0;
    }
    return same;
}

// The graph's pose, and the clips' poses blended by hand: layers of the
// clips' samples at time t, in input order.
static bool Matches(maniGraphInstance* instance, const uint32_t* clips, const float* weights,
                    uint32_t count, float t)
{
    maniPose* out = PoseAt(0);
    maniPose* want = PoseAt(1);
    maniBlendLayer layers[2];
    for (uint32_t k = 0; k < count; ++k)
    {
        maniPose* pose = PoseAt(2 + (int)k);
        CHECK(maniSampleClip(&s_clips[clips[k]], &s_skeleton, t, pose) == mani_success, "sample");
        layers[k] = (maniBlendLayer){pose, weights[k], NULL};
    }
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              maniBlendPoses(&s_skeleton, layers, count, MANI_BLEND_THRESHOLD, want) ==
                  mani_success,
          "evaluates");
    return SameBits(out, want);
}

static void TestEvaluatedBlends(void)
{
    maniGraph g;
    maniGraphInstance* instance = BlendOf(true, &g);
    Set(instance, 0, maniGraphParameterFloat, Float(3.0f));
    Update(instance, 0.25f);
    static const uint32_t kWalkRun[2] = {0, 1};
    static const float kDirect[2] = {1.0f, 0.5f};
    CHECK(Matches(instance, kWalkRun, kDirect, 2, 0.25f), "direct: capped, not normalized");
    // The walk at -3 (none), the run at 1 and the walk again at 3.
    static const uint32_t kInputs[3] = {1, 2, 3};
    static const maniGraphScalar kWeights[3] = {{0, 0.0f}, LITERAL(1.0f), LITERAL(3.0f)};
    maniGraphParameterSource a = Parameter("a", maniGraphParameterFloat, Float(-3.0f));
    maniGraphNodeSource nodes[4] = {{maniGraphNodeBlend, 3, kInputs, .blend = {kWeights, 0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip),
                                    ClipNode(0, 1.0f, maniGraphLoopClip)};
    instance = Make(
        (maniGraphSource){.parameters = &a, .nodes = nodes, .parameterCount = 1, .nodeCount = 4},
        &g);
    Update(instance, 0.25f);
    static const uint32_t kRunWalk[2] = {1, 0};
    static const float kShares[2] = {0.25f, 0.75f};
    CHECK(Matches(instance, kRunWalk, kShares, 2, 0.25f), "a negative weight counts as none");
    // The spine turned two ways, the first of no weight: it does not
    // choose the hemisphere.
    static const maniGraphScalar kTurns[2] = {LITERAL(0.0f), LITERAL(1.0f)};
    nodes[0] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kInputs, .blend = {kTurns, 0}};
    nodes[1] = ClipNode(7, 1.0f, maniGraphLoopClip);
    nodes[2] = ClipNode(8, 1.0f, maniGraphLoopClip);
    instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 3}, &g);
    Update(instance, 0.25f);
    static const uint32_t kTurnB[1] = {8};
    static const float kOne[1] = {1.0f};
    CHECK(Matches(instance, kTurnB, kOne, 1, 0.25f), "a layer of no weight chooses nothing");
}

// A saved state whose node weights are changed, then restored: as if an
// update had left those nodes unweighed.
static void RestoreWeights(maniGraphInstance* instance, const maniGraph* g, const uint32_t* nodes,
                           uint32_t count)
{
    static uint8_t state[1024];
    size_t size = SavedGraphSize(instance);
    CHECK(maniSaveGraphInstance(instance, state, sizeof state, NULL) == mani_success, "saves");
    for (uint32_t k = 0; k < count; ++k)
    {
        size_t at = MANI_GRAPH_STATE_HEADER_SIZE + 16 * (size_t)(g->parameterCount + nodes[k]);
        memset(state + at, 0, 4);
    }
    CHECK(maniRestoreGraphInstance(instance, state, size).result == mani_success, "restores");
}

static bool IsRest(const maniPose* pose)
{
    maniPose* rest = PoseAt(1);
    CHECK(maniPoseSetRest(rest, &s_skeleton) == mani_success, "the rest");
    return SameBits(pose, rest);
}

// Evaluated after a restore that left inputs unweighed: an unweighed
// input gives the rest (not its slot's last pose), an unweighed output
// the rest, an additive node without its base the rest, a difference
// without its input the identity.
static void TestRestoredEvaluation(void)
{
    static const uint32_t kScale[1] = {1};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeTimeScale, 1, kScale, .timeScale = {LITERAL(1.0f)}},
        ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    // The run 0, the time scale 1.
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 2}, &g);
    maniPose* out = PoseAt(0);
    Update(instance, 0.25f);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              !IsRest(out),
          "the run");
    static const uint32_t kRunNode[1] = {0};
    RestoreWeights(instance, &g, kRunNode, 1);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              IsRest(out),
          "an unweighed input: the rest");
    Update(instance, 0.0f);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              !IsRest(out),
          "the run again");
    static const uint32_t kOutput[1] = {1};
    RestoreWeights(instance, &g, kOutput, 1);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              IsRest(out),
          "an unweighed output: the rest");
    // The run 0, a wave 1 under a time scale 2, the wave again 3, the
    // additive node 4.
    static const uint32_t kAdd[3] = {3, 1, 4};
    static const uint32_t kWave[1] = {2};
    static const maniGraphScalar kOnes[2] = {LITERAL(1.0f), LITERAL(1.0f)};
    maniGraphNodeSource layered[5] = {
        {maniGraphNodeAdditive, 3, kAdd, .additive = {kOnes}},
        {maniGraphNodeTimeScale, 1, kWave, .timeScale = {LITERAL(1.0f)}},
        ClipNode(2, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(2, 1.0f, maniGraphLoopClip)};
    instance = Make((maniGraphSource){.nodes = layered, .nodeCount = 5}, &g);
    Update(instance, 0.25f);
    static const uint32_t kWaveNode[1] = {1};
    RestoreWeights(instance, &g, kWaveNode, 1);
    maniPose* run = PoseAt(2);
    maniPose* wave = PoseAt(3);
    const maniBlendLayer layer = {wave, 1.0f, NULL};
    CHECK(maniSampleClip(&s_clips[1], &s_skeleton, 0.25f, run) == mani_success &&
              maniSampleClip(&s_clips[2], &s_skeleton, 0.25f, wave) == mani_success &&
              maniAddPoses(&s_skeleton, run, &layer, 1) == mani_success &&
              maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              SameBits(out, run),
          "a missing difference adds nothing; the other layer is added");
    RestoreWeights(instance, &g, kRunNode, 1);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, out) == mani_success &&
              IsRest(out),
          "an additive node without its base: the rest");
}

// The select's choice evaluated: the run.
static void TestSelectEvaluated(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    maniGraphParameterSource n = Parameter("n", maniGraphParameterInt, Int(1));
    maniGraphNodeSource nodes[3] = {{maniGraphNodeSelect, 2, kInputs, .select = {0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){.parameters = &n, .nodes = nodes, .parameterCount = 1, .nodeCount = 3},
        &g);
    Update(instance, 0.25f);
    static const uint32_t kRun[1] = {1};
    static const float kFull[1] = {1.0f};
    CHECK(Matches(instance, kRun, kFull, 1, 0.25f), "the chosen input's pose");
}

// A bounded float set below its minimum takes the minimum.
static void TestLowClamp(void)
{
    maniGraphParameterSource pace = Parameter("pace", maniGraphParameterFloat, Float(1.0f));
    pace.bounded = 1;
    pace.minimum = Float(0.5f);
    pace.maximum = Float(2.0f);
    maniGraphNodeSource walk = ClipNode(0, 1.0f, maniGraphLoopClip);
    maniGraph g;
    maniGraphInstance* instance = Make(
        (maniGraphSource){.parameters = &pace, .nodes = &walk, .parameterCount = 1, .nodeCount = 1},
        &g);
    maniGraphValue v = {{0}};
    Set(instance, 0, maniGraphParameterFloat, Float(0.1f));
    CHECK(maniGetGraphParameter(instance, 0, &v) == mani_success && v.f[0] == 0.5f,
          "clamped up to the minimum");
}

int main(void)
{
    Open();
    TestFirstParent();
    TestBlendWeights();
    TestAdditiveCap();
    TestMaskShares();
    TestSelectClamp();
    TestNestedScales();
    TestLoops();
    TestLeaderTie();
    TestDeclaredKept();
    TestUnreachedFollower();
    TestOwnWeights();
    TestMarkerStepEnd();
    TestLeaderSeek();
    TestMarkerPhaseFallback();
    TestMarkerFollowerSilent();
    TestMarkerNearest();
    TestMarkerNames();
    TestEvaluatedBlends();
    TestRestoredEvaluation();
    TestSelectEvaluated();
    TestLowClamp();
    return s_failures == 0 ? 0 : 1;
}
