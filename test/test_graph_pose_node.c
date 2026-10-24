// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pose input node: the rest pose until its parameter is
// set, then the pose set; blended over a walk by weight (none, half,
// all) and by mask (the spine's subtree the ragdoll's, the hips the
// walk's); inside a subgraph through its parameter mapping; refused at
// build with inputs or a parameter of another type, and a pose parameter
// refused where a select, a weight or a condition reads it; a file whose
// pose node reads a float refused at open.

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_fixture.h"

#include "maul-anim/pose.h"

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[2][4096];
static _Alignas(16) uint8_t s_crafted[16384];
static maniPose* s_pose;
static maniPose* s_out;
static maniGraphParameterSource s_parameters[5];
static maniTransform s_walk[3];

// Parameters: 0 a float (the ragdoll's weight, 1), 1 a pose, 2 a bool,
// 3 a float (the walk's weight, 1), 4 a trigger (getting up).
static maniGraphSource PoseSource(const maniGraphNodeSource* nodes, uint32_t count)
{
    s_parameters[0] = Parameter("weight", maniGraphParameterFloat, Float(1.0f));
    s_parameters[1] = Parameter("ragdoll", maniGraphParameterPose, Int(0));
    s_parameters[2] = Parameter("lean", maniGraphParameterBool, Int(0));
    s_parameters[3] = Parameter("walk", maniGraphParameterFloat, Float(1.0f));
    s_parameters[4] = Parameter("getUp", maniGraphParameterTrigger, Int(0));
    return (maniGraphSource){.parameters = s_parameters,
                             .nodes = nodes,
                             .parameterCount = 5,
                             .nodeCount = count,
                             .output = count - 1};
}

static maniGraphNodeSource PoseNode(uint32_t parameter)
{
    return (maniGraphNodeSource){maniGraphNodePose, 0, NULL, .pose = {parameter}};
}

// Bone b of the ragdoll's pose: each bone moved and turned apart.
static maniTransform Moved(uint32_t b)
{
    float f = (float)(b + 1);
    return (maniTransform){
        {0.25f * f, -0.5f * f, 2.0f}, {0.0f, 0.6f, 0.0f, 0.8f}, {1.0f, 1.5f * f, 0.5f}};
}

static void SetRagdoll(maniGraphInstance* instance)
{
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniPoseSetLocal(s_pose, b, Moved(b));
    }
    CHECK(maniSetGraphPose(instance, 1, s_pose) == mani_success, "the ragdoll set");
}

// An update of 0.3 seconds and the output's bone b.
static maniTransform Out(maniGraphInstance* instance, uint32_t b)
{
    Update(instance, 0.3f);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_out) == mani_success,
          "evaluated");
    return maniPoseLocal(s_out, b);
}

static bool NearT(maniTransform a, maniTransform b)
{
    float u[10];
    float v[10];
    _Static_assert(sizeof u == sizeof a, "ten floats a transform");
    memcpy(u, &a, sizeof u);
    memcpy(v, &b, sizeof v);
    bool near = true;
    for (int k = 0; k < 10; ++k)
    {
        near = near && NearF(u[k], (double)v[k]);
    }
    return near;
}

static bool Same(maniTransform a, maniTransform b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

// The walk alone 0.3 seconds in, the reference the blends are read
// against.
static void Walk(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraphInstance* instance = Make(PoseSource(nodes, 1), &g);
    for (uint32_t b = 0; b < 3; ++b)
    {
        s_walk[b] = b == 0 ? Out(instance, 0) : maniPoseLocal(s_out, b);
    }
}

// The node alone: the rest until set, then the pose set, a pose (not
// differences) with its model computed by the evaluation's caller.
static void TestAlone(void)
{
    maniGraph g;
    maniGraphNodeSource nodes[1] = {PoseNode(1)};
    maniGraphInstance* instance = Make(PoseSource(nodes, 1), &g);
    bool rest = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        rest = rest && Same(b == 0 ? Out(instance, 0) : maniPoseLocal(s_out, b),
                            maniSkeletonRest(&s_skeleton, b));
    }
    CHECK(rest, "the rest until set");
    SetRagdoll(instance);
    bool moved = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        moved = moved && Same(b == 0 ? Out(instance, 0) : maniPoseLocal(s_out, b), Moved(b));
    }
    CHECK(moved && !maniPoseIsDifference(s_out), "the pose set");
}

// The walk and the ragdoll blended by the weight parameter: 0 the walk,
// 1 against the walk's 1 half of each (translations linear), the walk's
// weight 0 the ragdoll.
static void TestBlended(void)
{
    static const uint32_t kInputs[2] = {0, 1};
    static const maniGraphScalar kWeights[2] = {{3, 0.0f}, {0, 0.0f}};
    maniGraph g;
    maniGraphNodeSource nodes[3] = {
        ClipNode(0, 1.0f, maniGraphLoopRepeat), PoseNode(1),
        (maniGraphNodeSource){maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0, 0}}};
    maniGraphInstance* instance = Make(PoseSource(nodes, 3), &g);
    SetRagdoll(instance);
    Set(instance, 0, maniGraphParameterFloat, Float(0.0f));
    CHECK(NearT(Out(instance, 1), s_walk[1]), "weight 0: the walk");
    Set(instance, 0, maniGraphParameterFloat, Float(1.0f));
    maniTransform half = Out(instance, 2);
    CHECK(NearF(half.translation.x,
                0.5 * ((double)s_walk[2].translation.x + (double)Moved(2).translation.x)) &&
              NearF(half.translation.y,
                    0.5 * ((double)s_walk[2].translation.y + (double)Moved(2).translation.y)),
          "even weights: half of each");
    Set(instance, 3, maniGraphParameterFloat, Float(0.0f));
    CHECK(NearT(Out(instance, 1), Moved(1)), "the walk's weight 0: the ragdoll");
}

// A partial ragdoll: the spine's subtree the ragdoll's, the hips the
// walk's.
static void TestMasked(void)
{
    static const uint32_t kInputs[2] = {1, 0};
    static const maniMaskChain kChains[1] = {{1, 1.0f, true}};
    maniGraph g;
    maniGraphNodeSource nodes[3] = {
        ClipNode(0, 1.0f, maniGraphLoopRepeat), PoseNode(1),
        (maniGraphNodeSource){maniGraphNodeMask, 2, kInputs, .mask = {kChains, 1, 0.0f}}};
    maniGraphInstance* instance = Make(PoseSource(nodes, 3), &g);
    SetRagdoll(instance);
    maniTransform hips = Out(instance, 0);
    CHECK(NearT(hips, s_walk[0]) && NearT(maniPoseLocal(s_out, 1), Moved(1)) &&
              NearT(maniPoseLocal(s_out, 2), Moved(2)),
          "the spine's subtree the ragdoll's, the hips the walk's");
}

// The hips' translation after an update of delta.
static maniVec3 Hips(maniGraphInstance* instance, float delta)
{
    Update(instance, delta);
    CHECK(maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_out) == mani_success,
          "evaluated");
    return maniPoseLocal(s_out, 0).translation;
}

static double Gap(maniVec3 a, maniVec3 b)
{
    double x = (double)a.x - (double)b.x;
    double y = (double)a.y - (double)b.y;
    double z = (double)a.z - (double)b.z;
    return sqrt(x * x + y * y + z * z);
}

// Getting up: a machine leaves the ragdoll for the walk on the trigger
// with an inertial transition of half a second under an inertialize
// node: just after, the hips are still near the ragdoll's; past the
// half second they are the walk's as it plays alone from its start.
static void TestWayBack(void)
{
    static const maniGraphConditionSource kGo[1] = {
        {maniGraphConditionTrigger, 4, 0, 0, {{0}}, NULL}};
    static const maniGraphTransitionSource kUp[1] = {{0, 1, 0, maniGraphInterruptionNone,
                                                      maniGraphEntryRestart, maniGraphFadeInertial,
                                                      0.5f, 0.0f, kGo, 1, 0}};
    static const uint32_t kStates[2] = {0, 1};
    static const uint32_t kMachine[1] = {2};
    maniGraph g;
    maniGraphNodeSource alone[1] = {ClipNode(0, 1.0f, maniGraphLoopRepeat)};
    maniGraphInstance* walk = Make(PoseSource(alone, 1), &g);
    maniVec3 walkStart = Hips(walk, 0.02f);
    maniVec3 walkEnd = Hips(walk, 0.6f);
    maniGraphNodeSource nodes[4] = {
        PoseNode(1), ClipNode(0, 1.0f, maniGraphLoopRepeat),
        (maniGraphNodeSource){maniGraphNodeMachine, 2, kStates, .machine = {kUp, 1, 0, 0, 0, NULL}},
        (maniGraphNodeSource){
            maniGraphNodeInertialize, 1, kMachine,
            .inertialize = {MANI_NO_PARAMETER, 0, LITERAL(0.0f), 0.0f, 0.0f, 0.0f}}};
    maniGraphInstance* instance = Make(PoseSource(nodes, 4), &g);
    SetRagdoll(instance);
    CHECK(Gap(Hips(instance, 0.3f), Moved(0).translation) < 1e-6, "the ragdoll before");
    Set(instance, 4, maniGraphParameterTrigger, Int(1));
    maniVec3 after = Hips(instance, 0.02f);
    CHECK(Gap(after, Moved(0).translation) < 0.05 * Gap(walkStart, Moved(0).translation),
          "just after: near the ragdoll, not the walk");
    CHECK(Gap(Hips(instance, 0.6f), walkEnd) < 1e-5, "past the half second: the walk alone");
}

// A pose node in a subgraph reads the outer pose parameter its own maps
// to.
static void TestSubgraph(void)
{
    static maniGraphParameterSource inner[1];
    static maniGraphNodeSource innerNodes[1];
    static const uint32_t kMap[1] = {1};
    inner[0] = Parameter("body", maniGraphParameterPose, Int(0));
    innerNodes[0] = PoseNode(0);
    static maniGraphSource sub;
    sub = (maniGraphSource){
        .parameters = inner, .nodes = innerNodes, .parameterCount = 1, .nodeCount = 1, .output = 0};
    sub.skeleton = &s_skeleton;
    maniGraph g;
    maniGraphNodeSource nodes[1] = {(maniGraphNodeSource){(maniGraphNodeKind)maniGraphNodeSubgraph,
                                                          0, NULL, .subgraph = {&sub, kMap}}};
    maniGraphInstance* instance = Make(PoseSource(nodes, 1), &g);
    SetRagdoll(instance);
    CHECK(Same(Out(instance, 2), Moved(2)), "the outer pose through the mapping");
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

static bool RefusedAt(maniBuildResult r, uint32_t node)
{
    return r.result == mani_errorInvalid && r.item == node;
}

// Refused at build: a pose node with an input, reading a float or a
// parameter past the count; a select, a weight or a condition reading a
// pose parameter; a subgraph's pose parameter mapped to a float.
static void TestBuildRefusals(void)
{
    static const uint32_t kOne[1] = {0};
    maniGraphNodeSource nodes[3] = {ClipNode(0, 1.0f, maniGraphLoopRepeat),
                                    (maniGraphNodeSource){maniGraphNodePose, 1, kOne, .pose = {1}}};
    maniBuildResult r = Built(PoseSource(nodes, 2));
    CHECK(RefusedAt(r, 1) && r.reason == maniBuildReasonNode, "a pose node with an input");
    nodes[1] = PoseNode(0);
    r = Built(PoseSource(nodes, 2));
    CHECK(RefusedAt(r, 1) && r.reason == maniBuildReasonNode, "a pose node reading a float");
    nodes[1] = PoseNode(5);
    CHECK(RefusedAt(Built(PoseSource(nodes, 2)), 1), "a pose node past the parameters");
    static const uint32_t kPair[2] = {0, 1};
    nodes[1] = ClipNode(1, 1.0f, maniGraphLoopRepeat);
    nodes[2] = (maniGraphNodeSource){maniGraphNodeSelect, 2, kPair, .select = {1}};
    CHECK(RefusedAt(Built(PoseSource(nodes, 3)), 2), "a select reading a pose parameter");
    static const maniGraphScalar kWeights[2] = {{1, 0.0f}, {MANI_NO_PARAMETER, 1.0f}};
    nodes[2] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kPair, .blend = {kWeights, 0, 0}};
    CHECK(Built(PoseSource(nodes, 3)).result == mani_errorInvalid,
          "a weight reading a pose parameter");
    static const maniGraphConditionSource kCondition[1] = {
        {maniGraphConditionParameter, 1, maniGraphCompareEqual, 0, {{0}}, NULL}};
    static const maniGraphTransitionSource kTransition[1] = {
        {0, 1, 0, 0, 0, 0, 0.1f, 0.0f, kCondition, 1, 0}};
    nodes[2] = (maniGraphNodeSource){maniGraphNodeMachine, 2, kPair,
                                     .machine = {kTransition, 1, 0, 0, 0, NULL}};
    CHECK(Built(PoseSource(nodes, 3)).result == mani_errorInvalid,
          "a condition reading a pose parameter");
    static maniGraphParameterSource inner[1];
    static maniGraphNodeSource innerNodes[1];
    static const uint32_t kMap[1] = {0};
    inner[0] = Parameter("body", maniGraphParameterPose, Int(0));
    innerNodes[0] = PoseNode(0);
    static maniGraphSource sub;
    sub = (maniGraphSource){
        .parameters = inner, .nodes = innerNodes, .parameterCount = 1, .nodeCount = 1, .output = 0};
    sub.skeleton = &s_skeleton;
    nodes[0] = (maniGraphNodeSource){(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                     .subgraph = {&sub, kMap}};
    CHECK(Built(PoseSource(nodes, 1)).result == mani_errorInvalid,
          "a subgraph's pose parameter mapped to a float");
}

// The payload hash and identity made right after an edit.
static void Rehash(uint8_t* file, size_t size)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, size - 64));
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

// A file whose pose node reads the float (or past the parameters)
// refused at open at the node's word.
static void TestFileRefusals(void)
{
    maniGraphNodeSource nodes[1] = {PoseNode(1)};
    maniGraphSource source = PoseSource(nodes, 1);
    source.skeleton = &s_skeleton;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniGraph g;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, &g).result == mani_success,
          "a graph of a pose node");
    maniDestroyBuilder(builder);
    uint32_t data = maniLoadU32(g.nodes + 12);
    size_t at = (size_t)(g.data - s_file) + 4 * (size_t)data;
    CHECK(maniLoadU32(s_file + at) == 1, "the node's word is its parameter");
    static const uint32_t kWrong[2] = {0, 5};
    for (int k = 0; k < 2; ++k)
    {
        memcpy(s_crafted, s_file, size);
        maniStoreU32(s_crafted + at, kWrong[k]);
        Rehash(s_crafted, size);
        maniGraph crafted;
        maniFileResult r = maniOpenGraph(s_crafted, size, &crafted);
        CHECK(r.result == mani_errorFormat && r.offset == at, "refused at the node's word");
    }
}

int main(void)
{
    Open();
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[0], sizeof s_poseMemory[0], &s_pose) ==
                  mani_success &&
              maniInitPose(&s_skeleton, s_poseMemory[1], sizeof s_poseMemory[1], &s_out) ==
                  mani_success,
          "poses");
    Walk();
    TestAlone();
    TestBlended();
    TestMasked();
    TestSubgraph();
    TestWayBack();
    TestBuildRefusals();
    TestFileRefusals();
    return s_failures == 0 ? 0 : 1;
}
