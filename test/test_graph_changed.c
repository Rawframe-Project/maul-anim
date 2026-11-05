// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph's bytes changed after the open: the sweep of
// graph_changed.h over a graph of most kinds, its file a heap block of
// its exact length; then changes with their documented results.

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "graph_changed.h"
#include "graph_file.h"
#include "graph_fixture.h"

#include "maul-anim/pose.h"

#include <stdlib.h>

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[2][4096];
static maniPose* s_pose;
static maniPose* s_out;
static maniGraphParameterSource s_parameters[7];

// Parameters: 0 a float (a blend's weight), 1 a pose, 2 an int (the
// select's choice), 3 the one-shot's fire, 4 the machine's jump, 5 a
// float (the other weight), 6 the inertialization's trigger.
static void Parameters(void)
{
    s_parameters[0] = Parameter("weight", maniGraphParameterFloat, Float(0.5f));
    s_parameters[1] = Parameter("ragdoll", maniGraphParameterPose, Int(0));
    s_parameters[2] = Parameter("choice", maniGraphParameterInt, Int(1));
    s_parameters[3] = Parameter("fire", maniGraphParameterTrigger, Int(0));
    s_parameters[4] = Parameter("jump", maniGraphParameterTrigger, Int(0));
    s_parameters[5] = Parameter("speed", maniGraphParameterFloat, Float(1.0f));
    s_parameters[6] = Parameter("settle", maniGraphParameterTrigger, Int(0));
}

// A graph of most kinds: an inertialization over a one-shot whose base
// is a machine (a select of a walk and the pose, and a blend of two
// marked steps; by the jump and back on a left step) and whose shot is
// the run.
static maniGraphSource Source(maniGraphNodeSource* nodes)
{
    static const uint32_t kInertia[1] = {1};
    static const uint32_t kShot[2] = {2, 3};
    static const uint32_t kStates[2] = {4, 5};
    static const uint32_t kChoice[2] = {6, 7};
    static const uint32_t kPair[2] = {8, 9};
    static const maniGraphScalar kWeights[2] = {{0, 0.0f}, {5, 0.0f}};
    static const maniGraphConditionSource kJump[1] = {
        {maniGraphConditionTrigger, 4, 0, 0, {{0}}, NULL}};
    static const maniGraphConditionSource kLeft[1] = {
        {maniGraphConditionEvent, MANI_NO_PARAMETER, 0, 4, {{0}}, "left"}};
    static const maniGraphTransitionSource kTransitions[2] = {
        {0, 1, 0, maniGraphInterruptionNone, maniGraphEntryRestart, maniGraphFadeLinear, 0.2f, 0.0f,
         kJump, 1},
        {1, 0, 1, maniGraphInterruptionAny, maniGraphEntryKeep, maniGraphFadeSmoothstep, 0.1f, 0.0f,
         kLeft, 1}};
    Parameters();
    nodes[0] =
        (maniGraphNodeSource){maniGraphNodeInertialize, 1, kInertia,
                              .inertialize = {6, 0, {MANI_NO_PARAMETER, 0.2f}, 0.0f, 0.0f, 0.0f}};
    nodes[1] = (maniGraphNodeSource){maniGraphNodeOneShot, 2, kShot,
                                     .oneShot = {3, MANI_NO_PARAMETER, MANI_NO_PARAMETER,
                                                 maniGraphFadeLinear, 0.1f, 0.1f, NULL}};
    nodes[2] = (maniGraphNodeSource){maniGraphNodeMachine, 2, kStates,
                                     .machine = {kTransitions, 2, 0, 0, 0, NULL}};
    nodes[3] = ClipNode(1, 1.0f, maniGraphLoopRepeat);
    nodes[4] = (maniGraphNodeSource){maniGraphNodeSelect, 2, kChoice, .select = {2}};
    nodes[5] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kPair, .blend = {kWeights, 0}};
    nodes[6] = ClipNode(0, 1.0f, maniGraphLoopRepeat);
    nodes[7] = (maniGraphNodeSource){maniGraphNodePose, 0, NULL, .pose = {1}};
    nodes[8] = ClipNode(3, 1.0f, maniGraphLoopRepeat);
    nodes[9] = ClipNode(5, 1.0f, maniGraphLoopRepeat);
    return (maniGraphSource){.parameters = s_parameters,
                             .nodes = nodes,
                             .parameterCount = 7,
                             .nodeCount = 10,
                             .output = 0,
                             .skeleton = &s_skeleton,
                             .clips = kSlots,
                             .clipCount = CLIPS};
}

// The graph built and copied into a heap block of its exact length;
// *blockOut is what to free.
static uint8_t* Built(maniGraphSource source, size_t* sizeOut, uint8_t** blockOut)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success,
          "the graph built");
    maniDestroyBuilder(builder);
    uint8_t* file = TestExactCopy(size, blockOut);
    memcpy(file, s_file, size);
    *sizeOut = size;
    return file;
}

static const maniClip* const* Clips(void)
{
    static const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    return clips;
}

static void TestChangedWords(void)
{
    maniGraphNodeSource nodes[10];
    size_t size = 0;
    uint8_t* block = NULL;
    uint8_t* file = Built(Source(nodes), &size, &block);
    maniGraph g;
    CHECK(maniOpenGraph(file, size, &g).result == mani_success, "opened");
    maniGraphAssets assets = {&s_skeleton, Clips(), NULL, NULL, NULL, CLIPS, 0,
                              0,           0,       NULL, NULL, 0,    0,     NULL};
    TestChangedGraph(&g, &assets, "every word of a graph of most kinds changed and put back");
    free(block);
}

// Bytes changed after the bind, with the results the documentation gives: a
// pose node whose parameter is no longer a pose parameter gives the
// rest; a parameter made a pose past those bind counted is refused; a
// name past the names is none.
static void TestFound(void)
{
    maniGraphNodeSource nodes[10];
    size_t size = 0;
    uint8_t* block = NULL;
    uint8_t* file = Built(Source(nodes), &size, &block);
    maniGraph g;
    CHECK(maniOpenGraph(file, size, &g).result == mani_success, "opened");
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    maniGraphAssets assets = {&s_skeleton, Clips(), NULL, NULL, NULL, CLIPS, 0,
                              0,           0,       NULL, NULL, 0,    0,     NULL};
    CHECK(maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
                  mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "bound");
    uint8_t* type = (uint8_t*)(uintptr_t)g.parameters + 5 * (size_t)MANI_GRAPH_PARAMETER_SIZE;
    maniStoreU32(type, maniGraphParameterPose);
    CHECK(maniGraphInstance_SetPose(instance, 5, s_pose) == mani_errorInvalid &&
              maniGraphInstance_GetPose(instance, 5, s_out) == mani_errorInvalid,
          "a parameter made a pose past those bind counted is refused");
    maniStoreU32(type, maniGraphParameterFloat);
    uint8_t* name = (uint8_t*)(uintptr_t)g.parameters + 8;
    uint32_t kept = maniLoadU32(name + 4);
    maniStoreU32(name + 4, 0xFFFFFFFFu);
    size_t length = 1;
    CHECK(maniGraph_GetParameterName(&g, 0, &length) == NULL && length == 0 &&
              maniGraph_FindParameter(&g, "weight", 6) == MANI_NO_PARAMETER,
          "a name past the names is none");
    // Its hash made the empty name's: still none, not the empty name.
    uint8_t* hash = (uint8_t*)(uintptr_t)g.parameters + 16;
    uint64_t keptHash = maniLoadU64(hash);
    maniStoreU64(hash, maniHashBytes(MANI_HASH_INIT, NULL, 0));
    CHECK(maniGraph_FindParameter(&g, "", 0) == MANI_NO_PARAMETER,
          "a name past the names is not the empty name");
    maniStoreU64(hash, keptHash);
    maniStoreU32(name + 4, kept);
    // The pose node's parameter made the int's: the pose node gives the
    // rest, and the select (choice 1) passes it on.
    for (uint32_t p = 0; p < g.nodeCount; ++p)
    {
        const uint8_t* record = g.nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        if (maniLoadU16(record) == maniGraphNodePose)
        {
            maniStoreU32((uint8_t*)(uintptr_t)g.data + 4 * (size_t)maniLoadU32(record + 12), 2);
        }
    }
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniPose_SetLocal(
            s_pose, b,
            (maniTransform){{1.0f, 2.0f, 3.0f}, {0.0f, 0.6f, 0.0f, 0.8f}, {1.0f, 1.0f, 1.0f}});
    }
    CHECK(maniGraphInstance_SetPose(instance, 1, s_pose) == mani_success, "the pose set");
    Update(instance, 0.1f);
    CHECK(maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_out) == mani_success,
          "evaluated");
    bool rest = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_out, b);
        maniTransform r = maniSkeleton_GetRest(&s_skeleton, b);
        rest = rest && memcmp(&t, &r, sizeof t) == 0;
    }
    CHECK(rest, "a pose node whose parameter is not a pose gives the rest");
    free(block);
}

int main(void)
{
    Open();
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[0], sizeof s_poseMemory[0], &s_pose) ==
                  mani_success &&
              maniInitPose(&s_skeleton, s_poseMemory[1], sizeof s_poseMemory[1], &s_out) ==
                  mani_success,
          "poses");
    TestChangedWords();
    TestFound();
    return s_failures == 0 ? 0 : 1;
}
