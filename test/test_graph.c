// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph files: a graph built against a skeleton and its
// clips, its nodes ordered depth first from the output, poses and
// differences marked, slots reused; every refusal of the builder and of
// opening; the bytes pinned. Run as test_graph --seed <file> it writes a
// seed for the graph fuzz target, and as --seed-large <file> another, the
// marker graph of the largest time factor; run as test_graph large-steps
// it checks that graph's updates alone.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// The graph scene: the file, then an instance of it stepped across its
// clips' wraps (each step's pose, root motion and events) and its saved
// state; every platform and backend must give this value.
#define PINNED_SCENE_HASH 0x674fbaef8dc62397ull

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_clipFiles[3][8192];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;
static maniSkeleton s_skeleton;
static maniClip s_clips[3];

// Hips, a spine under them, an arm under the spine.
static const maniBoneSource kBones[3] = {
    {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"spine", 5, 0, {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"arm", 3, 1, {{0.3f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// A walk, a run (the hips along x, at 1 and 3 a second) and a wave (the
// arm turning, as differences from the rest).
static const float kTimes[2] = {0.0f, 1.0f};
static const float kWalk[6] = {0, 1, 0, 1, 1, 0};
static const float kRun[6] = {0, 1, 0, 3, 1, 0};
static const float kWave[8] = {0, 0, 0, 1, 0, 0, 0.38268343f, 0.92387953f};

static void OpenClips(void)
{
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    const maniBoneTrackSource tracks[3] = {
        {0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone},
        {0, {kTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone},
        {2, kNone, {kTimes, kWave, 2, maniInterpolationLinear}, kNone},
    };
    static const maniEventSource kSteps[2] = {{0.0f, 7, "left", 4}, {0.5f, 8, "right", 5}};
    for (int i = 0; i < 3; ++i)
    {
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .events = i == 0 ? kSteps : NULL,
                                 .eventCount = i == 0 ? 2u : 0u,
                                 .boneTracks = &tracks[i],
                                 .boneTrackCount = 1,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 10.0f,
                                 .additive = i == 2 ? maniAdditiveRest : maniAdditiveNone};
        maniBuilder* builder = NewBuilder();
        size_t size = 0;
        CHECK(maniBuildClip(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_clipFiles[i], sizeof s_clipFiles[i], &size) ==
                      mani_success &&
                  maniOpenClip(s_clipFiles[i], size, &s_clips[i]).result == mani_success,
              "a clip");
        maniDestroyBuilder(builder);
    }
}

static void OpenSkeleton(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {kBones, 3};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
          "skeleton");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
    OpenClips();
}

// The test graph, its parts in statics a test may change and restore.
static maniGraphParameterSource s_parameters[6];
static maniGraphClipSource s_clipSlots[3];
static maniGraphSyncGroupSource s_groups[1];
static maniGraphNodeSource s_nodes[9];
static maniGraphScalar s_blendWeights[2];
static maniGraphScalar s_layerWeights[1];
static maniMaskChain s_chains[1];
static const uint32_t kBlendInputs[2] = {2, 3};
static const uint32_t kAdditiveInputs[2] = {8, 5};
static const uint32_t kMaskInputs[2] = {0, 6};
static const uint32_t kSelectInputs[2] = {2, 3};
static const uint32_t kTimeInputs[1] = {4};
static const uint32_t kSeekInputs[1] = {7};

static maniGraphValue Float(float a, float b, float c, float d)
{
    maniGraphValue v = {{a, b, c, d}};
    return v;
}

static maniGraphValue Int(int32_t i)
{
    maniGraphValue v = {{0}};
    v.i = i;
    return v;
}

static maniGraphParameterSource Parameter(const char* name, maniGraphParameterType type,
                                          maniGraphValue value)
{
    return (maniGraphParameterSource){name, (uint32_t)strlen(name), type, value, Int(0), Int(0), 0};
}

// Nodes in the source's order: 0 blend (walk, run), 1 the output (an
// additive node: the seek, the wave), 2 walk, 3 run, 4 a mask (the blend
// on the spine and arm, the select elsewhere), 5 the wave, 6 a select
// (walk, run), 7 a time scale (the mask), 8 a seek (the time scale). Walk
// and run feed both the blend and the select.
static maniGraphSource Source(void)
{
    s_parameters[0] = Parameter("speed", maniGraphParameterFloat, Float(1, 0, 0, 0));
    s_parameters[0].bounded = 1;
    s_parameters[0].minimum = Float(0, 0, 0, 0);
    s_parameters[0].maximum = Float(2, 0, 0, 0);
    s_parameters[1] = Parameter("mix", maniGraphParameterFloat, Float(0.5f, 0, 0, 0));
    s_parameters[2] = Parameter("gait", maniGraphParameterInt, Int(0));
    s_parameters[2].bounded = 1;
    s_parameters[2].maximum = Int(1);
    s_parameters[3] = Parameter("jump", maniGraphParameterTrigger, Int(0));
    s_parameters[4] = Parameter("lean", maniGraphParameterBool, Int(1));
    s_parameters[5] = Parameter("aim", maniGraphParameterQuat, Float(0, 0, 0, 2));
    s_clipSlots[0] = (maniGraphClipSource){&s_clips[0], "walk", 4, NULL};
    s_clipSlots[1] = (maniGraphClipSource){&s_clips[1], "run", 3, NULL};
    s_clipSlots[2] = (maniGraphClipSource){&s_clips[2], "wave", 4, NULL};
    s_groups[0] = (maniGraphSyncGroupSource){"feet", 4, 0};
    s_blendWeights[0] = (maniGraphScalar){1, 0.0f};
    s_blendWeights[1] = (maniGraphScalar){MANI_NO_PARAMETER, 1.0f};
    s_layerWeights[0] = (maniGraphScalar){MANI_NO_PARAMETER, 1.0f};
    s_chains[0] = (maniMaskChain){1, 1.0f, true};
    const maniGraphScalar one = {MANI_NO_PARAMETER, 1.0f};
    s_nodes[0] =
        (maniGraphNodeSource){maniGraphNodeBlend, 2, kBlendInputs, .blend = {s_blendWeights, 0}};
    s_nodes[1] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kAdditiveInputs,
                                       .additive = {s_layerWeights}};
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                       .clip = {0, 0, one, maniGraphLoopClip, maniGraphClipLeader}};
    s_nodes[3] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                       .clip = {1, 0, {0, 0.0f}, maniGraphLoopClip, 0}};
    s_nodes[4] =
        (maniGraphNodeSource){maniGraphNodeMask, 2, kMaskInputs, .mask = {s_chains, 1, 0.0f}};
    s_nodes[5] = (maniGraphNodeSource){
        maniGraphNodeClip, 0, NULL, .clip = {2, MANI_NO_SYNC_GROUP, one, maniGraphLoopRepeat, 0}};
    s_nodes[6] = (maniGraphNodeSource){maniGraphNodeSelect, 2, kSelectInputs, .select = {2}};
    s_nodes[7] = (maniGraphNodeSource){maniGraphNodeTimeScale, 1, kTimeInputs,
                                       .timeScale = {{MANI_NO_PARAMETER, 1.5f}}};
    s_nodes[8] = (maniGraphNodeSource){maniGraphNodeSeek, 1, kSeekInputs,
                                       .seek = {3, {MANI_NO_PARAMETER, 0.25f}}};
    return (maniGraphSource){
        &s_skeleton, s_parameters, s_clipSlots, s_groups, s_nodes, 6,    3,   1, 9, 1,
        0,           NULL,         0,           0,        NULL,    NULL, NULL};
}

static maniBuildResult Build(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    s_size = 0;
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return r;
}

static uint32_t NodeWord(const maniGraph* g, uint32_t p, uint32_t at)
{
    return maniLoadU32(g->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + at);
}

static void TestBuild(void)
{
    maniGraphSource source = Source();
    maniGraph g;
    CHECK(Build(&source).result == mani_success &&
              maniOpenGraph(s_file, s_size, &g).result == mani_success,
          "builds and opens");
    CHECK(maniGraphNodeCount(&g) == 9 && maniGraphParameterCount(&g) == 6 &&
              maniGraphClipCount(&g) == 3 && g.maskCount == 1 && g.syncGroupCount == 1,
          "counts");
    CHECK(maniGraphSkeletonIdentity(&g) == maniSkeletonIdentity(&s_skeleton) &&
              maniGraphClipIdentity(&g, 2) == maniClipIdentity(&s_clips[2]) &&
              maniGraphClipIdentity(&g, 3) == 0,
          "identities");
    // Depth first from the output: walk, run, blend, select, mask, time
    // scale, seek, wave, the output.
    static const uint16_t kKinds[9] = {
        maniGraphNodeClip,   maniGraphNodeClip, maniGraphNodeBlend,
        maniGraphNodeSelect, maniGraphNodeMask, maniGraphNodeTimeScale,
        maniGraphNodeSeek,   maniGraphNodeClip, maniGraphNodeAdditive};
    bool kinds = true;
    for (uint32_t p = 0; p < 9; ++p)
    {
        kinds = kinds && (NodeWord(&g, p, 0) & 0xFFFFu) == kKinds[p];
    }
    CHECK(kinds, "each node after its inputs, the output last");
    // Slots: walk 0, run 1, blend 2, select 3, then reused.
    static const uint32_t kSlots[9] = {0, 1, 2, 3, 1, 3, 1, 3, 2};
    bool slots = g.slotCount == 4;
    for (uint32_t p = 0; p < 9; ++p)
    {
        slots = slots && NodeWord(&g, p, 20) == kSlots[p];
    }
    CHECK(slots, "four slots, reused once read");
    CHECK((NodeWord(&g, 7, 0) >> 16) == MANI_GRAPH_DIFFERENCE && (NodeWord(&g, 8, 0) >> 16) == 0,
          "the wave gives differences, the output poses");
    maniGraphParameterType type = 0;
    uint32_t length = 0;
    const char* name = maniGraphParameterName(&g, 3, &length);
    CHECK(maniGraphFindParameter(&g, "mix", 3) == 1 &&
              maniGraphFindParameter(&g, "mi", 2) == MANI_NO_PARAMETER &&
              maniGraphParameterTypeOf(&g, 3, &type) == mani_success &&
              type == maniGraphParameterTrigger && length == 4 && memcmp(name, "jump", 4) == 0 &&
              maniGraphParameterName(&g, 6, &length) == NULL && length == 0 &&
              maniGraphParameterTypeOf(&g, 6, &type) == mani_errorInvalid,
          "parameters by name and index");
    float w = 0.0f;
    memcpy(&w, g.parameters + 5 * MANI_GRAPH_PARAMETER_SIZE + 36, sizeof w);
    CHECK(w == 1.0f, "a quaternion default normalized");
    name = maniGraphClipName(&g, 1, &length);
    CHECK(length == 3 && memcmp(name, "run", 3) == 0 && maniGraphClipName(&g, 3, &length) == NULL &&
              length == 0,
          "clip names");
    float spine = 0.0f;
    float hips = 1.0f;
    memcpy(&hips, g.maskWeights, sizeof hips);
    memcpy(&spine, g.maskWeights + 8, sizeof spine);
    CHECK(hips == 0.0f && spine == 1.0f, "the mask's bone weights");
    size_t first = s_size;
    memcpy(s_crafted, s_file, s_size);
    CHECK(Build(&source).result == mani_success && s_size == first &&
              memcmp(s_crafted, s_file, s_size) == 0,
          "the same bytes every build");
}

static void ExpectRefused(const maniGraphSource* source, maniBuildReason reason, uint32_t item,
                          uint32_t other, const char* what)
{
    maniBuildResult r = Build(source);
    CHECK(r.result == mani_errorInvalid && r.reason == reason && r.item == item && r.other == other,
          what);
    if (r.reason != reason || r.item != item || r.other != other)
    {
        printf("  %s: reason %u item %u other %u\n", what, r.reason, r.item, r.other);
    }
}

static void TestParameterRefusals(void)
{
    maniGraphSource s = Source();
    s_parameters[1].nameLength = 0;
    ExpectRefused(&s, maniBuildReasonParameterList | maniBuildReasonName, 1, 0, "an empty name");
    s = Source();
    s_parameters[4].name = "mix";
    s_parameters[4].nameLength = 3;
    ExpectRefused(&s, maniBuildReasonParameterList | maniBuildReasonDuplicateName, 4, 1,
                  "a name twice");
    s = Source();
    s_parameters[1].type = 7;
    ExpectRefused(&s, maniBuildReasonParameter, 1, 0, "an unknown type");
    s = Source();
    s_parameters[0].defaultValue = Float(3, 0, 0, 0);
    ExpectRefused(&s, maniBuildReasonParameter, 0, 0, "a default past its bounds");
    s = Source();
    s_parameters[2].maximum = Int(-1);
    ExpectRefused(&s, maniBuildReasonParameter, 2, 0, "bounds inverted");
    s = Source();
    s_parameters[5].defaultValue = Float(0, 0, 0, 0);
    ExpectRefused(&s, maniBuildReasonParameter, 5, 0, "a quaternion of zero length");
    s = Source();
    s_parameters[4].defaultValue = Int(2);
    ExpectRefused(&s, maniBuildReasonParameter, 4, 0, "a bool of 2");
    s = Source();
    s_parameters[1].defaultValue = Float(NAN, 0, 0, 0);
    ExpectRefused(&s, maniBuildReasonParameter, 1, 0, "a default not finite");
    s = Source();
    s_parameters[4].bounded = 1;
    ExpectRefused(&s, maniBuildReasonParameter, 4, 0, "bounds on a bool");
}

static void TestNodeRefusals(void)
{
    maniGraphSource s = Source();
    s_clipSlots[1].clip = &s_clips[0];
    ExpectRefused(&s, maniBuildReasonClip, 1, 0, "a clip twice");
    s = Source();
    s_nodes[6].kind = 63;
    ExpectRefused(&s, maniBuildReasonNode, 6, 0, "an unknown kind");
    s = Source();
    s_nodes[4].inputCount = 1;
    ExpectRefused(&s, maniBuildReasonNode, 4, 0, "a mask of one input");
    s = Source();
    s_nodes[1].inputCount = 1;
    ExpectRefused(&s, maniBuildReasonNode, 1, 0, "an additive node of one input");
    s = Source();
    s_nodes[2].clip.clip = 3;
    ExpectRefused(&s, maniBuildReasonNode, 2, 0, "a clip slot past the count");
    s = Source();
    s_nodes[5].clip.flags = maniGraphClipLeader;
    ExpectRefused(&s, maniBuildReasonNode, 5, 0, "a leader in no group");
    s = Source();
    s_nodes[6].select.parameter = 1;
    ExpectRefused(&s, maniBuildReasonNode, 6, 0, "a select by a float");
    s = Source();
    s_nodes[8].seek.trigger = 4;
    ExpectRefused(&s, maniBuildReasonNode, 8, 0, "a seek by a bool");
    s = Source();
    static const uint32_t kSelf[1] = {7};
    s_nodes[7].inputs = kSelf;
    ExpectRefused(&s, maniBuildReasonInput, 7, 0, "a node its own input");
    s = Source();
    static const uint32_t kPast[1] = {9};
    s_nodes[7].inputs = kPast;
    ExpectRefused(&s, maniBuildReasonInput, 7, 0, "an input past the count");
    s = Source();
    s.output = 9;
    ExpectRefused(&s, maniBuildReasonInput, 9, 0, "an output past the count");
    s = Source();
    s_blendWeights[1].value = -1.0f;
    ExpectRefused(&s, maniBuildReasonScalar, 0, 1, "a negative weight");
    s = Source();
    s_blendWeights[0].parameter = 2;
    ExpectRefused(&s, maniBuildReasonScalar, 0, 0, "a weight from an int");
    s = Source();
    s_layerWeights[0].value = 1.5f;
    ExpectRefused(&s, maniBuildReasonScalar, 1, 0, "a layer weight past 1");
    s = Source();
    s_chains[0].weight = 2.0f;
    ExpectRefused(&s, maniBuildReasonWeight, 4, 0, "a chain weight past 1");
}

static void TestShapeRefusals(void)
{
    // The time scale reading the seek: a cycle, met from the seek.
    maniGraphSource s = Source();
    static const uint32_t kBack[1] = {8};
    s_nodes[7].inputs = kBack;
    ExpectRefused(&s, maniBuildReasonCycle, 7, 0, "a cycle");
    s = Source();
    s.output = 8;
    ExpectRefused(&s, maniBuildReasonUnreachable, 1, 0, "a node the output does not reach");
    s = Source();
    static const uint32_t kMixed[2] = {2, 5};
    s_nodes[0].inputs = kMixed;
    ExpectRefused(&s, maniBuildReasonDifference, 0, 1, "a pose blended with differences");
    s = Source();
    static const uint32_t kPoseLayer[2] = {8, 2};
    s_nodes[1].inputs = kPoseLayer;
    ExpectRefused(&s, maniBuildReasonUnreachable, 5, 0, "the wave left out");
    s = Source();
    s_nodes[5].clip.clip = 0;
    s_clipSlots[2].clip = &s_clips[2];
    ExpectRefused(&s, maniBuildReasonDifference, 1, 1, "a layer of poses");
    s = Source();
    s.output = 5;
    s.nodeCount = 9;
    ExpectRefused(&s, maniBuildReasonUnreachable, 0, 0, "an output of differences, unreachable");
    maniBuildResult r = maniBuildGraph(NULL, &s);
    CHECK(r.result == mani_errorInvalid, "no builder");
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildGraph(builder, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == 1,
          "no source");
    maniDestroyBuilder(builder);
    s = Source();
    s.nodeCount = 0;
    ExpectRefused(&s, maniBuildReasonCount, 0, 0, "no nodes");
}

// The payload hash and identity made right after an edit.
static void Rehash(uint8_t* file)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + MANI_CONTAINER_HEADER_SIZE,
                                          s_size - MANI_CONTAINER_HEADER_SIZE));
    maniContainer c;
    CHECK(maniOpenContainer(file, s_size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
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

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
                  mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

static void ExpectFormat(uint64_t offset, uint32_t value, uint64_t want, const char* what)
{
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + offset, value);
    Rehash(s_crafted);
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_crafted, s_size, &g);
    CHECK(r.result == mani_errorFormat && r.offset == want &&
              r.tag == TestTagAt(s_crafted, s_size, want),
          what);
    if (r.offset != want)
    {
        printf("  %s: at %llu, expected %llu\n", what, (unsigned long long)r.offset,
               (unsigned long long)want);
    }
}

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniGraph graph;
    return maniOpenGraph(bytes, size, &graph);
}

static void TestOpenRefusals(void)
{
    maniGraphSource source = Source();
    CHECK(Build(&source).result == mani_success, "builds");
    uint64_t header = SectionAt(maniGraphTagHeader);
    uint64_t parameters = SectionAt(maniGraphTagParameters);
    uint64_t nodes = SectionAt(maniGraphTagNodes);
    uint64_t inputs = SectionAt(maniGraphTagInputs);
    uint64_t data = SectionAt(maniGraphTagData);
    uint64_t clips = SectionAt(maniGraphTagClips);
    uint64_t masks = SectionAt(maniGraphTagMaskWeights);
    ExpectFormat(header + 40, 0, header + 40, "no slots");
    ExpectFormat(header + 44, MANI_MAX_GRAPH_DATABASES + 1, header + 44, "too many databases");
    ExpectFormat(header + 48, MANI_MAX_GRAPH_DATABASES * MANI_MAX_GRAPH_CLIPS + 1, header + 48,
                 "too many database clips");
    ExpectFormat(header + 52, MANI_MAX_GRAPH_MODELS + 1, header + 52, "too many models");
    ExpectFormat(header + 52, 1, 12, "a model counted without its section");
    ExpectFormat(parameters, 8, parameters, "a parameter of an unknown type");
    ExpectFormat(parameters + 3 * MANI_GRAPH_PARAMETER_SIZE + 4, 1,
                 parameters + 3 * MANI_GRAPH_PARAMETER_SIZE + 4, "bounds on a trigger");
    ExpectFormat(parameters + MANI_GRAPH_PARAMETER_SIZE + 12, 99,
                 parameters + MANI_GRAPH_PARAMETER_SIZE + 8, "a name past the names");
    ExpectFormat(parameters + 24, 0x40400000u, parameters + 24, "a default past its bound");
    ExpectFormat(parameters + 5 * MANI_GRAPH_PARAMETER_SIZE + 36, 0x40000000u,
                 parameters + 5 * MANI_GRAPH_PARAMETER_SIZE + 24, "a quaternion not unit");
    ExpectFormat(nodes, 63, nodes, "an unknown kind");
    ExpectFormat(nodes + 2 * MANI_GRAPH_NODE_SIZE, 0x00010002u,
                 nodes + 2 * MANI_GRAPH_NODE_SIZE + 2, "a blend of poses marked as differences");
    ExpectFormat(nodes + 2 * MANI_GRAPH_NODE_SIZE + 20, 0, nodes + 2 * MANI_GRAPH_NODE_SIZE + 20,
                 "a blend in its input's slot");
    ExpectFormat(nodes + 2 * MANI_GRAPH_NODE_SIZE + 20, 4, nodes + 2 * MANI_GRAPH_NODE_SIZE + 20,
                 "a slot past the count");
    ExpectFormat(inputs, 2, inputs, "an input after its reader");
    ExpectFormat(nodes + MANI_GRAPH_NODE_SIZE + 4, 1, nodes + MANI_GRAPH_NODE_SIZE + 4,
                 "inputs out of turn");
    ExpectFormat(data, 3, data, "a clip slot past the count");
    ExpectFormat(data + 4, 1, data + 4, "a sync group past the count");
    ExpectFormat(data + 8, 3, data + 8, "a loop past the kinds");
    ExpectFormat(data + 16, 2, data + 16, "a speed from an int");
    ExpectFormat(data + 20, 0x7F800000u, data + 16, "a speed not finite");
    ExpectFormat(clips + 24, 2, clips + 24, "a clip's flags");
    ExpectFormat(masks, 0x3FC00000u, masks, "a mask weight past 1");
    // The identity changed after the payload hash: only it is wrong.
    memcpy(s_crafted, s_file, s_size);
    maniStoreU64(s_crafted + 40, maniLoadU64(s_crafted + 40) + 1);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, s_size, &crafted);
    CHECK(r.result == mani_errorFormat && r.offset == 40, "an identity that is not the contents'");
    maniGraph g;
    CHECK(maniOpenGraph(NULL, 0, &g).result == mani_errorInvalid &&
              maniOpenGraph(s_file, s_size, NULL).result == mani_errorInvalid &&
              maniGraphNodeCount(NULL) == 0 && maniGraphIdentity(NULL) == 0 &&
              maniGraphFindParameter(NULL, "mix", 3) == MANI_NO_PARAMETER,
          "misuses");
    CHECK(TestOtherVersionsRefused(s_file, s_size, OpenFile), "another format version refused");
}

static _Alignas(8) uint8_t s_bindingMemory[512];
static _Alignas(8) uint8_t s_instanceMemory[8192];
static _Alignas(8) uint8_t s_copyMemory[8192];
static uint8_t s_state[1024];
static uint8_t s_craftedState[1024];

// The test graph built, opened and bound; NULL when any step fails.
static maniGraphBinding* Bound(maniGraph* g)
{
    maniGraphSource source = Source();
    const maniClip* clips[4] = {&s_clips[2], NULL, &s_clips[0], &s_clips[1]};
    maniGraphBinding* binding = NULL;
    bool ok =
        Build(&source).result == mani_success &&
        maniOpenGraph(s_file, s_size, g).result == mani_success &&
        maniBindGraph(g,
                      &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, 4, 0, 0, 0, NULL,
                                         NULL, 0, 0, NULL},
                      s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) == mani_success;
    return ok ? binding : NULL;
}

static void TestBind(void)
{
    maniGraph g;
    maniGraphBinding* binding = Bound(&g);
    CHECK(binding != NULL && maniGraphBindingSize(&g) <= sizeof s_bindingMemory, "binds");
    const maniClip* missing[2] = {&s_clips[0], &s_clips[2]};
    maniGraphBinding* none = NULL;
    uint32_t unbound = 0;
    CHECK(maniBindGraph(&g,
                        &(maniGraphAssets){&s_skeleton, missing, NULL, NULL, NULL, 2, 0, 0, 0, NULL,
                                           NULL, 0, 0, NULL},
                        s_bindingMemory, sizeof s_bindingMemory, &none,
                        &unbound) == mani_errorInvalid &&
              unbound == 1 && none == NULL,
          "a slot no clip binds: the run");
    const maniClip* clips[3] = {&s_clips[0], &s_clips[1], &s_clips[2]};
    CHECK(maniBindGraph(&g,
                        &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, 3, 0, 0, 0, NULL,
                                           NULL, 0, 0, NULL},
                        s_bindingMemory, 8, &none, NULL) == mani_errorCapacity &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, 3, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory + 4, sizeof s_bindingMemory - 4, &none,
                            NULL) == mani_errorInvalid &&
              maniBindGraph(NULL,
                            &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, 3, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &none,
                            NULL) == mani_errorInvalid,
          "misuses");
    maniGraph other = g;
    other.skeletonIdentity ^= 1;
    CHECK(maniBindGraph(&other,
                        &(maniGraphAssets){&s_skeleton, clips, NULL, NULL, NULL, 3, 0, 0, 0, NULL,
                                           NULL, 0, 0, NULL},
                        s_bindingMemory, sizeof s_bindingMemory, &none, NULL) == mani_errorInvalid,
          "another skeleton");
}

static maniGraphValue Get(const maniGraphInstance* instance, uint32_t parameter)
{
    maniGraphValue v = {{0}};
    CHECK(maniGetGraphParameter(instance, parameter, &v) == mani_success, "gets");
    return v;
}

static void TestParameters(void)
{
    maniGraph g;
    maniGraphBinding* binding = Bound(&g);
    maniGraphInstance* instance = NULL;
    size_t size = maniGraphInstanceSize(binding);
    CHECK(binding != NULL && size <= sizeof s_instanceMemory &&
              maniInitGraphInstance(binding, s_instanceMemory, size, &instance) == mani_success,
          "an instance");
    CHECK(Get(instance, 0).f[0] == 1.0f && Get(instance, 1).f[0] == 0.5f &&
              Get(instance, 4).b == 1 && Get(instance, 5).f[3] == 1.0f,
          "defaults");
    CHECK(maniSetGraphParameter(instance, 0, maniGraphParameterFloat, Float(5, 0, 0, 0)) ==
                  mani_success &&
              Get(instance, 0).f[0] == 2.0f,
          "a float clamped to its maximum");
    CHECK(maniSetGraphParameter(instance, 2, maniGraphParameterInt, Int(-7)) == mani_success &&
              Get(instance, 2).i == 0,
          "an int clamped to its minimum");
    CHECK(maniSetGraphParameter(instance, 5, maniGraphParameterQuat, Float(0, 3, 0, 4)) ==
                  mani_success &&
              Get(instance, 5).f[1] == 0.6f && Get(instance, 5).f[3] == 0.8f,
          "a quaternion normalized");
    CHECK(maniSetGraphParameter(instance, 3, maniGraphParameterTrigger, Int(1)) == mani_success &&
              Get(instance, 3).b == 1,
          "a trigger set");
    maniGraphValue before = Get(instance, 1);
    CHECK(maniSetGraphParameter(instance, 1, maniGraphParameterInt, Int(1)) == mani_errorInvalid &&
              maniSetGraphParameter(instance, 1, maniGraphParameterFloat,
                                    Float(INFINITY, 0, 0, 0)) == mani_errorInvalid &&
              maniSetGraphParameter(instance, 4, maniGraphParameterBool, Int(2)) ==
                  mani_errorInvalid &&
              maniSetGraphParameter(instance, 5, maniGraphParameterQuat, Float(0, 0, 0, 0)) ==
                  mani_errorInvalid &&
              maniSetGraphParameter(instance, 6, maniGraphParameterFloat, Float(0, 0, 0, 0)) ==
                  mani_errorInvalid &&
              maniSetGraphParameter(NULL, 1, maniGraphParameterFloat, Float(0, 0, 0, 0)) ==
                  mani_errorInvalid &&
              Get(instance, 1).f[0] == before.f[0],
          "values refused, nothing changed");
    CHECK(maniInitGraphInstance(binding, s_instanceMemory, size - 1, &instance) ==
                  mani_errorCapacity &&
              maniInitGraphInstance(binding, s_instanceMemory + 4, size, &instance) ==
                  mani_errorInvalid &&
              maniGraphInstanceSize(NULL) == 0,
          "instance misuses");
}

static void ExpectState(uint64_t offset, uint32_t value, uint64_t want, const char* what,
                        maniGraphInstance* instance, size_t size)
{
    memcpy(s_craftedState, s_state, size);
    maniStoreU32(s_craftedState + offset, value);
    maniFileResult r = maniRestoreGraphInstance(instance, s_craftedState, size);
    CHECK(r.result == mani_errorFormat && r.offset == want, what);
}

static void TestSaveRestore(void)
{
    maniGraph g;
    maniGraphBinding* binding = Bound(&g);
    maniGraphInstance* instance = NULL;
    size_t size = maniGraphInstanceSize(binding);
    CHECK(binding != NULL &&
              maniInitGraphInstance(binding, s_instanceMemory, size, &instance) == mani_success,
          "an instance");
    CHECK(maniSetGraphParameter(instance, 1, maniGraphParameterFloat, Float(0.25f, 0, 0, 0)) ==
                  mani_success &&
              maniSetGraphParameter(instance, 5, maniGraphParameterQuat, Float(1, 2, 3, 4)) ==
                  mani_success,
          "parameters set");
    // A node's state as an update would leave it: the select on its
    // second input, a clip's playhead moved.
    maniGraphNodeState* states = &instance->cells[g.parameterCount].node;
    states[3].word = 1;
    states[3].weight = 0.5f;
    states[0].time = 12.75;
    size_t stateSize = maniGraphStateSize(binding);
    CHECK(stateSize == MANI_GRAPH_STATE_HEADER_SIZE + 15 * 16 && stateSize <= sizeof s_state &&
              maniSaveGraphInstance(instance, s_state, stateSize) == mani_success &&
              maniSaveGraphInstance(instance, s_state, stateSize - 1) == mani_errorCapacity,
          "saves");
    memcpy(s_copyMemory, s_instanceMemory, size);
    maniGraphInstance* copy = (maniGraphInstance*)(void*)s_copyMemory;
    maniGraphInstance* fresh = NULL;
    CHECK(maniInitGraphInstance(binding, s_instanceMemory, size, &fresh) == mani_success &&
              Get(fresh, 1).f[0] == 0.5f && Get(copy, 1).f[0] == 0.25f,
          "a memcpy is a copy, the original made over");
    CHECK(maniRestoreGraphInstance(fresh, s_state, stateSize).result == mani_success &&
              Get(fresh, 1).f[0] == 0.25f && Get(fresh, 5).f[0] == Get(copy, 5).f[0] &&
              fresh->cells[g.parameterCount + 3].node.word == 1 &&
              fresh->cells[g.parameterCount + 0].node.time == 12.75,
          "restored");
    uint64_t first = MANI_GRAPH_STATE_HEADER_SIZE;
    uint64_t nodes = first + 6 * 16;
    ExpectState(0, 0, 0, "another magic", fresh, stateSize);
    ExpectState(4, 1, 4, "a word not zero", fresh, stateSize);
    ExpectState(8, 1, 8, "another graph", fresh, stateSize);
    ExpectState(16, 1, 16, "another size", fresh, stateSize);
    ExpectState(first, 0x40400000u, first, "a float past its bounds", fresh, stateSize);
    ExpectState(first + 2 * 16 + 4, 1, first + 2 * 16, "an int's unused word", fresh, stateSize);
    ExpectState(first + 4 * 16, 2, first + 4 * 16, "a bool of 2", fresh, stateSize);
    ExpectState(first + 5 * 16, 0x40000000u, first + 5 * 16, "a quaternion not unit", fresh,
                stateSize);
    ExpectState(nodes + 3 * 16 + 4, 2, nodes + 3 * 16, "a select past its inputs", fresh,
                stateSize);
    ExpectState(nodes + 2 * 16 + 4, 1, nodes + 2 * 16, "a word on a blend", fresh, stateSize);
    ExpectState(nodes, 0xBF800000u, nodes, "a negative weight", fresh, stateSize);
    ExpectState(nodes + 12, 0x7FF00000u, nodes, "a playhead not finite", fresh, stateSize);
    CHECK(maniRestoreGraphInstance(fresh, s_state, stateSize - 1).result == mani_errorFormat &&
              maniRestoreGraphInstance(NULL, s_state, stateSize).result == mani_errorInvalid &&
              Get(fresh, 1).f[0] == 0.25f,
          "a short state; the instance unchanged by refusals");
}

static maniGraphInstance* NewInstance(maniGraph* g, maniGraphBinding** bindingOut)
{
    maniGraphBinding* binding = Bound(g);
    maniGraphInstance* instance = NULL;
    CHECK(binding != NULL &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "an instance");
    if (bindingOut != NULL)
    {
        *bindingOut = binding;
    }
    return instance;
}

static maniGraphNodeState* State(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    return &instance->cells[g->parameterCount + p].node;
}

static bool Near(double a, double b)
{
    return fabs(a - b) <= 1e-5;
}

static bool NearF(float a, double b)
{
    return Near((double)a, b);
}

// Weights from the output down, playheads moved by speed and time scale,
// across a wrap; the select's choice kept.
static void TestWeights(void)
{
    maniGraph g;
    maniGraphInstance* instance = NewInstance(&g, NULL);
    maniGraphEvent events[8];
    uint32_t count = 0;
    maniRootMove move;
    CHECK(maniUpdateGraph(instance, 0.4f, events, 8, &count, &move) == mani_success, "updates");
    // Walk 0, run 1, blend 2, select 3, mask 4, time scale 5, seek 6,
    // wave 7, the output 8.
    static const float kWeights[9] = {4.0f / 3.0f, 2.0f / 3.0f, 1, 1, 1, 1, 1, 1, 1};
    bool weights = true;
    for (uint32_t p = 0; p < 9; ++p)
    {
        weights = weights && NearF(State(instance, &g, p)->weight, (double)kWeights[p]);
    }
    CHECK(weights, "weights shared down: the walk by both blend and select");
    CHECK(State(instance, &g, 0)->rootWeight == 1.0f &&
              State(instance, &g, 1)->rootWeight == 0.0f &&
              State(instance, &g, 7)->rootWeight == 0.0f,
          "root weights: the mask's outside (no root motion bone), no layer");
    CHECK(Near(State(instance, &g, 0)->time, 0.6) && Near(State(instance, &g, 1)->time, 0.6) &&
              Near(State(instance, &g, 7)->time, 0.4),
          "playheads by the time scale above them");
    CHECK(maniUpdateGraph(instance, 0.5f, NULL, 0, &count, NULL) == mani_success &&
              Near(State(instance, &g, 0)->time, 0.35) && count == 1,
          "a wrap: the walk at 0.35, its first event counted past no buffer");
    CHECK(maniSetGraphParameter(instance, 2, maniGraphParameterInt, Int(1)) == mani_success &&
              maniUpdateGraph(instance, 0.0f, NULL, 0, NULL, NULL) == mani_success &&
              State(instance, &g, 3)->word == 1 && NearF(State(instance, &g, 1)->weight, 5.0 / 3.0),
          "the select's choice: the run");
    CHECK(maniUpdateGraph(NULL, 0.1f, NULL, 0, NULL, NULL) == mani_errorInvalid &&
              maniUpdateGraph(instance, NAN, NULL, 0, NULL, NULL) == mani_errorInvalid &&
              maniUpdateGraph(instance, 0.1f, NULL, 3, NULL, NULL) == mani_errorInvalid,
          "misuses");
}

static void TestEvents(void)
{
    maniGraph g;
    maniGraphInstance* instance = NewInstance(&g, NULL);
    maniGraphEvent events[4];
    uint32_t count = 0;
    CHECK(maniUpdateGraph(instance, 0.4f, events, 4, &count, NULL) == mani_success && count == 2,
          "the walk's two steps from 0 to 0.6");
    CHECK(events[0].node == 0 && events[0].clip == 0 && events[0].event == 0 &&
              events[0].payload == 7 && events[0].direction == 1 && events[0].time == 0.0f &&
              NearF(events[0].weight, 4.0 / 3.0) && events[1].event == 1 && events[1].payload == 8,
          "records: node, slot, event, payload, weight, time, direction");
    CHECK(maniUpdateGraph(instance, -0.4f, events, 4, &count, NULL) == mani_success && count == 1 &&
              events[0].event == 1 && events[0].direction == -1,
          "backward from 0.6 to 0: the right step at 0.5 only");
    CHECK(maniUpdateGraph(instance, 0.4f, events, 1, &count, NULL) == mani_success && count == 2 &&
              events[0].event == 0,
          "past the capacity: the true count");
    // A million seconds at 1.5: 1.5 million loops of two steps each.
    CHECK(maniUpdateGraph(instance, 1.0e6f, events, 4, &count, NULL) == mani_success &&
              count >= 2999999 && count <= 3000001,
          "a huge step counted, not listed");
    CHECK(maniSetGraphParameter(instance, 3, maniGraphParameterTrigger, Int(1)) == mani_success &&
              maniUpdateGraph(instance, 0.3f, events, 4, &count, NULL) == mani_success &&
              count == 0 && Near(State(instance, &g, 0)->time, 0.25) &&
              Near(State(instance, &g, 1)->time, 0.25),
          "a seek: playheads at its time, no events");
    maniGraphValue trigger = Get(instance, 3);
    CHECK(trigger.b == 0, "the trigger consumed");
}

static _Alignas(16) uint8_t s_rootClipFiles[3][8192];
static maniClip s_rootClips[3];

// The walk with its hips' travel along x as root motion (0, from bone 0;
// 2, from the spine), and the run without.
static void OpenRootClips(void)
{
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const maniRootMotionSource kHips = {0, maniRootAxisX, 0, 0, 0, 0, 0};
    static const maniRootMotionSource kSpine = {1, maniRootAxisX, 0, 0, 0, 0, 0};
    const maniBoneTrackSource tracks[3] = {
        {0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone},
        {0, {kTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone},
        {1, {kTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone},
    };
    for (int i = 0; i < 3; ++i)
    {
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = &tracks[i],
                                 .boneTrackCount = 1,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 10.0f,
                                 .rootMotion = i == 0 ? &kHips : (i == 2 ? &kSpine : NULL)};
        maniBuilder* builder = NewBuilder();
        size_t size = 0;
        CHECK(maniBuildClip(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_rootClipFiles[i], sizeof s_rootClipFiles[i], &size) ==
                      mani_success &&
                  maniOpenClip(s_rootClipFiles[i], size, &s_rootClips[i]).result == mani_success,
              "a root motion clip");
        maniDestroyBuilder(builder);
    }
}

// A blend of a walk with root motion and a run without, by a parameter.
static void TestRootMotion(void)
{
    OpenRootClips();
    maniGraphParameterSource parameters[1] = {
        Parameter("mix", maniGraphParameterFloat, Float(0.5f, 0, 0, 0))};
    maniGraphClipSource clips[2] = {{&s_rootClips[0], "walk", 4, NULL},
                                    {&s_rootClips[1], "run", 3, NULL}};
    static const uint32_t kInputs[2] = {0, 1};
    const maniGraphScalar weights[2] = {{0, 0.0f}, {MANI_NO_PARAMETER, 0.5f}};
    const maniGraphScalar one = {MANI_NO_PARAMETER, 1.0f};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeClip, 0, NULL, .clip = {0, MANI_NO_SYNC_GROUP, one, maniGraphLoopClip, 0}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, MANI_NO_SYNC_GROUP, one, maniGraphLoopClip, 0}},
        {maniGraphNodeBlend, 2, kInputs, .blend = {weights, 0}},
    };
    maniGraphSource source = {&s_skeleton, parameters, clips, NULL, nodes, 1,    2,    0,   3,
                              2,           0,          NULL,  0,    0,     NULL, NULL, NULL};
    maniGraph g;
    const maniClip* bound[2] = {&s_rootClips[0], &s_rootClips[1]};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(Build(&source).result == mani_success &&
              maniOpenGraph(s_file, s_size, &g).result == mani_success &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, bound, NULL, NULL, NULL, 2, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a root motion graph");
    maniRootMove move;
    // The walk moves 1 a second; half the weight, half a second: 0.25.
    CHECK(maniUpdateGraph(instance, 0.5f, NULL, 0, NULL, &move) == mani_success &&
              Near((double)move.translation.x, 0.25) && move.translation.z == 0.0f &&
              move.rotation.w == 1.0f,
          "the walk's travel blended with the run's none");
    CHECK(maniSetGraphParameter(instance, 0, maniGraphParameterFloat, Float(1.5f, 0, 0, 0)) ==
                  mani_success &&
              maniUpdateGraph(instance, 0.8f, NULL, 0, NULL, &move) == mani_success &&
              Near((double)move.translation.x, 0.6),
          "three quarters of the walk across its wrap");
    clips[1].clip = &s_rootClips[2];
    bound[1] = &s_rootClips[2];
    CHECK(Build(&source).result == mani_success &&
              maniOpenGraph(s_file, s_size, &g).result == mani_success &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, bound, NULL, NULL, NULL, 2, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &binding,
                            NULL) == mani_errorInvalid,
          "clips moving different bones refused");
}

static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poseMemory[6][4096];

static maniPose* PoseAt(int i)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], &pose) == mani_success,
          "a pose");
    return pose;
}

static bool SamePose(const maniPose* a, const maniPose* b)
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

// The pose the test graph should give: walk and run at 0.6, blended a
// third and two thirds on the spine and arm, the walk (the select's
// choice) on the hips, and the wave at 0.4 added.
static void Reference(maniPose* out)
{
    maniPose* walk = PoseAt(1);
    maniPose* run = PoseAt(2);
    maniPose* blend = PoseAt(3);
    maniPose* wave = PoseAt(4);
    float inside[3];
    float outside[3];
    const maniMaskChain chain = {1, 1.0f, true};
    CHECK(maniBuildMask(&s_skeleton, &chain, 1, 0.0f, inside, 3, NULL) == mani_success, "mask");
    for (int b = 0; b < 3; ++b)
    {
        outside[b] = 1.0f - inside[b];
    }
    CHECK(maniSampleClip(&s_clips[0], &s_skeleton, 0.6f, walk) == mani_success &&
              maniSampleClip(&s_clips[1], &s_skeleton, 0.6f, run) == mani_success &&
              maniSampleClip(&s_clips[2], &s_skeleton, 0.4f, wave) == mani_success,
          "samples");
    const maniBlendLayer two[2] = {{walk, 0.5f / 1.5f, NULL}, {run, 1.0f / 1.5f, NULL}};
    const maniBlendLayer masked[2] = {{blend, 1.0f, inside}, {walk, 1.0f, outside}};
    const maniBlendLayer layer = {wave, 1.0f, NULL};
    CHECK(maniBlendPoses(&s_skeleton, two, 2, MANI_BLEND_THRESHOLD, blend) == mani_success &&
              maniBlendPoses(&s_skeleton, masked, 2, MANI_BLEND_THRESHOLD, out) == mani_success &&
              maniAddPoses(&s_skeleton, out, &layer, 1) == mani_success,
          "the reference");
}

static void TestEvaluate(void)
{
    maniGraph g;
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NewInstance(&g, &binding);
    size_t size = maniGraphScratchSize(binding);
    maniPose* pose = PoseAt(0);
    maniPose* rest = PoseAt(5);
    CHECK(size > 0 && size <= sizeof s_scratch &&
              maniEvaluateGraph(instance, s_scratch, size, pose) == mani_success &&
              SamePose(pose, rest),
          "the rest before the first update");
    maniPose* expected = PoseAt(5);
    Reference(expected);
    CHECK(maniUpdateGraph(instance, 0.4f, NULL, 0, NULL, NULL) == mani_success &&
              maniEvaluateGraph(instance, s_scratch, size, pose) == mani_success &&
              SamePose(pose, expected),
          "the graph's pose, bit for bit with the blending functions'");
    // Again, with the scratch's poses already made, and from a copy.
    memcpy(s_copyMemory, s_instanceMemory, maniGraphInstanceSize(binding));
    maniGraphInstance* copy = (maniGraphInstance*)(void*)s_copyMemory;
    maniPose* again = PoseAt(1);
    CHECK(maniEvaluateGraph(copy, s_scratch, size, again) == mani_success &&
              SamePose(again, expected),
          "a copy evaluates the same");
    CHECK(maniEvaluateGraph(instance, s_scratch, size - 1, pose) == mani_errorCapacity &&
              maniEvaluateGraph(instance, s_scratch + 16, size, pose) == mani_errorInvalid &&
              maniEvaluateGraph(NULL, s_scratch, size, pose) == mani_errorInvalid &&
              maniEvaluateGraph(instance, s_scratch, size, NULL) == mani_errorInvalid &&
              maniGraphScratchSize(NULL) == 0,
          "misuses");
}

static _Alignas(16) uint8_t s_longFile[8192];
static maniClip s_long;

// A clip twice the walk's length, with an event at its middle.
static void OpenLong(void)
{
    static const float kLongTimes[2] = {0.0f, 2.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const maniEventSource kMiddle = {1.0f, 9, "middle", 6};
    const maniBoneTrackSource track = {
        0, {kLongTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone};
    maniClipSource source = {.skeleton = &s_skeleton,
                             .boneTracks = &track,
                             .boneTrackCount = 1,
                             .events = &kMiddle,
                             .eventCount = 1,
                             .loop = maniLoopRepeat,
                             .sampleRate = 10.0f};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_longFile, sizeof s_longFile, &size) == mani_success &&
              maniOpenClip(s_longFile, size, &s_long).result == mani_success,
          "the long clip");
    maniDestroyBuilder(builder);
}

// The walk and the long clip blended in one group: the walk by the mix
// parameter, the long clip by 0.2; flags for the group and the long node.
static maniGraphInstance* SyncInstance(uint32_t groupFlags, uint32_t longFlags, maniGraph* g)
{
    static maniGraphParameterSource parameters[1];
    static maniGraphClipSource clips[2];
    static maniGraphSyncGroupSource groups[1];
    static maniGraphScalar weights[2];
    static maniGraphNodeSource nodes[3];
    static const uint32_t kInputs[2] = {0, 1};
    parameters[0] = Parameter("mix", maniGraphParameterFloat, Float(0.8f, 0, 0, 0));
    clips[0] = (maniGraphClipSource){&s_clips[0], "walk", 4, NULL};
    clips[1] = (maniGraphClipSource){&s_long, "long", 4, NULL};
    groups[0] = (maniGraphSyncGroupSource){"steps", 5, groupFlags};
    weights[0] = (maniGraphScalar){0, 0.0f};
    weights[1] = (maniGraphScalar){MANI_NO_PARAMETER, 0.2f};
    const maniGraphScalar one = {MANI_NO_PARAMETER, 1.0f};
    nodes[0] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                     .clip = {0, 0, one, maniGraphLoopClip, 0}};
    nodes[1] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                     .clip = {1, 0, one, maniGraphLoopClip, longFlags}};
    nodes[2] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kInputs, .blend = {weights, 0}};
    maniGraphSource source = {&s_skeleton, parameters, clips, groups, nodes, 1,    2,    1,   3,
                              2,           0,          NULL,  0,      0,     NULL, NULL, NULL};
    const maniClip* bound[2] = {&s_clips[0], &s_long};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(Build(&source).result == mani_success &&
              maniOpenGraph(s_file, s_size, g).result == mani_success &&
              maniBindGraph(g,
                            &(maniGraphAssets){&s_skeleton, bound, NULL, NULL, NULL, 2, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a sync graph");
    return instance;
}

static void TestSync(void)
{
    OpenLong();
    maniGraph g;
    maniGraphInstance* instance = SyncInstance(0, 0, &g);
    maniGraphEvent events[8];
    uint32_t count = 0;
    CHECK(maniUpdateGraph(instance, 0.25f, events, 8, &count, NULL) == mani_success &&
              Near(State(instance, &g, 0)->time, 0.25) && Near(State(instance, &g, 1)->time, 0.5) &&
              count == 1 && events[0].node == 0,
          "the walk leads by weight; the long clip a quarter through, its events quiet");
    CHECK(maniSetGraphParameter(instance, 0, maniGraphParameterFloat, Float(0.1f, 0, 0, 0)) ==
                  mani_success &&
              maniUpdateGraph(instance, 0.5f, events, 8, &count, NULL) == mani_success &&
              Near(State(instance, &g, 1)->time, 1.0) && Near(State(instance, &g, 0)->time, 0.5) &&
              count == 0,
          "the long clip leads once it weighs more; the walk follows its phase");
    CHECK(maniUpdateGraph(instance, 0.5f, events, 8, &count, NULL) == mani_success && count == 1 &&
              events[0].node == 1 && events[0].payload == 9,
          "the leader's event at its middle; the walk's quiet");
    instance = SyncInstance(0, maniGraphClipLeader, &g);
    CHECK(maniUpdateGraph(instance, 0.5f, events, 8, &count, NULL) == mani_success &&
              Near(State(instance, &g, 1)->time, 0.5) && Near(State(instance, &g, 0)->time, 0.25),
          "a declared leader leads whatever its weight");
    instance = SyncInstance(maniGraphSyncAllEvents, 0, &g);
    CHECK(maniUpdateGraph(instance, 0.75f, events, 8, &count, NULL) == mani_success && count == 3,
          "every member's events when the group asks: two steps and the middle");
}

static _Alignas(16) uint8_t s_flatFile[16384];

// Locomotion as a subgraph: walk and run blended by its pace (the
// parent's mix), nested once more around a time scale.
static void TestSubgraph(void)
{
    static const maniGraphParameterSource kPace[1] = {
        {"pace", 4, maniGraphParameterFloat, {{0.5f}}, {{0}}, {{0}}, 0}};
    static const maniGraphClipSource kLegs[2] = {{&s_clips[0], "walk", 4, NULL},
                                                 {&s_clips[1], "run", 3, NULL}};
    static const uint32_t kPair[2] = {0, 1};
    static const maniGraphScalar kWeights[2] = {{0, 0.0f}, {MANI_NO_PARAMETER, 1.0f}};
    static const maniGraphScalar kOne = {MANI_NO_PARAMETER, 1.0f};
    static const maniGraphNodeSource kLocomotion[3] = {
        {maniGraphNodeClip, 0, NULL, .clip = {0, MANI_NO_SYNC_GROUP, kOne, maniGraphLoopClip, 0}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, MANI_NO_SYNC_GROUP, kOne, maniGraphLoopClip, 0}},
        {maniGraphNodeBlend, 2, kPair, .blend = {kWeights, 0}},
    };
    static const maniGraphSource kInner = {&s_skeleton, kPace, kLegs, NULL, kLocomotion, 1,
                                           2,           0,     3,     2,    0,           NULL,
                                           0,           0,     NULL,  NULL, NULL};
    // The middle graph: the inner one, time scaled.
    static const uint32_t kPaceMap[1] = {0};
    static const uint32_t kFirst[1] = {0};
    static const maniGraphNodeSource kMiddle[2] = {
        {maniGraphNodeSubgraph, 0, NULL, .subgraph = {&kInner, kPaceMap}},
        {maniGraphNodeTimeScale, 1, kFirst, .timeScale = {{MANI_NO_PARAMETER, 1.5f}}},
    };
    static const maniGraphSource kOuter = {
        &s_skeleton, kPace, NULL, NULL, kMiddle, 1, 0, 0, 2, 1, 0, NULL, 0, 0, NULL, NULL, NULL};
    // The top: the middle graph with the wave added.
    maniGraphParameterSource top[1] = {
        Parameter("mix", maniGraphParameterFloat, Float(0.5f, 0, 0, 0))};
    const maniGraphClipSource waves[1] = {{&s_clips[2], "wave", 4, NULL}};
    static const uint32_t kMixMap[1] = {0};
    static const uint32_t kLayers[2] = {0, 1};
    const maniGraphScalar layerWeights[1] = {kOne};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeSubgraph, 0, NULL, .subgraph = {&kOuter, kMixMap}},
        {maniGraphNodeClip, 0, NULL, .clip = {0, MANI_NO_SYNC_GROUP, kOne, maniGraphLoopClip, 0}},
        {maniGraphNodeAdditive, 2, kLayers, .additive = {layerWeights}},
    };
    maniGraphSource source = {&s_skeleton, top, waves, NULL, nodes, 1,    1,    0,   3,
                              2,           0,   NULL,  0,    0,     NULL, NULL, NULL};
    // The same written flat: walk, run, blend, time scale, wave, output.
    const maniGraphClipSource flatClips[3] = {{&s_clips[2], "wave", 4, NULL},
                                              {&s_clips[0], "walk", 4, NULL},
                                              {&s_clips[1], "run", 3, NULL}};
    static const uint32_t kFlatPair[2] = {0, 1};
    static const uint32_t kFlatScale[1] = {2};
    static const uint32_t kFlatAdd[2] = {3, 4};
    maniGraphNodeSource flat[6] = {
        {maniGraphNodeClip, 0, NULL, .clip = {1, MANI_NO_SYNC_GROUP, kOne, maniGraphLoopClip, 0}},
        {maniGraphNodeClip, 0, NULL, .clip = {2, MANI_NO_SYNC_GROUP, kOne, maniGraphLoopClip, 0}},
        {maniGraphNodeBlend, 2, kFlatPair, .blend = {kWeights, 0}},
        {maniGraphNodeTimeScale, 1, kFlatScale, .timeScale = {{MANI_NO_PARAMETER, 1.5f}}},
        {maniGraphNodeClip, 0, NULL, .clip = {0, MANI_NO_SYNC_GROUP, kOne, maniGraphLoopClip, 0}},
        {maniGraphNodeAdditive, 2, kFlatAdd, .additive = {layerWeights}},
    };
    maniGraphSource flatSource = {&s_skeleton, top, flatClips, NULL, flat, 1,    3,    0,   6,
                                  5,           0,   NULL,      0,    0,    NULL, NULL, NULL};
    size_t flatSize = 0;
    maniGraph flatGraph;
    CHECK(Build(&flatSource).result == mani_success, "the flat graph builds");
    memcpy(s_flatFile, s_file, s_size);
    flatSize = s_size;
    maniGraph g;
    CHECK(Build(&source).result == mani_success && s_size == flatSize &&
              maniOpenGraph(s_file, s_size, &g).result == mani_success &&
              maniOpenGraph(s_flatFile, flatSize, &flatGraph).result == mani_success &&
              maniGraphNodeCount(&g) == 6 && maniGraphClipCount(&g) == 3,
          "two subgraphs inlined: six nodes, three clips");
    // Played alike: the same poses after the same updates.
    const maniClip* bound[3] = {&s_clips[0], &s_clips[1], &s_clips[2]};
    static _Alignas(8) uint8_t bindings[2][512];
    static _Alignas(8) uint8_t instances[2][4096];
    maniGraphBinding* b[2] = {NULL, NULL};
    maniGraphInstance* i[2] = {NULL, NULL};
    maniPose* poses[2] = {PoseAt(0), PoseAt(1)};
    const maniGraph* graphs[2] = {&g, &flatGraph};
    bool same = true;
    for (int k = 0; k < 2; ++k)
    {
        same =
            same &&
            maniBindGraph(graphs[k],
                          &(maniGraphAssets){&s_skeleton, bound, NULL, NULL, NULL, 3, 0, 0, 0, NULL,
                                             NULL, 0, 0, NULL},
                          bindings[k], sizeof bindings[k], &b[k], NULL) == mani_success &&
            maniInitGraphInstance(b[k], instances[k], sizeof instances[k], &i[k]) == mani_success &&
            maniSetGraphParameter(i[k], 0, maniGraphParameterFloat, Float(2, 0, 0, 0)) ==
                mani_success &&
            maniUpdateGraph(i[k], 0.3f, NULL, 0, NULL, NULL) == mani_success &&
            maniEvaluateGraph(i[k], s_scratch, sizeof s_scratch, poses[k]) == mani_success;
    }
    CHECK(same && SamePose(poses[0], poses[1]), "the inlined graph plays as the flat one");
    // Refusals: a map to a parameter of another type, a graph of itself.
    top[0].type = maniGraphParameterInt;
    top[0].defaultValue = Int(0);
    ExpectRefused(&source, maniBuildReasonNode, 0, 0, "a subgraph's parameter of another type");
    top[0] = Parameter("mix", maniGraphParameterFloat, Float(0.5f, 0, 0, 0));
    nodes[0].subgraph.graph = &source;
    // A graph inside itself nests past MANI_MAX_GRAPH_DEPTH.
    maniBuildResult inside = Build(&source);
    CHECK(inside.result == mani_errorLimit && inside.reason == maniBuildReasonCount,
          "a graph inside itself");
}

static _Alignas(16) uint8_t s_footFiles[2][8192];
static maniClip s_feet[2];

// A step of a second (left at 0, right at 0.5) and one of two seconds
// that starts between its feet (right at 0.5, left at 1.5).
static void OpenFeet(void)
{
    static const float kLongTimes[2] = {0.0f, 2.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const maniEventSource kShort[2] = {{0.0f, 0, "left", 4}, {0.5f, 0, "right", 5}};
    static const maniEventSource kLong[2] = {{0.5f, 0, "right", 5}, {1.5f, 0, "left", 4}};
    static const maniEventSource kMiddle = {1.0f, 3, "middle", 6};
    const maniBoneTrackSource tracks[2] = {
        {0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone},
        {0, {kLongTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone},
    };
    for (int i = 0; i < 2; ++i)
    {
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = &tracks[i],
                                 .boneTrackCount = 1,
                                 .markers = i == 0 ? kShort : kLong,
                                 .markerCount = 2,
                                 .events = i == 1 ? &kMiddle : NULL,
                                 .eventCount = i == 1 ? 1u : 0u,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 10.0f};
        maniBuilder* builder = NewBuilder();
        size_t size = 0;
        CHECK(maniBuildClip(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_footFiles[i], sizeof s_footFiles[i], &size) ==
                      mani_success &&
                  maniOpenClip(s_footFiles[i], size, &s_feet[i]).result == mani_success,
              "a clip with feet");
        maniDestroyBuilder(builder);
    }
}

static void TestMarkerSync(void)
{
    OpenFeet();
    maniGraphClipSource clips[2] = {{&s_feet[0], "short", 5, NULL}, {&s_feet[1], "long", 4, NULL}};
    maniGraphSyncGroupSource groups[1] = {{"feet", 4, maniGraphSyncAllEvents}};
    static const uint32_t kInputs[2] = {0, 1};
    const maniGraphScalar weights[2] = {{MANI_NO_PARAMETER, 0.5f}, {MANI_NO_PARAMETER, 0.5f}};
    const maniGraphScalar one = {MANI_NO_PARAMETER, 1.0f};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeClip, 0, NULL, .clip = {0, 0, one, maniGraphLoopClip, maniGraphClipLeader}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, 0, one, maniGraphLoopClip, 0}},
        {maniGraphNodeBlend, 2, kInputs, .blend = {weights, 0}},
    };
    maniGraphSource source = {&s_skeleton, NULL, clips, groups, nodes, 0,    2,    1,   3,
                              2,           0,    NULL,  0,      0,     NULL, NULL, NULL};
    maniGraph g;
    const maniClip* bound[2] = {&s_feet[0], &s_feet[1]};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(Build(&source).result == mani_success &&
              maniOpenGraph(s_file, s_size, &g).result == mani_success &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, bound, NULL, NULL, NULL, 2, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a marker graph");
    // The short clip from its left foot to halfway to its right: the long
    // one placed on its left foot, then halfway to its right (at 2, so 0).
    // Placed on its left foot without a sound: its middle is not passed.
    uint32_t count = 0;
    CHECK(maniUpdateGraph(instance, 0.25f, NULL, 0, &count, NULL) == mani_success &&
              Near(State(instance, &g, 0)->time, 0.25) && Near(State(instance, &g, 1)->time, 0.0) &&
              count == 0,
          "the long clip aligned by names, then halfway between its feet");
    CHECK(maniUpdateGraph(instance, 0.5f, NULL, 0, &count, NULL) == mani_success &&
              Near(State(instance, &g, 0)->time, 0.75) && Near(State(instance, &g, 1)->time, 1.0) &&
              count == 0,
          "past the right foot to halfway to the left, in each clip's own spacing");
    CHECK(maniUpdateGraph(instance, -0.5f, NULL, 0, NULL, NULL) == mani_success &&
              Near(State(instance, &g, 0)->time, 0.25) && Near(State(instance, &g, 1)->time, 0.0),
          "backward through the right foot again");
    CHECK(maniUpdateGraph(instance, 10.25f, NULL, 0, NULL, NULL) == mani_success &&
              Near(State(instance, &g, 0)->time, 0.5) && Near(State(instance, &g, 1)->time, 0.5),
          "ten loops and a quarter: both on the right foot");
}

// The marker graph under time scales of 1000 and 1000, the largest
// product a path may take, into s_file.
static maniBuildResult BuildLargeSteps(void)
{
    OpenFeet();
    static const maniGraphClipSource kClips[2] = {{&s_feet[0], "short", 5, NULL},
                                                  {&s_feet[1], "long", 4, NULL}};
    static const maniGraphSyncGroupSource kGroups[1] = {{"feet", 4, maniGraphSyncAllEvents}};
    static const uint32_t kInputs[2] = {0, 1};
    static const uint32_t kBlend[1] = {2};
    static const uint32_t kScale[1] = {3};
    static const maniGraphScalar kWeights[2] = {{MANI_NO_PARAMETER, 0.5f},
                                                {MANI_NO_PARAMETER, 0.5f}};
    const maniGraphScalar one = {MANI_NO_PARAMETER, 1.0f};
    const maniGraphScalar thousand = {MANI_NO_PARAMETER, 1000.0f};
    const maniGraphNodeSource nodes[5] = {
        {maniGraphNodeClip, 0, NULL, .clip = {0, 0, one, maniGraphLoopClip, maniGraphClipLeader}},
        {maniGraphNodeClip, 0, NULL, .clip = {1, 0, one, maniGraphLoopClip, 0}},
        {maniGraphNodeBlend, 2, kInputs, .blend = {kWeights, 0}},
        {maniGraphNodeTimeScale, 1, kBlend, .timeScale = {thousand}},
        {maniGraphNodeTimeScale, 1, kScale, .timeScale = {thousand}},
    };
    maniGraphSource source = {&s_skeleton, NULL, kClips, kGroups, nodes, 0,    2,    1,   5,
                              4,           0,    NULL,   0,       0,     NULL, NULL, NULL};
    return Build(&source);
}

// Run as test_graph large-steps, alone, under a short timeout: the
// largest deltas a host can pass, both ways, and others across many
// loops, under the largest time factor: every update succeeds, and both
// playheads stay within their clips.
static void TestLargeSteps(void)
{
    maniGraph g;
    const maniClip* bound[2] = {&s_feet[0], &s_feet[1]};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(BuildLargeSteps().result == mani_success &&
              maniOpenGraph(s_file, s_size, &g).result == mani_success &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_skeleton, bound, NULL, NULL, NULL, 2, 0, 0, 0,
                                               NULL, NULL, 0, 0, NULL},
                            s_bindingMemory, sizeof s_bindingMemory, &binding,
                            NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a marker graph of the largest time factor");
    if (instance == NULL)
    {
        return;
    }
    static const float kDeltas[6] = {FLT_MAX, -FLT_MAX, 1.0e16f, -3.3e9f, 0.37f, 0.0f};
    for (int k = 0; k < 6; ++k)
    {
        maniGraphEvent events[4];
        uint32_t count = 0;
        maniRootMove move;
        bool updated =
            maniUpdateGraph(instance, kDeltas[k], events, 4, &count, &move) == mani_success;
        double leader = State(instance, &g, 0)->time;
        double follower = State(instance, &g, 1)->time;
        CHECK(updated && leader >= 0.0 && leader <= (double)maniClipDuration(&s_feet[0]) &&
                  follower >= 0.0 && follower <= (double)maniClipDuration(&s_feet[1]),
              "an update across many loops, both playheads within their clips");
    }
}

static int WriteLargeSeed(const char* path)
{
    FILE* f = BuildLargeSteps().result == mani_success ? fopen(path, "wb") : NULL;
    bool written = f != NULL && fwrite(s_file, 1, s_size, f) == s_size;
    if (f != NULL)
    {
        written = fclose(f) == 0 && written;
    }
    return written ? 0 : 1;
}

static void TestPinned(void)
{
    maniGraphSource source = Source();
    CHECK(Build(&source).result == mani_success, "builds");
    // The writer's library version (bytes 16 to 21) changes with every release.
    memset(s_file + 16, 0, 6);
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    maniGraph g;
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NewInstance(&g, &binding);
    size_t size = maniGraphScratchSize(binding);
    maniPose* pose = PoseAt(0);
    for (int step = 0; step < 8; ++step)
    {
        maniGraphEvent events[8];
        uint32_t count = 0;
        maniRootMove move;
        CHECK(maniUpdateGraph(instance, 0.27f, events, 8, &count, &move) == mani_success &&
                  maniEvaluateGraph(instance, s_scratch, size, pose) == mani_success,
              "steps");
        maniTransform locals[3];
        CHECK(maniPoseReadLocals(pose, 0, 3, locals) == mani_success, "reads");
        hash = maniHash64(hash, locals, (int32_t)sizeof locals);
        const float parts[7] = {move.translation.x, move.translation.y, move.translation.z,
                                move.rotation.x,    move.rotation.y,    move.rotation.z,
                                move.rotation.w};
        hash = maniHash64(hash, parts, (int32_t)sizeof parts);
        for (uint32_t e = 0; e < count && e < 8; ++e)
        {
            const maniGraphEvent* x = &events[e];
            const uint32_t fields[7] = {x->node,
                                        x->clip,
                                        x->event,
                                        (uint32_t)x->payload,
                                        (uint32_t)x->direction,
                                        (uint32_t)x->nameHash,
                                        (uint32_t)(x->nameHash >> 32)};
            const float values[2] = {x->weight, x->time};
            hash = maniHash64(hash, fields, (int32_t)sizeof fields);
            hash = maniHash64(hash, values, (int32_t)sizeof values);
        }
    }
    size_t stateSize = maniGraphStateSize(binding);
    CHECK(stateSize <= sizeof s_state &&
              maniSaveGraphInstance(instance, s_state, stateSize) == mani_success,
          "saves");
    hash = maniHash64(hash, s_state, (int32_t)stateSize);
    CHECK(hash == PINNED_SCENE_HASH, "the scene's bits the same on every platform");
    if (hash != PINNED_SCENE_HASH)
    {
        printf("scene hash 0x%016llx\n", (unsigned long long)hash);
    }
}

static int WriteSeed(const char* path)
{
    maniGraphSource source = Source();
    FILE* f = Build(&source).result == mani_success ? fopen(path, "wb") : NULL;
    bool written = f != NULL && fwrite(s_file, 1, s_size, f) == s_size;
    if (f != NULL)
    {
        written = fclose(f) == 0 && written;
    }
    return written ? 0 : 1;
}

int main(int argc, char** argv)
{
    OpenSkeleton();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    if (argc == 3 && strcmp(argv[1], "--seed-large") == 0)
    {
        return WriteLargeSeed(argv[2]);
    }
    if (argc == 2 && strcmp(argv[1], "large-steps") == 0)
    {
        TestLargeSteps();
        return s_failures == 0 ? 0 : 1;
    }
    TestBuild();
    TestParameterRefusals();
    TestNodeRefusals();
    TestShapeRefusals();
    TestOpenRefusals();
    TestBind();
    TestParameters();
    TestSaveRestore();
    TestWeights();
    TestEvents();
    TestRootMotion();
    TestEvaluate();
    TestSync();
    TestSubgraph();
    TestMarkerSync();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
