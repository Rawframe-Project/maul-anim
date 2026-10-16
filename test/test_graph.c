// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph files (mani-0023): a graph built against a skeleton and its
// clips, its nodes ordered depth first from the output, poses and
// differences marked, slots reused; every refusal of the builder and of
// opening; the bytes pinned. Run as test_graph --seed <file> it writes a
// seed for the graph fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PINNED_GRAPH_HASH 0x74d949fc8a152768ull

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
        {0, 0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone},
        {0, 0, {kTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone},
        {2, 0, kNone, {kTimes, kWave, 2, maniInterpolationLinear}, kNone},
    };
    for (int i = 0; i < 3; ++i)
    {
        maniClipSource source = {.skeleton = &s_skeleton,
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
    maniSkeletonSource source = {kBones, 3, 0};
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
    return (maniGraphParameterSource){name, (uint32_t)strlen(name), type, value, Int(0), Int(0), 0,
                                      0};
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
    s_clipSlots[0] = (maniGraphClipSource){&s_clips[0], "walk", 4, 0};
    s_clipSlots[1] = (maniGraphClipSource){&s_clips[1], "run", 3, 0};
    s_clipSlots[2] = (maniGraphClipSource){&s_clips[2], "wave", 4, 0};
    s_groups[0] = (maniGraphSyncGroupSource){"feet", 4, 0};
    s_blendWeights[0] = (maniGraphScalar){1, 0.0f};
    s_blendWeights[1] = (maniGraphScalar){MANI_NO_PARAMETER, 1.0f};
    s_layerWeights[0] = (maniGraphScalar){MANI_NO_PARAMETER, 1.0f};
    s_chains[0] = (maniMaskChain){1, 1.0f, true};
    const maniGraphScalar one = {MANI_NO_PARAMETER, 1.0f};
    s_nodes[0] =
        (maniGraphNodeSource){maniGraphNodeBlend, 2, kBlendInputs, .blend = {s_blendWeights, 0, 0}};
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
        &s_skeleton, s_parameters, s_clipSlots, s_groups, s_nodes, 6, 3, 1, 9, 1, 0};
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
    s_nodes[6].kind = 9;
    ExpectRefused(&s, maniBuildReasonNode, 6, 0, "an unknown kind");
    s = Source();
    s_nodes[4].inputCount = 1;
    ExpectRefused(&s, maniBuildReasonNode, 4, 0, "a mask of one input");
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
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, s_size - 64));
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
    CHECK(r.result == mani_errorFormat && r.offset == want, what);
    if (r.offset != want)
    {
        printf("  %s: at %llu, expected %llu\n", what, (unsigned long long)r.offset,
               (unsigned long long)want);
    }
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
    ExpectFormat(header + 44, 1, header + 44, "the header's last word not zero");
    ExpectFormat(parameters, 7, parameters, "a parameter of an unknown type");
    ExpectFormat(parameters + 3 * MANI_GRAPH_PARAMETER_SIZE + 4, 1,
                 parameters + 3 * MANI_GRAPH_PARAMETER_SIZE + 4, "bounds on a trigger");
    ExpectFormat(parameters + MANI_GRAPH_PARAMETER_SIZE + 12, 99,
                 parameters + MANI_GRAPH_PARAMETER_SIZE + 8, "a name past the names");
    ExpectFormat(parameters + 24, 0x40400000u, parameters + 24, "a default past its bound");
    ExpectFormat(parameters + 5 * MANI_GRAPH_PARAMETER_SIZE + 36, 0x40000000u,
                 parameters + 5 * MANI_GRAPH_PARAMETER_SIZE + 24, "a quaternion not unit");
    ExpectFormat(nodes, 9, nodes, "an unknown kind");
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
    maniStoreU64(s_crafted + 32, maniHashBytes(MANI_HASH_INIT, s_crafted + 64, s_size - 64));
    maniGraph crafted;
    maniFileResult r = maniOpenGraph(s_crafted, s_size, &crafted);
    CHECK(r.result == mani_errorFormat && r.offset == 40, "an identity that is not the contents'");
    maniGraph g;
    CHECK(maniOpenGraph(NULL, 0, &g).result == mani_errorInvalid &&
              maniOpenGraph(s_file, s_size, NULL).result == mani_errorInvalid &&
              maniGraphNodeCount(NULL) == 0 && maniGraphIdentity(NULL) == 0 &&
              maniGraphFindParameter(NULL, "mix", 3) == MANI_NO_PARAMETER,
          "misuses");
}

static void TestPinned(void)
{
    maniGraphSource source = Source();
    CHECK(Build(&source).result == mani_success, "builds");
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_GRAPH_HASH, "pinned file");
    if (hash != PINNED_GRAPH_HASH)
    {
        printf("graph file hash 0x%016llx\n", (unsigned long long)hash);
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
    TestBuild();
    TestParameterRefusals();
    TestNodeRefusals();
    TestShapeRefusals();
    TestOpenRefusals();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
