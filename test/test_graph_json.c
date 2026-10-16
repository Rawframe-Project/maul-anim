// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The JSON text form of graphs (mani-0023): a source with every node
// kind, every parameter type and a subgraph written as the pinned text;
// a source that does not build refused as the build refuses it; the
// allocator failing at each allocation. Run as test_graph_json --seed
// <file> it writes the pinned text, the seed of the fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_clipFiles[3][8192];
static char s_text[16384];
static maniSkeleton s_skeleton;
static maniClip s_clips[3];

static const maniBoneSource kBones[3] = {
    {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"spine", 5, 0, {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"arm", 3, 1, {{0.3f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
};

static maniBuilder* NewBuilder(const maniAllocator* allocator)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    if (allocator != NULL)
    {
        def.allocator = *allocator;
    }
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Three clips of the hips moving at 1, 2 and 3 a second, the third as
// differences from the rest.
static void Open(void)
{
    maniBuilder* builder = NewBuilder(NULL);
    maniSkeletonSource skeleton = {kBones, 3, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &skeleton).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "skeleton");
    static const float kTimes[2] = {0.0f, 1.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    for (int i = 0; i < 3; ++i)
    {
        float values[6] = {0, 1, 0, (float)(i + 1), 1, 0};
        maniBoneTrackSource track = {
            0, 0, {kTimes, values, 2, maniInterpolationLinear}, kNone, kNone};
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = &track,
                                 .boneTrackCount = 1,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 10.0f,
                                 .additive = i == 2 ? maniAdditiveRest : maniAdditiveNone};
        CHECK(maniBuildClip(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_clipFiles[i], sizeof s_clipFiles[i], &size) ==
                      mani_success &&
                  maniOpenClip(s_clipFiles[i], size, &s_clips[i]).result == mani_success,
              "a clip");
    }
    maniDestroyBuilder(builder);
}

static maniGraphValue Value(float a, float b, float c, float d)
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

#define LITERAL(x) ((maniGraphScalar){MANI_NO_PARAMETER, (x)})

// The subgraph: run at its rate parameter, in the feet group.
static maniGraphParameterSource s_subParameters[1];
static maniGraphClipSource s_subClips[1];
static maniGraphSyncGroupSource s_subGroups[1];
static maniGraphNodeSource s_subNodes[1];
static maniGraphSource s_sub;
static const uint32_t kSubMap[1] = {0};

// The graph: 0 the output, an additive node (a time scale, the wave);
// 1 the time scale (a seek); 2 the seek (a mask); 3 the mask (a blend on
// the spine, a select elsewhere); 4 the blend and 5 the select, each of
// walk and the subgraph; 6 the wave; 7 walk; 8 the subgraph.
static maniGraphParameterSource s_parameters[9];
static maniGraphClipSource s_clipSlots[2];
static maniGraphSyncGroupSource s_groups[1];
static maniGraphNodeSource s_nodes[9];
static maniGraphScalar s_blendWeights[2];
static maniGraphScalar s_layerWeights[1];
static maniMaskChain s_chains[2];
static const uint32_t kAdditiveInputs[2] = {1, 6};
static const uint32_t kTimeInputs[1] = {2};
static const uint32_t kSeekInputs[1] = {3};
static const uint32_t kMaskInputs[2] = {4, 5};
static const uint32_t kPairInputs[2] = {7, 8};

static void Parameters(void)
{
    s_parameters[0] = Parameter("speed", maniGraphParameterFloat, Value(1, 0, 0, 0));
    s_parameters[0].bounded = 1;
    s_parameters[0].maximum = Value(2, 0, 0, 0);
    s_parameters[1] = Parameter("gait", maniGraphParameterInt, Int(-1));
    s_parameters[1].bounded = 1;
    s_parameters[1].minimum = Int(-1);
    s_parameters[1].maximum = Int(2147483647);
    s_parameters[2] = Parameter("jump", maniGraphParameterTrigger, Int(0));
    s_parameters[3] = Parameter("lean", maniGraphParameterBool, Int(1));
    s_parameters[4] = Parameter("aim", maniGraphParameterVec2, Value(0.1f, -0.2f, 0, 0));
    s_parameters[5] = Parameter("dir", maniGraphParameterVec3, Value(1e21f, 1e-7f, -0.0f, 0));
    s_parameters[6] = Parameter("look", maniGraphParameterQuat, Value(0, 0, 0.38268343f, 2));
    s_parameters[7] = Parameter("say \"hi\"\\\n\t\x01\x1f\xc3\xa9", maniGraphParameterFloat,
                                Value(123456789.0f, 0, 0, 0));
    s_parameters[8] = Parameter("mix", maniGraphParameterFloat, Value(0.5f, 0, 0, 0));
    s_parameters[8].bounded = 1;
    s_parameters[8].maximum = Value(1, 0, 0, 0);
}

static maniGraphSource Source(void)
{
    s_subParameters[0] = Parameter("rate", maniGraphParameterFloat, Value(1, 0, 0, 0));
    s_subClips[0] = (maniGraphClipSource){&s_clips[1], "run", 3, 0};
    s_subGroups[0] = (maniGraphSyncGroupSource){"feet", 4, maniGraphSyncAllEvents};
    s_subNodes[0] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                          .clip = {0, 0, {0, 0.0f}, maniGraphLoopClip, 0}};
    s_sub = (maniGraphSource){
        &s_skeleton, s_subParameters, s_subClips, s_subGroups, s_subNodes, 1, 1, 1, 1, 0, 0};
    Parameters();
    s_clipSlots[0] = (maniGraphClipSource){&s_clips[0], "walk", 4, 0};
    s_clipSlots[1] = (maniGraphClipSource){&s_clips[2], "wave", 4, 0};
    s_groups[0] = (maniGraphSyncGroupSource){"feet", 4, maniGraphSyncAllEvents};
    s_blendWeights[0] = (maniGraphScalar){8, 0.0f};
    s_blendWeights[1] = LITERAL(0.75f);
    s_layerWeights[0] = (maniGraphScalar){8, 0.0f};
    s_chains[0] = (maniMaskChain){1, 0.5f, true};
    s_chains[1] = (maniMaskChain){2, 1.0f, false};
    s_nodes[0] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kAdditiveInputs,
                                       .additive = {s_layerWeights}};
    s_nodes[1] =
        (maniGraphNodeSource){maniGraphNodeTimeScale, 1, kTimeInputs, .timeScale = {{0, 0.0f}}};
    s_nodes[2] =
        (maniGraphNodeSource){maniGraphNodeSeek, 1, kSeekInputs, .seek = {2, LITERAL(0.25f)}};
    s_nodes[3] =
        (maniGraphNodeSource){maniGraphNodeMask, 2, kMaskInputs, .mask = {s_chains, 2, 0.1f}};
    s_nodes[4] =
        (maniGraphNodeSource){maniGraphNodeBlend, 2, kPairInputs, .blend = {s_blendWeights, 0, 0}};
    s_nodes[5] = (maniGraphNodeSource){maniGraphNodeSelect, 2, kPairInputs, .select = {1}};
    s_nodes[6] = (maniGraphNodeSource){
        maniGraphNodeClip, 0, NULL,
        .clip = {1, MANI_NO_SYNC_GROUP, LITERAL(1.0f), maniGraphLoopRepeat, 0}};
    s_nodes[7] = (maniGraphNodeSource){
        maniGraphNodeClip, 0, NULL,
        .clip = {0, 0, LITERAL(-1.5f), maniGraphLoopClamp, maniGraphClipLeader}};
    s_nodes[8] = (maniGraphNodeSource){(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL,
                                       .subgraph = {&s_sub, kSubMap}};
    return (maniGraphSource){
        &s_skeleton, s_parameters, s_clipSlots, s_groups, s_nodes, 9, 2, 1, 9, 0, 0};
}

static const char kExpected[] =
    "{\n"
    "  \"format\": \"maul-anim graph\",\n"
    "  \"version\": 1,\n"
    "  \"parameters\": [\n"
    "    {\"name\": \"speed\", \"type\": \"float\", \"default\": 1, \"minimum\": 0, \"maximum\": "
    "2},\n"
    "    {\"name\": \"gait\", \"type\": \"int\", \"default\": -1, \"minimum\": -1, \"maximum\": "
    "2147483647},\n"
    "    {\"name\": \"jump\", \"type\": \"trigger\", \"default\": false},\n"
    "    {\"name\": \"lean\", \"type\": \"bool\", \"default\": true},\n"
    "    {\"name\": \"aim\", \"type\": \"vec2\", \"default\": [0.1, -0.2]},\n"
    "    {\"name\": \"dir\", \"type\": \"vec3\", \"default\": [1e+21, 1e-7, -0]},\n"
    "    {\"name\": \"look\", \"type\": \"quat\", \"default\": [0, 0, 0.38268343, 2]},\n"
    "    {\"name\": \"say \\\"hi\\\"\\\\\\n\\t\\u0001\\u001f\xc3\xa9"
    "\", \"type\": \"float\", \"default\": 123456790},\n"
    "    {\"name\": \"mix\", \"type\": \"float\", \"default\": 0.5, \"minimum\": 0, \"maximum\": "
    "1}\n"
    "  ],\n"
    "  \"clips\": [\n"
    "    \"walk\",\n"
    "    \"wave\"\n"
    "  ],\n"
    "  \"syncGroups\": [\n"
    "    {\"name\": \"feet\", \"allEvents\": true}\n"
    "  ],\n"
    "  \"nodes\": [\n"
    "    {\"kind\": \"additive\", \"inputs\": [1, 6], \"weights\": [{\"parameter\": \"mix\"}]},\n"
    "    {\"kind\": \"timeScale\", \"inputs\": [2], \"scale\": {\"parameter\": \"speed\"}},\n"
    "    {\"kind\": \"seek\", \"inputs\": [3], \"trigger\": \"jump\", \"time\": 0.25},\n"
    "    {\"kind\": \"mask\", \"inputs\": [4, 5], \"chains\": [{\"bone\": \"spine\", \"weight\": "
    "0.5, \"subtree\": true}, {\"bone\": \"arm\", \"weight\": 1, \"subtree\": false}], "
    "\"defaultWeight\": 0.1},\n"
    "    {\"kind\": \"blend\", \"inputs\": [7, 8], \"weights\": [{\"parameter\": \"mix\"}, 0.75], "
    "\"direct\": false},\n"
    "    {\"kind\": \"select\", \"inputs\": [7, 8], \"parameter\": \"gait\"},\n"
    "    {\"kind\": \"clip\", \"clip\": \"wave\", \"syncGroup\": null, \"speed\": 1, \"loop\": "
    "\"repeat\", \"leader\": false},\n"
    "    {\"kind\": \"clip\", \"clip\": \"walk\", \"syncGroup\": \"feet\", \"speed\": -1.5, "
    "\"loop\": \"clamp\", \"leader\": true},\n"
    "    {\"kind\": \"subgraph\", \"parameters\": {\"rate\": \"speed\"}, \"graph\": {\n"
    "      \"parameters\": [\n"
    "        {\"name\": \"rate\", \"type\": \"float\", \"default\": 1}\n"
    "      ],\n"
    "      \"clips\": [\n"
    "        \"run\"\n"
    "      ],\n"
    "      \"syncGroups\": [\n"
    "        {\"name\": \"feet\", \"allEvents\": true}\n"
    "      ],\n"
    "      \"nodes\": [\n"
    "        {\"kind\": \"clip\", \"clip\": \"run\", \"syncGroup\": \"feet\", \"speed\": "
    "{\"parameter\": \"rate\"}, \"loop\": \"clip\", \"leader\": false}\n"
    "      ],\n"
    "      \"output\": 0\n"
    "    }}\n"
    "  ],\n"
    "  \"output\": 0\n"
    "}\n";

static size_t Write(const maniGraphSource* source, maniBuildResult* resultOut)
{
    maniBuilder* builder = NewBuilder(NULL);
    *resultOut = maniWriteGraphJson(builder, source);
    size_t size = 0;
    if (resultOut->result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_text, sizeof s_text, &size) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return size;
}

static void TestWrite(void)
{
    maniGraphSource source = Source();
    maniBuildResult r;
    size_t size = Write(&source, &r);
    bool same = size == sizeof kExpected - 1 && memcmp(s_text, kExpected, size) == 0;
    CHECK(r.result == mani_success && same, "the pinned text");
    if (!same)
    {
        printf("%.*s", (int)size, s_text);
    }
}

static void TestRefusals(void)
{
    maniGraphSource source = Source();
    s_nodes[5].select.parameter = 0;
    maniBuildResult r;
    CHECK(Write(&source, &r) == 0 && r.result == mani_errorInvalid &&
              r.reason == maniBuildReasonNode && r.item == 5,
          "refused as the build refuses");
    source = Source();
    CHECK(maniWriteGraphJson(NULL, &source).result == mani_errorInvalid, "no builder");
    maniBuilder* builder = NewBuilder(NULL);
    CHECK(maniWriteGraphJson(builder, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == 1,
          "no source, a misuse");
    maniDestroyBuilder(builder);
}

static _Alignas(16) uint8_t s_built[16384];
static _Alignas(16) uint8_t s_read[16384];

static const maniGraphClipSource kHost[3] = {
    {&s_clips[0], "walk", 4, 0}, {&s_clips[1], "run", 3, 0}, {&s_clips[2], "wave", 4, 0}};

// A text built against the host's clips, its file copied into out.
static maniBuildResult Read(const char* text, uint8_t* out, size_t* sizeOut)
{
    maniBuilder* builder = NewBuilder(NULL);
    maniBuildResult r = maniBuildGraphJson(builder, text, strlen(text), &s_skeleton, kHost, 3);
    *sizeOut = 0;
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, out, 16384, sizeOut) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return r;
}

static size_t Built(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder(NULL);
    size_t size = 0;
    CHECK(maniBuildGraph(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &size) == mani_success,
          "built");
    maniDestroyBuilder(builder);
    return size;
}

// The pinned text reads back to the bytes its source builds.
static void TestRoundTrip(void)
{
    maniGraphSource source = Source();
    size_t want = Built(&source);
    size_t size = 0;
    maniBuildResult r = Read(kExpected, s_read, &size);
    CHECK(r.result == mani_success && size == want && memcmp(s_read, s_built, size) == 0,
          "read back bit for bit");
}

// A source with an empty list, as written.
static const char kMinimal[] =
    "{\n"
    "  \"format\": \"maul-anim graph\",\n"
    "  \"version\": 1,\n"
    "  \"parameters\": [\n"
    "    {\"name\": \"q\", \"type\": \"quat\", \"default\": [0, 0, 0, 1]},\n"
    "    {\"name\": \"n\", \"type\": \"int\", \"default\": 0}\n"
    "  ],\n"
    "  \"clips\": [\n"
    "    \"walk\"\n"
    "  ],\n"
    "  \"syncGroups\": [],\n"
    "  \"nodes\": [\n"
    "    {\"kind\": \"clip\", \"clip\": \"walk\", \"syncGroup\": null, \"speed\": 1, \"loop\": "
    "\"clip\", \"leader\": false}\n"
    "  ],\n"
    "  \"output\": 0\n"
    "}\n";

// Members left out take their off values: a parameter's default, a clip
// node's group, speed, loop and leader; written out in full.
static void TestDefaults(void)
{
    static const char kShort[] =
        "\xEF\xBB\xBF{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": "
        "[{\"name\": \"q\", \"type\": \"quat\"}, {\"name\": \"n\", \"type\": \"int\"}], "
        "\"clips\": [\"walk\"], \"nodes\": [{\"kind\": \"clip\", \"clip\": \"walk\"}], "
        "\"output\": 0}";
    maniGraphParameterSource parameters[2] = {
        Parameter("q", maniGraphParameterQuat, Value(0, 0, 0, 1)),
        Parameter("n", maniGraphParameterInt, Int(0))};
    maniGraphClipSource slot = {&s_clips[0], "walk", 4, 0};
    maniGraphNodeSource node = {
        maniGraphNodeClip, 0, NULL,
        .clip = {0, MANI_NO_SYNC_GROUP, LITERAL(1.0f), maniGraphLoopClip, 0}};
    maniGraphSource source = {&s_skeleton, parameters, &slot, NULL, &node, 2, 1, 0, 1, 0, 0};
    size_t want = Built(&source);
    size_t size = 0;
    maniBuildResult r = Read(kShort, s_read, &size);
    CHECK(r.result == mani_success && size == want && memcmp(s_read, s_built, size) == 0,
          "the short form");
    size = Write(&source, &r);
    CHECK(r.result == mani_success && size == sizeof kMinimal - 1 &&
              memcmp(s_text, kMinimal, size) == 0,
          "an empty list on one line");
}

// A seek's time, a mask's default weight, a chain's subtree, a blend's
// direct and a sync group's allEvents left out.
static void TestNodeDefaults(void)
{
    static const char kText[] =
        "{\"format\": \"maul-anim graph\", \"version\": 1, \"parameters\": [{\"name\": \"t\", "
        "\"type\": \"trigger\"}], \"clips\": [\"walk\"], \"syncGroups\": [{\"name\": \"g\"}], "
        "\"nodes\": [{\"kind\": \"seek\", \"inputs\": [1], \"trigger\": \"t\"}, {\"kind\": "
        "\"mask\", \"inputs\": [2, 3], \"chains\": [{\"bone\": \"spine\", \"weight\": 1}]}, "
        "{\"kind\": \"blend\", \"inputs\": [3], \"weights\": "
        "[1]}, {\"kind\": \"clip\", \"clip\": \"walk\", \"syncGroup\": \"g\"}], \"output\": 0}";
    static const uint32_t kSeek[1] = {1};
    static const uint32_t kMask[2] = {2, 3};
    static const uint32_t kBlend[1] = {3};
    static const maniGraphScalar kOne[1] = {{MANI_NO_PARAMETER, 1.0f}};
    static const maniMaskChain kSpine = {1, 1.0f, false};
    maniGraphParameterSource trigger = Parameter("t", maniGraphParameterTrigger, Int(0));
    maniGraphClipSource slot = {&s_clips[0], "walk", 4, 0};
    maniGraphSyncGroupSource group = {"g", 1, 0};
    maniGraphNodeSource nodes[4] = {
        {maniGraphNodeSeek, 1, kSeek, .seek = {0, LITERAL(0.0f)}},
        {maniGraphNodeMask, 2, kMask, .mask = {&kSpine, 1, 0.0f}},
        {maniGraphNodeBlend, 1, kBlend, .blend = {kOne, 0, 0}},
        {maniGraphNodeClip, 0, NULL, .clip = {0, 0, LITERAL(1.0f), maniGraphLoopClip, 0}}};
    maniGraphSource source = {&s_skeleton, &trigger, &slot, &group, nodes, 1, 1, 1, 4, 0, 0};
    size_t want = Built(&source);
    size_t size = 0;
    maniBuildResult r = Read(kText, s_read, &size);
    CHECK(r.result == mani_success && size == want && memcmp(s_read, s_built, size) == 0,
          "node members left out");
}

// A text refused: the reason, the fault and the byte offset of the
// marker's occurrence (inside a string's quotes).
typedef struct Refusal
{
    const char* text;
    maniBuildReason reason;
    uint32_t other;
    const char* marker;
    uint32_t occurrence;
} Refusal;

#define TOP  "{\"format\": \"maul-anim graph\", \"version\": 1, "
#define WALK "\"clips\": [\"walk\"], "
#define NODE "{\"kind\": \"clip\", \"clip\": \"walk\"}"

static const Refusal kRefusals[] = {
    {TOP WALK "\"nodes\": [x]}", maniBuildReasonJson, 1, "x]", 1},
    {TOP "\"parameters\": [{\"name\": \"a\", \"type\": \"int\", \"default\": 2147483648}], " WALK
         "\"nodes\": [" NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "2147483648", 1},
    {TOP "\"parameters\": [{\"name\": \"p\", \"type\": \"float\"}], " WALK
         "\"nodes\": [{\"kind\": \"subgraph\", \"graph\": {\"parameters\": [{\"name\": \"r\", "
         "\"type\": \"float\"}], " WALK "\"nodes\": [" NODE
         "], \"output\": 0}, \"parameters\": {}}], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMissing, "{}", 1},
    {TOP WALK "\"nodes\": [" NODE "], \"output\": 0, \"bogus\": 1}", maniBuildReasonForm,
     maniGraphFormMember, "\"bogus", 1},
    {TOP WALK "\"nodes\": [" NODE "], \"output\": 0, \"output\": 0}", maniBuildReasonForm,
     maniGraphFormRepeat, "\"output", 2},
    {TOP WALK "\"output\": 0}", maniBuildReasonForm, maniGraphFormMissing, "{", 1},
    {TOP WALK "\"nodes\": [" NODE "], \"output\": -1}", maniBuildReasonForm, maniGraphFormValue,
     "-1", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"clip\", \"clip\": \"jog\"}], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormName, "\"jog", 1},
    {"{\"format\": \"maul-anim graph\", \"version\": 2, " WALK "\"nodes\": [" NODE
     "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormVersion, "2,", 1},
    {"{\"format\": \"other\", \"version\": 1, " WALK "\"nodes\": [" NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormVersion, "\"other", 1},
    {TOP "\"clips\": [\"jog\"], \"nodes\": [" NODE "], \"output\": 0}", maniBuildReasonForm,
     maniGraphFormUnbound, "\"jog", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"dance\"}], \"output\": 0}", maniBuildReasonForm,
     maniGraphFormValue, "\"dance", 1},
    {TOP WALK "\"nodes\": [{\"clip\": \"walk\"}], \"output\": 0}", maniBuildReasonForm,
     maniGraphFormMissing, "{\"clip", 1},
    {TOP "\"parameters\": [{\"name\": \"a\", \"type\": \"float\", \"minimum\": 0}], " WALK
         "\"nodes\": [" NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMissing, "{\"name", 1},
    {TOP "\"parameters\": [{\"name\": \"a\", \"type\": \"bool\", \"minimum\": 0, \"maximum\": "
         "1}], " WALK "\"nodes\": [" NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMember, "\"minimum", 1},
    {TOP "\"parameters\": [{\"name\": \"a\", \"type\": \"vec4\"}], " WALK "\"nodes\": [" NODE
         "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "\"vec4", 1},
    {TOP "\"parameters\": [{\"name\": \"a\", \"type\": \"vec2\", \"default\": [1]}], " WALK
         "\"nodes\": [" NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "[1]", 1},
    {TOP "\"parameters\": [{\"name\": \"v\", \"type\": \"vec2\"}], " WALK
         "\"nodes\": [{\"kind\": \"blendSpace1D\", \"inputs\": [1, 2], \"position\": "
         "{\"vector\": \"v\", \"x\": 0}, \"points\": [0, 1]}, " NODE ", " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMember, "\"x", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"blendSpace1D\", \"inputs\": [1], \"position\": "
              "{\"x\": 0, \"y\": 0}, \"points\": [0]}, " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMember, "\"y", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"blendSpaceBands\", \"inputs\": [1], \"position\": "
              "{}, \"points\": [[0, 0]]}, " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMissing, "{}", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"blendSpace1D\", \"inputs\": [1, 2], \"position\": "
              "{\"x\": 0}, \"points\": [0]}, " NODE ", " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "[0]", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"blendSpaceBands\", \"inputs\": [1], \"position\": "
              "{\"x\": 0}, \"points\": [[0]]}, " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "[0]", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"clip\", \"clip\": \"walk\", \"speed\": 1e39}], "
              "\"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "1e39", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"clip\", \"clip\": \"walk\", \"loop\": \"twice\"}], "
              "\"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "\"twice", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"blend\", \"inputs\": [1, 1], \"weights\": [1]}, " NODE
              "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormValue, "[1]", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"mask\", \"inputs\": [1, 1], \"chains\": [{\"bone\": "
              "\"tail\", \"weight\": 1}]}, " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormName, "\"tail", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"timeScale\", \"inputs\": [1], \"scale\": "
              "{\"parameter\": \"pace\"}}, " NODE "], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormName, "\"pace", 1},
    {TOP WALK "\"nodes\": [{\"kind\": \"subgraph\", \"graph\": {\"format\": \"x\"}}], "
              "\"output\": 0}",
     maniBuildReasonForm, maniGraphFormMember, "\"format\": \"x", 1},
    {TOP "\"parameters\": [{\"name\": \"p\", \"type\": \"float\"}], " WALK
         "\"nodes\": [{\"kind\": \"subgraph\", \"graph\": {\"parameters\": [{\"name\": \"r\", "
         "\"type\": \"float\"}], " WALK "\"nodes\": [" NODE "], \"output\": 0}}], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormMissing, "{\"kind\": \"subgraph", 1},
    {TOP "\"parameters\": [{\"name\": \"p\", \"type\": \"float\"}], " WALK
         "\"nodes\": [{\"kind\": \"subgraph\", \"graph\": {\"parameters\": [{\"name\": \"r\", "
         "\"type\": \"float\"}], " WALK "\"nodes\": [" NODE
         "], \"output\": 0}, \"parameters\": {\"r\": \"p\", \"r\": \"p\"}}], \"output\": 0}",
     maniBuildReasonForm, maniGraphFormRepeat, "\"r\": \"p\"", 2},
    // Read, then refused by the build: a select on a float.
    {TOP "\"parameters\": [{\"name\": \"p\", \"type\": \"float\"}], " WALK
         "\"nodes\": [{\"kind\": \"select\", \"inputs\": [1, 1], \"parameter\": \"p\"}, " NODE
         "], \"output\": 0}",
     maniBuildReasonNode, 0, NULL, 0},
};

// The byte offset of a marker's occurrence, inside a string's quotes.
static uint32_t OffsetOf(const char* text, const char* marker, uint32_t occurrence)
{
    const char* at = text - 1;
    for (uint32_t k = 0; k < occurrence && at != NULL; ++k)
    {
        at = strstr(at + 1, marker);
    }
    return at == NULL ? UINT32_MAX : (uint32_t)(at - text) + (marker[0] == '"' ? 1u : 0u);
}

static void TestReadRefusals(void)
{
    uint32_t wrong = 0;
    for (size_t i = 0; i < sizeof kRefusals / sizeof kRefusals[0]; ++i)
    {
        const Refusal* c = &kRefusals[i];
        size_t size = 0;
        maniBuildResult r = Read(c->text, s_read, &size);
        bool ok = r.result == mani_errorInvalid && r.reason == c->reason;
        if (c->marker != NULL)
        {
            ok = ok && r.other == c->other && r.item == OffsetOf(c->text, c->marker, c->occurrence);
        }
        if (!ok)
        {
            printf("case %zu: %d %u %u %u\n", i, (int)r.result, r.reason, r.item, r.other);
            ++wrong;
        }
    }
    CHECK(wrong == 0, "each text refused at its offset");
    maniBuilder* builder = NewBuilder(NULL);
    CHECK(maniBuildGraphJson(NULL, "", 0, &s_skeleton, NULL, 0).result == mani_errorInvalid,
          "no builder");
    CHECK(maniBuildGraphJson(builder, NULL, 1, &s_skeleton, NULL, 0).result == mani_errorInvalid &&
              maniBuildGraphJson(builder, "", 0, NULL, NULL, 0).result == mani_errorInvalid &&
              maniBuildGraphJson(builder, "", 0, &s_skeleton, NULL, 1).result ==
                  mani_errorInvalid &&
              maniBuilderMisuses(builder) == 3,
          "misuses");
    maniDestroyBuilder(builder);
}

// An allocator failing at one allocation, counting the live blocks;
// aligned by hand (no aligned_alloc everywhere): the block's start just
// before what it returns.
typedef struct Failing
{
    uint32_t calls;
    uint32_t failAt;
    uint32_t live;
} Failing;

static void* FailingAlloc(size_t size, size_t alignment, void* context)
{
    Failing* f = context;
    if (f->calls++ == f->failAt)
    {
        return NULL;
    }
    uint8_t* block = malloc(size + alignment + sizeof(void*));
    if (block == NULL)
    {
        return NULL;
    }
    uint8_t* at = block + sizeof(void*);
    at += (alignment - (uintptr_t)at % alignment) % alignment;
    memcpy(at - sizeof(void*), (const void*)&block, sizeof block);
    ++f->live;
    return at;
}

static void FailingFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    Failing* f = context;
    void* block = NULL;
    memcpy((void*)&block, (uint8_t*)memory - sizeof(void*), sizeof block);
    free(block);
    --f->live;
}

// One run with the failAt-th allocation failing (none past the count):
// whether one failed, and whether it failed cleanly (Capacity, nothing
// kept, nothing leaked; success when none failed).
static bool FailOnce(bool reading, uint32_t failAt, bool* cleanOut)
{
    maniGraphSource source = Source();
    Failing f = {0, UINT32_MAX, 0};
    maniAllocator allocator = {FailingAlloc, FailingFree, &f};
    maniBuilder* builder = NewBuilder(&allocator);
    f.failAt = f.calls + failAt;
    maniBuildResult r = reading ? maniBuildGraphJson(builder, kExpected, sizeof kExpected - 1,
                                                     &s_skeleton, kHost, 3)
                                : maniWriteGraphJson(builder, &source);
    size_t size = 1;
    bool failed = f.calls > f.failAt;
    bool clean = failed ? r.result == mani_errorCapacity &&
                              maniCopyBuilt(builder, NULL, 0, &size) == mani_errorInvalid
                        : r.result == mani_success;
    maniDestroyBuilder(builder);
    *cleanOut = clean && f.live == 0;
    return failed;
}

// Each allocation of the writer, then of the reader, failing in turn.
static void TestAllocationFailure(void)
{
    for (int reading = 0; reading < 2; ++reading)
    {
        bool clean = true;
        uint32_t failAt = 0;
        bool once = true;
        while (FailOnce(reading != 0, failAt, &once))
        {
            clean = clean && once;
            ++failAt;
        }
        CHECK(clean && once && failAt > 2, "each failure Capacity, nothing kept or leaked");
    }
}

static int WriteSeed(const char* path)
{
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(kExpected, 1, sizeof kExpected - 1, out);
    return fclose(out) == 0 && written == sizeof kExpected - 1 ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    Open();
    TestWrite();
    TestRefusals();
    TestAllocationFailure();
    TestRoundTrip();
    TestDefaults();
    TestNodeDefaults();
    TestReadRefusals();
    return s_failures == 0 ? 0 : 1;
}
