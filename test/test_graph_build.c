// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graph builds in detail: the refusals the main suite leaves
// to these sources (a name not UTF-8, a clip of another skeleton, a sync
// group's unknown flag, a loop, mask bone and weights, a direct flag and
// weight, differences where poses go); a node reading one input twice
// freeing its slot once; ignored values written as zeros, so the file
// opens; each mask its own weights; a mask on a skeleton of 32 bones
// evaluated as the blending functions do; opening's checks of records,
// data words, parameters and sections, each on a crafted file; a
// subgraph whose output is not its last node, its parameter mapped and
// its clips joined, written as the flat graph; subgraphs refused; each
// node's time factor, written and checked; a binding's room for the
// largest cursor and the most inputs.

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_fixture.h"
#include "negative_kinds.h"

#include "maul-anim/pose.h"

#include <stdio.h>

#define CHAIN 32

static maniBuildResult BuildOnly(maniGraphSource source)
{
    source.skeleton = source.skeleton != NULL ? source.skeleton : &s_skeleton;
    if (source.clips == NULL)
    {
        source.clips = kSlots;
        source.clipCount = CLIPS;
    }
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, &source);
    maniDestroyBuilder(builder);
    return r;
}

static bool Refused(maniGraphSource source, maniBuildReason reason, uint32_t item, uint32_t other)
{
    maniBuildResult r = BuildOnly(source);
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item &&
           r.other == other;
}

// Refused as Refused says, for passing a fixed bound.
static bool PastLimit(maniGraphSource source, maniBuildReason reason, uint32_t item, uint32_t other)
{
    maniBuildResult r = BuildOnly(source);
    return r.result == mani_errorLimit && r.reason == reason && r.item == item && r.other == other;
}

// A skeleton of 32 bones in a chain (enough that the mask's two weight
// rows fill more than a pose's header) and two clips on it: the root and
// the last bone moving along x.
static _Alignas(16) uint8_t s_chainFile[16384];
static _Alignas(16) uint8_t s_chainClipFiles[3][32768];
static maniSkeleton s_chain;
static maniClip s_chainClips[3];

static void OpenChain(void)
{
    static maniBoneSource bones[CHAIN];
    static char names[CHAIN][16];
    for (int i = 0; i < CHAIN; ++i)
    {
        snprintf(names[i], sizeof names[i], "b%d", i);
        bones[i] = (maniBoneSource){
            names[i],
            (uint32_t)strlen(names[i]),
            i - 1,
            {{0.0f, 0.1f * (float)i, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource skeleton = {bones, CHAIN};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &skeleton).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_chainFile, sizeof s_chainFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_chainFile, size, &s_chain).result == mani_success,
          "the chain");
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kMoves[2][6] = {{0, 0, 0, 1, 0, 0}, {0, 0, 0, 3, 0, 0}};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    for (int i = 0; i < 2; ++i)
    {
        maniBoneTrackSource track = {(uint32_t)((1 - i) * (CHAIN - 1)),
                                     {kTimes, kMoves[i], 2, maniInterpolationLinear},
                                     kNone,
                                     kNone};
        maniClipSource source = {.skeleton = &s_chain,
                                 .boneTracks = &track,
                                 .boneTrackCount = 1,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 10.0f};
        CHECK(maniBuildClip(builder, &source).result == mani_success &&
                  maniBuilder_CopyBuilt(builder, s_chainClipFiles[i], sizeof s_chainClipFiles[i],
                                        &size) == mani_success &&
                  maniOpenClip(s_chainClipFiles[i], size, &s_chainClips[i]).result == mani_success,
              "a chain clip");
    }
    // The third clip: every bone moving, compressed (its cursor caches
    // samples, so it is larger than a full clip's).
    static maniBoneTrackSource all[CHAIN];
    for (int b = 0; b < CHAIN; ++b)
    {
        all[b] = (maniBoneTrackSource){
            (uint32_t)b, {kTimes, kMoves[0], 2, maniInterpolationLinear}, kNone, kNone};
    }
    maniClipSource every = {.skeleton = &s_chain,
                            .boneTracks = all,
                            .boneTrackCount = CHAIN,
                            .loop = maniLoopRepeat,
                            .sampleRate = 10.0f};
    static _Alignas(16) uint8_t full[32768];
    maniClip uncompressed;
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(maniBuildClip(builder, &every).result == mani_success &&
              maniBuilder_CopyBuilt(builder, full, sizeof full, &size) == mani_success &&
              maniOpenClip(full, size, &uncompressed).result == mani_success &&
              maniCompressClip(builder, &uncompressed, &s_chain, &options, NULL) == mani_success &&
              maniBuilder_CopyBuilt(builder, s_chainClipFiles[2], sizeof s_chainClipFiles[2],
                                    &size) == mani_success &&
              maniOpenClip(s_chainClipFiles[2], size, &s_chainClips[2]).result == mani_success,
          "a compressed clip moving every bone");
    maniDestroyBuilder(builder);
}

static void TestRefusals(void)
{
    static const uint32_t kPair[2] = {1, 2};
    static const maniGraphScalar kOnes[2] = {LITERAL(1.0f), LITERAL(1.0f)};
    maniGraphNodeSource walk = ClipNode(0, 1.0f, maniGraphLoopClip);
    maniGraphSource one = {.nodes = &walk, .nodeCount = 1};
    maniGraphParameterSource bad = Parameter("\xff", maniGraphParameterFloat, Float(0.0f));
    maniGraphSource named = one;
    named.parameters = &bad;
    named.parameterCount = 1;
    CHECK(Refused(named, maniBuildReasonParameterList | maniBuildReasonName, 0, 0),
          "a name not UTF-8");
    maniGraphClipSource foreign = {&s_chainClips[0], "chain", 5, NULL};
    maniGraphSource other = one;
    other.clips = &foreign;
    other.clipCount = 1;
    CHECK(Refused(other, maniBuildReasonClip, 0, 0), "a clip of another skeleton");
    maniGraphSyncGroupSource group = {"feet", 4, 2};
    maniGraphSource flagged = one;
    flagged.syncGroups = &group;
    flagged.syncGroupCount = 1;
    CHECK(Refused(flagged, maniBuildReasonSyncGroupList | maniBuildReasonKind, 0, 0),
          "a sync group's unknown flag");
    walk.clip.loop = 3;
    CHECK(Refused(one, maniBuildReasonNode, 0, 0), "a loop past repeat");
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        walk.clip.loop = kNegativeKinds[k];
        CHECK(Refused(one, maniBuildReasonNode, 0, 0), "a negative loop");
    }
    maniMaskChain chain = {3, 1.0f, false};
    maniGraphNodeSource mask[3] = {{maniGraphNodeMask, 2, kPair, .mask = {&chain, 1, 0.0f}},
                                   ClipNode(0, 1.0f, maniGraphLoopClip),
                                   ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraphSource masked = {.nodes = mask, .nodeCount = 3};
    CHECK(Refused(masked, maniBuildReasonNode, 0, 0), "a chain past the bones");
    chain.bone = 1;
    mask[0].mask.defaultWeight = 1.5f;
    CHECK(Refused(masked, maniBuildReasonWeight, 0, 1), "a default weight past 1");
    maniGraphScalar direct[2] = {LITERAL(1.0f), LITERAL(1.5f)};
    maniGraphNodeSource blend[3] = {{maniGraphNodeBlend, 2, kPair, .blend = {kOnes, 2}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraphSource blended = {.nodes = blend, .nodeCount = 3};
    CHECK(Refused(blended, maniBuildReasonNode, 0, 0), "a direct flag past 1");
    blend[0].blend = (maniGraphBlendNodeSource){direct, 1};
    CHECK(Refused(blended, maniBuildReasonScalar, 0, 1), "a direct weight past 1");
    // The wave (2) is differences.
    maniGraphNodeSource add[3] = {{maniGraphNodeAdditive, 2, kPair, .additive = {kOnes}},
                                  ClipNode(2, 1.0f, maniGraphLoopClip),
                                  ClipNode(2, 1.0f, maniGraphLoopClip)};
    CHECK(Refused((maniGraphSource){.nodes = add, .nodeCount = 3}, maniBuildReasonDifference, 0, 0),
          "an additive base of differences");
    // A layer adding onto differences, under a node that adds it to the
    // run: refused where it is, not at the output.
    static const uint32_t kOuter[2] = {1, 2};
    static const uint32_t kInner[2] = {3, 4};
    maniGraphNodeSource nested[5] = {{maniGraphNodeAdditive, 2, kOuter, .additive = {kOnes}},
                                     ClipNode(1, 1.0f, maniGraphLoopClip),
                                     {maniGraphNodeAdditive, 2, kInner, .additive = {kOnes}},
                                     ClipNode(2, 1.0f, maniGraphLoopClip),
                                     ClipNode(2, 1.0f, maniGraphLoopClip)};
    CHECK(Refused((maniGraphSource){.nodes = nested, .nodeCount = 5}, maniBuildReasonDifference, 2,
                  0),
          "differences under an additive node's layer");
    maniGraphNodeSource wave = ClipNode(2, 1.0f, maniGraphLoopClip);
    CHECK(
        Refused((maniGraphSource){.nodes = &wave, .nodeCount = 1}, maniBuildReasonDifference, 0, 0),
        "an output of differences");
}

static _Alignas(16) uint8_t s_built[16384];
static size_t s_builtSize;

// The source built (kept in s_built) and opened.
static bool Opens(maniGraphSource source, maniGraph* g)
{
    source.skeleton = source.skeleton != NULL ? source.skeleton : &s_skeleton;
    if (source.clips == NULL)
    {
        source.clips = kSlots;
        source.clipCount = CLIPS;
    }
    maniBuilder* builder = NewBuilder();
    bool ok =
        maniBuildGraph(builder, &source).result == mani_success &&
        maniBuilder_CopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success &&
        maniOpenGraph(s_built, s_builtSize, g).result == mani_success;
    maniDestroyBuilder(builder);
    return ok;
}

// A blend of the walk with itself frees the walk's slot once: of the two
// clips the next blend reads, neither shares the other's slot.
static void TestSlotFreedOnce(void)
{
    static const uint32_t kOutput[2] = {1, 2};
    static const uint32_t kTwice[2] = {3, 3};
    static const uint32_t kPair[2] = {4, 5};
    static const maniGraphScalar kOnes[2] = {LITERAL(1.0f), LITERAL(1.0f)};
    maniGraphNodeSource nodes[6] = {{maniGraphNodeBlend, 2, kOutput, .blend = {kOnes, 0}},
                                    {maniGraphNodeBlend, 2, kTwice, .blend = {kOnes, 0}},
                                    {maniGraphNodeBlend, 2, kPair, .blend = {kOnes, 0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 2.0f, maniGraphLoopClip)};
    maniGraph g;
    // The walk 0 (slot 0), its blend 1 (1), the runs 2 (0) and 3 (2), their
    // blend 4 (3), the output 5 (2).
    CHECK(Opens((maniGraphSource){.nodes = nodes, .nodeCount = 6}, &g) && g.slotCount == 4,
          "four slots");
}

// A bound scalar's literal and an unbounded int's bounds are ignored and
// written as zeros, so the file opens.
static void TestIgnoredValues(void)
{
    maniGraphParameterSource parameters[2] = {
        Parameter("pace", maniGraphParameterFloat, Float(1.0f)),
        Parameter("n", maniGraphParameterInt, Int(0))};
    parameters[1].minimum = Int(3);
    parameters[1].maximum = Int(-3);
    maniGraphNodeSource walk = ClipNode(0, 1.0f, maniGraphLoopClip);
    walk.clip.speed = (maniGraphScalar){0, 5.0f};
    maniGraph g;
    CHECK(Opens(
              (maniGraphSource){
                  .parameters = parameters, .nodes = &walk, .parameterCount = 2, .nodeCount = 1},
              &g),
          "ignored values written as zeros");
}

// A mask of the hips at 0.5 over a mask of the spine: the outer mask's
// weights its own (the inner mask weighed 0.5, the walk inside it too).
static void TestTwoMasks(void)
{
    static const uint32_t kOuter[2] = {1, 3};
    static const uint32_t kInner[2] = {2, 3};
    static const maniMaskChain kHips = {0, 0.5f, false};
    static const maniMaskChain kSpine = {1, 1.0f, false};
    maniGraphNodeSource nodes[4] = {{maniGraphNodeMask, 2, kOuter, .mask = {&kHips, 1, 0.0f}},
                                    {maniGraphNodeMask, 2, kInner, .mask = {&kSpine, 1, 0.0f}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    // The walk 0, the run 1, the inner mask 2, the outer 3.
    maniGraphInstance* instance = Make((maniGraphSource){.nodes = nodes, .nodeCount = 4}, &g);
    Update(instance, 0.0f);
    CHECK(NearF(State(instance, &g, 2)->weight, 0.5) && NearF(State(instance, &g, 0)->weight, 0.5),
          "each mask its own weights");
}

static _Alignas(8) uint8_t s_chainBinding[512];
static _Alignas(8) uint8_t s_chainInstance[8192];
static _Alignas(32) uint8_t s_scratch[65536];
static _Alignas(32) uint8_t s_poses[4][16384];

static maniPose* ChainPose(int i)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_chain, s_poses[i], sizeof s_poses[i], &pose) == mani_success, "a pose");
    return pose;
}

// The two chain clips masked at b5 and below: the graph's pose as the
// blending functions make it from the mask's bone weights.
static void TestChainMask(void)
{
    static const uint32_t kInputs[2] = {1, 2};
    static const maniMaskChain kChain = {5, 1.0f, true};
    const maniGraphClipSource slots[2] = {{&s_chainClips[0], "a", 1, NULL},
                                          {&s_chainClips[1], "b", 1, NULL}};
    maniGraphNodeSource nodes[3] = {{maniGraphNodeMask, 2, kInputs, .mask = {&kChain, 1, 0.0f}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    const maniClip* clips[2] = {&s_chainClips[0], &s_chainClips[1]};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    maniPose* out = ChainPose(0);
    CHECK(
        Opens((maniGraphSource){.skeleton = &s_chain,
                                .clips = slots,
                                .nodes = nodes,
                                .clipCount = 2,
                                .nodeCount = 3},
              &g) &&
            maniBindGraph(&g,
                          &(maniGraphAssets){&s_chain, clips, NULL, NULL, NULL, 2, 0, 0, 0, NULL,
                                             NULL, 0, 0, NULL},
                          s_chainBinding, sizeof s_chainBinding, &binding, NULL) == mani_success &&
            maniInitGraphInstance(binding, s_chainInstance, sizeof s_chainInstance, &instance) ==
                mani_success &&
            maniGraphInstance_Update(instance, 0.25f, NULL, 0, NULL, NULL) == mani_success &&
            maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, out) == mani_success,
        "a chain graph");
    float inside[CHAIN];
    float outside[CHAIN];
    CHECK(maniSkeleton_BuildMask(&s_chain, &kChain, 1, 0.0f, inside, CHAIN, NULL) == mani_success,
          "a mask");
    for (int b = 0; b < CHAIN; ++b)
    {
        outside[b] = 1.0f - inside[b];
    }
    maniPose* a = ChainPose(1);
    maniPose* b = ChainPose(2);
    maniPose* want = ChainPose(3);
    const maniBlendLayer layers[2] = {{a, 1.0f, inside}, {b, 1.0f, outside}};
    CHECK(maniClip_Sample(&s_chainClips[0], &s_chain, 0.25f, a) == mani_success &&
              maniClip_Sample(&s_chainClips[1], &s_chain, 0.25f, b) == mani_success &&
              maniSkeleton_BlendPoses(&s_chain, layers, 2, MANI_BLEND_THRESHOLD, want) ==
                  mani_success,
          "by hand");
    bool same = true;
    for (uint32_t bone = 0; bone < CHAIN; ++bone)
    {
        maniTransform x = maniPose_GetLocal(out, bone);
        maniTransform y = maniPose_GetLocal(want, bone);
        same = same && memcmp(&x, &y, sizeof x) == 0;
    }
    CHECK(same, "the masked pose, bit for bit");
}

static _Alignas(16) uint8_t s_crafted[16384];

// The payload hash and identity made right after an edit.
static void Rehash(uint8_t* file, size_t size)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + MANI_CONTAINER_HEADER_SIZE,
                                          size - MANI_CONTAINER_HEADER_SIZE));
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

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_built, s_builtSize, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

// The built file with words set (offset, value pairs), opened: refused
// at want.
static bool RefusedAt(const uint64_t* edits, uint32_t count, uint64_t want)
{
    memcpy(s_crafted, s_built, s_builtSize);
    for (uint32_t k = 0; k < count; ++k)
    {
        maniStoreU32(s_crafted + edits[2 * k], (uint32_t)edits[2 * k + 1]);
    }
    Rehash(s_crafted, s_builtSize);
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_crafted, s_builtSize, &g);
    if (r.result != mani_errorFormat || r.offset != want)
    {
        printf("  refused at %llu, expected %llu\n", (unsigned long long)r.offset,
               (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want;
}

static bool Edit(uint64_t offset, uint32_t value, uint64_t want)
{
    const uint64_t edit[2] = {offset, value};
    return RefusedAt(edit, 1, want);
}

// The crafted graph: parameters speed (float, 0 to 2), n (int), lean
// (bool), jump (trigger), mix (float); its nodes by place: the walk 0
// (grouped, at speed), the run 1 (grouped), a mask 2, the wave 3, an
// additive node 4, a time scale 5, a seek 6, a blend 7, a select 8.
static bool BuildCrafted(void)
{
    static maniGraphParameterSource parameters[5];
    parameters[0] = Parameter("speed", maniGraphParameterFloat, Float(1.0f));
    parameters[0].bounded = 1;
    parameters[0].maximum = Float(2.0f);
    parameters[1] = Parameter("n", maniGraphParameterInt, Int(0));
    parameters[2] = Parameter("lean", maniGraphParameterBool, Int(1));
    parameters[3] = Parameter("jump", maniGraphParameterTrigger, Int(0));
    parameters[4] = Parameter("mix", maniGraphParameterFloat, Float(0.5f));
    static const maniGraphSyncGroupSource kGroup = {"feet", 4, 0};
    static const uint32_t kSelect[2] = {1, 6};
    static const uint32_t kSeek[1] = {2};
    static const uint32_t kScale[1] = {3};
    static const uint32_t kAdd[2] = {4, 5};
    static const uint32_t kPair[2] = {7, 8};
    static const maniGraphScalar kLayer[1] = {{4, 0.0f}};
    static const maniGraphScalar kBlend[2] = {{4, 0.0f}, LITERAL(1.0f)};
    static const maniMaskChain kSpine = {1, 1.0f, false};
    static maniGraphNodeSource nodes[9];
    nodes[0] = (maniGraphNodeSource){maniGraphNodeSelect, 2, kSelect, .select = {1}};
    nodes[1] = (maniGraphNodeSource){maniGraphNodeSeek, 1, kSeek, .seek = {3, LITERAL(0.0f)}};
    nodes[2] = (maniGraphNodeSource){maniGraphNodeTimeScale, 1, kScale, .timeScale = {{0, 0.0f}}};
    nodes[3] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kAdd, .additive = {kLayer}};
    nodes[4] = (maniGraphNodeSource){maniGraphNodeMask, 2, kPair, .mask = {&kSpine, 1, 0.0f}};
    nodes[5] = ClipNode(2, 1.0f, maniGraphLoopClip);
    nodes[6] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kPair, .blend = {kBlend, 0}};
    nodes[7] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                     .clip = {0, 0, {0, 0.0f}, maniGraphLoopClip, 0}};
    nodes[8] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                     .clip = {1, 0, LITERAL(1.0f), maniGraphLoopClip, 0}};
    maniGraphSource source = {&s_skeleton, parameters, kSlots, &kGroup, nodes, 5,    CLIPS, 1,   9,
                              0,           0,          NULL,   0,       0,     NULL, NULL,  NULL};
    maniBuilder* builder = NewBuilder();
    bool ok = maniBuildGraph(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success;
    maniDestroyBuilder(builder);
    return ok;
}

static uint64_t Record(uint32_t p)
{
    return SectionAt(maniGraphTagNodes) + (uint64_t)p * MANI_GRAPH_NODE_SIZE;
}

// The file offset of node p's data word k.
static uint64_t Word(uint32_t p, uint32_t k)
{
    uint32_t first = maniLoadU32(s_built + Record(p) + 12);
    return SectionAt(maniGraphTagData) + 4 * (uint64_t)(first + k);
}

// Every check opening makes of the records, the data words and the lists.
static void TestOpenRefusals(void)
{
    CHECK(BuildCrafted(), "the crafted graph");
    maniGraph g;
    CHECK(maniOpenGraph(s_built, s_builtSize, &g).result == mani_success, "opens as built");
    CHECK(Edit(Word(0, 5), 1, Word(0, 4)), "a bound scalar with a literal");
    CHECK(Edit(Word(7, 4), 0xBF800000u, Word(7, 3)), "a blend weight below 0");
    CHECK(Edit(Word(3, 3), 1, Word(3, 3)), "an ungrouped clip node led");
    CHECK(Edit(Word(2, 0), 1, Word(2, 0)), "a mask past the count");
    CHECK(Edit(Word(8, 0), 0, Word(8, 0)), "a select by a float");
    CHECK(Edit(Record(3), 1, Record(3) + 2), "a clip node of differences marked as poses");
    CHECK(Edit(Record(4), 0x00010004u, Record(4) + 2), "an additive node marked differences");
    CHECK(Edit(Record(0), 0x00040001u, Record(0) + 2), "flags past differences and mirroring");
    CHECK(Edit(Record(0), 0x00020001u, Record(2) + 2), "a node mirrored under no mirror node");
    CHECK(Edit(Record(8) + 8, 3, Record(8) + 8), "inputs past the section");
    CHECK(Edit(Record(0) + 24, g.nodeCount, Record(0) + 24), "a source place past the count");
    CHECK(Edit(Record(0) + 28, 1, Record(0) + 28), "a time factor below its reader's");
    CHECK(Edit(Record(8) + 8, 1, SectionAt(maniGraphTagInputs)), "inputs left over");
    uint64_t parameters = SectionAt(maniGraphTagParameters);
    CHECK(Edit(parameters + 12, 0, parameters + 8), "an empty name");
    CHECK(Edit(parameters + 16, 1, parameters + 8), "a name of another hash");
    CHECK(Edit(parameters + 28, 1, parameters + 24), "a float's unused word");
    CHECK(Edit(parameters + 2 * MANI_GRAPH_PARAMETER_SIZE + 24, 2,
               parameters + 2 * MANI_GRAPH_PARAMETER_SIZE + 24),
          "a bool of 2");
    CHECK(Edit(parameters + 40, 0xFF800000u, parameters + 24), "a bound not finite");
    uint64_t first = MANI_CONTAINER_HEADER_SIZE;
    uint32_t sections = maniLoadU32(s_built + 12);
    uint64_t last = first + (uint64_t)(sections - 1) * MANI_CONTAINER_ENTRY_SIZE;
    CHECK(Edit(first, 0, first), "a section tag of 0");
    CHECK(Edit(last, MANI_GRAPH_SECTIONS + 1, last), "a section tag past the kinds");
    CHECK(Edit(first + 16, 40, first + 16), "a header of another size");
    CHECK(Edit(last + 16, 0, last + 16), "an empty section");
    uint64_t empty = maniHashBytes(MANI_HASH_INIT, NULL, 0);
    const uint64_t unnamed[6] = {parameters + 12, 0,
                                 parameters + 16, (uint32_t)empty,
                                 parameters + 20, (uint32_t)(empty >> 32)};
    CHECK(RefusedAt(unnamed, 3, parameters + 8), "an empty name with its hash");
    uint64_t groups = SectionAt(maniGraphTagSyncGroups);
    CHECK(Edit(groups + 16, 2, groups + 16), "a sync group's flag past allEvents");
    // The output a clip node of the wave, marked as differences: the
    // output gives differences.
    maniGraphNodeSource walk = ClipNode(0, 1.0f, maniGraphLoopClip);
    maniGraphSource single = {&s_skeleton, NULL, kSlots, NULL, &walk, 0,    CLIPS, 0,   1,
                              0,           0,    NULL,   0,    0,     NULL, NULL,  NULL};
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildGraph(builder, &single).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) == mani_success,
          "a single walk");
    maniDestroyBuilder(builder);
    const uint64_t edits[4] = {Word(0, 0), 2, Record(0), 0x00010001u};
    CHECK(RefusedAt(edits, 2, Record(0) + 2), "an output of differences");
}

// A subgraph blending the run and the walk by x, its output first: x
// read from b, its clips joined to the graph's by identity. Built as the
// same graph written flat.
static void TestSubgraphs(void)
{
    static const uint32_t kInner[2] = {1, 2};
    static const maniGraphScalar kWeights[2] = {{0, 0.0f}, LITERAL(1.0f)};
    static const maniGraphClipSource kLegs[2] = {{&s_clips[1], "run", 3, NULL},
                                                 {&s_clips[0], "walk", 4, NULL}};
    maniGraphParameterSource x = Parameter("x", maniGraphParameterFloat, Float(0.5f));
    maniGraphNodeSource inner[3] = {{maniGraphNodeBlend, 2, kInner, .blend = {kWeights, 0}},
                                    ClipNode(0, 1.0f, maniGraphLoopClip),
                                    ClipNode(1, 1.0f, maniGraphLoopClip)};
    maniGraphSource sub = {&s_skeleton, &x, kLegs, NULL, inner, 1,    2,    0,   3,
                           0,           0,  NULL,  0,    0,     NULL, NULL, NULL};
    maniGraphParameterSource parameters[2] = {Parameter("a", maniGraphParameterFloat, Float(0.0f)),
                                              Parameter("b", maniGraphParameterFloat, Float(0.5f))};
    static const uint32_t kMap[1] = {1};
    static const uint32_t kScale[1] = {1};
    maniGraphNodeSource outer[2] = {
        {maniGraphNodeTimeScale, 1, kScale, .timeScale = {LITERAL(1.5f)}},
        {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL, .subgraph = {&sub, kMap}}};
    static const uint32_t kFlatPair[2] = {2, 3};
    static const maniGraphScalar kFlatWeights[2] = {{1, 0.0f}, LITERAL(1.0f)};
    maniGraphNodeSource flat[4] = {
        {maniGraphNodeTimeScale, 1, kScale, .timeScale = {LITERAL(1.5f)}},
        {maniGraphNodeBlend, 2, kFlatPair, .blend = {kFlatWeights, 0}},
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    maniGraph g;
    CHECK(Opens(
              (maniGraphSource){
                  .parameters = parameters, .nodes = flat, .parameterCount = 2, .nodeCount = 4},
              &g),
          "the flat graph");
    static _Alignas(16) uint8_t flatFile[16384];
    size_t flatSize = s_builtSize;
    memcpy(flatFile, s_built, s_builtSize);
    maniGraphSource source = {
        .parameters = parameters, .nodes = outer, .parameterCount = 2, .nodeCount = 2};
    maniBuilder* builder = NewBuilder();
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_built, sizeof s_built, &s_builtSize) ==
                  mani_success &&
              s_builtSize == flatSize && memcmp(s_built, flatFile, flatSize) == 0,
          "inlined as written flat");
    maniDestroyBuilder(builder);
    sub.skeleton = &s_chain;
    CHECK(Refused(source, maniBuildReasonNode, 1, 0), "a subgraph of another skeleton");
    sub.skeleton = &s_skeleton;
    sub.output = 3;
    CHECK(Refused(source, maniBuildReasonNode, 1, 0), "a subgraph's output past its nodes");
    sub.output = 0;
    outer[1].inputCount = 1;
    outer[1].inputs = kScale;
    CHECK(Refused(source, maniBuildReasonNode, 1, 0), "a subgraph node with inputs");
    // A slot without its clip under two clip nodes: one slot, refused.
    static const maniGraphClipSource kNone[1] = {{NULL, "none", 4, NULL}};
    outer[1].inputCount = 0;
    outer[1].inputs = NULL;
    inner[2] = ClipNode(0, 1.0f, maniGraphLoopClip);
    sub.clips = kNone;
    sub.clipCount = 1;
    CHECK(Refused(source, maniBuildReasonClip, CLIPS, 0), "a subgraph's slot without its clip");
    // A slot past the subgraph's clips, and a sync group past its groups,
    // where the root lists one of that slot: refused at the clip node's
    // place, not played as the root's.
    sub.clips = kLegs;
    sub.clipCount = 2;
    inner[2] = ClipNode(3, 1.0f, maniGraphLoopClip);
    CHECK(Refused(source, maniBuildReasonNode, 3, 0), "a subgraph's slot past its clips");
    static const maniGraphSyncGroupSource kFeet = {"feet", 4, 0};
    source.syncGroups = &kFeet;
    source.syncGroupCount = 1;
    inner[2] = ClipNode(1, 1.0f, maniGraphLoopClip);
    inner[2].clip.syncGroup = 0;
    sub.syncGroups = &kFeet;
    sub.syncGroupCount = 1;
    CHECK(BuildOnly(source).result == mani_success, "a subgraph's sync group joined by name");
    sub.syncGroups = NULL;
    sub.syncGroupCount = 0;
    CHECK(Refused(source, maniBuildReasonNode, 3, 0), "a subgraph's sync group past its groups");
    // Each of a subgraph's lists not given with its count, refused as the
    // root's would be.
    inner[2].clip.syncGroup = MANI_NO_SYNC_GROUP;
    CHECK(BuildOnly(source).result == mani_success, "the subgraph without its sync group");
    sub.parameters = NULL;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's parameters not given");
    sub.parameters = &x;
    sub.clips = NULL;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's clips not given");
    sub.clips = kLegs;
    sub.syncGroupCount = 1;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's sync groups not given");
    sub.syncGroupCount = 0;
    sub.databaseCount = 1;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's databases not given");
    sub.databaseCount = 0;
    sub.modelCount = 1;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's models not given");
    sub.modelCount = 0;
    sub.springSetCount = 1;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's spring sets not given");
    // A subgraph's sync group without its name, the key it joins the
    // graph's groups by: refused with the lists, before inlining.
    static const maniGraphSyncGroupSource kNameless = {NULL, 0, 0};
    sub.springSetCount = 0;
    sub.syncGroups = &kNameless;
    sub.syncGroupCount = 1;
    inner[2].clip.syncGroup = 0;
    CHECK(Refused(source, maniBuildReasonCount, 0, 0), "a subgraph's sync group without its name");
}

// A clip slot past the root's own clips, and a sync group past its own
// groups, where a subgraph lists one of that number: refused at the clip
// node's place, not played as the subgraph's.
static void TestRootSlots(void)
{
    static const maniGraphClipSource kRun[1] = {{&s_clips[1], "run", 3, NULL}};
    static const maniGraphSyncGroupSource kFeet = {"feet", 4, 0};
    maniGraphNodeSource inner[1] = {ClipNode(0, 1.0f, maniGraphLoopClip)};
    inner[0].clip.syncGroup = 0;
    maniGraphSource sub = {.skeleton = &s_skeleton,
                           .clips = kRun,
                           .syncGroups = &kFeet,
                           .nodes = inner,
                           .clipCount = 1,
                           .syncGroupCount = 1,
                           .nodeCount = 1};
    static const uint32_t kPair[2] = {1, 2};
    static const maniGraphScalar kHalves[2] = {LITERAL(0.5f), LITERAL(0.5f)};
    maniGraphNodeSource nodes[3] = {
        {maniGraphNodeBlend, 2, kPair, .blend = {kHalves, 0}},
        {(maniGraphNodeKind)maniGraphNodeSubgraph, 0, NULL, .subgraph = {&sub, NULL}},
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    maniGraphSource source = {.skeleton = &s_skeleton,
                              .clips = kSlots,
                              .syncGroups = &kFeet,
                              .nodes = nodes,
                              .clipCount = 1,
                              .syncGroupCount = 1,
                              .nodeCount = 3};
    CHECK(BuildOnly(source).result == mani_success, "the root's own clip");
    nodes[2].clip.clip = 1;
    CHECK(Refused(source, maniBuildReasonNode, 2, 0), "a root's slot past its clips");
    nodes[2].clip.clip = 0;
    nodes[2].clip.syncGroup = 0;
    CHECK(BuildOnly(source).result == mani_success, "the root's own sync group");
    source.syncGroups = NULL;
    source.syncGroupCount = 0;
    CHECK(Refused(source, maniBuildReasonNode, 2, 0), "a root's sync group past its groups");
}

static float FactorAt(uint32_t p)
{
    uint32_t bits = maniLoadU32(s_built + Record(p) + 28);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// The place in the file of source node v.
static uint32_t PlaceOf(const maniGraph* g, uint32_t v)
{
    for (uint32_t p = 0; p < g->nodeCount; ++p)
    {
        if (maniGraph_GetNodeSourcePlace(g, p) == v)
        {
            return p;
        }
    }
    return UINT32_MAX;
}

// Time factors: each record keeps the largest product of the time scales'
// sizes above its node, a parameter's at the larger size of its bounds,
// so scales on parallel branches do not multiply. A path past
// MANI_MAX_GRAPH_TIME_FACTOR, a scale from a parameter without bounds and
// a factor that does not bound its inputs' are refused.
static void TestTimeFactors(void)
{
    // An output blend of four time scales: 1e4 above the walk, 1e4 above
    // the run, -4 and rate (bounds -5 and 2) both above the short clip.
    static const uint32_t kBranches[4] = {1, 2, 3, 4};
    static const maniGraphScalar kOnes[4] = {LITERAL(1.0f), LITERAL(1.0f), LITERAL(1.0f),
                                             LITERAL(1.0f)};
    static const uint32_t kWalk[1] = {5};
    static const uint32_t kRun[1] = {6};
    static const uint32_t kShort[1] = {7};
    maniGraphParameterSource rate = Parameter("rate", maniGraphParameterFloat, Float(1.0f));
    rate.bounded = 1;
    rate.minimum = Float(-5.0f);
    rate.maximum = Float(2.0f);
    maniGraphNodeSource nodes[8] = {
        {maniGraphNodeBlend, 4, kBranches, .blend = {kOnes, 0}},
        {maniGraphNodeTimeScale, 1, kWalk, .timeScale = {LITERAL(1.0e4f)}},
        {maniGraphNodeTimeScale, 1, kRun, .timeScale = {LITERAL(1.0e4f)}},
        {maniGraphNodeTimeScale, 1, kShort, .timeScale = {LITERAL(-4.0f)}},
        {maniGraphNodeTimeScale, 1, kShort, .timeScale = {{0, 0.0f}}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(3, 1.0f, maniGraphLoopClip)};
    maniGraphSource source = {
        .parameters = &rate, .nodes = nodes, .parameterCount = 1, .nodeCount = 8};
    maniGraph g;
    bool opens = Opens(source, &g);
    CHECK(opens, "parallel scales past the bound together, not on a path");
    if (!opens)
    {
        return;
    }
    static const float kWant[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0e4f, 1.0e4f, 5.0f};
    bool factors = true;
    for (uint32_t p = 0; p < g.nodeCount; ++p)
    {
        factors = factors && FactorAt(p) == kWant[maniGraph_GetNodeSourcePlace(&g, p)];
    }
    CHECK(factors, "each node's largest product, a parameter's at its bounds' larger size");
    uint32_t output = g.nodeCount - 1;
    uint32_t walk = PlaceOf(&g, 5);
    uint32_t clip = PlaceOf(&g, 7);
    uint32_t scale = PlaceOf(&g, 1);
    CHECK(Edit(Record(output) + 28, 0x40000000u, Record(output) + 28), "an output factor of 2");
    CHECK(Edit(Record(output) + 28, 0x3F000000u, Record(output) + 28), "an output factor of 0.5");
    CHECK(Edit(Record(scale) + 28, 0x3F000000u, Record(scale) + 28), "a factor below its reader's");
    CHECK(Edit(Record(walk) + 28, 0x461C3FFFu, Record(walk) + 28),
          "a factor below its reader's times the scale");
    CHECK(Edit(Record(clip) + 28, 0x409FFFFFu, Record(clip) + 28),
          "a factor below its parameter's larger bound");
    CHECK(Edit(Record(walk) + 28, 0x49742401u, Record(walk) + 28), "a factor past the bound");
    CHECK(Edit(Record(walk) + 28, 0x7F800000u, Record(walk) + 28), "a factor not finite");
    CHECK(Edit(Record(walk) + 28, 0x7FC00000u, Record(walk) + 28), "a factor not a number");
    memcpy(s_crafted, s_built, s_builtSize);
    maniStoreU32(s_crafted + Record(walk) + 28, 0x49742400u);
    Rehash(s_crafted, s_builtSize);
    maniGraph crafted;
    CHECK(maniOpenGraph(s_crafted, s_builtSize, &crafted).result == mani_success,
          "a factor at the bound, past its readers' need");
    uint64_t parameters = SectionAt(maniGraphTagParameters);
    const uint64_t unbounded[6] = {parameters + 4, 0, parameters + 40, 0, parameters + 44, 0};
    CHECK(RefusedAt(unbounded, 3, Word(PlaceOf(&g, 4), 0)), "a scale from an unbounded parameter");
    rate.bounded = 0;
    rate.minimum = Float(0.0f);
    rate.maximum = Float(0.0f);
    CHECK(Refused(source, maniBuildReasonScalar, 4, 0), "a scale from an unbounded parameter");
    // Two scales on a path: 1000 above 1000 reaches the bound, 1000 above
    // 1001 passes it.
    static const uint32_t kInner[1] = {1};
    static const uint32_t kClip[1] = {2};
    maniGraphNodeSource path[3] = {
        {maniGraphNodeTimeScale, 1, kInner, .timeScale = {LITERAL(1000.0f)}},
        {maniGraphNodeTimeScale, 1, kClip, .timeScale = {LITERAL(1000.0f)}},
        ClipNode(0, 1.0f, maniGraphLoopClip)};
    maniGraphSource chain = {.nodes = path, .nodeCount = 3};
    CHECK(Opens(chain, &g) && FactorAt(PlaceOf(&g, 2)) == MANI_MAX_GRAPH_TIME_FACTOR,
          "a path at the bound");
    path[1].timeScale.scale.value = 0.0f;
    opens = Opens(chain, &g);
    CHECK(opens && FactorAt(PlaceOf(&g, 2)) == 0.0f, "a path through a scale of 0");
    CHECK(opens && Edit(Record(PlaceOf(&g, 2)) + 28, 0x80000000u, Record(PlaceOf(&g, 2)) + 28),
          "a factor of -0");
    path[1].timeScale.scale.value = 1001.0f;
    CHECK(PastLimit(chain, maniBuildReasonScalar, 1, 0), "a path past the bound");
    path[1].timeScale.scale.value = 1.0f;
    path[0].timeScale.scale.value = 1.5e6f;
    CHECK(PastLimit(chain, maniBuildReasonScalar, 0, 0), "a scale past the bound");
}

// A binding's cursors as large as its largest clip's, and its layers as
// many as its node of most inputs, wherever they come.
static void TestBindingRoom(void)
{
    static const uint32_t kScale[1] = {1};
    static const uint32_t kThree[3] = {2, 3, 4};
    static const maniGraphScalar kOnes[3] = {LITERAL(1.0f), LITERAL(1.0f), LITERAL(1.0f)};
    const maniGraphClipSource slots[2] = {{&s_chainClips[2], "all", 3, NULL},
                                          {&s_chainClips[0], "one", 3, NULL}};
    maniGraphNodeSource nodes[5] = {
        {maniGraphNodeTimeScale, 1, kScale, .timeScale = {LITERAL(1.0f)}},
        {maniGraphNodeBlend, 3, kThree, .blend = {kOnes, 0}},
        ClipNode(0, 1.0f, maniGraphLoopClip),
        ClipNode(1, 1.0f, maniGraphLoopClip),
        ClipNode(1, 2.0f, maniGraphLoopClip)};
    maniGraph g;
    const maniClip* clips[2] = {&s_chainClips[2], &s_chainClips[0]};
    maniGraphBinding* binding = NULL;
    CHECK(Opens((maniGraphSource){.skeleton = &s_chain,
                                  .clips = slots,
                                  .nodes = nodes,
                                  .clipCount = 2,
                                  .nodeCount = 5},
                &g) &&
              maniBindGraph(&g,
                            &(maniGraphAssets){&s_chain, clips, NULL, NULL, NULL, 2, 0, 0, 0, NULL,
                                               NULL, 0, 0, NULL},
                            s_chainBinding, sizeof s_chainBinding, &binding, NULL) == mani_success,
          "bound");
    size_t largest = (maniClip_GetCursorSize(&s_chainClips[2]) + 7u) & ~(size_t)7u;
    CHECK(binding->cursorSize == largest && maniClip_GetCursorSize(&s_chainClips[0]) < largest &&
              binding->maxInputs == 3,
          "room for the largest cursor and the most inputs");
}

int main(void)
{
    Open();
    OpenChain();
    TestRefusals();
    TestSlotFreedOnce();
    TestIgnoredValues();
    TestTwoMasks();
    TestChainMask();
    TestOpenRefusals();
    TestSubgraphs();
    TestRootSlots();
    TestTimeFactors();
    TestBindingRoom();
    return s_failures == 0 ? 0 : 1;
}
