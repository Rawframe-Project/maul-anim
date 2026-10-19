// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs with mirror nodes (mani-0033) on a rig whose left and right
// arms have unmirrored axes: a walk under a mirror node evaluated as its
// pose mirrored, its root motion reflected and its events renamed; two
// mirrors cancelling; a mirrored follower synced to its unmirrored leader
// by its markers renamed (half a loop on); the map's identity read back;
// the builder's refusals (no map, a map of another rig, a node reached
// both mirrored and not, differences, a subgraph's other map); the
// binder's and the opener's; the JSON form and subgraphs keeping the
// map.

#include "bytes.h"
#include "container.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum
{
    kRoot,
    kHips,
    kChest,
    kClavL,
    kArmL,
    kClavR,
    kArmR,
    kBones
};

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

static const maniBoneSource kRig[kBones] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"hips", 4, kRoot, {{0, 1, 0}, REST, {1, 1, 1}}},
    {"chest", 5, kHips, {{0, 0.4f, 0}, REST, {1, 1, 1}}},
    {"clavL", 5, kChest, {{0.1f, 0.1f, 0}, {0, 0, 0.25881904f, 0.96592583f}, {1, 1, 1}}},
    {"armL", 4, kClavL, {{0.2f, 0, 0}, REST, {1, 1, 1}}},
    {"clavR", 5, kChest, {{-0.1f, 0.1f, 0}, {0, 0.70710678f, 0, 0.70710678f}, {1, 1, 1}}},
    {"armR", 4, kClavR, {{0, 0, 0.2f}, REST, {1, 1, 1}}},
};

static const maniMirrorRuleSource kRules[2] = {{"L", 1, maniMirrorSuffix, "R", 1, 0},
                                               {"R", 1, maniMirrorSuffix, "L", 1, 0}};
static const maniMirrorNameSource kNames[3] = {{"stepL", 5, 0}, {"L", 1, 0}, {"R", 1, 0}};

static const float kTimes[3] = {0.0f, 0.5f, 1.0f};
static const float kRootMove[9] = {0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.6f, 0.9f, 0.05f, 1.2f};
static const float kYaw[12] = {0.0f, 0.0f,        0.0f, 1.0f,        0.0f, 0.17364818f,
                               0.0f, 0.98480775f, 0.0f, 0.38268343f, 0.0f, 0.92387953f};
static const float kArm[12] = {0.0f, 0.0f,        0.25881904f, 0.96592583f, 0.1f, 0.2f,
                               0.3f, 0.92736185f, 0.0f,        0.5f,        0.0f, 0.8660254f};
static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};

static _Alignas(16) uint8_t s_skeletonFiles[3][4096];
static _Alignas(16) uint8_t s_mapFiles[4][4096];
static _Alignas(16) uint8_t s_clipFiles[2][65536];
static _Alignas(16) uint8_t s_graphFile[65536];
static _Alignas(16) uint8_t s_crafted[65536];
static _Alignas(16) uint8_t s_text[65536];
static _Alignas(MANI_GRAPH_BINDING_ALIGNMENT) uint8_t s_bindingMemory[8192];
static _Alignas(MANI_GRAPH_INSTANCE_ALIGNMENT) uint8_t s_instanceMemory[65536];
static _Alignas(MANI_GRAPH_SCRATCH_ALIGNMENT) uint8_t s_scratch[131072];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[3][2048];
static size_t s_graphSize;
static maniSkeleton s_skeleton;
static maniSkeleton s_other;
static maniSkeleton s_moved;
// Mirror maps across x and z, and of the other rig and the moved one.
static maniMirrorMap s_map;
static maniMirrorMap s_mapZ;
static maniMirrorMap s_otherMap;
static maniMirrorMap s_movedMap;
static maniClip s_walk;
static maniClip s_lean;
static maniPose* s_in;
static maniPose* s_out;
static maniPose* s_expected;

static maniChannelSource Keys(const float* values)
{
    return (maniChannelSource){kTimes, values, 3, maniInterpolationLinear};
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

static void OpenSkeleton(const maniBoneSource* bones, uint32_t count, uint8_t* file,
                         maniSkeleton* out)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, count, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, 4096, &size) == mani_success &&
              maniOpenSkeleton(file, size, out).result == mani_success,
          "a rig");
    maniDestroyBuilder(builder);
}

static void OpenMap(const maniSkeleton* skeleton, maniMirrorAxis axis, uint8_t* file,
                    maniMirrorMap* out)
{
    maniMirrorMapSource source = {.skeleton = skeleton,
                                  .rules = kRules,
                                  .names = kNames,
                                  .ruleCount = 2,
                                  .nameCount = 3,
                                  .axis = axis};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, 4096, &size) == mani_success &&
              maniOpenMirrorMap(file, size, out).result == mani_success,
          "a map");
    maniDestroyBuilder(builder);
}

static void OpenClip(const maniClipSource* source, int slot, maniClip* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_clipFiles[slot], sizeof s_clipFiles[slot], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[slot], size, out).result == mani_success,
          "a clip");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    OpenSkeleton(kRig, kBones, s_skeletonFiles[0], &s_skeleton);
    OpenSkeleton(kRig, kBones - 1, s_skeletonFiles[1], &s_other);
    maniBoneSource moved[kBones];
    memcpy(moved, kRig, sizeof moved);
    moved[kChest].rest.translation.y = 0.45f;
    OpenSkeleton(moved, kBones, s_skeletonFiles[2], &s_moved);
    OpenMap(&s_skeleton, maniMirrorAxisX, s_mapFiles[0], &s_map);
    OpenMap(&s_skeleton, maniMirrorAxisZ, s_mapFiles[1], &s_mapZ);
    OpenMap(&s_other, maniMirrorAxisX, s_mapFiles[2], &s_otherMap);
    OpenMap(&s_moved, maniMirrorAxisX, s_mapFiles[3], &s_movedMap);
    const maniBoneTrackSource walk[2] = {{kRoot, 0, Keys(kRootMove), Keys(kYaw), kNone},
                                         {kArmL, 0, kNone, Keys(kArm), kNone}};
    const maniEventSource events[2] = {{0.25f, 7, "stepL", 5, 0}, {0.75f, 9, "jump", 4, 0}};
    const maniEventSource markers[2] = {{0.0f, 0, "L", 1, 0}, {0.5f, 0, "R", 1, 0}};
    const maniRootMotionSource root = {kRoot,
                                       maniRootAxisX | maniRootAxisZ,
                                       maniRootRotationTwist,
                                       1,
                                       maniRootReferenceRest,
                                       maniRootReferenceRest,
                                       0,
                                       0};
    const maniClipSource w = {.skeleton = &s_skeleton,
                              .boneTracks = walk,
                              .events = events,
                              .markers = markers,
                              .boneTrackCount = 2,
                              .eventCount = 2,
                              .markerCount = 2,
                              .loop = maniLoopRepeat,
                              .sampleRate = 30.0f,
                              .duration = 1.0f,
                              .rootMotion = &root};
    OpenClip(&w, 0, &s_walk);
    const maniBoneTrackSource lean[1] = {{kChest, 0, kNone, Keys(kArm), kNone}};
    const maniClipSource l = {.skeleton = &s_skeleton,
                              .boneTracks = lean,
                              .boneTrackCount = 1,
                              .loop = maniLoopRepeat,
                              .sampleRate = 30.0f,
                              .additive = maniAdditiveRest};
    OpenClip(&l, 1, &s_lean);
    maniPose** poses[3] = {&s_in, &s_out, &s_expected};
    for (int i = 0; i < 3; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], poses[i]) ==
                  mani_success,
              "a pose");
    }
}

static const uint32_t kFirst[1] = {0};
static const uint32_t kSecond[1] = {1};
static const uint32_t kPair[2] = {0, 2};
static const maniGraphScalar kHalves[2] = {{MANI_NO_PARAMETER, 0.5f}, {MANI_NO_PARAMETER, 0.5f}};
static maniGraphNodeSource s_nodes[4];
static maniGraphClipSource s_clips[2];
static const maniGraphSyncGroupSource kFeet = {"feet", 4, 0};

static maniGraphNodeSource ClipNode(uint32_t clip, uint32_t group, uint32_t flags)
{
    const maniGraphClipNodeSource play = {
        clip, group, {MANI_NO_PARAMETER, 1.0f}, maniGraphLoopClip, flags};
    return (maniGraphNodeSource){maniGraphNodeClip, 0, NULL, .clip = play};
}

static maniGraphNodeSource MirrorNode(const uint32_t* input)
{
    return (maniGraphNodeSource){maniGraphNodeMirror, 1, input, .clip = {0}};
}

// The walk under mirrors nodes deep (0 to 2).
static maniGraphSource Graph(uint32_t mirrors, const maniMirrorMap* map)
{
    s_clips[0] = (maniGraphClipSource){&s_walk, "walk", 4, 0, NULL};
    s_clips[1] = (maniGraphClipSource){&s_lean, "lean", 4, 0, NULL};
    s_nodes[0] = ClipNode(0, MANI_NO_SYNC_GROUP, 0);
    s_nodes[1] = MirrorNode(kFirst);
    s_nodes[2] = MirrorNode(kSecond);
    return (maniGraphSource){.skeleton = &s_skeleton,
                             .clips = s_clips,
                             .nodes = s_nodes,
                             .clipCount = 1,
                             .nodeCount = mirrors + 1,
                             .output = mirrors,
                             .mirrorMap = map};
}

static maniBuildResult BuildGraph(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    s_graphSize = 0;
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_graphFile, sizeof s_graphFile, &s_graphSize) == mani_success,
              "the graph copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static const maniClip* const kAllClips[2] = {&s_walk, &s_lean};

static maniGraphAssets Assets(const maniMirrorMap* map)
{
    return (maniGraphAssets){
        .skeleton = &s_skeleton, .clips = kAllClips, .clipCount = 2, .mirrorMap = map};
}

static bool Bound(const maniGraph* g, maniGraphInstance** instance)
{
    maniGraphAssets assets = Assets(&s_map);
    maniGraphBinding* binding = NULL;
    return maniBindGraph(g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
               mani_success &&
           maniGraphScratchSize(binding) <= sizeof s_scratch &&
           maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory, instance) ==
               mani_success;
}

static double QuatDot(maniQuat a, maniQuat b)
{
    const double p[4] = {(double)a.x, (double)a.y, (double)a.z, (double)a.w};
    const double q[4] = {(double)b.x, (double)b.y, (double)b.z, (double)b.w};
    double d = 0.0;
    double na = 0.0;
    double nb = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        d += p[i] * q[i];
        na += p[i] * p[i];
        nb += q[i] * q[i];
    }
    return fabs(d) / sqrt(na * nb);
}

static double Gap(maniVec3 a, maniVec3 b)
{
    return fmax(fabs((double)a.x - (double)b.x),
                fmax(fabs((double)a.y - (double)b.y), fabs((double)a.z - (double)b.z)));
}

static bool Same(const maniPose* a, const maniPose* b)
{
    bool same = true;
    for (uint32_t bone = 0; bone < kBones; ++bone)
    {
        maniTransform x = maniPoseLocal(a, bone);
        maniTransform y = maniPoseLocal(b, bone);
        same = same && Gap(x.translation, y.translation) < 1e-5 &&
               QuatDot(x.rotation, y.rotation) > 1.0 - 1e-10 && Gap(x.scale, y.scale) < 1e-5;
    }
    return same;
}

static uint64_t Hash(const char* name)
{
    return maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, strlen(name));
}

// Ten updates of a tenth through the walk under mirrors nodes: each pose
// the walk's (mirrored when odd), each move the walk's (reflected), the
// events renamed.
static void Play(uint32_t mirrors)
{
    maniGraphSource source = Graph(mirrors, &s_map);
    maniGraph g;
    maniGraphInstance* instance = NULL;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              Bound(&g, &instance),
          "built, opened and bound");
    CHECK(maniGraphMirrorMapIdentity(&g) == s_map.identity && maniGraphMirrorMapIdentity(NULL) == 0,
          "the map's identity read back");
    bool odd = mirrors % 2 == 1;
    bool poses = true;
    bool moves = true;
    uint64_t names[4] = {0};
    uint32_t heard = 0;
    for (int k = 1; k <= 10; ++k)
    {
        maniRootMove move;
        maniRootMove want;
        maniGraphEvent events[4];
        uint32_t count = 0;
        float from = (float)(k - 1) * 0.1f;
        poses =
            poses && maniUpdateGraph(instance, 0.1f, events, 4, &count, &move) == mani_success &&
            maniEvaluateGraph(instance, s_scratch, sizeof s_scratch, s_out) == mani_success &&
            maniSampleClip(&s_walk, &s_skeleton, fmodf((float)k * 0.1f, 1.0f), s_in) ==
                mani_success &&
            (odd ? maniMirrorPose(&s_map, &s_skeleton, s_in, s_expected) == mani_success
                 : maniSampleClip(&s_walk, &s_skeleton, fmodf((float)k * 0.1f, 1.0f), s_expected) ==
                       mani_success) &&
            Same(s_out, s_expected);
        moves = moves && maniClipRootMotion(&s_walk, from, 0.1f, &want) == mani_success;
        want = odd ? maniMirrorRootMotion(&s_map, want) : want;
        moves = moves && Gap(move.translation, want.translation) < 1e-6 &&
                QuatDot(move.rotation, want.rotation) > 1.0 - 1e-12;
        for (uint32_t i = 0; i < count && heard < 4; ++i)
        {
            names[heard++] = events[i].nameHash;
        }
    }
    CHECK(poses, odd ? "each pose the walk's mirrored" : "each pose the walk's");
    CHECK(moves, odd ? "each move the walk's reflected" : "each move the walk's");
    CHECK(heard == 2 && names[0] == Hash(odd ? "stepR" : "stepL") && names[1] == Hash("jump"),
          odd ? "the events renamed" : "the events as named");
}

static maniGraphNodeState* State(maniGraphInstance* instance, const maniGraph* g, uint32_t p)
{
    return &instance->cells[g->parameterCount + p].node;
}

// A walk leading its group, a walk following it under a mirror: the
// follower's markers renamed, so it plays half a loop on.
static void TestSync(void)
{
    maniGraphSource source = Graph(1, &s_map);
    s_nodes[0] = ClipNode(0, 0, maniGraphClipLeader);
    s_nodes[1] = ClipNode(0, 0, 0);
    s_nodes[2] = MirrorNode(kSecond);
    s_nodes[3] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kPair, .blend = {kHalves, 0, 0}};
    source.syncGroups = &kFeet;
    source.syncGroupCount = 1;
    source.nodeCount = 4;
    source.output = 3;
    maniGraph g;
    maniGraphInstance* instance = NULL;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              Bound(&g, &instance),
          "a synced graph bound");
    bool half = true;
    for (int k = 1; k <= 7; ++k)
    {
        half = half && maniUpdateGraph(instance, 0.15f, NULL, 0, NULL, NULL) == mani_success;
        double lead = State(instance, &g, 0)->time;
        double follow = State(instance, &g, 1)->time;
        half = half && fabs(fmod(lead + 0.5, 1.0) - follow) < 1e-6;
    }
    CHECK(half, "the mirrored follower half a loop on");
}

static bool Refused(const maniGraphSource* source, maniBuildReason reason, uint32_t item,
                    uint32_t other)
{
    maniBuildResult r = BuildGraph(source);
    if (r.reason != reason || r.item != item || r.other != other)
    {
        printf("  reason %u item %u other %u\n", r.reason, r.item, r.other);
    }
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item &&
           r.other == other;
}

static void TestBuildRefusals(void)
{
    maniGraphSource source = Graph(1, NULL);
    CHECK(Refused(&source, maniBuildReasonMirrorGraph, 1, 1), "a mirror node without a map");
    source = Graph(1, &s_otherMap);
    CHECK(Refused(&source, maniBuildReasonMirrorGraph, 0, 0), "a map of another rig");
    source = Graph(1, &s_movedMap);
    CHECK(Refused(&source, maniBuildReasonMirrorGraph, 0, 0), "a map of a rig of the bone count");
    maniMirrorMap counted = s_map;
    counted.boneCount = kBones - 1;
    source = Graph(1, &counted);
    CHECK(Refused(&source, maniBuildReasonMirrorGraph, 0, 0), "a map of another bone count");
    // The walk blended with its own mirror.
    source = Graph(1, &s_map);
    static const uint32_t kBoth[2] = {0, 1};
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeBlend, 2, kBoth, .blend = {kHalves, 0, 0}};
    source.nodeCount = 3;
    source.output = 2;
    CHECK(Refused(&source, maniBuildReasonMirrorGraph, 0, 2), "a node reached both ways");
    source = Graph(1, &s_map);
    s_nodes[0].clip.clip = 1;
    source.clipCount = 2;
    CHECK(Refused(&source, maniBuildReasonDifference, 1, 0), "differences mirrored");
    // The lean mirrored as an additive node's layer.
    source = Graph(1, &s_map);
    static const uint32_t kLayer[2] = {2, 1};
    static const maniGraphScalar kFull[1] = {{MANI_NO_PARAMETER, 1.0f}};
    s_nodes[0].clip.clip = 1;
    s_nodes[2] = ClipNode(0, MANI_NO_SYNC_GROUP, 0);
    s_nodes[3] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kLayer, .additive = {kFull}};
    source.clipCount = 2;
    source.nodeCount = 4;
    source.output = 3;
    CHECK(Refused(&source, maniBuildReasonDifference, 1, 0), "differences mirrored as a layer");
    // A map without a mirror node is kept.
    source = Graph(0, &s_map);
    maniGraph g;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              maniGraphMirrorMapIdentity(&g) == s_map.identity,
          "a map kept without a mirror node");
}

static void TestBindRefusals(void)
{
    maniGraphSource source = Graph(1, &s_map);
    maniGraph g;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success,
          "built and opened");
    const maniMirrorMap* maps[2] = {NULL, &s_mapZ};
    const char* what[2] = {"no map", "another map"};
    for (int i = 0; i < 2; ++i)
    {
        maniGraphAssets assets = Assets(maps[i]);
        maniGraphBinding* binding = NULL;
        uint32_t unbound = 0;
        CHECK(maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding,
                            &unbound) == mani_errorInvalid &&
                  unbound == 1,
              what[i]);
    }
    // A map of the identity but another rig: a crafted view.
    maniMirrorMap crafted = s_map;
    crafted.boneCount = kBones - 1;
    maniGraphAssets assets = Assets(&crafted);
    maniGraphBinding* binding = NULL;
    CHECK(maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
              mani_errorInvalid,
          "a map of another bone count");
    crafted = s_map;
    crafted.skeletonIdentity ^= 1u;
    CHECK(maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
              mani_errorInvalid,
          "a map of another skeleton");
}

static bool Section(const uint8_t* file, size_t size, uint32_t tag, const uint8_t** data,
                    uint64_t* length, uint64_t* at)
{
    maniContainer c;
    return maniOpenContainer(file, size, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c).result ==
               mani_success &&
           maniContainerSection(&c, tag, data, length, at);
}

static bool RefusedAt(uint64_t offset, const void* value, size_t length, uint64_t want)
{
    memcpy(s_crafted, s_graphFile, s_graphSize);
    memcpy(s_crafted + offset, value, length);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + 64, (uint64_t)s_graphSize - 64));
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_crafted, s_graphSize, &g);
    if (r.offset != want)
    {
        printf("  at %llu, expected %llu\n", (unsigned long long)r.offset,
               (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want;
}

static void TestOpen(void)
{
    maniGraphSource source = Graph(1, &s_map);
    CHECK(BuildGraph(&source).result == mani_success, "built");
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(Section(s_graphFile, s_graphSize, maniGraphTagMirror, &data, &size, &at) && size == 8,
          "the mirror section");
    const uint8_t zero[8] = {0};
    CHECK(RefusedAt(at, zero, 8, at), "a mirror map identity of 0");
    // Two mirrors over the walk, the walk and the first mirror marked as
    // differences: the first mirror refused for taking them.
    source = Graph(2, &s_map);
    CHECK(BuildGraph(&source).result == mani_success, "two mirrors built");
    uint64_t clips = 0;
    uint64_t records = 0;
    CHECK(Section(s_graphFile, s_graphSize, maniGraphTagClips, &data, &size, &clips) &&
              Section(s_graphFile, s_graphSize, maniGraphTagNodes, &data, &size, &records),
          "the clips and nodes");
    memcpy(s_crafted, s_graphFile, s_graphSize);
    maniStoreU32(s_crafted + clips + 24, MANI_GRAPH_DIFFERENCE);
    maniStoreU16(s_crafted + records + 2,
                 (uint16_t)(maniLoadU16(s_crafted + records + 2) | MANI_GRAPH_DIFFERENCE));
    maniStoreU16(s_crafted + records + MANI_GRAPH_NODE_SIZE + 2,
                 (uint16_t)(maniLoadU16(s_crafted + records + MANI_GRAPH_NODE_SIZE + 2) |
                            MANI_GRAPH_DIFFERENCE));
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + 64, (uint64_t)s_graphSize - 64));
    maniGraph crafted;
    maniFileResult differences = maniOpenGraph(s_crafted, s_graphSize, &crafted);
    CHECK(differences.result == mani_errorFormat &&
              differences.offset == records + MANI_GRAPH_NODE_SIZE + 2,
          "a mirror of differences");
    source = Graph(1, &s_map);
    CHECK(BuildGraph(&source).result == mani_success &&
              Section(s_graphFile, s_graphSize, maniGraphTagMirror, &data, &size, &at),
          "one mirror built again");
    // The section's size in the table: its entry is the last.
    maniContainer c;
    CHECK(maniOpenContainer(s_graphFile, s_graphSize, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c)
                  .result == mani_success,
          "the container");
    uint64_t entry = MANI_CONTAINER_HEADER_SIZE + (uint64_t)(c.sectionCount - 1) * 24;
    CHECK(maniLoadU32(s_graphFile + entry) == maniGraphTagMirror, "the mirror entry last");
    uint8_t four[8];
    maniStoreU64(four, 4);
    CHECK(RefusedAt(entry + 16, four, 8, entry + 16), "a mirror section of 4 bytes");
    // The file written again without the section: the mirror node
    // refused.
    maniSectionSource sections[MANI_GRAPH_SECTIONS];
    const uint8_t* parts[MANI_GRAPH_SECTIONS];
    uint64_t sizes[MANI_GRAPH_SECTIONS];
    uint32_t count = 0;
    for (uint32_t t = 1; t <= MANI_GRAPH_SECTIONS; ++t)
    {
        parts[t - 1] = data;
        sizes[t - 1] = 0;
        if (t < maniGraphTagMirror &&
            Section(s_graphFile, s_graphSize, t, &parts[t - 1], &sizes[t - 1], &at))
        {
            sections[count++] = (maniSectionSource){t, parts[t - 1], sizes[t - 1]};
        }
    }
    uint64_t fileSize = 0;
    CHECK(maniContainerSize(sections, count, &fileSize) && fileSize <= sizeof s_crafted,
          "room for the file");
    maniWriteContainer(s_crafted, maniKindGraph, MANI_GRAPH_FORMAT_VERSION,
                       maniGraphComputeIdentity(parts, sizes), sections, count);
    uint64_t nodes = 0;
    CHECK(Section(s_crafted, (size_t)fileSize, maniGraphTagNodes, &data, &size, &nodes),
          "the nodes");
    maniGraph g;
    maniFileResult r = maniOpenGraph(s_crafted, (size_t)fileSize, &g);
    CHECK(r.result == mani_errorFormat && r.offset == nodes + MANI_GRAPH_NODE_SIZE,
          "a mirror node without a map");
}

static void TestJson(void)
{
    maniGraphSource source = Graph(2, &s_map);
    maniGraph g;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success,
          "built and opened");
    uint64_t identity = maniGraphIdentity(&g);
    maniBuilder* builder = NewBuilder();
    size_t length = 0;
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success,
          "the JSON written");
    CHECK(strstr((const char*)s_text, "{\"kind\": \"mirror\", \"inputs\": [0]}") != NULL,
          "a mirror node by its inputs alone");
    maniGraphNamedAssets named = {
        .skeleton = &s_skeleton, .clips = s_clips, .clipCount = 1, .mirrorMap = &s_map};
    size_t size = 0;
    CHECK(maniBuildGraphJson(builder, (const char*)s_text, length, &named).result == mani_success &&
              maniCopyBuilt(builder, s_graphFile, sizeof s_graphFile, &size) == mani_success &&
              maniOpenGraph(s_graphFile, size, &g).result == mani_success &&
              maniGraphIdentity(&g) == identity,
          "built back the same");
    maniDestroyBuilder(builder);
}

// A subgraph mirroring the walk through the root's map, through none
// (taking the subgraph's), or through another map: refused.
static void TestSubgraph(void)
{
    static maniGraphNodeSource inner[2];
    maniGraphSource sub = Graph(1, &s_map);
    inner[0] = s_nodes[0];
    inner[1] = s_nodes[1];
    sub.nodes = inner;
    static maniGraphNodeSource outer[1];
    outer[0] = (maniGraphNodeSource){maniGraphNodeSubgraph, 0, NULL, .subgraph = {&sub, NULL}};
    maniGraphSource source = Graph(0, &s_map);
    source.nodes = outer;
    source.clipCount = 0;
    const maniMirrorMap* roots[2] = {&s_map, NULL};
    for (int i = 0; i < 2; ++i)
    {
        source.mirrorMap = roots[i];
        maniGraph g;
        CHECK(BuildGraph(&source).result == mani_success &&
                  maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
                  maniGraphMirrorMapIdentity(&g) == s_map.identity,
              i == 0 ? "the subgraph's map the root's" : "the subgraph's map taken");
    }
    source.mirrorMap = &s_mapZ;
    CHECK(Refused(&source, maniBuildReasonMirrorGraph, 0, 0), "a subgraph's other map");
}

int main(void)
{
    Setup();
    Play(1);
    Play(2);
    TestSync();
    TestBuildRefusals();
    TestBindRefusals();
    TestOpen();
    TestJson();
    TestSubgraph();
    return s_failures == 0 ? 0 : 1;
}
