// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Graphs playing another skeleton's clips through retargeting maps:
// a walk with root motion and an additive lean of the
// source, both mapped, under an additive node of the target's graph;
// each evaluation the transfer of the walk plus the lean carried as the
// bake carries it, each update's root motion the walk's scaled; the map
// slots read back; the builder's, binder's and opener's refusals; the
// JSON form and subgraphs keeping the maps.

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
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFiles[4][4096];
static _Alignas(16) uint8_t s_mapFiles[4][16384];
static _Alignas(16) uint8_t s_clipFiles[2][65536];
static _Alignas(16) uint8_t s_graphFile[65536];
static _Alignas(16) uint8_t s_crafted[65536];
static _Alignas(16) uint8_t s_text[65536];
static _Alignas(MANI_GRAPH_BINDING_ALIGNMENT) uint8_t s_bindingMemory[8192];
static _Alignas(MANI_GRAPH_INSTANCE_ALIGNMENT) uint8_t s_instanceMemory[65536];
static _Alignas(MANI_GRAPH_SCRATCH_ALIGNMENT) uint8_t s_scratch[131072];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[7][4096];
static _Alignas(16) uint8_t s_transfer[8192];
static size_t s_graphSize;
static maniSkeleton s_source;
static maniSkeleton s_target;
static maniSkeleton s_otherSource;
static maniSkeleton s_otherTarget;
static maniRetargetMap s_map;
static maniRetargetMap s_chainsOnly;
static maniRetargetMap s_elsewhere;
static maniRetargetMap s_fromOther;
static maniClip s_walk;
static maniClip s_lean;
static maniPose* s_in;
static maniPose* s_whole;
static maniPose* s_out;
static maniPose* s_base;
static maniPose* s_expected;
static maniPose* s_layer;
static maniPose* s_otherIn;

typedef struct Bone
{
    const char* name;
    int32_t parent;
    maniVec3 offset;
    maniQuat rotation;
} Bone;

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

static const Bone kSource[8] = {
    {"root", -1, {0.0f, 0.0f, 0.0f}, REST},
    {"hips", 0, {0.0f, 1.0f, 0.0f}, REST},
    {"s1", 1, {0.0f, 0.2f, 0.0f}, {0.17364818f, 0.0f, 0.0f, 0.98480775f}},
    {"s2", 2, {0.0f, 0.2f, 0.0f}, REST},
    {"chest", 3, {0.0f, 0.2f, 0.0f}, REST},
    {"armL", 4, {0.2f, 0.1f, 0.0f}, {0.0f, 0.0f, 0.25881904f, 0.96592583f}},
    {"handL", 5, {0.3f, 0.0f, 0.0f}, REST},
    {"head", 4, {0.0f, 0.3f, 0.0f}, REST}};

static const Bone kTarget[10] = {
    {"root", -1, {0.1f, 0.0f, 0.0f}, REST},
    {"pelvis", 0, {0.0f, 0.9f, 0.0f}, REST},
    {"sp1", 1, {0.0f, 0.15f, 0.0f}, {0.0f, 0.0f, 0.25881904f, 0.96592583f}},
    {"sp2", 2, {0.0f, 0.15f, 0.0f}, REST},
    {"sp3", 3, {0.0f, 0.15f, 0.0f}, REST},
    {"chest", 4, {0.0f, 0.15f, 0.0f}, REST},
    {"upperarm", 5, {0.2f, 0.1f, 0.0f}, {0.0f, 0.70710678f, 0.0f, 0.70710678f}},
    {"forearm", 6, {0.0f, 0.0f, -0.15f}, REST},
    {"hand", 7, {0.0f, 0.0f, -0.15f}, REST},
    {"head", 5, {0.0f, 0.3f, 0.0f}, REST}};

static const maniRetargetChainSource kChains[2] = {
    {2, 4, 2, 5, maniRetargetRotationInterpolated, 1.0f, maniRetargetTranslationOwn, false},
    {5, 6, 6, 8, maniRetargetRotationOneToOne, 0.5f, maniRetargetTranslationScaled, true}};
static const maniRetargetPelvisSource kPelvis = {
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 1.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 0.75f, 1.0f};

static const float kTimes[3] = {0.0f, 0.5f, 1.0f};
static const float kRootMove[9] = {0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.6f, 0.9f, 0.05f, 1.2f};
static const float kYaw[12] = {0.0f, 0.0f,        0.0f, 1.0f,        0.0f, 0.17364818f,
                               0.0f, 0.98480775f, 0.0f, 0.38268343f, 0.0f, 0.92387953f};
static const float kHipsMove[9] = {0.0f, 1.0f, 0.0f, 0.0f, 0.95f, 0.02f, 0.0f, 1.0f, 0.0f};
static const float kSpine[12] = {0.17364818f, 0.0f, 0.0f, 0.98480775f,
                                 0.30070580f, 0.0f, 0.0f, 0.95371695f,
                                 0.17364818f, 0.0f, 0.0f, 0.98480775f};
static const float kArm[12] = {0.0f,        0.0f, 0.25881904f, 0.96592583f, 0.0f,       0.0f, 0.5f,
                               0.86602540f, 0.0f, 0.0f,        0.38268343f, 0.92387953f};
static const float kLean[12] = {0.17364818f, 0.0f, 0.0f, 0.98480775f,
                                0.25881904f, 0.0f, 0.0f, 0.96592583f,
                                0.34202014f, 0.0f, 0.0f, 0.93969262f};
static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};

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

static void Skeleton(const Bone* bones, uint32_t count, int side, maniSkeleton* out)
{
    maniBoneSource sources[16];
    for (uint32_t b = 0; b < count; ++b)
    {
        sources[b] = (maniBoneSource){bones[b].name,
                                      (uint32_t)strlen(bones[b].name),
                                      bones[b].parent,
                                      {bones[b].offset, bones[b].rotation, {1, 1, 1}}};
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {sources, count};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFiles[side], sizeof s_skeletonFiles[side],
                                    &size) == mani_success &&
              maniOpenSkeleton(s_skeletonFiles[side], size, out).result == mani_success,
          "a skeleton");
    maniDestroyBuilder(builder);
}

static void Map(const maniRetargetMapSource* source, int slot, maniRetargetMap* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_mapFiles[slot], sizeof s_mapFiles[slot], &size) ==
                  mani_success &&
              maniOpenRetargetMap(s_mapFiles[slot], size, out).result == mani_success,
          "a map");
    maniDestroyBuilder(builder);
}

static void Clip(const maniClipSource* source, int slot, maniClip* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_clipFiles[slot], sizeof s_clipFiles[slot], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[slot], size, out).result == mani_success,
          "a clip");
    maniDestroyBuilder(builder);
}

static void Clips(void)
{
    const maniBoneTrackSource walk[4] = {{0, Keys(kRootMove), Keys(kYaw), kNone},
                                         {1, Keys(kHipsMove), kNone, kNone},
                                         {2, kNone, Keys(kSpine), kNone},
                                         {5, kNone, Keys(kArm), kNone}};
    const maniRootMotionSource root = {0, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
                                       1, maniRootReferenceRest,         maniRootReferenceRest,
                                       0};
    const maniClipSource w = {.skeleton = &s_source,
                              .boneTracks = walk,
                              .boneTrackCount = 4,
                              .loop = maniLoopRepeat,
                              .sampleRate = 30.0f,
                              .rootMotion = &root};
    Clip(&w, 0, &s_walk);
    const maniBoneTrackSource lean[1] = {{3, kNone, Keys(kLean), kNone}};
    const maniClipSource l = {.skeleton = &s_source,
                              .boneTracks = lean,
                              .boneTrackCount = 1,
                              .loop = maniLoopRepeat,
                              .sampleRate = 30.0f,
                              .additive = maniAdditiveRest};
    Clip(&l, 1, &s_lean);
}

static void Setup(void)
{
    Skeleton(kSource, 8, 0, &s_source);
    Skeleton(kTarget, 10, 1, &s_target);
    Bone sourceBones[8];
    Bone targetBones[10];
    memcpy(sourceBones, kSource, sizeof sourceBones);
    memcpy(targetBones, kTarget, sizeof targetBones);
    sourceBones[7].offset.y = 0.4f;
    targetBones[9].offset.y = 0.4f;
    Skeleton(sourceBones, 8, 2, &s_otherSource);
    Skeleton(targetBones, 10, 3, &s_otherTarget);
    maniRetargetMapSource source = {.source = &s_source,
                                    .target = &s_target,
                                    .chains = kChains,
                                    .pelvis = &kPelvis,
                                    .chainCount = 2,
                                    .matchNames = true};
    Map(&source, 0, &s_map);
    source.matchNames = false;
    Map(&source, 1, &s_chainsOnly);
    source.matchNames = true;
    source.target = &s_otherTarget;
    Map(&source, 2, &s_elsewhere);
    source.source = &s_otherSource;
    source.target = &s_target;
    Map(&source, 3, &s_fromOther);
    Clips();
    const maniSkeleton* skeletons[7] = {&s_source, &s_source, &s_target,     &s_target,
                                        &s_target, &s_target, &s_otherSource};
    maniPose** poses[7] = {&s_in, &s_whole, &s_out, &s_base, &s_expected, &s_layer, &s_otherIn};
    for (int i = 0; i < 7; ++i)
    {
        CHECK(maniInitPose(skeletons[i], s_poseMemory[i], sizeof s_poseMemory[i], poses[i]) ==
                  mani_success,
              "a pose");
    }
}

// The walk and the lean through maps, the lean added at weight 1.
static const uint32_t kBoth[2] = {0, 1};
static const maniGraphScalar kFull[1] = {{MANI_NO_PARAMETER, 1.0f}};
static maniGraphNodeSource s_nodes[3];
static maniGraphClipSource s_clips[2];

static maniGraphSource Graph(const maniRetargetMap* walk, const maniRetargetMap* lean)
{
    s_clips[0] = (maniGraphClipSource){&s_walk, "walk", 4, walk};
    s_clips[1] = (maniGraphClipSource){&s_lean, "lean", 4, lean};
    const maniGraphClipNodeSource play = {
        0, MANI_NO_SYNC_GROUP, {MANI_NO_PARAMETER, 1.0f}, maniGraphLoopClip, 0};
    s_nodes[0] = (maniGraphNodeSource){maniGraphNodeClip, 0, NULL, .clip = play};
    s_nodes[1] = s_nodes[0];
    s_nodes[1].clip.clip = 1;
    s_nodes[2] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kBoth, .additive = {kFull}};
    return (maniGraphSource){.skeleton = &s_target,
                             .clips = s_clips,
                             .nodes = s_nodes,
                             .clipCount = 2,
                             .nodeCount = 3,
                             .output = 2};
}

static maniBuildResult BuildGraph(const maniGraphSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildGraph(builder, source);
    s_graphSize = 0;
    if (r.result == mani_success)
    {
        CHECK(maniBuilder_CopyBuilt(builder, s_graphFile, sizeof s_graphFile, &s_graphSize) ==
                  mani_success,
              "the graph copied");
    }
    maniDestroyBuilder(builder);
    return r;
}

static const maniClip* const kAllClips[2] = {&s_walk, &s_lean};

static maniGraphAssets Assets(const maniRetargetMap* const* maps, uint32_t mapCount,
                              const maniSkeleton* const* sources, uint32_t sourceCount)
{
    return (maniGraphAssets){.skeleton = &s_target,
                             .clips = kAllClips,
                             .clipCount = 2,
                             .retargetMaps = maps,
                             .sourceSkeletons = sources,
                             .retargetMapCount = mapCount,
                             .sourceSkeletonCount = sourceCount};
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
    for (uint32_t bone = 0; bone < 10; ++bone)
    {
        maniTransform x = maniPose_GetLocal(a, bone);
        maniTransform y = maniPose_GetLocal(b, bone);
        same = same && Gap(x.translation, y.translation) < 1e-5 &&
               QuatDot(x.rotation, y.rotation) > 1.0 - 1e-10 && Gap(x.scale, y.scale) < 1e-5;
    }
    return same;
}

static bool Transfer(const maniPose* in, maniPose* out)
{
    return maniRetargetMap_RetargetPose(&s_map, &s_source, in, &s_target, out, s_transfer,
                                        sizeof s_transfer) == mani_success;
}

// The walk carried at a time plus the lean carried as the bake carries
// it: its difference on the source's rest, carried, less the source's
// rest carried.
static bool Expected(float time)
{
    maniBlendLayer layer = {s_in, 1.0f, NULL};
    bool ok =
        maniClip_Sample(&s_walk, &s_source, time, s_in) == mani_success &&
        Transfer(s_in, s_expected) &&
        maniClip_Sample(&s_lean, &s_source, time, s_in) == mani_success &&
        maniPose_SetRest(s_whole, &s_source) == mani_success &&
        maniPose_AddLayers(s_whole, &s_source, &layer, 1) == mani_success &&
        Transfer(s_whole, s_layer) && maniPose_SetRest(s_whole, &s_source) == mani_success &&
        Transfer(s_whole, s_base) &&
        maniSkeleton_ComputePoseDifference(&s_target, s_layer, s_base, s_layer) == mani_success;
    layer.pose = s_layer;
    return ok && maniPose_AddLayers(s_expected, &s_target, &layer, 1) == mani_success;
}

static bool Bound(const maniGraph* g, maniGraphInstance** instance)
{
    const maniRetargetMap* maps[1] = {&s_map};
    const maniSkeleton* sources[1] = {&s_source};
    maniGraphAssets assets = Assets(maps, 1, sources, 1);
    maniGraphBinding* binding = NULL;
    return maniBindGraph(g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
               mani_success &&
           maniGraphBinding_GetScratchSize(binding) <= sizeof s_scratch &&
           maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory, instance) ==
               mani_success;
}

static void TestPlayback(void)
{
    maniGraphSource source = Graph(&s_map, &s_map);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of mapped clips builds");
    maniGraph g;
    maniGraphInstance* instance = NULL;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              Bound(&g, &instance),
          "opened and bound");
    CHECK(
        maniGraph_GetRetargetMapCount(&g) == 1 &&
            maniGraph_GetRetargetMapIdentity(&g, 0) == s_map.identity &&
            maniGraph_GetRetargetMapSourceIdentity(&g, 0) == maniSkeleton_GetIdentity(&s_source) &&
            maniGraph_GetRetargetMapIdentity(&g, 1) == 0 &&
            maniGraph_GetRetargetMapSourceIdentity(&g, 1) == 0 &&
            maniGraph_GetClipRetargetMap(&g, 0) == 0 && maniGraph_GetClipRetargetMap(&g, 1) == 0 &&
            maniGraph_GetClipRetargetMap(&g, 2) == MANI_NO_TRACK &&
            maniGraph_GetRetargetMapCount(NULL) == 0,
        "the map slot read back");
    bool poses = true;
    bool moves = true;
    for (int k = 1; k <= 12; ++k)
    {
        maniRootMove move;
        maniRootMove want;
        float from = (float)(k - 1) * 0.1f;
        poses = poses &&
                maniGraphInstance_Update(instance, 0.1f, NULL, 0, NULL, &move) == mani_success &&
                maniGraphInstance_Evaluate(instance, s_scratch, sizeof s_scratch, s_out) ==
                    mani_success &&
                Expected(fmodf((float)k * 0.1f, 1.0f)) && Same(s_out, s_expected);
        moves = moves &&
                maniClip_GetRootMotion(&s_walk, fmodf(from, 1.0f), 0.1f, &want) == mani_success &&
                Gap(move.translation,
                    maniRetargetMap_RetargetRootMotion(&s_map, want.translation)) < 1e-5 &&
                QuatDot(move.rotation, want.rotation) > 1.0 - 1e-10;
    }
    CHECK(poses, "each pose the walk carried plus the lean carried");
    CHECK(moves, "each move the walk's scaled");
}

static void TestBuildRefusals(void)
{
    maniGraphSource source = Graph(&s_elsewhere, &s_map);
    maniBuildResult r = BuildGraph(&source);
    CHECK(r.reason == maniBuildReasonClip && r.item == 0, "a map for another target");
    source = Graph(NULL, &s_map);
    r = BuildGraph(&source);
    CHECK(r.reason == maniBuildReasonClip && r.item == 0, "a source clip without a map");
    // Clips of the target and of a rig of the source's count through a
    // map from the source.
    const maniSkeleton* rigs[2] = {&s_target, &s_otherSource};
    for (int i = 0; i < 2; ++i)
    {
        static _Alignas(16) uint8_t file[4096];
        const maniClipSource still = {.skeleton = rigs[i], .sampleRate = 30.0f};
        maniBuilder* builder = NewBuilder();
        size_t size = 0;
        maniClip clip;
        CHECK(maniBuildClip(builder, &still).result == mani_success &&
                  maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
                  maniOpenClip(file, size, &clip).result == mani_success,
              "a still clip");
        maniDestroyBuilder(builder);
        source = Graph(&s_map, &s_map);
        s_clips[1].clip = &clip;
        r = BuildGraph(&source);
        CHECK(r.reason == maniBuildReasonClip && r.item == 1, "a clip not of the map's source");
    }
}

// MANI_MAX_GRAPH_RETARGET_MAPS maps and one more, each playing a still clip of
// its own: the last slot refused.
static void TestTooManyMaps(void)
{
    enum
    {
        kCount = MANI_MAX_GRAPH_RETARGET_MAPS + 1
    };
    static _Alignas(16) uint8_t mapFiles[kCount][2048];
    static _Alignas(16) uint8_t clipFiles[kCount][1024];
    static maniRetargetMap maps[kCount];
    static maniClip clips[kCount];
    static maniGraphClipSource slots[kCount];
    static char names[kCount][8];
    maniBuilder* builder = NewBuilder();
    bool made = true;
    for (uint32_t k = 0; k < kCount; ++k)
    {
        maniRetargetPelvisSource pelvis = kPelvis;
        pelvis.translationWeight = (float)k / (float)kCount;
        const maniRetargetMapSource m = {
            .source = &s_source, .target = &s_target, .pelvis = &pelvis, .matchNames = true};
        const maniClipSource c = {
            .skeleton = &s_source, .sampleRate = 30.0f, .duration = (float)(k + 1) / 30.0f};
        size_t size = 0;
        made = made && maniBuildRetargetMap(builder, &m).result == mani_success &&
               maniBuilder_CopyBuilt(builder, mapFiles[k], sizeof mapFiles[k], &size) ==
                   mani_success &&
               maniOpenRetargetMap(mapFiles[k], size, &maps[k]).result == mani_success &&
               maniBuildClip(builder, &c).result == mani_success &&
               maniBuilder_CopyBuilt(builder, clipFiles[k], sizeof clipFiles[k], &size) ==
                   mani_success &&
               maniOpenClip(clipFiles[k], size, &clips[k]).result == mani_success;
        int length = snprintf(names[k], sizeof names[k], "c%u", (unsigned)k);
        slots[k] = (maniGraphClipSource){&clips[k], names[k], (size_t)length, &maps[k]};
    }
    maniDestroyBuilder(builder);
    CHECK(made, "the maps and clips");
    maniGraphSource source = Graph(&s_map, &s_map);
    source.clips = slots;
    source.clipCount = kCount;
    source.nodeCount = 1;
    source.output = 0;
    maniBuildResult r = BuildGraph(&source);
    CHECK(r.reason == maniBuildReasonClip && r.item == MANI_MAX_GRAPH_RETARGET_MAPS,
          "a map too many");
    source.clipCount = kCount - 1;
    CHECK(BuildGraph(&source).result == mani_success, "as many maps as the graph takes");
}

static void TestBindRefusals(void)
{
    maniGraphSource source = Graph(&s_map, &s_map);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of mapped clips builds");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    const maniRetargetMap* maps[2] = {&s_elsewhere, &s_map};
    const maniSkeleton* sources[2] = {&s_otherSource, &s_source};
    const maniGraphAssets cases[4] = {Assets(NULL, 0, sources, 2), Assets(maps, 2, NULL, 0),
                                      Assets(maps, 2, sources, 1), Assets(maps, 1, sources, 2)};
    const char* what[4] = {"no map", "no source skeleton", "a source skeleton of another rig",
                           "only a map of another target"};
    maniGraphBinding* binding = NULL;
    for (int i = 0; i < 4; ++i)
    {
        uint32_t unbound = 0;
        CHECK(maniBindGraph(&g, &cases[i], s_bindingMemory, sizeof s_bindingMemory, &binding,
                            &unbound) == mani_errorInvalid &&
                  unbound == 2,
              what[i]);
    }
    maniGraphAssets wrong = Assets(maps + 1, 1, sources + 1, 1);
    wrong.retargetMaps = NULL;
    CHECK(maniBindGraph(&g, &wrong, s_bindingMemory, sizeof s_bindingMemory, &binding, NULL) ==
              mani_errorInvalid,
          "maps NULL with a count");
    // With a spring set slot, the map slot's place after it.
    static _Alignas(16) uint8_t setFile[4096];
    const maniSpringJointSource joints[2] = {{.bone = 7, .stiffness = 1.0f},
                                             {.bone = 8, .stiffness = 1.0f}};
    const maniSpringSource spring = {"arm", 3, 2, joints, NULL, 0, MANI_NO_BONE};
    const maniSpringSetSource setSource = {
        .skeleton = &s_target, .springs = &spring, .springCount = 1};
    maniBuilder* builder = NewBuilder();
    size_t setSize = 0;
    maniSpringSet set;
    CHECK(maniBuildSpringSet(builder, &setSource).result == mani_success &&
              maniBuilder_CopyBuilt(builder, setFile, sizeof setFile, &setSize) == mani_success &&
              maniOpenSpringSet(setFile, setSize, &set).result == mani_success,
          "a spring set");
    maniDestroyBuilder(builder);
    const maniGraphSpringSetSource slot = {&set, "arm", 3};
    source = Graph(&s_map, &s_map);
    source.springSets = &slot;
    source.springSetCount = 1;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success,
          "a graph of a spring set slot");
    const maniSpringSet* sets[1] = {&set};
    maniGraphAssets withSet = Assets(NULL, 0, sources, 2);
    withSet.springSets = sets;
    withSet.springSetCount = 1;
    uint32_t place = 0;
    CHECK(maniBindGraph(&g, &withSet, s_bindingMemory, sizeof s_bindingMemory, &binding, &place) ==
                  mani_errorInvalid &&
              place == 3,
          "the map slot past the spring set");
    // No target bone follows the source's root alone.
    source = Graph(&s_chainsOnly, &s_chainsOnly);
    CHECK(BuildGraph(&source).result == mani_success, "a graph through the chains' map builds");
    const maniRetargetMap* chains[1] = {&s_chainsOnly};
    maniGraphAssets assets = Assets(chains, 1, sources + 1, 1);
    uint32_t unbound = 0;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              maniBindGraph(&g, &assets, s_bindingMemory, sizeof s_bindingMemory, &binding,
                            &unbound) == mani_errorInvalid &&
              unbound == MANI_NO_TRACK,
          "root motion no target bone follows");
}

// Two graphs whose maps take different source rigs, evaluated in turn
// with one scratch: each carries its own clip.
static void TestSharedScratch(void)
{
    static _Alignas(16) uint8_t file[4096];
    static _Alignas(16) uint8_t otherGraph[16384];
    static _Alignas(MANI_GRAPH_BINDING_ALIGNMENT) uint8_t otherBinding[8192];
    static _Alignas(MANI_GRAPH_INSTANCE_ALIGNMENT) uint8_t otherInstance[16384];
    const maniBoneTrackSource spine[1] = {{2, kNone, Keys(kSpine), kNone}};
    const maniClipSource bend = {
        .skeleton = &s_otherSource, .boneTracks = spine, .boneTrackCount = 1, .sampleRate = 30.0f};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniClip clip;
    CHECK(maniBuildClip(builder, &bend).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenClip(file, size, &clip).result == mani_success,
          "a clip of the other rig");
    maniDestroyBuilder(builder);
    // The other graph: that clip alone through its map.
    maniGraphSource source = Graph(&s_fromOther, &s_fromOther);
    s_clips[0].clip = &clip;
    source.clipCount = 1;
    source.nodeCount = 1;
    source.output = 0;
    maniGraph other;
    CHECK(BuildGraph(&source).result == mani_success, "the other graph builds");
    memcpy(otherGraph, s_graphFile, s_graphSize);
    const maniClip* clips[1] = {&clip};
    const maniRetargetMap* maps[1] = {&s_fromOther};
    const maniSkeleton* sources[1] = {&s_otherSource};
    maniGraphAssets assets = {.skeleton = &s_target,
                              .clips = clips,
                              .clipCount = 1,
                              .retargetMaps = maps,
                              .sourceSkeletons = sources,
                              .retargetMapCount = 1,
                              .sourceSkeletonCount = 1};
    maniGraphBinding* binding = NULL;
    maniGraphInstance* b = NULL;
    CHECK(maniOpenGraph(otherGraph, s_graphSize, &other).result == mani_success &&
              maniBindGraph(&other, &assets, otherBinding, sizeof otherBinding, &binding, NULL) ==
                  mani_success &&
              maniInitGraphInstance(binding, otherInstance, sizeof otherInstance, &b) ==
                  mani_success,
          "the other graph bound");
    source = Graph(&s_map, &s_map);
    maniGraph g;
    maniGraphInstance* a = NULL;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success && Bound(&g, &a),
          "the walk's graph bound");
    bool same = true;
    for (int k = 0; k < 3; ++k)
    {
        float t = 0.1f * (float)(k + 1);
        same = same && maniGraphInstance_Update(a, 0.1f, NULL, 0, NULL, NULL) == mani_success &&
               maniGraphInstance_Evaluate(a, s_scratch, sizeof s_scratch, s_out) == mani_success &&
               Expected(t) && Same(s_out, s_expected);
        same = same && maniGraphInstance_Update(b, 0.1f, NULL, 0, NULL, NULL) == mani_success &&
               maniGraphInstance_Evaluate(b, s_scratch, sizeof s_scratch, s_out) == mani_success &&
               maniClip_Sample(&clip, &s_otherSource, t, s_otherIn) == mani_success &&
               maniRetargetMap_RetargetPose(&s_fromOther, &s_otherSource, s_otherIn, &s_target,
                                            s_expected, s_transfer,
                                            sizeof s_transfer) == mani_success &&
               Same(s_out, s_expected);
    }
    CHECK(same, "each graph its own clip carried");
}

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_graphFile, s_graphSize, maniKindGraph, MANI_GRAPH_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

static bool RefusedAt(uint64_t offset, const void* value, size_t length, uint64_t want)
{
    memcpy(s_crafted, s_graphFile, s_graphSize);
    memcpy(s_crafted + offset, value, length);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               (uint64_t)s_graphSize - MANI_CONTAINER_HEADER_SIZE));
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
    // Two maps: the walk's and the lean's.
    maniGraphSource source = Graph(&s_map, &s_chainsOnly);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of two maps builds");
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              maniGraph_GetRetargetMapCount(&g) == 2 && maniGraph_GetClipRetargetMap(&g, 1) == 1 &&
              maniGraph_GetRetargetMapIdentity(&g, 1) == s_chainsOnly.identity,
          "two map slots");
    uint64_t clips = SectionAt(maniGraphTagClips);
    uint64_t maps = SectionAt(maniGraphTagMaps);
    uint8_t word[8];
    maniStoreU32(word, 2);
    CHECK(RefusedAt(clips + MANI_GRAPH_CLIP_SIZE + 28, word, 4, clips + MANI_GRAPH_CLIP_SIZE + 28),
          "a map slot past the maps");
    maniStoreU64(word, s_map.identity);
    CHECK(RefusedAt(maps + MANI_GRAPH_MAP_SIZE, word, 8, maps + MANI_GRAPH_MAP_SIZE),
          "a map twice");
    maniStoreU32(word, MANI_GRAPH_NO_MAP);
    CHECK(RefusedAt(clips + 28, word, 4, 40), "no map passes every check but the identity");
}

// The JSON form names the clip slots, whose host sources carry the maps:
// built back, the same file.
static void TestJson(void)
{
    maniGraphSource source = Graph(&s_map, &s_map);
    CHECK(BuildGraph(&source).result == mani_success, "a graph of mapped clips builds");
    uint64_t identity = 0;
    maniGraph g;
    CHECK(maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success, "opened");
    identity = maniGraph_GetIdentity(&g);
    maniBuilder* builder = NewBuilder();
    size_t length = 0;
    CHECK(maniWriteGraphJson(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_text, sizeof s_text, &length) == mani_success,
          "the JSON written");
    maniGraphNamedAssets named = {.skeleton = &s_target, .clips = s_clips, .clipCount = 2};
    size_t size = 0;
    CHECK(maniBuildGraphJson(builder, (const char*)s_text, length, &named).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_graphFile, sizeof s_graphFile, &size) ==
                  mani_success &&
              maniOpenGraph(s_graphFile, size, &g).result == mani_success &&
              maniGraph_GetIdentity(&g) == identity,
          "built back the same");
    maniDestroyBuilder(builder);
}

// A subgraph playing the walk through the same map joins its slot;
// through another map it is a slot of its own, refused (its name the
// walk's).
static void TestSubgraph(void)
{
    maniGraphSource inner = Graph(&s_map, &s_map);
    inner.nodeCount = 1;
    inner.output = 0;
    inner.clipCount = 1;
    static maniGraphClipSource clips[2];
    static maniGraphNodeSource nodes[2];
    clips[0] = s_clips[0];
    clips[1] = s_clips[1];
    nodes[0] = s_nodes[0];
    nodes[1] = s_nodes[2];
    inner.clips = clips;
    inner.nodes = nodes;
    static maniGraphNodeSource outer[3];
    static const uint32_t kPair[2] = {0, 1};
    outer[0] = (maniGraphNodeSource){maniGraphNodeSubgraph, 0, NULL, .subgraph = {&inner, NULL}};
    outer[1] = s_nodes[1];
    outer[2] = (maniGraphNodeSource){maniGraphNodeAdditive, 2, kPair, .additive = {kFull}};
    maniGraphSource source = Graph(&s_map, &s_map);
    source.nodes = outer;
    maniGraph g;
    CHECK(BuildGraph(&source).result == mani_success &&
              maniOpenGraph(s_graphFile, s_graphSize, &g).result == mani_success &&
              maniGraph_GetClipCount(&g) == 2 && maniGraph_GetRetargetMapCount(&g) == 1,
          "the subgraph's walk joins the slot");
    clips[0].retargetMap = &s_chainsOnly;
    maniBuildResult r = BuildGraph(&source);
    // Its own slot, whose name the root's walk has.
    CHECK(r.reason == (maniBuildReasonClipList | maniBuildReasonDuplicateName) && r.item == 2,
          "one clip through two maps a slot of its own, refused");
}

int main(void)
{
    Setup();
    TestPlayback();
    TestBuildRefusals();
    TestTooManyMaps();
    TestBindRefusals();
    TestSharedScratch();
    TestOpen();
    TestJson();
    TestSubgraph();
    return s_failures == 0 ? 0 : 1;
}
