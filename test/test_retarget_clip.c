// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking clips through a retargeting map: every sample of the
// baked clip the pose transfer of the source's (from a full-precision and
// a compressed source); the timing, float tracks (a step one stays a
// step), discrete tracks, events and markers kept; channels at the
// target's rest left out; additive clips (rest and first-frame bases)
// as differences carried from the source's rest; root motion's moves
// scaled, and turned with their axes into a turned parent's frame and
// out of one (all axes and the whole rotation where the turn leaves the
// axes, the axes kept where a scale of 0 takes them to nothing); the
// refusals; the baked clip's bits pinned, the same on every platform.

#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFiles[6][4096];
static _Alignas(16) uint8_t s_mapFiles[5][16384];
static _Alignas(16) uint8_t s_clipFiles[4][65536];
static _Alignas(16) uint8_t s_baked[65536];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[6][4096];
static _Alignas(16) uint8_t s_scratch[8192];
static maniSkeleton s_source;
static maniSkeleton s_target;
static maniSkeleton s_otherSource;
static maniSkeleton s_otherTarget;
static maniSkeleton s_turnedSource;
static maniSkeleton s_turnedTarget;
static maniRetargetMap s_map;
static maniRetargetMap s_chainsOnly;
static maniRetargetMap s_turnedMap;
static maniRetargetMap s_flat;
static maniRetargetMap s_flatHand;
static maniPose* s_in;
static maniPose* s_whole;
static maniPose* s_out;
static maniPose* s_base;
static maniPose* s_expected;
static maniPose* s_turnedIn;

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

// A two-bone pair whose target root rests turned 90 degrees about x.
static const Bone kTurnedSource[2] = {{"root", -1, {0.0f, 0.0f, 0.0f}, REST},
                                      {"hips", 0, {0.0f, 1.0f, 0.0f}, REST}};
static const Bone kTurnedTarget[2] = {
    {"root", -1, {0.0f, 0.0f, 0.0f}, {0.70710678f, 0.0f, 0.0f, 0.70710678f}},
    {"hips", 0, {0.0f, 0.0f, -1.0f}, REST}};

static const maniRetargetChainSource kChains[2] = {
    {2, 4, 2, 5, maniRetargetRotationInterpolated, 1.0f, maniRetargetTranslationOwn, false},
    {5, 6, 6, 8, maniRetargetRotationOneToOne, 0.5f, maniRetargetTranslationScaled, true}};
static const maniRetargetPelvisSource kPelvis = {
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 1.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 0.75f, 1.0f};

// The source clip's keys, at 0, 0.5 and 1 seconds.
static const float kTimes[3] = {0.0f, 0.5f, 1.0f};
static const float kRootMove[9] = {0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.6f, 0.9f, 0.05f, 1.2f};
static const float kYaw[12] = {0.0f, 0.0f,        0.0f, 1.0f,        0.0f, 0.17364818f,
                               0.0f, 0.98480775f, 0.0f, 0.38268343f, 0.0f, 0.92387953f};
static const float kHipsMove[9] = {0.0f, 1.0f, 0.0f, 0.0f, 0.95f, 0.02f, 0.0f, 1.0f, 0.0f};
static const float kHipsTilt[12] = {0.0f, 0.0f,        0.0f, 1.0f, 0.08715574f, 0.0f,
                                    0.0f, 0.99619470f, 0.0f, 0.0f, 0.0f,        1.0f};
static const float kSpine[12] = {0.17364818f, 0.0f, 0.0f, 0.98480775f,
                                 0.30070580f, 0.0f, 0.0f, 0.95371695f,
                                 0.17364818f, 0.0f, 0.0f, 0.98480775f};
static const float kArm[12] = {0.0f,        0.0f, 0.25881904f, 0.96592583f, 0.0f,       0.0f, 0.5f,
                               0.86602540f, 0.0f, 0.0f,        0.38268343f, 0.92387953f};
static const float kHand[9] = {0.5f, 1.0f, 1.0f, 0.6f, 1.1f, 1.0f, 0.5f, 1.0f, 1.0f};
static const float kTurnedMove[9] = {0.0f, 1.0f, 0.0f, 0.5f, 1.05f, 0.3f, 1.0f, 1.0f, 0.8f};
static const float kBlink[3] = {0.0f, 1.0f, 0.25f};
static const float kFlag[2] = {0.0f, 1.0f};
static const float kFootTimes[3] = {0.0f, 0.4f, 0.8f};
static const int32_t kFoot[3] = {1, 2, -3};
static const float kGripTimes[2] = {0.0f, 0.6f};
static const int32_t kGrip[2] = {5, 6};
static const float kArmMove[9] = {0.2f, 0.1f, 0.0f, 0.25f, 0.1f, 0.05f, 0.3f, 0.12f, 0.1f};
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
        // The source's hand rests scaled.
        sources[b].rest.scale.x = side == 0 && b == 6 ? 0.5f : 1.0f;
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
    Skeleton(kTurnedSource, 2, 4, &s_turnedSource);
    Skeleton(kTurnedTarget, 2, 5, &s_turnedTarget);
    maniRetargetMapSource source = {.source = &s_source,
                                    .target = &s_target,
                                    .chains = kChains,
                                    .pelvis = &kPelvis,
                                    .chainCount = 2,
                                    .matchNames = true};
    Map(&source, 0, &s_map);
    source.matchNames = false;
    Map(&source, 1, &s_chainsOnly);
    maniRetargetMapSource turned = {
        .source = &s_turnedSource, .target = &s_turnedTarget, .matchNames = true};
    Map(&turned, 2, &s_turnedMap);
    // No horizontal scale; and the target's forearm at a scale of 0.
    maniRetargetPelvisSource flat = kPelvis;
    flat.horizontal = 0.0f;
    source.matchNames = true;
    source.pelvis = &flat;
    Map(&source, 3, &s_flat);
    maniTransform reference[10];
    for (uint32_t b = 0; b < 10; ++b)
    {
        reference[b] = maniSkeleton_GetRest(&s_target, b);
    }
    reference[7].scale.x = 0.0f;
    source.pelvis = &kPelvis;
    source.targetReference = reference;
    Map(&source, 4, &s_flatHand);
    const maniSkeleton* skeletons[6] = {&s_source, &s_source, &s_target,
                                        &s_target, &s_target, &s_turnedSource};
    maniPose** poses[6] = {&s_in, &s_whole, &s_out, &s_base, &s_expected, &s_turnedIn};
    for (int i = 0; i < 6; ++i)
    {
        CHECK(maniInitPose(skeletons[i], s_poseMemory[i], sizeof s_poseMemory[i], poses[i]) ==
                  mani_success,
              "a pose");
    }
}

// Builds and opens a clip of a source rig in slot, with root motion on
// a bone (the root's twist about y, another's whole rotation) or none:
// the arm then moving, s2 then tracked.
static void SourceClip(const maniSkeleton* skeleton, maniAdditiveBasis additive, uint32_t root,
                       int slot, maniClip* out)
{
    const maniBoneTrackSource tracks[6] = {
        {0, Keys(kRootMove), Keys(kYaw), kNone},
        {1, Keys(kHipsMove), Keys(kHipsTilt), kNone},
        {2, kNone, Keys(kSpine), kNone},
        {5, root == 5 ? Keys(kArmMove) : kNone, Keys(kArm), kNone},
        {6, kNone, kNone, Keys(kHand)},
        {3, kNone, Keys(kSpine), kNone}};
    const maniFloatTrackSource floats[2] = {{"blink", 5, Keys(kBlink)},
                                            {"flag", 4, {kTimes, kFlag, 2, maniInterpolationStep}}};
    const maniDiscreteTrackSource discrete[2] = {{"foot", 4, 3, kFootTimes, kFoot},
                                                 {"grip", 4, 2, kGripTimes, kGrip}};
    const maniEventSource events[2] = {{0.25f, 7, "step", 4}, {0.75f, 9, "step", 4}};
    const maniEventSource markers[2] = {{0.0f, 0, "L", 1}, {0.5f, 0, "R", 1}};
    const maniRootMotionSource motion = {root,
                                         maniRootAxisX | maniRootAxisZ,
                                         root == 0 ? maniRootRotationTwist : maniRootRotationWhole,
                                         1,
                                         maniRootReferenceRest,
                                         maniRootReferenceRest,
                                         0};
    maniClipSource source = {.skeleton = skeleton,
                             .boneTracks = tracks,
                             .floatTracks = floats,
                             .discreteTracks = discrete,
                             .events = events,
                             .markers = markers,
                             .boneTrackCount = root == 3 ? 6 : 5,
                             .floatTrackCount = 2,
                             .discreteTrackCount = 2,
                             .eventCount = 2,
                             .markerCount = 2,
                             .loop = maniLoopRepeat,
                             .sampleRate = 30.0f,
                             .duration = 1.0f,
                             .additive = additive,
                             .rootMotion = root != MANI_NO_BONE ? &motion : NULL};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_clipFiles[slot], sizeof s_clipFiles[slot], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[slot], size, out).result == mani_success,
          "a source clip");
    maniDestroyBuilder(builder);
}

// Whether the source bakes to the size bytes at built whatever bytes pad
// it.
static bool PaddingIgnored(const maniRetargetClipSource* source, const uint8_t* built, size_t size)
{
    bool same = true;
    for (int k = 0; k < 2; ++k)
    {
        maniRetargetClipSource s = *source;
        TEST_PAD(&s, 1, kFieldsRetargetClipSource, kTestPadPatterns[k]);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        size_t padded = 0;
        uint8_t* bytes = maniBuildRetargetedClip(builder, &s).result == mani_success
                             ? TestCopyBuilt(builder, &padded)
                             : NULL;
        same = same && bytes != NULL && padded == size && memcmp(bytes, built, size) == 0;
        free(bytes);
        maniDestroyBuilder(builder);
    }
    return same;
}

static maniBuildResult Bake(const maniRetargetClipSource* source, maniClip* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniBuildResult r = maniBuildRetargetedClip(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniBuilder_CopyBuilt(builder, s_baked, sizeof s_baked, &size) == mani_success &&
                  maniOpenClip(s_baked, size, out).result == mani_success,
              "the baked clip opens");
        CHECK(PaddingIgnored(source, s_baked, size), "the same clip under either padding");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniRetargetClipSource Source(const maniRetargetMap* map, const maniClip* clip)
{
    return (maniRetargetClipSource){map, &s_source, &s_target, clip, MANI_NO_BONE};
}

// The normalized dot product's size of two rotations, in binary64.
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

// Every bone of two poses within tolerance.
static bool Same(const maniPose* a, const maniPose* b, uint32_t bones)
{
    bool same = true;
    for (uint32_t bone = 0; bone < bones; ++bone)
    {
        maniTransform x = maniPose_GetLocal(a, bone);
        maniTransform y = maniPose_GetLocal(b, bone);
        same = same && Gap(x.translation, y.translation) < 1e-5 &&
               QuatDot(x.rotation, y.rotation) > 1.0 - 1e-10 && Gap(x.scale, y.scale) < 1e-5;
    }
    return same;
}

// The baked clip's sample k against the transfer of the source's.
static bool SampleCarried(const maniRetargetMap* map, const maniClip* source, const maniClip* baked,
                          uint32_t k)
{
    float time = (float)k / 30.0f;
    bool ok = maniClip_Sample(source, &s_source, time, s_in) == mani_success &&
              maniRetargetMap_RetargetPose(map, &s_source, s_in, &s_target, s_expected, s_scratch,
                                           sizeof s_scratch) == mani_success &&
              maniClip_Sample(baked, &s_target, time, s_out) == mani_success;
    return ok && Same(s_out, s_expected, 10);
}

static void TestSamples(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, MANI_NO_BONE, 0, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip bakes");
    CHECK(maniClip_GetSampleCount(&baked) == 31 && maniClip_GetSampleRate(&baked) == 30.0f &&
              maniClip_GetDuration(&baked) == 1.0f &&
              maniClip_GetLoopPolicy(&baked) == maniLoopRepeat &&
              maniClip_GetAdditiveBasis(&baked) == maniAdditiveNone,
          "the timing kept");
    bool carried = true;
    for (uint32_t k = 0; k < 31; ++k)
    {
        carried = carried && SampleCarried(&s_map, &source, &baked, k);
    }
    CHECK(carried, "every sample the transfer of the source's");
}

static void TestNamed(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, MANI_NO_BONE, 0, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip bakes");
    bool floats = maniClip_GetFloatTrackCount(&baked) == 2;
    const float times[4] = {0.1f, 0.49f, 0.51f, 0.9f};
    for (int i = 0; floats && i < 4; ++i)
    {
        float a[2];
        float b[2];
        floats = maniClip_SampleFloats(&source, times[i], a, 2, NULL) == mani_success &&
                 maniClip_SampleFloats(&baked, times[i], b, 2, NULL) == mani_success &&
                 fabsf(a[0] - b[0]) < 1e-6f && a[1] == b[1];
    }
    CHECK(floats, "float tracks kept, the step one a step");
    bool discrete = maniClip_GetDiscreteTrackCount(&baked) == 2;
    for (int i = 0; discrete && i < 8; ++i)
    {
        discrete = maniClip_GetDiscreteValue(&baked, (uint32_t)i % 2, times[i / 2]) ==
                   maniClip_GetDiscreteValue(&source, (uint32_t)i % 2, times[i / 2]);
    }
    CHECK(discrete, "discrete tracks copied");
    bool timed = maniClip_GetEventCount(&baked) == 2 && maniClip_GetMarkerCount(&baked) == 2;
    for (uint32_t i = 0; timed && i < 2; ++i)
    {
        maniClipEvent a = maniClip_GetEvent(&source, i);
        maniClipEvent b = maniClip_GetEvent(&baked, i);
        maniClipEvent c = maniClip_GetMarker(&source, i);
        maniClipEvent d = maniClip_GetMarker(&baked, i);
        timed = a.time == b.time && a.payload == b.payload && a.nameHash == b.nameHash &&
                c.time == d.time && c.nameHash == d.nameHash;
    }
    CHECK(timed, "events and markers copied");
}

// Bones no part of the map takes stay at their rest and are left out.
static void TestLeftOut(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, MANI_NO_BONE, 0, &source);
    maniRetargetClipSource s = Source(&s_chainsOnly, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip bakes");
    // root and head unmapped, the hand past the one-to-one arm chain's
    // source: the other seven bones move.
    CHECK(baked.trackCount == 7, "channels at the rest left out");
    bool carried = true;
    for (uint32_t k = 0; k < 31; k += 5)
    {
        carried = carried && SampleCarried(&s_chainsOnly, &source, &baked, k);
    }
    CHECK(carried, "the samples carried");
}

static void TestCompressed(void)
{
    static _Alignas(16) uint8_t compressed[65536];
    maniClip source;
    maniClip small;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, MANI_NO_BONE, 0, &source);
    maniBuilder* builder = NewBuilder();
    maniCompressOptions options = maniDefaultCompressOptions();
    size_t size = 0;
    CHECK(maniCompressClip(builder, &source, &s_source, &options, NULL) == mani_success &&
              maniBuilder_CopyBuilt(builder, compressed, sizeof compressed, &size) ==
                  mani_success &&
              maniOpenClip(compressed, size, &small).result == mani_success,
          "a compressed source");
    maniDestroyBuilder(builder);
    maniRetargetClipSource s = Source(&s_map, &small);
    CHECK(Bake(&s, &baked).result == mani_success, "a compressed clip bakes");
    bool carried = true;
    for (uint32_t k = 0; k < 31; ++k)
    {
        carried = carried && SampleCarried(&s_map, &small, &baked, k);
    }
    CHECK(carried, "every sample the transfer of the compressed source's");
}

// An additive clip's sample k against the difference, from the source's
// rest carried, of its difference added to the source's rest carried.
static bool DifferenceCarried(const maniClip* source, const maniClip* baked, uint32_t k)
{
    float time = (float)k / 30.0f;
    maniBlendLayer layer = {s_in, 1.0f, NULL};
    bool ok = maniPose_SetRest(s_whole, &s_source) == mani_success &&
              maniRetargetMap_RetargetPose(&s_map, &s_source, s_whole, &s_target, s_base, s_scratch,
                                           sizeof s_scratch) == mani_success &&
              maniClip_Sample(source, &s_source, time, s_in) == mani_success &&
              maniPose_AddLayers(s_whole, &s_source, &layer, 1) == mani_success &&
              maniRetargetMap_RetargetPose(&s_map, &s_source, s_whole, &s_target, s_expected,
                                           s_scratch, sizeof s_scratch) == mani_success &&
              maniSkeleton_ComputePoseDifference(&s_target, s_expected, s_base, s_expected) ==
                  mani_success &&
              maniClip_Sample(baked, &s_target, time, s_out) == mani_success;
    return ok && maniPose_IsDifference(s_out) && Same(s_out, s_expected, 10);
}

static void TestAdditive(void)
{
    const maniAdditiveBasis bases[2] = {maniAdditiveRest, maniAdditiveFirstFrame};
    for (int i = 0; i < 2; ++i)
    {
        maniClip source;
        maniClip baked;
        SourceClip(&s_source, bases[i], MANI_NO_BONE, 1, &source);
        maniRetargetClipSource s = Source(&s_map, &source);
        CHECK(Bake(&s, &baked).result == mani_success, "an additive clip bakes");
        CHECK(maniClip_GetAdditiveBasis(&baked) == bases[i], "the basis kept");
        bool carried = true;
        for (uint32_t k = 0; k < 31; ++k)
        {
            carried = carried && DifferenceCarried(&source, &baked, k);
        }
        CHECK(carried, "every difference carried");
    }
}

// Moves of the source and the baked clip over some spans: the baked's
// translation the source's scaled, its rotation the source's.
static void TestRootMotion(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, 0, 0, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip with root motion bakes");
    maniRootMotionSource declared;
    CHECK(maniClip_GetRootMotionSource(&baked, &declared) && declared.bone == 0 &&
              declared.translationAxes == (maniRootAxisX | maniRootAxisZ) &&
              declared.rotation == maniRootRotationTwist && declared.twistAxis == 1 &&
              declared.translationReference == maniRootReferenceRest && declared.loop == 0,
          "the root motion declared on the root's follower");
    const float spans[3][2] = {{0.0f, 1.0f}, {0.2f, 0.5f}, {0.7f, 0.25f}};
    bool moves = true;
    for (int i = 0; i < 3; ++i)
    {
        maniRootMove a;
        maniRootMove b;
        moves =
            moves &&
            maniClip_GetRootMotion(&source, spans[i][0], spans[i][1], &a) == mani_success &&
            maniClip_GetRootMotion(&baked, spans[i][0], spans[i][1], &b) == mani_success &&
            Gap(b.translation, maniRetargetMap_RetargetRootMotion(&s_map, a.translation)) < 1e-5 &&
            QuatDot(a.rotation, b.rotation) > 1.0 - 1e-10;
    }
    CHECK(moves, "the moves scaled");
    bool carried = true;
    for (uint32_t k = 0; k < 31; k += 3)
    {
        carried = carried && SampleCarried(&s_map, &source, &baked, k);
    }
    CHECK(carried, "the poses in place carried");
    // A bone whose parent's frame turns off the axes takes all of them.
    s.rootMotionBone = 7;
    CHECK(Bake(&s, &baked).result == mani_success &&
              maniClip_GetRootMotionSource(&baked, &declared) && declared.bone == 7 &&
              declared.translationAxes == 7 && declared.rotation == maniRootRotationWhole,
          "all axes and the whole rotation off the axes");
}

static maniQuat Mul(maniQuat a, maniQuat b)
{
    return (maniQuat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// The hips' motion into a target whose root turns 90 degrees about x:
// x stays, z becomes y, the twist about y one about z.
static void TestTurned(void)
{
    const maniBoneTrackSource tracks[1] = {{1, Keys(kTurnedMove), Keys(kYaw), kNone}};
    const maniRootMotionSource root = {1, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
                                       1, maniRootReferenceRest,         maniRootReferenceRest,
                                       0};
    maniClipSource clip = {.skeleton = &s_turnedSource,
                           .boneTracks = tracks,
                           .boneTrackCount = 1,
                           .loop = maniLoopClamp,
                           .sampleRate = 30.0f,
                           .rootMotion = &root};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniClip source;
    maniClip baked;
    CHECK(maniBuildClip(builder, &clip).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_clipFiles[2], sizeof s_clipFiles[2], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[2], size, &source).result == mani_success,
          "a turned source clip");
    maniDestroyBuilder(builder);
    maniRetargetClipSource s = {&s_turnedMap, &s_turnedSource, &s_turnedTarget, &source,
                                MANI_NO_BONE};
    maniRootMotionSource declared;
    CHECK(Bake(&s, &baked).result == mani_success &&
              maniClip_GetRootMotionSource(&baked, &declared) && declared.bone == 1 &&
              declared.translationAxes == (maniRootAxisX | maniRootAxisY) &&
              declared.rotation == maniRootRotationTwist && declared.twistAxis == 2,
          "the axes turned");
    // The turn from the source parent's frame to the target's.
    const maniQuat g = {-0.70710678f, 0.0f, 0.0f, 0.70710678f};
    const maniQuat back = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
    maniRootMove a;
    maniRootMove b;
    CHECK(maniClip_GetRootMotion(&source, 0.1f, 0.6f, &a) == mani_success &&
              maniClip_GetRootMotion(&baked, 0.1f, 0.6f, &b) == mani_success,
          "moves read");
    maniQuat t =
        Mul(Mul(g, (maniQuat){a.translation.x, a.translation.y, a.translation.z, 0.0f}), back);
    CHECK(Gap(b.translation, (maniVec3){t.x, t.y, t.z}) < 1e-5 &&
              QuatDot(b.rotation, Mul(Mul(g, a.rotation), back)) > 1.0 - 1e-10,
          "the moves turned");
}

static maniQuat Conj(maniQuat q)
{
    return (maniQuat){-q.x, -q.y, -q.z, q.w};
}

static maniVec3 Turn(maniQuat q, maniVec3 v)
{
    maniQuat p = Mul(Mul(q, (maniQuat){v.x, v.y, v.z, 0.0f}), Conj(q));
    return (maniVec3){p.x, p.y, p.z};
}

// A bone's rest model rotation: its root path's rest rotations.
static maniQuat RestModel(const maniSkeleton* skeleton, uint32_t bone)
{
    maniQuat q = {0.0f, 0.0f, 0.0f, 1.0f};
    for (uint32_t b = bone; b != MANI_NO_BONE; b = maniSkeleton_GetParent(skeleton, b))
    {
        q = Mul(maniSkeleton_GetRest(skeleton, b).rotation, q);
    }
    return q;
}

// The arm's motion (from its rest, so the move from the start is the
// sample's) into the upper arm, under chests whose rests turn apart:
// into the model by the source chest's frame, scaled, out by the
// target's; the rotation turned between the frames.
static void TestTurnedSource(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, 5, 3, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    maniRootMotionSource declared;
    CHECK(Bake(&s, &baked).result == mani_success &&
              maniClip_GetRootMotionSource(&baked, &declared) && declared.bone == 6 &&
              declared.translationAxes == 7 && declared.rotation == maniRootRotationWhole,
          "the arm's motion on the upper arm, off the axes");
    maniQuat from = RestModel(&s_source, 4);
    maniQuat to = RestModel(&s_target, 5);
    maniQuat g = Mul(Conj(to), from);
    bool moves = true;
    const float ends[2] = {0.5f, 1.0f};
    for (int i = 0; i < 2; ++i)
    {
        maniRootMove a;
        maniRootMove b;
        moves = moves && maniClip_GetRootMotion(&source, 0.0f, ends[i], &a) == mani_success &&
                maniClip_GetRootMotion(&baked, 0.0f, ends[i], &b) == mani_success &&
                Gap(b.translation, Turn(Conj(to), maniRetargetMap_RetargetRootMotion(
                                                      &s_map, Turn(from, a.translation)))) < 1e-5 &&
                QuatDot(b.rotation, Mul(Mul(g, a.rotation), Conj(g))) > 1.0 - 1e-10;
    }
    CHECK(moves, "the moves carried through both chests' frames");
}

// No horizontal scale: the walk's axes still x and z, though its
// horizontal moves are scaled to nothing.
static void TestFlat(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, 0, 0, &source);
    maniRetargetClipSource s = Source(&s_flat, &source);
    maniRootMotionSource declared;
    maniRootMove move;
    CHECK(Bake(&s, &baked).result == mani_success &&
              maniClip_GetRootMotionSource(&baked, &declared) &&
              declared.translationAxes == (maniRootAxisX | maniRootAxisZ) &&
              maniClip_GetRootMotion(&baked, 0.0f, 1.0f, &move) == mani_success &&
              move.translation.x == 0.0f && move.translation.z == 0.0f,
          "the axes kept where the scale takes them to nothing");
}

static maniBuildResult Refused(maniRetargetClipSource s)
{
    maniClip baked;
    return Bake(&s, &baked);
}

static void TestRefusals(void)
{
    maniClip source;
    maniClip moving;
    maniClip baked;
    SourceClip(&s_source, maniAdditiveNone, MANI_NO_BONE, 0, &source);
    maniBuilder* builder = NewBuilder();
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(maniBuildRetargetedClip(NULL, &s).result == mani_errorInvalid, "a NULL builder");
    CHECK(maniBuildRetargetedClip(builder, NULL).result == mani_errorInvalid, "a NULL source");
    s = Source(NULL, &source);
    CHECK(maniBuildRetargetedClip(builder, &s).result == mani_errorInvalid, "a NULL map");
    CHECK(maniBuilder_GetMisuse(builder) == 2, "misuses counted");
    maniDestroyBuilder(builder);
    s = Source(&s_map, &source);
    s.source = &s_otherSource;
    maniBuildResult r = Refused(s);
    CHECK(r.reason == maniBuildReasonRetargetClip && r.other == 0, "another source");
    s = Source(&s_map, &source);
    s.target = &s_otherTarget;
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRetargetClip && r.other == 1, "another target");
    s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip bakes");
    static _Alignas(16) uint8_t copy[65536];
    memcpy(copy, s_baked, sizeof copy);
    CHECK(maniOpenClip(copy, sizeof copy, &baked).result == mani_success, "its copy opens");
    s = Source(&s_map, &baked);
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRetargetClip && r.other == 2, "a clip of the target");
    s = Source(&s_map, &source);
    s.rootMotionBone = 10;
    CHECK(Refused(s).result == mani_success, "a bone for no root motion ignored");
    SourceClip(&s_source, maniAdditiveNone, 0, 1, &moving);
    s = Source(&s_map, &moving);
    s.rootMotionBone = 10;
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRootMotion && r.item == 10, "a root bone past the target");
    s = Source(&s_chainsOnly, &moving);
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRootMotion && r.item == MANI_NO_BONE,
          "no bone follows the root motion bone");
    // s2 is the second source of two target spine bones, alone of none.
    SourceClip(&s_source, maniAdditiveNone, 3, 1, &moving);
    s = Source(&s_map, &moving);
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRootMotion && r.item == MANI_NO_BONE,
          "no bone follows the root motion bone alone");
    SourceClip(&s_otherSource, maniAdditiveNone, MANI_NO_BONE, 1, &moving);
    s = Source(&s_map, &moving);
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRetargetClip && r.other == 2, "a clip of another source");
    SourceClip(&s_source, maniAdditiveRest, MANI_NO_BONE, 1, &moving);
    s = Source(&s_flatHand, &moving);
    r = Refused(s);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonBasis,
          "an additive clip over a basis of scale 0");
}

int main(void)
{
    Setup();
    TestSamples();
    TestNamed();
    TestLeftOut();
    TestCompressed();
    TestAdditive();
    TestRootMotion();
    TestTurned();
    TestTurnedSource();
    TestFlat();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
