// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking clips through a retargeting map (mani-0032): every sample of the
// baked clip the pose transfer of the source's (from a full-precision and
// a compressed source); the timing, float tracks (a step one stays a
// step), discrete tracks, events and markers kept; channels at the
// target's rest left out; additive clips (rest and first-frame bases)
// as differences carried from the source's rest; root motion's moves
// scaled, and turned with their axes into a turned parent's frame (all
// axes and the whole rotation where the turn leaves the axes); the
// refusals; the baked clip's bits pinned, the same on every platform.

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

#define PINNED_IDENTITY 0x981eca0313382b7aull

static _Alignas(16) uint8_t s_skeletonFiles[6][4096];
static _Alignas(16) uint8_t s_mapFiles[3][16384];
static _Alignas(16) uint8_t s_clipFiles[3][65536];
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
    maniSkeletonSource source = {sources, count, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFiles[side], sizeof s_skeletonFiles[side], &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFiles[side], size, out).result == mani_success,
          "a skeleton");
    maniDestroyBuilder(builder);
}

static void Map(const maniRetargetMapSource* source, int slot, maniRetargetMap* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_mapFiles[slot], sizeof s_mapFiles[slot], &size) ==
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

// Builds and opens a clip of the main source in slot, with root motion
// on the root bone or not.
static void SourceClip(maniAdditiveBasis additive, bool moving, int slot, maniClip* out)
{
    const maniBoneTrackSource tracks[5] = {{0, 0, Keys(kRootMove), Keys(kYaw), kNone},
                                           {1, 0, Keys(kHipsMove), Keys(kHipsTilt), kNone},
                                           {2, 0, kNone, Keys(kSpine), kNone},
                                           {5, 0, kNone, Keys(kArm), kNone},
                                           {6, 0, kNone, kNone, Keys(kHand)}};
    const maniFloatTrackSource floats[2] = {
        {"blink", 5, 0, Keys(kBlink)}, {"flag", 4, 0, {kTimes, kFlag, 2, maniInterpolationStep}}};
    const maniDiscreteTrackSource discrete[1] = {{"foot", 4, 3, kFootTimes, kFoot}};
    const maniEventSource events[2] = {{0.25f, 7, "step", 4, 0}, {0.75f, 9, "step", 4, 0}};
    const maniEventSource markers[2] = {{0.0f, 0, "L", 1, 0}, {0.5f, 0, "R", 1, 0}};
    const maniRootMotionSource root = {0,
                                       maniRootAxisX | maniRootAxisZ,
                                       maniRootRotationTwist,
                                       1,
                                       maniRootReferenceRest,
                                       maniRootReferenceRest,
                                       0,
                                       0};
    maniClipSource source = {&s_source,
                             tracks,
                             floats,
                             discrete,
                             events,
                             markers,
                             5,
                             2,
                             1,
                             2,
                             2,
                             maniLoopRepeat,
                             30.0f,
                             1.0f,
                             additive,
                             0,
                             moving ? &root : NULL};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildClip(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_clipFiles[slot], sizeof s_clipFiles[slot], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[slot], size, out).result == mani_success,
          "a source clip");
    maniDestroyBuilder(builder);
}

static maniBuildResult Bake(const maniRetargetClipSource* source, maniClip* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniBuildResult r = maniBuildRetargetedClip(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_baked, sizeof s_baked, &size) == mani_success &&
                  maniOpenClip(s_baked, size, out).result == mani_success,
              "the baked clip opens");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniRetargetClipSource Source(const maniRetargetMap* map, const maniClip* clip)
{
    return (maniRetargetClipSource){map, &s_source, &s_target, clip, MANI_NO_BONE, 0};
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
        maniTransform x = maniPoseLocal(a, bone);
        maniTransform y = maniPoseLocal(b, bone);
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
    bool ok = maniSampleClip(source, &s_source, time, s_in) == mani_success &&
              maniRetargetPose(map, &s_source, s_in, &s_target, s_expected, s_scratch,
                               sizeof s_scratch) == mani_success &&
              maniSampleClip(baked, &s_target, time, s_out) == mani_success;
    return ok && Same(s_out, s_expected, 10);
}

static void TestSamples(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(maniAdditiveNone, false, 0, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip bakes");
    CHECK(maniClipSampleCount(&baked) == 31 && maniClipSampleRate(&baked) == 30.0f &&
              maniClipDuration(&baked) == 1.0f && maniClipLoopPolicy(&baked) == maniLoopRepeat &&
              maniClipAdditiveBasis(&baked) == maniAdditiveNone,
          "the timing kept");
    bool carried = true;
    for (uint32_t k = 0; k < 31; ++k)
    {
        carried = carried && SampleCarried(&s_map, &source, &baked, k);
    }
    CHECK(carried, "every sample the transfer of the source's");
    uint64_t identity = maniClipIdentity(&baked);
    CHECK(identity == PINNED_IDENTITY, "the baked clip's bits the same on every platform");
    if (identity != PINNED_IDENTITY)
    {
        printf("baked identity 0x%016llx\n", (unsigned long long)identity);
    }
}

static void TestNamed(void)
{
    maniClip source;
    maniClip baked;
    SourceClip(maniAdditiveNone, false, 0, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip bakes");
    bool floats = maniClipFloatTrackCount(&baked) == 2;
    const float times[4] = {0.1f, 0.49f, 0.51f, 0.9f};
    for (int i = 0; floats && i < 4; ++i)
    {
        float a[2];
        float b[2];
        floats = maniSampleClipFloats(&source, times[i], a, 2, NULL) == mani_success &&
                 maniSampleClipFloats(&baked, times[i], b, 2, NULL) == mani_success &&
                 fabsf(a[0] - b[0]) < 1e-6f && a[1] == b[1];
    }
    CHECK(floats, "float tracks kept, the step one a step");
    bool discrete = maniClipDiscreteTrackCount(&baked) == 1;
    for (int i = 0; discrete && i < 4; ++i)
    {
        discrete = maniClipDiscreteValue(&baked, 0, times[i]) ==
                   maniClipDiscreteValue(&source, 0, times[i]);
    }
    CHECK(discrete, "discrete tracks copied");
    bool timed = maniClipEventCount(&baked) == 2 && maniClipMarkerCount(&baked) == 2;
    for (uint32_t i = 0; timed && i < 2; ++i)
    {
        maniClipEvent a = maniClipEventAt(&source, i);
        maniClipEvent b = maniClipEventAt(&baked, i);
        maniClipEvent c = maniClipMarkerAt(&source, i);
        maniClipEvent d = maniClipMarkerAt(&baked, i);
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
    SourceClip(maniAdditiveNone, false, 0, &source);
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
    SourceClip(maniAdditiveNone, false, 0, &source);
    maniBuilder* builder = NewBuilder();
    maniCompressOptions options = maniDefaultCompressOptions();
    size_t size = 0;
    CHECK(maniCompressClip(builder, &source, &s_source, &options, NULL) == mani_success &&
              maniCopyBuilt(builder, compressed, sizeof compressed, &size) == mani_success &&
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
    bool ok = maniPoseSetRest(s_whole, &s_source) == mani_success &&
              maniRetargetPose(&s_map, &s_source, s_whole, &s_target, s_base, s_scratch,
                               sizeof s_scratch) == mani_success &&
              maniSampleClip(source, &s_source, time, s_in) == mani_success &&
              maniAddPoses(&s_source, s_whole, &layer, 1) == mani_success &&
              maniRetargetPose(&s_map, &s_source, s_whole, &s_target, s_expected, s_scratch,
                               sizeof s_scratch) == mani_success &&
              maniPoseDifference(&s_target, s_expected, s_base, s_expected) == mani_success &&
              maniSampleClip(baked, &s_target, time, s_out) == mani_success;
    return ok && maniPoseIsDifference(s_out) && Same(s_out, s_expected, 10);
}

static void TestAdditive(void)
{
    const maniAdditiveBasis bases[2] = {maniAdditiveRest, maniAdditiveFirstFrame};
    for (int i = 0; i < 2; ++i)
    {
        maniClip source;
        maniClip baked;
        SourceClip(bases[i], false, 1, &source);
        maniRetargetClipSource s = Source(&s_map, &source);
        CHECK(Bake(&s, &baked).result == mani_success, "an additive clip bakes");
        CHECK(maniClipAdditiveBasis(&baked) == bases[i], "the basis kept");
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
    SourceClip(maniAdditiveNone, true, 0, &source);
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(Bake(&s, &baked).result == mani_success, "a clip with root motion bakes");
    maniRootMotionSource declared;
    CHECK(maniClipRootMotionSource(&baked, &declared) && declared.bone == 0 &&
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
        moves = moves &&
                maniClipRootMotion(&source, spans[i][0], spans[i][1], &a) == mani_success &&
                maniClipRootMotion(&baked, spans[i][0], spans[i][1], &b) == mani_success &&
                Gap(b.translation, maniRetargetRootMotion(&s_map, a.translation)) < 1e-5 &&
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
    CHECK(Bake(&s, &baked).result == mani_success && maniClipRootMotionSource(&baked, &declared) &&
              declared.bone == 7 && declared.translationAxes == 7 &&
              declared.rotation == maniRootRotationWhole,
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
    const maniBoneTrackSource tracks[1] = {{1, 0, Keys(kTurnedMove), Keys(kYaw), kNone}};
    const maniRootMotionSource root = {1,
                                       maniRootAxisX | maniRootAxisZ,
                                       maniRootRotationTwist,
                                       1,
                                       maniRootReferenceRest,
                                       maniRootReferenceRest,
                                       0,
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
              maniCopyBuilt(builder, s_clipFiles[2], sizeof s_clipFiles[2], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[2], size, &source).result == mani_success,
          "a turned source clip");
    maniDestroyBuilder(builder);
    maniRetargetClipSource s = {&s_turnedMap, &s_turnedSource, &s_turnedTarget,
                                &source,      MANI_NO_BONE,    0};
    maniRootMotionSource declared;
    CHECK(Bake(&s, &baked).result == mani_success && maniClipRootMotionSource(&baked, &declared) &&
              declared.bone == 1 && declared.translationAxes == (maniRootAxisX | maniRootAxisY) &&
              declared.rotation == maniRootRotationTwist && declared.twistAxis == 2,
          "the axes turned");
    // The turn from the source parent's frame to the target's.
    const maniQuat g = {-0.70710678f, 0.0f, 0.0f, 0.70710678f};
    const maniQuat back = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
    maniRootMove a;
    maniRootMove b;
    CHECK(maniClipRootMotion(&source, 0.1f, 0.6f, &a) == mani_success &&
              maniClipRootMotion(&baked, 0.1f, 0.6f, &b) == mani_success,
          "moves read");
    maniQuat t =
        Mul(Mul(g, (maniQuat){a.translation.x, a.translation.y, a.translation.z, 0.0f}), back);
    CHECK(Gap(b.translation, (maniVec3){t.x, t.y, t.z}) < 1e-5 &&
              QuatDot(b.rotation, Mul(Mul(g, a.rotation), back)) > 1.0 - 1e-10,
          "the moves turned");
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
    SourceClip(maniAdditiveNone, false, 0, &source);
    maniBuilder* builder = NewBuilder();
    maniRetargetClipSource s = Source(&s_map, &source);
    CHECK(maniBuildRetargetedClip(NULL, &s).result == mani_errorInvalid, "a NULL builder");
    CHECK(maniBuildRetargetedClip(builder, NULL).result == mani_errorInvalid, "a NULL source");
    s.reserved = 1;
    CHECK(maniBuildRetargetedClip(builder, &s).result == mani_errorInvalid, "reserved not 0");
    s = Source(NULL, &source);
    CHECK(maniBuildRetargetedClip(builder, &s).result == mani_errorInvalid, "a NULL map");
    CHECK(maniBuilderMisuses(builder) == 3, "misuses counted");
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
    SourceClip(maniAdditiveNone, true, 1, &moving);
    s = Source(&s_map, &moving);
    s.rootMotionBone = 10;
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRootMotion && r.item == 10, "a root bone past the target");
    s = Source(&s_chainsOnly, &moving);
    r = Refused(s);
    CHECK(r.reason == maniBuildReasonRootMotion && r.item == MANI_NO_BONE,
          "no bone follows the root motion bone");
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
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
