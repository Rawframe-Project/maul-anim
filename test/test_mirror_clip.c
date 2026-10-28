// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mirrored clips on a rig whose left and right clavicles,
// arms and hands have unmirrored axes: every sample of the baked clip
// the mirror of the clip's, its bits pinned, baking it again giving the
// clip back; float tracks, discrete tracks, events and markers renamed
// by the map's name pairs, their values kept; an additive clip's
// differences mirrored from the rest mirrored; root motion on the root
// reflected, on the left arm moved to the right arm with its model
// change reflected and its axes carried; the refusals.

#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
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
    kSpine,
    kChest,
    kHead,
    kClavL,
    kArmL,
    kHandL,
    kClavR,
    kArmR,
    kHandR,
    kPropR,
    kLegL,
    kLegR,
    kBones
};

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

static const maniBoneSource kRig[kBones] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"hips", 4, kRoot, {{0, 1, 0}, REST, {1, 1, 1}}},
    {"spine", 5, kHips, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
    {"chest", 5, kSpine, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
    {"head", 4, kChest, {{0, 0.3f, 0}, REST, {1, 1, 1}}},
    {"clavL", 5, kChest, {{0.1f, 0.1f, 0}, {0, 0, 0.25881904f, 0.96592583f}, {1, 1, 1}}},
    {"armL", 4, kClavL, {{0.2f, 0, 0}, REST, {1, 1, 1}}},
    {"handL", 5, kArmL, {{0.3f, 0, 0}, {0.5f, 0.5f, 0.5f, 0.5f}, {1, 1, 1}}},
    {"clavR", 5, kChest, {{-0.1f, 0.1f, 0}, {0, 0.70710678f, 0, 0.70710678f}, {1, 1, 1}}},
    {"armR", 4, kClavR, {{0, 0, 0.2f}, REST, {1, 1, 1}}},
    {"handR", 5, kArmR, {{0, 0, 0.3f}, {0.70710678f, 0, 0, 0.70710678f}, {1, 1, 1}}},
    {"propR", 5, kHandR, {{0, 0, 0.1f}, REST, {1, 1, 1}}},
    {"legL", 4, kHips, {{0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
    {"legR", 4, kHips, {{-0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
};

static const maniMirrorRuleSource kRules[2] = {{"L", 1, maniMirrorSuffix, "R", 1},
                                               {"R", 1, maniMirrorSuffix, "L", 1}};
// The names the clip uses, paired by the rules.
// A pair of names of different lengths.
static const maniMirrorNamePairSource kNamePairs[1] = {{"jump", 4, 7, "hopOver"}};
static const maniMirrorNameSource kNames[5] = {
    {"blinkL", 6}, {"footL", 5}, {"stepL", 5}, {"L", 1}, {"R", 1}};

static const float kTimes[3] = {0.0f, 0.5f, 1.0f};
static const float kRootMove[9] = {0.0f, 0.0f, 0.0f, 0.3f, 0.0f, 0.6f, 0.9f, 0.05f, 1.2f};
static const float kYaw[12] = {0.0f, 0.0f,        0.0f, 1.0f,        0.0f, 0.17364818f,
                               0.0f, 0.98480775f, 0.0f, 0.34202015f, 0.0f, 0.93969262f};
static const float kTilt[12] = {0.0f, 0.0f,       0.0f, 1.0f, 0.08715574f, 0.0f,
                                0.0f, 0.9961947f, 0.0f, 0.0f, 0.08715574f, 0.9961947f};
static const float kArm[12] = {0.0f, 0.0f,        0.25881904f, 0.96592583f, 0.1f, 0.2f,
                               0.3f, 0.92736185f, 0.0f,        0.5f,        0.0f, 0.8660254f};
static const float kArmMove[9] = {0.2f, 0.0f, 0.0f, 0.25f, 0.1f, 0.05f, 0.3f, 0.12f, 0.1f};
static const float kHand[9] = {1.0f, 1.0f, 1.0f, 1.5f, 1.5f, 1.5f, 2.0f, 2.0f, 2.0f};
static const float kBlink[3] = {0.0f, 1.0f, 0.25f};
static const float kFlag[2] = {0.0f, 1.0f};
static const float kFootTimes[3] = {0.0f, 0.4f, 0.8f};
static const int32_t kFoot[3] = {1, 2, -3};
static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};

static _Alignas(16) uint8_t s_skeletonFiles[3][4096];
static _Alignas(16) uint8_t s_mapFile[4096];
static _Alignas(16) uint8_t s_clipFile[65536];
static _Alignas(16) uint8_t s_baked[2][65536];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[5][2048];
static maniSkeleton s_skeleton;
static maniSkeleton s_other;
static maniSkeleton s_moved;
static maniMirrorMap s_map;
static maniPose* s_in;
static maniPose* s_out;
static maniPose* s_expected;
static maniPose* s_rest;
static maniPose* s_base;

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
    maniSkeletonSource source = {bones, count};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, 4096, &size) == mani_success &&
              maniOpenSkeleton(file, size, out).result == mani_success,
          "a rig");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    OpenSkeleton(kRig, kBones, s_skeletonFiles[0], &s_skeleton);
    OpenSkeleton(kRig, kBones - 1, s_skeletonFiles[1], &s_other);
    maniBoneSource moved[kBones];
    memcpy(moved, kRig, sizeof moved);
    moved[kHead].rest.translation.y = 0.35f;
    OpenSkeleton(moved, kBones, s_skeletonFiles[2], &s_moved);
    maniMirrorMapSource source = {.skeleton = &s_skeleton,
                                  .rules = kRules,
                                  .namePairs = kNamePairs,
                                  .names = kNames,
                                  .ruleCount = 2,
                                  .namePairCount = 1,
                                  .nameCount = 5,
                                  .axis = maniMirrorAxisX};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_mapFile, sizeof s_mapFile, &size) == mani_success &&
              maniOpenMirrorMap(s_mapFile, size, &s_map).result == mani_success,
          "the map");
    maniDestroyBuilder(builder);
    maniPose** poses[5] = {&s_in, &s_out, &s_expected, &s_rest, &s_base};
    for (uint32_t i = 0; i < 5; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], poses[i]) ==
                  mani_success,
              "a pose");
    }
}

// The clip: the root and hips moving and turning, the left clavicle and
// arm turning, the right hand scaling; root motion on a bone (its twist
// about y) or none.
static void Clip(maniAdditiveBasis additive, uint32_t root, maniClip* out)
{
    const maniBoneTrackSource tracks[6] = {
        {kRoot, Keys(kRootMove), Keys(kYaw), kNone},
        {kHips, kNone, Keys(kTilt), kNone},
        {kClavL, kNone, Keys(kArm), kNone},
        {kArmL, root == kArmL ? Keys(kArmMove) : kNone, Keys(kArm), kNone},
        {kHandR, kNone, kNone, Keys(kHand)},
        {kLegL, kNone, Keys(kTilt), kNone}};
    const maniFloatTrackSource floats[2] = {{"blinkL", 6, Keys(kBlink)},
                                            {"flag", 4, {kTimes, kFlag, 2, maniInterpolationStep}}};
    const maniDiscreteTrackSource discrete[1] = {{"footL", 5, 3, kFootTimes, kFoot}};
    const maniEventSource events[2] = {{0.25f, 7, "stepL", 5}, {0.75f, 9, "jump", 4}};
    const maniEventSource markers[2] = {{0.0f, 0, "L", 1}, {0.5f, 0, "R", 1}};
    const maniRootMotionSource motion = {root, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
                                         1,    maniRootReferenceRest,         maniRootReferenceRest,
                                         0};
    maniClipSource source = {.skeleton = &s_skeleton,
                             .boneTracks = tracks,
                             .floatTracks = floats,
                             .discreteTracks = discrete,
                             .events = events,
                             .markers = markers,
                             .boneTrackCount = 6,
                             .floatTrackCount = 2,
                             .discreteTrackCount = 1,
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
              maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
              maniOpenClip(s_clipFile, size, out).result == mani_success,
          "a clip");
    maniDestroyBuilder(builder);
}

// Whether the source bakes to the size bytes at built whatever bytes pad
// it.
static bool PaddingIgnored(const maniMirrorClipSource* source, const uint8_t* built, size_t size)
{
    bool same = true;
    for (int k = 0; k < 2; ++k)
    {
        maniMirrorClipSource s = *source;
        TEST_PAD(&s, 1, kFieldsMirrorClipSource, kTestPadPatterns[k]);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        size_t padded = 0;
        uint8_t* bytes = maniBuildMirroredClip(builder, &s).result == mani_success
                             ? TestCopyBuilt(builder, &padded)
                             : NULL;
        same = same && bytes != NULL && padded == size && memcmp(bytes, built, size) == 0;
        free(bytes);
        maniDestroyBuilder(builder);
    }
    return same;
}

static maniBuildResult Bake(const maniMirrorClipSource* source, int slot, maniClip* out)
{
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniBuildResult r = maniBuildMirroredClip(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_baked[slot], sizeof s_baked[slot], &size) == mani_success &&
                  maniOpenClip(s_baked[slot], size, out).result == mani_success,
              "the baked clip opens");
        CHECK(PaddingIgnored(source, s_baked[slot], size), "the same clip under either padding");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniMirrorClipSource Source(const maniClip* clip)
{
    return (maniMirrorClipSource){&s_map, &s_skeleton, clip};
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

// The baked clip's sample k against the mirror of the clip's.
static bool SampleMirrored(const maniClip* clip, const maniClip* baked, uint32_t k)
{
    float time = (float)k / 30.0f;
    return maniSampleClip(clip, &s_skeleton, time, s_in) == mani_success &&
           maniMirrorPose(&s_map, &s_skeleton, s_in, s_expected) == mani_success &&
           maniSampleClip(baked, &s_skeleton, time, s_out) == mani_success &&
           Same(s_out, s_expected);
}

// Two clips' samples the same.
static bool SampleSame(const maniClip* a, const maniClip* b, uint32_t k)
{
    float time = (float)k / 30.0f;
    return maniSampleClip(a, &s_skeleton, time, s_in) == mani_success &&
           maniSampleClip(b, &s_skeleton, time, s_out) == mani_success && Same(s_out, s_in);
}

static void TestSamples(void)
{
    maniClip clip;
    maniClip baked;
    maniClip back;
    Clip(maniAdditiveNone, MANI_NO_BONE, &clip);
    maniMirrorClipSource s = Source(&clip);
    CHECK(Bake(&s, 0, &baked).result == mani_success, "a clip bakes");
    CHECK(maniClipSampleCount(&baked) == 31 && maniClipSampleRate(&baked) == 30.0f &&
              maniClipDuration(&baked) == 1.0f && maniClipLoopPolicy(&baked) == maniLoopRepeat &&
              maniClipAdditiveBasis(&baked) == maniAdditiveNone,
          "the timing kept");
    s = Source(&baked);
    CHECK(Bake(&s, 1, &back).result == mani_success, "the mirrored clip bakes");
    bool mirrored = true;
    bool same = true;
    for (uint32_t k = 0; k < 31; ++k)
    {
        mirrored = mirrored && SampleMirrored(&clip, &baked, k);
        same = same && SampleSame(&clip, &back, k);
    }
    CHECK(mirrored, "every sample the mirror of the clip's");
    CHECK(same, "mirrored twice, the clip");
}

static bool Named(const char* name, size_t length, const char* want)
{
    return length == strlen(want) && memcmp(name, want, length) == 0;
}

static void TestNamed(void)
{
    maniClip clip;
    maniClip baked;
    Clip(maniAdditiveNone, MANI_NO_BONE, &clip);
    maniMirrorClipSource s = Source(&clip);
    CHECK(Bake(&s, 0, &baked).result == mani_success, "a clip bakes");
    uint32_t blink = maniClipFindFloatTrack(&baked, "blinkR", 6);
    uint32_t flag = maniClipFindFloatTrack(&baked, "flag", 4);
    bool floats = maniClipFloatTrackCount(&baked) == 2 && blink < 2 && flag < 2 &&
                  maniClipFindFloatTrack(&baked, "blinkL", 6) == MANI_NO_TRACK;
    const float times[4] = {0.1f, 0.49f, 0.51f, 0.9f};
    for (int i = 0; floats && i < 4; ++i)
    {
        float a[2];
        float b[2];
        floats = maniSampleClipFloats(&clip, times[i], a, 2, NULL) == mani_success &&
                 maniSampleClipFloats(&baked, times[i], b, 2, NULL) == mani_success &&
                 fabsf(a[0] - b[blink]) < 1e-6f && a[1] == b[flag];
    }
    CHECK(floats, "float tracks renamed, their values kept");
    uint32_t foot = maniClipFindDiscreteTrack(&baked, "footR", 5);
    bool discrete = foot == 0;
    for (int i = 0; discrete && i < 4; ++i)
    {
        discrete = maniClipDiscreteValue(&baked, foot, times[i]) ==
                   maniClipDiscreteValue(&clip, 0, times[i]);
    }
    CHECK(discrete, "a discrete track renamed, its keys kept");
    maniClipEvent step = maniClipEventAt(&baked, 0);
    maniClipEvent jump = maniClipEventAt(&baked, 1);
    maniClipEvent left = maniClipMarkerAt(&baked, 0);
    maniClipEvent right = maniClipMarkerAt(&baked, 1);
    CHECK(maniClipEventCount(&baked) == 2 && step.time == 0.25f && step.payload == 7 &&
              Named(step.name, step.nameLength, "stepR") && jump.payload == 9 &&
              Named(jump.name, jump.nameLength, "hopOver"),
          "events renamed");
    CHECK(maniClipMarkerCount(&baked) == 2 && left.time == 0.0f &&
              Named(left.name, left.nameLength, "R") && right.time == 0.5f &&
              Named(right.name, right.nameLength, "L"),
          "markers renamed");
}

// An additive clip's sample k against the difference, from the rest
// mirrored, of its difference added to the rest mirrored.
static bool DifferenceMirrored(const maniClip* clip, const maniClip* baked, uint32_t k)
{
    float time = (float)k / 30.0f;
    maniBlendLayer layer = {s_in, 1.0f, NULL};
    bool ok = maniPoseSetRest(s_rest, &s_skeleton) == mani_success &&
              maniMirrorPose(&s_map, &s_skeleton, s_rest, s_base) == mani_success &&
              maniSampleClip(clip, &s_skeleton, time, s_in) == mani_success &&
              maniAddPoses(&s_skeleton, s_rest, &layer, 1) == mani_success &&
              maniMirrorPose(&s_map, &s_skeleton, s_rest, s_expected) == mani_success &&
              maniPoseDifference(&s_skeleton, s_expected, s_base, s_expected) == mani_success &&
              maniSampleClip(baked, &s_skeleton, time, s_out) == mani_success;
    return ok && maniPoseIsDifference(s_out) && Same(s_out, s_expected);
}

static void TestAdditive(void)
{
    const maniAdditiveBasis bases[2] = {maniAdditiveRest, maniAdditiveFirstFrame};
    for (int i = 0; i < 2; ++i)
    {
        maniClip clip;
        maniClip baked;
        Clip(bases[i], MANI_NO_BONE, &clip);
        maniMirrorClipSource s = Source(&clip);
        CHECK(Bake(&s, 0, &baked).result == mani_success, "an additive clip bakes");
        CHECK(maniClipAdditiveBasis(&baked) == bases[i], "the basis kept");
        bool mirrored = true;
        for (uint32_t k = 0; k < 31; ++k)
        {
            mirrored = mirrored && DifferenceMirrored(&clip, &baked, k);
        }
        CHECK(mirrored, "every difference mirrored");
    }
}

static maniQuat Mul(maniQuat a, maniQuat b)
{
    return (maniQuat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static maniVec3 Rotate(maniQuat q, maniVec3 v)
{
    maniQuat p = Mul(Mul(q, (maniQuat){v.x, v.y, v.z, 0.0f}), (maniQuat){-q.x, -q.y, -q.z, q.w});
    return (maniVec3){p.x, p.y, p.z};
}

// A bone's parent's rest model rotation.
static maniQuat ParentFrame(uint32_t bone)
{
    maniQuat q = REST;
    for (uint32_t p = maniSkeletonParent(&s_skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeletonParent(&s_skeleton, p))
    {
        q = Mul(maniSkeletonRest(&s_skeleton, p).rotation, q);
    }
    return q;
}

// Moves of the clip and the baked over some spans: the baked's change in
// the model (through the partner's parent's frame) the clip's reflected,
// its turn the clip's mirrored.
static bool MovesMirrored(const maniClip* clip, const maniClip* baked, uint32_t bone)
{
    maniQuat from = ParentFrame(bone);
    maniQuat to = ParentFrame(maniMirrorPartner(&s_map, bone));
    const float spans[3][2] = {{0.0f, 1.0f}, {0.2f, 0.5f}, {0.7f, 0.25f}};
    bool moves = true;
    for (int i = 0; i < 3; ++i)
    {
        maniRootMove a;
        maniRootMove b;
        moves = moves && maniClipRootMotion(clip, spans[i][0], spans[i][1], &a) == mani_success &&
                maniClipRootMotion(baked, spans[i][0], spans[i][1], &b) == mani_success;
        maniRootMove m = maniMirrorRootMotion(
            &s_map, (maniRootMove){
                        Rotate(from, a.translation),
                        Mul(Mul(from, a.rotation), (maniQuat){-from.x, -from.y, -from.z, from.w})});
        maniQuat turned = Mul(Mul(to, b.rotation), (maniQuat){-to.x, -to.y, -to.z, to.w});
        moves = moves && Gap(Rotate(to, b.translation), m.translation) < 1e-5 &&
                QuatDot(turned, m.rotation) > 1.0 - 1e-9;
    }
    return moves;
}

static void TestRootMotion(void)
{
    maniClip clip;
    maniClip baked;
    Clip(maniAdditiveNone, kRoot, &clip);
    maniMirrorClipSource s = Source(&clip);
    CHECK(Bake(&s, 0, &baked).result == mani_success, "a clip with root motion bakes");
    maniRootMotionSource declared;
    CHECK(maniClipRootMotionSource(&baked, &declared) && declared.bone == kRoot &&
              declared.translationAxes == (maniRootAxisX | maniRootAxisZ) &&
              declared.rotation == maniRootRotationTwist && declared.twistAxis == 1 &&
              declared.translationReference == maniRootReferenceRest && declared.loop == 0,
          "the root's root motion declared as it was");
    CHECK(MovesMirrored(&clip, &baked, kRoot), "the root's moves reflected");
    bool carried = true;
    for (uint32_t k = 0; k < 31; k += 3)
    {
        carried = carried && SampleMirrored(&clip, &baked, k);
    }
    CHECK(carried, "the samples mirrored with the motion out");
    Clip(maniAdditiveNone, kArmL, &clip);
    s = Source(&clip);
    CHECK(Bake(&s, 0, &baked).result == mani_success, "an arm's root motion bakes");
    CHECK(maniClipRootMotionSource(&baked, &declared) && declared.bone == kArmR &&
              declared.translationAxes == (maniRootAxisX | maniRootAxisY | maniRootAxisZ) &&
              declared.rotation == maniRootRotationWhole,
          "on the partner, the axes and twist off the axes all taken");
    CHECK(MovesMirrored(&clip, &baked, kArmL), "the arm's moves reflected in the model");
}

static void TestRefusals(void)
{
    maniClip clip;
    Clip(maniAdditiveNone, MANI_NO_BONE, &clip);
    maniClip baked;
    maniMirrorClipSource s = Source(&clip);
    s.skeleton = &s_other;
    maniBuildResult r = Bake(&s, 0, &baked);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonMirrorClip && r.other == 0,
          "a skeleton not the map's");
    s.skeleton = &s_moved;
    r = Bake(&s, 0, &baked);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonMirrorClip && r.other == 0,
          "a skeleton of the bone count not the map's");
    maniMirrorMap counted = s_map;
    counted.boneCount = kBones + 1;
    s = Source(&clip);
    s.map = &counted;
    r = Bake(&s, 0, &baked);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonMirrorClip && r.other == 0,
          "a map of another bone count");
    s = Source(&clip);
    maniClip other = clip;
    other.skeletonIdentity ^= 1u;
    s.clip = &other;
    r = Bake(&s, 0, &baked);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonMirrorClip && r.other == 1,
          "a clip of another skeleton");
    maniBuilder* builder = NewBuilder();
    s = Source(&clip);
    CHECK(maniBuildMirroredClip(NULL, &s).result == mani_errorInvalid, "a NULL builder");
    CHECK(maniBuildMirroredClip(builder, NULL).result == mani_errorInvalid, "a NULL source");
    const maniMirrorClipSource fields[3] = {
        {NULL, &s_skeleton, &clip}, {&s_map, NULL, &clip}, {&s_map, &s_skeleton, NULL}};
    bool refused = true;
    for (int i = 0; i < 3; ++i)
    {
        refused = refused && maniBuildMirroredClip(builder, &fields[i]).result == mani_errorInvalid;
    }
    CHECK(refused && maniBuilderMisuses(builder) == 4, "a NULL field; misuses");
    maniDestroyBuilder(builder);
}

int main(void)
{
    Setup();
    TestSamples();
    TestNamed();
    TestAdditive();
    TestRootMotion();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
