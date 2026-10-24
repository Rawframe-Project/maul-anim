// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Compressed clips: compressing a full-precision clip (default, constant
// and animated channels, several segments and one, constants within a
// segment, raw values, float tracks), playing it against the full clip,
// the report, the same bytes from the same input, every refusal of
// opening and of compressing, and the bytes pinned. Run as
// test_clip_compress --seed <file> it writes the fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_view.h"
#include "container.h"
#include "pose_memory.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PINNED_COMPRESSED_HASH 0xa866c371429ce0fbull

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_fullFile[65536];
static size_t s_fullSize;
static _Alignas(16) uint8_t s_file[65536];
static size_t s_size;
static _Alignas(32) uint8_t s_poseA[4096];
static _Alignas(32) uint8_t s_poseB[4096];
static maniSkeleton s_skeleton;
static maniClip s_full;

#define BONE(n, p, y)                                                                              \
    {                                                                                              \
        n, sizeof(n) - 1, p,                                                                       \
        {                                                                                          \
            {0.0f, y, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}                          \
        }                                                                                          \
    }

// A chain of five and a branch, rests 0.3 apart in y.
static const maniBoneSource kBones[6] = {
    BONE("root", -1, 0.0f), BONE("spine", 0, 0.3f), BONE("chest", 1, 0.3f),
    BONE("arm", 2, 0.3f),   BONE("hand", 3, 0.3f),  BONE("leg", 0, -0.3f),
};

#define KEYS 21
static float s_times[KEYS];
static float s_rootT[3 * KEYS];
static float s_rootR[4 * KEYS];
static float s_spineR[4 * KEYS];
static float s_armR[4 * KEYS];
static float s_handT[3 * KEYS];
static float s_legR[4 * KEYS];
static float s_wave[KEYS];
static float s_wide[KEYS];
static const float kChestT[3] = {0.0f, 0.35f, 0.0f};
static const float kChestR[4] = {0.0f, 0.0f, 0.0f, 1.0f};
static const float kArmS[3] = {1.0f, 1.0f, 1.0f};
static const float kOneTime[1] = {0.0f};
static const float kHalf[1] = {0.5f};
static const float kGateTimes[2] = {0.0f, 0.75f};
static const float kGateValues[2] = {1.0f, 0.0f};

static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
static maniBoneTrackSource s_tracks[6];
static maniFloatTrackSource s_floats[4];

static void StoreAxisAngle(float* q, float ax, float ay, float az, float angle)
{
    float s = sinf(angle * 0.5f);
    q[0] = ax * s;
    q[1] = ay * s;
    q[2] = az * s;
    q[3] = cosf(angle * 0.5f);
}

// Keys every 0.1 s over 2 s: smooth motion; the hand still after 1 s;
// the leg turning from 0.5 to 5.5 radians about x, so w passes 0 and x
// stays clear of it.
static void FillKeys(void)
{
    for (int k = 0; k < KEYS; ++k)
    {
        float t = 0.1f * (float)k;
        s_times[k] = t;
        s_rootT[3 * k] = 0.4f * sinf(t * 2.0f);
        s_rootT[3 * k + 1] = 0.9f + 0.05f * cosf(t * 5.0f);
        s_rootT[3 * k + 2] = 1.5f * t;
        StoreAxisAngle(&s_rootR[4 * k], 0.0f, 1.0f, 0.0f, 0.3f * sinf(t * 3.0f));
        StoreAxisAngle(&s_spineR[4 * k], 1.0f, 0.0f, 0.0f, 0.2f * sinf(t * 4.0f + 1.0f));
        StoreAxisAngle(&s_armR[4 * k], 0.0f, 0.0f, 1.0f, 1.2f * sinf(t * 2.5f));
        float still = t < 1.0f ? t : 1.0f;
        s_handT[3 * k] = 0.02f * still;
        s_handT[3 * k + 1] = 0.3f;
        s_handT[3 * k + 2] = 0.0f;
        StoreAxisAngle(&s_legR[4 * k], 1.0f, 0.0f, 0.0f, 0.5f + 2.5f * t);
        s_wave[k] = sinf(t * 6.0f);
        s_wide[k] = 5000.0f * t;
    }
}

static maniChannelSource Linear(const float* values, uint32_t count)
{
    return (maniChannelSource){count == 1 ? kOneTime : s_times, values, count,
                               maniInterpolationLinear};
}

static maniClipSource Source(float duration)
{
    s_tracks[0] = (maniBoneTrackSource){0, 0, Linear(s_rootT, KEYS), Linear(s_rootR, KEYS), kNone};
    s_tracks[1] = (maniBoneTrackSource){1, 0, kNone, Linear(s_spineR, KEYS), kNone};
    // Chest: a constant translation, a rotation at the rest (default).
    s_tracks[2] = (maniBoneTrackSource){2, 0, Linear(kChestT, 1), Linear(kChestR, 1), kNone};
    // Arm: a scale at the rest.
    s_tracks[3] = (maniBoneTrackSource){3, 0, kNone, Linear(s_armR, KEYS), Linear(kArmS, 1)};
    s_tracks[4] = (maniBoneTrackSource){4, 0, Linear(s_handT, KEYS), kNone, kNone};
    s_tracks[5] = (maniBoneTrackSource){5, 0, kNone, Linear(s_legR, KEYS), kNone};
    s_floats[0] = (maniFloatTrackSource){"wave", 4, 0, Linear(s_wave, KEYS)};
    s_floats[1] = (maniFloatTrackSource){"half", 4, 0, Linear(kHalf, 1)};
    s_floats[2] =
        (maniFloatTrackSource){"gate", 4, 0, {kGateTimes, kGateValues, 2, maniInterpolationStep}};
    // Wide enough that the float precision needs more than 23 bits.
    s_floats[3] = (maniFloatTrackSource){"wide", 4, 0, Linear(s_wide, KEYS)};
    return (maniClipSource){.skeleton = &s_skeleton,
                            .boneTracks = s_tracks,
                            .floatTracks = s_floats,
                            .boneTrackCount = 6,
                            .floatTrackCount = 4,
                            .loop = maniLoopClamp,
                            .sampleRate = 30.0f,
                            .duration = duration};
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void OpenSkeleton(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {kBones, 6, 0};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "skeleton builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
          "copies");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
}

// The full clip of a source, opened into s_full.
static void BuildFullFrom(const maniClipSource* source)
{
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildClip(builder, source).result == mani_success, "full clip builds");
    CHECK(maniCopyBuilt(builder, s_fullFile, sizeof s_fullFile, &s_fullSize) == mani_success,
          "copies the full clip");
    maniDestroyBuilder(builder);
    CHECK(maniOpenClip(s_fullFile, s_fullSize, &s_full).result == mani_success, "full opens");
}

static void BuildFull(float duration)
{
    maniClipSource source = Source(duration);
    BuildFullFrom(&source);
}

// s_full compressed into s_file.
static maniResult Compress(const maniCompressOptions* options, maniCompressReport* report)
{
    maniBuilder* builder = NewBuilder();
    maniResult r = maniCompressClip(builder, &s_full, &s_skeleton, options, report);
    if (r == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success,
              "copies the compressed clip");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniFileResult Open(maniClip* clip)
{
    return maniOpenClip(s_file, s_size, clip);
}

static float Distance(const maniAffine* a, const maniAffine* b)
{
    float dx = a->m[9] - b->m[9];
    float dy = a->m[10] - b->m[10];
    float dz = a->m[11] - b->m[11];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

// The farthest any bone's origin and float track lies from the full
// clip's over times across the clip and past its ends.
static void Compare(const maniClip* packed, float* boneOut, float* floatOut)
{
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    *boneOut = 0.0f;
    *floatOut = 0.0f;
    for (float t = -0.1f; t < s_full.duration + 0.1f; t += 0.0071f)
    {
        CHECK(maniSampleClip(&s_full, &s_skeleton, t, a) == mani_success, "full samples");
        CHECK(maniSampleClip(packed, &s_skeleton, t, b) == mani_success, "compressed samples");
        CHECK(maniComputeModel(a, &s_skeleton) == mani_success, "full models");
        CHECK(maniComputeModel(b, &s_skeleton) == mani_success, "compressed models");
        for (uint32_t bone = 0; bone < 6; ++bone)
        {
            maniAffine ma = maniPoseModel(a, bone);
            maniAffine mb = maniPoseModel(b, bone);
            *boneOut = fmaxf(*boneOut, Distance(&ma, &mb));
        }
        float fa[4];
        float fb[4];
        uint32_t n = s_full.floatTrackCount;
        CHECK(maniSampleClipFloats(&s_full, t, fa, 4, NULL) == mani_success, "floats");
        CHECK(maniSampleClipFloats(packed, t, fb, 4, NULL) == mani_success, "floats");
        for (uint32_t i = 0; i < n; ++i)
        {
            *floatOut = fmaxf(*floatOut, fabsf(fa[i] - fb[i]) / fmaxf(1.0f, fabsf(fa[i])));
        }
    }
}

// The rate bytes of every segment: how many at 0 and at the raw rate.
static void CountRates(const maniClip* clip, uint32_t* zeros, uint32_t* raws)
{
    *zeros = 0;
    *raws = 0;
    uint32_t n = clip->animatedChannels + clip->animatedFloats;
    for (uint32_t s = 0; s < clip->segmentCount; ++s)
    {
        const uint8_t* rates = clip->segmentData + maniLoadU32(clip->segments + 20 * (size_t)s + 8);
        for (uint32_t j = 0; j < n; ++j)
        {
            *zeros += rates[j] == 0 ? 1 : 0;
            *raws += rates[j] == 24 ? 1 : 0;
        }
    }
}

static void TestSegments(void)
{
    BuildFull(0.0f);
    maniCompressOptions options = maniDefaultCompressOptions();
    maniCompressReport report;
    CHECK(Compress(&options, &report) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK(maniClipCompressed(&clip) && !maniClipCompressed(&s_full), "forms");
    CHECK(maniClipSampleCount(&clip) == 61 && clip.segmentCount == 4, "four segments");
    CHECK(maniClipDuration(&clip) == maniClipDuration(&s_full), "duration");
    CHECK(report.bytes == s_size && s_size < s_fullSize / 2, "smaller");
    CHECK(report.shellError > 0.0 && report.shellError <= 0.0001, "shell error within precision");
    CHECK(report.floatError <= 0.00001, "float error within precision");
    // Translation, rotation per bone track; the chest's constant and its
    // default, the arm's default scale.
    CHECK(clip.animatedChannels == 6 && clip.animatedFloats == 3, "animated sub-tracks");
    CHECK((clip.forms[2] & 3) == maniClipFormConstant, "chest translation constant");
    CHECK(((clip.forms[2] >> 2) & 3) == maniClipFormDefault, "chest rotation default");
    CHECK(((clip.forms[3] >> 4) & 3) == maniClipFormDefault, "arm scale default");
    CHECK((clip.forms[0] >> 6) == 0, "the root's rotation leaves w out");
    CHECK((clip.forms[5] >> 6) == 3, "the leg's leaves x out");
    CHECK(clip.forms[6 + maniClipFindFloatTrack(&clip, "half", 4)] == maniClipFormConstant,
          "half constant");
    uint32_t zeros = 0;
    uint32_t raws = 0;
    CountRates(&clip, &zeros, &raws);
    CHECK(zeros > 0, "the still hand at rate 0");
    CHECK(raws >= 2, "the wide track raw");
    float bone = 0.0f;
    float value = 0.0f;
    Compare(&clip, &bone, &value);
    // Between samples too, a little more: interpolation.
    CHECK(bone < 0.0002f, "bones play near the full clip");
    CHECK(value < 0.0001f, "floats play near the full clip");
    // A finer precision: the error follows it down.
    options.precision = 1e-7f;
    CHECK(Compress(&options, &report) == mani_success, "compresses finely");
    CHECK(Open(&clip).result == mani_success, "opens");
    Compare(&clip, &bone, &value);
    // Playback in binary32 adds its own rounding between samples.
    CHECK(report.shellError <= 1e-7 && bone < 1e-6f, "finely");
}

// The hand's height wobbling by 0.8 of the precision about 0.8: neither
// the rest nor its first value folds it, but one value at the middle
// would; with one segment there are no segment ranges, so rate 0 is no
// choice.
static float s_handWobble[3 * KEYS];

static void TestOneSegment(void)
{
    for (int k = 0; k < KEYS; ++k)
    {
        s_handWobble[3 * k] = 0.0f;
        s_handWobble[3 * k + 1] = 0.8f + 0.8e-4f * cosf(0.5f * (float)k);
        s_handWobble[3 * k + 2] = 0.0f;
    }
    maniClipSource source = Source(0.5f);
    s_tracks[4].translation = Linear(s_handWobble, KEYS);
    BuildFullFrom(&source);
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK(maniClipSampleCount(&clip) == 16 && clip.segmentCount == 1, "one segment");
    uint32_t zeros = 0;
    uint32_t raws = 0;
    CountRates(&clip, &zeros, &raws);
    CHECK(zeros == 0, "no rate 0 in one segment");
    float bone = 0.0f;
    float value = 0.0f;
    Compare(&clip, &bone, &value);
    CHECK(bone < 0.0002f && value < 0.0001f, "plays near the full clip");
}

static void TestOneSample(void)
{
    // Keys only at 0: one sample, nothing animated.
    maniClipSource source = Source(0.0f);
    source.boneTracks = &s_tracks[2];
    source.boneTrackCount = 1;
    source.floatTracks = &s_floats[1];
    source.floatTrackCount = 1;
    BuildFullFrom(&source);
    CHECK(maniClipSampleCount(&s_full) == 1, "one sample");
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK(clip.segmentCount == 1 && clip.animatedChannels == 0 && clip.animatedFloats == 0,
          "nothing animated");
    float bone = 0.0f;
    float value = 0.0f;
    Compare(&clip, &bone, &value);
    CHECK(bone == 0.0f && value == 0.0f, "plays the full clip");
}

// Channels that move less than the precision fold: a translation
// wobbling by a micrometre keeps its first value, a rotation by a
// microradian is the rest.
static float s_chestT[3 * KEYS];
static float s_chestR[4 * KEYS];

static void TestFoldWithinPrecision(void)
{
    for (int k = 0; k < KEYS; ++k)
    {
        float wobble = 1e-6f * sinf((float)k);
        s_chestT[3 * k] = wobble;
        s_chestT[3 * k + 1] = 0.35f;
        s_chestT[3 * k + 2] = 0.0f;
        StoreAxisAngle(&s_chestR[4 * k], 0.0f, 1.0f, 0.0f, wobble);
    }
    maniClipSource source = Source(0.0f);
    s_tracks[2].translation = Linear(s_chestT, KEYS);
    s_tracks[2].rotation = Linear(s_chestR, KEYS);
    BuildFullFrom(&source);
    maniCompressOptions options = maniDefaultCompressOptions();
    maniCompressReport report;
    CHECK(Compress(&options, &report) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK((clip.forms[2] & 3) == maniClipFormConstant, "a wobbling translation constant");
    CHECK(((clip.forms[2] >> 2) & 3) == maniClipFormDefault, "a wobbling rotation the rest");
    CHECK(report.shellError <= 0.0001, "within precision");
    // At a precision below the wobble they stay animated.
    options.precision = 1e-7f;
    CHECK(Compress(&options, &report) == mani_success, "compresses finely");
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK((clip.forms[2] & 3) == maniClipFormAnimated, "the translation animated");
    CHECK(((clip.forms[2] >> 2) & 3) == maniClipFormAnimated, "the rotation animated");
}

// A precision no rate meets (below binary32's resolution at these
// values): the search keeps what misses by least.
static void TestNoRatePasses(void)
{
    BuildFull(0.0f);
    maniCompressOptions options = maniDefaultCompressOptions();
    options.precision = 1e-12f;
    maniCompressReport report;
    CHECK(Compress(&options, &report) == mani_success, "compresses");
    CHECK(report.shellError > 1e-12 && report.shellError < 1e-6, "misses by little");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    uint32_t zeros = 0;
    uint32_t raws = 0;
    CountRates(&clip, &zeros, &raws);
    // More than the wide float track's.
    CHECK(raws > 4, "raw rates");
}

// Random hierarchies and motion: every bone within its precision at
// every sample, whatever the tolerances.
// Random skeletons draw 2 to RANDOM_DRAW - 1 bones; the arrays hold up
// to RANDOM_BONES, for the batch tails' fixed counts.
#define RANDOM_BONES 16
#define RANDOM_DRAW  12
static uint32_t s_random = 12345;
static maniBoneSource s_randomBones[RANDOM_BONES];
static float s_randomT[RANDOM_BONES][3 * KEYS];
static float s_randomR[RANDOM_BONES][4 * KEYS];
static float s_randomS[RANDOM_BONES][3 * KEYS];
static maniBoneTrackSource s_randomTracks[RANDOM_BONES];

static float Random(void)
{
    s_random = s_random * 1664525u + 1013904223u;
    return (float)(s_random >> 8) / 16777216.0f;
}

static void RandomSkeleton(uint32_t bones)
{
    for (uint32_t b = 0; b < bones; ++b)
    {
        s_randomBones[b] =
            (maniBoneSource){"b",
                             1,
                             b == 0 ? -1 : (int32_t)(Random() * (float)b),
                             {{Random() - 0.5f, 0.1f + 0.4f * Random(), Random() - 0.5f},
                              {0.0f, 0.0f, 0.0f, 1.0f},
                              {1.0f, 1.0f, 1.0f}}};
    }
    // Names must differ: one letter each.
    static char names[RANDOM_BONES][2];
    for (uint32_t b = 0; b < bones; ++b)
    {
        names[b][0] = (char)('a' + b);
        s_randomBones[b].name = names[b];
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {s_randomBones, bones, 0};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "random skeleton builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
          "copies");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
}

// Bone b's keys: a translation drifting, a rotation swinging about a
// random axis, a scale breathing unevenly.
static void RandomKeys(uint32_t b)
{
    float axis[3] = {Random() - 0.5f, Random() - 0.5f, Random() - 0.5f};
    float length = sqrtf(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]) + 1e-3f;
    float swing = 3.0f * Random();
    // Half the bones keep their rotations' negatives: the same turns, with
    // the component left out negative.
    float flip = Random() < 0.5f ? -1.0f : 1.0f;
    float speed = 1.0f + 4.0f * Random();
    for (int k = 0; k < KEYS; ++k)
    {
        float t = s_times[k];
        for (int c = 0; c < 3; ++c)
        {
            s_randomT[b][3 * k + c] = 0.3f * sinf(speed * t + (float)c);
            s_randomS[b][3 * k + c] = 1.0f + 0.2f * sinf(speed * t * (float)(c + 1));
        }
        StoreAxisAngle(&s_randomR[b][4 * k], axis[0] / length, axis[1] / length, axis[2] / length,
                       swing * sinf(speed * t));
        for (int c = 0; c < 4; ++c)
        {
            s_randomR[b][4 * k + c] *= flip;
        }
    }
    uint32_t pick = (uint32_t)(Random() * 8.0f);
    s_randomTracks[b] = (maniBoneTrackSource){b, 0, (pick & 1) ? Linear(s_randomT[b], KEYS) : kNone,
                                              (pick & 2) ? Linear(s_randomR[b], KEYS) : kNone,
                                              (pick & 4) ? Linear(s_randomS[b], KEYS) : kNone};
}

static void TestRandomWithinPrecision(void)
{
    static const float kPrecisions[3] = {1e-3f, 1e-4f, 1e-5f};
    static const float kShells[3] = {0.01f, 0.03f, 0.1f};
    for (int round = 0; round < 24; ++round)
    {
        uint32_t bones = 2 + (uint32_t)(Random() * (RANDOM_DRAW - 2));
        RandomSkeleton(bones);
        uint32_t tracks = 0;
        for (uint32_t b = 0; b < bones; ++b)
        {
            RandomKeys(b);
            bool any = s_randomTracks[b].translation.keyCount +
                           s_randomTracks[b].rotation.keyCount + s_randomTracks[b].scale.keyCount >
                       0;
            s_randomTracks[tracks] = s_randomTracks[b];
            tracks += any ? 1 : 0;
        }
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = s_randomTracks,
                                 .boneTrackCount = tracks,
                                 .sampleRate = 30.0f,
                                 .duration = 0.5f + 1.5f * Random()};
        BuildFullFrom(&source);
        maniCompressOptions options = maniDefaultCompressOptions();
        options.precision = kPrecisions[round % 3];
        options.shellDistance = kShells[(round / 3) % 3];
        maniCompressReport report;
        CHECK(Compress(&options, &report) == mani_success, "random compresses");
        CHECK(report.shellError <= (double)options.precision, "random within precision");
        if (report.shellError > (double)options.precision)
        {
            printf("  round %d: %g over %g at bone %u sample %u\n", round, report.shellError,
                   (double)options.precision, report.shellBone, report.shellSample);
        }
    }
    OpenSkeleton();
}

// Two poses' locals, bit for bit.
static bool SameLocals(const maniPose* a, const maniPose* b)
{
    for (uint32_t bone = 0; bone < maniPoseBoneCount(a); ++bone)
    {
        maniTransform ta = maniPoseLocal(a, bone);
        maniTransform tb = maniPoseLocal(b, bone);
        if (memcmp(&ta, &tb, sizeof ta) != 0)
        {
            return false;
        }
    }
    return true;
}

static _Alignas(8) uint8_t s_cursorMemory[4096];

// A walk of times: steps forward of a fraction of a sample and of
// several, jumps back, past both ends, NaN.
static float WalkTime(uint32_t step, float t, float duration)
{
    switch (step % 7)
    {
    case 0:
        return t + 0.004f;
    case 1:
        return t + 0.0333f;
    case 2:
        return t + 0.21f * Random();
    case 3:
        return duration * Random();
    case 4:
        return t - 0.02f;
    case 5:
        return step % 3 == 0 ? -1.0f : duration + 1.0f;
    default:
        return step % 5 == 0 ? NAN : t;
    }
}

// A clip played through a cursor, against random access: the same bits.
static void CursorWalk(const maniClip* clip, maniClipCursor* cursor)
{
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    float t = 0.0f;
    bool same = true;
    for (uint32_t step = 0; step < 2000; ++step)
    {
        t = WalkTime(step, t, clip->duration);
        CHECK(maniSampleClip(clip, &s_skeleton, t, a) == mani_success, "samples");
        CHECK(maniSampleClipCursor(cursor, clip, &s_skeleton, t, b) == mani_success, "cursor");
        float fa[4];
        float fb[4];
        uint32_t n = 0;
        CHECK(maniSampleClipFloats(clip, t, fa, 4, &n) == mani_success, "floats");
        CHECK(maniSampleClipFloatsCursor(cursor, clip, t, fb, 4, &n) == mani_success,
              "cursor floats");
        same = same && SameLocals(a, b) && memcmp(fa, fb, n * sizeof(float)) == 0 &&
               maniPoseIsDifference(a) == maniPoseIsDifference(b);
        t = t == t ? t : 0.0f;
    }
    CHECK(same, "the cursor plays random access's bits");
}

static void TestCursor(void)
{
    BuildFull(0.0f);
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    maniClip packed;
    CHECK(Open(&packed).result == mani_success, "opens");
    size_t size = maniClipCursorSize(&packed);
    CHECK(size > maniClipCursorSize(&s_full) && size <= sizeof s_cursorMemory, "cursor size");
    maniClipCursor* cursor = NULL;
    CHECK(maniInitClipCursor(s_cursorMemory, size, &cursor) == mani_success, "cursor");
    CursorWalk(&packed, cursor);
    // Another clip rebinds it, and so does the same clip at another
    // address; then back.
    CursorWalk(&s_full, cursor);
    maniClip copy = packed;
    CursorWalk(&copy, cursor);
    CursorWalk(&packed, cursor);
    // One view, one address: another clip of the same shape (another
    // precision) rebinds by identity; the same identity grown in shape
    // rebinds and is refused for room.
    static uint8_t other[65536];
    size_t otherSize = s_size;
    memcpy(other, s_file, s_size);
    maniCompressOptions finer = maniDefaultCompressOptions();
    finer.precision = 0.00005f;
    CHECK(Compress(&finer, NULL) == mani_success, "compresses finer");
    static maniClip view;
    CHECK(maniOpenClip(other, otherSize, &view).result == mani_success, "opens the first");
    maniClip second;
    CHECK(Open(&second).result == mani_success, "opens the second");
    CHECK(maniClipCursorSize(&second) == maniClipCursorSize(&view), "the same shape");
    CursorWalk(&view, cursor);
    // The same time on both: the first clip's samples held at the address
    // must not play for the second.
    maniPose* held = NULL;
    maniPose* fresh = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &held) == mani_success, "pose");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &fresh) == mani_success, "pose");
    CHECK(maniSampleClipCursor(cursor, &view, &s_skeleton, 0.37f, held) == mani_success, "first");
    view = second;
    CHECK(maniSampleClipCursor(cursor, &view, &s_skeleton, 0.37f, held) == mani_success, "second");
    CHECK(maniSampleClip(&view, &s_skeleton, 0.37f, fresh) == mani_success, "random");
    CHECK(SameLocals(held, fresh), "rebound by identity");
    CursorWalk(&view, cursor);
    view.animatedFloats += 1000;
    maniPose* grownPose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &grownPose) == mani_success, "pose");
    CHECK(maniSampleClipCursor(cursor, &view, &s_skeleton, 0.5f, grownPose) == mani_errorCapacity,
          "a shape grown at the same identity and address");
    CHECK(Compress(&options, NULL) == mani_success, "compresses again");
    CHECK(Open(&packed).result == mani_success, "opens again");
    // Too small for the compressed clip: refused, nothing written.
    maniClipCursor* small = NULL;
    CHECK(maniInitClipCursor(s_cursorMemory, maniClipCursorSize(&s_full), &small) == mani_success,
          "small cursor");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &pose) == mani_success, "pose");
    CHECK(maniSampleClipCursor(small, &s_full, &s_skeleton, 0.5f, pose) == mani_success,
          "a small cursor plays a full clip");
    CHECK(maniSampleClipCursor(small, &packed, &s_skeleton, 0.5f, pose) == mani_errorCapacity,
          "too small");
    float values[4];
    uint32_t n = 0;
    CHECK(maniSampleClipFloatsCursor(small, &packed, 0.5f, values, 4, &n) == mani_errorCapacity &&
              n == 4,
          "too small for floats");
    // The smallest cursor: no larger, the same bits; below it, refused.
    size_t compact = maniClipCursorCompactSize(&packed);
    CHECK(compact > 0 && maniClipCursorCompactSize(NULL) == 0, "compact size");
    // Smaller where a vector backend holds the groups, the same on the
    // scalar one.
    bool scalar = strcmp(maniGetSimdBackend(), "scalar") == 0;
    CHECK(scalar ? compact == size : compact < size, "the groups' bytes");
    maniClipCursor* small2 = NULL;
    CHECK(maniInitClipCursor(s_cursorMemory, compact, &small2) == mani_success, "compact cursor");
    // Nothing past its memory is written: a guard pattern after it stays.
    memset(s_cursorMemory + compact, 0xA5, sizeof s_cursorMemory - compact);
    CursorWalk(&packed, small2);
    bool guarded = true;
    for (size_t i = compact; i < sizeof s_cursorMemory; ++i)
    {
        guarded = guarded && s_cursorMemory[i] == 0xA5;
    }
    CHECK(guarded, "a compact cursor stays within its memory");
    CHECK(maniInitClipCursor(s_cursorMemory, compact - sizeof(float), &small2) == mani_success,
          "too small a cursor");
    CHECK(maniSampleClipCursor(small2, &packed, &s_skeleton, 0.5f, pose) == mani_errorCapacity,
          "below the compact size");
    // Misuse.
    CHECK(maniClipCursorSize(NULL) == 0, "no clip");
    CHECK(maniInitClipCursor(NULL, size, &cursor) == mani_errorInvalid, "no memory");
    CHECK(maniInitClipCursor(s_cursorMemory, size, NULL) == mani_errorInvalid, "no out");
    CHECK(maniInitClipCursor(s_cursorMemory + 4, size, &cursor) == mani_errorInvalid, "misaligned");
    CHECK(maniInitClipCursor(s_cursorMemory, 8, &cursor) == mani_errorCapacity, "no room");
    CHECK(maniInitClipCursor(s_cursorMemory, size, &cursor) == mani_success, "again");
    CHECK(maniSampleClipCursor(NULL, &packed, &s_skeleton, 0.5f, pose) == mani_errorInvalid,
          "no cursor");
    CHECK(maniSampleClipCursor(cursor, NULL, &s_skeleton, 0.5f, pose) == mani_errorInvalid,
          "no clip to sample");
    CHECK(maniSampleClipFloatsCursor(NULL, &packed, 0.5f, values, 4, &n) == mani_errorInvalid,
          "no cursor for floats");
    CHECK(maniSampleClipFloatsCursor(cursor, &packed, 0.5f, NULL, 4, &n) == mani_errorInvalid,
          "no values");
    CHECK(maniSampleClipFloatsCursor(cursor, &packed, 0.5f, values, 3, &n) == mani_errorCapacity,
          "few values");
    static _Alignas(8) uint8_t zeros[256];
    CHECK(maniSampleClipCursor((maniClipCursor*)(void*)zeros, &packed, &s_skeleton, 0.5f, pose) ==
              mani_errorInvalid,
          "not a cursor");
}

// A random subset of a random clip's bones, its groups filled in part:
// the listed bones as random access samples them, the others left.
static void RandomSubset(const maniClip* clip, uint32_t bones)
{
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    uint32_t list[RANDOM_BONES];
    uint32_t count = 0;
    for (uint32_t bone = 0; bone < bones; ++bone)
    {
        if (Random() < 0.5f)
        {
            list[count++] = bone;
        }
    }
    const maniTransform marker = {{9.0f, 9.0f, 9.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {2.0f, 2.0f, 2.0f}};
    bool right = true;
    for (float t = 0.0f; t < clip->duration; t += 0.071f)
    {
        CHECK(maniSampleClip(clip, &s_skeleton, t, a) == mani_success, "samples");
        // b of the clip's kind, then marked.
        CHECK(maniSampleClip(clip, &s_skeleton, 0.0f, b) == mani_success, "kind");
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            maniPoseSetLocal(b, bone, marker);
        }
        CHECK(maniSampleClipBones(clip, &s_skeleton, t, list, count, b) == mani_success,
              "random subset");
        uint32_t next = 0;
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            bool listed = next < count && list[next] == bone;
            next += listed ? 1 : 0;
            maniTransform want = listed ? maniPoseLocal(a, bone) : marker;
            maniTransform got = maniPoseLocal(b, bone);
            right = right && memcmp(&want, &got, sizeof want) == 0;
        }
    }
    CHECK(right, "a random subset: listed bones as random access, others left");
}

// Random clips through a cursor: one group of bones and two, rests
// turned (not exactly unit in floats) with -0 in their translations,
// constant channels, and rotations turning past w = 0 so samples meet
// on the longer arc, constant rotations; every other clip additive.
static void TestCursorRandom(void)
{
    for (int round = 0; round < 24; ++round)
    {
        uint32_t bones = 2 + (uint32_t)(Random() * (RANDOM_DRAW - 2));
        for (uint32_t b = 0; b < bones; ++b)
        {
            float x = Random() - 0.5f;
            float y = Random() - 0.5f;
            float z = Random() - 0.5f;
            float length = sqrtf(x * x + y * y + z * z) + 1e-3f;
            StoreAxisAngle(s_randomR[b], x / length, y / length, z / length, 6.0f * Random());
        }
        RandomSkeleton(bones);
        for (uint32_t b = 0; b < bones; ++b)
        {
            const float* q = s_randomR[b];
            s_randomBones[b].rest.rotation = (maniQuat){q[0], q[1], q[2], q[3]};
            s_randomBones[b].rest.translation.z =
                b % 3 == 0 ? -0.0f : s_randomBones[b].rest.translation.z;
        }
        maniBuilder* builder = NewBuilder();
        maniSkeletonSource skeleton = {s_randomBones, bones, 0};
        CHECK(maniBuildSkeleton(builder, &skeleton).result == mani_success, "turned skeleton");
        size_t size = 0;
        CHECK(maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
              "copies");
        maniDestroyBuilder(builder);
        CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
        uint32_t tracks = 0;
        for (uint32_t b = 0; b < bones; ++b)
        {
            RandomKeys(b);
            float x = Random() - 0.5f;
            float y = Random() - 0.5f;
            float z = Random() - 0.5f;
            float length = sqrtf(x * x + y * y + z * z) + 1e-3f;
            bool still = Random() < 0.25f;
            // A quarter of the rotations held at one turn: constant, and
            // not the rest.
            bool held = Random() < 0.25f;
            for (int k = 0; k < KEYS; ++k)
            {
                StoreAxisAngle(&s_randomR[b][4 * k], x / length, y / length, z / length,
                               held ? 1.3f : 7.0f * sinf(3.0f * s_times[k]));
                for (int c = 0; c < 3 && still; ++c)
                {
                    s_randomT[b][3 * k + c] = s_randomT[b][c];
                }
            }
            bool any = s_randomTracks[b].translation.keyCount +
                           s_randomTracks[b].rotation.keyCount + s_randomTracks[b].scale.keyCount >
                       0;
            s_randomTracks[tracks] = s_randomTracks[b];
            tracks += any ? 1 : 0;
        }
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = s_randomTracks,
                                 .boneTrackCount = tracks,
                                 .sampleRate = 30.0f,
                                 .duration = 0.5f + 1.5f * Random(),
                                 .additive = round % 2 == 0 ? maniAdditiveNone : maniAdditiveRest};
        BuildFullFrom(&source);
        maniCompressOptions options = maniDefaultCompressOptions();
        CHECK(Compress(&options, NULL) == mani_success, "random compresses");
        maniClip packed;
        CHECK(Open(&packed).result == mani_success, "opens");
        // Half the rounds through the smallest cursor, which walks its
        // samples where the full one holds them in groups too.
        maniClipCursor* cursor = NULL;
        size_t memory =
            round % 4 < 2 ? maniClipCursorSize(&packed) : maniClipCursorCompactSize(&packed);
        CHECK(memory <= sizeof s_cursorMemory, "cursor size");
        CHECK(maniInitClipCursor(s_cursorMemory, memory, &cursor) == mani_success, "cursor");
        CursorWalk(&packed, cursor);
        RandomSubset(&packed, bones);
    }
    OpenSkeleton();
}

// Random access's rotations pass the lanes in batches of eight (on a
// vector backend), the last one part full, its unused lanes never
// written: clips rotating every bone of 7, 9 and 15 bones leave a last
// batch of seven, one and seven, played against the cursor, whose held
// groups never batch.
static void TestBatchTails(void)
{
    static const uint32_t kTails[3] = {7, 9, 15};
    for (int i = 0; i < 3; ++i)
    {
        uint32_t bones = kTails[i];
        RandomSkeleton(bones);
        for (uint32_t b = 0; b < bones; ++b)
        {
            RandomKeys(b);
            s_randomTracks[b] =
                (maniBoneTrackSource){b, 0, kNone, Linear(s_randomR[b], KEYS), kNone};
        }
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = s_randomTracks,
                                 .boneTrackCount = bones,
                                 .sampleRate = 30.0f,
                                 .duration = 1.0f};
        BuildFullFrom(&source);
        maniCompressOptions options = maniDefaultCompressOptions();
        CHECK(Compress(&options, NULL) == mani_success, "rotations compress");
        maniClip packed;
        CHECK(Open(&packed).result == mani_success, "opens");
        maniClipCursor* cursor = NULL;
        CHECK(maniInitClipCursor(s_cursorMemory, maniClipCursorSize(&packed), &cursor) ==
                  mani_success,
              "cursor");
        CursorWalk(&packed, cursor);
    }
    OpenSkeleton();
}

// A clip of no tracks: every bone its rest, bit for bit as the skeleton
// holds it (the identity for an additive clip), on 7, 9 and 12 bones: a
// group of eight turned whole and the bones past it one by one, a
// difference's bones one by one.
static void TestRestFill(void)
{
    static const uint32_t kCounts[3] = {7, 9, 12};
    const maniTransform identity = {
        {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    bool right = true;
    for (int i = 0; i < 6; ++i)
    {
        uint32_t bones = kCounts[i % 3];
        bool additive = i >= 3;
        RandomSkeleton(bones);
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = NULL,
                                 .boneTrackCount = 0,
                                 .sampleRate = 30.0f,
                                 .duration = 0.5f,
                                 .additive = additive ? maniAdditiveRest : maniAdditiveNone};
        BuildFullFrom(&source);
        maniCompressOptions options = maniDefaultCompressOptions();
        CHECK(Compress(&options, NULL) == mani_success, "a clip of rests compresses");
        maniClip packed;
        CHECK(Open(&packed).result == mani_success, "opens");
        maniPose* pose = NULL;
        CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &pose) == mani_success, "pose");
        CHECK(maniSampleClip(&packed, &s_skeleton, 0.25f, pose) == mani_success, "samples");
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            maniTransform want = additive ? identity : maniSkeletonRest(&s_skeleton, bone);
            maniTransform got = maniPoseLocal(pose, bone);
            right = right && memcmp(&want, &got, sizeof want) == 0;
        }
    }
    CHECK(right, "a clip of rests samples every bone's rest bits");
    OpenSkeleton();
}

// Some bones sampled: those as the whole clip samples them, every other
// one as it was.
static void SubsetOf(const maniClip* clip)
{
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    // Listed: the spine, the chest (a constant channel and a default
    // one) and the leg.
    static const uint32_t kList[3] = {1, 2, 5};
    const maniTransform marker = {{9.0f, 9.0f, 9.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {2.0f, 2.0f, 2.0f}};
    bool right = true;
    for (float t = 0.0f; t < clip->duration; t += 0.137f)
    {
        CHECK(maniSampleClip(clip, &s_skeleton, t, a) == mani_success, "samples");
        for (uint32_t bone = 0; bone < 6; ++bone)
        {
            maniPoseSetLocal(b, bone, marker);
        }
        CHECK(maniSampleClipBones(clip, &s_skeleton, t, kList, 3, b) == mani_success, "subset");
        for (uint32_t bone = 0; bone < 6; ++bone)
        {
            bool listed = bone == 1 || bone == 2 || bone == 5;
            maniTransform want = listed ? maniPoseLocal(a, bone) : marker;
            maniTransform got = maniPoseLocal(b, bone);
            right = right && memcmp(&want, &got, sizeof want) == 0;
        }
    }
    CHECK(right, "listed bones sampled, others left");
    static const uint32_t kDown[2] = {2, 1};
    static const uint32_t kSame[2] = {2, 2};
    static const uint32_t kPast[1] = {6};
    CHECK(maniSampleClipBones(clip, &s_skeleton, 0.5f, kDown, 2, b) == mani_errorInvalid,
          "decreasing");
    CHECK(maniSampleClipBones(clip, &s_skeleton, 0.5f, kSame, 2, b) == mani_errorInvalid,
          "repeated");
    CHECK(maniSampleClipBones(clip, &s_skeleton, 0.5f, kPast, 1, b) == mani_errorInvalid,
          "past the bones");
    CHECK(maniSampleClipBones(clip, &s_skeleton, 0.5f, NULL, 1, b) == mani_errorInvalid, "no list");
    CHECK(maniSampleClipBones(clip, &s_skeleton, 0.5f, NULL, 0, b) == mani_success, "none");
    CHECK(maniSampleClipBones(NULL, &s_skeleton, 0.5f, kList, 3, b) == mani_errorInvalid,
          "no clip");
    // A subset keeps the other bones: never into a difference.
    maniPoseSetKind(b, true);
    CHECK(maniSampleClipBones(clip, &s_skeleton, 0.5f, kList, 3, b) == mani_errorInvalid,
          "a difference");
    maniTransform got = maniPoseLocal(b, 0);
    CHECK(memcmp(&got, &marker, sizeof got) == 0, "refusals leave the pose");
    // A whole sample makes it a pose again.
    CHECK(maniSampleClip(clip, &s_skeleton, 0.5f, b) == mani_success && !maniPoseIsDifference(b),
          "a whole sample: a pose");
}

static void TestSubsets(void)
{
    BuildFull(0.0f);
    SubsetOf(&s_full);
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    maniClip packed;
    CHECK(Open(&packed).result == mani_success, "opens");
    SubsetOf(&packed);
    CHECK(maniClipIdentity(&packed) == maniLoadU64(s_file + 40), "identity");
}

// The test's own shell metric, binary64: each bone's matrix through the
// hierarchy, three points at the shell distance on its axes.
typedef struct Matrix
{
    double m[12];
} Matrix;

static Matrix LocalMatrix(maniTransform t)
{
    double x = (double)t.rotation.x;
    double y = (double)t.rotation.y;
    double z = (double)t.rotation.z;
    double w = (double)t.rotation.w;
    double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + w * z),     2 * (x * z - w * y),
                   2 * (x * y - w * z),     1 - 2 * (x * x + z * z), 2 * (y * z + w * x),
                   2 * (x * z + w * y),     2 * (y * z - w * x),     1 - 2 * (x * x + y * y)};
    double s[3] = {(double)t.scale.x, (double)t.scale.y, (double)t.scale.z};
    Matrix out;
    for (int c = 0; c < 9; ++c)
    {
        out.m[c] = r[c] * s[c / 3];
    }
    out.m[9] = (double)t.translation.x;
    out.m[10] = (double)t.translation.y;
    out.m[11] = (double)t.translation.z;
    return out;
}

static Matrix Product(const Matrix* a, const Matrix* b)
{
    Matrix out;
    for (int c = 0; c < 4; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            double v = a->m[k] * b->m[3 * c] + a->m[3 + k] * b->m[3 * c + 1] +
                       a->m[6 + k] * b->m[3 * c + 2];
            out.m[3 * c + k] = c == 3 ? v + a->m[9 + k] : v;
        }
    }
    return out;
}

static void PoseMatrices(const maniPose* pose, Matrix* out)
{
    for (uint32_t b = 0; b < maniSkeletonBoneCount(&s_skeleton); ++b)
    {
        Matrix local = LocalMatrix(maniPoseLocal(pose, b));
        uint32_t parent = maniSkeletonParent(&s_skeleton, b);
        out[b] = parent == MANI_NO_BONE ? local : Product(&out[parent], &local);
    }
}

static double ShellApart(const Matrix* a, const Matrix* b, double distance)
{
    double worst = 0.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        double squared = 0.0;
        for (int k = 0; k < 3; ++k)
        {
            double d =
                (a->m[3 * axis + k] - b->m[3 * axis + k]) * distance + a->m[9 + k] - b->m[9 + k];
            squared += d * d;
        }
        worst = sqrt(squared) > worst ? sqrt(squared) : worst;
    }
    return worst;
}

// The report's largest shell error, its bone and sample, are what the
// test measures itself at every sample.
static void TestReportMatches(void)
{
    BuildFull(0.0f);
    maniCompressOptions options = maniDefaultCompressOptions();
    maniCompressReport report;
    CHECK(Compress(&options, &report) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    Matrix ma[6];
    Matrix mb[6];
    double worst = 0.0;
    uint32_t worstBone = 0;
    uint32_t worstSample = 0;
    for (uint32_t k = 0; k < maniClipSampleCount(&clip); ++k)
    {
        maniSampleClipAt(&s_full, &s_skeleton, k, a, NULL);
        maniSampleClipAt(&clip, &s_skeleton, k, b, NULL);
        PoseMatrices(a, ma);
        PoseMatrices(b, mb);
        for (uint32_t bone = 0; bone < 6; ++bone)
        {
            double e = ShellApart(&ma[bone], &mb[bone], (double)options.shellDistance);
            if (e > worst)
            {
                worst = e;
                worstBone = bone;
                worstSample = k;
            }
        }
    }
    CHECK(fabs(worst - report.shellError) <= 1e-15, "the report's error is the test's");
    CHECK(worstBone == report.shellBone && worstSample == report.shellSample, "and where");
}

// The codec alone: packing at the top of 23 bits (v * max + 0.5 rounds
// past max in binary32), segment ranges around every k / 255 covering
// their values within [0, 1], rotations rebuilt from each dropped
// component, and normalization's guard.
static bool RangeCovers(float low, float high)
{
    uint8_t q = 0;
    uint8_t e = 0;
    maniCodecSegmentRange(low, high, &q, &e);
    float top = maniCodecRangeByte(q) + maniCodecRangeByte(e);
    return maniCodecRangeByte(q) <= low && (top >= high || q + e == 255) && q + e <= 255;
}

static void TestCodec(void)
{
    CHECK(maniCodecPack(1.0f, 23) == 8388607u, "1 on 23 bits");
    CHECK(maniCodecPack(1.0f, 8) == 255u && maniCodecPack(0.0f, 8) == 0u, "the ends");
    CHECK(maniCodecPack(2.0f, 4) == 15u && maniCodecPack(-1.0f, 4) == 0u, "clamped");
    bool covers = true;
    for (int k = 0; k <= 255; ++k)
    {
        float v = (float)k / 255.0f;
        const float near[5] = {nextafterf(nextafterf(v, 0.0f), 0.0f), nextafterf(v, 0.0f), v,
                               nextafterf(v, 1.0f), nextafterf(nextafterf(v, 1.0f), 1.0f)};
        for (int i = 0; i < 5; ++i)
        {
            float low = near[i] < 0.0f ? 0.0f : near[i];
            covers = covers && RangeCovers(low, low) && RangeCovers(low, 1.0f) &&
                     RangeCovers(0.0f, low > 1.0f ? 1.0f : low) &&
                     RangeCovers(low, low + 0.3f > 1.0f ? 1.0f : low + 0.3f);
        }
    }
    CHECK(covers, "segment ranges cover their values");
    // The extent is the smallest byte that covers: a ceiling of the
    // product gives a step more at 0x1.818184p-7 and 125 more values.
    const float tight[3] = {0x1.818184p-7f, 0x1.818184p-6f, 0x1.c1c1c4p-6f};
    bool least = true;
    for (int i = 0; i < 3; ++i)
    {
        uint8_t q = 0;
        uint8_t e = 0;
        maniCodecSegmentRange(0.0f, tight[i], &q, &e);
        least = least && maniCodecRangeByte(e) >= tight[i] &&
                maniCodecRangeByte((uint8_t)(e - 1)) < tight[i];
    }
    CHECK(least, "the smallest extent byte");
    const float kept[3] = {0.1f, 0.2f, 0.3f};
    float r = maniCodecRebuild(0.1f, 0.2f, 0.3f);
    const float want[4][4] = {
        {r, 0.1f, 0.2f, 0.3f}, {0.1f, r, 0.2f, 0.3f}, {0.1f, 0.2f, r, 0.3f}, {0.1f, 0.2f, 0.3f, r}};
    bool rebuilt = true;
    for (uint32_t d = 0; d < 4; ++d)
    {
        float q[4];
        maniCodecRotation(kept, d, q);
        rebuilt = rebuilt && memcmp(q, want[d], sizeof q) == 0;
    }
    CHECK(rebuilt, "each dropped component rebuilt in place");
    float huge[4] = {3e38f, 3e38f, 0.0f, 0.0f};
    maniCodecNormalize(huge);
    CHECK(huge[0] == 0.0f && huge[3] == 1.0f, "past binary32's range: the identity");
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    maniCodecNormalize(zero);
    CHECK(zero[3] == 1.0f, "zero: the identity");
}

// An additive clip compressed: its report is the test's own measure of
// the rest plus each difference, within the precision; channels at the
// rest fold to the identity difference.

static void AdditiveCompressed(float precision)
{
    maniClipSource source = Source(0.0f);
    source.additive = maniAdditiveRest;
    BuildFullFrom(&source);
    maniCompressOptions options = maniDefaultCompressOptions();
    options.precision = precision;
    maniCompressReport report;
    CHECK(Compress(&options, &report) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK(maniClipAdditiveBasis(&clip) == maniAdditiveRest, "keeps the basis");
    static _Alignas(32) uint8_t restMemory[2][8192];
    maniPose* a = NULL;
    maniPose* b = NULL;
    maniPose* ra = NULL;
    maniPose* rb = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    CHECK(maniInitPose(&s_skeleton, restMemory[0], sizeof restMemory[0], &ra) == mani_success,
          "rest a");
    CHECK(maniInitPose(&s_skeleton, restMemory[1], sizeof restMemory[1], &rb) == mani_success,
          "rest b");
    Matrix ma[6];
    Matrix mb[6];
    double worst = 0.0;
    for (uint32_t k = 0; k < maniClipSampleCount(&clip); ++k)
    {
        maniSampleClipAt(&s_full, &s_skeleton, k, a, NULL);
        maniSampleClipAt(&clip, &s_skeleton, k, b, NULL);
        CHECK(maniPoseSetRest(ra, &s_skeleton) == mani_success &&
                  maniPoseSetRest(rb, &s_skeleton) == mani_success,
              "rests");
        maniBlendLayer la = {a, 1.0f, NULL};
        maniBlendLayer lb = {b, 1.0f, NULL};
        CHECK(maniAddPoses(&s_skeleton, ra, &la, 1) == mani_success &&
                  maniAddPoses(&s_skeleton, rb, &lb, 1) == mani_success,
              "adds");
        PoseMatrices(ra, ma);
        PoseMatrices(rb, mb);
        for (uint32_t bone = 0; bone < 6; ++bone)
        {
            double e = ShellApart(&ma[bone], &mb[bone], (double)options.shellDistance);
            worst = e > worst ? e : worst;
        }
    }
    CHECK(worst <= (double)options.precision, "within the precision on the rest");
    CHECK(fabs(worst - report.shellError) <= 1e-12, "the report is the test's measure");
    // Channels without keys decode as the identity difference.
    maniTransform spine = maniPoseLocal(b, 1);
    maniTransform root = maniPoseLocal(b, 0);
    maniTransform hand = maniPoseLocal(b, 4);
    CHECK(spine.translation.x == 0.0f && spine.translation.y == 0.0f && root.scale.x == 1.0f &&
              root.scale.z == 1.0f && hand.rotation.w == 1.0f && hand.rotation.x == 0.0f,
          "no keys: the identity");
}

// At the default precision and a tight one (the dominant shells grow by
// the children's reach on the rest, not by their differences); and a
// channel at its rest folds to the identity difference, the default.
static const float kChestRestT[3] = {0.0f, 0.3f, 0.0f};

static void TestAdditiveCompressed(void)
{
    AdditiveCompressed(0.0001f);
    AdditiveCompressed(0.00001f);
    maniClipSource source = Source(0.0f);
    source.additive = maniAdditiveRest;
    s_tracks[2].translation = Linear(kChestRestT, 1);
    BuildFullFrom(&source);
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK((clip.forms[2] & 3) == maniClipFormDefault, "a translation at its rest: the default");
}

// Per-bone precisions, tight near the root and loose at the leaves: each
// bone within its own, though its dominant shell carries a leaf's.
static void TestBonePrecisions(void)
{
    BuildFull(0.0f);
    const float precisions[6] = {1e-5f, 1e-5f, 1e-5f, 1e-3f, 1e-3f, 1e-3f};
    maniCompressOptions options = maniDefaultCompressOptions();
    options.bonePrecisions = precisions;
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseA, sizeof s_poseA, &a) == mani_success, "pose a");
    CHECK(maniInitPose(&s_skeleton, s_poseB, sizeof s_poseB, &b) == mani_success, "pose b");
    Matrix ma[6];
    Matrix mb[6];
    bool within = true;
    for (uint32_t k = 0; k < maniClipSampleCount(&clip); ++k)
    {
        maniSampleClipAt(&s_full, &s_skeleton, k, a, NULL);
        maniSampleClipAt(&clip, &s_skeleton, k, b, NULL);
        PoseMatrices(a, ma);
        PoseMatrices(b, mb);
        for (uint32_t bone = 0; bone < 6; ++bone)
        {
            double e = ShellApart(&ma[bone], &mb[bone], (double)options.shellDistance);
            within = within && e <= (double)precisions[bone];
        }
    }
    CHECK(within, "every bone within its own precision");
}

static void TestSameBytes(void)
{
    BuildFull(0.0f);
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
    static uint8_t first[65536];
    size_t firstSize = s_size;
    memcpy(first, s_file, s_size);
    CHECK(Compress(&options, NULL) == mani_success, "again");
    CHECK(s_size == firstSize && memcmp(first, s_file, s_size) == 0, "the same bytes");
    // A coarser precision makes a smaller file.
    options.precision = 0.01f;
    CHECK(Compress(&options, NULL) == mani_success, "coarse");
    CHECK(s_size < firstSize, "coarser is smaller");
}

static void TestMisuse(void)
{
    BuildFull(0.0f);
    maniBuilder* builder = NewBuilder();
    maniCompressOptions o = maniDefaultCompressOptions();
    CHECK(maniCompressClip(NULL, &s_full, &s_skeleton, &o, NULL) == mani_errorInvalid, "builder");
    CHECK(maniCompressClip(builder, NULL, &s_skeleton, &o, NULL) == mani_errorInvalid, "clip");
    CHECK(maniCompressClip(builder, &s_full, NULL, &o, NULL) == mani_errorInvalid, "skeleton");
    CHECK(maniCompressClip(builder, &s_full, &s_skeleton, NULL, NULL) == mani_errorInvalid,
          "options");
    const float bad[4] = {0.0f, -1.0f, NAN, INFINITY};
    for (int i = 0; i < 4; ++i)
    {
        maniCompressOptions b = o;
        b.precision = bad[i];
        CHECK(maniCompressClip(builder, &s_full, &s_skeleton, &b, NULL) == mani_errorInvalid,
              "precision");
        b = o;
        b.shellDistance = bad[i];
        CHECK(maniCompressClip(builder, &s_full, &s_skeleton, &b, NULL) == mani_errorInvalid,
              "shell distance");
        b = o;
        b.floatPrecision = bad[i];
        CHECK(maniCompressClip(builder, &s_full, &s_skeleton, &b, NULL) == mani_errorInvalid,
              "float precision");
    }
    float perBone[6] = {1e-4f, 1e-4f, 1e-4f, 1e-4f, 1e-4f, 0.0f};
    float perFloat[4] = {1e-5f, 1e-5f, 1e-5f, NAN};
    maniCompressOptions b = o;
    b.bonePrecisions = perBone;
    CHECK(maniCompressClip(builder, &s_full, &s_skeleton, &b, NULL) == mani_errorInvalid,
          "a bone's precision");
    b = o;
    b.boneShellDistances = perBone;
    CHECK(maniCompressClip(builder, &s_full, &s_skeleton, &b, NULL) == mani_errorInvalid,
          "a bone's shell distance");
    b = o;
    b.floatPrecisions = perFloat;
    CHECK(maniCompressClip(builder, &s_full, &s_skeleton, &b, NULL) == mani_errorInvalid,
          "a float track's precision");
    CHECK(Compress(&o, NULL) == mani_success, "compresses");
    maniClip packed;
    CHECK(Open(&packed).result == mani_success, "opens");
    CHECK(maniCompressClip(builder, &packed, &s_skeleton, &o, NULL) == mani_errorInvalid,
          "a compressed clip");
    maniClip other = s_full;
    other.skeletonIdentity ^= 1;
    CHECK(maniCompressClip(builder, &other, &s_skeleton, &o, NULL) == mani_errorInvalid,
          "another skeleton's clip");
    CHECK(maniBuilderMisuses(builder) == 20, "misuses counted");
    maniDestroyBuilder(builder);
}

static void Rehash(void)
{
    maniStoreU64(s_file + 32, maniHashBytes(MANI_HASH_INIT, s_file + 64, s_size - 64));
}

// A section's table entry by tag (empty ones are left out), or 0.
static uint64_t EntryOf(uint32_t tag)
{
    for (uint32_t i = 0; i < maniLoadU32(s_file + 12); ++i)
    {
        if (maniLoadU32(s_file + 64 + 24 * (uint64_t)i) == tag)
        {
            return 64 + 24 * (uint64_t)i;
        }
    }
    return 0;
}

static uint64_t SectionAt(uint32_t tag)
{
    return maniLoadU64(s_file + EntryOf(tag) + 8);
}

// The clip's identity made right after an edit that should open.
static void Reidentify(void)
{
    const uint8_t* sections[MANI_CLIP_COMPRESSED_SECTIONS];
    uint64_t sizes[MANI_CLIP_COMPRESSED_SECTIONS];
    for (uint32_t i = 0; i < MANI_CLIP_COMPRESSED_SECTIONS; ++i)
    {
        uint64_t entry = EntryOf(i + 1);
        sections[i] = s_file + (entry != 0 ? SectionAt(i + 1) : 0);
        sizes[i] = entry != 0 ? maniLoadU64(s_file + entry + 16) : 0;
    }
    maniStoreU64(s_file + 40,
                 maniClipComputeIdentity(sections, sizes, MANI_CLIP_COMPRESSED_SECTIONS));
}

static void Reset(void)
{
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, NULL) == mani_success, "compresses");
}

static void ExpectFormat(uint64_t offset, const char* what)
{
    Rehash();
    maniClip clip;
    maniFileResult r = Open(&clip);
    CHECK(r.result == mani_errorFormat && r.offset == offset, what);
    if (r.offset != offset)
    {
        printf("  %s: %llu, expected %llu\n", what, (unsigned long long)r.offset,
               (unsigned long long)offset);
    }
    Reset();
}

static void StoreF32(uint64_t at, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(s_file + at, bits);
}

static void TestOpenRefusals(void)
{
    BuildFull(0.0f);
    Reset();
    uint64_t forms = SectionAt(maniClipTagForms);
    uint64_t constants = SectionAt(maniClipTagConstants);
    uint64_t ranges = SectionAt(maniClipTagRanges);
    uint64_t table = SectionAt(maniClipTagSegments);
    uint64_t data = SectionAt(maniClipTagSegmentData);
    // The table: empty sections left out, so a compressed clip has no
    // samples sections; tags past 16 and compressed sections in a
    // full-precision clip refused.
    uint32_t count = maniLoadU32(s_file + 12);
    CHECK(EntryOf(maniClipTagSamples) == 0 && EntryOf(maniClipTagFloatSamples) == 0,
          "no empty sections");
    maniStoreU32(s_file + 64 + 24 * (uint64_t)(count - 1), 17);
    ExpectFormat(64 + 24 * (uint64_t)(count - 1), "a tag past 16");
    maniStoreU32(s_file + SectionAt(maniClipTagHeader) + 56, 0);
    maniStoreU32(s_file + SectionAt(maniClipTagHeader) + 52, 0);
    ExpectFormat(EntryOf(maniClipTagForms) + 16, "compressed sections at full precision");
    s_file[forms + 2] = 0x3f;
    ExpectFormat(forms + 2, "a form for an absent channel");
    s_file[forms + 0] &= 0xf3;
    ExpectFormat(forms + 0, "no form for a present channel");
    s_file[forms + 2] |= 0x40;
    ExpectFormat(forms + 2, "a dropped component for a default rotation");
    s_file[forms + 4] = 0x01 | 0x40;
    ExpectFormat(forms + 4, "a dropped component for no rotation");
    s_file[forms + 6] = maniClipFormDefault;
    ExpectFormat(forms + 6, "a default float track");
    StoreF32(constants, NAN);
    ExpectFormat(constants, "a constant not finite");
    StoreF32(ranges, INFINITY);
    ExpectFormat(ranges, "a minimum not finite");
    StoreF32(ranges + 12, -1.0f);
    ExpectFormat(ranges + 12, "a negative extent");
    StoreF32(ranges + 24 * 6 + 4, -1.0f);
    ExpectFormat(ranges + 24 * 6 + 4, "a float track's negative extent");
    StoreF32(ranges + 24, -1.5f);
    ExpectFormat(ranges + 24, "a rotation's minimum below -1");
    StoreF32(ranges + 24 + 16, 2.5f);
    ExpectFormat(ranges + 24 + 16, "a rotation's extent past 2");
    StoreF32(ranges + 4, 3e38f);
    StoreF32(ranges + 16, 3e38f);
    ExpectFormat(ranges + 16, "a range's top not finite");
    // Root translation's first segment range: minimum and extent bytes.
    s_file[data + 9] = 199;
    s_file[data + 12] = 56;
    Rehash();
    maniClip clip;
    Reidentify();
    Rehash();
    CHECK(Open(&clip).result == mani_success, "a range of 255 in all");
    s_file[data + 12] = 57;
    ExpectFormat(data + 12, "a segment range past 1");
    maniStoreU32(s_file + table, 1);
    ExpectFormat(table, "the first segment not at 0");
    maniStoreU32(s_file + table + 20 + 4, 0);
    ExpectFormat(table + 24, "an empty segment");
    maniStoreU32(s_file + table + 60 + 4, 16);
    ExpectFormat(table + 64, "past the samples");
    maniStoreU32(s_file + table + 40 + 8, 0);
    ExpectFormat(table + 48, "an offset not after the last");
    maniStoreU32(s_file + table + 12, 1);
    ExpectFormat(table + 12, "channel bits");
    maniStoreU32(s_file + table + 16, 1);
    ExpectFormat(table + 16, "float bits");
    s_file[data] = 25;
    ExpectFormat(table + 8, "a rate past the table");
}

// The compressed file rebuilt with one section grown by `extra` zero
// bytes, or with an empty section of an absent tag put in the table; the
// identity and the payload hash made right.
static _Alignas(16) uint8_t s_old[65536];
static uint8_t s_grown[65536];

static void Rebuild(uint32_t tag, uint32_t extra)
{
    memcpy(s_old, s_file, s_size);
    maniSectionSource table[MANI_CLIP_COMPRESSED_SECTIONS];
    const uint8_t* sections[MANI_CLIP_COMPRESSED_SECTIONS];
    uint64_t sizes[MANI_CLIP_COMPRESSED_SECTIONS];
    uint32_t count = 0;
    for (uint32_t t = 1; t <= MANI_CLIP_COMPRESSED_SECTIONS; ++t)
    {
        uint64_t entry = EntryOf(t);
        const uint8_t* data = entry != 0 ? s_old + maniLoadU64(s_old + entry + 8) : s_old;
        uint64_t size = entry != 0 ? maniLoadU64(s_old + entry + 16) : 0;
        if (t == tag && size > 0)
        {
            memset(s_grown, 0, sizeof s_grown);
            memcpy(s_grown, data, (size_t)size);
            data = s_grown;
            size += extra;
        }
        sections[t - 1] = data;
        sizes[t - 1] = size;
        if (size > 0 || t == tag)
        {
            table[count++] = (maniSectionSource){t, data, size};
        }
    }
    uint64_t fileSize = 0;
    CHECK(maniContainerSize(table, count, &fileSize) && fileSize <= sizeof s_file, "rebuilt");
    maniWriteContainer(s_file, maniKindClip, MANI_CLIP_FORMAT_VERSION,
                       maniClipComputeIdentity(sections, sizes, MANI_CLIP_COMPRESSED_SECTIONS),
                       table, count);
    s_size = (size_t)fileSize;
}

// Sections larger than their counts make them, a present empty section,
// the last segment a sample short, a raw value NaN.
static void TestTableRefusals(void)
{
    BuildFull(0.0f);
    Reset();
    const uint32_t grown[3] = {maniClipTagForms, maniClipTagRanges, maniClipTagSegmentData};
    for (uint32_t i = 0; i < 3; ++i)
    {
        Rebuild(grown[i], 16);
        ExpectFormat(EntryOf(grown[i]) + 16, "a section past its size");
    }
    Rebuild(maniClipTagSamples, 0);
    ExpectFormat(EntryOf(maniClipTagSamples) + 16, "an empty section in the table");
    uint64_t table = SectionAt(maniClipTagSegments);
    uint64_t last = table + 20 * 3;
    maniStoreU32(s_file + last + 4, maniLoadU32(s_file + last + 4) - 1);
    ExpectFormat(table, "the segments a sample short");
    // The first raw sub-track's first component at segment 0, sample 0.
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    uint32_t n = clip.animatedChannels + clip.animatedFloats;
    uint64_t data = SectionAt(maniClipTagSegmentData);
    const uint8_t* rates = s_file + data + maniLoadU32(s_file + table + 8);
    uint64_t head = n + 6 * (uint64_t)clip.animatedChannels + 2 * (uint64_t)clip.animatedFloats;
    uint64_t bit = 0;
    uint32_t j = 0;
    while (j < n && rates[j] != 24)
    {
        bit += (j < clip.animatedChannels ? 3u : 1u) * (rates[j] == 0 ? 0u : rates[j]);
        ++j;
    }
    CHECK(j < n, "a raw sub-track");
    uint8_t* stream = s_file + data + maniLoadU32(s_file + table + 8) + head;
    for (uint32_t b = 0; b < 32; ++b)
    {
        uint64_t at = bit + b;
        uint8_t mask = (uint8_t)(1u << (at & 7u));
        bool one = ((0x7FC00000u >> b) & 1u) != 0;
        stream[at >> 3] =
            one ? (uint8_t)(stream[at >> 3] | mask) : (uint8_t)(stream[at >> 3] & ~mask);
    }
    ExpectFormat((uint64_t)(stream - s_file) + bit / 8, "a raw value NaN");
}

// A rotation constant over the clip, stored and bounded.
static void TestConstantRotation(void)
{
    static float turn[4];
    StoreAxisAngle(turn, 0.0f, 1.0f, 0.0f, 0.3f);
    maniClipSource source = Source(0.0f);
    s_tracks[2].rotation = Linear(turn, 1);
    BuildFullFrom(&source);
    maniCompressReport report;
    maniCompressOptions options = maniDefaultCompressOptions();
    CHECK(Compress(&options, &report) == mani_success && report.shellError <= 0.0001, "compresses");
    maniClip clip;
    CHECK(Open(&clip).result == mani_success, "opens");
    CHECK(((clip.forms[2] >> 2) & 3) == maniClipFormConstant, "the chest's rotation constant");
    // The chest's constants: its translation, then its rotation's kept three.
    uint64_t constants = SectionAt(maniClipTagConstants);
    StoreF32(constants + 12, 1.5f);
    Rehash();
    maniFileResult r = Open(&clip);
    CHECK(r.result == mani_errorFormat && r.offset == constants + 12, "a rotation constant past 1");
}

static void TestRateZeroAlone(void)
{
    // One segment has no segment ranges, so rate 0 is refused there.
    BuildFull(0.5f);
    Reset();
    s_file[SectionAt(maniClipTagSegmentData)] = 0;
    ExpectFormat(SectionAt(maniClipTagSegments) + 8, "rate 0 in one segment");
}

static void TestEveryByteFlip(void)
{
    BuildFull(0.5f);
    Reset();
    maniClip clip;
    for (size_t i = 0; i < s_size; ++i)
    {
        bool isFree = i >= 16 && i < 22;
        s_file[i] ^= 0x20;
        bool opened = Open(&clip).result == mani_success;
        s_file[i] ^= 0x20;
        CHECK(opened == isFree, "byte flip");
    }
}

static void TestPinnedBytes(void)
{
    BuildFull(0.0f);
    Reset();
    memset(s_file + 16, 0, 6);
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_COMPRESSED_HASH, "pinned file");
    if (hash != PINNED_COMPRESSED_HASH)
    {
        printf("compressed hash 0x%016llx\n", (unsigned long long)hash);
    }
}

static int WriteSeed(const char* path)
{
    BuildFull(0.0f);
    Reset();
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(s_file, 1, s_size, out);
    return fclose(out) == 0 && written == s_size ? 0 : 1;
}

int main(int argc, char** argv)
{
    FillKeys();
    OpenSkeleton();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestSegments();
    TestOneSegment();
    TestOneSample();
    TestFoldWithinPrecision();
    TestNoRatePasses();
    TestRandomWithinPrecision();
    TestCodec();
    TestReportMatches();
    TestBonePrecisions();
    TestAdditiveCompressed();
    TestCursor();
    TestCursorRandom();
    TestBatchTails();
    TestRestFill();
    TestSubsets();
    TestSameBytes();
    TestMisuse();
    TestOpenRefusals();
    TestRateZeroAlone();
    TestTableRefusals();
    TestConstantRotation();
    TestEveryByteFlip();
    TestPinnedBytes();
    return s_failures == 0 ? 0 : 1;
}
