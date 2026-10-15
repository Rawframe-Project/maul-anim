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
#include "clip_file.h"
#include "container.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PINNED_COMPRESSED_HASH 0xdf52fd43a5865a20ull

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
// the leg turning past half a turn, so w changes sign.
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
        StoreAxisAngle(&s_legR[4 * k], 1.0f, 0.0f, 0.0f, 3.4f * t);
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

static void TestOneSegment(void)
{
    BuildFull(0.5f);
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
#define RANDOM_BONES 12
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
        uint32_t bones = 2 + (uint32_t)(Random() * (RANDOM_BONES - 2));
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

static uint64_t SectionAt(uint32_t tag)
{
    return maniLoadU64(s_file + 64 + (uint64_t)(tag - 1) * 24 + 8);
}

// The clip's identity made right after an edit that should open.
static void Reidentify(void)
{
    const uint8_t* sections[MANI_CLIP_COMPRESSED_SECTIONS];
    uint64_t sizes[MANI_CLIP_COMPRESSED_SECTIONS];
    for (uint32_t i = 0; i < MANI_CLIP_COMPRESSED_SECTIONS; ++i)
    {
        sections[i] = s_file + SectionAt(i + 1);
        sizes[i] = maniLoadU64(s_file + 64 + (uint64_t)i * 24 + 16);
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
    s_file[forms + 2] = 0x3f;
    ExpectFormat(forms + 2, "a form for an absent channel");
    s_file[forms + 0] &= 0xf3;
    ExpectFormat(forms + 0, "no form for a present channel");
    s_file[forms + 0] |= 0x40;
    ExpectFormat(forms + 0, "high bits");
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
    TestSameBytes();
    TestMisuse();
    TestOpenRefusals();
    TestRateZeroAlone();
    TestEveryByteFlip();
    TestPinnedBytes();
    return s_failures == 0 ? 0 : 1;
}
