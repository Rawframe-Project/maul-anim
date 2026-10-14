// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Clips: building from glTF's interpolations, opening, sampling (key
// values at key times, Appendix C's formulas at sample times, the rest
// where nothing is animated, clamped times), every refusal of the builder
// and of opening, and the bytes pinned. Run as test_clip --seed <file> it
// writes the fuzz target's seed.

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

#define PINNED_CLIP_HASH 0x42a8f119163cb13cull

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_file[65536];
static size_t s_size;
static _Alignas(32) uint8_t s_poseMemory[4096];
static maniSkeleton s_skeleton;

#define BONE(n, p)                                                                                 \
    {                                                                                              \
        n, sizeof(n) - 1, p,                                                                       \
        {                                                                                          \
            {0.5f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}                       \
        }                                                                                          \
    }

static const maniBoneSource kBones[4] = {BONE("root", -1), BONE("a", 0), BONE("b", 1),
                                         BONE("c", 0)};

// Bone 1: translation, linear, keys at 0, 0.5 and 1.
static const float kLinearTimes[3] = {0.0f, 0.5f, 1.0f};
static const float kLinearValues[9] = {0, 0, 0, 1, 2, 3, 3, 2, 1};
// Bone 2: rotation about z from 0 to 90 degrees, linear (a slerp).
static const float kTurnTimes[2] = {0.0f, 1.0f};
static const float kTurnValues[8] = {0, 0, 0, 1, 0, 0, 0.70710677f, 0.70710677f};
// Bone 2: scale, a cubic spline: in-tangent, value, out-tangent per key.
static const float kCubicTimes[2] = {0.0f, 1.0f};
static const float kCubicValues[18] = {0, 0, 0, 1, 1, 1, 2, 0, -1, 0, 1, 0, 2, 2, 2, 0, 0, 0};
// Bone 0: translation, step.
static const float kStepTimes[2] = {0.0f, 0.55f};
static const float kStepValues[6] = {0, 0, 0, 0, 5, 0};

static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};

static maniBoneTrackSource s_tracks[3];

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
    maniSkeletonSource source = {kBones, 4, 0};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "skeleton builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
          "copies");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
}

static maniClipSource SampleSource(void)
{
    // Listed out of bone order: the build sorts them.
    s_tracks[0] = (maniBoneTrackSource){
        2, 0, kNone, (maniChannelSource){kTurnTimes, kTurnValues, 2, maniInterpolationLinear},
        (maniChannelSource){kCubicTimes, kCubicValues, 2, maniInterpolationCubicSpline}};
    s_tracks[1] = (maniBoneTrackSource){
        1, 0, (maniChannelSource){kLinearTimes, kLinearValues, 3, maniInterpolationLinear}, kNone,
        kNone};
    s_tracks[2] = (maniBoneTrackSource){
        0, 0, (maniChannelSource){kStepTimes, kStepValues, 2, maniInterpolationStep}, kNone, kNone};
    return (maniClipSource){&s_skeleton, s_tracks, 3, maniLoopRepeat, 10.0f, 0.0f};
}

static maniBuildResult BuildFrom(const maniClipSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildClip(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return r;
}

static void BuildSample(void)
{
    OpenSkeleton();
    maniClipSource source = SampleSource();
    CHECK(BuildFrom(&source).result == mani_success, "clip builds");
}

static maniFileResult Open(void)
{
    maniClip clip;
    return maniOpenClip(s_file, s_size, &clip);
}

static maniPose* SampleAt(const maniClip* clip, float time)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    CHECK(maniSampleClip(clip, &s_skeleton, time, pose) == mani_success, "samples");
    return pose;
}

static bool Near(float a, double b, double tolerance)
{
    return fabs((double)a - b) <= tolerance;
}

static void TestHeader(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipDuration(&clip) == 1.0f, "duration from the last key");
    CHECK(maniClipSampleRate(&clip) == 10.0f, "rate");
    CHECK(maniClipSampleCount(&clip) == 11, "samples");
    CHECK(maniClipLoopPolicy(&clip) == maniLoopRepeat, "loop");
    CHECK(maniClipSkeletonIdentity(&clip) == maniSkeletonIdentity(&s_skeleton), "bound");
}

static void TestLinearAndRest(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    for (int k = 0; k <= 20; ++k)
    {
        float time = 0.05f * (float)k;
        maniPose* pose = SampleAt(&clip, time);
        maniTransform a = maniPoseLocal(pose, 1);
        // Piecewise linear source on a grid that contains its keys: exact
        // up to rounding everywhere.
        double t = (double)time;
        double x = t <= 0.5 ? 2.0 * t : 1.0 + 4.0 * (t - 0.5);
        double z = t <= 0.5 ? 6.0 * t : 3.0 - 4.0 * (t - 0.5);
        CHECK(Near(a.translation.x, x, 2e-6) && Near(a.translation.z, z, 2e-6), "linear");
        CHECK(a.rotation.w == 1.0f && a.scale.x == 1.0f, "rest channels");
        maniTransform c = maniPoseLocal(pose, 3);
        CHECK(c.translation.x == 0.5f && c.rotation.w == 1.0f, "an unanimated bone at rest");
    }
}

static void TestSlerpAtSamples(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    double worstMid = 0.0;
    for (int k = 0; k <= 10; ++k)
    {
        // At sample times, glTF's slerp: a turn of 90 t degrees about z.
        float time = (float)k / 10.0f;
        maniTransform r = maniPoseLocal(SampleAt(&clip, time), 2);
        double half = 0.25 * 3.14159265358979323846 * (double)time;
        CHECK(Near(r.rotation.z, sin(half), 2e-7) && Near(r.rotation.w, cos(half), 2e-7),
              "slerp at a sample");
        // Between samples, normalized lerp: close to the slerp.
        float mid = time + 0.05f;
        if (k < 10)
        {
            maniTransform m = maniPoseLocal(SampleAt(&clip, mid), 2);
            double h = 0.25 * 3.14159265358979323846 * (double)mid;
            worstMid = fmax(worstMid, fabs((double)m.rotation.z - sin(h)));
        }
    }
    CHECK(worstMid < 1e-4, "nlerp between samples near the slerp");
}

static void TestCubicAtSamples(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    for (int k = 0; k <= 10; ++k)
    {
        float time = (float)k / 10.0f;
        maniTransform r = maniPoseLocal(SampleAt(&clip, time), 2);
        double u = (double)time;
        double h00 = 2 * u * u * u - 3 * u * u + 1;
        double h10 = u * u * u - 2 * u * u + u;
        double h01 = -2 * u * u * u + 3 * u * u;
        double h11 = u * u * u - u * u;
        // v0 (1, 1, 1), out-tangent (2, 0, -1); v1 (2, 2, 2), in-tangent
        // (0, 1, 0); the span is 1.
        double x = h00 * 1 + h10 * 2 + h01 * 2 + h11 * 0;
        double y = h00 * 1 + h10 * 0 + h01 * 2 + h11 * 1;
        double z = h00 * 1 + h10 * -1 + h01 * 2 + h11 * 0;
        CHECK(Near(r.scale.x, x, 1e-6) && Near(r.scale.y, y, 1e-6) && Near(r.scale.z, z, 1e-6),
              "cubic at a sample");
    }
}

static void TestStepAndClamp(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    // The step key at 0.55 lands between the samples at 0.5 and 0.6:
    // sample 0.5 holds 0, sample 0.6 holds 5.
    CHECK(maniPoseLocal(SampleAt(&clip, 0.5f), 0).translation.y == 0.0f, "before the step");
    CHECK(maniPoseLocal(SampleAt(&clip, 0.6f), 0).translation.y == 5.0f, "after the step");
    maniTransform before = maniPoseLocal(SampleAt(&clip, 0.0f), 1);
    maniTransform early = maniPoseLocal(SampleAt(&clip, -3.0f), 1);
    maniTransform nan = maniPoseLocal(SampleAt(&clip, NAN), 1);
    CHECK(memcmp(&before, &early, sizeof before) == 0, "clamped below");
    CHECK(memcmp(&before, &nan, sizeof before) == 0, "NaN is 0");
    maniTransform end = maniPoseLocal(SampleAt(&clip, 1.0f), 1);
    maniTransform late = maniPoseLocal(SampleAt(&clip, 9.0f), 1);
    CHECK(memcmp(&end, &late, sizeof end) == 0, "clamped above");
    CHECK(end.translation.x == 3.0f && end.translation.z == 1.0f, "the last key");
}

static void TestDurationAndRate(void)
{
    OpenSkeleton();
    maniClipSource source = SampleSource();
    source.duration = 0.95f;
    source.sampleRate = 0.0f;
    CHECK(BuildFrom(&source).result == mani_success, "builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    // 30 per second over 0.95 s: 28.5 intervals, so 29, and 30 samples,
    // the last at 0.95.
    CHECK(maniClipSampleRate(&clip) == 30.0f && maniClipSampleCount(&clip) == 30, "default rate");
    maniTransform last = maniPoseLocal(SampleAt(&clip, 0.95f), 1);
    CHECK(Near(last.translation.x, 1.0 + 4.0 * 0.45, 2e-6), "the last sample at the duration");

    source.boneTrackCount = 0;
    source.duration = 0.0f;
    CHECK(BuildFrom(&source).result == mani_success, "no tracks");
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipSampleCount(&clip) == 1 && maniClipDuration(&clip) == 0.0f, "one sample");
    maniTransform rest = maniPoseLocal(SampleAt(&clip, 0.3f), 1);
    CHECK(rest.translation.x == 0.5f, "every bone at rest");
}

static void TestWrongSkeleton(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    static _Alignas(16) uint8_t other[4096];
    maniBoneSource moved[4];
    memcpy(moved, kBones, sizeof moved);
    moved[3].rest.translation.y = 1.0f;
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {moved, 4, 0};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, other, sizeof other, &size) == mani_success, "copies");
    maniDestroyBuilder(builder);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(other, size, &skeleton).result == mani_success, "opens");
    static _Alignas(32) uint8_t memory[4096];
    maniPose* pose = NULL;
    CHECK(maniInitPose(&skeleton, memory, sizeof memory, &pose) == mani_success, "pose");
    CHECK(maniSampleClip(&clip, &skeleton, 0.5f, pose) == mani_errorInvalid, "another skeleton");
    CHECK(maniSampleClip(NULL, &skeleton, 0.5f, pose) == mani_errorInvalid, "null clip");
}

static void ExpectRefusal(const maniClipSource* source, maniBuildReason reason, uint32_t item,
                          const char* what)
{
    maniBuildResult r = BuildFrom(source);
    CHECK(r.result == mani_errorInvalid && r.reason == reason && r.item == item, what);
}

static void TestBuildRefusals(void)
{
    OpenSkeleton();
    maniClipSource source = SampleSource();
    source.loop = 2;
    ExpectRefusal(&source, maniBuildReasonKind, 0, "loop policy");
    source = SampleSource();
    source.sampleRate = -1.0f;
    ExpectRefusal(&source, maniBuildReasonTiming, 0, "rate");
    source = SampleSource();
    source.duration = INFINITY;
    ExpectRefusal(&source, maniBuildReasonTiming, 0, "duration");
    source = SampleSource();
    source.duration = 2e6f;
    ExpectRefusal(&source, maniBuildReasonTiming, 0, "too many samples");
    source = SampleSource();
    s_tracks[1].bone = 9;
    ExpectRefusal(&source, maniBuildReasonTrack, 1, "a bone past the skeleton");
    source = SampleSource();
    s_tracks[1].bone = 2;
    maniBuildResult r = BuildFrom(&source);
    CHECK(r.reason == maniBuildReasonTrack && r.item == 1 && r.other == 0, "a bone twice");
    source = SampleSource();
    s_tracks[0].rotation.interpolation = 7;
    ExpectRefusal(&source, maniBuildReasonKind, 0, "interpolation");
    source = SampleSource();
    static const float backwards[3] = {0.0f, 0.5f, 0.5f};
    s_tracks[1].translation.times = backwards;
    ExpectRefusal(&source, maniBuildReasonKeyTime, 1, "times not increasing");
    source = SampleSource();
    static const float negative[2] = {-0.5f, 1.0f};
    s_tracks[0].rotation.times = negative;
    ExpectRefusal(&source, maniBuildReasonKeyTime, 0, "a time before 0");
    source = SampleSource();
    s_tracks[0].scale.keyCount = 1;
    ExpectRefusal(&source, maniBuildReasonKeyCount, 0, "one cubic key");
    source = SampleSource();
    static const float nanValues[9] = {0, 0, 0, NAN, 2, 3, 3, 2, 1};
    s_tracks[1].translation.values = nanValues;
    ExpectRefusal(&source, maniBuildReasonNotFinite, 1, "a value not finite");
    source = SampleSource();
    static const float zeroTurn[8] = {0, 0, 0, 1, 0, 0, 0, 0};
    s_tracks[0].rotation.values = zeroTurn;
    ExpectRefusal(&source, maniBuildReasonRotation, 0, "a zero rotation");
    source = SampleSource();
    s_tracks[1].translation.values = NULL;
    ExpectRefusal(&source, maniBuildReasonCount, 1, "keys without values");

    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildClip(builder, NULL).result == mani_errorInvalid, "null source");
    CHECK(maniBuilderMisuses(builder) == 1, "a misuse");
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

static void ExpectFormat(uint64_t offset, const char* what)
{
    Rehash();
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == offset, what);
    BuildSample();
}

static void StoreF32(uint64_t at, float value)
{
    memcpy(s_file + at, &value, sizeof value);
}

static void TestOpenRefusals(void)
{
    BuildSample();
    uint64_t header = SectionAt(maniClipTagHeader);
    uint64_t tracks = SectionAt(maniClipTagTracks);
    uint64_t samples = SectionAt(maniClipTagSamples);
    maniStoreU32(s_file + header + 8, 0);
    ExpectFormat(header + 8, "no bones");
    StoreF32(header + 16, 0.0f);
    ExpectFormat(header + 16, "rate zero");
    StoreF32(header + 16, NAN);
    ExpectFormat(header + 16, "rate NaN");
    StoreF32(header + 20, -1.0f);
    ExpectFormat(header + 20, "duration below 0");
    maniStoreU32(s_file + header + 12, 12);
    ExpectFormat(header + 12, "sample count not the duration's");
    maniStoreU32(s_file + header + 24, 2);
    ExpectFormat(header + 24, "loop policy");
    s_file[header + 40] = 1;
    ExpectFormat(header + 40, "header padding");
    maniStoreU32(s_file + header + 32, 7);
    ExpectFormat(header + 32, "floats per sample");
    // Tracks are bones 0, 1, 2: make the second bone 0.
    maniStoreU16(s_file + tracks + 4, 0);
    ExpectFormat(tracks + 4, "tracks not increasing");
    maniStoreU16(s_file + tracks + 8, 4);
    ExpectFormat(tracks + 8, "a bone past the skeleton");
    s_file[tracks + 2] = 0;
    ExpectFormat(tracks + 2, "an empty mask");
    s_file[tracks + 6] = 8;
    ExpectFormat(tracks + 6, "a mask past the channels");
    s_file[tracks + 3] = 1;
    ExpectFormat(tracks + 3, "track padding");
    StoreF32(samples + 4, INFINITY);
    ExpectFormat(samples + 4, "a sample not finite");
    // Track 2 (bone 2) after 3 + 3 floats: its rotation's x.
    StoreF32(samples + 24, 0.5f);
    ExpectFormat(samples + 24, "a rotation not unit");
    s_file[43] ^= 1;
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == 40, "identity");
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, NULL).result == mani_errorInvalid, "null view");
    CHECK(maniOpenClip(s_skeletonFile, sizeof s_skeletonFile, &clip).result == mani_errorFormat,
          "a skeleton is not a clip");
}

static void TestEveryByteFlip(void)
{
    BuildSample();
    for (size_t i = 0; i < s_size; ++i)
    {
        bool isFree = i >= 16 && i < 22;
        s_file[i] ^= 0x20;
        bool opened = Open().result == mani_success;
        s_file[i] ^= 0x20;
        CHECK(opened == isFree, "byte flip");
    }
}

static void TestPinnedBytes(void)
{
    BuildSample();
    memset(s_file + 16, 0, 6);
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_CLIP_HASH, "pinned file");
    if (hash != PINNED_CLIP_HASH)
    {
        printf("clip hash 0x%016llx\n", (unsigned long long)hash);
    }
}

static int WriteSeed(const char* path)
{
    BuildSample();
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
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestHeader();
    TestLinearAndRest();
    TestSlerpAtSamples();
    TestCubicAtSamples();
    TestStepAndClamp();
    TestDurationAndRate();
    TestWrongSkeleton();
    TestBuildRefusals();
    TestOpenRefusals();
    TestEveryByteFlip();
    TestPinnedBytes();
    return s_failures == 0 ? 0 : 1;
}
