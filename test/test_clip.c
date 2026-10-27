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
#include "file_tag.h"
#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// The clip scene: the sample clip with root motion, sampled at times
// across its loop (local poses, float tracks, root motion), and its file;
// every platform and backend must give this value.
#define PINNED_SCENE_HASH 0xe8ae210836ba1cedull

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

// Float tracks: a linear ramp, a step and a cubic spline.
static const float kRampTimes[2] = {0.0f, 1.0f};
static const float kRampValues[2] = {0.0f, 2.0f};
static const float kGateTimes[2] = {0.0f, 0.3f};
static const float kGateValues[2] = {1.0f, 0.0f};
static const float kWaveTimes[2] = {0.0f, 1.0f};
static const float kWaveValues[6] = {0, 0, 4, 0, 1, 0};
static const maniFloatTrackSource s_floats[3] = {
    {"ramp", 4, {kRampTimes, kRampValues, 2, maniInterpolationLinear}},
    {"gate", 4, {kGateTimes, kGateValues, 2, maniInterpolationStep}},
    {"wave", 4, {kWaveTimes, kWaveValues, 2, maniInterpolationCubicSpline}},
};
// Discrete tracks.
static const float kFootTimes[3] = {0.0f, 0.4f, 0.8f};
static const int32_t kFootValues[3] = {1, 2, 1};
static const float kStateTimes[1] = {0.25f};
static const int32_t kStateValues[1] = {7};
static const maniDiscreteTrackSource s_discrete[2] = {
    {"foot", 4, 3, kFootTimes, kFootValues},
    {"state", 5, 1, kStateTimes, kStateValues},
};
// Events out of time order, two at one time; markers.
static const maniEventSource s_events[4] = {
    {0.5f, 11, "step", 4},
    {0.2f, 12, "breath", 6},
    {0.5f, 13, "sound", 5},
    {1.0f, 14, "end", 3},
};
static const maniEventSource s_markers[2] = {
    {0.4f, 99, "right", 5},
    {0.0f, 0, "left", 4},
};

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
    maniSkeletonSource source = {kBones, 4};
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
        2, kNone, (maniChannelSource){kTurnTimes, kTurnValues, 2, maniInterpolationLinear},
        (maniChannelSource){kCubicTimes, kCubicValues, 2, maniInterpolationCubicSpline}};
    s_tracks[1] = (maniBoneTrackSource){
        1, (maniChannelSource){kLinearTimes, kLinearValues, 3, maniInterpolationLinear}, kNone,
        kNone};
    s_tracks[2] = (maniBoneTrackSource){
        0, (maniChannelSource){kStepTimes, kStepValues, 2, maniInterpolationStep}, kNone, kNone};
    return (maniClipSource){.skeleton = &s_skeleton,
                            .boneTracks = s_tracks,
                            .floatTracks = s_floats,
                            .discreteTracks = s_discrete,
                            .events = s_events,
                            .markers = s_markers,
                            .boneTrackCount = 3,
                            .floatTrackCount = 3,
                            .discreteTrackCount = 2,
                            .eventCount = 4,
                            .markerCount = 2,
                            .loop = maniLoopRepeat,
                            .sampleRate = 10.0f,
                            .duration = 0.0f};
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

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniClip clip;
    return maniOpenClip(bytes, size, &clip);
}

static void TestHeader(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(TestOtherVersionsRefused(s_file, s_size, OpenFile), "another format version refused");
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

// floor(t * rate) can round up past t: at 25 per second 3.87999988f
// lands on sample 97 at 3.88000011f, a factor just below 0 that must not
// extrapolate.
static void TestFactorClamped(void)
{
    OpenSkeleton();
    static const float times[2] = {0.0f, 4.0f};
    static const float values[6] = {0, 0, 0, 100, 0, 0};
    maniBoneTrackSource track = {1, (maniChannelSource){times, values, 2, maniInterpolationLinear},
                                 kNone, kNone};
    maniClipSource source = {
        .skeleton = &s_skeleton, .boneTracks = &track, .boneTrackCount = 1, .sampleRate = 25.0f};
    CHECK(BuildFrom(&source).result == mani_success, "builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    float at = maniPoseLocal(SampleAt(&clip, 3.88f), 1).translation.x;
    float before = maniPoseLocal(SampleAt(&clip, 3.87999988f), 1).translation.x;
    CHECK(before == at, "no extrapolation below a sample");
}

static void TestDurationAndRate(void)
{
    OpenSkeleton();
    maniClipSource source = SampleSource();
    // Only the bone tracks: the named items reach 1 s.
    source.floatTrackCount = 0;
    source.discreteTrackCount = 0;
    source.eventCount = 0;
    source.markerCount = 0;
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

static void TestFloatTracks(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipFloatTrackCount(&clip) == 3, "three float tracks");
    uint32_t ramp = maniClipFindFloatTrack(&clip, "ramp", 4);
    uint32_t gate = maniClipFindFloatTrack(&clip, "gate", 4);
    uint32_t wave = maniClipFindFloatTrack(&clip, "wave", 4);
    CHECK(ramp != MANI_NO_TRACK && gate != MANI_NO_TRACK && wave != MANI_NO_TRACK, "found");
    CHECK(maniClipFindFloatTrack(&clip, "foot", 4) == MANI_NO_TRACK, "not a float track");
    uint32_t length = 0;
    const char* name = maniClipFloatTrackName(&clip, gate, &length);
    CHECK(length == 4 && memcmp(name, "gate", 4) == 0, "its name");
    CHECK(maniClipFloatTrackName(&clip, 3, &length) == NULL && length == 0, "name past");
    float values[3];
    uint32_t count = 0;
    for (int k = 0; k <= 20; ++k)
    {
        float time = 0.05f * (float)k;
        CHECK(maniSampleClipFloats(&clip, time, values, 3, &count) == mani_success && count == 3,
              "samples");
        CHECK(Near(values[ramp], 2.0 * (double)time, 2e-6), "linear");
        // The step key at 0.3f, a float just past 0.3, lands on the first
        // sample at or after it, 0.4: before it 1, then 0.
        CHECK(values[gate] == (time < 0.39f ? 1.0f : 0.0f), "step held");
    }
    // The spline at its samples: out-tangent 4 from 0, toward 1 with no
    // in-tangent.
    for (int k = 0; k <= 10; ++k)
    {
        float time = (float)k / 10.0f;
        CHECK(maniSampleClipFloats(&clip, time, values, 3, NULL) == mani_success, "samples");
        double u = (double)time;
        double expected = (u * u * u - 2 * u * u + u) * 4 + (-2 * u * u * u + 3 * u * u);
        CHECK(Near(values[wave], expected, 1e-6), "cubic");
    }
    CHECK(maniSampleClipFloats(&clip, 0.5f, values, 2, &count) == mani_errorCapacity && count == 3,
          "short array");
    CHECK(maniSampleClipFloats(&clip, 0.5f, NULL, 1, NULL) == mani_errorInvalid, "null array");
}

static void TestDiscreteTracks(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipDiscreteTrackCount(&clip) == 2, "two discrete tracks");
    uint32_t foot = maniClipFindDiscreteTrack(&clip, "foot", 4);
    uint32_t state = maniClipFindDiscreteTrack(&clip, "state", 5);
    CHECK(foot != MANI_NO_TRACK && state != MANI_NO_TRACK, "found");
    CHECK(maniClipDiscreteValue(&clip, foot, 0.0f) == 1, "first key");
    CHECK(maniClipDiscreteValue(&clip, foot, 0.39f) == 1, "held");
    CHECK(maniClipDiscreteValue(&clip, foot, 0.4f) == 2, "at a key");
    CHECK(maniClipDiscreteValue(&clip, foot, 0.9f) == 1, "after the last");
    CHECK(maniClipDiscreteValue(&clip, state, 0.1f) == 7, "before the first key, the first");
    CHECK(maniClipDiscreteValue(&clip, foot, NAN) == 1, "NaN is 0");
    CHECK(maniClipDiscreteValue(&clip, 2, 0.5f) == 0, "a track past the count");
    uint32_t length = 0;
    CHECK(maniClipDiscreteTrackName(&clip, state, &length) != NULL && length == 5, "its name");
}

static void TestEventsAndMarkers(void)
{
    BuildSample();
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipEventCount(&clip) == 4 && maniClipMarkerCount(&clip) == 2, "counts");
    // By time, the tie at 0.5 in source order: breath, step, sound, end.
    static const int32_t payloads[4] = {12, 11, 13, 14};
    for (uint32_t i = 0; i < 4; ++i)
    {
        CHECK(maniClipEventAt(&clip, i).payload == payloads[i], "in time order");
    }
    maniClipEvent step = maniClipEventAt(&clip, 1);
    CHECK(step.time == 0.5f && step.nameLength == 4 && memcmp(step.name, "step", 4) == 0,
          "an event's fields");
    CHECK(maniClipEventAt(&clip, 4).name == NULL, "an event past the count");
    uint32_t first = 0;
    uint32_t count = 0;
    maniClipEventRange(&clip, 0.2f, 0.5f, &first, &count);
    CHECK(first == 0 && count == 1, "forward includes from, not to");
    maniClipEventRange(&clip, 0.5f, 1.0f, &first, &count);
    CHECK(first == 1 && count == 2, "forward from the tie");
    maniClipEventRange(&clip, 1.0f, 0.5f, &first, &count);
    CHECK(first == 3 && count == 1, "backward includes from, not to");
    maniClipEventRange(&clip, 0.5f, 0.2f, &first, &count);
    CHECK(first == 1 && count == 2, "backward over the tie");
    maniClipEventRange(&clip, 0.5f, 0.5f, &first, &count);
    CHECK(count == 0, "no move, no events");
    maniClipEventRange(&clip, NAN, 0.5f, &first, &count);
    CHECK(count == 0, "NaN, no events");
    maniClipEvent left = maniClipMarkerAt(&clip, 0);
    maniClipEvent right = maniClipMarkerAt(&clip, 1);
    CHECK(left.time == 0.0f && memcmp(left.name, "left", 4) == 0, "markers by time");
    CHECK(right.time == 0.4f && right.payload == 0, "a marker's payload is 0");
}

// Durations of whole frames written in decimal: no near-duplicate last
// sample at any common rate.
static void TestWholeFrameDurations(void)
{
    const float rates[] = {24.0f, 25.0f, 30.0f, 48.0f, 50.0f, 60.0f, 90.0f, 120.0f, 29.97f};
    uint32_t wrong = 0;
    for (size_t r = 0; r < sizeof rates / sizeof rates[0]; ++r)
    {
        for (uint32_t k = 1; k <= 20000; ++k)
        {
            float duration = (float)((double)k / (double)rates[r]);
            uint32_t count = 0;
            if (!maniClipSampleCountFor(duration, rates[r], &count) || count != k + 1)
            {
                ++wrong;
            }
        }
    }
    CHECK(wrong == 0, "whole frames, whole sample counts");
    uint32_t count = 0;
    CHECK(maniClipSampleCountFor(1.0f / 3.0f, 30.0f, &count) && count == 11, "1/3 s at 30");
    CHECK(maniClipSampleCountFor(0.1f, 60.0f, &count) && count == 7, "0.1 s at 60");
    CHECK(maniClipSampleCountFor(0.95f, 30.0f, &count) && count == 30, "a part frame rounds up");
    CHECK(maniClipSampleCountFor(16777215.0f, 1.0f, &count) && count == MANI_MAX_SAMPLES,
          "the most samples");
    CHECK(!maniClipSampleCountFor(16777216.0f, 1.0f, &count), "one past the most");
    // Long durations: a real part frame is never snapped away.
    CHECK(maniClipSampleCountFor(1000000.5f, 1.0f, &count) && count == 1000002,
          "half a frame at a million");
    CHECK(maniClipSampleCountFor(100000.0625f, 1.0f, &count) && count == 100002,
          "a sixteenth of a frame at a hundred thousand");
    // Built and sampled: the last interval is a whole one.
    OpenSkeleton();
    maniClipSource source = SampleSource();
    source.floatTrackCount = 0;
    source.discreteTrackCount = 0;
    source.eventCount = 0;
    source.markerCount = 0;
    source.duration = 0.6f;
    source.sampleRate = 25.0f;
    CHECK(BuildFrom(&source).result == mani_success, "builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipSampleCount(&clip) == 16, "0.6 s at 25: 16 samples, not 17");
}

// The scale channel's Hermite curve (v0 1 1 1, out-tangent 2 0 -1; v1
// 2 2 2, in-tangent 0 1 0) and the wave's (0, out-tangent 4; 1, 0).
static void Hermite(double u, double out[4])
{
    double h00 = 2 * u * u * u - 3 * u * u + 1;
    double h10 = u * u * u - 2 * u * u + u;
    double h01 = -2 * u * u * u + 3 * u * u;
    double h11 = u * u * u - u * u;
    out[0] = h00 + 2 * h10 + 2 * h01;
    out[1] = h00 + 2 * h01 + h11;
    out[2] = h00 - h10 + 2 * h01;
    out[3] = 4 * h10 + h01;
}

// The largest distance halfway between samples from a curve to its chord.
static void ChordErrors(double rate, double* scaleOut, double* waveOut)
{
    uint32_t n = (uint32_t)rate;
    *scaleOut = 0.0;
    *waveOut = 0.0;
    for (uint32_t k = 0; k < n; ++k)
    {
        double a[4];
        double b[4];
        double m[4];
        Hermite((double)k / rate, a);
        Hermite((double)(k + 1) / rate, b);
        Hermite((double)(float)((k + 0.5) / rate), m);
        double d = 0.0;
        for (int c = 0; c < 3; ++c)
        {
            double e = m[c] - 0.5 * (a[c] + b[c]);
            d += e * e;
        }
        *scaleOut = fmax(*scaleOut, sqrt(d));
        *waveOut = fmax(*waveOut, fabs(m[3] - 0.5 * (a[3] + b[3])));
    }
}

static maniClipReport Report(const maniClipSource* source)
{
    maniClipReport report;
    memset(&report, 0xFF, sizeof report);
    CHECK(BuildFrom(source).result == mani_success, "builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    maniBuilder* builder = NewBuilder();
    CHECK(maniMeasureClip(builder, source, &clip, &report) == mani_success, "measures");
    maniDestroyBuilder(builder);
    return report;
}

static void TestReport(void)
{
    OpenSkeleton();
    maniClipSource source = SampleSource();
    maniClipReport r = Report(&source);
    // At the samples: binary32 rounding only.
    CHECK(r.sampleTranslation.error < 1e-6 && r.sampleScale.error < 1e-6, "samples stored");
    CHECK(r.sampleRotation.error < 1e-6 && r.sampleFloat.error < 1e-6, "samples stored");
    // Halfway: the linear translation's keys are on the grid, and a
    // normalized lerp halfway is the slerp: both near 0.
    CHECK(r.midTranslation.error < 1e-6 && r.midRotation.error < 1e-6, "linear midpoints");
    double scale = 0.0;
    double wave = 0.0;
    ChordErrors(10.0, &scale, &wave);
    CHECK(fabs(r.midScale.error - scale) < 1e-6 && r.midScale.item == 2, "cubic midpoints");
    CHECK(fabs(r.midFloat.error - wave) < 1e-6 && r.midFloat.item == 2, "a float's midpoints");
    CHECK(r.midScale.time > 0.0f && r.midScale.time < 1.0f, "where");
    // Step keys: bone 0's key at 0.55 plays at 0.6; the gate's at 0.3f,
    // a float just past 0.3, at 0.4.
    CHECK(fabs(r.boneStepShift.error - (0.6 - (double)0.55f)) < 1e-12 &&
              r.boneStepShift.item == 0 && r.boneStepShift.time == 0.55f,
          "a bone's step");
    CHECK(fabs(r.floatStepShift.error - (0.4 - (double)0.3f)) < 1e-12 && r.floatStepShift.item == 1,
          "a float's step");
    // Four times the rate: the chord error falls sixteen times.
    source.sampleRate = 40.0f;
    maniClipReport fine = Report(&source);
    ChordErrors(40.0, &scale, &wave);
    CHECK(fabs(fine.midScale.error - scale) < 1e-6, "cubic midpoints at 40");
    CHECK(fine.midScale.error < r.midScale.error / 12.0, "a higher rate, a smaller error");
    CHECK(fine.boneStepShift.error < 0.025 + 1e-6, "steps land closer");

    // Refusals.
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    maniBuilder* builder = NewBuilder();
    maniClipReport report;
    source = SampleSource();
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_errorInvalid, "another rate");
    source.sampleRate = 40.0f;
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_success, "the clip's rate");
    source.floatTrackCount = 0;
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_success,
          "a source's float tracks may be fewer");
    source = SampleSource();
    source.sampleRate = 40.0f;
    source.loop = 5;
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_errorInvalid,
          "a source the build refuses");
    CHECK(maniMeasureClip(builder, NULL, &clip, &report) == mani_errorInvalid, "null source");
    CHECK(maniMeasureClip(builder, &source, NULL, &report) == mani_errorInvalid, "null clip");
    CHECK(maniMeasureClip(builder, &source, &clip, NULL) == mani_errorInvalid, "null report");
    CHECK(maniBuilderMisuses(builder) == 3, "misuses");
    CHECK(maniMeasureClip(NULL, &source, &clip, &report) == mani_errorInvalid, "null builder");
    maniDestroyBuilder(builder);
    // A clip without the source's float tracks.
    source = SampleSource();
    maniClipSource bare = source;
    bare.floatTrackCount = 0;
    CHECK(BuildFrom(&bare).result == mani_success, "builds");
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    builder = NewBuilder();
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_errorInvalid,
          "a float track the clip lacks");
    maniDestroyBuilder(builder);
}

// Keys on opposite hemispheres turn the short way; a step to a key
// written with w below 0 is stored on the previous sample's side, which
// the measure sees as no error.
static void TestHemispheres(void)
{
    OpenSkeleton();
    static const float times[2] = {0.0f, 1.0f};
    static const float negated[8] = {0, 0, 0, 1, 0, 0, -0.70710677f, -0.70710677f};
    maniBoneTrackSource track = {
        2, kNone, (maniChannelSource){times, negated, 2, maniInterpolationLinear}, kNone};
    maniClipSource source = {
        .skeleton = &s_skeleton, .boneTracks = &track, .boneTrackCount = 1, .sampleRate = 10.0f};
    CHECK(BuildFrom(&source).result == mani_success, "builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    maniTransform r = maniPoseLocal(SampleAt(&clip, 0.5f), 2);
    double half = 0.25 * 3.14159265358979323846 * 0.5;
    CHECK(Near(fabsf(r.rotation.z), sin(half), 2e-7) && Near(fabsf(r.rotation.w), cos(half), 2e-7),
          "the short way");

    static const float steps[8] = {0, 0, 0, 1, 0, 0, -0.38268343f, -0.92387953f};
    track.rotation = (maniChannelSource){times, steps, 2, maniInterpolationStep};
    CHECK(BuildFrom(&source).result == mani_success, "builds");
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    maniTransform last = maniPoseLocal(SampleAt(&clip, 1.0f), 2);
    CHECK(last.rotation.w > 0.0f, "kept on the previous sample's side");
    maniBuilder* builder = NewBuilder();
    maniClipReport report;
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_success, "measures");
    CHECK(report.sampleRotation.error < 1e-6, "a flipped sign is no error");
    maniDestroyBuilder(builder);
}

// A cubic's tangents scale with the span between keys (Appendix C).
static void TestCubicSpan(void)
{
    OpenSkeleton();
    static const float times[2] = {0.0f, 0.5f};
    static const float values[6] = {0, 0, 0, 2, 1, 0};
    maniFloatTrackSource curve = {"curve", 5, {times, values, 2, maniInterpolationCubicSpline}};
    maniClipSource source = {
        .skeleton = &s_skeleton, .floatTracks = &curve, .floatTrackCount = 1, .sampleRate = 10.0f};
    CHECK(BuildFrom(&source).result == mani_success, "builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    for (int k = 0; k <= 5; ++k)
    {
        float value = 0.0f;
        float time = (float)k / 10.0f;
        CHECK(maniSampleClipFloats(&clip, time, &value, 1, NULL) == mani_success, "samples");
        double u = (double)time / 0.5;
        double expected = (-2 * u * u * u + 3 * u * u) * 1 + 0.5 * (u * u * u - u * u) * 2;
        CHECK(Near(value, expected, 1e-6), "the in-tangent times the span");
    }
}

static void TestNamedItemsAlone(void)
{
    OpenSkeleton();
    // An event past every key sets the default duration; -0 sorts as 0.
    static const maniEventSource events[2] = {{1.5f, 1, "late", 4}, {-0.0f, 2, "start", 5}};
    maniBoneTrackSource empty = {3, kNone, kNone, kNone};
    maniClipSource source = {.skeleton = &s_skeleton,
                             .boneTracks = &empty,
                             .events = events,
                             .boneTrackCount = 1,
                             .eventCount = 2,
                             .sampleRate = 10.0f};
    CHECK(BuildFrom(&source).result == mani_success, "a track without channels builds");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    CHECK(maniClipDuration(&clip) == 1.5f, "the duration from an event");
    maniClipEvent first = maniClipEventAt(&clip, 0);
    CHECK(first.payload == 2 && first.time == 0.0f && !signbit(first.time), "-0 first, as 0");
    CHECK(maniPoseLocal(SampleAt(&clip, 0.5f), 3).translation.x == 0.5f, "at rest");
    // The measure checks the rate, not only the count: 9.5 per second
    // over 1.5 s gives 16 samples, as 10 does.
    maniBuilder* builder = NewBuilder();
    maniClipReport report;
    source.sampleRate = 9.5f;
    uint32_t count = 0;
    CHECK(maniClipSampleCountFor(1.5f, 9.5f, &count) && count == maniClipSampleCount(&clip),
          "the same count");
    CHECK(maniMeasureClip(builder, &source, &clip, &report) == mani_errorInvalid, "another rate");
    maniDestroyBuilder(builder);
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
    maniSkeletonSource source = {moved, 4};
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

    // Named items: each list's refusals carry the list in the reason.
    maniFloatTrackSource floats[3];
    memcpy(floats, s_floats, sizeof floats);
    source = SampleSource();
    source.floatTracks = floats;
    floats[1].nameLength = 0;
    ExpectRefusal(&source, maniBuildReasonFloatTrack | maniBuildReasonName, 1, "a float name");
    memcpy(floats, s_floats, sizeof floats);
    floats[2].name = "ramp";
    r = BuildFrom(&source);
    CHECK(r.reason == (maniBuildReasonFloatTrack | maniBuildReasonTrack) && r.item == 2 &&
              r.other == 0,
          "a float name twice");
    memcpy(floats, s_floats, sizeof floats);
    floats[0].keys.keyCount = 0;
    ExpectRefusal(&source, maniBuildReasonFloatTrack | maniBuildReasonKeyCount, 0,
                  "a float track without keys");
    memcpy(floats, s_floats, sizeof floats);
    floats[0].keys.times = backwards;
    floats[0].keys.keyCount = 3;
    ExpectRefusal(&source, maniBuildReasonFloatTrack | maniBuildReasonKeyTime, 0,
                  "float keys not increasing");
    maniDiscreteTrackSource discrete[2];
    memcpy(discrete, s_discrete, sizeof discrete);
    source = SampleSource();
    source.discreteTracks = discrete;
    discrete[1].name = "foot";
    discrete[1].nameLength = 4;
    r = BuildFrom(&source);
    CHECK(r.reason == (maniBuildReasonDiscreteTrack | maniBuildReasonTrack) && r.item == 1,
          "a discrete name twice");
    memcpy(discrete, s_discrete, sizeof discrete);
    discrete[0].times = backwards;
    ExpectRefusal(&source, maniBuildReasonDiscreteTrack | maniBuildReasonKeyTime, 0,
                  "discrete keys not increasing");
    memcpy(discrete, s_discrete, sizeof discrete);
    discrete[1].keyCount = 0;
    ExpectRefusal(&source, maniBuildReasonDiscreteTrack | maniBuildReasonKeyCount, 1,
                  "a discrete track without keys");
    maniEventSource events[4];
    memcpy(events, s_events, sizeof events);
    source = SampleSource();
    source.events = events;
    events[2].time = -1.0f;
    ExpectRefusal(&source, maniBuildReasonEvent | maniBuildReasonKeyTime, 2, "an event before 0");
    memcpy(events, s_events, sizeof events);
    events[3].name = "\xFF";
    events[3].nameLength = 1;
    ExpectRefusal(&source, maniBuildReasonEvent | maniBuildReasonName, 3, "an event's name");
    source = SampleSource();
    source.duration = 0.75f;
    ExpectRefusal(&source, maniBuildReasonDiscreteTrack | maniBuildReasonKeyTime, 0,
                  "a discrete key past the duration");
    source.discreteTrackCount = 0;
    ExpectRefusal(&source, maniBuildReasonEvent | maniBuildReasonKeyTime, 3,
                  "an event past the duration");
    source.eventCount = 3;
    source.markers = s_events;
    source.markerCount = 4;
    ExpectRefusal(&source, maniBuildReasonMarker | maniBuildReasonKeyTime, 3,
                  "a marker past the duration");
    source = SampleSource();
    source.markers = NULL;
    ExpectRefusal(&source, maniBuildReasonMarker | maniBuildReasonCount, 0, "markers missing");

    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildClip(builder, NULL).result == mani_errorInvalid, "null source");
    CHECK(maniBuilderMisuses(builder) == 1, "a misuse");
    maniDestroyBuilder(builder);
}

static void Rehash(void)
{
    maniStoreU64(s_file + 32, maniHashBytes(MANI_HASH_INIT, s_file + MANI_CONTAINER_HEADER_SIZE,
                                            s_size - MANI_CONTAINER_HEADER_SIZE));
}

static uint64_t SectionAt(uint32_t tag)
{
    return maniLoadU64(s_file + MANI_CONTAINER_HEADER_SIZE + (uint64_t)(tag - 1) * 24 + 8);
}

static void ExpectFormat(uint64_t offset, const char* what)
{
    Rehash();
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == offset &&
              r.tag == TestTagAt(s_file, s_size, offset),
          what);
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
    maniStoreU32(s_file + header + 60, 3);
    ExpectFormat(header + 60, "an additive basis past first frame");
    s_file[header + 63] = 1;
    ExpectFormat(header + 60, "the basis word's last byte");
    s_file[header + 56] = 2;
    ExpectFormat(header + 56, "form past compressed");
    s_file[header + 56] = 1;
    ExpectFormat(header + 52, "compressed without segments");
    s_file[header + 52] = 1;
    ExpectFormat(header + 52, "segments at full precision");
    s_file[header + 52] = 1;
    s_file[header + 56] = 1;
    // A compressed header over full-precision samples: the samples
    // section's size is wrong.
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 2 * 24 + 16, "compressed form with samples");
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
    // The named sections.
    uint64_t floatTracks = SectionAt(maniClipTagFloatTracks);
    uint64_t floatSamples = SectionAt(maniClipTagFloatSamples);
    uint64_t discrete = SectionAt(maniClipTagDiscreteTracks);
    uint64_t keys = SectionAt(maniClipTagDiscreteKeys);
    uint64_t events = SectionAt(maniClipTagEvents);
    uint64_t markers = SectionAt(maniClipTagMarkers);
    // Swapping two float track entries keeps each name right but breaks
    // the hash order.
    uint8_t entry[24];
    memcpy(entry, s_file + floatTracks, 24);
    memcpy(s_file + floatTracks, s_file + floatTracks + 24, 24);
    memcpy(s_file + floatTracks + 24, entry, 24);
    ExpectFormat(floatTracks + 24 + 8, "float tracks out of hash order");
    // foot keeps two keys, state one: a key left over in the section.
    maniStoreU32(s_file + discrete + 20, 2);
    maniStoreU32(s_file + discrete + 24 + 16, 2);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 6 * 24 + 16, "discrete keys left over");
    maniStoreU32(s_file + floatTracks + 4, 0);
    ExpectFormat(floatTracks, "an empty float name");
    s_file[floatTracks + 8] ^= 1;
    ExpectFormat(floatTracks + 8, "a float name's hash");
    maniStoreU32(s_file + floatTracks + 16, 2);
    ExpectFormat(floatTracks + 16, "an interpolation");
    maniStoreU32(s_file + floatTracks + 20, 1);
    ExpectFormat(floatTracks + 20, "float track padding");
    StoreF32(floatSamples + 8, NAN);
    ExpectFormat(floatSamples + 8, "a float sample not finite");
    // The second discrete track starts where the first ends.
    maniStoreU32(s_file + discrete + 24 + 16, 0);
    ExpectFormat(discrete + 24 + 16, "discrete keys not contiguous");
    maniStoreU32(s_file + discrete + 20, 0);
    ExpectFormat(discrete + 20, "a discrete track without keys");
    StoreF32(keys + 8, 0.0f);
    ExpectFormat(keys + 8, "discrete keys not increasing");
    StoreF32(keys, 2.0f);
    ExpectFormat(keys, "a discrete key past the duration");
    StoreF32(events + 24, 0.1f);
    ExpectFormat(events + 24, "events out of time order");
    maniStoreU32(s_file + markers + 4, 5);
    ExpectFormat(markers + 4, "a marker's payload");
    maniStoreU32(s_file + markers + 8, 100000);
    ExpectFormat(markers + 8, "a marker's name past the names");
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

static void TestPinnedScene(void)
{
    OpenSkeleton();
    maniClipSource source = SampleSource();
    // Root motion from bone a's translation along x and z.
    const maniRootMotionSource root = {1, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
                                       1, maniRootReferenceRest,         maniRootReferenceRest,
                                       0};
    source.rootMotion = &root;
    CHECK(BuildFrom(&source).result == mani_success, "builds with root motion");
    maniClip clip;
    CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens");
    uint64_t hash = MANI_HASH_INIT;
    for (int i = 0; i < 16; ++i)
    {
        float time = -0.35f + 0.173f * (float)i;
        maniTransform locals[4];
        CHECK(maniPoseReadLocals(SampleAt(&clip, time), 0, 4, locals) == mani_success, "reads");
        hash = maniHash64(hash, locals, (int32_t)sizeof locals);
        float values[3];
        CHECK(maniSampleClipFloats(&clip, time, values, 3, NULL) == mani_success, "floats");
        hash = maniHash64(hash, values, (int32_t)sizeof values);
        maniRootMove move;
        CHECK(maniClipRootMotion(&clip, time, 0.1f, &move) == mani_success, "root motion");
        const float parts[7] = {move.translation.x, move.translation.y, move.translation.z,
                                move.rotation.x,    move.rotation.y,    move.rotation.z,
                                move.rotation.w};
        hash = maniHash64(hash, parts, (int32_t)sizeof parts);
    }
    // The writer's library version (bytes 16 to 21) changes with every release.
    memset(s_file + 16, 0, 6);
    hash = maniHash64(hash, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_SCENE_HASH, "the scene's bits the same on every platform");
    if (hash != PINNED_SCENE_HASH)
    {
        printf("scene hash 0x%016llx\n", (unsigned long long)hash);
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

// An additive clip, from either basis: its difference at a time applied
// to the basis gives the plain clip's pose there; channels without a
// track are the identity; subsets keep the clip's kind.
static _Alignas(16) uint8_t s_plain[65536];
static _Alignas(32) uint8_t s_posesMemory[3][4096];

static double PoseApart(const maniPose* a, const maniPose* b)
{
    double worst = 0.0;
    for (uint32_t bone = 0; bone < 4; ++bone)
    {
        maniTransform x = maniPoseLocal(a, bone);
        maniTransform y = maniPoseLocal(b, bone);
        float dot = x.rotation.x * y.rotation.x + x.rotation.y * y.rotation.y +
                    x.rotation.z * y.rotation.z + x.rotation.w * y.rotation.w;
        float s = dot < 0.0f ? -1.0f : 1.0f;
        const float d[10] = {x.translation.x - y.translation.x,
                             x.translation.y - y.translation.y,
                             x.translation.z - y.translation.z,
                             x.rotation.x - s * y.rotation.x,
                             x.rotation.y - s * y.rotation.y,
                             x.rotation.z - s * y.rotation.z,
                             x.rotation.w - s * y.rotation.w,
                             x.scale.x - y.scale.x,
                             x.scale.y - y.scale.y,
                             x.scale.z - y.scale.z};
        for (int c = 0; c < 10; ++c)
        {
            double e = fabs((double)d[c]);
            worst = e > worst ? e : worst;
        }
    }
    return worst;
}

static void TestAdditiveClip(void)
{
    maniClipSource source = SampleSource();
    CHECK(BuildFrom(&source).result == mani_success, "builds the plain clip");
    memcpy(s_plain, s_file, s_size);
    maniClip plain;
    CHECK(maniOpenClip(s_plain, s_size, &plain).result == mani_success, "opens the plain clip");
    CHECK(maniClipAdditiveBasis(&plain) == maniAdditiveNone, "a clip of poses");
    maniPose* p[3];
    for (int i = 0; i < 3; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_posesMemory[i], sizeof s_posesMemory[i], &p[i]) ==
                  mani_success,
              "pose");
    }
    const maniAdditiveBasis bases[2] = {maniAdditiveRest, maniAdditiveFirstFrame};
    const float times[6] = {0.0f, 0.13f, 0.5f, 0.55f, 0.77f, 1.0f};
    for (int i = 0; i < 2; ++i)
    {
        source.additive = bases[i];
        CHECK(BuildFrom(&source).result == mani_success, "builds an additive clip");
        maniClip clip;
        CHECK(maniOpenClip(s_file, s_size, &clip).result == mani_success, "opens it");
        CHECK(maniClipAdditiveBasis(&clip) == bases[i], "its basis");
        double worst = 0.0;
        bool identity = true;
        for (int k = 0; k < 6; ++k)
        {
            CHECK(maniSampleClip(&plain, &s_skeleton, times[k], p[0]) == mani_success, "plain");
            CHECK(maniSampleClip(&clip, &s_skeleton, times[k], p[1]) == mani_success, "additive");
            CHECK(maniPoseIsDifference(p[1]), "samples a difference");
            maniTransform t = maniPoseLocal(p[1], 3);
            identity = identity && t.translation.x == 0.0f && t.rotation.w == 1.0f &&
                       t.scale.x == 1.0f && t.rotation.x == 0.0f;
            if (bases[i] == maniAdditiveRest)
            {
                CHECK(maniPoseSetRest(p[2], &s_skeleton) == mani_success, "the rest");
            }
            else
            {
                CHECK(maniSampleClip(&plain, &s_skeleton, 0.0f, p[2]) == mani_success, "frame 0");
            }
            maniBlendLayer layer = {p[1], 1.0f, NULL};
            CHECK(maniAddPoses(&s_skeleton, p[2], &layer, 1) == mani_success, "adds");
            double e = PoseApart(p[0], p[2]);
            worst = e > worst ? e : worst;
        }
        CHECK(worst < 2e-6, "the basis plus the difference: the plain clip");
        CHECK(identity, "a bone without a track: the identity");
        const uint32_t bone[1] = {2};
        CHECK(maniSampleClipBones(&clip, &s_skeleton, 0.5f, bone, 1, p[0]) == mani_errorInvalid,
              "a subset into a pose");
        CHECK(maniSampleClipBones(&clip, &s_skeleton, 0.5f, bone, 1, p[1]) == mani_success,
              "a subset into a difference");
        CHECK(maniSampleClipBones(&plain, &s_skeleton, 0.5f, bone, 1, p[1]) == mani_errorInvalid,
              "a plain subset into a difference");
    }
    source.additive = 3;
    maniBuildResult r = BuildFrom(&source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonKind, "a basis unknown");
    // A scale whose first value is 0: no first-frame basis.
    source.additive = maniAdditiveFirstFrame;
    s_tracks[1].scale =
        (maniChannelSource){kLinearTimes, kLinearValues, 3, maniInterpolationLinear};
    r = BuildFrom(&source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonBasis && r.item == 1,
          "a basis scale 0");
    // Only z at 0: refused as well.
    static const float kFlatZ[6] = {1, 1, 0, 1, 1, 1};
    s_tracks[1].scale = (maniChannelSource){kTurnTimes, kFlatZ, 2, maniInterpolationLinear};
    r = BuildFrom(&source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonBasis, "a basis z of 0");
    source.additive = maniAdditiveRest;
    CHECK(BuildFrom(&source).result == mani_success, "the rest's scale is 1");
}

// The same file whatever bytes pad the source, its tracks and events.
static void TestPaddingIgnored(void)
{
    OpenSkeleton();
    uint8_t* built[2];
    size_t sizes[2];
    for (int k = 0; k < 2; ++k)
    {
        uint8_t pattern = kTestPadPatterns[k];
        maniClipSource source = SampleSource();
        maniBoneTrackSource tracks[3];
        maniFloatTrackSource floats[3];
        maniDiscreteTrackSource discrete[2];
        maniEventSource events[4];
        maniEventSource markers[2];
        memcpy(tracks, s_tracks, sizeof tracks);
        memcpy(floats, s_floats, sizeof floats);
        memcpy(discrete, s_discrete, sizeof discrete);
        memcpy(events, s_events, sizeof events);
        memcpy(markers, s_markers, sizeof markers);
        TEST_PAD(tracks, 3, kFieldsBoneTrackSource, pattern);
        TEST_PAD(floats, 3, kFieldsFloatTrackSource, pattern);
        TEST_PAD(discrete, 2, kFieldsDiscreteTrackSource, pattern);
        TEST_PAD(events, 4, kFieldsEventSource, pattern);
        TEST_PAD(markers, 2, kFieldsEventSource, pattern);
        source.boneTracks = tracks;
        source.floatTracks = floats;
        source.discreteTracks = discrete;
        source.events = events;
        source.markers = markers;
        TEST_PAD(&source, 1, kFieldsClipSource, pattern);
        maniBuilder* builder = NewBuilder();
        CHECK(maniBuildClip(builder, &source).result == mani_success, "builds padded");
        built[k] = TestCopyBuilt(builder, &sizes[k]);
        maniDestroyBuilder(builder);
    }
    CHECK(TestSameBuilt(built, sizes), "the same bytes under either padding");
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
    TestFactorClamped();
    TestDurationAndRate();
    TestFloatTracks();
    TestDiscreteTracks();
    TestEventsAndMarkers();
    TestWholeFrameDurations();
    TestReport();
    TestHemispheres();
    TestCubicSpan();
    TestNamedItemsAlone();
    TestWrongSkeleton();
    TestBuildRefusals();
    TestAdditiveClip();
    TestOpenRefusals();
    TestEveryByteFlip();
    TestPinnedScene();
    TestPaddingIgnored();
    return s_failures == 0 ? 0 : 1;
}
