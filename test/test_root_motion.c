// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion at build: the motion taken from a bone's local
// transform at each sample, from each reference, along the chosen axes
// and as a twist or the whole rotation; the bone keeping the rest, so the
// motion times the stored bone is the source; the loop option; the
// section carried by the compressor; every refusal of the builder and of
// opening; the bytes pinned. Run as test_root_motion --seed <file> it
// writes a seed for the clip fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "clip_build.h"
#include "clip_file.h"
#include "container.h"
#include "pose_memory.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_plainFile[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;
static size_t s_plainSize;
static _Alignas(32) uint8_t s_poseMemory[4096];
static _Alignas(32) uint8_t s_plainMemory[4096];
static maniSkeleton s_skeleton;

// Hips (a root at height 1), a spine under it, a prop without a track.
static const maniBoneSource kBones[3] = {
    {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"spine", 5, 0, {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"prop", 4, -1, {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
};

// The hips walk from (0, 1, 0) to (2, 1.5, 4) over a second, turning a
// quarter about y with a 30 degree pitch about x; the spine sways.
static const float kTimes[2] = {0.0f, 1.0f};
static const float kWalk[6] = {0, 1, 0, 2, 1.5f, 4};
static const float kTurn[8] = {0, 0, 0, 1, 0.18301270f, 0.68301270f, -0.18301270f, 0.68301270f};
static const float kSway[8] = {0, 0, 0, 1, 0, 0, 0.25881905f, 0.96592583f};
static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
static maniBoneTrackSource s_tracks[2];
static maniRootMotionSource s_root;

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
    maniSkeletonSource source = {kBones, 3};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success,
          "skeleton");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
}

// The walk, at 4 samples a second: horizontal translation and the twist
// about y from the rest.
static maniClipSource Source(const maniRootMotionSource* root)
{
    s_tracks[0] = (maniBoneTrackSource){0,
                                        {kTimes, kWalk, 2, maniInterpolationLinear},
                                        {kTimes, kTurn, 2, maniInterpolationLinear},
                                        kNone};
    s_tracks[1] =
        (maniBoneTrackSource){1, kNone, {kTimes, kSway, 2, maniInterpolationLinear}, kNone};
    return (maniClipSource){.skeleton = &s_skeleton,
                            .boneTracks = s_tracks,
                            .boneTrackCount = 2,
                            .loop = maniLoopRepeat,
                            .sampleRate = 4.0f,
                            .rootMotion = root};
}

static maniRootMotionSource Walk(void)
{
    return (maniRootMotionSource){0, maniRootAxisX | maniRootAxisZ, maniRootRotationTwist,
                                  1, maniRootReferenceRest,         maniRootReferenceRest,
                                  0};
}

static maniBuildResult Build(const maniClipSource* source, uint8_t* file, size_t* size)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildClip(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniBuilder_CopyBuilt(builder, file, sizeof s_file, size) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return r;
}

// The clip with root motion into s_file and without into s_plainFile.
static bool BuildBoth(const maniRootMotionSource* root)
{
    s_root = *root;
    maniClipSource with = Source(&s_root);
    maniClipSource without = Source(NULL);
    return Build(&with, s_file, &s_size).result == mani_success &&
           Build(&without, s_plainFile, &s_plainSize).result == mani_success;
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// Sample k's motion from the section: translation, then rotation.
static void MotionAt(const maniClip* clip, uint32_t k, float out[7])
{
    const uint8_t* p = clip->rootMotion + MANI_ROOT_MOTION_HEADER_SIZE +
                       MANI_ROOT_MOTION_PERIOD_SIZE + 28 * (size_t)k;
    for (int i = 0; i < 7; ++i)
    {
        out[i] = LoadF32(p + 4 * i);
    }
}

static void Multiply(const float a[4], const float b[4], float out[4])
{
    out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

static void Rotate(const float q[4], const float v[3], float out[3])
{
    float c[3] = {q[1] * v[2] - q[2] * v[1], q[2] * v[0] - q[0] * v[2], q[0] * v[1] - q[1] * v[0]};
    float cc[3] = {q[1] * c[2] - q[2] * c[1], q[2] * c[0] - q[0] * c[2], q[0] * c[1] - q[1] * c[0]};
    for (int i = 0; i < 3; ++i)
    {
        out[i] = v[i] + 2.0f * (q[3] * c[i] + cc[i]);
    }
}

// The largest difference over every sample between the motion times the
// stored hips and the hips without root motion; the spine compared too.
static float Recomposed(const maniClip* with, const maniClip* without)
{
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &a) == mani_success &&
              maniInitPose(&s_skeleton, s_plainMemory, sizeof s_plainMemory, &b) == mani_success,
          "poses");
    float worst = 0.0f;
    for (uint32_t k = 0; k < maniClip_GetSampleCount(with); ++k)
    {
        float time = (float)k / 4.0f;
        CHECK(maniClip_Sample(with, &s_skeleton, time, a) == mani_success &&
                  maniClip_Sample(without, &s_skeleton, time, b) == mani_success,
              "samples");
        float m[7];
        MotionAt(with, k, m);
        maniTransform stored = maniPose_GetLocal(a, 0);
        maniTransform source = maniPose_GetLocal(b, 0);
        const float st[3] = {stored.translation.x, stored.translation.y, stored.translation.z};
        const float sr[4] = {stored.rotation.x, stored.rotation.y, stored.rotation.z,
                             stored.rotation.w};
        float t[3];
        float r[4];
        Rotate(m + 3, st, t);
        Multiply(m + 3, sr, r);
        const float want[7] = {source.translation.x, source.translation.y, source.translation.z,
                               source.rotation.x,    source.rotation.y,    source.rotation.z,
                               source.rotation.w};
        float dot = r[0] * want[3] + r[1] * want[4] + r[2] * want[5] + r[3] * want[6];
        for (int i = 0; i < 3; ++i)
        {
            worst = fmaxf(worst, fabsf(m[i] + t[i] - want[i]));
        }
        worst = fmaxf(worst, 1.0f - fabsf(dot));
        maniTransform spine = maniPose_GetLocal(a, 1);
        maniTransform plain = maniPose_GetLocal(b, 1);
        worst = fmaxf(worst, fabsf(spine.rotation.z - plain.rotation.z));
    }
    return worst;
}

static bool OpenBoth(maniClip* with, maniClip* without)
{
    return maniOpenClip(s_file, s_size, with).result == mani_success &&
           maniOpenClip(s_plainFile, s_plainSize, without).result == mani_success;
}

static void TestWalk(void)
{
    maniRootMotionSource walk = Walk();
    maniClip with;
    maniClip without;
    CHECK(BuildBoth(&walk) && OpenBoth(&with, &without), "builds and opens");
    maniRootMotionSource read;
    CHECK(maniClip_GetRootMotionSource(&with, &read) && memcmp(&read, &walk, sizeof read) == 0 &&
              !maniClip_GetRootMotionSource(&without, &read) &&
              !maniClip_GetRootMotionSource(NULL, &read) &&
              !maniClip_GetRootMotionSource(&with, NULL),
          "the source read back; none without");
    float worst = Recomposed(&with, &without);
    CHECK(worst < 2e-6f, "the motion times the stored bone is the source");
    if (worst >= 2e-6f)
    {
        printf("  apart by %g\n", (double)worst);
    }
    // Along x and z only, a twist about y only.
    bool shaped = true;
    for (uint32_t k = 0; k < maniClip_GetSampleCount(&with); ++k)
    {
        float m[7];
        MotionAt(&with, k, m);
        shaped = shaped && m[1] == 0.0f && m[3] == 0.0f && m[5] == 0.0f;
    }
    float last[7];
    MotionAt(&with, 4, last);
    CHECK(shaped && last[0] == 2.0f && last[2] == 4.0f && fabsf(last[4] - 0.70710677f) < 1e-6f,
          "x and z, the twist about y: a quarter turn");
    // The period: from the first sample to the last, in the first's frame.
    const uint8_t* p = with.rootMotion + MANI_ROOT_MOTION_HEADER_SIZE;
    CHECK(LoadF32(p) == 2.0f && LoadF32(p + 8) == 4.0f &&
              fabsf(LoadF32(p + 16) - 0.70710677f) < 1e-6f,
          "the whole period's move");
}

static void TestReferences(void)
{
    // The first sample as the reference: the motion starts at zero.
    maniRootMotionSource first = Walk();
    first.translationAxes = maniRootAxisX | maniRootAxisY | maniRootAxisZ;
    first.translationReference = maniRootReferenceFirst;
    first.rotation = maniRootRotationWhole;
    first.rotationReference = maniRootReferenceFirst;
    maniClip with;
    maniClip without;
    CHECK(BuildBoth(&first) && OpenBoth(&with, &without), "builds");
    float m[7];
    MotionAt(&with, 0, m);
    CHECK(m[0] == 0.0f && m[1] == 0.0f && m[2] == 0.0f && m[6] == 1.0f,
          "from the first sample: none at the start");
    MotionAt(&with, 4, m);
    CHECK(m[1] == 0.5f && fabsf(m[3] - 0.18301270f) < 1e-6f, "the whole rotation, y too");
    CHECK(Recomposed(&with, &without) < 2e-6f, "recomposed from the first sample");
    // The origin: the motion is the bone itself, and the bone keeps none.
    maniRootMotionSource origin = first;
    origin.translationReference = maniRootReferenceOrigin;
    origin.rotationReference = maniRootReferenceOrigin;
    CHECK(BuildBoth(&origin) && OpenBoth(&with, &without), "builds");
    MotionAt(&with, 0, m);
    CHECK(m[1] == 1.0f && Recomposed(&with, &without) < 2e-6f, "from the origin");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success &&
              maniClip_Sample(&with, &s_skeleton, 0.5f, pose) == mani_success,
          "samples");
    maniTransform hips = maniPose_GetLocal(pose, 0);
    CHECK(fabsf(hips.translation.y) < 1e-6f && fabsf(hips.rotation.w - 1.0f) < 1e-6f,
          "the whole motion out: the bone at the origin");
}

static void TestLoop(void)
{
    maniRootMotionSource loop = Walk();
    loop.loop = 1;
    maniClip with;
    maniClip without;
    CHECK(BuildBoth(&loop) && OpenBoth(&with, &without), "builds");
    float a[7];
    float b[7];
    MotionAt(&with, 0, a);
    MotionAt(&with, 4, b);
    bool same = true;
    for (int i = 0; i < 7; ++i)
    {
        same = same && fabsf(a[i] - b[i]) < 1e-6f;
    }
    CHECK(same, "the loop option: the motion ends where it began");
    MotionAt(&with, 2, a);
    CHECK(fabsf(a[0]) < 1e-6f && fabsf(a[2]) < 1e-6f,
          "the walk's steady drift spread out: none left halfway");
    CHECK(Recomposed(&with, &without) < 2e-6f, "still the source, recomposed");
    const uint8_t* p = with.rootMotion + MANI_ROOT_MOTION_HEADER_SIZE;
    CHECK(fabsf(LoadF32(p)) < 1e-6f && fabsf(LoadF32(p + 24) - 1.0f) < 1e-6f,
          "a period of no move");
}

// The largest component difference between two moves (a rotation and
// its negative the same).
static float MoveApart(maniRootMove a, maniRootMove b)
{
    const float* fa = &a.translation.x;
    const float* fb = &b.translation.x;
    for (int i = 0; i < 3; ++i)
    {
        if (!isfinite(fa[i]) || !isfinite(fb[i]))
        {
            return INFINITY;
        }
    }
    if (!isfinite(a.rotation.x + a.rotation.y + a.rotation.z + a.rotation.w) ||
        !isfinite(b.rotation.x + b.rotation.y + b.rotation.z + b.rotation.w))
    {
        return INFINITY;
    }
    float worst = fmaxf(
        fabsf(a.translation.x - b.translation.x),
        fmaxf(fabsf(a.translation.y - b.translation.y), fabsf(a.translation.z - b.translation.z)));
    float dot = a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y +
                a.rotation.z * b.rotation.z + a.rotation.w * b.rotation.w;
    return fmaxf(worst, 1.0f - fabsf(dot));
}

static maniRootMove Move(const maniClip* clip, float time, float step)
{
    maniRootMove m = maniGetIdentityRootMove();
    CHECK(maniClip_GetRootMotion(clip, time, step, &m) == mani_success, "a move");
    return m;
}

// Sample k of the section as a move.
static maniRootMove SampleMove(const maniClip* clip, uint32_t k)
{
    float m[7];
    MotionAt(clip, k, m);
    return (maniRootMove){{m[0], m[1], m[2]}, {m[3], m[4], m[5], m[6]}};
}

static maniRootMove Inverted(maniRootMove m)
{
    const float back[4] = {-m.rotation.x, -m.rotation.y, -m.rotation.z, m.rotation.w};
    const float t[3] = {m.translation.x, m.translation.y, m.translation.z};
    float r[3];
    Rotate(back, t, r);
    return (maniRootMove){{-r[0], -r[1], -r[2]}, {back[0], back[1], back[2], back[3]}};
}

static void TestCompressed(void)
{
    maniRootMotionSource walk = Walk();
    maniClip clip;
    CHECK(BuildBoth(&walk) && maniOpenClip(s_file, s_size, &clip).result == mani_success, "builds");
    maniBuilder* builder = NewBuilder();
    maniCompressOptions options = maniDefaultCompressOptions();
    size_t size = 0;
    maniClip compressed;
    CHECK(maniCompressClip(builder, &clip, &s_skeleton, &options, NULL) == mani_success &&
              maniBuilder_CopyBuilt(builder, s_crafted, sizeof s_crafted, &size) == mani_success &&
              maniOpenClip(s_crafted, size, &compressed).result == mani_success,
          "compresses");
    maniDestroyBuilder(builder);
    CHECK(compressed.rootMotion != NULL &&
              memcmp(compressed.rootMotion, clip.rootMotion, maniRootMotionSize(5)) == 0,
          "the compressor carries the root motion unchanged");
    maniRootMove moves[4] = {Move(&compressed, 0.3f, 2.9f), Move(&clip, 0.3f, 2.9f),
                             Move(&compressed, 0.9f, -1.6f), Move(&clip, 0.9f, -1.6f)};
    CHECK(memcmp(&moves[0], &moves[1], sizeof moves[0]) == 0 &&
              memcmp(&moves[2], &moves[3], sizeof moves[0]) == 0,
          "the same moves from either form");
}

// Moves between two playhead times, against the samples; a step split
// anywhere composes to the whole, across wraps and backward.
static void TestMoves(void)
{
    maniRootMotionSource walk = Walk();
    maniClip clip;
    maniClip without;
    CHECK(BuildBoth(&walk) && OpenBoth(&clip, &without), "builds");
    maniRootMove between =
        maniComposeRootMotion(Inverted(SampleMove(&clip, 1)), SampleMove(&clip, 3));
    CHECK(MoveApart(Move(&clip, 0.25f, 0.5f), between) < 1e-6f, "between two samples");
    CHECK(MoveApart(Move(&clip, 0.6f, 0.0f), maniGetIdentityRootMove()) < 1e-6f,
          "no step, no move");
    maniRootMove period =
        maniComposeRootMotion(Inverted(SampleMove(&clip, 0)), SampleMove(&clip, 4));
    CHECK(MoveApart(Move(&clip, 0.0f, 1.0f), period) < 1e-6f &&
              MoveApart(Move(&clip, 0.3f, 1.0f), period) > 1e-3f,
          "a whole period from the start is the period's move");
    // Splits: (time, a, b) over wraps either way.
    static const float kSplits[6][3] = {{0.1f, 0.3f, 0.4f},  {0.75f, 1.5f, 1.0f},
                                        {0.9f, 3.25f, 2.6f}, {0.5f, -0.7f, -1.9f},
                                        {0.2f, 2.5f, -1.3f}, {0.6f, -4.1f, 3.3f}};
    float worst = 0.0f;
    for (int i = 0; i < 6; ++i)
    {
        float t = kSplits[i][0];
        float a = kSplits[i][1];
        float b = kSplits[i][2];
        maniRootMove two = maniComposeRootMotion(Move(&clip, t, a), Move(&clip, t + a, b));
        worst = fmaxf(worst, MoveApart(two, Move(&clip, t, a + b)));
    }
    CHECK(worst < 2e-5f, "a step split anywhere composes to the whole, across wraps");
    if (worst >= 2e-5f)
    {
        printf("  splits apart by %g\n", (double)worst);
    }
    maniRootMove there = Move(&clip, 0.4f, 2.7f);
    maniRootMove back = Move(&clip, 0.4f + 2.7f, -2.7f);
    CHECK(MoveApart(maniComposeRootMotion(there, back), maniGetIdentityRootMove()) < 2e-5f,
          "backward undoes forward");
    // The period turns a quarter about y, so four periods come back to
    // the start: a thousand periods by squaring are no move, a thousand
    // and one the period, and back the period's inverse.
    CHECK(MoveApart(Move(&clip, 0.0f, 1000.0f), maniGetIdentityRootMove()) < 1e-5f &&
              MoveApart(Move(&clip, 0.0f, 1001.0f), period) < 1e-5f &&
              MoveApart(Move(&clip, 1001.0f, -1001.0f), Inverted(period)) < 1e-5f,
          "a thousand periods by squaring");
    // A clamped clip stops at its ends.
    clip.loop = maniLoopClamp;
    maniRootMove rest = maniComposeRootMotion(Inverted(SampleMove(&clip, 2)), SampleMove(&clip, 4));
    CHECK(MoveApart(Move(&clip, 0.5f, 9.0f), rest) < 1e-6f &&
              MoveApart(Move(&clip, -3.0f, 3.5f),
                        maniComposeRootMotion(Inverted(SampleMove(&clip, 0)),
                                              SampleMove(&clip, 2))) < 1e-6f,
          "a clamped clip between its clamped times");
    CHECK(MoveApart(Move(&clip, -3.0f, 2.0f), maniGetIdentityRootMove()) == 0.0f,
          "both times before the start: no move");
    maniRootMove m;
    CHECK(maniClip_GetRootMotion(&without, 0.0f, 1.0f, &m) == mani_success &&
              MoveApart(m, maniGetIdentityRootMove()) == 0.0f,
          "a clip without root motion moves nothing");
    CHECK(maniClip_GetRootMotion(NULL, 0.0f, 1.0f, &m) == mani_errorInvalid &&
              maniClip_GetRootMotion(&clip, 0.0f, 1.0f, NULL) == mani_errorInvalid &&
              maniClip_GetRootMotion(&clip, NAN, 1.0f, &m) == mani_errorInvalid &&
              maniClip_GetRootMotion(&clip, 0.0f, INFINITY, &m) == mani_errorInvalid,
          "misuses");
}

static void TestBlend(void)
{
    // Thirty degrees either way about y, a metre along x and along z.
    const maniRootMove moves[3] = {
        {{1.0f, 0.0f, 0.0f}, {0.0f, 0.25881905f, 0.0f, 0.96592583f}},
        {{0.0f, 0.0f, 1.0f}, {0.0f, -0.25881905f, 0.0f, 0.96592583f}},
        // The second's rotation negated: the same turn.
        {{0.0f, 0.0f, 1.0f}, {0.0f, 0.25881905f, 0.0f, -0.96592583f}},
    };
    const float even[2] = {2.0f, 2.0f};
    maniRootMove out;
    CHECK(maniBlendRootMotion(moves, even, 2, &out) == mani_success &&
              MoveApart(out, (maniRootMove){{0.5f, 0.0f, 0.5f}, {0.0f, 0.0f, 0.0f, 1.0f}}) < 1e-6f,
          "even weights: the mean translation, no turn");
    const float skip[3] = {1.0f, 0.0f, 1.0f};
    CHECK(maniBlendRootMotion(moves, skip, 3, &out) == mani_success &&
              MoveApart(out, (maniRootMove){{0.5f, 0.0f, 0.5f}, {0.0f, 0.0f, 0.0f, 1.0f}}) < 1e-6f,
          "a negated rotation turned into the first's hemisphere; a weight of 0 left out");
    // A move of weight 0 first, turned against the second: the second,
    // not it, leads the hemispheres (else a half turn).
    const maniRootMove turned[3] = {
        {{0.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f, 0.8660254f}},
        {{0.0f, 0.0f, 0.0f}, {0.0f, -0.5f, 0.0f, 0.8660254f}},
    };
    const float later[3] = {0.0f, 1.0f, 1.0f};
    CHECK(maniBlendRootMotion(turned, later, 3, &out) == mani_success &&
              MoveApart(out, maniGetIdentityRootMove()) < 1e-6f,
          "a first move of weight 0 does not lead");
    const float one[2] = {0.0f, 3.0f};
    CHECK(maniBlendRootMotion(moves, one, 2, &out) == mani_success &&
              MoveApart(out, moves[1]) < 1e-6f,
          "one weight: that move");
    const float none[2] = {0.0f, 0.0f};
    CHECK(maniBlendRootMotion(moves, none, 2, &out) == mani_success &&
              MoveApart(out, maniGetIdentityRootMove()) == 0.0f &&
              maniBlendRootMotion(NULL, NULL, 0, &out) == mani_success,
          "no weight: no move");
    const float negative[2] = {1.0f, -1.0f};
    const float nan[2] = {1.0f, NAN};
    maniRootMove bad[2] = {moves[0], moves[1]};
    bad[1].translation.y = INFINITY;
    CHECK(maniBlendRootMotion(moves, negative, 2, &out) == mani_errorInvalid &&
              maniBlendRootMotion(moves, nan, 2, &out) == mani_errorInvalid &&
              maniBlendRootMotion(bad, even, 2, &out) == mani_errorInvalid &&
              maniBlendRootMotion(NULL, even, 2, &out) == mani_errorInvalid &&
              maniBlendRootMotion(moves, even, 2, NULL) == mani_errorInvalid,
          "misuses");
    maniRootMove id = maniGetIdentityRootMove();
    CHECK(MoveApart(maniComposeRootMotion(id, moves[0]), moves[0]) < 1e-7f &&
              MoveApart(maniComposeRootMotion(moves[0], id), moves[0]) < 1e-7f,
          "composing with no move");
}

static float TransformApart(maniTransform a, maniTransform b)
{
    if (!isfinite(a.translation.x + a.translation.y + a.translation.z + a.rotation.x +
                  a.rotation.y + a.rotation.z + a.rotation.w + a.scale.x) ||
        !isfinite(b.translation.x + b.translation.y + b.translation.z + b.rotation.x +
                  b.rotation.y + b.rotation.z + b.rotation.w + b.scale.x))
    {
        return INFINITY;
    }
    float worst = fmaxf(
        fabsf(a.translation.x - b.translation.x),
        fmaxf(fabsf(a.translation.y - b.translation.y), fabsf(a.translation.z - b.translation.z)));
    float dot = a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y +
                a.rotation.z * b.rotation.z + a.rotation.w * b.rotation.w;
    worst = fmaxf(worst, 1.0f - fabsf(dot));
    return fmaxf(worst, fabsf(a.scale.x - b.scale.x));
}

static bool SameTransform(maniTransform a, maniTransform b)
{
    return a.translation.x == b.translation.x && a.translation.y == b.translation.y &&
           a.translation.z == b.translation.z && a.rotation.x == b.rotation.x &&
           a.rotation.y == b.rotation.y && a.rotation.z == b.rotation.z &&
           a.rotation.w == b.rotation.w && a.scale.x == b.scale.x && a.scale.y == b.scale.y &&
           a.scale.z == b.scale.z;
}

// Restored at the samples, the hips are the source's; between them close
// to it; the other bones untouched.
static void TestRestore(void)
{
    maniRootMotionSource walk = Walk();
    maniClip clip;
    maniClip without;
    maniPose* a = NULL;
    maniPose* b = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &a) == mani_success &&
              maniInitPose(&s_skeleton, s_plainMemory, sizeof s_plainMemory, &b) == mani_success,
          "poses");
    static const float kAt[7] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f, 9.0f, -1.0f};
    float worst = 0.0f;
    // Between samples the motion and the stored bone interpolate apart:
    // close for the twist, about a centimetre for the whole rotation.
    float between[2] = {0.0f, 0.0f};
    // The twist about y, then the whole rotation: its pitch turns the
    // stored translation off y, so the restore must turn it back.
    for (int i = 0; i < 18; ++i)
    {
        if (i % 9 == 0)
        {
            walk.rotation = i == 0 ? maniRootRotationTwist : maniRootRotationWhole;
            CHECK(BuildBoth(&walk) && OpenBoth(&clip, &without), "builds");
        }
        int j = i % 9;
        float time = j < 7 ? kAt[j] : 0.1f + 0.4f * (float)(j - 7);
        CHECK(maniClip_Sample(&clip, &s_skeleton, time, a) == mani_success &&
                  maniClip_RestoreRootMotion(&clip, &s_skeleton, time, a) == mani_success &&
                  maniClip_Sample(&without, &s_skeleton, time, b) == mani_success,
              "restores");
        float apart = TransformApart(maniPose_GetLocal(a, 0), maniPose_GetLocal(b, 0));
        if (j < 7)
        {
            worst = fmaxf(worst, apart);
        }
        else
        {
            between[i / 9] = fmaxf(between[i / 9], apart);
        }
        worst = fmaxf(worst, TransformApart(maniPose_GetLocal(a, 1), maniPose_GetLocal(b, 1)));
    }
    CHECK(worst < 2e-6f, "restored at the samples: the source's hips");
    CHECK(between[0] < 1e-4f && between[1] < 2e-2f,
          "restored between samples: close to the source's hips");
    if (worst >= 2e-6f || between[0] >= 1e-4f || between[1] >= 2e-2f)
    {
        printf("  restored apart by %g, between by %g and %g\n", (double)worst, (double)between[0],
               (double)between[1]);
    }
    // Without root motion: unchanged.
    CHECK(maniClip_Sample(&without, &s_skeleton, 0.3f, b) == mani_success, "samples");
    maniTransform before = maniPose_GetLocal(b, 0);
    CHECK(maniClip_RestoreRootMotion(&without, &s_skeleton, 0.3f, b) == mani_success &&
              SameTransform(before, maniPose_GetLocal(b, 0)),
          "a clip without root motion leaves the pose");
    CHECK(maniClip_RestoreRootMotion(NULL, &s_skeleton, 0.0f, a) == mani_errorInvalid &&
              maniClip_RestoreRootMotion(&clip, NULL, 0.0f, a) == mani_errorInvalid &&
              maniClip_RestoreRootMotion(&clip, &s_skeleton, 0.0f, NULL) == mani_errorInvalid,
          "misuses");
    maniPoseSetKind(a, true);
    before = maniPose_GetLocal(a, 0);
    CHECK(maniClip_RestoreRootMotion(&clip, &s_skeleton, 0.5f, a) == mani_errorInvalid &&
              SameTransform(before, maniPose_GetLocal(a, 0)),
          "a difference pose refused, untouched");
}

// A twist about x keeps only x and w; a walk that starts turned has a
// period from its first sample's frame.
static void TestStartTurned(void)
{
    maniRootMotionSource roll = Walk();
    roll.twistAxis = 0;
    maniClip clip;
    maniClip without;
    float m[7];
    CHECK(BuildBoth(&roll) && OpenBoth(&clip, &without), "builds");
    MotionAt(&clip, 4, m);
    CHECK(m[3] > 0.1f && m[4] == 0.0f && m[5] == 0.0f, "the twist about x");
    // From a quarter turn about y to a half turn.
    static const float kTurned[8] = {0, 0.70710678f, 0, 0.70710678f, 0, 1, 0, 0};
    maniRootMotionSource walk = Walk();
    maniClipSource source = Source(&walk);
    s_tracks[0].rotation.values = kTurned;
    CHECK(Build(&source, s_file, &s_size).result == mani_success &&
              maniOpenClip(s_file, s_size, &clip).result == mani_success,
          "builds turned");
    maniRootMove period =
        maniComposeRootMotion(Inverted(SampleMove(&clip, 0)), SampleMove(&clip, 4));
    maniRootMove twice = Move(&clip, 0.0f, 2.0f);
    CHECK(MoveApart(twice, maniComposeRootMotion(period, period)) < 1e-5f &&
              fabsf(twice.rotation.w) < 1e-5f,
          "two periods of a walk that starts turned: a half turn");
}

static void ExpectRefused(const maniRootMotionSource* root, maniAdditiveBasis additive,
                          const char* what)
{
    maniClipSource source = Source(root);
    source.additive = additive;
    size_t size = 0;
    maniBuildResult r = Build(&source, s_crafted, &size);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonRootMotion &&
              r.item == root->bone,
          what);
}

static void TestBuildRefusals(void)
{
    maniRootMotionSource s = Walk();
    s.bone = 3;
    ExpectRefused(&s, maniAdditiveNone, "a bone outside the skeleton");
    s.bone = 2;
    ExpectRefused(&s, maniAdditiveNone, "a bone without a track");
    s = Walk();
    s.translationAxes = 0;
    s.rotation = maniRootRotationNone;
    ExpectRefused(&s, maniAdditiveNone, "no channel chosen");
    const uint32_t kField[6] = {8, 3, 3, 3, 3, 2};
    for (int i = 0; i < 6; ++i)
    {
        s = Walk();
        uint32_t* fields = &s.translationAxes;
        fields[i] = kField[i];
        ExpectRefused(&s, maniAdditiveNone, "a field out of range");
    }
    s = Walk();
    ExpectRefused(&s, maniAdditiveRest, "an additive clip");
    // A walk of 2^65 along x: its translation past 2^64.
    static const float kFar[6] = {0, 1, 0, 36893488147419103232.0f, 1, 0};
    maniClipSource far = Source(&s);
    s_tracks[0].translation.values = kFar;
    size_t size = 0;
    maniBuildResult r = Build(&far, s_crafted, &size);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonRootMotion && r.item == 0,
          "a translation past 2^64");
    // Along y, which the walk does not take: built.
    static const float kHigh[6] = {0, 1, 0, 0, 36893488147419103232.0f, 0};
    s_tracks[0].translation.values = kHigh;
    CHECK(Build(&far, s_crafted, &size).result == mani_success, "a translation not taken");
}

// Root motion given to the clip builder, as retargeting bakes it: the
// moves written as they come and the bone's channels kept; refused
// beside the source's own, on a bone without keyed translation and
// rotation, in an additive clip, or with a move count not the samples'.
static void TestGiven(void)
{
    static double moves[5 * 7];
    for (uint32_t k = 0; k < 5; ++k)
    {
        const double move[7] = {0.25 * k, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        memcpy(moves + 7 * k, move, sizeof move);
    }
    maniRootMotionWork work = {Walk(), moves, 5};
    maniClipSource source = Source(NULL);
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniClip clip;
    maniRootMove move;
    maniPose* pose = NULL;
    CHECK(maniBuildClipMoved(builder, &source, &work).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_crafted, sizeof s_crafted, &size) == mani_success &&
              maniOpenClip(s_crafted, size, &clip).result == mani_success &&
              maniClip_GetRootMotion(&clip, 0.0f, 1.0f, &move) == mani_success &&
              fabsf(move.translation.x - 1.0f) < 1e-6f &&
              maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success &&
              maniClip_Sample(&clip, &s_skeleton, 1.0f, pose) == mani_success &&
              maniPose_GetLocal(pose, 0).translation.z == 4.0f,
          "given moves written, the bone kept");
    maniRootMotionSource own = Walk();
    source.rootMotion = &own;
    maniBuildResult r = maniBuildClipMoved(builder, &source, &work);
    CHECK(r.reason == maniBuildReasonRootMotion, "given beside the source's own");
    source.rootMotion = NULL;
    work.source.bone = 1;
    r = maniBuildClipMoved(builder, &source, &work);
    CHECK(r.reason == maniBuildReasonRootMotion && r.item == 1, "a bone without translation keys");
    work.source.bone = 0;
    source.additive = maniAdditiveRest;
    r = maniBuildClipMoved(builder, &source, &work);
    CHECK(r.reason == maniBuildReasonRootMotion, "an additive clip");
    source.additive = maniAdditiveNone;
    work.sampleCount = 4;
    r = maniBuildClipMoved(builder, &source, &work);
    CHECK(r.reason == maniBuildReasonRootMotion, "a move short");
    maniDestroyBuilder(builder);
}

// Where a section of s_file begins.
static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindClip, MANI_CLIP_FORMAT_VERSION, &c).result ==
                  mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

// The payload hash and the identity made right after an edit.
static void Rehash(uint8_t* file)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + MANI_CONTAINER_HEADER_SIZE,
                                          s_size - MANI_CONTAINER_HEADER_SIZE));
    maniContainer c;
    CHECK(maniOpenContainer(file, s_size, maniKindClip, MANI_CLIP_FORMAT_VERSION, &c).result ==
              mani_success,
          "container");
    const uint8_t* sections[MANI_CLIP_ALL_SECTIONS];
    uint64_t sizes[MANI_CLIP_ALL_SECTIONS];
    for (uint32_t t = 1; t <= MANI_CLIP_ALL_SECTIONS; ++t)
    {
        uint64_t at = 0;
        sections[t - 1] = file;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &sections[t - 1], &sizes[t - 1], &at);
    }
    maniStoreU64(file + 40, maniClipComputeIdentity(sections, sizes, MANI_CLIP_ALL_SECTIONS));
}

static void ExpectFormat(uint64_t offset, uint32_t value, uint64_t want, const char* what)
{
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + offset, value);
    Rehash(s_crafted);
    maniClip clip;
    maniFileResult r = maniOpenClip(s_crafted, s_size, &clip);
    CHECK(r.result == mani_errorFormat && r.offset == want, what);
    if (r.offset != want)
    {
        printf("  %s: at %llu, expected %llu\n", what, (unsigned long long)r.offset,
               (unsigned long long)want);
    }
}

static void TestOpenRefusals(void)
{
    maniRootMotionSource walk = Walk();
    CHECK(BuildBoth(&walk), "builds");
    uint64_t at = SectionAt(maniClipTagRootMotion);
    ExpectFormat(at, 2, at, "a bone without a track");
    ExpectFormat(at, 9, at, "a bone outside the skeleton");
    ExpectFormat(at, 1, at, "a bone whose track lacks a channel");
    ExpectFormat(at + 4, 8, at, "axes out of range");
    ExpectFormat(at + 8, 3, at, "a rotation out of range");
    ExpectFormat(at + 12, 3, at, "a twist axis out of range");
    ExpectFormat(at + 16, 3, at, "a reference out of range");
    ExpectFormat(at + 20, 3, at, "a reference out of range");
    ExpectFormat(at + 24, 2, at, "a loop out of range");
    // The period's move, then each sample's: seven floats each.
    uint64_t period = at + MANI_ROOT_MOTION_HEADER_SIZE;
    uint64_t samples = period + MANI_ROOT_MOTION_PERIOD_SIZE;
    ExpectFormat(samples + 12, 0x7F800000u, samples + 12, "a move not finite");
    // 2^65 (0x60000000) in a translation, the period's or a sample's.
    ExpectFormat(period, 0x60000000u, period, "a period translation past 2^64");
    ExpectFormat(samples + 28 + 8, 0x60000000u, samples + 28 + 8, "a sample translation past 2^64");
    ExpectFormat(samples + 28 + 4, 0xE0000000u, samples + 28 + 4,
                 "a negative translation past 2^64");
    // A translation of 2^64 itself, and 2^65 in a rotation: opened.
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + samples, 0x5F800000u);
    maniStoreU32(s_crafted + samples + 28 + 12, 0x60000000u);
    Rehash(s_crafted);
    maniClip clip;
    CHECK(maniOpenClip(s_crafted, s_size, &clip).result == mani_success,
          "2^64 in a translation, 2^65 in a rotation");
    // The additive basis in the header: root motion on a difference clip.
    ExpectFormat(SectionAt(maniClipTagHeader) + 60, 1, at, "an additive clip");
}

static int WriteSeed(const char* path)
{
    maniRootMotionSource walk = Walk();
    FILE* f = BuildBoth(&walk) ? fopen(path, "wb") : NULL;
    bool written = f != NULL && fwrite(s_file, 1, s_size, f) == s_size;
    if (f != NULL)
    {
        written = fclose(f) == 0 && written;
    }
    return written ? 0 : 1;
}

int main(int argc, char** argv)
{
    OpenSkeleton();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestWalk();
    TestReferences();
    TestLoop();
    TestCompressed();
    TestMoves();
    TestBlend();
    TestRestore();
    TestStartTurned();
    TestBuildRefusals();
    TestGiven();
    TestOpenRefusals();
    return s_failures == 0 ? 0 : 1;
}
