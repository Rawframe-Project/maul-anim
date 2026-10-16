// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion at build (mani-0022): the motion taken from a bone's local
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
#include "clip_file.h"
#include "container.h"
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

#define PINNED_ROOT_HASH 0xcfd51c694f3e10e9ull

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
    maniSkeletonSource source = {kBones, 3, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
          "skeleton");
    maniDestroyBuilder(builder);
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
}

// The walk, at 4 samples a second: horizontal translation and the twist
// about y from the rest.
static maniClipSource Source(const maniRootMotionSource* root)
{
    s_tracks[0] = (maniBoneTrackSource){0,
                                        0,
                                        {kTimes, kWalk, 2, maniInterpolationLinear},
                                        {kTimes, kTurn, 2, maniInterpolationLinear},
                                        kNone};
    s_tracks[1] =
        (maniBoneTrackSource){1, 0, kNone, {kTimes, kSway, 2, maniInterpolationLinear}, kNone};
    return (maniClipSource){.skeleton = &s_skeleton,
                            .boneTracks = s_tracks,
                            .boneTrackCount = 2,
                            .loop = maniLoopRepeat,
                            .sampleRate = 4.0f,
                            .rootMotion = root};
}

static maniRootMotionSource Walk(void)
{
    return (maniRootMotionSource){0,
                                  maniRootAxisX | maniRootAxisZ,
                                  maniRootRotationTwist,
                                  1,
                                  maniRootReferenceRest,
                                  maniRootReferenceRest,
                                  0,
                                  0};
}

static maniBuildResult Build(const maniClipSource* source, uint8_t* file, size_t* size)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildClip(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, file, sizeof s_file, size) == mani_success, "copies");
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
    for (uint32_t k = 0; k < maniClipSampleCount(with); ++k)
    {
        float time = (float)k / 4.0f;
        CHECK(maniSampleClip(with, &s_skeleton, time, a) == mani_success &&
                  maniSampleClip(without, &s_skeleton, time, b) == mani_success,
              "samples");
        float m[7];
        MotionAt(with, k, m);
        maniTransform stored = maniPoseLocal(a, 0);
        maniTransform source = maniPoseLocal(b, 0);
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
        maniTransform spine = maniPoseLocal(a, 1);
        maniTransform plain = maniPoseLocal(b, 1);
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
    CHECK(maniClipRootMotionSource(&with, &read) && memcmp(&read, &walk, sizeof read) == 0 &&
              !maniClipRootMotionSource(&without, &read) &&
              !maniClipRootMotionSource(NULL, &read) && !maniClipRootMotionSource(&with, NULL),
          "the source read back; none without");
    float worst = Recomposed(&with, &without);
    CHECK(worst < 2e-6f, "the motion times the stored bone is the source");
    if (worst >= 2e-6f)
    {
        printf("  apart by %g\n", (double)worst);
    }
    // Along x and z only, a twist about y only.
    bool shaped = true;
    for (uint32_t k = 0; k < maniClipSampleCount(&with); ++k)
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
              maniSampleClip(&with, &s_skeleton, 0.5f, pose) == mani_success,
          "samples");
    maniTransform hips = maniPoseLocal(pose, 0);
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
              maniCopyBuilt(builder, s_crafted, sizeof s_crafted, &size) == mani_success &&
              maniOpenClip(s_crafted, size, &compressed).result == mani_success,
          "compresses");
    maniDestroyBuilder(builder);
    CHECK(compressed.rootMotion != NULL &&
              memcmp(compressed.rootMotion, clip.rootMotion, maniRootMotionSize(5)) == 0,
          "the compressor carries the root motion unchanged");
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
    const uint32_t kField[7] = {8, 3, 3, 3, 3, 2, 1};
    for (int i = 0; i < 7; ++i)
    {
        s = Walk();
        uint32_t* fields = &s.translationAxes;
        fields[i] = kField[i];
        ExpectRefused(&s, maniAdditiveNone, "a field out of range");
    }
    s = Walk();
    ExpectRefused(&s, maniAdditiveRest, "an additive clip");
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
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + 64, s_size - 64));
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
    ExpectFormat(at + 28, 1, at, "a header's last word not zero");
    ExpectFormat(at + 60, 1, at + 60, "the period's last word not zero");
    ExpectFormat(at + 64 + 12, 0x7F800000u, at + 76, "a move not finite");
    // The additive basis in the header: root motion on a difference clip.
    ExpectFormat(SectionAt(maniClipTagHeader) + 60, 1, at, "an additive clip");
}

static void TestPinned(void)
{
    maniRootMotionSource walk = Walk();
    CHECK(BuildBoth(&walk), "builds");
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_ROOT_HASH, "pinned file");
    if (hash != PINNED_ROOT_HASH)
    {
        printf("root motion clip hash 0x%016llx\n", (unsigned long long)hash);
    }
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
    TestBuildRefusals();
    TestOpenRefusals();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
