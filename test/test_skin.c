// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skins: building, opening, every refusal, and palettes: the identity at
// the bind pose, the model times the inverse bind elsewhere, refused for a
// stale model or another skeleton. Run as test_skin --seed <file> it
// writes the fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "pose_memory.h"
#include "skin_file.h"
#include "source_padding.h"
#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

enum
{
    kBones = 6,
    kJoints = 4,
};

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_otherFile[4096];
static _Alignas(16) uint8_t s_file[1024];
static size_t s_size;
static _Alignas(32) uint8_t s_poseMemory[4096];
static maniSkeleton s_skeleton;

#define BONE(n, p, tx, ty, qy, qw, sx)                                                             \
    {                                                                                              \
        n, sizeof(n) - 1, p,                                                                       \
        {                                                                                          \
            {tx, ty, 0.0f}, {0.0f, qy, 0.0f, qw}, {sx, 1.0f, 1.0f}                                 \
        }                                                                                          \
    }

static const maniBoneSource kBonesSource[kBones] = {
    BONE("root", -1, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f),
    BONE("spine", 0, 0.0f, 0.5f, 0.38268343f, 0.92387953f, 1.0f),
    BONE("arm", 1, 0.3f, 0.2f, 0.0f, 1.0f, 1.5f),
    BONE("hand", 2, 0.4f, 0.0f, 0.70710677f, 0.70710677f, 1.0f),
    BONE("leg", 0, -0.2f, -0.5f, 0.0f, 1.0f, -1.0f),
    BONE("foot", 4, 0.0f, -0.6f, 0.0f, 1.0f, 1.0f),
};

static const uint32_t kJointBones[kJoints] = {3, 1, 5, 0};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static maniSkeleton OpenSkeleton(const maniBoneSource* bones, uint8_t* file, size_t capacity)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, kBones};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "skeleton builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, file, capacity, &size) == mani_success, "skeleton copies");
    maniDestroyBuilder(builder);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(file, size, &skeleton).result == mani_success, "skeleton opens");
    return skeleton;
}

// The inverse binds of the rest pose, so the palette at rest is the
// identity.
static void RestInverseBinds(maniAffine inverseBinds[kJoints])
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        maniAffine model = maniPoseModel(pose, kJointBones[j]);
        CHECK(maniAffineInverse(&model, &inverseBinds[j]), "invertible");
    }
}

static void BuildSample(void)
{
    s_skeleton = OpenSkeleton(kBonesSource, s_skeletonFile, sizeof s_skeletonFile);
    maniAffine inverseBinds[kJoints];
    RestInverseBinds(inverseBinds);
    maniBuilder* builder = NewBuilder();
    maniSkinSource source = {&s_skeleton, kJointBones, inverseBinds, kJoints};
    CHECK(maniBuildSkin(builder, &source).result == mani_success, "skin builds");
    CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success, "skin copies");
    maniDestroyBuilder(builder);
}

static maniFileResult Open(void)
{
    maniSkin skin;
    return maniOpenSkin(s_file, s_size, &skin);
}

static void TestOpensAndReads(void)
{
    BuildSample();
    maniSkin skin;
    CHECK(maniOpenSkin(s_file, s_size, &skin).result == mani_success, "opens");
    CHECK(maniSkinJointCount(&skin) == kJoints, "count");
    CHECK(maniSkinSkeletonIdentity(&skin) == maniSkeletonIdentity(&s_skeleton), "bound");
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        CHECK(maniSkinJoint(&skin, j) == kJointBones[j], "joint");
    }
    CHECK(maniSkinJoint(&skin, kJoints) == MANI_NO_BONE, "joint past");
    maniAffine past = maniSkinInverseBind(&skin, kJoints);
    CHECK(past.m[0] == 1.0f && past.m[4] == 1.0f && past.m[9] == 0.0f, "identity past");
}

static void TestPaletteAtBindIsIdentity(void)
{
    BuildSample();
    maniSkin skin;
    CHECK(maniOpenSkin(s_file, s_size, &skin).result == mani_success, "opens");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    maniAffine palette[kJoints];
    uint32_t count = 0;
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, palette, kJoints, &count) == mani_success,
          "palette");
    CHECK(count == kJoints, "count");
    static const float identity[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        for (int k = 0; k < 12; ++k)
        {
            CHECK(fabsf(palette[j].m[k] - identity[k]) < 1e-6f, "identity at bind");
        }
    }
}

static void TestPaletteIsModelTimesInverseBind(void)
{
    BuildSample();
    maniSkin skin;
    CHECK(maniOpenSkin(s_file, s_size, &skin).result == mani_success, "opens");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    maniPoseSetLocal(
        pose, 1,
        (maniTransform){{0.1f, 0.6f, -0.2f}, {0.0f, 0.0f, 0.6f, 0.8f}, {1.2f, 0.8f, 1.0f}});
    maniAffine palette[kJoints];
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, palette, kJoints, NULL) == mani_errorInvalid,
          "stale model");
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "composes");
    // A difference has no model to skin with, even one left current.
    maniPoseSetKind(pose, true);
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, palette, kJoints, NULL) == mani_errorInvalid,
          "a difference");
    maniPoseSetKind(pose, false);
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, palette, kJoints, NULL) == mani_success,
          "palette");
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        maniAffine model = maniPoseModel(pose, kJointBones[j]);
        maniAffine inverseBind = maniSkinInverseBind(&skin, j);
        maniAffine expected = maniAffineMul(&model, &inverseBind);
        CHECK(memcmp(&expected, &palette[j], sizeof expected) == 0, "model times inverse bind");
    }
    uint32_t count = 0;
    maniAffine three[kJoints - 1];
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, three, kJoints - 1, &count) ==
                  mani_errorCapacity &&
              count == kJoints,
          "one short");
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, palette, 2, &count) == mani_errorCapacity &&
              count == kJoints,
          "short array");
    CHECK(maniComputePalette(pose, &s_skeleton, &skin, NULL, 0, &count) == mani_errorCapacity,
          "count only");
    CHECK(maniComputePalette(pose, &s_skeleton, NULL, palette, kJoints, NULL) == mani_errorInvalid,
          "null skin");

    // Another skeleton: the same bones with one rest moved.
    maniBoneSource moved[kBones];
    memcpy(moved, kBonesSource, sizeof moved);
    moved[5].rest.translation.x = 0.5f;
    maniSkeleton other = OpenSkeleton(moved, s_otherFile, sizeof s_otherFile);
    CHECK(maniComputePalette(pose, &other, &skin, palette, kJoints, NULL) == mani_errorInvalid,
          "the pose's skeleton is not this one");
    static _Alignas(32) uint8_t otherPose[4096];
    maniPose* second = NULL;
    CHECK(maniInitPose(&other, otherPose, sizeof otherPose, &second) == mani_success, "pose");
    CHECK(maniComputePalette(second, &other, &skin, palette, kJoints, NULL) == mani_errorInvalid,
          "the skin's skeleton is not this one");
}

static maniBuildResult BuildWith(const uint32_t* joints, const maniAffine* binds, uint32_t count)
{
    maniBuilder* builder = NewBuilder();
    maniSkinSource source = {&s_skeleton, joints, binds, count};
    maniBuildResult r = maniBuildSkin(builder, &source);
    maniDestroyBuilder(builder);
    return r;
}

static void TestBuildRefusals(void)
{
    BuildSample();
    maniAffine binds[kJoints];
    RestInverseBinds(binds);
    maniBuildResult r = BuildWith(kJointBones, binds, 0);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonCount, "no joints");
    uint32_t joints[kJoints] = {3, 1, kBones, 0};
    r = BuildWith(joints, binds, kJoints);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonJoint && r.item == 2,
          "a bone past the skeleton");
    binds[3].m[10] = NAN;
    r = BuildWith(kJointBones, binds, kJoints);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonNotFinite && r.item == 3,
          "a matrix not finite");
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildSkin(builder, NULL).result == mani_errorInvalid, "null source");
    maniSkinSource noSkeleton = {NULL, kJointBones, binds, kJoints};
    CHECK(maniBuildSkin(builder, &noSkeleton).result == mani_errorInvalid, "null skeleton");
    CHECK(maniBuilderMisuses(builder) == 2, "two misuses");
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

static void TestOpenRefusals(void)
{
    BuildSample();
    uint64_t binding = SectionAt(maniSkinTagBinding);
    uint64_t joints = SectionAt(maniSkinTagJoints);
    uint64_t binds = SectionAt(maniSkinTagInverseBinds);
    maniStoreU32(s_file + binding + 8, 0);
    ExpectFormat(binding + 8, "no bones");
    maniStoreU32(s_file + binding + 8, MANI_MAX_BONES + 1);
    ExpectFormat(binding + 8, "too many bones");
    maniStoreU32(s_file + binding + 12, 0);
    ExpectFormat(binding + 12, "no joints");
    maniStoreU32(s_file + binding + 12, kJoints + 1);
    ExpectFormat(64 + 24 + 16, "joints of the wrong size");
    maniStoreU16(s_file + joints + 4, kBones);
    ExpectFormat(joints + 4, "a joint past the bones");
    float nan = NAN;
    memcpy(s_file + binds + 48 + 8, &nan, 4);
    ExpectFormat(binds + 56, "a matrix not finite");
    maniStoreU64(s_file + 64 + 16, 15);
    ExpectFormat(64 + 16, "a binding of the wrong size");
    s_file[41] ^= 1;
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == 40, "identity");
    BuildSample();
    maniSkin skin;
    CHECK(maniOpenSkin(s_file, s_size, NULL).result == mani_errorInvalid, "null view");
    CHECK(maniOpenSkin(s_skeletonFile, sizeof s_skeletonFile, &skin).result == mani_errorFormat,
          "a skeleton is not a skin");
}

// Rewrites the sample from its sections with one replaced or dropped.
static void Rewrite(uint32_t tag, const void* data, uint64_t size)
{
    static _Alignas(16) uint8_t copy[1024];
    memcpy(copy, s_file, s_size);
    maniSectionSource sections[MANI_SKIN_SECTIONS];
    uint32_t count = 0;
    for (uint32_t t = 1; t <= MANI_SKIN_SECTIONS; ++t)
    {
        uint64_t at = 64 + (uint64_t)(t - 1) * 24;
        if (t == tag && data == NULL)
        {
            continue;
        }
        sections[count++] = t == tag ? (maniSectionSource){t, data, size}
                                     : (maniSectionSource){t, copy + maniLoadU64(copy + at + 8),
                                                           maniLoadU64(copy + at + 16)};
    }
    uint64_t total = 0;
    CHECK(maniContainerSize(sections, count, &total) && total <= sizeof s_file, "fits");
    maniWriteContainer(s_file, maniKindSkin, 1, maniLoadU64(copy + 40), sections, count);
    s_size = (size_t)total;
}

static void TestTableRefusals(void)
{
    BuildSample();
    Rewrite(maniSkinTagInverseBinds, NULL, 0);
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == 12, "two sections");
    BuildSample();
    static const uint8_t binds[48 * (kJoints - 1)] = {0};
    Rewrite(maniSkinTagInverseBinds, binds, sizeof binds);
    r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == 64 + 2 * 24 + 16, "binds of the wrong size");
    BuildSample();
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

// The same file whatever bytes pad the source.
static void TestPaddingIgnored(void)
{
    BuildSample();
    maniAffine inverseBinds[kJoints];
    RestInverseBinds(inverseBinds);
    uint8_t* built[2];
    size_t sizes[2];
    for (int k = 0; k < 2; ++k)
    {
        maniSkinSource source = {&s_skeleton, kJointBones, inverseBinds, kJoints};
        TEST_PAD(&source, 1, kFieldsSkinSource, kTestPadPatterns[k]);
        maniBuilder* builder = NewBuilder();
        CHECK(maniBuildSkin(builder, &source).result == mani_success, "builds padded");
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
    TestOpensAndReads();
    TestPaletteAtBindIsIdentity();
    TestPaletteIsModelTimesInverseBind();
    TestBuildRefusals();
    TestOpenRefusals();
    TestTableRefusals();
    TestEveryByteFlip();
    TestPaddingIgnored();
    return s_failures == 0 ? 0 : 1;
}
