// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Poses: memory, locals, and the model pass against glTF's matrix
// composition computed in binary64, over random hierarchies with mirrors
// and non-uniform scales; the results pinned.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <string.h>

enum
{
    kBones = 37,
    kFileCapacity = 16384,
    kPoseCapacity = 8192,
};

static _Alignas(16) uint8_t s_file[kFileCapacity];
static _Alignas(32) uint8_t s_poseMemory[kPoseCapacity];
static _Alignas(32) uint8_t s_otherMemory[kPoseCapacity];
static char s_names[kBones][8];

static uint64_t s_random = 0x9E3779B97F4A7C15ull;

static uint32_t Next(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (uint32_t)(s_random >> 32);
}

// A value in [low, high), from 24 random bits so it is a float exactly.
static float Uniform(float low, float high)
{
    return low + (high - low) * (float)(Next() >> 8) * 0x1p-24f;
}

static maniQuat RandomRotation(void)
{
    double x = (double)Uniform(-1.0f, 1.0f);
    double y = (double)Uniform(-1.0f, 1.0f);
    double z = (double)Uniform(-1.0f, 1.0f);
    double w = (double)Uniform(-1.0f, 1.0f);
    double length = sqrt(x * x + y * y + z * z + w * w);
    return (maniQuat){(float)(x / length), (float)(y / length), (float)(z / length),
                      (float)(w / length)};
}

static maniTransform RandomTransform(void)
{
    maniTransform t;
    t.translation = (maniVec3){Uniform(-2.0f, 2.0f), Uniform(-2.0f, 2.0f), Uniform(-2.0f, 2.0f)};
    t.rotation = RandomRotation();
    // Non-uniform scales, one bone in four mirrored on an axis.
    t.scale = (maniVec3){Uniform(0.5f, 1.5f), Uniform(0.5f, 1.5f), Uniform(0.5f, 1.5f)};
    if (Next() % 4 == 0)
    {
        t.scale.y = -t.scale.y;
    }
    return t;
}

// A random hierarchy (each bone's parent any earlier bone), built and
// opened.
static maniSkeleton BuildRandomSkeleton(uint32_t count, uint8_t* file, size_t capacity)
{
    maniBoneSource bones[kBones];
    for (uint32_t i = 0; i < count; ++i)
    {
        s_names[i][0] = 'b';
        s_names[i][1] = (char)('a' + i / 26);
        s_names[i][2] = (char)('a' + i % 26);
        bones[i].name = s_names[i];
        bones[i].nameLength = 3;
        bones[i].parent = i == 0 ? -1 : (int32_t)(Next() % i);
        bones[i].rest = RandomTransform();
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    maniSkeletonSource source = {bones, count};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds");
    size_t size = 0;
    CHECK(maniCopyBuilt(builder, file, capacity, &size) == mani_success, "copies");
    maniDestroyBuilder(builder);
    maniSkeleton skeleton;
    CHECK(maniOpenSkeleton(file, size, &skeleton).result == mani_success, "opens");
    return skeleton;
}

// glTF's composition in binary64: column-major 3x4, scale, rotation,
// translation.
typedef struct Matrix
{
    double m[12];
} Matrix;

static Matrix FromTransform(const maniTransform* t)
{
    double x = (double)t->rotation.x;
    double y = (double)t->rotation.y;
    double z = (double)t->rotation.z;
    double w = (double)t->rotation.w;
    double r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),
                   2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
                   2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y)};
    double s[3] = {(double)t->scale.x, (double)t->scale.y, (double)t->scale.z};
    Matrix out;
    for (int c = 0; c < 3; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            out.m[c * 3 + k] = r[c * 3 + k] * s[c];
        }
    }
    out.m[9] = (double)t->translation.x;
    out.m[10] = (double)t->translation.y;
    out.m[11] = (double)t->translation.z;
    return out;
}

static Matrix Multiply(const Matrix* a, const Matrix* b)
{
    Matrix out;
    for (int c = 0; c < 4; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            double sum = c == 3 ? a->m[9 + k] : 0.0;
            for (int j = 0; j < 3; ++j)
            {
                sum += a->m[j * 3 + k] * b->m[c * 3 + j];
            }
            out.m[c * 3 + k] = sum;
        }
    }
    return out;
}

static void TestMemory(void)
{
    maniSkeleton s = BuildRandomSkeleton(kBones, s_file, sizeof s_file);
    size_t size = maniPoseSize(&s);
    CHECK(size % MANI_POSE_ALIGNMENT == 0 && size <= kPoseCapacity, "size");
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s, s_poseMemory + 16, size, &pose) == mani_errorInvalid, "misaligned");
    CHECK(maniInitPose(&s, s_poseMemory, size - 1, &pose) == mani_errorCapacity, "too small");
    CHECK(maniInitPose(NULL, s_poseMemory, size, &pose) == mani_errorInvalid, "null skeleton");
    CHECK(maniInitPose(&s, s_poseMemory, size, NULL) == mani_errorInvalid, "null out");
    CHECK(maniInitPose(&s, NULL, size, &pose) == mani_errorInvalid, "null memory");
    CHECK(maniInitPose(&s, s_poseMemory, size, &pose) == mani_success, "inits");
    CHECK((void*)pose == (void*)s_poseMemory, "in the caller's memory");
    CHECK(maniPoseBoneCount(pose) == kBones, "count");
    CHECK(maniPoseModelCurrent(pose), "composed at init");
}

static void TestLocals(void)
{
    maniSkeleton s = BuildRandomSkeleton(kBones, s_file, sizeof s_file);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success, "inits");
    for (uint32_t bone = 0; bone < kBones; ++bone)
    {
        maniTransform rest = maniSkeletonRest(&s, bone);
        maniTransform local = maniPoseLocal(pose, bone);
        CHECK(memcmp(&rest, &local, sizeof rest) == 0, "rest at init");
    }
    maniTransform t = RandomTransform();
    maniPoseSetLocal(pose, 9, t);
    maniTransform back = maniPoseLocal(pose, 9);
    CHECK(memcmp(&t, &back, sizeof t) == 0, "set and read");
    CHECK(!maniPoseModelCurrent(pose), "stale after a set");
    maniPoseSetLocal(pose, kBones, t);
    maniTransform past = maniPoseLocal(pose, kBones);
    CHECK(past.rotation.w == 1.0f && past.scale.x == 1.0f && past.translation.y == 0.0f,
          "identity past the count");

    maniTransform range[5];
    for (uint32_t i = 0; i < 5; ++i)
    {
        range[i] = RandomTransform();
    }
    CHECK(maniPoseWriteLocals(pose, kBones - 5, 5, range) == mani_success, "writes the end");
    maniTransform read[5];
    CHECK(maniPoseReadLocals(pose, kBones - 5, 5, read) == mani_success, "reads the end");
    CHECK(memcmp(range, read, sizeof range) == 0, "the range back");
    CHECK(maniPoseReadLocals(pose, kBones - 4, 5, read) == mani_errorInvalid, "past the end");
    CHECK(maniPoseWriteLocals(pose, kBones + 1, 0, range) == mani_errorInvalid, "start past");
    CHECK(maniPoseReadLocals(pose, 0, 0, NULL) == mani_success, "an empty range");
    CHECK(maniPoseReadLocals(pose, 0, 1, NULL) == mani_errorInvalid, "a null array");
    CHECK(maniPoseSetRest(pose, &s) == mani_success && maniPoseModelCurrent(pose), "rest again");
}

// A count that fills its last group: the bone past it is not read.
static void TestFullGroupEdge(void)
{
    maniSkeleton s = BuildRandomSkeleton(8, s_file, sizeof s_file);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success, "inits");
    for (uint32_t bone = 0; bone < 8; ++bone)
    {
        maniPoseSetLocal(pose, bone, RandomTransform());
    }
    CHECK(maniComputeModel(pose, &s) == mani_success, "composes");
    maniTransform past = maniPoseLocal(pose, 8);
    CHECK(past.rotation.w == 1.0f && past.rotation.x == 0.0f && past.translation.x == 0.0f &&
              past.scale.z == 1.0f,
          "identity just past a full group");
    maniAffine model = maniPoseModel(pose, 8);
    CHECK(model.m[0] == 1.0f && model.m[9] == 0.0f, "identity model just past");
}

static void TestModelAgainstBinary64(void)
{
    for (uint32_t round = 0; round < 20; ++round)
    {
        maniSkeleton s = BuildRandomSkeleton(kBones, s_file, sizeof s_file);
        maniPose* pose = NULL;
        CHECK(maniInitPose(&s, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success, "inits");
        for (uint32_t bone = 0; bone < kBones; ++bone)
        {
            maniPoseSetLocal(pose, bone, RandomTransform());
        }
        CHECK(maniComputeModel(pose, &s) == mani_success, "composes");
        Matrix reference[kBones];
        double worst = 0.0;
        for (uint32_t bone = 0; bone < kBones; ++bone)
        {
            maniTransform local = maniPoseLocal(pose, bone);
            Matrix m = FromTransform(&local);
            uint32_t parent = maniSkeletonParent(&s, bone);
            reference[bone] = parent == MANI_NO_BONE ? m : Multiply(&reference[parent], &m);
            maniAffine model = maniPoseModel(pose, bone);
            // Errors relative to the matrix's size, which grows with depth.
            double magnitude = 1.0;
            for (int k = 0; k < 12; ++k)
            {
                magnitude = fmax(magnitude, fabs(reference[bone].m[k]));
            }
            for (int k = 0; k < 12; ++k)
            {
                worst = fmax(worst, fabs((double)model.m[k] - reference[bone].m[k]) / magnitude);
            }
        }
        CHECK(worst < 2e-5, "within float rounding of binary64");
    }
}

static void TestIdentityRefused(void)
{
    static _Alignas(16) uint8_t other[kFileCapacity];
    maniSkeleton a = BuildRandomSkeleton(kBones, s_file, sizeof s_file);
    maniSkeleton b = BuildRandomSkeleton(kBones, other, sizeof other);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&a, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success, "inits");
    CHECK(maniComputeModel(pose, &b) == mani_errorInvalid, "another skeleton");
    CHECK(maniPoseSetRest(pose, &b) == mani_errorInvalid, "another skeleton's rest");
    CHECK(maniComputeModel(NULL, &a) == mani_errorInvalid, "null pose");
    // Memory that no init made is not a pose: a copy of a real pose
    // without the tag in its first four bytes is refused.
    memcpy(s_otherMemory, s_poseMemory, maniPoseSize(&a));
    memset(s_otherMemory, 0, 4);
    CHECK(maniComputeModel((maniPose*)(void*)s_otherMemory, &a) == mani_errorInvalid, "no tag");
}

// A pose is its bytes: a copy composes the same matrices.
static void TestCopiedBytes(void)
{
    maniSkeleton s = BuildRandomSkeleton(kBones, s_file, sizeof s_file);
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success, "inits");
    for (uint32_t bone = 0; bone < kBones; ++bone)
    {
        maniPoseSetLocal(pose, bone, RandomTransform());
    }
    size_t size = maniPoseSize(&s);
    memcpy(s_otherMemory, s_poseMemory, size);
    maniPose* copy = (maniPose*)(void*)s_otherMemory;
    CHECK(maniComputeModel(pose, &s) == mani_success, "original");
    CHECK(maniComputeModel(copy, &s) == mani_success, "copy");
    maniAffine a[kBones];
    maniAffine b[kBones];
    CHECK(maniPoseReadModels(pose, 0, kBones, a) == mani_success, "reads");
    CHECK(maniPoseReadModels(copy, 0, kBones, b) == mani_success, "reads the copy");
    CHECK(memcmp(a, b, sizeof a) == 0, "same bits");
    CHECK(maniPoseReadModels(pose, 1, kBones, a) == mani_errorInvalid, "past the end");
}

int main(void)
{
    TestMemory();
    TestLocals();
    TestFullGroupEdge();
    TestModelAgainstBinary64();
    TestIdentityRefused();
    TestCopiedBytes();
    return s_failures == 0 ? 0 : 1;
}
