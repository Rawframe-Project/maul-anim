// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body suites' figure (mani-0031): hips under a root, a spine
// and chest above them, two arms of three bones from the chest and a leg
// of three from the hips; two poses, a scratch, solves of either hand or
// both, and the checks they share. Included by one suite each.

#ifndef MAUL_ANIM_TEST_BODY_FIGURE_H
#define MAUL_ANIM_TEST_BODY_FIGURE_H

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BONES 13u

enum
{
    kRoot,
    kHips,
    kSpine,
    kChest,
    kArmL,
    kForeL,
    kHandL,
    kArmR,
    kForeR,
    kHandR,
    kThigh,
    kShin,
    kFoot,
};

static _Alignas(16) uint8_t s_skeletonFile[8192];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_memory[2][8192];
static _Alignas(16) uint8_t s_scratch[65536];
static maniSkeleton s_skeleton;
static maniPose* s_pose;
static maniPose* s_other;

static inline maniQuat Turn(maniVec3 axis, float angle)
{
    float s = sinf(0.5f * angle);
    return (maniQuat){axis.x * s, axis.y * s, axis.z * s, cosf(0.5f * angle)};
}

static inline float Distance(maniVec3 a, maniVec3 b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return sqrtf(x * x + y * y + z * z);
}

static inline float Angle(maniQuat a, maniQuat b)
{
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

static inline maniVec3 At(maniPose* pose, uint32_t bone)
{
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "the model");
    return maniPoseModelTransform(pose, bone).translation;
}

static inline void Setup(void)
{
    static const char* const kNames[BONES] = {"root",  "hips",  "spine", "chest", "armL",
                                              "foreL", "handL", "armR",  "foreR", "handR",
                                              "thigh", "shin",  "foot"};
    static const int32_t kParents[BONES] = {-1, 0, 1, 2, 3, 4, 5, 3, 7, 8, 1, 10, 11};
    static const maniVec3 kOffsets[BONES] = {
        {0.0f, 0.0f, 0.0f},  {0.0f, 1.0f, 0.0f},   {0.0f, 0.2f, 0.0f},  {0.0f, 0.25f, 0.0f},
        {0.15f, 0.2f, 0.0f}, {0.3f, 0.0f, 0.0f},   {0.25f, 0.0f, 0.0f}, {-0.15f, 0.2f, 0.0f},
        {-0.3f, 0.0f, 0.0f}, {-0.25f, 0.0f, 0.0f}, {0.1f, -0.1f, 0.0f}, {0.0f, -0.45f, 0.0f},
        {0.0f, -0.45f, 0.0f}};
    maniBoneSource bones[BONES];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        bones[b] = (maniBoneSource){kNames[b],
                                    (uint32_t)strlen(kNames[b]),
                                    kParents[b],
                                    {kOffsets[b], {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    maniSkeletonSource source = {bones, BONES, 0};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success &&
              maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "the skeleton");
    maniDestroyBuilder(builder);
    CHECK(maniInitPose(&s_skeleton, s_memory[0], sizeof s_memory[0], &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_memory[1], sizeof s_memory[1], &s_other) == mani_success,
          "the poses");
}

// The figure at rest, its arms and leg bent a little.
static inline void Bent(maniPose* pose)
{
    CHECK(maniPoseSetRest(pose, &s_skeleton) == mani_success, "rest");
    const uint32_t bent[4] = {kForeL, kForeR, kShin, kSpine};
    for (int k = 0; k < 4; ++k)
    {
        maniTransform t = maniPoseLocal(pose, bent[k]);
        t.rotation = Turn((maniVec3){0.0f, 0.6f, 0.8f}, 0.3f);
        maniPoseSetLocal(pose, bent[k], t);
    }
}

static maniBodyEffector s_effectors[2];

// Both hands toward goals, from the hips, pinned.
static inline maniBodySolve Solve(maniVec3 left, maniVec3 right)
{
    s_effectors[0] = (maniBodyEffector){.bone = kHandL,
                                        .position = left,
                                        .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                                        .positionWeight = 1.0f,
                                        .strength = 1.0f};
    s_effectors[1] = s_effectors[0];
    s_effectors[1].bone = kHandR;
    s_effectors[1].position = right;
    return (maniBodySolve){.top = kHips,
                           .topMode = maniBodyTopPinned,
                           .effectors = s_effectors,
                           .effectorCount = 2,
                           .iterations = 128,
                           .tolerance = 1e-3f,
                           .weight = 1.0f};
}

static inline bool Solved(maniPose* pose, const maniBodySolve* s, bool* reached)
{
    size_t size = maniBodyIkScratchSize(&s_skeleton, s);
    return size > 0 && size <= sizeof s_scratch &&
           maniSolveBodyXpbd(pose, &s_skeleton, s, s_scratch, size, reached) == mani_success;
}

static inline bool LengthsKept(maniPose* pose, maniPose* input)
{
    bool kept = true;
    for (uint32_t b = 2; b < BONES; ++b)
    {
        uint32_t p = maniSkeletonParent(&s_skeleton, b);
        float now = Distance(At(pose, b), At(pose, p));
        float was = Distance(At(input, b), At(input, p));
        kept = kept && fabsf(now - was) < 1e-5f;
    }
    return kept;
}

static inline bool Same(const maniPose* a, const maniPose* b)
{
    bool same = true;
    for (uint32_t k = 0; k < BONES; ++k)
    {
        maniTransform x = maniPoseLocal(a, k);
        maniTransform y = maniPoseLocal(b, k);
        same = same && memcmp(&x, &y, sizeof x) == 0;
    }
    return same;
}

// The left hand alone toward goal, from the hips.
static inline maniBodySolve Left(maniVec3 goal)
{
    maniBodySolve s = Solve(goal, goal);
    s.effectorCount = 1;
    return s;
}

static inline bool Finite(const maniPose* pose)
{
    bool finite = true;
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        finite = finite && isfinite(t.rotation.x) && isfinite(t.rotation.w) &&
                 isfinite(t.translation.x) && isfinite(t.translation.y);
    }
    return finite;
}

#endif // MAUL_ANIM_TEST_BODY_FIGURE_H
