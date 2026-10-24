// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body XPBD solve on the figure (body_figure.h):
// both hands at reachable goals reached with every length kept; a solve
// on its own reached result unchanged bit for bit; weight 0 and
// strength 0 leaving the pose; a locked forearm keeping its local
// rotation, its hand reached where the pose puts it; a cone on a
// forearm held, wider limits changing no bit; a hand taking a wanted
// rotation; a goal out of reach: stretch, a free top and pre-pull each
// bringing the hand closer; stretching, a reached hand where the pose
// puts it, position stiffness 1 keeping lengths against a compliant
// effector, strength 0 leaving the pose; a free top at weight 0.5
// moving half as far; one unreached effector or weight 0.5 not reached;
// a position weight of 0.5 stopping halfway; a weaker effector stopping
// further against stiff bones; a stiff spine turning less; a solve short
// of convergence pinned with and without stretch, the same on reused
// scratch; an effector on the held top; goals at the hips and nearly
// every bone locked without a NaN; the scratch size and its refusals;
// the description's refusals.

#include "body_figure.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PINNED_XPBD_HASH 0x485c56db6c30e444ull

static void TestReach(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    bool r = false;
    CHECK(Solved(s_pose, &s, &r) && r, "both hands reached");
    CHECK(Distance(At(s_pose, kHandL), s_effectors[0].position) <= 1e-3f &&
              Distance(At(s_pose, kHandR), s_effectors[1].position) <= 1e-3f,
          "within the tolerance");
    CHECK(LengthsKept(s_pose, s_other), "every length kept");
    maniTransform hips[2] = {maniPoseLocal(s_pose, kHips), maniPoseLocal(s_other, kHips)};
    CHECK(memcmp(&hips[0], &hips[1], sizeof hips[0]) == 0, "the pinned top held");
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniPoseSetLocal(s_other, b, maniPoseLocal(s_pose, b));
    }
    CHECK(Solved(s_other, &s, &r) && r && Same(s_pose, s_other),
          "a solve on its own reached result unchanged");
}

static void TestWeights(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s.weight = 0.0f;
    CHECK(Solved(s_pose, &s, NULL) && Same(s_pose, s_other), "weight 0 leaves the pose");
    s.weight = 1.0f;
    s_effectors[0].strength = 0.0f;
    s_effectors[1].strength = 0.0f;
    s_effectors[1].rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 1.0f);
    s_effectors[1].rotationWeight = 1.0f;
    bool r = true;
    CHECK(Solved(s_pose, &s, &r) && !r && Same(s_pose, s_other),
          "strength 0 pulls nothing, nor turns, unreached");
}

static void TestLockAndLimit(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    maniBodyBone bones[2] = {
        {.bone = kForeL, .rotationStiffness = 1.0f},
        {.bone = kForeR,
         .limit = {
             .kind = maniJointLimitCone, .cone = 0.4f, .twistLow = -0.2f, .twistHigh = 0.2f}}};
    s.bones = bones;
    s.boneCount = 2;
    CHECK(Solved(s_pose, &s, NULL), "a lock and a limit");
    maniQuat locked[2] = {maniPoseLocal(s_pose, kForeL).rotation,
                          maniPoseLocal(s_other, kForeL).rotation};
    CHECK(memcmp(&locked[0], &locked[1], sizeof locked[0]) == 0, "the locked forearm kept");
    CHECK(Distance(At(s_pose, kHandL), s_effectors[0].position) <= 1e-3f,
          "the hand under a locked forearm reached where the pose puts it");
    maniQuat t = maniPoseLocal(s_pose, kForeR).rotation;
    t = t.w < 0.0f ? (maniQuat){-t.x, -t.y, -t.z, -t.w} : t;
    // The forearm's axis is the hand's direction at rest, -x.
    float twist = -2.0f * atan2f(t.x, t.w);
    maniQuat q = Turn((maniVec3){-1.0f, 0.0f, 0.0f}, -twist);
    float sw = t.w * q.w - t.x * q.x - t.y * q.y - t.z * q.z;
    CHECK(2.0f * acosf(fminf(1.0f, fabsf(sw))) <= 0.4f + 1e-4f && fabsf(twist) <= 0.2f + 1e-4f,
          "the limited forearm within its cone and twist");
    CHECK(LengthsKept(s_pose, s_other), "lengths kept with a lock and a limit");
}

static void TestRotation(void)
{
    Bent(s_pose);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s_effectors[0].rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 1.0f);
    s_effectors[0].rotationWeight = 1.0f;
    CHECK(Solved(s_pose, &s, NULL) && maniComputeModel(s_pose, &s_skeleton) == mani_success,
          "a rotation goal");
    CHECK(Angle(maniPoseModelTransform(s_pose, kHandL).rotation, s_effectors[0].rotation) < 1e-2f,
          "the hand takes the wanted rotation");
}

static void TestStretchAndTop(void)
{
    maniVec3 far = {1.8f, 1.5f, 0.2f};
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Left(far);
    bool r = true;
    CHECK(Solved(s_pose, &s, &r) && !r, "out of reach");
    float pinned = Distance(At(s_pose, kHandL), far);
    CHECK(LengthsKept(s_pose, s_other), "lengths kept out of reach");
    Bent(s_pose);
    s.stretch = true;
    CHECK(Solved(s_pose, &s, NULL), "stretching");
    float stretched = Distance(At(s_pose, kHandL), far);
    CHECK(stretched < pinned - 1e-2f && !LengthsKept(s_pose, s_other),
          "stretch brings the hand closer, lengths given up");
    Bent(s_pose);
    s = Left(far);
    s.topMode = maniBodyTopFree;
    CHECK(Solved(s_pose, &s, NULL), "a free top");
    maniVec3 hips = At(s_pose, kHips);
    CHECK(Distance(At(s_pose, kHandL), far) < pinned - 1e-2f &&
              Distance(hips, At(s_other, kHips)) > 1e-2f,
          "a free top moves, the hand closer");
    Bent(s_pose);
    s.topMode = maniBodyTopPrePull;
    CHECK(Solved(s_pose, &s, NULL), "a pre-pulled top");
    maniVec3 moved = At(s_pose, kHips);
    maniVec3 was = At(s_other, kHips);
    maniQuat turned[2] = {maniPoseLocal(s_pose, kHips).rotation,
                          maniPoseLocal(s_other, kHips).rotation};
    CHECK(moved.x > was.x + 1e-2f && memcmp(&turned[0], &turned[1], sizeof turned[0]) == 0 &&
              Distance(At(s_pose, kHandL), far) < pinned - 1e-2f,
          "pre-pull moves the top toward the goal, unturned");
}

static void TestPullChain(void)
{
    maniVec3 goal = {0.3f, 1.9f, 0.4f};
    Bent(s_pose);
    maniBodySolve s = Left(goal);
    s.iterations = 1;
    s.tolerance = 0.0f;
    CHECK(Solved(s_pose, &s, NULL), "one pass");
    float plain = Distance(At(s_pose, kHandL), goal);
    Bent(s_pose);
    s_effectors[0].pullChain = 1.0f;
    CHECK(Solved(s_pose, &s, NULL), "one pass with the chain pulled");
    CHECK(Distance(At(s_pose, kHandL), goal) < plain, "the pulled chain starts closer");
}

// The foot from the hips toward a goal 0.6 below the thigh on a
// straight leg: preferred angles about x choose the bend.
static void TestPreferred(void)
{
    const float sides[2] = {0.5f, -0.5f};
    for (int k = 0; k < 2; ++k)
    {
        CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
        maniBodyEffector foot = {.bone = kFoot,
                                 .position = {0.1f, 0.3f, 0.0f},
                                 .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                                 .positionWeight = 1.0f,
                                 .strength = 1.0f};
        maniBodyBone shin = {.bone = kShin, .preferred = {sides[k], 0.0f, 0.0f}};
        maniBodySolve s = {.top = kHips,
                           .effectors = &foot,
                           .bones = &shin,
                           .effectorCount = 1,
                           .boneCount = 1,
                           .iterations = 128,
                           .tolerance = 1e-3f,
                           .weight = 1.0f};
        bool r = false;
        CHECK(Solved(s_pose, &s, &r) && r, "a compressed leg reached");
        CHECK(maniPoseLocal(s_pose, kShin).rotation.x * sides[k] > 0.05f,
              "the knee bends the preferred way");
    }
}

static void TestStiffness(void)
{
    maniVec3 goal = {0.9f, 1.1f, 0.5f};
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Left(goal);
    CHECK(Solved(s_pose, &s, NULL), "free");
    float free =
        Angle(maniPoseLocal(s_pose, kSpine).rotation, maniPoseLocal(s_other, kSpine).rotation);
    Bent(s_pose);
    maniBodyBone stiff[2] = {{.bone = kSpine, .rotationStiffness = 0.9f},
                             {.bone = kChest, .rotationStiffness = 0.9f}};
    s.bones = stiff;
    s.boneCount = 2;
    CHECK(Solved(s_pose, &s, NULL), "stiff");
    float held =
        Angle(maniPoseLocal(s_pose, kSpine).rotation, maniPoseLocal(s_other, kSpine).rotation);
    CHECK(free > 1e-2f && held < 0.5f * free, "a stiff spine turns less");
}

// An unreached left hand before a reached right: not reached; weight
// 0.5: not reached; a position weight of 0.5: the hand halfway.
static void TestReachedAndWeights(void)
{
    Bent(s_pose);
    // The trunk locked, so the right hand reaches whatever the left does.
    maniBodySolve s = Solve((maniVec3){1.8f, 1.5f, 0.2f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    maniBodyBone trunk[2] = {{.bone = kSpine, .rotationStiffness = 1.0f},
                             {.bone = kChest, .rotationStiffness = 1.0f}};
    s.bones = trunk;
    s.boneCount = 2;
    bool r = true;
    CHECK(Solved(s_pose, &s, &r) && !r &&
              Distance(At(s_pose, kHandR), s_effectors[1].position) <= 1e-3f,
          "one hand out of reach, the other reached: not reached");
    Bent(s_pose);
    s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s.weight = 0.5f;
    r = true;
    CHECK(Solved(s_pose, &s, &r) && !r, "weight 0.5: not reached");
    Bent(s_pose);
    Bent(s_other);
    maniVec3 from = At(s_other, kHandL);
    maniVec3 to = {0.6f, 1.2f, 0.3f};
    s = Left(to);
    s_effectors[0].positionWeight = 0.5f;
    CHECK(Solved(s_pose, &s, NULL), "a position weight of 0.5");
    maniVec3 half = {0.5f * (from.x + to.x), 0.5f * (from.y + to.y), 0.5f * (from.z + to.z)};
    CHECK(Distance(At(s_pose, kHandL), half) <= 1e-3f, "the hand halfway");
}

// Cones and twists wider than any turn change no bit.
static void TestWideLimits(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s.iterations = 16;
    CHECK(Solved(s_other, &s, NULL), "unlimited");
    maniBodyBone wide[2];
    const uint32_t fore[2] = {kForeL, kForeR};
    for (int k = 0; k < 2; ++k)
    {
        wide[k] = (maniBodyBone){
            .bone = fore[k],
            .limit = {
                .kind = maniJointLimitCone, .cone = 3.1f, .twistLow = -3.1f, .twistHigh = 3.1f}};
    }
    s.bones = wide;
    s.boneCount = 2;
    CHECK(Solved(s_pose, &s, NULL) && Same(s_pose, s_other), "wide limits change no bit");
}

static void TestStretchDetails(void)
{
    maniVec3 goal = {0.6f, 1.2f, 0.3f};
    Bent(s_pose);
    maniBodySolve s = Left(goal);
    s.stretch = true;
    bool r = false;
    CHECK(Solved(s_pose, &s, &r) && r && Distance(At(s_pose, kHandL), goal) <= 1e-3f,
          "stretching, a reached hand where the pose puts it");
    maniVec3 far = {1.8f, 1.5f, 0.2f};
    Bent(s_pose);
    Bent(s_other);
    s = Left(far);
    s.stretch = true;
    maniBodyBone held[5];
    const uint32_t part[5] = {kSpine, kChest, kArmL, kForeL, kHandL};
    for (int k = 0; k < 5; ++k)
    {
        held[k] = (maniBodyBone){.bone = part[k], .positionStiffness = 1.0f};
    }
    s.bones = held;
    s.boneCount = 5;
    s.iterations = 256;
    // A compliant effector: hard joints and a hard effector out of reach
    // pull against each other without end.
    s_effectors[0].strength = 0.5f;
    CHECK(Solved(s_pose, &s, NULL), "stretch held by position stiffness");
    float worst = 0.0f;
    for (int k = 0; k < 5; ++k)
    {
        uint32_t p = maniSkeletonParent(&s_skeleton, part[k]);
        float now = Distance(At(s_pose, part[k]), At(s_pose, p));
        float was = Distance(At(s_other, part[k]), At(s_other, p));
        worst = fmaxf(worst, fabsf(now - was));
    }
    CHECK(worst < 1e-2f, "position stiffness 1 keeps lengths");
    Bent(s_pose);
    s = Left(far);
    s.stretch = true;
    s_effectors[0].strength = 0.0f;
    CHECK(Solved(s_pose, &s, NULL) && Finite(s_pose) && Same(s_pose, s_other),
          "strength 0 with stretch leaves the pose");
}

// A free top at weight 0.5 moves half as far as at 1.
static void TestHalfMove(void)
{
    maniVec3 far = {1.8f, 1.5f, 0.2f};
    Bent(s_pose);
    maniBodySolve s = Left(far);
    s.topMode = maniBodyTopFree;
    CHECK(Solved(s_pose, &s, NULL), "a free top");
    maniVec3 full = maniPoseLocal(s_pose, kHips).translation;
    Bent(s_pose);
    Bent(s_other);
    s.weight = 0.5f;
    CHECK(Solved(s_pose, &s, NULL), "half of it");
    maniVec3 was = maniPoseLocal(s_other, kHips).translation;
    maniVec3 half = maniPoseLocal(s_pose, kHips).translation;
    maniVec3 want = {was.x + 0.5f * (full.x - was.x), was.y + 0.5f * (full.y - was.y),
                     was.z + 0.5f * (full.z - was.z)};
    CHECK(Distance(half, want) < 1e-6f, "the top moved half as far");
}

// Strength against stiff arm bones: the weaker effector stops short.
static void TestStrength(void)
{
    maniVec3 goal = {0.9f, 1.1f, 0.5f};
    maniBodyBone stiff[2] = {{.bone = kArmL, .rotationStiffness = 0.5f},
                             {.bone = kForeL, .rotationStiffness = 0.5f}};
    float miss[2];
    const float strengths[2] = {1.0f, 0.3f};
    for (int k = 0; k < 2; ++k)
    {
        Bent(s_pose);
        maniBodySolve s = Left(goal);
        s_effectors[0].strength = strengths[k];
        s.bones = stiff;
        s.boneCount = 2;
        s.iterations = 256;
        s.tolerance = 0.0f;
        CHECK(Solved(s_pose, &s, NULL), "against stiffness");
        miss[k] = Distance(At(s_pose, kHandL), goal);
    }
    CHECK(miss[1] > miss[0] + 1e-3f, "a weaker effector stops further from its goal");
}

// Every compliance, a stiffness, a limit and a rotation weight, short
// of convergence, without stretch and twice with it on the same
// scratch: the second the same as the first, both pinned.
static void TestPinned(void)
{
    uint64_t hashes[3];
    for (int run = 0; run < 3; ++run)
    {
        Bent(s_pose);
        maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
        s_effectors[0].strength = 0.5f;
        s_effectors[1].strength = 0.8f;
        s_effectors[1].rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, -0.8f);
        s_effectors[1].rotationWeight = 0.5f;
        maniBodyBone bones[3] = {
            {.bone = kSpine, .rotationStiffness = 0.5f},
            {.bone = kForeL, .rotationStiffness = 0.5f},
            {.bone = kForeR,
             .limit = {
                 .kind = maniJointLimitCone, .cone = 0.4f, .twistLow = -0.2f, .twistHigh = 0.2f}}};
        s.bones = bones;
        s.boneCount = 3;
        s.iterations = 16;
        s.tolerance = 0.0f;
        s.stretch = run > 0;
        CHECK(Solved(s_pose, &s, NULL), "the pinned solve");
        hashes[run] = MANI_HASH_INIT;
        for (uint32_t b = 0; b < BONES; ++b)
        {
            maniTransform t = maniPoseLocal(s_pose, b);
            hashes[run] = maniHash64(hashes[run], &t, (int32_t)sizeof t);
        }
    }
    CHECK(hashes[2] == hashes[1], "a second solve on the same scratch the same");
    // The plain solve and the stretching one, together.
    uint64_t hash = maniHash64(hashes[0], &hashes[1], (int32_t)sizeof hashes[1]);
    CHECK(hash == PINNED_XPBD_HASH, "the same bits on every platform");
    if (hash != PINNED_XPBD_HASH)
    {
        printf("pinned 0x%016llx\n", (unsigned long long)hash);
    }
    Bent(s_pose);
    maniBodySolve s = Solve(At(s_pose, kHandL), (maniVec3){-0.5f, 1.6f, 0.2f});
    maniBodyEffector both[2] = {s_effectors[0], s_effectors[1]};
    both[0].bone = kHips;
    both[0].position = (maniVec3){0.5f, 0.5f, 0.5f};
    s.effectors = both;
    Bent(s_other);
    CHECK(Solved(s_pose, &s, NULL) && Finite(s_pose) &&
              Distance(At(s_pose, kHandR), both[1].position) <= 1e-3f,
          "an effector on the held top, the other still reached");
    maniTransform hips[2] = {maniPoseLocal(s_pose, kHips), maniPoseLocal(s_other, kHips)};
    CHECK(memcmp(&hips[0], &hips[1], sizeof hips[0]) == 0, "the held top unmoved");
}

static void TestEdges(void)
{
    Bent(s_pose);
    maniVec3 hips = At(s_pose, kHips);
    maniBodySolve s = Solve(hips, hips);
    CHECK(Solved(s_pose, &s, NULL) && Finite(s_pose), "goals at the top: no NaN");
    Bent(s_pose);
    Bent(s_other);
    s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    maniBodyBone locked[7];
    const uint32_t part[7] = {kSpine, kChest, kArmL, kForeL, kHandL, kArmR, kForeR};
    for (int k = 0; k < 7; ++k)
    {
        locked[k] = (maniBodyBone){.bone = part[k], .rotationStiffness = 1.0f};
    }
    s.bones = locked;
    s.boneCount = 7;
    CHECK(Solved(s_pose, &s, NULL) && Finite(s_pose), "nearly all locked: no NaN");
    maniTransform spine[2] = {maniPoseLocal(s_pose, kSpine), maniPoseLocal(s_other, kSpine)};
    CHECK(memcmp(&spine[0], &spine[1], sizeof spine[0]) == 0, "a locked spine kept");
}

static void TestRefusals(void)
{
    Bent(s_pose);
    maniBodySolve good = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    size_t size = maniBodyIkScratchSize(&s_skeleton, &good);
    CHECK(size > 0 && maniBodyIkScratchSize(NULL, &good) == 0 &&
              maniBodyIkScratchSize(&s_skeleton, NULL) == 0,
          "the scratch size, 0 for NULL");
    CHECK(maniSolveBodyXpbd(NULL, &s_skeleton, &good, s_scratch, size, NULL) == mani_errorInvalid &&
              maniSolveBodyXpbd(s_pose, NULL, &good, s_scratch, size, NULL) == mani_errorInvalid &&
              maniSolveBodyXpbd(s_pose, &s_skeleton, NULL, s_scratch, size, NULL) ==
                  mani_errorInvalid &&
              maniSolveBodyXpbd(s_pose, &s_skeleton, &good, NULL, size, NULL) ==
                  mani_errorInvalid &&
              maniSolveBodyXpbd(s_pose, &s_skeleton, &good, s_scratch + 4, size, NULL) ==
                  mani_errorInvalid &&
              maniSolveBodyXpbd(s_pose, &s_skeleton, &good, s_scratch, size - 1, NULL) ==
                  mani_errorInvalid,
          "NULLs, misaligned or too little scratch");
    maniBodyEffector saved[2] = {s_effectors[0], s_effectors[1]};
    maniBodySolve bad[10];
    for (int k = 0; k < 10; ++k)
    {
        bad[k] = good;
    }
    bad[0].top = BONES;
    bad[1].top = kForeL;
    bad[2].effectorCount = 0;
    bad[3].effectorCount = MANI_BODY_IK_MAX_EFFECTORS + 1;
    bad[4].iterations = 0;
    bad[5].iterations = MANI_IK_MAX_ITERATIONS + 1;
    bad[6].tolerance = -1.0f;
    bad[7].weight = 1.5f;
    bad[8].topMode = 3;
    bad[9].boneCount = 1;
    int refused = 0;
    for (int k = 0; k < 10; ++k)
    {
        refused += maniBodyIkScratchSize(&s_skeleton, &bad[k]) == 0 &&
                   maniSolveBodyXpbd(s_pose, &s_skeleton, &bad[k], s_scratch, sizeof s_scratch,
                                     NULL) == mani_errorInvalid;
    }
    CHECK(refused == 10, "tops, counts and numbers out of range refused");
    maniBodyEffector wrong[6];
    for (int k = 0; k < 6; ++k)
    {
        wrong[k] = saved[0];
    }
    wrong[0].bone = BONES;
    wrong[1].position.x = NAN;
    wrong[2].rotation = (maniQuat){0.0f, 0.0f, 0.0f, 0.0f};
    wrong[3].strength = -0.1f;
    wrong[4].pullChain = 2.0f;
    wrong[5].rotationWeight = NAN;
    refused = 0;
    for (int k = 0; k < 6; ++k)
    {
        maniBodyEffector pair[2] = {wrong[k], saved[1]};
        maniBodySolve s = good;
        s.effectors = pair;
        refused += maniSolveBodyXpbd(s_pose, &s_skeleton, &s, s_scratch, sizeof s_scratch, NULL) ==
                   mani_errorInvalid;
    }
    maniBodyEffector twice[2] = {saved[0], saved[0]};
    maniBodySolve s = good;
    s.effectors = twice;
    refused += maniSolveBodyXpbd(s_pose, &s_skeleton, &s, s_scratch, sizeof s_scratch, NULL) ==
               mani_errorInvalid;
    CHECK(refused == 7, "effectors out of range, not finite, of no rotation, twice refused");
    const maniBodyBone settings[5] = {
        {.bone = kThigh},
        {.bone = kForeL, .rotationStiffness = 1.5f},
        {.bone = kForeL, .preferred = {NAN, 0.0f, 0.0f}},
        {.bone = kForeL, .limit = {.kind = 3}},
        {.bone = kForeL,
         .limit = {.kind = maniJointLimitHinge, .axis = {1.0f, 0.0f, 0.0f}, .high = 0.5f}}};
    refused = 0;
    for (int k = 0; k < 5; ++k)
    {
        s = good;
        s.bones = &settings[k];
        s.boneCount = 1;
        refused += maniSolveBodyXpbd(s_pose, &s_skeleton, &s, s_scratch, sizeof s_scratch, NULL) ==
                   mani_errorInvalid;
    }
    CHECK(refused == 5, "settings off the part, out of range, a hinge along its bone refused");
    maniBodyBone pair[2] = {{.bone = kForeL}, {.bone = kForeL}};
    s = good;
    s.bones = pair;
    s.boneCount = 2;
    CHECK(maniSolveBodyXpbd(s_pose, &s_skeleton, &s, s_scratch, sizeof s_scratch, NULL) ==
              mani_errorInvalid,
          "a bone's settings twice refused");
}

int main(void)
{
    Setup();
    TestReach();
    TestWeights();
    TestLockAndLimit();
    TestRotation();
    TestStretchAndTop();
    TestPullChain();
    TestPreferred();
    TestStiffness();
    TestReachedAndWeights();
    TestWideLimits();
    TestStretchDetails();
    TestHalfMove();
    TestStrength();
    TestPinned();
    TestEdges();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
