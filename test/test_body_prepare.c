// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the full-body solvers do before their passes, seen
// through XPBD solves whose effectors have strength 0, so the passes
// move nothing: a pulled chain aimed exactly at its goal, its shape and
// the other arm kept, half a pull turning half the angle; preferred
// angles turning a bone by exactly its compression, and not at all for a
// bone whose own effectors come no closer, for a goal farther than the
// input or for a locked bone; pre-pull leaving a top whose goals are in
// reach, and moving one by exactly the excess past the reach. Then a
// branch bone with no limit axis (its twist range unused), and the part
// held to MANI_MAX_BODY_IK_BONES on a chain of 131 bones.

#include "body_figure.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define LONG 131u

static _Alignas(16) uint8_t s_longFile[32768];
static maniSkeleton s_long;

// The sine of the angle between a and b.
static float Sine(maniVec3 a, maniVec3 b)
{
    maniVec3 c = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    float lengths =
        sqrtf((a.x * a.x + a.y * a.y + a.z * a.z) * (b.x * b.x + b.y * b.y + b.z * b.z));
    return sqrtf(c.x * c.x + c.y * c.y + c.z * c.z) / lengths;
}

static maniVec3 Sub(maniVec3 a, maniVec3 b)
{
    return (maniVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

// A pull chain alone: both hands on the part (the chest a branch, so
// the left chain starts at the shoulder), strength 0, one pass.
static maniBodySolve Pulled(maniVec3 goal, float pull)
{
    maniBodySolve s = Solve(goal, At(s_pose, kHandR));
    s_effectors[0].strength = 0.0f;
    s_effectors[1].strength = 0.0f;
    s_effectors[0].pullChain = pull;
    s.iterations = 1;
    s.tolerance = 0.0f;
    return s;
}

static void TestPullChain(void)
{
    maniVec3 goal = {0.3f, 1.9f, 0.4f};
    Bent(s_pose);
    Bent(s_other);
    maniVec3 shoulder = At(s_pose, kArmL);
    maniBodySolve s = Pulled(goal, 1.0f);
    CHECK(Solved(s_pose, &s, NULL), "a pull chain alone");
    CHECK(Sine(Sub(At(s_pose, kHandL), shoulder), Sub(goal, shoulder)) < 1e-5f,
          "the arm aimed at the goal from its first bone");
    CHECK(Angle(maniPoseLocal(s_pose, kForeL).rotation, maniPoseLocal(s_other, kForeL).rotation) <
                  1e-3f &&
              Angle(maniPoseLocal(s_pose, kHandL).rotation,
                    maniPoseLocal(s_other, kHandL).rotation) < 1e-3f,
          "the arm turned whole, its shape kept");
    CHECK(Distance(At(s_pose, kHandR), At(s_other, kHandR)) < 1e-6f, "the other arm left");
    float before = asinf(Sine(Sub(At(s_other, kHandL), shoulder), Sub(goal, shoulder)));
    Bent(s_pose);
    s = Pulled(goal, 0.5f);
    CHECK(Solved(s_pose, &s, NULL), "half a pull");
    float after = asinf(Sine(Sub(At(s_pose, kHandL), shoulder), Sub(goal, shoulder)));
    CHECK(fabsf(after - 0.5f * before) < 1e-4f, "half the angle");
}

// The foot toward goal with preferred angles of 0.5 about x on the shin,
// strength 0, one pass.
static bool Folded(maniVec3 goal, float stiffness)
{
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success &&
              maniPoseSetRest(s_other, &s_skeleton) == mani_success,
          "rest");
    maniBodyEffector foot = {.bone = kFoot,
                             .position = goal,
                             .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                             .positionWeight = 1.0f};
    maniBodyBone shin = {
        .bone = kShin, .preferred = {0.5f, 0.0f, 0.0f}, .rotationStiffness = stiffness};
    maniBodySolve s = {.top = kHips,
                       .effectors = &foot,
                       .bones = &shin,
                       .effectorCount = 1,
                       .boneCount = 1,
                       .iterations = 1,
                       .weight = 1.0f};
    return Solved(s_pose, &s, NULL);
}

static void TestPreferred(void)
{
    // The shin's origin at (0.1, 0.45, 0), the foot's 0.45 below it: a
    // goal 0.15 below compresses by 2/3.
    CHECK(Folded((maniVec3){0.1f, 0.3f, 0.0f}, 0.0f), "a compressed leg");
    maniQuat shin = maniPoseLocal(s_pose, kShin).rotation;
    CHECK(fabsf(shin.x - sinf(0.5f / 3.0f)) < 1e-6f && fabsf(shin.y) < 1e-7f &&
              fabsf(shin.z) < 1e-7f,
          "the shin turned by its angles times the compression");
    CHECK(Folded((maniVec3){0.1f, -0.3f, 0.0f}, 0.0f) && Same(s_pose, s_other),
          "a goal farther than the input: no turn");
    CHECK(Folded((maniVec3){0.1f, 0.3f, 0.0f}, 1.0f) && Same(s_pose, s_other),
          "a locked bone: no turn");
    // Preferred angles on the left forearm, its hand's goal where it is;
    // the right hand's goal close to its shoulder.
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve(At(s_pose, kHandL), At(s_pose, kArmR));
    s_effectors[0].strength = 0.0f;
    s_effectors[1].strength = 0.0f;
    s.iterations = 1;
    s.tolerance = 0.0f;
    maniBodyBone fore = {.bone = kForeL, .preferred = {0.5f, 0.0f, 0.0f}};
    s.bones = &fore;
    s.boneCount = 1;
    CHECK(Solved(s_pose, &s, NULL) && Same(s_pose, s_other),
          "another limb's compression turns nothing");
}

static void TestPrePull(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Left((maniVec3){0.6f, 1.2f, 0.3f});
    s.topMode = maniBodyTopPrePull;
    CHECK(Solved(s_pose, &s, NULL), "pre-pull in reach");
    maniTransform hips[2] = {maniPoseLocal(s_pose, kHips), maniPoseLocal(s_other, kHips)};
    CHECK(memcmp(&hips[0], &hips[1], sizeof hips[0]) == 0, "a goal in reach moves no top");
    // The left hand's reach from the hips: 0.2 + 0.25 + 0.25 + 0.3 + 0.25.
    maniVec3 far = {1.8f, 1.5f, 0.2f};
    maniVec3 top = At(s_other, kHips);
    maniVec3 line = Sub(far, top);
    float d = sqrtf(line.x * line.x + line.y * line.y + line.z * line.z);
    float excess = (d - 1.25f) / d;
    maniVec3 want = {top.x + line.x * excess, top.y + line.y * excess, top.z + line.z * excess};
    Bent(s_pose);
    s = Left(far);
    s.topMode = maniBodyTopPrePull;
    CHECK(Solved(s_pose, &s, NULL), "pre-pull out of reach");
    CHECK(Distance(At(s_pose, kHips), want) < 1e-5f, "the top moved by the excess past the reach");
}

// A cone on the chest (a branch, no axis) whose twist range leaves out
// 0, goals just off the hands: the twist range is unused.
static void TestBranchAxis(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniVec3 left = At(s_pose, kHandL);
    maniVec3 right = At(s_pose, kHandR);
    // Goals just off the hands, so the passes (and the limits) run.
    maniBodySolve s = Solve((maniVec3){left.x + 1e-3f, left.y, left.z},
                            (maniVec3){right.x - 1e-3f, right.y, right.z});
    s.iterations = 4;
    s.tolerance = 0.0f;
    maniBodyBone chest = {
        .bone = kChest,
        .limit = {.kind = maniJointLimitCone, .cone = 3.0f, .twistLow = 0.1f, .twistHigh = 0.2f}};
    s.bones = &chest;
    s.boneCount = 1;
    CHECK(Solved(s_pose, &s, NULL), "a branch's cone");
    CHECK(Angle(maniPoseLocal(s_pose, kChest).rotation, maniPoseLocal(s_other, kChest).rotation) <
              1e-3f,
          "a branch has no axis: its twist range unused");
}

// A chain of LONG bones, each 0.1 above its parent.
static void BuildLong(void)
{
    static char names[LONG][4];
    static maniBoneSource bones[LONG];
    for (uint32_t b = 0; b < LONG; ++b)
    {
        (void)snprintf(names[b], sizeof names[b], "%03u", b);
        bones[b] = (maniBoneSource){
            names[b],
            3,
            (int32_t)b - 1,
            {{0.0f, b == 0 ? 0.0f : 0.1f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    maniSkeletonSource source = {bones, LONG};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success &&
              maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_longFile, sizeof s_longFile, &size) == mani_success &&
              maniOpenSkeleton(s_longFile, size, &s_long).result == mani_success,
          "the long chain");
    maniDestroyBuilder(builder);
}

static void TestBound(void)
{
    BuildLong();
    maniBodyEffector tip = {.bone = LONG - 1,
                            .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                            .positionWeight = 1.0f,
                            .strength = 1.0f};
    maniBodySolve s = {.effectors = &tip, .effectorCount = 1, .iterations = 1, .weight = 1.0f};
    s.top = LONG - MANI_MAX_BODY_IK_BONES;
    CHECK(maniBodyIkScratchSize(&s_long, &s) > 0, "a part of 128 bones");
    s.top = LONG - MANI_MAX_BODY_IK_BONES - 1;
    CHECK(maniBodyIkScratchSize(&s_long, &s) == 0, "a part of 129 bones refused");
}

int main(void)
{
    Setup();
    TestPullChain();
    TestPreferred();
    TestPrePull();
    TestBranchAxis();
    TestBound();
    return s_failures == 0 ? 0 : 1;
}
