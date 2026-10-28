// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// CCD chains and joint limits on a tail of five bones of
// 0.3 under a root: goals within reach reached within the tolerance
// with every length kept; one out of reach straightens the chain toward
// it; cone, twist and hinge limits never exceeded, read back from each
// bone's turn from its rest, on a chain along x resting turned too;
// limits of no kind solving as none bit for bit; the step bound bounding
// each bone's turn in a sweep; a solve on its own reached result
// unchanged bit for bit; the end taking the wanted rotation; weights 0
// and 0.5; a goal at the root left without a NaN, the root unturned; a
// goal within the tolerance leaving the pose, one at the end reached
// with tolerance 0; the refusals, a path of 33 bones among them.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BONES 6u
#define LONG  34u

static _Alignas(16) uint8_t s_skeletonFile[3][8192];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_memory[5][8192];
static maniSkeleton s_skeleton;
static maniSkeleton s_long;
static maniSkeleton s_turned;
static maniPose* s_pose;
static maniPose* s_other;
static maniPose* s_longPose;
static maniPose* s_turnedPose;
static maniPose* s_turnedOther;

static maniQuat Turn(maniVec3 axis, float angle)
{
    float s = sinf(0.5f * angle);
    return (maniQuat){axis.x * s, axis.y * s, axis.z * s, cosf(0.5f * angle)};
}

static maniQuat Mul(maniQuat a, maniQuat b)
{
    return (maniQuat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static float Distance(maniVec3 a, maniVec3 b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return sqrtf(x * x + y * y + z * z);
}

static maniVec3 At(maniPose* pose, uint32_t bone)
{
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "the model");
    return maniPoseModelTransform(pose, bone).translation;
}

// A chain of count bones under a root, each 0.3 along y from its parent;
// turned, each 0.3 along x resting turned 0.4 about z.
static void Build(maniSkeleton* skeleton, uint8_t* file, size_t capacity, uint32_t count,
                  bool turned)
{
    static char names[LONG][4];
    maniBoneSource bones[LONG];
    if (count > LONG)
    {
        CHECK(false, "a chain fits the names");
        return;
    }
    // Every name, whatever the count: a fixed bound GCC's vectorizer can
    // see, which a bound of count is not (its -mavx2 build warned).
    for (uint32_t b = 0; b < LONG; ++b)
    {
        names[b][0] = 'b';
        names[b][1] = (char)('a' + b / 26);
        names[b][2] = (char)('a' + b % 26);
    }
    for (uint32_t b = 0; b < count; ++b)
    {
        float d = b == 0 ? 0.0f : 0.3f;
        maniTransform rest = {{0.0f, d, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
        if (turned && b > 0)
        {
            rest.translation = (maniVec3){d, 0.0f, 0.0f};
            rest.rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 0.4f);
        }
        bones[b] = (maniBoneSource){names[b], 3, (int32_t)b - 1, rest};
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    maniSkeletonSource source = {bones, count};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success &&
              maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, capacity, &size) == mani_success &&
              maniOpenSkeleton(file, size, skeleton).result == mani_success,
          "a skeleton");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    Build(&s_skeleton, s_skeletonFile[0], sizeof s_skeletonFile[0], BONES, false);
    Build(&s_long, s_skeletonFile[1], sizeof s_skeletonFile[1], LONG, false);
    Build(&s_turned, s_skeletonFile[2], sizeof s_skeletonFile[2], BONES, true);
    CHECK(maniInitPose(&s_skeleton, s_memory[0], sizeof s_memory[0], &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_memory[1], sizeof s_memory[1], &s_other) ==
                  mani_success &&
              maniInitPose(&s_long, s_memory[2], sizeof s_memory[2], &s_longPose) == mani_success &&
              maniInitPose(&s_turned, s_memory[3], sizeof s_memory[3], &s_turnedPose) ==
                  mani_success &&
              maniInitPose(&s_turned, s_memory[4], sizeof s_memory[4], &s_turnedOther) ==
                  mani_success,
          "the poses");
}

// The tail at rest, each bone bent a little about x and z.
static void Bent(maniPose* pose)
{
    CHECK(maniPoseSetRest(pose, &s_skeleton) == mani_success, "rest");
    for (uint32_t b = 1; b < BONES; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        t.rotation = Turn((maniVec3){0.6f, 0.0f, 0.8f}, 0.15f);
        maniPoseSetLocal(pose, b, t);
    }
}

static maniChainSolve Solve(maniVec3 goal)
{
    return (maniChainSolve){.root = 1,
                            .end = BONES - 1,
                            .goal = goal,
                            .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                            .iterations = 64,
                            .tolerance = 1e-4f,
                            .weight = 1.0f};
}

// Whether every bone keeps its 0.3 from its parent.
static bool LengthsKept(maniPose* pose)
{
    bool kept = true;
    for (uint32_t b = 2; b < BONES; ++b)
    {
        kept = kept && fabsf(Distance(At(pose, b), At(pose, b - 1)) - 0.3f) < 1e-5f;
    }
    return kept;
}

static void TestReach(void)
{
    const maniVec3 goals[4] = {
        {0.4f, 0.6f, 0.3f}, {-0.6f, 0.2f, 0.5f}, {0.1f, 1.1f, 0.0f}, {0.3f, -0.4f, 0.2f}};
    for (int g = 0; g < 4; ++g)
    {
        Bent(s_pose);
        maniChainSolve s = Solve(goals[g]);
        bool r = false;
        CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
                  Distance(At(s_pose, BONES - 1), goals[g]) <= 1e-4f,
              "a goal within reach reached");
        CHECK(LengthsKept(s_pose), "every length kept");
    }
}

static void TestOutOfReach(void)
{
    Bent(s_pose);
    maniVec3 goal = {1.5f, 2.0f, 1.0f};
    maniChainSolve s = Solve(goal);
    bool r = true;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "out of reach");
    maniVec3 root = At(s_pose, 1);
    maniVec3 end = At(s_pose, BONES - 1);
    float reach = Distance(end, root);
    float toGoal = Distance(goal, root);
    float along = ((end.x - root.x) * (goal.x - root.x) + (end.y - root.y) * (goal.y - root.y) +
                   (end.z - root.z) * (goal.z - root.z)) /
                  (reach * toGoal);
    CHECK(reach > 1.2f - 1e-3f && along > 1.0f - 1e-5f, "straight toward the goal");
}

// A bone's turn from its rest.
static maniQuat FromRest(const maniSkeleton* skeleton, maniPose* pose, uint32_t bone)
{
    maniQuat q = maniPoseLocal(pose, bone).rotation;
    maniQuat r = maniSkeletonRest(skeleton, bone).rotation;
    maniQuat t = {r.w * q.x - r.x * q.w - r.y * q.z + r.z * q.y,
                  r.w * q.y + r.x * q.z - r.y * q.w - r.z * q.x,
                  r.w * q.z - r.x * q.y + r.y * q.x - r.z * q.w,
                  r.w * q.w + r.x * q.x + r.y * q.y + r.z * q.z};
    return t.w < 0.0f ? (maniQuat){-t.x, -t.y, -t.z, -t.w} : t;
}

// A cone of 0.3 with a twist of +-0.1 about y on every bone: each bone's
// swing within 0.3 and twist within 0.1; a hinge about x from -0.2 to
// 0.6: each turns about x alone within the range.
static void TestLimits(void)
{
    maniJointLimit limits[BONES - 2];
    for (uint32_t k = 0; k < BONES - 2; ++k)
    {
        limits[k] = (maniJointLimit){
            .kind = maniJointLimitCone, .cone = 0.3f, .twistLow = -0.1f, .twistHigh = 0.1f};
    }
    Bent(s_pose);
    maniChainSolve s = Solve((maniVec3){0.9f, 0.4f, 0.3f});
    s.limits = limits;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success, "limited by cones");
    bool within = true;
    for (uint32_t b = 1; b < BONES - 1; ++b)
    {
        maniQuat t = FromRest(&s_skeleton, s_pose, b);
        float twist = 2.0f * atan2f(t.y, t.w);
        maniQuat twistQ = Turn((maniVec3){0.0f, 1.0f, 0.0f}, twist);
        maniQuat swing = {t.x * twistQ.w - t.w * twistQ.x - t.y * twistQ.z + t.z * twistQ.y,
                          t.y * twistQ.w - t.w * twistQ.y - t.z * twistQ.x + t.x * twistQ.z,
                          t.z * twistQ.w - t.w * twistQ.z - t.x * twistQ.y + t.y * twistQ.x,
                          t.w * twistQ.w + t.x * twistQ.x + t.y * twistQ.y + t.z * twistQ.z};
        float swingAngle = 2.0f * acosf(fminf(1.0f, fabsf(swing.w)));
        within = within && swingAngle <= 0.3f + 1e-4f && fabsf(twist) <= 0.1f + 1e-4f;
    }
    CHECK(within, "each swing within its cone and twist within its range");
    CHECK(LengthsKept(s_pose), "lengths kept under limits");
    for (uint32_t k = 0; k < BONES - 2; ++k)
    {
        limits[k] = (maniJointLimit){
            .kind = maniJointLimitHinge, .axis = {1.0f, 0.0f, 0.0f}, .low = -0.2f, .high = 0.6f};
    }
    Bent(s_pose);
    s.goal = (maniVec3){0.0f, 0.8f, 0.7f};
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success, "limited by hinges");
    within = true;
    for (uint32_t b = 1; b < BONES - 1; ++b)
    {
        maniQuat t = FromRest(&s_skeleton, s_pose, b);
        float angle = 2.0f * atan2f(t.x, t.w);
        within = within && fabsf(t.y) < 1e-6f && fabsf(t.z) < 1e-6f && angle >= -0.2f - 1e-5f &&
                 angle <= 0.6f + 1e-5f;
    }
    CHECK(within, "each turns about the hinge alone within its range");
    maniVec3 end = At(s_pose, BONES - 1);
    CHECK(fabsf(end.x) < 1e-5f, "the chain stays in the hinges' plane");
}

// The angle between two unit rotations.
static float Angle(maniQuat a, maniQuat b)
{
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

// The turned chain bent a little off its rest about y and z.
static void TurnedBent(maniPose* pose)
{
    CHECK(maniPoseSetRest(pose, &s_turned) == mani_success, "turned rest");
    for (uint32_t b = 1; b < BONES; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        t.rotation = Mul(t.rotation, Turn((maniVec3){0.0f, 0.6f, 0.8f}, 0.15f));
        maniPoseSetLocal(pose, b, t);
    }
}

// Whether two poses of the skeleton have the same local bits.
static bool SameLocals(const maniPose* a, const maniPose* b)
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

// The turned chain, its bones along x resting turned about z, under a
// cone of 0.3 with a twist of +-0.1: each turn from rest split about the
// bone's own axis, x, has its swing within 0.3 and twist within 0.1.
// Limits all of no kind solve as none, bit for bit; limits wider than
// every turn from rest as none but for rounding.
static void TestTurnedRest(void)
{
    maniJointLimit limits[BONES - 2];
    for (uint32_t k = 0; k < BONES - 2; ++k)
    {
        limits[k] = (maniJointLimit){
            .kind = maniJointLimitCone, .cone = 0.3f, .twistLow = -0.1f, .twistHigh = 0.1f};
    }
    TurnedBent(s_turnedPose);
    maniChainSolve s = Solve((maniVec3){0.2f, 0.9f, 0.5f});
    s.limits = limits;
    CHECK(maniSolveCcd(s_turnedPose, &s_turned, &s, NULL) == mani_success, "turned, limited");
    bool within = true;
    for (uint32_t b = 1; b < BONES - 1; ++b)
    {
        maniQuat t = FromRest(&s_turned, s_turnedPose, b);
        float twist = 2.0f * atan2f(t.x, t.w);
        maniQuat twistQ = Turn((maniVec3){1.0f, 0.0f, 0.0f}, -twist);
        float swingAngle = 2.0f * acosf(fminf(1.0f, fabsf(Mul(t, twistQ).w)));
        within = within && swingAngle <= 0.3f + 1e-4f && fabsf(twist) <= 0.1f + 1e-4f;
    }
    CHECK(within, "limits about each bone's own axis from its rest");
    for (uint32_t k = 0; k < BONES - 2; ++k)
    {
        limits[k] = (maniJointLimit){.kind = maniJointLimitNone};
    }
    TurnedBent(s_turnedPose);
    TurnedBent(s_turnedOther);
    maniChainSolve free = s;
    free.limits = NULL;
    CHECK(maniSolveCcd(s_turnedPose, &s_turned, &s, NULL) == mani_success &&
              maniSolveCcd(s_turnedOther, &s_turned, &free, NULL) == mani_success &&
              SameLocals(s_turnedPose, s_turnedOther),
          "limits of no kind solve as none");
    for (uint32_t k = 0; k < BONES - 2; ++k)
    {
        limits[k] = (maniJointLimit){
            .kind = maniJointLimitCone, .cone = 3.1f, .twistLow = -3.1f, .twistHigh = 3.1f};
    }
    TurnedBent(s_turnedPose);
    CHECK(maniSolveCcd(s_turnedPose, &s_turned, &s, NULL) == mani_success, "wide limits");
    float worst = 0.0f;
    for (uint32_t b = 1; b < BONES; ++b)
    {
        worst = fmaxf(worst, Angle(maniPoseLocal(s_turnedPose, b).rotation,
                                   maniPoseLocal(s_turnedOther, b).rotation));
    }
    CHECK(worst < 1e-3f, "limits wider than every turn from rest solve as none");
}

static void TestStep(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniChainSolve s = Solve((maniVec3){0.8f, 0.2f, -0.6f});
    s.iterations = 1;
    s.step = 0.05f;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success, "a step bound");
    bool bounded = true;
    bool turned = false;
    for (uint32_t b = 1; b < BONES - 1; ++b)
    {
        float a = Angle(maniPoseLocal(s_pose, b).rotation, maniPoseLocal(s_other, b).rotation);
        bounded = bounded && a <= 0.05f + 1e-4f;
        turned = turned || a > 0.04f;
    }
    CHECK(bounded && turned, "each bone turned at most the step in one sweep");
}

static void TestIdempotent(void)
{
    Bent(s_pose);
    maniChainSolve s = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    bool r = false;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, &r) == mani_success && r, "reached");
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniPoseSetLocal(s_other, b, maniPoseLocal(s_pose, b));
    }
    CHECK(maniSolveCcd(s_other, &s_skeleton, &s, &r) == mani_success && r, "again");
    CHECK(SameLocals(s_pose, s_other), "a solve on its own reached result unchanged");
}

static void TestEndRotation(void)
{
    Bent(s_pose);
    maniChainSolve s = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    s.rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 1.2f);
    s.rotationWeight = 1.0f;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success, "an end rotation");
    CHECK(maniComputeModel(s_pose, &s_skeleton) == mani_success &&
              Angle(maniPoseModelTransform(s_pose, BONES - 1).rotation, s.rotation) < 1e-5f &&
              Distance(At(s_pose, BONES - 1), s.goal) <= 1e-4f,
          "the end takes the wanted rotation, its origin kept");
    Bent(s_pose);
    s.rotationWeight = 0.5f;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success &&
              maniComputeModel(s_pose, &s_skeleton) == mani_success,
          "half of it");
    float half = Angle(maniPoseModelTransform(s_pose, BONES - 1).rotation, s.rotation);
    CHECK(half > 0.05f && half < 1.2f, "half the way");
}

static void TestWeight(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniChainSolve s = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    s.weight = 0.0f;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success, "weight 0");
    bool same = true;
    for (uint32_t b = 0; b < BONES; ++b)
    {
        same = same &&
               Angle(maniPoseLocal(s_pose, b).rotation, maniPoseLocal(s_other, b).rotation) < 1e-6f;
    }
    CHECK(same, "weight 0 leaves the pose");
    s.weight = 0.5f;
    bool r = true;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "weight 0.5");
    float part = Distance(At(s_pose, BONES - 1), s.goal);
    float none = Distance(At(s_other, BONES - 1), s.goal);
    CHECK(part > 1e-3f && part < none, "half the correction");
}

static void TestEdges(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniChainSolve s = Solve(At(s_pose, 1));
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_success, "a goal at the root");
    maniTransform root[2] = {maniPoseLocal(s_pose, 1), maniPoseLocal(s_other, 1)};
    CHECK(memcmp(&root[0], &root[1], sizeof root[0]) == 0,
          "the root, its direction to the goal none, left as it was");
    bool finite = true;
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniQuat q = maniPoseLocal(s_pose, b).rotation;
        finite = finite && isfinite(q.x) && isfinite(q.y) && isfinite(q.z) && isfinite(q.w);
    }
    CHECK(finite, "no NaN");
    Bent(s_pose);
    Bent(s_other);
    maniVec3 end = At(s_pose, BONES - 1);
    s = Solve((maniVec3){end.x + 0.01f, end.y, end.z});
    s.tolerance = 0.05f;
    bool r = false;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
              SameLocals(s_pose, s_other),
          "a goal within the tolerance: reached, the pose left as it was");
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    s = Solve(At(s_pose, BONES - 1));
    s.tolerance = 0.0f;
    r = false;
    CHECK(maniSolveCcd(s_pose, &s_skeleton, &s, &r) == mani_success && r,
          "a goal at the end with tolerance 0 reached");
    CHECK(maniPoseSetRest(s_longPose, &s_long) == mani_success, "the long pose");
    maniChainSolve l = Solve((maniVec3){1.0f, 2.0f, 0.0f});
    l.root = 1;
    l.end = 32;
    CHECK(maniSolveCcd(s_longPose, &s_long, &l, NULL) == mani_success, "a path of 32 bones");
    l.root = 0;
    CHECK(maniSolveCcd(s_longPose, &s_long, &l, NULL) == mani_errorLimit &&
              maniSolveFabrik(s_longPose, &s_long, &l, NULL) == mani_errorLimit,
          "a path of 33 bones at the limit");
}

static void TestRefusals(void)
{
    Bent(s_pose);
    maniChainSolve good = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    CHECK(maniSolveCcd(NULL, &s_skeleton, &good, NULL) == mani_errorInvalid &&
              maniSolveCcd(s_pose, NULL, &good, NULL) == mani_errorInvalid &&
              maniSolveCcd(s_pose, &s_skeleton, NULL, NULL) == mani_errorInvalid,
          "NULLs");
    maniChainSolve bad[16];
    for (int k = 0; k < 16; ++k)
    {
        bad[k] = good;
    }
    bad[0].root = BONES;
    bad[1].end = 1;
    bad[2].root = 3;
    bad[2].end = 2;
    bad[3].pole = (maniVec3){0.0f, 0.0f, 1.0f};
    bad[4].rotation = (maniQuat){0.0f, 0.0f, 0.0f, 0.0f};
    bad[5].iterations = 0;
    bad[6].iterations = MANI_MAX_IK_ITERATIONS + 1;
    bad[7].tolerance = -1.0f;
    bad[8].tolerance = INFINITY;
    bad[9].step = -0.1f;
    bad[10].step = 3.2f;
    bad[11].rotationWeight = 1.5f;
    bad[12].weight = -0.5f;
    bad[13].goal.y = NAN;
    bad[14].rotation.x = NAN;
    bad[15].weight = NAN;
    int refused = 0;
    for (int k = 0; k < 16; ++k)
    {
        maniResult want = k == 6 ? mani_errorLimit : mani_errorInvalid;
        refused += maniSolveCcd(s_pose, &s_skeleton, &bad[k], NULL) == want;
    }
    CHECK(refused == 16, "bones, pole, rotation, counts and numbers out of range refused, past a "
                         "bound at the limit");
    const maniJointLimit wrong[8] = {
        {.kind = 3},
        {.kind = maniJointLimitCone, .cone = 3.2f},
        {.kind = maniJointLimitCone, .cone = 0.2f, .twistLow = 0.2f, .twistHigh = 0.1f},
        {.kind = maniJointLimitHinge, .axis = {1.1f, 0.0f, 0.0f}},
        {.kind = maniJointLimitHinge, .axis = {1.0f, 0.0f, 0.0f}, .low = 0.5f, .high = 0.4f},
        {.kind = maniJointLimitCone, .cone = NAN},
        {.kind = maniJointLimitHinge, .axis = {0.0f, 1.0f, 0.0f}, .low = -0.5f, .high = 0.5f},
        {.kind = maniJointLimitHinge, .axis = {0.0f, 0.0f, 0.0f}}};
    refused = 0;
    for (int k = 0; k < 8; ++k)
    {
        maniJointLimit limits[BONES - 2] = {{0}};
        limits[3] = wrong[k];
        maniChainSolve s = good;
        s.limits = limits;
        refused += maniSolveCcd(s_pose, &s_skeleton, &s, NULL) == mani_errorInvalid;
    }
    CHECK(refused == 8, "limits of an unknown kind, out of range, or a hinge along its bone or "
                        "of no axis refused");
    CHECK(maniPoseDifference(&s_skeleton, s_pose, s_pose, s_other) == mani_success &&
              maniSolveCcd(s_other, &s_skeleton, &good, NULL) == mani_errorInvalid,
          "a difference refused");
}

int main(void)
{
    Setup();
    TestReach();
    TestOutOfReach();
    TestLimits();
    TestTurnedRest();
    TestStep();
    TestIdempotent();
    TestEndRotation();
    TestWeight();
    TestEdges();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
