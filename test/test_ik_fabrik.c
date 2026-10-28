// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FABRIK chains on a tail of five bones of 0.3 under a root:
// goals within reach reached with every length kept; the joints' places
// after 1, 2 and 8 iterations those of a FABRIK in double precision
// written here from Aristidou and Lasenby's pseudocode; a goal out of
// reach straightening the chain; a pole turning the bend toward itself
// either way, the pole's turn coming before the first pass; a bone of
// length 0 left as it was; cone limits never exceeded; hinges that make
// the goal unreachable running the iterations out with tolerance 0, the
// result's bits pinned; a solve on its own reached result unchanged; the
// end taking the wanted rotation; weight 0; the refusals.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BONES 6u

static _Alignas(16) uint8_t s_skeletonFile[2][8192];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_memory[3][8192];
static maniSkeleton s_skeleton;
static maniSkeleton s_short;
static maniPose* s_pose;
static maniPose* s_other;
static maniPose* s_shortPose;

static maniQuat Turn(maniVec3 axis, float angle)
{
    float s = sinf(0.5f * angle);
    return (maniQuat){axis.x * s, axis.y * s, axis.z * s, cosf(0.5f * angle)};
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

// The tail of bones 0.3 along y from their parents; the bone short, if
// any, at its parent's origin.
static void Build(maniSkeleton* skeleton, uint8_t* file, size_t capacity, uint32_t shortBone)
{
    static const char* const kNames[BONES] = {"root", "a", "b", "c", "d", "e"};
    maniBoneSource bones[BONES];
    for (uint32_t b = 0; b < BONES; ++b)
    {
        float y = b == 0 || b == shortBone ? 0.0f : 0.3f;
        bones[b] =
            (maniBoneSource){kNames[b],
                             (uint32_t)strlen(kNames[b]),
                             (int32_t)b - 1,
                             {{0.0f, y, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    }
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    maniSkeletonSource source = {bones, BONES};
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
    Build(&s_skeleton, s_skeletonFile[0], sizeof s_skeletonFile[0], BONES);
    Build(&s_short, s_skeletonFile[1], sizeof s_skeletonFile[1], 3);
    CHECK(maniInitPose(&s_skeleton, s_memory[0], sizeof s_memory[0], &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_memory[1], sizeof s_memory[1], &s_other) ==
                  mani_success &&
              maniInitPose(&s_short, s_memory[2], sizeof s_memory[2], &s_shortPose) == mani_success,
          "the poses");
}

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
        CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
                  Distance(At(s_pose, BONES - 1), goals[g]) <= 1e-4f,
              "a goal within reach reached");
        CHECK(LengthsKept(s_pose), "every length kept");
    }
}

// FABRIK in double precision, the paper's: joints p[0..n-1], lengths
// d, the root fixed.
static void Reference(double (*p)[3], uint32_t n, const double* goal, int iterations)
{
    double d[BONES];
    double root[3] = {p[0][0], p[0][1], p[0][2]};
    for (uint32_t k = 0; k + 1 < n; ++k)
    {
        double x = p[k + 1][0] - p[k][0];
        double y = p[k + 1][1] - p[k][1];
        double z = p[k + 1][2] - p[k][2];
        d[k] = sqrt(x * x + y * y + z * z);
    }
    for (int i = 0; i < iterations; ++i)
    {
        for (int c = 0; c < 3; ++c)
        {
            p[n - 1][c] = goal[c];
        }
        for (uint32_t k = n - 1; k-- > 0;)
        {
            double v[3] = {p[k][0] - p[k + 1][0], p[k][1] - p[k + 1][1], p[k][2] - p[k + 1][2]};
            double r = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            for (int c = 0; c < 3; ++c)
            {
                p[k][c] = p[k + 1][c] + v[c] * d[k] / r;
            }
        }
        for (int c = 0; c < 3; ++c)
        {
            p[0][c] = root[c];
        }
        for (uint32_t k = 0; k + 1 < n; ++k)
        {
            double v[3] = {p[k + 1][0] - p[k][0], p[k + 1][1] - p[k][1], p[k + 1][2] - p[k][2]};
            double r = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            for (int c = 0; c < 3; ++c)
            {
                p[k + 1][c] = p[k][c] + v[c] * d[k] / r;
            }
        }
    }
}

static void TestReference(void)
{
    const int counts[3] = {1, 2, 8};
    const double goal[3] = {0.5, 0.7, -0.4};
    float worst = 0.0f;
    for (int c = 0; c < 3; ++c)
    {
        Bent(s_pose);
        double p[BONES - 1][3];
        for (uint32_t b = 1; b < BONES; ++b)
        {
            maniVec3 v = At(s_pose, b);
            p[b - 1][0] = (double)v.x;
            p[b - 1][1] = (double)v.y;
            p[b - 1][2] = (double)v.z;
        }
        Reference(p, BONES - 1, goal, counts[c]);
        maniChainSolve s = Solve((maniVec3){0.5f, 0.7f, -0.4f});
        s.iterations = (uint32_t)counts[c];
        s.tolerance = 0.0f;
        CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, NULL) == mani_success, "solved");
        for (uint32_t b = 1; b < BONES; ++b)
        {
            maniVec3 v = At(s_pose, b);
            maniVec3 r = {(float)p[b - 1][0], (float)p[b - 1][1], (float)p[b - 1][2]};
            worst = fmaxf(worst, Distance(v, r));
        }
    }
    CHECK(worst < 2e-6f, "the paper's places after 1, 2 and 8 iterations");
}

static void TestOutOfReach(void)
{
    Bent(s_pose);
    maniVec3 goal = {1.5f, 2.0f, 1.0f};
    maniChainSolve s = Solve(goal);
    bool r = true;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "out of reach");
    maniVec3 root = At(s_pose, 1);
    maniVec3 end = At(s_pose, BONES - 1);
    float reach = Distance(end, root);
    float along = ((end.x - root.x) * (goal.x - root.x) + (end.y - root.y) * (goal.y - root.y) +
                   (end.z - root.z) * (goal.z - root.z)) /
                  (reach * Distance(goal, root));
    CHECK(reach > 1.2f - 1e-3f && along > 1.0f - 1e-5f, "straight toward the goal");
}

// The middle joints' mean offset from the root-to-goal line, along z.
static float Bend(maniPose* pose, maniVec3 goal)
{
    maniVec3 root = At(pose, 1);
    maniVec3 u = {goal.x - root.x, goal.y - root.y, goal.z - root.z};
    float l = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
    u = (maniVec3){u.x / l, u.y / l, u.z / l};
    float z = 0.0f;
    for (uint32_t b = 2; b < BONES - 1; ++b)
    {
        maniVec3 v = At(pose, b);
        maniVec3 d = {v.x - root.x, v.y - root.y, v.z - root.z};
        float along = d.x * u.x + d.y * u.y + d.z * u.z;
        z += d.z - along * u.z;
    }
    return z;
}

static void TestPole(void)
{
    maniVec3 goal = {0.6f, 0.6f, 0.0f};
    const float sides[2] = {1.0f, -1.0f};
    for (int k = 0; k < 2; ++k)
    {
        Bent(s_pose);
        maniChainSolve s = Solve(goal);
        s.pole = (maniVec3){0.0f, 0.0f, sides[k]};
        bool r = false;
        CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, &r) == mani_success && r, "with a pole");
        CHECK(Bend(s_pose, goal) * sides[k] > 0.05f, "the bend toward the pole");
        CHECK(LengthsKept(s_pose), "lengths kept with a pole");
    }
}

// A pole solve of one iteration is the pole's turn, a solve whose
// tolerance takes the start, then a pass without a pole: the joints the
// same but for rounding.
static void TestPoleFirst(void)
{
    maniVec3 goal = {0.6f, 0.6f, 0.0f};
    Bent(s_pose);
    maniChainSolve s = Solve(goal);
    s.pole = (maniVec3){0.0f, 0.0f, -1.0f};
    s.iterations = 1;
    s.tolerance = 0.0f;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, NULL) == mani_success, "a pole, one pass");
    Bent(s_other);
    maniChainSolve turn = s;
    turn.tolerance = 100.0f;
    maniChainSolve pass = s;
    pass.pole = (maniVec3){0.0f, 0.0f, 0.0f};
    CHECK(maniSolveFabrik(s_other, &s_skeleton, &turn, NULL) == mani_success &&
              maniSolveFabrik(s_other, &s_skeleton, &pass, NULL) == mani_success,
          "the pole's turn, then a pass");
    float worst = 0.0f;
    for (uint32_t b = 1; b < BONES; ++b)
    {
        worst = fmaxf(worst, Distance(At(s_pose, b), At(s_other, b)));
    }
    CHECK(worst < 1e-5f, "the pass starts from the pole's turn");
}

// A bone of length 0 has no direction to turn: left as it was.
static void TestShortBone(void)
{
    CHECK(maniPoseSetRest(s_shortPose, &s_short) == mani_success, "short rest");
    for (uint32_t b = 1; b < BONES; ++b)
    {
        maniTransform t = maniPoseLocal(s_shortPose, b);
        t.rotation = Turn((maniVec3){0.6f, 0.0f, 0.8f}, 0.15f);
        maniPoseSetLocal(s_shortPose, b, t);
    }
    maniTransform before = maniPoseLocal(s_shortPose, 2);
    maniChainSolve s = Solve((maniVec3){0.3f, 0.5f, 0.2f});
    bool r = false;
    CHECK(maniSolveFabrik(s_shortPose, &s_short, &s, &r) == mani_success && r, "reached");
    maniTransform after = maniPoseLocal(s_shortPose, 2);
    CHECK(memcmp(&before, &after, sizeof before) == 0, "the bone of length 0 left as it was");
}

static maniQuat FromRest(maniPose* pose, uint32_t bone)
{
    maniQuat t = maniPoseLocal(pose, bone).rotation;
    return t.w < 0.0f ? (maniQuat){-t.x, -t.y, -t.z, -t.w} : t;
}

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
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, NULL) == mani_success, "limited by cones");
    bool within = true;
    for (uint32_t b = 1; b < BONES - 1; ++b)
    {
        maniQuat t = FromRest(s_pose, b);
        float twist = 2.0f * atan2f(t.y, t.w);
        maniQuat q = Turn((maniVec3){0.0f, 1.0f, 0.0f}, -twist);
        float sw = t.w * q.w - t.x * q.x - t.y * q.y - t.z * q.z;
        within = within && 2.0f * acosf(fminf(1.0f, fabsf(sw))) <= 0.3f + 1e-4f &&
                 fabsf(twist) <= 0.1f + 1e-4f;
    }
    CHECK(within, "each swing within its cone and twist within its range");
    CHECK(LengthsKept(s_pose), "lengths kept under limits");
}

// Hinges about x from 0 to 0.2 and a goal off to the side: unreachable
// by the limits, the iterations run out with tolerance 0; the result
// finite, within the hinges, the same bits on every platform.
static void TestHingedOut(void)
{
    maniJointLimit limits[BONES - 2];
    for (uint32_t k = 0; k < BONES - 2; ++k)
    {
        limits[k] = (maniJointLimit){
            .kind = maniJointLimitHinge, .axis = {1.0f, 0.0f, 0.0f}, .low = 0.0f, .high = 0.2f};
    }
    Bent(s_pose);
    maniChainSolve s = Solve((maniVec3){0.8f, 0.3f, -0.5f});
    s.limits = limits;
    s.tolerance = 0.0f;
    bool r = true;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, &r) == mani_success && !r,
          "unreachable by its limits");
    bool within = true;
    for (uint32_t b = 1; b < BONES; ++b)
    {
        maniQuat q = FromRest(s_pose, b);
        float angle = 2.0f * atan2f(q.x, q.w);
        within = within && isfinite(q.w) &&
                 (b == BONES - 1 || (fabsf(q.y) < 1e-6f && fabsf(q.z) < 1e-6f && angle >= -1e-5f &&
                                     angle <= 0.2f + 1e-5f));
    }
    CHECK(within, "within the hinges");
}

static void TestIdempotent(void)
{
    Bent(s_pose);
    maniChainSolve s = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    bool r = false;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, &r) == mani_success && r, "reached");
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniPoseSetLocal(s_other, b, maniPoseLocal(s_pose, b));
    }
    CHECK(maniSolveFabrik(s_other, &s_skeleton, &s, &r) == mani_success && r, "again");
    bool same = true;
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniTransform x = maniPoseLocal(s_pose, b);
        maniTransform y = maniPoseLocal(s_other, b);
        same = same && memcmp(&x, &y, sizeof x) == 0;
    }
    CHECK(same, "a solve on its own reached result unchanged");
}

static float Angle(maniQuat a, maniQuat b)
{
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

static void TestEndAndWeight(void)
{
    Bent(s_pose);
    maniChainSolve s = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    s.rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 1.2f);
    s.rotationWeight = 1.0f;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, NULL) == mani_success &&
              maniComputeModel(s_pose, &s_skeleton) == mani_success &&
              Angle(maniPoseModelTransform(s_pose, BONES - 1).rotation, s.rotation) < 1e-5f,
          "the end takes the wanted rotation");
    Bent(s_pose);
    Bent(s_other);
    s = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    s.weight = 0.0f;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &s, NULL) == mani_success, "weight 0");
    bool same = true;
    for (uint32_t b = 0; b < BONES; ++b)
    {
        same = same &&
               Angle(maniPoseLocal(s_pose, b).rotation, maniPoseLocal(s_other, b).rotation) < 1e-6f;
    }
    CHECK(same, "weight 0 leaves the pose");
}

static void TestRefusals(void)
{
    Bent(s_pose);
    maniChainSolve good = Solve((maniVec3){0.4f, 0.6f, 0.3f});
    maniChainSolve stepped = good;
    stepped.step = 0.1f;
    maniChainSolve far = good;
    far.root = 3;
    far.end = 2;
    maniChainSolve pole = good;
    pole.pole = (maniVec3){NAN, 0.0f, 0.0f};
    CHECK(maniSolveFabrik(NULL, &s_skeleton, &good, NULL) == mani_errorInvalid &&
              maniSolveFabrik(s_pose, NULL, &good, NULL) == mani_errorInvalid &&
              maniSolveFabrik(s_pose, &s_skeleton, NULL, NULL) == mani_errorInvalid &&
              maniSolveFabrik(s_pose, &s_skeleton, &stepped, NULL) == mani_errorInvalid &&
              maniSolveFabrik(s_pose, &s_skeleton, &far, NULL) == mani_errorInvalid &&
              maniSolveFabrik(s_pose, &s_skeleton, &pole, NULL) == mani_errorInvalid,
          "NULLs, a step, a root below the end, a pole not finite");
    maniChainSolve many = good;
    many.iterations = MANI_MAX_IK_ITERATIONS + 1;
    CHECK(maniSolveFabrik(s_pose, &s_skeleton, &many, NULL) == mani_errorLimit,
          "iterations past MANI_MAX_IK_ITERATIONS at the limit");
}

int main(void)
{
    Setup();
    TestReach();
    TestReference();
    TestOutOfReach();
    TestPole();
    TestPoleFirst();
    TestShortBone();
    TestLimits();
    TestHingedOut();
    TestIdempotent();
    TestEndAndWeight();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
