// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Two-bone inverse kinematics (mani-0029) on a leg: hip, knee, a shin
// twist bone and the ankle (the end two bones below the middle). Goals
// within reach are reached with the bones' lengths kept and the knee
// toward the pole; a goal out of reach straightens the leg toward it,
// softening keeps it short of full extension; a solve on its own result
// changes nothing; a pole swept across the goal line moves the knee
// continuously; weight 0 changes nothing; a goal at the hip turns
// nothing there; the hinge given or the bend plane's alike; a mirrored
// parent; the refusals.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    kRoot,
    kHip,
    kKnee,
    kShin,
    kAnkle,
    kBones,
};

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_memory[2][4096];
static maniSkeleton s_skeleton;
static maniPose* s_pose;
static maniPose* s_other;
static uint32_t s_bone[kBones];

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

static maniVec3 At(maniPose* pose, int bone)
{
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "the model");
    maniAffine m = maniPoseModel(pose, s_bone[bone]);
    return (maniVec3){m.m[9], m.m[10], m.m[11]};
}

static void Setup(void)
{
    const maniBoneSource bones[kBones] = {
        {"root", 4, -1, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"hip", 3, 0, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"knee", 4, 1, {{0.0f, -0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"shin", 4, 2, {{0.0f, -0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"ankle", 5, 3, {{0.0f, -0.25f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}}};
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    maniSkeletonSource source = {bones, kBones, 0};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success &&
              maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "the skeleton");
    maniDestroyBuilder(builder);
    const char* names[kBones] = {"root", "hip", "knee", "shin", "ankle"};
    for (int b = 0; b < kBones; ++b)
    {
        s_bone[b] = maniSkeletonFindBone(&s_skeleton, names[b], (uint32_t)strlen(names[b]));
    }
    CHECK(maniInitPose(&s_skeleton, s_memory[0], sizeof s_memory[0], &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_memory[1], sizeof s_memory[1], &s_other) == mani_success,
          "the poses");
}

// The leg at rest with the knee bent forward (about x) by bend and the
// hip turned about y by turn.
static void Bent(maniPose* pose, float bend, float turn)
{
    CHECK(maniPoseSetRest(pose, &s_skeleton) == mani_success, "rest");
    maniTransform knee = maniPoseLocal(pose, s_bone[kKnee]);
    knee.rotation = Turn((maniVec3){1.0f, 0.0f, 0.0f}, bend);
    maniPoseSetLocal(pose, s_bone[kKnee], knee);
    maniTransform hip = maniPoseLocal(pose, s_bone[kHip]);
    hip.rotation = Turn((maniVec3){0.0f, 1.0f, 0.0f}, turn);
    maniPoseSetLocal(pose, s_bone[kHip], hip);
}

static maniTwoBoneSolve Solve(maniVec3 goal, maniVec3 pole)
{
    return (maniTwoBoneSolve){.start = s_bone[kHip],
                              .middle = s_bone[kKnee],
                              .end = s_bone[kAnkle],
                              .goal = goal,
                              .pole = pole,
                              .soften = 1.0f,
                              .weight = 1.0f};
}

static void TestReach(void)
{
    // Goals around the hip within reach (0.05 to 0.95 away), the knee
    // toward a pole ahead (+z).
    const maniVec3 goals[4] = {
        {0.1f, 0.3f, 0.2f}, {-0.3f, 0.6f, 0.4f}, {0.0f, 0.2f, 0.0f}, {0.5f, 0.9f, -0.2f}};
    bool reached = true;
    bool kept = true;
    bool toward = true;
    for (int k = 0; k < 4; ++k)
    {
        Bent(s_pose, 0.4f, 0.3f);
        maniTwoBoneSolve solve = Solve(goals[k], (maniVec3){0.0f, 0.0f, 1.0f});
        bool r = false;
        CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, &r) == mani_success, "solved");
        maniVec3 hip = At(s_pose, kHip);
        maniVec3 knee = At(s_pose, kKnee);
        maniVec3 ankle = At(s_pose, kAnkle);
        reached = reached && r && Distance(ankle, goals[k]) < 1e-5f;
        kept = kept && fabsf(Distance(hip, knee) - 0.5f) < 1e-5f &&
               fabsf(Distance(knee, ankle) - 0.45f) < 1e-5f;
        // The knee off the hip-to-goal line toward the pole's side.
        maniVec3 u = {goals[k].x - hip.x, goals[k].y - hip.y, goals[k].z - hip.z};
        float l = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
        maniVec3 d = {knee.x - hip.x, knee.y - hip.y, knee.z - hip.z};
        float along = (d.x * u.x + d.y * u.y + d.z * u.z) / (l * l);
        float off = d.z - along * u.z;
        toward = toward && off > 0.0f;
    }
    CHECK(reached, "goals within reach reached");
    CHECK(kept, "the bones' lengths kept");
    CHECK(toward, "the knee toward the pole");
}

static void TestOutOfReach(void)
{
    // 2 m away: straight toward it, not reached; softened, short of it.
    maniVec3 goal = {0.0f, -1.0f, 1.0f};
    Bent(s_pose, 0.4f, 0.0f);
    maniTwoBoneSolve solve = Solve(goal, (maniVec3){0.0f, 0.0f, 1.0f});
    bool r = true;
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, &r) == mani_success && !r, "out of reach");
    maniVec3 hip = At(s_pose, kHip);
    maniVec3 ankle = At(s_pose, kAnkle);
    float straight = Distance(hip, ankle);
    maniVec3 u = {goal.x - hip.x, goal.y - hip.y, goal.z - hip.z};
    maniVec3 d = {ankle.x - hip.x, ankle.y - hip.y, ankle.z - hip.z};
    float cosine = (u.x * d.x + u.y * d.y + u.z * d.z) / (straight * Distance(goal, hip));
    CHECK(fabsf(straight - 0.95f) < 1e-5f && cosine > 1.0f - 1e-6f, "straight toward the goal");
    Bent(s_pose, 0.4f, 0.0f);
    solve.soften = 0.8f;
    goal = (maniVec3){0.0f, 0.1f, 0.0f};
    solve.goal = goal;
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, &r) == mani_success && !r, "softened");
    float softened = Distance(At(s_pose, kHip), At(s_pose, kAnkle));
    CHECK(softened > 0.8f * 0.95f && softened < 0.9f, "short of the goal, past the soft start");
}

static void TestIdempotent(void)
{
    Bent(s_pose, 0.4f, 0.3f);
    maniTwoBoneSolve solve = Solve((maniVec3){0.2f, 0.4f, 0.3f}, (maniVec3){0.3f, 0.0f, 1.0f});
    solve.twist = 0.0f;
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, NULL) == mani_success, "once");
    maniTransform hip = maniPoseLocal(s_pose, s_bone[kHip]);
    maniTransform knee = maniPoseLocal(s_pose, s_bone[kKnee]);
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, NULL) == mani_success, "twice");
    maniTransform hip2 = maniPoseLocal(s_pose, s_bone[kHip]);
    maniTransform knee2 = maniPoseLocal(s_pose, s_bone[kKnee]);
    float apart =
        fabsf(hip.rotation.x - hip2.rotation.x) + fabsf(hip.rotation.y - hip2.rotation.y) +
        fabsf(hip.rotation.z - hip2.rotation.z) + fabsf(hip.rotation.w - hip2.rotation.w) +
        fabsf(knee.rotation.x - knee2.rotation.x) + fabsf(knee.rotation.w - knee2.rotation.w);
    CHECK(apart < 1e-5f, "a solve on its own result changes nothing");
}

static void TestPoleSweep(void)
{
    // The pole swept through the hip-to-goal line: the knee moves in
    // small steps throughout.
    maniVec3 goal = {0.0f, 0.4f, 0.1f};
    float worst = 0.0f;
    maniVec3 last = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i <= 400; ++i)
    {
        float x = -1.0f + (float)i / 200.0f;
        Bent(s_pose, 0.4f, 0.0f);
        // The line from the hip runs (0, -0.6, 0.1); the pole crosses it.
        maniTwoBoneSolve solve = Solve(goal, (maniVec3){x * 0.1f, -0.6f, 0.1f + x * 0.02f});
        CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, NULL) == mani_success, "swept");
        maniVec3 knee = At(s_pose, kKnee);
        if (i > 0)
        {
            float step = Distance(knee, last);
            worst = step > worst ? step : worst;
        }
        last = knee;
    }
    CHECK(worst < 0.02f, "the knee continuous through the pole's line");
}

static void TestEdges(void)
{
    // Weight 0: nothing turns.
    Bent(s_pose, 0.4f, 0.3f);
    Bent(s_other, 0.4f, 0.3f);
    maniTwoBoneSolve solve = Solve((maniVec3){0.3f, 0.2f, 0.2f}, (maniVec3){0.0f, 0.0f, 1.0f});
    solve.weight = 0.0f;
    bool r = true;
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, &r) == mani_success && !r, "weight 0");
    maniTransform a = maniPoseLocal(s_pose, s_bone[kHip]);
    maniTransform b = maniPoseLocal(s_other, s_bone[kHip]);
    CHECK(fabsf(a.rotation.x - b.rotation.x) + fabsf(a.rotation.y - b.rotation.y) < 1e-6f,
          "nothing turned at weight 0");
    // A goal at the hip: the hip keeps its turn, the knee folds.
    Bent(s_pose, 0.4f, 0.3f);
    solve = Solve((maniVec3){0.0f, 1.0f, 0.0f}, (maniVec3){0.0f, 0.0f, 1.0f});
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, NULL) == mani_success, "at the hip");
    a = maniPoseLocal(s_pose, s_bone[kHip]);
    CHECK(fabsf(a.rotation.y - b.rotation.y) < 1e-6f && fabsf(a.rotation.w - b.rotation.w) < 1e-6f,
          "the hip unturned for a goal at it");
    // The hinge given as the knee's x, the bend plane's: alike.
    Bent(s_pose, 0.4f, 0.3f);
    Bent(s_other, 0.4f, 0.3f);
    solve = Solve((maniVec3){0.1f, 0.3f, 0.2f}, (maniVec3){0.0f, 0.0f, 1.0f});
    maniTwoBoneSolve hinged = solve;
    hinged.hinge = (maniVec3){-1.0f, 0.0f, 0.0f};
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, NULL) == mani_success &&
              maniSolveTwoBone(s_other, &s_skeleton, &hinged, NULL) == mani_success,
          "with and without the hinge");
    CHECK(Distance(At(s_pose, kKnee), At(s_other, kKnee)) < 1e-5f, "the hinge as the bend plane's");
    // A straight leg without hinge or pole still bends to reach.
    Bent(s_pose, 0.0f, 0.0f);
    solve = Solve((maniVec3){0.0f, 0.4f, 0.0f}, (maniVec3){0.0f, 0.0f, 0.0f});
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, &r) == mani_success && r &&
              Distance(At(s_pose, kAnkle), solve.goal) < 1e-5f,
          "a straight leg bends to reach");
}

static void TestTwistAndMirror(void)
{
    // A twist turns the knee about the hip-to-goal line by its angle.
    maniVec3 goal = {0.2f, 0.4f, 0.1f};
    Bent(s_pose, 0.4f, 0.3f);
    Bent(s_other, 0.4f, 0.3f);
    maniTwoBoneSolve solve = Solve(goal, (maniVec3){0.0f, 0.0f, 1.0f});
    maniTwoBoneSolve twisted = solve;
    twisted.twist = 0.7f;
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, NULL) == mani_success &&
              maniSolveTwoBone(s_other, &s_skeleton, &twisted, NULL) == mani_success,
          "twisted and not");
    maniVec3 hip = At(s_pose, kHip);
    maniVec3 u = {goal.x - hip.x, goal.y - hip.y, goal.z - hip.z};
    float l = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
    u = (maniVec3){u.x / l, u.y / l, u.z / l};
    maniVec3 k = At(s_pose, kKnee);
    maniVec3 d = {k.x - hip.x, k.y - hip.y, k.z - hip.z};
    // d turned by 0.7 about u (Rodrigues).
    float c = cosf(0.7f);
    float sn = sinf(0.7f);
    float dot = d.x * u.x + d.y * u.y + d.z * u.z;
    maniVec3 cross = {u.y * d.z - u.z * d.y, u.z * d.x - u.x * d.z, u.x * d.y - u.y * d.x};
    maniVec3 turned = {hip.x + d.x * c + cross.x * sn + u.x * dot * (1.0f - c),
                       hip.y + d.y * c + cross.y * sn + u.y * dot * (1.0f - c),
                       hip.z + d.z * c + cross.z * sn + u.z * dot * (1.0f - c)};
    CHECK(Distance(At(s_other, kKnee), turned) < 1e-5f &&
              Distance(At(s_other, kAnkle), goal) < 1e-5f,
          "the twist turns the bend about the goal line");
    // Under a root mirrored through x the goal is still reached, the
    // knee toward the pole.
    Bent(s_pose, 0.4f, 0.3f);
    maniTransform root = maniPoseLocal(s_pose, s_bone[kRoot]);
    root.scale = (maniVec3){-1.0f, 1.0f, 1.0f};
    root.rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 0.2f);
    maniPoseSetLocal(s_pose, s_bone[kRoot], root);
    goal = (maniVec3){0.3f, 0.5f, 0.2f};
    solve = Solve(goal, (maniVec3){0.0f, 0.0f, 1.0f});
    bool r = false;
    CHECK(maniSolveTwoBone(s_pose, &s_skeleton, &solve, &r) == mani_success && r, "under a mirror");
    hip = At(s_pose, kHip);
    k = At(s_pose, kKnee);
    u = (maniVec3){goal.x - hip.x, goal.y - hip.y, goal.z - hip.z};
    l = u.x * u.x + u.y * u.y + u.z * u.z;
    d = (maniVec3){k.x - hip.x, k.y - hip.y, k.z - hip.z};
    float along = (d.x * u.x + d.y * u.y + d.z * u.z) / l;
    CHECK(Distance(At(s_pose, kAnkle), goal) < 1e-5f && d.z - along * u.z > 0.0f,
          "the goal reached under a mirrored root, the knee toward the pole");
}

static void TestRefusals(void)
{
    Bent(s_pose, 0.4f, 0.0f);
    maniTwoBoneSolve good = Solve((maniVec3){0.1f, 0.3f, 0.2f}, (maniVec3){0.0f, 0.0f, 1.0f});
    maniTwoBoneSolve bad[9];
    for (int k = 0; k < 9; ++k)
    {
        bad[k] = good;
    }
    bad[0].start = s_bone[kKnee];
    bad[1].middle = s_bone[kRoot];
    bad[2].end = 99;
    bad[3].goal.x = NAN;
    bad[4].pole.y = INFINITY;
    bad[5].hinge.z = NAN;
    bad[6].twist = INFINITY;
    bad[7].soften = 1.5f;
    bad[8].weight = -0.1f;
    bool refused = maniSolveTwoBone(NULL, &s_skeleton, &good, NULL) == mani_errorInvalid &&
                   maniSolveTwoBone(s_pose, NULL, &good, NULL) == mani_errorInvalid &&
                   maniSolveTwoBone(s_pose, &s_skeleton, NULL, NULL) == mani_errorInvalid;
    for (int k = 0; k < 9; ++k)
    {
        refused =
            refused && maniSolveTwoBone(s_pose, &s_skeleton, &bad[k], NULL) == mani_errorInvalid;
    }
    CHECK(refused, "the refusals");
}

int main(void)
{
    Setup();
    TestReach();
    TestOutOfReach();
    TestIdempotent();
    TestPoleSweep();
    TestEdges();
    TestTwistAndMirror();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
