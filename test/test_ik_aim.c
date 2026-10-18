// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Aim and look-at (mani-0029) on a spine, neck and head, the head's
// forward its z and its up its y, the eyes 0.1 ahead and 0.05 up: a head
// aimed at goals all around points at them, its up toward the pole; a
// twist turns the up about the aim line; the eyes' line passes through
// the goal; spread over the three bones by
// shares it ends aimed with every bone turned, shares of 1, 0, 0 too; a
// cone stops the forward at its edge; a pole swept through the aim line
// turns the head continuously; weight 0 changes nothing and a second aim
// nothing more; a goal at the head turns nothing; the refusals.

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
    kSpine,
    kNeck,
    kHead,
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

static maniVec3 Rotate(maniQuat q, maniVec3 v)
{
    maniVec3 u = {q.x, q.y, q.z};
    maniVec3 t = {2.0f * (u.y * v.z - u.z * v.y), 2.0f * (u.z * v.x - u.x * v.z),
                  2.0f * (u.x * v.y - u.y * v.x)};
    return (maniVec3){v.x + q.w * t.x + u.y * t.z - u.z * t.y,
                      v.y + q.w * t.y + u.z * t.x - u.x * t.z,
                      v.z + q.w * t.z + u.x * t.y - u.y * t.x};
}

static float Dot(maniVec3 a, maniVec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static maniVec3 Sub(maniVec3 a, maniVec3 b)
{
    return (maniVec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static float Length(maniVec3 v)
{
    return sqrtf(Dot(v, v));
}

static maniTransform Model(maniPose* pose, int bone)
{
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "the model");
    return maniPoseModelTransform(pose, s_bone[bone]);
}

// The angle between the head's forward (from its eyes for eyes) and the
// direction to the goal.
static float Off(maniPose* pose, maniVec3 goal, bool eyes)
{
    maniTransform head = Model(pose, kHead);
    maniVec3 from = head.translation;
    if (eyes)
    {
        maniVec3 o = Rotate(head.rotation, (maniVec3){0.0f, 0.05f, 0.1f});
        from = (maniVec3){from.x + o.x, from.y + o.y, from.z + o.z};
    }
    maniVec3 f = Rotate(head.rotation, (maniVec3){0.0f, 0.0f, 1.0f});
    maniVec3 t = Sub(goal, from);
    float c = Dot(f, t) / Length(t);
    return acosf(c > 1.0f ? 1.0f : c);
}

static void Setup(void)
{
    const maniBoneSource bones[kBones] = {
        {"root", 4, -1, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"spine", 5, 0, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"neck", 4, 1, {{0.0f, 0.4f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"head", 4, 2, {{0.0f, 0.15f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}}};
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
    const char* names[kBones] = {"root", "spine", "neck", "head"};
    for (int b = 0; b < kBones; ++b)
    {
        s_bone[b] = maniSkeletonFindBone(&s_skeleton, names[b], (uint32_t)strlen(names[b]));
    }
    CHECK(maniInitPose(&s_skeleton, s_memory[0], sizeof s_memory[0], &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_memory[1], sizeof s_memory[1], &s_other) == mani_success,
          "the poses");
}

// The pose at rest with the spine bent forward and the neck turned.
static void Posed(maniPose* pose)
{
    CHECK(maniPoseSetRest(pose, &s_skeleton) == mani_success, "rest");
    maniTransform t = maniPoseLocal(pose, s_bone[kSpine]);
    t.rotation = Turn((maniVec3){1.0f, 0.0f, 0.0f}, 0.2f);
    maniPoseSetLocal(pose, s_bone[kSpine], t);
    t = maniPoseLocal(pose, s_bone[kNeck]);
    t.rotation = Turn((maniVec3){0.0f, 1.0f, 0.0f}, -0.3f);
    maniPoseSetLocal(pose, s_bone[kNeck], t);
}

static maniAimSolve Head(maniVec3 goal, maniVec3 pole)
{
    maniAimSolve s = {.count = 1,
                      .goal = goal,
                      .pole = pole,
                      .forward = {0.0f, 0.0f, 1.0f},
                      .up = {0.0f, 1.0f, 0.0f},
                      .weight = 1.0f};
    s.bones[0] = s_bone[kHead];
    s.shares[0] = 1.0f;
    return s;
}

static void TestAim(void)
{
    const maniVec3 goals[5] = {{0.0f, 1.5f, 2.0f},
                               {2.0f, 1.0f, 0.5f},
                               {-1.0f, 3.0f, -0.5f},
                               {0.3f, 1.6f, -2.0f},
                               {-0.5f, 0.2f, 0.4f}};
    bool aimed = true;
    bool upward = true;
    for (int k = 0; k < 5; ++k)
    {
        Posed(s_pose);
        maniAimSolve s = Head(goals[k], (maniVec3){0.0f, 1.0f, 0.0f});
        bool r = false;
        CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success, "aimed");
        aimed = aimed && r && Off(s_pose, goals[k], false) < 1e-3f;
        // The up axis on the pole's side of the aim line.
        maniTransform head = Model(s_pose, kHead);
        maniVec3 up = Rotate(head.rotation, (maniVec3){0.0f, 1.0f, 0.0f});
        maniVec3 t = Sub(goals[k], head.translation);
        float along = Dot(up, t) / Dot(t, t);
        upward = upward && up.y - along * t.y > 0.0f;
    }
    CHECK(aimed, "the head points at each goal");
    CHECK(upward, "its up toward the pole");
    // A twist turns the up axis about the aim line by its angle.
    Posed(s_pose);
    Posed(s_other);
    maniAimSolve plain = Head(goals[1], (maniVec3){0.0f, 1.0f, 0.0f});
    maniAimSolve twisted = plain;
    twisted.twist = 0.5f;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &plain, NULL) == mani_success &&
              maniSolveAim(s_other, &s_skeleton, &twisted, NULL) == mani_success,
          "twisted and not");
    maniVec3 u0 = Rotate(Model(s_pose, kHead).rotation, (maniVec3){0.0f, 1.0f, 0.0f});
    maniVec3 u1 = Rotate(Model(s_other, kHead).rotation, (maniVec3){0.0f, 1.0f, 0.0f});
    CHECK(fabsf(acosf(Dot(u0, u1)) - 0.5f) < 1e-3f && Off(s_other, goals[1], false) < 1e-3f,
          "the twist about the aim line");
    // The eyes' line through the goal.
    Posed(s_pose);
    maniAimSolve s = Head(goals[1], (maniVec3){0.0f, 1.0f, 0.0f});
    s.offset = (maniVec3){0.0f, 0.05f, 0.1f};
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success &&
              Off(s_pose, goals[1], true) < 1e-3f,
          "the eyes aimed");
}

static void TestChain(void)
{
    maniVec3 goal = {1.5f, 0.8f, 1.0f};
    const float shares[2][3] = {{0.2f, 0.3f, 0.5f}, {1.0f, 0.0f, 0.0f}};
    for (int k = 0; k < 2; ++k)
    {
        Posed(s_pose);
        Posed(s_other);
        maniAimSolve s = Head(goal, (maniVec3){0.0f, 1.0f, 0.0f});
        s.count = 3;
        for (int b = 0; b < 3; ++b)
        {
            s.bones[b] = s_bone[kSpine + b];
            s.shares[b] = shares[k][b];
        }
        bool r = false;
        CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
                  Off(s_pose, goal, false) < 1e-3f,
              "the chain ends aimed");
        bool turned = true;
        for (int b = kSpine; b <= kHead; ++b)
        {
            maniQuat a = maniPoseLocal(s_pose, s_bone[b]).rotation;
            maniQuat q = maniPoseLocal(s_other, s_bone[b]).rotation;
            float apart = fabsf(a.x - q.x) + fabsf(a.y - q.y) + fabsf(a.z - q.z);
            turned = turned && (k == 1 && b != kSpine ? true : apart > 1e-3f);
        }
        CHECK(turned, "the bones sharing the turn each turned");
    }
}

// A chain of the spine and the head, the turned neck between them left
// out of it: the head ends aimed.
static void TestGap(void)
{
    const maniVec3 goals[2] = {{1.5f, 0.8f, 1.0f}, {-0.5f, 2.5f, 0.6f}};
    for (int g = 0; g < 2; ++g)
    {
        Posed(s_pose);
        maniAimSolve s = Head(goals[g], (maniVec3){0.0f, 1.0f, 0.0f});
        s.count = 2;
        s.bones[0] = s_bone[kSpine];
        s.bones[1] = s_bone[kHead];
        s.shares[0] = 1.0f;
        s.shares[1] = 1.0f;
        bool r = false;
        CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
                  Off(s_pose, goals[g], false) < 1e-3f,
              "a chain with a bone between its bones ends aimed");
    }
}

static float Angle(maniQuat a, maniQuat b)
{
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

static void TestShares(void)
{
    // Equal shares: the spine turns a third of the head's own aim turn.
    maniVec3 goal = {1.5f, 0.8f, 1.0f};
    Posed(s_pose);
    Posed(s_other);
    maniAimSolve one = Head(goal, (maniVec3){0.0f, 1.0f, 0.0f});
    CHECK(maniSolveAim(s_other, &s_skeleton, &one, NULL) == mani_success, "the head alone");
    float whole = Angle(maniPoseLocal(s_other, s_bone[kHead]).rotation,
                        Turn((maniVec3){0.0f, 0.0f, 1.0f}, 0.0f));
    maniAimSolve s = one;
    s.count = 3;
    for (int b = 0; b < 3; ++b)
    {
        s.bones[b] = s_bone[kSpine + b];
        s.shares[b] = 1.0f / 3.0f;
    }
    maniQuat spine = maniPoseLocal(s_pose, s_bone[kSpine]).rotation;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "shared");
    float third = Angle(maniPoseLocal(s_pose, s_bone[kSpine]).rotation, spine);
    CHECK(fabsf(third - whole / 3.0f) < 1e-4f && whole > 0.5f, "the first bone's share");
    // Shares 0.5, 0.5, 0: the neck takes all that is left, the head only
    // the little its place below the neck needs.
    Posed(s_pose);
    s.shares[0] = 0.5f;
    s.shares[1] = 0.5f;
    s.shares[2] = 0.0f;
    maniQuat head = maniPoseLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success &&
              Off(s_pose, goal, false) < 1e-3f,
          "aimed by spine and neck");
    CHECK(Angle(maniPoseLocal(s_pose, s_bone[kHead]).rotation, head) < 0.1f * whole,
          "the head left with little");
}

static void TestCone(void)
{
    // A goal behind: the forward stops 0.5 from the input's.
    Posed(s_pose);
    maniVec3 before = Rotate(Model(s_pose, kHead).rotation, (maniVec3){0.0f, 0.0f, 1.0f});
    maniAimSolve s = Head((maniVec3){0.2f, 1.5f, -2.0f}, (maniVec3){0.0f, 1.0f, 0.0f});
    s.cone = 0.5f;
    bool r = true;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "coned");
    maniVec3 after = Rotate(Model(s_pose, kHead).rotation, (maniVec3){0.0f, 0.0f, 1.0f});
    maniVec3 toward = Sub((maniVec3){0.2f, 1.5f, -2.0f}, Model(s_pose, kHead).translation);
    float was = acosf(Dot(before, toward) / Length(toward));
    float now = acosf(Dot(after, toward) / Length(toward));
    CHECK(fabsf(acosf(Dot(before, after)) - 0.5f) < 1e-3f && fabsf(was - now - 0.5f) < 1e-3f,
          "the forward at the cone's edge, toward the goal");
    // Within the cone: aimed.
    Posed(s_pose);
    s.goal = (maniVec3){-0.4f, 1.3f, 2.0f};
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
              Off(s_pose, s.goal, false) < 1e-3f,
          "within the cone aimed");
}

static void TestPoleSweep(void)
{
    // The pole swept through the aim line: the up axis turns in small
    // steps.
    maniVec3 goal = {0.0f, 3.0f, 0.3f};
    maniVec3 last = {0.0f, 0.0f, 0.0f};
    float worst = 0.0f;
    for (int i = 0; i <= 400; ++i)
    {
        float x = -1.0f + (float)i / 200.0f;
        Posed(s_pose);
        maniAimSolve s = Head(goal, (maniVec3){x * 0.1f, 1.0f, 0.1f});
        CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "swept");
        maniVec3 up = Rotate(Model(s_pose, kHead).rotation, (maniVec3){0.0f, 1.0f, 0.0f});
        if (i > 0)
        {
            float step = Length(Sub(up, last));
            worst = step > worst ? step : worst;
        }
        last = up;
    }
    CHECK(worst < 0.05f, "the up axis continuous through the aim line");
}

static void TestEdges(void)
{
    maniVec3 goal = {1.0f, 2.0f, 1.0f};
    Posed(s_pose);
    Posed(s_other);
    maniAimSolve s = Head(goal, (maniVec3){0.0f, 1.0f, 0.0f});
    s.weight = 0.0f;
    bool r = true;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "weight 0");
    maniQuat a = maniPoseLocal(s_pose, s_bone[kHead]).rotation;
    maniQuat b = maniPoseLocal(s_other, s_bone[kHead]).rotation;
    CHECK(fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) < 1e-6f, "nothing at weight 0");
    s.weight = 1.0f;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "once");
    a = maniPoseLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "twice");
    b = maniPoseLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) + fabsf(a.w - b.w) < 1e-5f,
          "an aim on its own result changes nothing");
    // A goal at the head, or within the eyes' offset from the line: no
    // turn, not reached.
    Posed(s_pose);
    maniVec3 at = Model(s_pose, kHead).translation;
    s = Head(at, (maniVec3){0.0f, 1.0f, 0.0f});
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "at the head");
    b = maniPoseLocal(s_other, s_bone[kHead]).rotation;
    a = maniPoseLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) < 1e-6f, "the head unturned");
    s.goal = (maniVec3){at.x + 0.02f, at.y, at.z};
    s.offset = (maniVec3){0.0f, 0.05f, 0.1f};
    CHECK(maniSolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r,
          "a goal nearer than the eyes' line not reached");
}

static void TestRefusals(void)
{
    Posed(s_pose);
    maniAimSolve good = Head((maniVec3){1.0f, 2.0f, 1.0f}, (maniVec3){0.0f, 1.0f, 0.0f});
    maniAimSolve bad[12];
    for (int k = 0; k < 12; ++k)
    {
        bad[k] = good;
    }
    bad[0].count = 0;
    bad[1].count = MANI_AIM_MAX_BONES + 1;
    bad[2].bones[0] = 99;
    bad[3].count = 2;
    bad[3].bones[1] = s_bone[kSpine];
    bad[4].shares[0] = -1.0f;
    bad[5].shares[0] = 0.0f;
    bad[6].forward = (maniVec3){0.0f, 0.0f, 1.1f};
    bad[7].goal.x = NAN;
    bad[8].up.y = INFINITY;
    bad[9].cone = 3.2f;
    bad[10].weight = 1.5f;
    bad[11].twist = NAN;
    bool refused = maniSolveAim(NULL, &s_skeleton, &good, NULL) == mani_errorInvalid &&
                   maniSolveAim(s_pose, NULL, &good, NULL) == mani_errorInvalid &&
                   maniSolveAim(s_pose, &s_skeleton, NULL, NULL) == mani_errorInvalid;
    for (int k = 0; k < 12; ++k)
    {
        refused = refused && maniSolveAim(s_pose, &s_skeleton, &bad[k], NULL) == mani_errorInvalid;
    }
    CHECK(refused, "the refusals");
}

int main(void)
{
    Setup();
    TestAim();
    TestChain();
    TestGap();
    TestShares();
    TestCone();
    TestPoleSweep();
    TestEdges();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
