// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Aim and look-at on a spine, neck and head, the head's
// forward its z and its up its y, the eyes 0.1 ahead and 0.05 up: a head
// aimed at goals all around points at them, its up toward the pole; a
// twist turns the up about the aim line; the eyes' line passes through
// the goal; spread over the three bones by
// shares it ends aimed with every bone turned, shares of 1, 0, 0 too; a
// cone stops the forward at its edge; a pole swept through the aim line
// turns the head continuously; weight 0 changes nothing and a second aim
// nothing more; a goal at the head turns nothing; the same turns at any
// power-of-two scale, a short up and pole as unit ones; the eyes'
// offset, forward and up carried by the head's whole frame, scale and
// mirror included; the refusals.

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
    CHECK(maniPose_ComputeModel(pose, &s_skeleton) == mani_success, "the model");
    return maniPose_GetModelTransform(pose, s_bone[bone]);
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
    maniSkeletonSource source = {bones, kBones};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success &&
              maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "the skeleton");
    maniDestroyBuilder(builder);
    const char* names[kBones] = {"root", "spine", "neck", "head"};
    for (int b = 0; b < kBones; ++b)
    {
        s_bone[b] = maniSkeleton_FindBone(&s_skeleton, names[b], (uint32_t)strlen(names[b]));
    }
    CHECK(maniInitPose(&s_skeleton, s_memory[0], sizeof s_memory[0], &s_pose) == mani_success &&
              maniInitPose(&s_skeleton, s_memory[1], sizeof s_memory[1], &s_other) == mani_success,
          "the poses");
}

// The pose at rest with the spine bent forward and the neck turned.
static void Posed(maniPose* pose)
{
    CHECK(maniPose_SetRest(pose, &s_skeleton) == mani_success, "rest");
    maniTransform t = maniPose_GetLocal(pose, s_bone[kSpine]);
    t.rotation = Turn((maniVec3){1.0f, 0.0f, 0.0f}, 0.2f);
    maniPose_SetLocal(pose, s_bone[kSpine], t);
    t = maniPose_GetLocal(pose, s_bone[kNeck]);
    t.rotation = Turn((maniVec3){0.0f, 1.0f, 0.0f}, -0.3f);
    maniPose_SetLocal(pose, s_bone[kNeck], t);
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
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success, "aimed");
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
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &plain, NULL) == mani_success &&
              maniPose_SolveAim(s_other, &s_skeleton, &twisted, NULL) == mani_success,
          "twisted and not");
    maniVec3 u0 = Rotate(Model(s_pose, kHead).rotation, (maniVec3){0.0f, 1.0f, 0.0f});
    maniVec3 u1 = Rotate(Model(s_other, kHead).rotation, (maniVec3){0.0f, 1.0f, 0.0f});
    CHECK(fabsf(acosf(Dot(u0, u1)) - 0.5f) < 1e-3f && Off(s_other, goals[1], false) < 1e-3f,
          "the twist about the aim line");
    // The eyes' line through the goal.
    Posed(s_pose);
    maniAimSolve s = Head(goals[1], (maniVec3){0.0f, 1.0f, 0.0f});
    s.offset = (maniVec3){0.0f, 0.05f, 0.1f};
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success &&
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
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
                  Off(s_pose, goal, false) < 1e-3f,
              "the chain ends aimed");
        bool turned = true;
        for (int b = kSpine; b <= kHead; ++b)
        {
            maniQuat a = maniPose_GetLocal(s_pose, s_bone[b]).rotation;
            maniQuat q = maniPose_GetLocal(s_other, s_bone[b]).rotation;
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
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
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
    CHECK(maniPose_SolveAim(s_other, &s_skeleton, &one, NULL) == mani_success, "the head alone");
    float whole = Angle(maniPose_GetLocal(s_other, s_bone[kHead]).rotation,
                        Turn((maniVec3){0.0f, 0.0f, 1.0f}, 0.0f));
    maniAimSolve s = one;
    s.count = 3;
    for (int b = 0; b < 3; ++b)
    {
        s.bones[b] = s_bone[kSpine + b];
        s.shares[b] = 1.0f / 3.0f;
    }
    maniQuat spine = maniPose_GetLocal(s_pose, s_bone[kSpine]).rotation;
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "shared");
    float third = Angle(maniPose_GetLocal(s_pose, s_bone[kSpine]).rotation, spine);
    CHECK(fabsf(third - whole / 3.0f) < 1e-4f && whole > 0.5f, "the first bone's share");
    // Shares 0.5, 0.5, 0: the neck takes all that is left, the head only
    // the little its place below the neck needs.
    Posed(s_pose);
    s.shares[0] = 0.5f;
    s.shares[1] = 0.5f;
    s.shares[2] = 0.0f;
    maniQuat head = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success &&
              Off(s_pose, goal, false) < 1e-3f,
          "aimed by spine and neck");
    CHECK(Angle(maniPose_GetLocal(s_pose, s_bone[kHead]).rotation, head) < 0.1f * whole,
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
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "coned");
    maniVec3 after = Rotate(Model(s_pose, kHead).rotation, (maniVec3){0.0f, 0.0f, 1.0f});
    maniVec3 toward = Sub((maniVec3){0.2f, 1.5f, -2.0f}, Model(s_pose, kHead).translation);
    float was = acosf(Dot(before, toward) / Length(toward));
    float now = acosf(Dot(after, toward) / Length(toward));
    CHECK(fabsf(acosf(Dot(before, after)) - 0.5f) < 1e-3f && fabsf(was - now - 0.5f) < 1e-3f,
          "the forward at the cone's edge, toward the goal");
    // Within the cone: aimed.
    Posed(s_pose);
    s.goal = (maniVec3){-0.4f, 1.3f, 2.0f};
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r &&
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
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "swept");
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
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "weight 0");
    maniQuat a = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
    maniQuat b = maniPose_GetLocal(s_other, s_bone[kHead]).rotation;
    CHECK(fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) < 1e-6f, "nothing at weight 0");
    s.weight = 1.0f;
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "once");
    a = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, NULL) == mani_success, "twice");
    b = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) + fabsf(a.w - b.w) < 1e-5f,
          "an aim on its own result changes nothing");
    // A goal at the head, or within the eyes' offset from the line: no
    // turn, not reached.
    Posed(s_pose);
    maniVec3 at = Model(s_pose, kHead).translation;
    s = Head(at, (maniVec3){0.0f, 1.0f, 0.0f});
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r, "at the head");
    b = maniPose_GetLocal(s_other, s_bone[kHead]).rotation;
    a = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
    CHECK(fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z) < 1e-6f, "the head unturned");
    s.goal = (maniVec3){at.x + 0.02f, at.y, at.z};
    s.offset = (maniVec3){0.0f, 0.05f, 0.1f};
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && !r,
          "a goal nearer than the eyes' line not reached");
}

// An aim scaled by a power of two takes the same turns bit for bit:
// the spine under a root scale from 2^-100 to 2^100 against 2^1, the
// goal scaled with it and the eyes' offset carried by the head's frame,
// without and with a cone.
// A binary32 squared distance to the goal overflowed from 2^64, aiming
// the head 2.5 radians off while reporting it reached, and read the goal
// as at the head below about 2^-75. An up and a pole 2^-80 long turn the
// head as unit ones.
static void TestFarScales(void)
{
    const int exponents[7] = {1, -100, -80, -60, 60, 80, 100};
    bool same = true;
    bool reached = true;
    for (int cone = 0; cone < 2; ++cone)
    {
        maniQuat first = {0};
        bool firstReached = false;
        for (int e = 0; e < 7; ++e)
        {
            float k = ldexpf(1.0f, exponents[e]);
            Posed(s_pose);
            maniTransform root = maniPose_GetLocal(s_pose, s_bone[kRoot]);
            root.scale = (maniVec3){k, k, k};
            maniPose_SetLocal(s_pose, s_bone[kRoot], root);
            maniAimSolve s =
                Head((maniVec3){0.7f * k, 1.9f * k, 0.9f * k}, (maniVec3){0.0f, 1.0f, 0.0f});
            s.offset = (maniVec3){0.0f, 0.05f, 0.1f};
            s.cone = cone ? 0.3f : 0.0f;
            bool r = false;
            CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success, "aimed");
            maniQuat q = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
            if (e == 0)
            {
                first = q;
                firstReached = r;
            }
            reached = reached && r == firstReached && (cone == 1 || r);
            same = same && memcmp(&first, &q, sizeof q) == 0;
        }
    }
    CHECK(reached, "reached at every scale without the cone, as at 2^1 with it");
    CHECK(same, "the same turns at every scale");
    Posed(s_pose);
    Posed(s_other);
    maniAimSolve unit = Head((maniVec3){0.7f, 1.9f, 0.9f}, (maniVec3){0.3f, 1.0f, 0.0f});
    maniAimSolve shorter = unit;
    shorter.up = (maniVec3){0.0f, 0x1p-80f, 0.0f};
    shorter.pole = (maniVec3){0.3f * 0x1p-80f, 0x1p-80f, 0.0f};
    CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &unit, NULL) == mani_success &&
              maniPose_SolveAim(s_other, &s_skeleton, &shorter, NULL) == mani_success,
          "with either up and pole");
    maniQuat heads[2] = {maniPose_GetLocal(s_pose, s_bone[kHead]).rotation,
                         maniPose_GetLocal(s_other, s_bone[kHead]).rotation};
    CHECK(memcmp(&heads[0], &heads[1], sizeof heads[0]) == 0, "a short up and pole as unit ones");
}

// The eyes' offset is a point of the head's frame, carried by its whole
// model frame: under a uniform root scale the turns are the unscaled
// spine's bit for bit, and under a head scaled (1, 2, 0.5) or a root
// mirrored through x, the line from the eyes' point in the model along
// the forward passes through the goal.
static void TestOffsetInFrame(void)
{
    const maniVec3 eyes = {0.03f, 0.05f, 0.1f};
    const int exponents[5] = {1, -20, -2, 3, 20};
    maniQuat first = {0};
    bool same = true;
    for (int e = 0; e < 5; ++e)
    {
        float k = ldexpf(1.0f, exponents[e]);
        Posed(s_pose);
        maniTransform root = maniPose_GetLocal(s_pose, s_bone[kRoot]);
        root.scale = (maniVec3){k, k, k};
        maniPose_SetLocal(s_pose, s_bone[kRoot], root);
        maniAimSolve s =
            Head((maniVec3){0.7f * k, 1.9f * k, 0.9f * k}, (maniVec3){0.0f, 1.0f, 0.0f});
        s.offset = eyes;
        bool r = false;
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r, "aimed");
        maniQuat q = maniPose_GetLocal(s_pose, s_bone[kHead]).rotation;
        if (e == 0)
        {
            first = q;
        }
        same = same && memcmp(&first, &q, sizeof q) == 0;
    }
    CHECK(same, "the same turns under every uniform scale");
    const maniVec3 scales[2][2] = {{{1.0f, 1.0f, 1.0f}, {1.0f, 2.0f, 0.5f}},
                                   {{-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    bool through = true;
    for (int c = 0; c < 2; ++c)
    {
        Posed(s_pose);
        maniTransform root = maniPose_GetLocal(s_pose, s_bone[kRoot]);
        root.scale = scales[c][0];
        maniPose_SetLocal(s_pose, s_bone[kRoot], root);
        maniTransform head = maniPose_GetLocal(s_pose, s_bone[kHead]);
        head.scale = scales[c][1];
        maniPose_SetLocal(s_pose, s_bone[kHead], head);
        maniVec3 goal = {0.7f, 1.9f, 0.9f};
        maniAimSolve s = Head(goal, (maniVec3){0.0f, 1.0f, 0.0f});
        s.offset = eyes;
        bool r = false;
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r, "aimed");
        CHECK(maniPose_ComputeModel(s_pose, &s_skeleton) == mani_success, "the model");
        maniAffine m = maniPose_GetModel(s_pose, s_bone[kHead]);
        // The eyes' point, the forward's direction and the goal's, in
        // binary64.
        double p[3];
        double f[3];
        double t[3];
        const double g[3] = {(double)goal.x, (double)goal.y, (double)goal.z};
        double cross = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            p[i] = (double)m.m[9 + i] + (double)m.m[i] * (double)eyes.x +
                   (double)m.m[3 + i] * (double)eyes.y + (double)m.m[6 + i] * (double)eyes.z;
            f[i] = (double)m.m[6 + i];
            t[i] = g[i] - p[i];
        }
        for (int i = 0; i < 3; ++i)
        {
            double c3 = f[(i + 1) % 3] * t[(i + 2) % 3] - f[(i + 2) % 3] * t[(i + 1) % 3];
            cross += c3 * c3;
        }
        double ft = f[0] * t[0] + f[1] * t[1] + f[2] * t[2];
        through = through && ft > 0.0 &&
                  sqrt(cross) <= 1e-5 * sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]) *
                                     sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    }
    CHECK(through, "the eyes' line through the goal under a scaled head and a mirror");
}

// The cosine between a column of the head's model matrix and a model
// direction, in binary64, each made perpendicular to unit line u when u
// is not NULL.
static double Along(const maniAffine* m, int column, const double* d, const double* u)
{
    double c[3];
    double e[3];
    for (int i = 0; i < 3; ++i)
    {
        c[i] = (double)m->m[3 * column + i];
        e[i] = d[i];
    }
    if (u != NULL)
    {
        double cu = c[0] * u[0] + c[1] * u[1] + c[2] * u[2];
        double eu = e[0] * u[0] + e[1] * u[1] + e[2] * u[2];
        for (int i = 0; i < 3; ++i)
        {
            c[i] -= cu * u[i];
            e[i] -= eu * u[i];
        }
    }
    double ce = c[0] * e[0] + c[1] * e[1] + c[2] * e[2];
    return ce / (sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) *
                 sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]));
}

// The forward and up are carried by the head's whole model frame: under
// a root mirrored through x a forward of +x, and under a head scaled
// (1, 2, 0.5) a forward off its axes, end with the head's real forward
// (its model matrix times the forward) on the goal; under the mirror an
// up of +x ends with the head's real +x toward the pole.
static void TestDirectionsInFrame(void)
{
    const maniVec3 goal = {0.7f, 1.9f, 0.9f};
    const maniVec3 pole = {0.3f, 1.0f, 0.0f};
    const struct
    {
        maniVec3 root;
        maniVec3 head;
        maniVec3 forward;
        maniVec3 up;
    } cases[3] = {
        {{-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{1.0f, 1.0f, 1.0f}, {1.0f, 2.0f, 0.5f}, {0.6f, 0.0f, 0.8f}, {0.0f, 1.0f, 0.0f}},
        {{-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}},
    };
    bool forward = true;
    bool up = true;
    for (int c = 0; c < 3; ++c)
    {
        Posed(s_pose);
        maniTransform root = maniPose_GetLocal(s_pose, s_bone[kRoot]);
        root.scale = cases[c].root;
        maniPose_SetLocal(s_pose, s_bone[kRoot], root);
        maniTransform head = maniPose_GetLocal(s_pose, s_bone[kHead]);
        head.scale = cases[c].head;
        maniPose_SetLocal(s_pose, s_bone[kHead], head);
        maniAimSolve s = Head(goal, pole);
        s.forward = cases[c].forward;
        s.up = cases[c].up;
        bool r = false;
        CHECK(maniPose_SolveAim(s_pose, &s_skeleton, &s, &r) == mani_success && r, "aimed");
        CHECK(maniPose_ComputeModel(s_pose, &s_skeleton) == mani_success, "the model");
        maniAffine m = maniPose_GetModel(s_pose, s_bone[kHead]);
        // The real forward: the matrix's columns weighted by the forward.
        maniAffine f = m;
        for (int i = 0; i < 3; ++i)
        {
            f.m[i] = m.m[i] * s.forward.x + m.m[3 + i] * s.forward.y + m.m[6 + i] * s.forward.z;
        }
        double t[3] = {(double)goal.x - (double)m.m[9], (double)goal.y - (double)m.m[10],
                       (double)goal.z - (double)m.m[11]};
        forward = forward && Along(&f, 0, t, NULL) > 1.0 - 1e-9;
        if (c == 2)
        {
            double l = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
            double u[3] = {t[0] / l, t[1] / l, t[2] / l};
            double p[3] = {(double)pole.x, (double)pole.y, (double)pole.z};
            up = Along(&m, 0, p, u) > 1.0 - 1e-6;
        }
    }
    CHECK(forward, "the real forward on the goal under a mirror and a non-uniform scale");
    CHECK(up, "the real up toward the pole under a mirror");
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
    bad[1].count = MANI_MAX_AIM_BONES + 1;
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
    bool refused = maniPose_SolveAim(NULL, &s_skeleton, &good, NULL) == mani_errorInvalid &&
                   maniPose_SolveAim(s_pose, NULL, &good, NULL) == mani_errorInvalid &&
                   maniPose_SolveAim(s_pose, &s_skeleton, NULL, NULL) == mani_errorInvalid;
    for (int k = 0; k < 12; ++k)
    {
        maniResult want = k == 1 ? mani_errorLimit : mani_errorInvalid;
        refused = refused && maniPose_SolveAim(s_pose, &s_skeleton, &bad[k], NULL) == want;
    }
    CHECK(refused, "the refusals, a count past MANI_MAX_AIM_BONES at the limit");
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
    TestFarScales();
    TestOffsetInFrame();
    TestDirectionsInFrame();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
