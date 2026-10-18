// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body SDLS solve (mani-0031) on the figure (body_figure.h):
// both hands at reachable goals reached with every length kept and the
// pinned top held; a solve on its own reached result unchanged bit for
// bit; weight 0 and strength 0 leaving the pose; a locked forearm
// keeping its local bits, its hand reached; a cone on a forearm held; a
// hand taking a wanted rotation; a free top bringing an out-of-reach
// hand closer; stretch refused; goals at the top and nearly every bone
// locked without a NaN; a free top taking its wanted rotation; two
// solves' bits pinned, one from the straight rest with a free top. Then the arm alone, a
// chain of three turning bones: one iteration's joints within 1e-5 of a
// binary64 SDLS written here from the paper; on two bones bent about z
// by hinges, two effectors pulling apart end where a brute-force search
// finds the least squared misses' minimum, within 1e-4.

#include "body_figure.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PINNED_SDLS_HASH 0x21099bba1e67ca8bull
#define PINNED_FREE_HASH 0xb7607b5dc8c659b2ull

static bool Sdls(maniPose* pose, const maniBodySolve* s, bool* reached)
{
    size_t size = maniBodyIkScratchSize(&s_skeleton, s);
    return size > 0 && size <= sizeof s_scratch &&
           maniSolveBodySdls(pose, &s_skeleton, s, s_scratch, size, reached) == mani_success;
}

static void TestReach(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    bool r = false;
    CHECK(Sdls(s_pose, &s, &r) && r, "both hands reached");
    CHECK(Distance(At(s_pose, kHandL), s_effectors[0].position) <= 1e-3f &&
              Distance(At(s_pose, kHandR), s_effectors[1].position) <= 1e-3f,
          "within the tolerance where the pose puts them");
    CHECK(LengthsKept(s_pose, s_other), "every length kept");
    maniTransform hips[2] = {maniPoseLocal(s_pose, kHips), maniPoseLocal(s_other, kHips)};
    CHECK(memcmp(&hips[0], &hips[1], sizeof hips[0]) == 0, "the pinned top held");
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniPoseSetLocal(s_other, b, maniPoseLocal(s_pose, b));
    }
    CHECK(Sdls(s_other, &s, &r) && r && Same(s_pose, s_other),
          "a solve on its own reached result unchanged");
}

static void TestWeights(void)
{
    Bent(s_pose);
    Bent(s_other);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s.weight = 0.0f;
    CHECK(Sdls(s_pose, &s, NULL) && Same(s_pose, s_other), "weight 0 leaves the pose");
    s.weight = 1.0f;
    s_effectors[0].strength = 0.0f;
    s_effectors[1].strength = 0.0f;
    bool r = true;
    CHECK(Sdls(s_pose, &s, &r) && !r && Same(s_pose, s_other), "strength 0 pulls nothing");
}

static void TestLockLimitRotation(void)
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
    CHECK(Sdls(s_pose, &s, NULL), "a lock and a limit");
    maniQuat locked[2] = {maniPoseLocal(s_pose, kForeL).rotation,
                          maniPoseLocal(s_other, kForeL).rotation};
    CHECK(memcmp(&locked[0], &locked[1], sizeof locked[0]) == 0 &&
              Distance(At(s_pose, kHandL), s_effectors[0].position) <= 1e-3f,
          "the locked forearm kept, its hand reached");
    maniQuat t = maniPoseLocal(s_pose, kForeR).rotation;
    t = t.w < 0.0f ? (maniQuat){-t.x, -t.y, -t.z, -t.w} : t;
    float twist = -2.0f * atan2f(t.x, t.w);
    maniQuat q = Turn((maniVec3){-1.0f, 0.0f, 0.0f}, -twist);
    float sw = t.w * q.w - t.x * q.x - t.y * q.y - t.z * q.z;
    CHECK(2.0f * acosf(fminf(1.0f, fabsf(sw))) <= 0.4f + 1e-4f && fabsf(twist) <= 0.2f + 1e-4f,
          "the limited forearm within its cone and twist");
    Bent(s_pose);
    s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s_effectors[0].rotation = Turn((maniVec3){0.0f, 0.0f, 1.0f}, 1.0f);
    s_effectors[0].rotationWeight = 1.0f;
    CHECK(Sdls(s_pose, &s, NULL) && maniComputeModel(s_pose, &s_skeleton) == mani_success &&
              Angle(maniPoseModelTransform(s_pose, kHandL).rotation, s_effectors[0].rotation) <
                  1e-2f &&
              Distance(At(s_pose, kHandL), s_effectors[0].position) <= 1e-3f,
          "the hand takes the wanted rotation at its goal");
}

static void TestTopAndEdges(void)
{
    maniVec3 far = {1.8f, 1.5f, 0.2f};
    Bent(s_pose);
    maniBodySolve s = Left(far);
    CHECK(Sdls(s_pose, &s, NULL), "pinned, out of reach");
    float pinned = Distance(At(s_pose, kHandL), far);
    Bent(s_pose);
    Bent(s_other);
    s.topMode = maniBodyTopFree;
    CHECK(Sdls(s_pose, &s, NULL) && Distance(At(s_pose, kHandL), far) < pinned - 1e-2f &&
              Distance(At(s_pose, kHips), At(s_other, kHips)) > 1e-2f,
          "a free top moves, the hand closer");
    s.stretch = true;
    CHECK(maniSolveBodySdls(s_pose, &s_skeleton, &s, s_scratch, sizeof s_scratch, NULL) ==
              mani_errorInvalid,
          "stretch refused");
    Bent(s_pose);
    maniVec3 hips = At(s_pose, kHips);
    s = Solve(hips, hips);
    CHECK(Sdls(s_pose, &s, NULL) && Finite(s_pose), "goals at the top: no NaN");
    Bent(s_pose);
    s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    maniBodyBone locked[8];
    const uint32_t part[8] = {kSpine, kChest, kArmL, kForeL, kHandL, kArmR, kForeR, kHandR};
    for (int k = 0; k < 8; ++k)
    {
        locked[k] = (maniBodyBone){.bone = part[k], .rotationStiffness = 1.0f};
    }
    s.bones = locked;
    s.boneCount = 8;
    Bent(s_other);
    CHECK(Sdls(s_pose, &s, NULL) && Same(s_pose, s_other), "every bone locked: the pose kept");
}

// A free top as the effector, its rotation wanted: its rotation rows
// scaled by the mean bone length, as its reach is none.
static void TestTopRotation(void)
{
    Bent(s_pose);
    maniBodyEffector top = {.bone = kHips,
                            .position = At(s_pose, kHips),
                            .rotation = Turn((maniVec3){0.0f, 1.0f, 0.0f}, 0.6f),
                            .positionWeight = 1.0f,
                            .rotationWeight = 1.0f,
                            .strength = 1.0f};
    maniBodySolve s = {.top = kHips,
                       .topMode = maniBodyTopFree,
                       .effectors = &top,
                       .effectorCount = 1,
                       .iterations = 64,
                       .tolerance = 1e-4f,
                       .weight = 1.0f};
    CHECK(Sdls(s_pose, &s, NULL) && maniComputeModel(s_pose, &s_skeleton) == mani_success &&
              Angle(maniPoseModelTransform(s_pose, kHips).rotation, top.rotation) < 1e-2f,
          "a free top turned to its wanted rotation");
}

// A hash of the pose's locals.
static uint64_t Hash(const maniPose* pose)
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t b = 0; b < BONES; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        hash = maniHash64(hash, &t, (int32_t)sizeof t);
    }
    return hash;
}

static void TestPinned(void)
{
    Bent(s_pose);
    maniBodySolve s = Solve((maniVec3){0.6f, 1.2f, 0.3f}, (maniVec3){-0.5f, 1.6f, 0.2f});
    s_effectors[0].strength = 0.5f;
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
    s.iterations = 8;
    s.tolerance = 0.0f;
    CHECK(Sdls(s_pose, &s, NULL), "the pinned solve");
    uint64_t hash = Hash(s_pose);
    // From the rest (straight arms, small singular values), a free top
    // and goals out of reach (the step's clamps acting on turns and
    // moves together), four iterations.
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    s = Solve((maniVec3){1.6f, 1.9f, 0.4f}, (maniVec3){-1.5f, 0.9f, -0.3f});
    s.topMode = maniBodyTopFree;
    s.iterations = 4;
    s.tolerance = 0.0f;
    CHECK(Sdls(s_pose, &s, NULL), "the pinned free solve");
    uint64_t free = Hash(s_pose);
    CHECK(hash == PINNED_SDLS_HASH && free == PINNED_FREE_HASH, "the same bits on every platform");
    if (hash != PINNED_SDLS_HASH || free != PINNED_FREE_HASH)
    {
        printf("pinned 0x%016llx 0x%016llx\n", (unsigned long long)hash, (unsigned long long)free);
    }
}

typedef struct Vec
{
    double x, y, z;
} Vec;

static Vec V(maniVec3 v)
{
    return (Vec){(double)v.x, (double)v.y, (double)v.z};
}

static Vec Sub3(Vec a, Vec b)
{
    return (Vec){a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec Cross3(Vec a, Vec b)
{
    return (Vec){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static double Length3(Vec a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// p turned by rotation vector r about center c (Rodrigues).
static Vec Spin3(Vec p, Vec c, Vec r)
{
    double angle = Length3(r);
    if (!(angle > 0.0))
    {
        return p;
    }
    Vec k = {r.x / angle, r.y / angle, r.z / angle};
    Vec v = Sub3(p, c);
    Vec kv = Cross3(k, v);
    double d = k.x * v.x + k.y * v.y + k.z * v.z;
    double cs = cos(angle);
    double sn = sin(angle);
    return (Vec){c.x + v.x * cs + kv.x * sn + k.x * d * (1.0 - cs),
                 c.y + v.y * cs + kv.y * sn + k.y * d * (1.0 - cs),
                 c.z + v.z * cs + kv.z * sn + k.z * d * (1.0 - cs)};
}

// One SDLS iteration in binary64 as the paper writes it, for one
// effector at joint 3 of joints 0..3 (joint 0 held), the miss clamped
// to most: the effector's new place.
static Vec ReferenceStep(Vec* joints, Vec goal, double most)
{
    Vec e = Sub3(goal, joints[3]);
    double l = Length3(e);
    e = l > most ? (Vec){e.x * most / l, e.y * most / l, e.z * most / l} : e;
    // J: 3 rows by 9 columns, three axes for joints 1..3.
    double j[3][9];
    for (int k = 0; k < 3; ++k)
    {
        Vec arm = Sub3(joints[3], joints[k + 1]);
        const Vec axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int a = 0; a < 3; ++a)
        {
            Vec c = Cross3(axes[a], arm);
            j[0][3 * k + a] = c.x;
            j[1][3 * k + a] = c.y;
            j[2][3 * k + a] = c.z;
        }
    }
    double rho[9];
    for (int c = 0; c < 9; ++c)
    {
        rho[c] = sqrt(j[0][c] * j[0][c] + j[1][c] * j[1][c] + j[2][c] * j[2][c]);
    }
    // One-sided Jacobi on J^T's three columns of 9, w accumulating.
    double a[3][9];
    double w[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    memcpy(a, j, sizeof a);
    for (int sweep = 0; sweep < 60; ++sweep)
    {
        for (int p = 0; p < 2; ++p)
        {
            for (int q = p + 1; q < 3; ++q)
            {
                double al = 0, be = 0, ga = 0;
                for (int k = 0; k < 9; ++k)
                {
                    al += a[p][k] * a[p][k];
                    be += a[q][k] * a[q][k];
                    ga += a[p][k] * a[q][k];
                }
                if (!(fabs(ga) > 1e-300))
                {
                    continue;
                }
                double zeta = (be - al) / (2.0 * ga);
                double t = (zeta >= 0.0 ? 1.0 : -1.0) / (fabs(zeta) + sqrt(1.0 + zeta * zeta));
                double cs = 1.0 / sqrt(1.0 + t * t);
                double sn = cs * t;
                for (int k = 0; k < 9; ++k)
                {
                    double x = a[p][k];
                    double y = a[q][k];
                    a[p][k] = cs * x - sn * y;
                    a[q][k] = sn * x + cs * y;
                }
                for (int k = 0; k < 3; ++k)
                {
                    double x = w[p][k];
                    double y = w[q][k];
                    w[p][k] = cs * x - sn * y;
                    w[q][k] = sn * x + cs * y;
                }
            }
        }
    }
    const double gammaMax = 0.785398163397448;
    double step[9] = {0};
    double sigmaMax = 0.0;
    double sigma[3];
    for (int i = 0; i < 3; ++i)
    {
        double sum = 0.0;
        for (int k = 0; k < 9; ++k)
        {
            sum += a[i][k] * a[i][k];
        }
        sigma[i] = sqrt(sum);
        sigmaMax = fmax(sigmaMax, sigma[i]);
    }
    for (int i = 0; i < 3; ++i)
    {
        if (!(sigma[i] > 1e-6 * sigmaMax))
        {
            continue;
        }
        double alpha = w[i][0] * e.x + w[i][1] * e.y + w[i][2] * e.z;
        double n = sqrt(w[i][0] * w[i][0] + w[i][1] * w[i][1] + w[i][2] * w[i][2]);
        double m = 0.0;
        double phi[9];
        double largest = 0.0;
        for (int k = 0; k < 9; ++k)
        {
            double v = a[i][k] / sigma[i];
            m += fabs(v) * rho[k];
            phi[k] = alpha / sigma[i] * v;
            largest = fmax(largest, fabs(phi[k]));
        }
        m /= sigma[i];
        double gamma = m > n ? gammaMax * n / m : gammaMax;
        double scale = largest > gamma ? gamma / largest : 1.0;
        for (int k = 0; k < 9; ++k)
        {
            step[k] += phi[k] * scale;
        }
    }
    double largest = 0.0;
    for (int k = 0; k < 9; ++k)
    {
        largest = fmax(largest, fabs(step[k]));
    }
    double scale = largest > gammaMax ? gammaMax / largest : 1.0;
    // From the tips up: each joint's turn moves the joints past it.
    for (int k = 2; k >= 0; --k)
    {
        Vec r = {step[3 * k] * scale, step[3 * k + 1] * scale, step[3 * k + 2] * scale};
        for (int after = k + 2; after <= 3; ++after)
        {
            joints[after] = Spin3(joints[after], joints[k + 1], r);
        }
    }
    return joints[3];
}

// The left arm from the chest, held: one iteration against binary64.
static void TestReference(void)
{
    Bent(s_pose);
    Vec joints[4] = {V(At(s_pose, kChest)), V(At(s_pose, kArmL)), V(At(s_pose, kForeL)),
                     V(At(s_pose, kHandL))};
    maniVec3 hand = At(s_pose, kHandL);
    maniVec3 goal = {hand.x + 0.3f, hand.y - 0.2f, hand.z + 0.1f};
    maniBodySolve s = Left(goal);
    s.top = kChest;
    s.iterations = 1;
    s.tolerance = 0.0f;
    // Half the mean of the offsets armL (0.25), foreL (0.3), handL (0.25).
    Vec want = ReferenceStep(joints, V(goal), 0.5 * (0.25 + 0.3 + 0.25) / 3.0);
    CHECK(Sdls(s_pose, &s, NULL), "one iteration");
    double off = Length3(Sub3(V(At(s_pose, kHandL)), want));
    CHECK(off < 1e-5, "the hand where a binary64 SDLS puts it");
    if (!(off < 1e-5))
    {
        printf("reference off by %g\n", off);
    }
}

// The squared misses of the elbow and the hand for the arm's two hinge
// angles in the plane, from the shoulder at s.
static double Misses(double t1, double t2, Vec s, Vec g1, Vec g2)
{
    Vec elbow = {s.x + 0.3 * cos(t1), s.y + 0.3 * sin(t1), 0.0};
    Vec hand = {elbow.x + 0.25 * cos(t1 + t2), elbow.y + 0.25 * sin(t1 + t2), 0.0};
    Vec a = Sub3(elbow, g1);
    Vec b = Sub3(hand, g2);
    return a.x * a.x + a.y * a.y + b.x * b.x + b.y * b.y;
}

// The arm at rest in the xy plane, hinges about z; the elbow pulled up
// and the hand down: the least squared misses, found by a search.
static void TestLeastSquares(void)
{
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    Vec shoulder = V(At(s_pose, kArmL));
    maniVec3 elbow = At(s_pose, kForeL);
    maniVec3 hand = At(s_pose, kHandL);
    maniVec3 goals[2] = {{elbow.x, elbow.y + 0.06f, 0.0f}, {hand.x, hand.y - 0.08f, 0.0f}};
    maniBodyEffector pulls[2];
    const uint32_t bones[2] = {kForeL, kHandL};
    for (int k = 0; k < 2; ++k)
    {
        pulls[k] = (maniBodyEffector){.bone = bones[k],
                                      .position = goals[k],
                                      .rotation = {0.0f, 0.0f, 0.0f, 1.0f},
                                      .positionWeight = 1.0f,
                                      .strength = 1.0f};
    }
    maniBodyBone hinges[2];
    const uint32_t hinged[2] = {kArmL, kForeL};
    for (int k = 0; k < 2; ++k)
    {
        hinges[k] = (maniBodyBone){.bone = hinged[k],
                                   .limit = {.kind = maniJointLimitHinge,
                                             .axis = {0.0f, 0.0f, 1.0f},
                                             .low = -3.1f,
                                             .high = 3.1f}};
    }
    maniBodySolve s = {.top = kChest,
                       .effectors = pulls,
                       .bones = hinges,
                       .effectorCount = 2,
                       .boneCount = 2,
                       .iterations = 256,
                       .weight = 1.0f};
    CHECK(Sdls(s_pose, &s, NULL), "two effectors pulling apart");
    Vec g1 = V(goals[0]);
    Vec g2 = V(goals[1]);
    Vec e = V(At(s_pose, kForeL));
    Vec h = V(At(s_pose, kHandL));
    Vec a = Sub3(e, g1);
    Vec b = Sub3(h, g2);
    double solved = a.x * a.x + a.y * a.y + a.z * a.z + b.x * b.x + b.y * b.y + b.z * b.z;
    // A coarse grid, then a finer one around its best, twice.
    double best = 1e30;
    double t1 = 0.0;
    double t2 = 0.0;
    double span = 1.0;
    for (int round = 0; round < 4; ++round)
    {
        double c1 = t1;
        double c2 = t2;
        for (int i = -200; i <= 200; ++i)
        {
            for (int k = -200; k <= 200; ++k)
            {
                double x = c1 + span * i / 200.0;
                double y = c2 + span * k / 200.0;
                double m = Misses(x, y, shoulder, g1, g2);
                if (m < best)
                {
                    best = m;
                    t1 = x;
                    t2 = y;
                }
            }
        }
        span /= 100.0;
    }
    CHECK(fabs(solved - best) < 1e-4 * best + 1e-10, "the least squared misses");
    if (!(fabs(solved - best) < 1e-4 * best + 1e-10))
    {
        printf("solved %.9g best %.9g\n", solved, best);
    }
}

int main(void)
{
    Setup();
    TestReach();
    TestWeights();
    TestLockLimitRotation();
    TestTopAndEdges();
    TestTopRotation();
    TestPinned();
    TestReference();
    TestLeastSquares();
    return s_failures == 0 ? 0 : 1;
}
