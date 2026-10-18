// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body XPBD solve (mani-0031) on a figure of hips, spine,
// chest, two arms of three bones and a leg of three: both hands at
// reachable goals reached with every length kept; a solve on its own
// reached result unchanged bit for bit; weight 0 and strength 0
// leaving the pose; a locked forearm keeping its local rotation; a cone
// on a forearm never exceeded; a hand taking a wanted rotation; a goal
// out of reach: stretch, a free top and pre-pull each bringing the hand
// closer (pre-pull moving the top unturned); a pulled chain starting
// closer; preferred angles choosing a straight leg's bend; a stiff spine
// turning less; goals at the hips and nearly every bone locked without
// a NaN; the scratch size and its refusals; the description's refusals.

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

static float Angle(maniQuat a, maniQuat b)
{
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

static maniVec3 At(maniPose* pose, uint32_t bone)
{
    CHECK(maniComputeModel(pose, &s_skeleton) == mani_success, "the model");
    return maniPoseModelTransform(pose, bone).translation;
}

static void Setup(void)
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
static void Bent(maniPose* pose)
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
static maniBodySolve Solve(maniVec3 left, maniVec3 right)
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

static bool Solved(maniPose* pose, const maniBodySolve* s, bool* reached)
{
    size_t size = maniBodyIkScratchSize(&s_skeleton, s);
    return size > 0 && size <= sizeof s_scratch &&
           maniSolveBodyXpbd(pose, &s_skeleton, s, s_scratch, size, reached) == mani_success;
}

static bool LengthsKept(maniPose* pose, maniPose* input)
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

static bool Same(const maniPose* a, const maniPose* b)
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
    bool r = true;
    CHECK(Solved(s_pose, &s, &r) && !r && Same(s_pose, s_other),
          "strength 0 pulls nothing, unreached");
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

// The left hand alone toward goal, from the hips.
static maniBodySolve Left(maniVec3 goal)
{
    maniBodySolve s = Solve(goal, goal);
    s.effectorCount = 1;
    return s;
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

static bool Finite(const maniPose* pose)
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
    TestEdges();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
