// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spring state suites' fixture (mani-0026): a root, b0 a unit above
// it and a chain b1, b2 half a unit apart along +x; a set of one spring
// over b0, b1, b2 made from a Config (stiffness, gravity, drag, limit,
// center, a ball or a floor on b0, gravity's direction, the yaw limit,
// the step rate); states in two buffers; host frames over a pose whose
// root and b0's scale a test sets; a saved state's words. Included by
// one suite each.

#ifndef MAUL_ANIM_TEST_SPRING_FIXTURE_H
#define MAUL_ANIM_TEST_SPRING_FIXTURE_H

#include "bytes.h"
#include "spring_file.h"
#include "test_harness.h"

#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_loneFile[1024];
static _Alignas(16) uint8_t s_setFile[8192];
static _Alignas(32) uint8_t s_poseMemory[4096];
static _Alignas(32) uint8_t s_lonePoseMemory[1024];
static _Alignas(8) uint8_t s_memory[2][4096];
static uint8_t s_saved[2][4096];
static maniSkeleton s_skeleton;
static maniSkeleton s_lone;
static maniBuilder* s_builder;
static maniSpringSet s_set;
static maniPose* s_pose;
static maniPose* s_lonePose;
static uint32_t s_root;
static uint32_t s_b[3];

static inline maniBoneSource Bone(const char* name, int32_t parent, float x, float y, float z)
{
    return (maniBoneSource){name,
                            (uint32_t)strlen(name),
                            parent,
                            {{x, y, z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
}

static inline bool OpenBones(const maniBoneSource* bones, uint32_t count, uint8_t* file,
                             size_t capacity, maniSkeleton* skeleton)
{
    maniSkeletonSource source = {bones, count, 0};
    size_t size = 0;
    return maniBuildSkeleton(s_builder, &source).result == mani_success &&
           maniCopyBuilt(s_builder, file, capacity, &size) == mani_success &&
           maniOpenSkeleton(file, size, skeleton).result == mani_success;
}

static inline void Setup(void)
{
    // The chain: b0 a unit above the root, b1 and b2 half a unit apart
    // along +x.
    const maniBoneSource bones[4] = {
        Bone("root", -1, 0.0f, 0.0f, 0.0f), Bone("b0", 0, 0.0f, 1.0f, 0.0f),
        Bone("b1", 1, 0.5f, 0.0f, 0.0f), Bone("b2", 2, 0.5f, 0.0f, 0.0f)};
    const maniBoneSource lone[1] = {Bone("lone", -1, 0.0f, 0.0f, 0.0f)};
    maniBuilderDef def = maniDefaultBuilderDef();
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success &&
              OpenBones(bones, 4, s_skeletonFile, sizeof s_skeletonFile, &s_skeleton) &&
              OpenBones(lone, 1, s_loneFile, sizeof s_loneFile, &s_lone),
          "the skeletons");
    const char* names[4] = {"root", "b0", "b1", "b2"};
    s_root = maniSkeletonFindBone(&s_skeleton, names[0], 4);
    for (int k = 0; k < 3; ++k)
    {
        s_b[k] = maniSkeletonFindBone(&s_skeleton, names[k + 1], 2);
    }
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) == mani_success &&
              maniInitPose(&s_lone, s_lonePoseMemory, sizeof s_lonePoseMemory, &s_lonePose) ==
                  mani_success,
          "the poses");
}

// What a test's set holds.
typedef struct Config
{
    float stiffness;
    float gravity;
    float drag;
    maniSpringLimit limit;
    float angle;
    uint32_t center;
    // 0 for none, 1 the ball, 2 the floor, 3 the shell (a sphere about
    // b0 that keeps tails in).
    uint32_t collider;
    // Down when zero.
    maniVec3 gravityDirection;
    // A spherical limit's yaw; the step rate, 0 for the default.
    float yaw;
    float rate;
} Config;

static const Config kSwing = {
    0.5f, 1.0f, 0.2f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f};

static const float kHit = 0.05f;
// On b0: where the ball is with the root at the origin, the floor's
// height there and the shell's radius.
static const maniVec3 kBall = {0.35f, 0.65f, 0.0f};
static const float kBallRadius = 0.15f;
static const float kFloor = 0.7f;
static const float kShell = 0.6f;
// Far spheres a set may list before the ball in its group, past the
// colliders an apply makes once for a spring's joints.
#define kCrowd 17u
static bool s_crowd = false;

static inline void MakeSet(const Config* c)
{
    maniVec3 down = c->gravityDirection;
    if (down.x == 0.0f && down.y == 0.0f && down.z == 0.0f)
    {
        down.y = -1.0f;
    }
    maniSpringJointSource joints[3];
    for (int k = 0; k < 3; ++k)
    {
        joints[k] = (maniSpringJointSource){.bone = s_b[k],
                                            .hitRadius = kHit,
                                            .stiffness = c->stiffness,
                                            .gravityPower = c->gravity,
                                            .gravityDirection = down,
                                            .drag = c->drag,
                                            .limit = c->limit,
                                            .limitAngle = c->angle,
                                            .limitYaw = c->yaw,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    maniColliderSource colliders[3 + kCrowd] = {
        {s_b[0], maniColliderSphere, {kBall.x, kBall.y - 1.0f, 0.0f}, {0, 0, 0}, kBallRadius, 0},
        {s_b[0], maniColliderPlane, {0.0f, kFloor - 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.0f, 0},
        {s_b[0], maniColliderSphere, {0.0f, 0.0f, 0.0f}, {0, 0, 0}, kShell, 1}};
    // The ball's group: the ball alone, or with s_crowd the far spheres
    // before it.
    uint32_t members[3 + kCrowd + 1] = {0, 1, 2};
    for (uint32_t k = 0; k < kCrowd; ++k)
    {
        colliders[3 + k] = (maniColliderSource){
            s_b[0], maniColliderSphere, {0.0f, -10.0f, (float)k}, {0, 0, 0}, 0.01f, 0};
        members[3 + k] = 3 + k;
    }
    members[3 + kCrowd] = 0;
    const maniColliderGroupSource groups[3] = {
        {"ball", 4, s_crowd ? kCrowd + 1 : 1, s_crowd ? &members[3] : &members[0]},
        {"floor", 5, 1, &members[1]},
        {"shell", 5, 1, &members[2]}};
    uint32_t chosen[1] = {c->collider != 0 ? c->collider - 1 : 0};
    maniSpringSource spring = {"chain",  5, 3, joints, chosen, c->collider != 0 ? 1u : 0u,
                               c->center};
    maniSpringSetSource source = {&s_skeleton, colliders, groups, &spring, s_crowd ? 3 + kCrowd : 3,
                                  3,           1,         0.0f,   c->rate, 0};
    size_t size = 0;
    CHECK(maniBuildSpringSet(s_builder, &source).result == mani_success &&
              maniCopyBuilt(s_builder, s_setFile, sizeof s_setFile, &size) == mani_success &&
              maniOpenSpringSet(s_setFile, size, &s_set).result == mani_success,
          "a set");
}

static inline maniSpringState* NewState(int k)
{
    maniSpringState* state = NULL;
    CHECK(maniSpringStateSize(&s_set) <= sizeof s_memory[k] &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[k], sizeof s_memory[k], &state) ==
                  mani_success,
          "a state");
    return state;
}

static inline maniVec3 Position(uint32_t bone)
{
    maniAffine m = maniPoseModel(s_pose, bone);
    return (maniVec3){m.m[9], m.m[10], m.m[11]};
}

static inline float Distance(maniVec3 a, maniVec3 b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return sqrtf(x * x + y * y + z * z);
}

// The pose's root, b0's scale (a mirror when negative) and b1's
// rotation.
static maniTransform s_rootLocal = {
    {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
static maniVec3 s_b0Scale = {1.0f, 1.0f, 1.0f};
static maniQuat s_b1Rotation = {0.0f, 0.0f, 0.0f, 1.0f};
// The external force each frame takes when s_forced.
static maniVec3 s_force = {0.0f, 0.0f, 0.0f};
static bool s_forced = false;

static inline void Still(void)
{
    s_rootLocal = (maniTransform){{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    s_b0Scale = (maniVec3){1.0f, 1.0f, 1.0f};
    s_b1Rotation = (maniQuat){0.0f, 0.0f, 0.0f, 1.0f};
    s_forced = false;
}

// One host frame: the pose at rest with its root at s_rootLocal, b0's
// scale s_b0Scale and b1's rotation s_b1Rotation, the state stepped at
// a placement and applied, the model composed.
static inline bool Frame(maniSpringState* state, float delta, const maniTransform* placement)
{
    bool ok = maniPoseSetRest(s_pose, &s_skeleton) == mani_success;
    maniPoseSetLocal(s_pose, s_root, s_rootLocal);
    maniTransform b0 = maniPoseLocal(s_pose, s_b[0]);
    b0.scale = s_b0Scale;
    maniPoseSetLocal(s_pose, s_b[0], b0);
    maniTransform b1 = maniPoseLocal(s_pose, s_b[1]);
    b1.rotation = s_b1Rotation;
    maniPoseSetLocal(s_pose, s_b[1], b1);
    ok = ok && maniStepSprings(state, s_pose, placement, s_forced ? &s_force : NULL, delta) ==
                   mani_success;
    ok = ok && maniApplySprings(state, s_pose) == mani_success;
    return ok && maniComputeModel(s_pose, &s_skeleton) == mani_success;
}

static inline bool Frames(maniSpringState* state, float delta, uint32_t count)
{
    bool ok = true;
    for (uint32_t i = 0; i < count; ++i)
    {
        ok = Frame(state, delta, NULL) && ok;
    }
    return ok;
}

// A saved state's word (no header) as binary32.
static inline float SavedFloat(const maniSpringState* state, uint64_t word)
{
    CHECK(maniSaveSpringState(state, s_saved[0], sizeof s_saved[0]) == mani_success, "saved");
    uint32_t bits = maniLoadU32(s_saved[0] + 24 + 4 * word);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

#endif // MAUL_ANIM_TEST_SPRING_FIXTURE_H
