// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring states (mani-0026) on a chain of two joints along +x: the
// steps against a binary64 reference of VRMC_springBone's update, a
// chain at rest kept, gravity hanging it at its bones' lengths, the same
// motion at host rates of 30, 60 and 144 Hz, the character's motion in
// the world swaying it, a center bone carrying it, a collider and a cone
// limit kept, the step bound, reset, save and restore with every check
// of the saved bytes, and the calls' refusals.

#include "bytes.h"
#include "spring_file.h"
#include "spring_shape.h"
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

static maniBoneSource Bone(const char* name, int32_t parent, float x, float y, float z)
{
    return (maniBoneSource){name,
                            (uint32_t)strlen(name),
                            parent,
                            {{x, y, z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
}

static bool OpenBones(const maniBoneSource* bones, uint32_t count, uint8_t* file, size_t capacity,
                      maniSkeleton* skeleton)
{
    maniSkeletonSource source = {bones, count, 0};
    size_t size = 0;
    return maniBuildSkeleton(s_builder, &source).result == mani_success &&
           maniCopyBuilt(s_builder, file, capacity, &size) == mani_success &&
           maniOpenSkeleton(file, size, skeleton).result == mani_success;
}

static void Setup(void)
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
    bool collider;
} Config;

static const Config kSwing = {0.5f, 1.0f, 0.2f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, false};

static const float kHit = 0.05f;
static const maniVec3 kBall = {0.35f, 0.65f, 0.0f};
static const float kBallRadius = 0.15f;

static void MakeSet(const Config* c)
{
    maniSpringJointSource joints[3];
    for (int k = 0; k < 3; ++k)
    {
        joints[k] = (maniSpringJointSource){.bone = s_b[k],
                                            .hitRadius = kHit,
                                            .stiffness = c->stiffness,
                                            .gravityPower = c->gravity,
                                            .gravityDirection = {0.0f, -1.0f, 0.0f},
                                            .drag = c->drag,
                                            .limit = c->limit,
                                            .limitAngle = c->angle,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    maniColliderSource ball = {s_root, maniColliderSphere, kBall, {0.0f, 0.0f, 0.0f}, kBallRadius,
                               0};
    uint32_t members[1] = {0};
    maniColliderGroupSource group = {"ball", 4, 1, members};
    uint32_t groups[1] = {0};
    maniSpringSource spring = {"chain", 5, 3, joints, groups, c->collider ? 1u : 0u, c->center};
    maniSpringSetSource source = {&s_skeleton, &ball, &group, &spring, 1, 1, 1, 0.0f, 0.0f, 0};
    size_t size = 0;
    CHECK(maniBuildSpringSet(s_builder, &source).result == mani_success &&
              maniCopyBuilt(s_builder, s_setFile, sizeof s_setFile, &size) == mani_success &&
              maniOpenSpringSet(s_setFile, size, &s_set).result == mani_success,
          "a set");
}

static maniSpringState* NewState(int k)
{
    maniSpringState* state = NULL;
    CHECK(maniSpringStateSize(&s_set) <= sizeof s_memory[k] &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[k], sizeof s_memory[k], &state) ==
                  mani_success,
          "a state");
    return state;
}

static maniVec3 Position(uint32_t bone)
{
    maniAffine m = maniPoseModel(s_pose, bone);
    return (maniVec3){m.m[9], m.m[10], m.m[11]};
}

static float Distance(maniVec3 a, maniVec3 b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return sqrtf(x * x + y * y + z * z);
}

// One host frame: the pose at rest with its root moved by root, the
// state stepped at a placement and applied, the model composed.
static bool Frame(maniSpringState* state, float delta, const maniTransform* placement,
                  maniVec3 root)
{
    bool ok = maniPoseSetRest(s_pose, &s_skeleton) == mani_success;
    maniTransform t = maniPoseLocal(s_pose, s_root);
    t.translation = root;
    maniPoseSetLocal(s_pose, s_root, t);
    ok = ok && maniStepSprings(state, s_pose, placement, delta) == mani_success;
    ok = ok && maniApplySprings(state, s_pose) == mani_success;
    return ok && maniComputeModel(s_pose, &s_skeleton) == mani_success;
}

static const maniVec3 kOrigin = {0.0f, 0.0f, 0.0f};

static bool Frames(maniSpringState* state, float delta, uint32_t count)
{
    bool ok = true;
    for (uint32_t i = 0; i < count; ++i)
    {
        ok = Frame(state, delta, NULL, kOrigin) && ok;
    }
    return ok;
}

// ---- A binary64 reference of the update for a still chain.

typedef struct V
{
    double x, y, z;
} V;

static V Add(V a, V b)
{
    return (V){a.x + b.x, a.y + b.y, a.z + b.z};
}

static V Sub(V a, V b)
{
    return (V){a.x - b.x, a.y - b.y, a.z - b.z};
}

static V Scale(V a, double s)
{
    return (V){a.x * s, a.y * s, a.z * s};
}

static double Dot(V a, V b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static V Cross(V a, V b)
{
    return (V){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static V Unit(V a)
{
    return Scale(a, 1.0 / sqrt(Dot(a, a)));
}

// A rotation as an axis's sine-scaled vector and cosine of half the
// angle.
typedef struct Q
{
    V v;
    double w;
} Q;

static V Rotate(Q q, V a)
{
    V t = Scale(Cross(q.v, a), 2.0);
    return Add(Add(a, Scale(t, q.w)), Cross(q.v, t));
}

static Q Between(V a, V b)
{
    Q q = {Cross(a, b), 1.0 + Dot(a, b)};
    double n = sqrt(Dot(q.v, q.v) + q.w * q.w);
    return (Q){Scale(q.v, 1.0 / n), q.w / n};
}

static Q Mul(Q a, Q b)
{
    return (Q){Add(Add(Scale(b.v, a.w), Scale(a.v, b.w)), Cross(a.v, b.v)),
               a.w * b.w - Dot(a.v, b.v)};
}

// A joint's binary32 value, widened.
static double Field(uint32_t joint, uint32_t at)
{
    uint32_t bits = maniLoadU32(s_set.joints + (size_t)joint * MANI_SPRING_JOINT_SIZE + at);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return (double)v;
}

// The tails after steps steps from rest: per joint, next = now + (now -
// before)(1 - drag) + the turned rest direction * stiffness + gravity,
// at the bone's length from the turned head.
static void Reference(uint32_t steps, V tails[2])
{
    const V rest[3] = {{0.0, 1.0, 0.0}, {0.5, 1.0, 0.0}, {1.0, 1.0, 0.0}};
    V now[2] = {rest[1], rest[2]};
    V before[2] = {rest[1], rest[2]};
    for (uint32_t s = 0; s < steps; ++s)
    {
        Q turn = {{0.0, 0.0, 0.0}, 1.0};
        V moved = {0.0, 0.0, 0.0};
        for (uint32_t j = 0; j < 2; ++j)
        {
            V head = Add(Rotate(turn, rest[j]), moved);
            V pull = Unit(Rotate(turn, Sub(rest[j + 1], rest[j])));
            V gravity = {Field(j, maniSpringJointGravity), Field(j, maniSpringJointGravity + 4),
                         Field(j, maniSpringJointGravity + 8)};
            V next =
                Add(now[j], Scale(Sub(now[j], before[j]), 1.0 - Field(j, maniSpringJointDrag)));
            next = Add(Add(next, Scale(pull, Field(j, maniSpringJointStiffness))), gravity);
            V direction = Unit(Sub(next, head));
            next = Add(head, Scale(direction, 0.5));
            Q q = Between(pull, direction);
            moved = Add(Rotate(q, Sub(moved, head)), head);
            turn = Mul(q, turn);
            before[j] = now[j];
            now[j] = next;
        }
    }
    tails[0] = now[0];
    tails[1] = now[1];
}

// The tail a state holds now, from its saved form (no center, model
// space at an identity placement).
static V SavedTail(const maniSpringState* state, uint32_t joint)
{
    CHECK(maniSaveSpringState(state, s_saved[0], sizeof s_saved[0]) == mani_success, "saved");
    const uint8_t* p = s_saved[0] + 24 + 4 * (18 + 17 * (size_t)joint);
    float v[3];
    for (int k = 0; k < 3; ++k)
    {
        uint32_t bits = maniLoadU32(p + 4 * k);
        memcpy(&v[k], &bits, sizeof v[k]);
    }
    return (V){(double)v[0], (double)v[1], (double)v[2]};
}

static void TestReference(void)
{
    MakeSet(&kSwing);
    maniSpringState* state = NewState(0);
    // A host step of a float sixtieth runs one step a call.
    for (uint32_t frame = 1; frame <= 240; ++frame)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL, kOrigin), "a frame");
        if (frame % 60 == 0)
        {
            V want[2];
            Reference(frame, want);
            for (uint32_t j = 0; j < 2; ++j)
            {
                V got = SavedTail(state, j);
                CHECK(sqrt(Dot(Sub(got, want[j]), Sub(got, want[j]))) < 1e-4,
                      "the reference's tail");
            }
        }
    }
}

static void TestRest(void)
{
    Config c = {1.0f, 0.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, false};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / 60.0f, 120), "frames");
    maniVec3 b1 = {0.5f, 1.0f, 0.0f};
    maniVec3 b2 = {1.0f, 1.0f, 0.0f};
    CHECK(Distance(Position(s_b[1]), b1) < 1e-5f && Distance(Position(s_b[2]), b2) < 1e-5f,
          "a chain at rest stays");
}

static void TestGravity(void)
{
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, false};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    bool lengths = true;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL, kOrigin), "a frame");
        lengths = lengths && fabsf(Distance(Position(s_b[0]), Position(s_b[1])) - 0.5f) < 1e-4f &&
                  fabsf(Distance(Position(s_b[1]), Position(s_b[2])) - 0.5f) < 1e-4f;
    }
    CHECK(lengths, "the bones keep their lengths");
    maniVec3 b1 = {0.0f, 0.5f, 0.0f};
    maniVec3 b2 = {0.0f, 0.0f, 0.0f};
    CHECK(Distance(Position(s_b[1]), b1) < 1e-2f && Distance(Position(s_b[2]), b2) < 2e-2f,
          "gravity hangs the chain");
}

// The chain's last tail after a second at a host rate.
static maniVec3 AfterSecond(uint32_t rate)
{
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / (float)rate, rate), "frames");
    return Position(s_b[2]);
}

// Likewise with the character swaying in the world, a fifth of a unit
// along x at a hertz.
static maniVec3 AfterSway(uint32_t rate)
{
    maniSpringState* state = NewState(0);
    maniTransform place = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    for (uint32_t i = 1; i <= rate; ++i)
    {
        place.translation.x = 0.2f * sinf(6.2831853f * (float)i / (float)rate);
        CHECK(Frame(state, 1.0f / (float)rate, &place, kOrigin), "a frame");
    }
    return Position(s_b[2]);
}

static void TestHostRates(void)
{
    MakeSet(&kSwing);
    maniVec3 at60 = AfterSecond(60);
    CHECK(Distance(AfterSecond(30), at60) < 1e-4f && Distance(AfterSecond(144), at60) < 1e-4f,
          "the same motion at every host rate");
    at60 = AfterSway(60);
    // The steps see the inputs interpolated between host frames: 0.5 mm
    // off at 30 Hz, 0.05 mm at 144 Hz.
    CHECK(Distance(AfterSway(30), at60) < 2e-3f && Distance(AfterSway(144), at60) < 5e-4f,
          "and with the character moving");
    // A host faster than the step rate shows the tails between steps.
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / 240.0f, 10), "frames");
    maniVec3 a = Position(s_b[2]);
    CHECK(Frames(state, 1.0f / 240.0f, 1), "a frame");
    maniVec3 b = Position(s_b[2]);
    CHECK(Distance(a, b) > 0.0f, "a frame between steps moves");
}

static void TestPlacement(void)
{
    Config c = {1.0f, 0.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, false};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    maniTransform place = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    for (int i = 0; i < 30; ++i)
    {
        place.translation.y += 0.02f;
        CHECK(Frame(state, 1.0f / 60.0f, &place, kOrigin), "a frame");
    }
    CHECK(Position(s_b[2]).y < 0.99f, "moving up in the world sways the tail down");
    for (int i = 0; i < 600; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, &place, kOrigin), "a frame");
    }
    maniVec3 b2 = {1.0f, 1.0f, 0.0f};
    CHECK(Distance(Position(s_b[2]), b2) < 1e-3f, "and stiffness brings it back");
}

// The last tail's height above the root after the root rises in the pose.
static float AfterRise(uint32_t center)
{
    Config c = {1.0f, 0.0f, 0.4f, maniSpringLimitNone, 0.0f, center, false};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    maniVec3 root = kOrigin;
    for (int i = 0; i < 30; ++i)
    {
        root.y += 0.02f;
        CHECK(Frame(state, 1.0f / 60.0f, NULL, root), "a frame");
    }
    return Position(s_b[2]).y - root.y;
}

static void TestCenter(void)
{
    CHECK(AfterRise(MANI_NO_BONE) < 0.99f, "a rising root sways a chain without a center");
    CHECK(fabsf(AfterRise(s_root) - 1.0f) < 1e-4f, "and carries one in its space");
}

static void TestCollider(void)
{
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, true};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    float nearest = 1.0f;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL, kOrigin), "a frame");
        float d = Distance(Position(s_b[1]), kBall);
        nearest = d < nearest ? d : nearest;
    }
    // The bone's length, kept after the push as VRMC_springBone keeps it,
    // lets a tail pressed on the ball back in by a little (1.7 mm here).
    CHECK(nearest > kBallRadius + kHit - 5e-3f, "the tail stays out of the ball");
    CHECK(nearest < kBallRadius + kHit + 2e-2f, "resting on it");
}

static void TestLimit(void)
{
    // A cone about the bone's rest direction (+x).
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitCone, 0.3f, MANI_NO_BONE, false};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    float widest = 0.0f;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL, kOrigin), "a frame");
        maniVec3 b0 = Position(s_b[0]);
        maniVec3 b1 = Position(s_b[1]);
        float x = (b1.x - b0.x) / Distance(b1, b0);
        float angle = acosf(x < 1.0f ? x : 1.0f);
        widest = angle > widest ? angle : widest;
    }
    CHECK(widest < 0.3f + 2e-3f && widest > 0.29f, "the cone holds gravity at its edge");
}

static void TestStepBound(void)
{
    MakeSet(&kSwing);
    maniSpringState* a = NewState(0);
    maniSpringState* b = NewState(1);
    CHECK(Frame(a, 10.0f, NULL, kOrigin) && Frame(b, 20.0f / 60.0f, NULL, kOrigin), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveSpringState(b, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], maniSpringSavedSize(&s_set)) == 0,
          "a long delta plays as the bound's steps");
    // Either sign of delta moves on alike.
    a = NewState(0);
    b = NewState(1);
    CHECK(Frame(a, -0.05f, NULL, kOrigin) && Frame(b, 0.05f, NULL, kOrigin), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveSpringState(b, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], maniSpringSavedSize(&s_set)) == 0,
          "a negative delta moves on by its magnitude");
}

static void TestReset(void)
{
    MakeSet(&kSwing);
    maniSpringState* a = NewState(0);
    maniSpringState* b = NewState(1);
    CHECK(Frames(a, 1.0f / 60.0f, 50), "frames");
    maniResetSprings(a);
    maniResetSprings(NULL);
    CHECK(Frames(a, 1.0f / 60.0f, 1) && Frames(b, 1.0f / 60.0f, 1), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveSpringState(b, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], maniSpringSavedSize(&s_set)) == 0,
          "a reset state starts over");
}

// Restoring the saved bytes with one word changed fails at want, and
// leaves the state as it was.
static bool RefusedAt(maniSpringState* state, uint64_t at, uint32_t value, uint64_t want)
{
    size_t size = maniSpringSavedSize(&s_set);
    static uint8_t s_before[4096];
    CHECK(maniSaveSpringState(state, s_before, sizeof s_before) == mani_success, "saved");
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + at, value);
    maniFileResult r = maniRestoreSpringState(state, s_saved[1], size);
    uint8_t after[4096];
    CHECK(maniSaveSpringState(state, after, sizeof after) == mani_success, "saved");
    return r.result == mani_errorFormat && r.offset == want && memcmp(after, s_before, size) == 0;
}

static uint32_t Bits(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

static void TestSaveRestore(void)
{
    Config c = {0.5f, 1.0f, 0.2f, maniSpringLimitNone, 0.0f, s_root, false};
    MakeSet(&c);
    maniSpringState* a = NewState(0);
    maniSpringState* b = NewState(1);
    size_t size = maniSpringSavedSize(&s_set);
    CHECK(size == 24 + 4 * (18 + 2 * 17 + 10 * 2) && maniSpringSavedSize(NULL) == 0,
          "the saved size");
    CHECK(Frames(a, 1.0f / 50.0f, 30), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], size - 1) == mani_errorCapacity &&
              maniSaveSpringState(a, s_saved[0], size) == mani_success &&
              maniRestoreSpringState(b, s_saved[0], size).result == mani_success,
          "saved and restored");
    CHECK(Frames(a, 1.0f / 50.0f, 30) && Frames(b, 1.0f / 50.0f, 30), "frames");
    uint8_t other[4096];
    CHECK(maniSaveSpringState(a, s_saved[0], size) == mani_success &&
              maniSaveSpringState(b, other, size) == mani_success &&
              memcmp(s_saved[0], other, size) == 0,
          "a restored state moves as the saved one");
    // The header, then the block's words (byte 24 + 4 * word).
    uint64_t joint1 = 24 + 4 * (18 + 17);
    uint64_t center = 24 + 4 * (18 + 2 * 17 + 10);
    CHECK(RefusedAt(b, 0, 0, 0) && RefusedAt(b, 4, 1, 4) && RefusedAt(b, 8, 1, 8) &&
              RefusedAt(b, 16, (uint32_t)size + 4, 16) && RefusedAt(b, 24, 2, 24) &&
              RefusedAt(b, 28, 1, 28) && RefusedAt(b, 36, 0x7FF00000u, 32) &&
              RefusedAt(b, 36, 0xBFF00000u, 32) && RefusedAt(b, 36, 0x3FA00000u, 32),
          "the header, motion and clock checked");
    CHECK(RefusedAt(b, 24 + 4 * 7, Bits(2.0f), 24 + 4 * 7) &&
              RefusedAt(b, 24 + 4 * 11, Bits(NAN), 24 + 4 * 11) &&
              RefusedAt(b, joint1, Bits(INFINITY), joint1) &&
              RefusedAt(b, joint1 + 4 * 9, Bits(0.5f), joint1 + 4 * 9) &&
              RefusedAt(b, joint1 + 4 * 13, Bits(-1.0f), joint1 + 4 * 13) &&
              RefusedAt(b, joint1 + 4 * 16, Bits(NAN), joint1 + 4 * 16) &&
              RefusedAt(b, center + 4 * 9, Bits(NAN), center + 4 * 9),
          "every value checked");
    // Without motion every word is 0.
    maniResetSprings(b);
    CHECK(maniSaveSpringState(b, s_saved[0], size) == mani_success &&
              RefusedAt(b, 24 + 4 * 30, 1, 24 + 4 * 30),
          "a state without motion holds only zeros");
    CHECK(maniRestoreSpringState(b, s_saved[0], 23).offset == 0 &&
              maniRestoreSpringState(b, s_saved[0], size - 1).offset == 16 &&
              maniRestoreSpringState(NULL, s_saved[0], size).result == mani_errorInvalid &&
              maniRestoreSpringState(b, NULL, size).result == mani_errorInvalid &&
              maniSaveSpringState(NULL, s_saved[0], size) == mani_errorInvalid &&
              maniSaveSpringState(b, NULL, size) == mani_errorInvalid,
          "the sizes and arguments checked");
    // A spring without a center keeps zeros in its center's words.
    MakeSet(&kSwing);
    a = NewState(0);
    CHECK(Frames(a, 1.0f / 60.0f, 2), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], size) == mani_success &&
              RefusedAt(a, center + 4 * 3, Bits(1.0f), center + 4 * 3),
          "no center, no transform");
}

static void TestRefusals(void)
{
    MakeSet(&kSwing);
    maniSpringState* state = NULL;
    size_t size = maniSpringStateSize(&s_set);
    maniSpringSet other = s_set;
    other.skeletonIdentity ^= 1u;
    CHECK(size > 0 && maniSpringStateSize(NULL) == 0, "the state's size");
    CHECK(maniInitSpringState(NULL, &s_skeleton, s_memory[0], size, &state) == mani_errorInvalid &&
              maniInitSpringState(&s_set, NULL, s_memory[0], size, &state) == mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, NULL, size, &state) == mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[0], size, NULL) ==
                  mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[0] + 4, size, &state) ==
                  mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_lone, s_memory[0], size, &state) ==
                  mani_errorInvalid &&
              maniInitSpringState(&other, &s_skeleton, s_memory[0], size, &state) ==
                  mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[0], size - 1, &state) ==
                  mani_errorCapacity &&
              state == NULL,
          "making a state checked");
    state = NewState(0);
    maniTransform bad = {{NAN, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    maniTransform flat = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
    CHECK(maniStepSprings(NULL, s_pose, NULL, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, NULL, NULL, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, NULL, NAN) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, NULL, INFINITY) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, &bad, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, &flat, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, s_lonePose, NULL, 0.1f) == mani_errorInvalid &&
              maniApplySprings(NULL, s_pose) == mani_errorInvalid &&
              maniApplySprings(state, NULL) == mani_errorInvalid &&
              maniApplySprings(state, s_lonePose) == mani_errorInvalid,
          "the calls' arguments checked");
    // Applying before any step leaves the pose; a stale model is composed.
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    maniTransform local = maniPoseLocal(s_pose, s_b[1]);
    maniPoseSetLocal(s_pose, s_root, maniPoseLocal(s_pose, s_root));
    CHECK(maniApplySprings(state, s_pose) == mani_success, "applied");
    maniTransform after = maniPoseLocal(s_pose, s_b[1]);
    CHECK(memcmp(&local, &after, sizeof local) == 0 && maniPoseModelCurrent(s_pose),
          "no motion, no change");
}

static bool Near3(maniVec3 a, maniVec3 b)
{
    return Distance(a, b) < 1e-6f;
}

// A collision's direction and distance, or no collision.
static bool Hits(maniSpringShape shape, maniVec3 tail, maniVec3 direction, float distance)
{
    maniVec3 d = {0.0f, 0.0f, 0.0f};
    float at = 0.0f;
    return maniSpringCollide(&shape, tail, 0.1f, &d, &at) && Near3(d, direction) &&
           fabsf(at - distance) < 1e-6f;
}

static bool Misses(maniSpringShape shape, maniVec3 tail)
{
    maniVec3 d = {0.0f, 0.0f, 0.0f};
    float at = 0.0f;
    return !maniSpringCollide(&shape, tail, 0.1f, &d, &at);
}

static void TestShapes(void)
{
    const maniVec3 o = {0.0f, 0.0f, 0.0f};
    maniSpringShape ball = {o, o, 1.0f, maniColliderSphere, 0};
    CHECK(Misses(ball, (maniVec3){1.5f, 0.0f, 0.0f}) &&
              Hits(ball, (maniVec3){0.5f, 0.0f, 0.0f}, (maniVec3){1.0f, 0.0f, 0.0f}, -0.6f) &&
              Hits(ball, o, (maniVec3){0.0f, 1.0f, 0.0f}, -1.1f),
          "a sphere pushes out, up from its center");
    ball.inside = 1;
    CHECK(Misses(ball, (maniVec3){0.5f, 0.0f, 0.0f}) &&
              Hits(ball, (maniVec3){0.0f, 0.0f, 0.95f}, (maniVec3){0.0f, 0.0f, -1.0f}, -0.05f),
          "an inside sphere pulls in");
    maniSpringShape capsule = {o, {2.0f, 0.0f, 0.0f}, 0.5f, maniColliderCapsule, 0};
    CHECK(Misses(capsule, (maniVec3){1.0f, 0.7f, 0.0f}) &&
              Hits(capsule, (maniVec3){1.0f, 0.3f, 0.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.3f) &&
              Hits(capsule, (maniVec3){-0.3f, 0.0f, 0.0f}, (maniVec3){-1.0f, 0.0f, 0.0f}, -0.3f) &&
              Hits(capsule, (maniVec3){2.0f, 0.0f, 0.4f}, (maniVec3){0.0f, 0.0f, 1.0f}, -0.2f) &&
              Hits(capsule, (maniVec3){2.3f, 0.0f, 0.0f}, (maniVec3){1.0f, 0.0f, 0.0f}, -0.3f),
          "a capsule pushes out from its segment");
    capsule.inside = 1;
    CHECK(Misses(capsule, (maniVec3){1.0f, 0.3f, 0.0f}) &&
              Hits(capsule, (maniVec3){1.0f, -0.45f, 0.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.05f),
          "an inside capsule pulls in");
    maniSpringShape point = {o, o, 0.5f, maniColliderCapsule, 0};
    CHECK(Hits(point, (maniVec3){0.0f, 0.0f, 0.3f}, (maniVec3){0.0f, 0.0f, 1.0f}, -0.3f),
          "a capsule of no length is a sphere");
    maniSpringShape plane = {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.0f, maniColliderPlane, 0};
    CHECK(Misses(plane, (maniVec3){0.0f, 1.2f, 0.0f}) &&
              Hits(plane, (maniVec3){3.0f, 1.05f, 2.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.05f) &&
              Hits(plane, (maniVec3){0.0f, 0.5f, 0.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.6f),
          "a plane pushes along its normal");
}

static bool Limited(maniSpringLimit limit, float angle, float yaw, maniVec3 in, maniVec3 out)
{
    maniVec3 got = maniSpringLimitDirection(limit, angle, yaw, in);
    return Distance(got, out) < 1e-5f;
}

static void TestLimitDirections(void)
{
    const maniVec3 x = {1.0f, 0.0f, 0.0f};
    const maniVec3 y = {0.0f, 1.0f, 0.0f};
    const maniVec3 z = {0.0f, 0.0f, 1.0f};
    const maniVec3 down = {0.0f, -1.0f, 0.0f};
    float c = cosf(0.5f);
    float s = sinf(0.5f);
    CHECK(Limited(maniSpringLimitNone, 0.5f, 0.0f, down, down) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, y, y) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, x, (maniVec3){s, c, 0.0f}) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, down, (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, (maniVec3){0.0f, 0.95f, 0.31224990f},
                      (maniVec3){0.0f, 0.95f, 0.31224990f}),
          "a cone");
    CHECK(Limited(maniSpringLimitHinge, 0.5f, 0.0f, x, y) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.6f, 0.8f, 0.0f}, y) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.0f, 0.0f, -1.0f},
                      (maniVec3){0.0f, c, -s}) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.6f, 0.0f, 0.8f},
                      (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.0f, 0.95f, -0.31224990f},
                      (maniVec3){0.0f, 0.95f, -0.31224990f}),
          "a hinge in the yz plane");
    float cy = cosf(0.3f);
    CHECK(Limited(maniSpringLimitSpherical, 0.5f, 0.3f, y, y) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, z, (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, (maniVec3){0.0f, 0.0f, -1.0f},
                      (maniVec3){0.0f, c, -s}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, x, (maniVec3){sinf(0.3f), cy, 0.0f}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, (maniVec3){-1.0f, 0.0f, 0.0f},
                      (maniVec3){-sinf(0.3f), cy, 0.0f}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, down, (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitSpherical, 3.0f, 1.5f, (maniVec3){0.6f, 0.8f, 0.0f},
                      (maniVec3){0.6f, 0.8f, 0.0f}),
          "a spherical limit's pitch and yaw");
}

int main(void)
{
    Setup();
    TestShapes();
    TestLimitDirections();
    TestReference();
    TestRest();
    TestGravity();
    TestHostRates();
    TestPlacement();
    TestCenter();
    TestCollider();
    TestLimit();
    TestStepBound();
    TestReset();
    TestSaveRestore();
    TestRefusals();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
