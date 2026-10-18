// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Getting up from a ragdoll (mani-0027) on the fixture's profile (the
// pelvis the root body, facing +z, up +y): upright (face up by the tie,
// looking where the chest faces), turned; lying face down (looking where
// the head lay) and face up (toward the feet), and face up turned a
// quarter and a half (opposite the model's forward); the
// body turned about its side from -130 to 130 degrees looking the same
// way throughout; inverted at 135 degrees and at 142, where the weighed
// sum vanishes; on its side and rolled either way; exactly opposite the
// model's forward; tilted and twisted, both terms apart; other
// model axes; a tilted ground; a pose from bodies under the placement
// placing the bodies back; the refusals.

#include "ragdoll_fixture.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/ragdoll.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_memory[2][8192];
static maniPose* s_poses[2];
static maniRagdoll s_ragdoll;
static maniRagdollBodyState s_states[5];

// A Y-up world, the ground at height 0.1, glTF's model axes.
static const maniRagdollGround kGround = {
    {0.0f, 0.1f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}};

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

static maniVec3 Rotate(maniQuat q, maniVec3 v)
{
    maniQuat p = Mul(Mul(q, (maniQuat){v.x, v.y, v.z, 0.0f}), (maniQuat){-q.x, -q.y, -q.z, q.w});
    return (maniVec3){p.x, p.y, p.z};
}

static bool Near(float a, double b, double tolerance)
{
    return fabs((double)a - b) <= tolerance;
}

static bool NearV(maniVec3 a, maniVec3 b, double tolerance)
{
    return Near(a.x, (double)b.x, tolerance) && Near(a.y, (double)b.y, tolerance) &&
           Near(a.z, (double)b.z, tolerance);
}

static const maniVec3 kX = {1.0f, 0.0f, 0.0f};
static const maniVec3 kY = {0.0f, 1.0f, 0.0f};
static const maniVec3 kZ = {0.0f, 0.0f, 1.0f};

// Every body at the root's place and turn, at rest.
static void Lie(maniVec3 at, maniQuat turn)
{
    for (int i = 0; i < 5; ++i)
    {
        s_states[i] = (maniRagdollBodyState){at, turn, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }
}

static maniRagdollGetUp GetUp(const maniRagdollGround* ground)
{
    maniRagdollGetUp up = {0};
    CHECK(maniRagdollGetUpPlacement(&s_ragdoll, s_states, 5, ground, &up) == mani_success,
          "a placement");
    return up;
}

// The placement stands on the ground under the root, its model up on
// the normal and its model forward on heading.
static bool Placed(maniRagdollGetUp up, maniVec3 at, maniVec3 heading)
{
    maniTransform p = up.placement;
    return NearV(p.translation, at, 1e-6) && NearV(Rotate(p.rotation, kZ), heading, 1e-5) &&
           NearV(Rotate(p.rotation, kY), kY, 1e-5) && p.scale.x == 1.0f && p.scale.y == 1.0f &&
           p.scale.z == 1.0f;
}

static void TestUpright(void)
{
    Lie((maniVec3){3.0f, 1.0f, 2.0f}, (maniQuat){0.0f, 0.0f, 0.0f, 1.0f});
    maniRagdollGetUp up = GetUp(&kGround);
    CHECK(up.faceUp && Placed(up, (maniVec3){3.0f, 0.1f, 2.0f}, kZ),
          "upright: face up by the tie, looking where the chest faces");
    Lie((maniVec3){3.0f, 1.0f, 2.0f}, Turn(kY, 1.5707964f));
    up = GetUp(&kGround);
    CHECK(Placed(up, (maniVec3){3.0f, 0.1f, 2.0f}, kX), "upright turned a quarter: along +x");
}

static void TestLying(void)
{
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Turn(kX, 1.5707964f));
    maniRagdollGetUp up = GetUp(&kGround);
    CHECK(!up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, kZ),
          "face down: looking where the head lay");
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Turn(kX, -1.5707964f));
    up = GetUp(&kGround);
    CHECK(up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, kZ),
          "face up: the head at -z, looking toward the feet");
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Mul(Turn(kY, 1.5707964f), Turn(kX, -1.5707964f)));
    up = GetUp(&kGround);
    CHECK(up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, kX),
          "face up turned a quarter: the head at -x, looking along +x");
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Mul(Turn(kY, 3.1415927f), Turn(kX, -1.5707964f)));
    up = GetUp(&kGround);
    CHECK(up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, (maniVec3){0.0f, 0.0f, -1.0f}),
          "face up, the head at +z: looking along -z, half a turn from the model's forward");
}

// Turned about its side from -130 to 130 degrees, the body looks along
// +z throughout, face up below 0; inverted at 135 degrees the two
// projections cancel and the head's is taken.
static void TestSweep(void)
{
    bool same = true;
    bool sides = true;
    for (int degrees = -130; degrees <= 130; degrees += 5)
    {
        Lie((maniVec3){0.0f, 0.3f, 0.0f}, Turn(kX, (float)degrees * 0.017453292f));
        maniRagdollGetUp up = GetUp(&kGround);
        same = same && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, kZ);
        sides = sides && up.faceUp == (degrees <= 0);
    }
    CHECK(same && sides, "one heading from -130 to 130 degrees, face up below 0");
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Turn(kX, 2.3561945f));
    maniRagdollGetUp up = GetUp(&kGround);
    CHECK(!up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, kZ),
          "inverted at 135 degrees: still the head's way");
    // About 142 degrees, sin t = cos^2 t: the weighted sum vanishes and
    // the longer projection (the head's) is taken.
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Turn(kX, 2.4753533f));
    up = GetUp(&kGround);
    CHECK(!up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, kZ),
          "inverted at 142 degrees: the head's projection");
}

// On its side (turned a quarter about z: the head at -x, the chest
// facing +z, across the normal): face up by the tie and looking toward
// the feet, as ALS has it; rolled a little forward, face down and
// looking where the head lies; a little back, face up again.
static void TestSide(void)
{
    static const struct
    {
        float roll;
        bool faceUp;
        float x;
        const char* what;
    } kCases[3] = {{0.0f, true, 1.0f, "on its side: toward the feet"},
                   {0.17f, false, -1.0f, "rolled forward: where the head lies"},
                   {-0.17f, true, 1.0f, "rolled back: toward the feet"}};
    for (int k = 0; k < 3; ++k)
    {
        Lie((maniVec3){0.0f, 0.3f, 0.0f}, Mul(Turn(kX, kCases[k].roll), Turn(kZ, 1.5707964f)));
        maniRagdollGetUp up = GetUp(&kGround);
        CHECK(up.faceUp == kCases[k].faceUp &&
                  Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, (maniVec3){kCases[k].x, 0.0f, 0.0f}),
              kCases[k].what);
    }
    // Face up with the head exactly at +z (half a turn about (0, 1, 1)):
    // the heading exactly opposite the model's forward.
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, (maniQuat){0.0f, 0.70710677f, 0.70710677f, 0.0f});
    maniRagdollGetUp up = GetUp(&kGround);
    CHECK(up.faceUp && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, (maniVec3){0.0f, 0.0f, -1.0f}),
          "exactly opposite the model's forward: half a turn about the normal");
}

// Tilted 45 degrees forward and twisted about its own up axis: both
// terms count and point apart, the heading the head's way plus the
// chest's weighed by how upright the body stands (the documented sum,
// worked here in binary64).
static void TestTwisted(void)
{
    static const float kTwists[3] = {0.6f, 1.2f, -0.9f};
    bool ok = true;
    for (int k = 0; k < 3; ++k)
    {
        maniQuat turn = Mul(Turn(kX, 0.78539819f), Turn(kY, kTwists[k]));
        Lie((maniVec3){0.0f, 0.3f, 0.0f}, turn);
        maniRagdollGetUp up = GetUp(&kGround);
        maniVec3 head = Rotate(turn, kY);
        maniVec3 chest = Rotate(turn, kZ);
        double sign = (double)chest.y >= 0.0 ? -1.0 : 1.0;
        double weight = fabs((double)head.y);
        double x = sign * (double)head.x + weight * (double)chest.x;
        double z = sign * (double)head.z + weight * (double)chest.z;
        double length = sqrt(x * x + z * z);
        maniVec3 want = {(float)(x / length), 0.0f, (float)(z / length)};
        ok = ok && up.faceUp == (chest.y >= 0.0f) && Placed(up, (maniVec3){0.0f, 0.1f, 0.0f}, want);
    }
    CHECK(ok, "tilted and twisted: the head's way plus the chest's, weighed");
}

// Unreal's model axes (+x forward, +z up) in the Y-up world; a ground
// tilted toward +z through another point.
static void TestFrames(void)
{
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, Turn(kX, 1.5707964f));
    maniRagdollGround ground = kGround;
    ground.forward = kX;
    ground.up = kZ;
    maniRagdollGetUp up = GetUp(&ground);
    CHECK(NearV(Rotate(up.placement.rotation, kX), kZ, 1e-5) &&
              NearV(Rotate(up.placement.rotation, kZ), kY, 1e-5),
          "the model's own axes onto the heading and the normal");
    ground = kGround;
    ground.point = (maniVec3){1.0f, 0.5f, -1.0f};
    ground.normal = (maniVec3){0.0f, 0.8f, 0.6f};
    Lie((maniVec3){2.0f, 1.0f, 3.0f}, (maniQuat){0.0f, 0.0f, 0.0f, 1.0f});
    up = GetUp(&ground);
    // (p - g).n = (1, 0.5, 4).(0, 0.8, 0.6) = 2.8.
    maniVec3 at = {2.0f, 1.0f - 2.8f * 0.8f, 3.0f - 2.8f * 0.6f};
    CHECK(NearV(up.placement.translation, at, 1e-5) &&
              NearV(Rotate(up.placement.rotation, kY), ground.normal, 1e-5),
          "on the tilted plane under the root, model up on its normal");
    maniVec3 forward = Rotate(up.placement.rotation, kZ);
    CHECK(Near(forward.x, 0.0, 1e-5) && Near(forward.y, -0.6, 1e-5) && Near(forward.z, 0.8, 1e-5),
          "forward the chest on the tilted plane");
}

// The fixture's bodies from a bent pose: a pose from them under the
// get-up placement places them back where they lay.
static void TestRoundTrip(void)
{
    static const maniTransform kLying = {
        {2.0f, 0.4f, -1.0f}, {0.70710677f, 0.0f, 0.0f, 0.70710677f}, {1.0f, 1.0f, 1.0f}};
    CHECK(maniComputeModel(s_poses[0], &s_skeleton) == mani_success &&
              maniRagdollBodiesFromPose(&s_ragdoll, s_poses[0], &kLying, NULL, NULL, 0.0f, s_states,
                                        5) == mani_success,
          "bodies lying face down");
    maniRagdollGetUp up = GetUp(&kGround);
    maniRagdollBodyState back[5];
    CHECK(maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, s_states, 5, &up.placement, s_poses[0],
                                    s_poses[1]) == mani_success &&
              maniRagdollBodiesFromPose(&s_ragdoll, s_poses[1], &up.placement, NULL, NULL, 0.0f,
                                        back, 5) == mani_success,
          "a pose in the get-up frame");
    bool same = true;
    for (int i = 0; i < 5; ++i)
    {
        same = same && NearV(back[i].position, s_states[i].position, 1e-5) &&
               fabs(fabs((double)(back[i].rotation.x * s_states[i].rotation.x +
                                  back[i].rotation.y * s_states[i].rotation.y +
                                  back[i].rotation.z * s_states[i].rotation.z +
                                  back[i].rotation.w * s_states[i].rotation.w)) -
                    1.0) < 1e-5;
    }
    CHECK(same, "the bodies placed back where they lay");
}

static maniResult With(const maniRagdollGround* ground)
{
    maniRagdollGetUp up = {0};
    return maniRagdollGetUpPlacement(&s_ragdoll, s_states, 5, ground, &up);
}

static void TestRefusals(void)
{
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, (maniQuat){0.0f, 0.0f, 0.0f, 1.0f});
    maniRagdollGetUp up = {0};
    CHECK(maniRagdollGetUpPlacement(NULL, s_states, 5, &kGround, &up) == mani_errorInvalid &&
              maniRagdollGetUpPlacement(&s_ragdoll, NULL, 5, &kGround, &up) == mani_errorInvalid &&
              maniRagdollGetUpPlacement(&s_ragdoll, s_states, 5, NULL, &up) == mani_errorInvalid &&
              maniRagdollGetUpPlacement(&s_ragdoll, s_states, 5, &kGround, NULL) ==
                  mani_errorInvalid,
          "NULL arguments");
    CHECK(maniRagdollGetUpPlacement(&s_ragdoll, s_states, 4, &kGround, &up) == mani_errorCapacity,
          "too few bodies");
    s_states[4].position.y = HUGE_VALF;
    CHECK(With(&kGround) == mani_errorInvalid, "the last body's place not finite");
    Lie((maniVec3){0.0f, 0.3f, 0.0f}, (maniQuat){0.0f, 0.0f, 0.0f, 1.0f});
    s_states[2].rotation.w = 1.0011f;
    CHECK(With(&kGround) == mani_errorInvalid, "a rotation of length 1.0011");
    s_states[2].rotation.w = 1.0009f;
    CHECK(With(&kGround) == mani_success, "one of length 1.0009 taken");
    s_states[2].rotation.w = 1.0f;
    maniRagdollGround g = kGround;
    g.point.z = NAN;
    CHECK(With(&g) == mani_errorInvalid, "a ground point not finite");
    g = kGround;
    g.normal.y = 1.0011f;
    CHECK(With(&g) == mani_errorInvalid, "a normal not of length 1");
    g = kGround;
    g.forward = (maniVec3){0.0f, 0.0011f, 1.0f};
    CHECK(With(&g) == mani_errorInvalid, "axes not perpendicular");
    g.forward = (maniVec3){0.0f, 0.0009f, 1.0f};
    CHECK(With(&g) == mani_success, "axes perpendicular within a thousandth");
    g = kGround;
    g.up = (maniVec3){0.0f, 0.0f, 0.0f};
    CHECK(With(&g) == mani_errorInvalid, "a zero up axis");
}

int main(void)
{
    OpenSkeleton();
    maniRagdollSource source = Source();
    CHECK(Build(&source).result == mani_success &&
              maniOpenRagdoll(s_file, s_size, &s_ragdoll).result == mani_success,
          "a profile");
    for (int k = 0; k < 2; ++k)
    {
        CHECK(maniInitPose(&s_skeleton, s_memory[k], sizeof s_memory[k], &s_poses[k]) ==
                  mani_success,
              "a pose");
    }
    TestUpright();
    TestLying();
    TestSweep();
    TestSide();
    TestTwisted();
    TestFrames();
    TestRoundTrip();
    TestRefusals();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
