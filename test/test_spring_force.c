// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring bones' external force (mani-0026) on the fixture's chain: a
// force moves the chain as gravity of the same size and direction does,
// in the world whatever the character's turn, scaled with the head, at
// the reference's step and at another; a wind changing every frame moves
// alike at 30, 60 and 144 Hz hosts, the force interpolated between calls;
// a steady force steady from the first call, and from the first after a
// reset, at 30 and 120 Hz hosts; no force ramping down as a zero force,
// stopped at once by a call of no time; a force not finite is refused
// and a saved state's force is checked.

#include "spring_fixture.h"
#include "spring_state.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdint.h>

static const Config kStill = {
    0.5f, 0.0f, 0.2f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f}, 0.0f, 0.0f};

// The last tail after frames at 60 Hz under the placement.
static maniVec3 After(uint32_t frames, const maniTransform* place)
{
    maniSpringState* state = NewState(0);
    for (uint32_t i = 0; i < frames; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, place), "a frame");
    }
    return Position(s_b[2]);
}

// A force as gravity: the chain moves as under gravity of the force's
// size and direction, in the world (the character turned a quarter about
// x, the force still along the world's -y, across the chain), scaled
// with b0.
static void TestAsGravity(void)
{
    const maniTransform turned = {
        {0.3f, 0.0f, 0.0f}, {0.70710678f, 0.0f, 0.0f, 0.70710678f}, {1.0f, 1.0f, 1.0f}};
    const maniTransform* places[2] = {NULL, &turned};
    for (int scaled = 0; scaled < 3; ++scaled)
    {
        for (int p = 0; p < 2; ++p)
        {
            // The third pass steps at 120 Hz: a step's share of the force
            // is no longer the reference frame's.
            Config c = kStill;
            c.rate = scaled == 2 ? 120.0f : 0.0f;
            Still();
            s_b0Scale = scaled == 1 ? (maniVec3){1.5f, 1.5f, 1.5f} : s_b0Scale;
            c.gravity = 1.5f;
            MakeSet(&c);
            maniVec3 gravity = After(90, places[p]);
            Still();
            s_b0Scale = scaled == 1 ? (maniVec3){1.5f, 1.5f, 1.5f} : s_b0Scale;
            c.gravity = 0.0f;
            MakeSet(&c);
            s_force = (maniVec3){0.0f, -1.5f, 0.0f};
            s_forced = true;
            maniVec3 forced = After(90, places[p]);
            s_forced = false;
            maniVec3 none = After(90, places[p]);
            CHECK(Distance(gravity, forced) < 1e-5f && Distance(forced, none) > 0.05f,
                  "a force moves the chain as gravity of its size does");
        }
    }
    Still();
}

// The chain's last tail after a second of a wind across the chain (along
// z) changing at a hertz, at a host rate.
static maniVec3 AfterWind(uint32_t rate)
{
    maniSpringState* state = NewState(0);
    s_forced = true;
    for (uint32_t i = 1; i <= rate; ++i)
    {
        s_force = (maniVec3){0.0f, 0.0f, 3.0f * sinf(6.2831853f * (float)i / (float)rate)};
        CHECK(Frame(state, 1.0f / (float)rate, NULL), "a frame");
    }
    Still();
    return Position(s_b[2]);
}

static void TestHostRates(void)
{
    MakeSet(&kStill);
    maniVec3 at60 = AfterWind(60);
    maniVec3 calm = After(60, NULL);
    // Interpolated between frames, the wind moves the chain alike at
    // every host rate; held over a call, it would not.
    CHECK(Distance(at60, calm) > 0.05f && Distance(AfterWind(30), at60) < 2e-3f &&
              Distance(AfterWind(144), at60) < 5e-4f,
          "a changing wind alike at every host rate");
}

// The chain's last tail after a second of a steady force from the first
// call (or the first after a reset, a half second of another force
// before it) at a host rate.
static maniVec3 AfterSteady(uint32_t rate, bool reset)
{
    maniSpringState* state = NewState(0);
    s_forced = true;
    if (reset)
    {
        s_force = (maniVec3){2.0f, 0.0f, 0.0f};
        CHECK(Frames(state, 1.0f / 60.0f, 30), "frames before");
        maniResetSprings(state);
    }
    s_force = (maniVec3){0.0f, 0.0f, 3.0f};
    CHECK(Frames(state, 1.0f / (float)rate, rate), "a second of a steady force");
    Still();
    return Position(s_b[2]);
}

static void TestFirstCall(void)
{
    // The first call has no last force and takes its own for both ends:
    // a steady force is steady from the first step at any host rate.
    MakeSet(&kStill);
    maniVec3 at30 = AfterSteady(30, false);
    maniVec3 calm = After(60, NULL);
    CHECK(Distance(at30, calm) > 0.05f && Distance(AfterSteady(120, false), at30) < 1e-5f,
          "a steady force from the first call alike at two rates");
    CHECK(Distance(AfterSteady(30, true), at30) < 1e-5f &&
              Distance(AfterSteady(120, true), at30) < 1e-5f,
          "and from the first call after a reset");
}

static void TestNoForce(void)
{
    // No force after one ramps it down over the call; a call of no time
    // without it first stops it at once.
    MakeSet(&kStill);
    maniVec3 tails[3];
    for (int k = 0; k < 3; ++k)
    {
        maniSpringState* state = NewState(0);
        s_forced = true;
        s_force = (maniVec3){0.0f, 0.0f, 3.0f};
        CHECK(Frames(state, 1.0f / 60.0f, 30), "frames in a force");
        s_forced = k == 2;
        s_force = (maniVec3){0.0f, 0.0f, 0.0f};
        if (k == 1)
        {
            CHECK(Frame(state, 0.0f, NULL), "a call of no time without a force");
        }
        CHECK(Frame(state, 0.25f, NULL), "a call without a force");
        tails[k] = Position(s_b[2]);
    }
    Still();
    CHECK(Distance(tails[0], tails[1]) > 1e-3f && Distance(tails[0], tails[2]) == 0.0f,
          "no force ramps down as a zero force does; stopped at once by a call of no time");
}

static void TestRefusalsAndSaved(void)
{
    MakeSet(&kStill);
    maniSpringState* state = NewState(0);
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    const maniVec3 bad[3] = {{NAN, 0.0f, 0.0f}, {0.0f, INFINITY, 0.0f}, {0.0f, 0.0f, -INFINITY}};
    bool refused = true;
    for (int k = 0; k < 3; ++k)
    {
        refused =
            refused && maniStepSprings(state, s_pose, NULL, &bad[k], 0.01f) == mani_errorInvalid;
    }
    CHECK(refused, "a force not finite refused");
    const maniVec3 wind = {0.5f, 0.0f, 0.25f};
    CHECK(maniStepSprings(state, s_pose, NULL, &wind, 1.0f / 60.0f) == mani_success &&
              maniStepSprings(state, s_pose, NULL, &wind, 1.0f / 60.0f) == mani_success,
          "stepped in a wind");
    maniVec3 kept = maniSpringVec(maniSpringConstBlock(state), maniSpringForce);
    CHECK(kept.x == 0.5f && kept.y == 0.0f && kept.z == 0.25f, "the call's force kept");
    size_t size = maniSpringSavedSize(&s_set);
    CHECK(maniSaveSpringState(state, s_saved[0], size) == mani_success, "saved");
    maniStoreU32(s_saved[0] + 24 + 4 * (maniSpringForce + 1), 0x7FC00000u);
    maniFileResult r = maniRestoreSpringState(state, s_saved[0], size);
    CHECK(r.result == mani_errorFormat && r.offset == 24 + 4 * (maniSpringForce + 1),
          "a saved force not finite refused at its word");
}

int main(void)
{
    Setup();
    TestAsGravity();
    TestHostRates();
    TestFirstCall();
    TestNoForce();
    TestRefusalsAndSaved();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
