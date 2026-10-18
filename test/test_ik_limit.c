// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Joint limits on their own (mani-0030): a turn within its limit comes
// back bit for bit; a cone cuts the swing to its edge about the same
// axis and the twist to its range; a hinge keeps only its own turn; a
// half turn about an axis perpendicular to the bone's (the bone pointing
// opposite its rest direction) has no twist, its whole turn the swing,
// and turns just short of it limit to results that approach that one; a
// bone of no axis takes its whole turn as the swing.

#include "ik_limit.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/ik.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

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

static float Angle(maniQuat a, maniQuat b)
{
    float d = fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

static const maniVec3 kX = {1.0f, 0.0f, 0.0f};
static const maniVec3 kY = {0.0f, 1.0f, 0.0f};
static const maniVec3 kNone = {0.0f, 0.0f, 0.0f};

static void TestWithin(void)
{
    maniJointLimit cone = {
        .kind = maniJointLimitCone, .cone = 0.8f, .twistLow = -0.3f, .twistHigh = 0.3f};
    maniQuat r = Mul(Turn((maniVec3){0.6f, 0.0f, 0.8f}, 0.5f), Turn(kY, 0.2f));
    maniQuat limited = maniIkLimit(&cone, kY, r);
    CHECK(memcmp(&limited, &r, sizeof r) == 0, "a turn within its cone comes back as it was");
    maniQuat free = maniIkLimit(NULL, kY, r);
    CHECK(memcmp(&free, &r, sizeof r) == 0, "no limit");
}

static void TestCone(void)
{
    maniJointLimit cone = {
        .kind = maniJointLimitCone, .cone = 0.4f, .twistLow = -0.1f, .twistHigh = 0.2f};
    maniQuat r = Mul(Turn(kX, 1.2f), Turn(kY, 0.5f));
    maniQuat limited = maniIkLimit(&cone, kY, r);
    maniQuat want = Mul(Turn(kX, 0.4f), Turn(kY, 0.2f));
    CHECK(Angle(limited, want) < 1e-5f, "swing cut to the cone about its axis, twist to its range");
}

static void TestHinge(void)
{
    maniJointLimit hinge = {.kind = maniJointLimitHinge, .axis = kX, .low = -0.5f, .high = 0.3f};
    maniQuat r = Mul(Turn(kX, 0.2f), Turn((maniVec3){0.0f, 0.6f, 0.8f}, 0.4f));
    maniQuat limited = maniIkLimit(&hinge, kY, r);
    CHECK(fabsf(limited.y) < 1e-7f && fabsf(limited.z) < 1e-7f && limited.x > 0.0f,
          "a hinge keeps only its own turn");
    limited = maniIkLimit(&hinge, kY, Turn(kX, -0.9f));
    CHECK(Angle(limited, Turn(kX, -0.5f)) < 1e-5f, "within its range");
}

static void TestHalfTurn(void)
{
    maniJointLimit cone = {
        .kind = maniJointLimitCone, .cone = 1.0f, .twistLow = -0.2f, .twistHigh = 0.2f};
    maniQuat half = Turn(kX, 3.14159265f);
    half.w = 0.0f;
    maniQuat limited = maniIkLimit(&cone, kY, half);
    CHECK(Angle(limited, Turn(kX, 1.0f)) < 1e-5f, "a half turn: no twist, the swing to the cone");
    float worst = 0.0f;
    for (int k = 1; k <= 8; ++k)
    {
        float e = 1e-3f / (float)(1 << k);
        maniQuat near = maniIkLimit(&cone, kY, Turn(kX, 3.14159265f - e));
        worst = fmaxf(worst, Angle(near, limited));
    }
    CHECK(worst < 1e-4f, "turns just short of the half turn approach its result");
    maniQuat twisted = Mul(Turn(kX, 3.1f), Turn(kY, 0.5f));
    maniQuat out = maniIkLimit(&cone, kY, twisted);
    CHECK(isfinite(out.x) && isfinite(out.w) &&
              Angle(out, Mul(Turn(kX, 1.0f), Turn(kY, 0.2f))) < 1e-4f,
          "near the half turn, twisted: both limited");
}

static void TestNoAxis(void)
{
    maniJointLimit cone = {
        .kind = maniJointLimitCone, .cone = 0.3f, .twistLow = -0.1f, .twistHigh = 0.1f};
    maniQuat limited = maniIkLimit(&cone, kNone, Turn(kY, 1.0f));
    CHECK(Angle(limited, Turn(kY, 0.3f)) < 1e-5f, "no axis: the whole turn is the swing");
}

int main(void)
{
    TestWithin();
    TestCone();
    TestHinge();
    TestHalfTurn();
    TestNoAxis();
    return s_failures == 0 ? 0 : 1;
}
