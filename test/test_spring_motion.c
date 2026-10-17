// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring motion (mani-0026) on the fixture's chain: the steps against a
// binary64 reference of VRMC_springBone's update, a chain at rest kept,
// gravity hanging it at its bones' lengths, the same motion at host
// rates of 30, 60 and 144 Hz (still, moving, turning), the character's
// motion in the world swaying it, a center bone carrying it, a ball, a
// floor and cones kept (on a mirrored bone too), a turned, moved and
// scaled character moving alike, gravity in the world.

#include "spring_fixture.h"

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
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
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
    Config c = {1.0f, 0.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
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
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    bool lengths = true;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
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
        CHECK(Frame(state, 1.0f / (float)rate, &place), "a frame");
    }
    return Position(s_b[2]);
}

static void TestHostRates(void)
{
    MakeSet(&kSwing);
    maniVec3 at60 = AfterSecond(60);
    CHECK(Distance(AfterSecond(30), at60) < 1e-6f && Distance(AfterSecond(144), at60) < 1e-6f,
          "the same motion at every host rate");
    at60 = AfterSway(60);
    // The steps see the inputs interpolated between host frames: 0.5 mm
    // off at 30 Hz, 0.04 mm at 144 Hz.
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
    Config c = {1.0f, 0.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    maniTransform place = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    for (int i = 0; i < 30; ++i)
    {
        place.translation.y += 0.02f;
        CHECK(Frame(state, 1.0f / 60.0f, &place), "a frame");
    }
    CHECK(Position(s_b[2]).y < 0.99f, "moving up in the world sways the tail down");
    for (int i = 0; i < 600; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, &place), "a frame");
    }
    maniVec3 b2 = {1.0f, 1.0f, 0.0f};
    CHECK(Distance(Position(s_b[2]), b2) < 1e-3f, "and stiffness brings it back");
}

// The last tail's height above the root after the root rises in the pose.
static float AfterRise(uint32_t center)
{
    Config c = {1.0f, 0.0f, 0.4f, maniSpringLimitNone, 0.0f, center, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    for (int i = 0; i < 30; ++i)
    {
        s_rootLocal.translation.y += 0.02f;
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
    }
    float above = Position(s_b[2]).y - s_rootLocal.translation.y;
    Still();
    return above;
}

static void TestCenter(void)
{
    CHECK(AfterRise(MANI_NO_BONE) < 0.99f, "a rising root sways a chain without a center");
    CHECK(fabsf(AfterRise(s_root) - 1.0f) < 1e-4f, "and carries one in its space");
}

static void TestCollider(void)
{
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 1, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    float nearest = 1.0f;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
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
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitCone, 0.3f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    float widest = 0.0f;
    float below = 0.0f;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
        maniVec3 b0 = Position(s_b[0]);
        maniVec3 b1 = Position(s_b[1]);
        maniVec3 b2 = Position(s_b[2]);
        float x = (b1.x - b0.x) / Distance(b1, b0);
        float angle = acosf(x < 1.0f ? x : 1.0f);
        widest = angle > widest ? angle : widest;
        x = (b2.x - b1.x) / Distance(b2, b1);
        angle = acosf(x < 1.0f ? x : 1.0f);
        below = angle > below ? angle : below;
    }
    CHECK(widest < 0.3f + 2e-3f && widest > 0.29f, "the cone holds gravity at its edge");
    // The second joint's cone turns with the first.
    CHECK(below > 0.55f && below < 0.6f + 4e-3f, "a cone about the turned parent");
}

static maniVec3 Turned(maniTransform t, maniVec3 x)
{
    // t's rotation is about y alone here.
    float c = 1.0f - 2.0f * t.rotation.y * t.rotation.y;
    float s = 2.0f * t.rotation.y * t.rotation.w;
    maniVec3 v = {x.x * t.scale.x, x.y * t.scale.y, x.z * t.scale.z};
    return (maniVec3){c * v.x + s * v.z + t.translation.x, v.y + t.translation.y,
                      -s * v.x + c * v.z + t.translation.z};
}

static void TestTransformed(void)
{
    // Limits, a collider and gravity about y: a character turned about
    // y, moved and twice the size moves alike, twice the size. Compared
    // early: the chain comes to rest on top of the ball, a balance that
    // rounding (here the turn) tips over in time.
    Config c = {0.2f, 2.0f, 0.4f, maniSpringLimitCone, 1.0f, MANI_NO_BONE, 1, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniVec3 still[2][2];
    maniSpringState* state = NewState(0);
    for (int i = 1; i <= 25; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
        if (i == 10 || i == 25)
        {
            still[i / 25][0] = Position(s_b[1]);
            still[i / 25][1] = Position(s_b[2]);
        }
    }
    CHECK(Distance(still[1][0], kBall) < kBallRadius + kHit + 2e-2f, "on the ball");
    s_rootLocal = (maniTransform){
        {1.0f, 0.0f, 2.0f}, {0.0f, 0.70710678f, 0.0f, 0.70710678f}, {2.0f, 2.0f, 2.0f}};
    state = NewState(0);
    bool alike = true;
    for (int i = 1; i <= 25; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
        if (i == 10 || i == 25)
        {
            alike = alike &&
                    Distance(Position(s_b[1]), Turned(s_rootLocal, still[i / 25][0])) < 2e-4f &&
                    Distance(Position(s_b[2]), Turned(s_rootLocal, still[i / 25][1])) < 2e-4f;
        }
    }
    Still();
    CHECK(alike, "a turned, moved and scaled character moves alike");
}

static void TestLying(void)
{
    // A character on its back (placement turned about x, its rotation
    // not unit) feels gravity along its +z.
    Config side = {0.5f, 1.0f, 0.2f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 0, {0.0f, 0.0f, 1.0f},
                   0.0f, 0.0f};
    MakeSet(&side);
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / 60.0f, 60), "frames");
    maniVec3 want = Position(s_b[2]);
    MakeSet(&kSwing);
    state = NewState(0);
    maniTransform place = {{3.0f, 1.0f, 0.0f}, {1.41421356f, 0.0f, 0.0f, 1.41421356f}, {1, 1, 1}};
    for (int i = 0; i < 60; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, &place), "a frame");
    }
    CHECK(Distance(Position(s_b[2]), want) < 1e-5f && Distance(want, (maniVec3){1, 1, 0}) > 0.1f,
          "gravity is the world's");
}

// The last tail after a second at a host rate, the character turning
// in the world, its root tilting in the pose and moving.
static maniVec3 AfterTurns(uint32_t rate)
{
    maniSpringState* state = NewState(0);
    for (uint32_t i = 1; i <= rate; ++i)
    {
        float a = sinf(6.2831853f * (float)i / (float)rate);
        maniTransform place = {
            {0.0f, 0.0f, 0.0f}, {0.0f, sinf(0.25f * a), 0.0f, cosf(0.25f * a)}, {1, 1, 1}};
        s_rootLocal.rotation = (maniQuat){0.0f, 0.0f, sinf(0.2f * a), cosf(0.2f * a)};
        s_rootLocal.translation.y = 0.2f * a;
        CHECK(Frame(state, 1.0f / (float)rate, &place), "a frame");
    }
    Still();
    return Position(s_b[2]);
}

static void TestTurningRates(void)
{
    Config c = {0.5f, 1.0f, 0.2f, maniSpringLimitCone, 0.4f, MANI_NO_BONE, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniVec3 at60 = AfterTurns(60);
    // 0.6 mm off at 30 Hz, 0.02 mm at 144 Hz.
    CHECK(Distance(AfterTurns(30), at60) < 3e-3f && Distance(AfterTurns(144), at60) < 2e-4f,
          "turning inputs at every host rate");
    c.center = s_root;
    MakeSet(&c);
    at60 = AfterTurns(60);
    // In the root's space the same to rounding.
    CHECK(Distance(AfterTurns(30), at60) < 1e-5f && Distance(AfterTurns(144), at60) < 1e-5f,
          "and in a center's space");
}

static void TestFloor(void)
{
    // A floor on a mirrored b0: the chain runs along -x, within cones
    // about it, and falls on the floor.
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitCone, 1.2f, MANI_NO_BONE, 2, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    s_b0Scale = (maniVec3){-1.0f, 1.0f, 1.0f};
    maniSpringState* state = NewState(0);
    float lowest = 1.0f;
    for (int i = 0; i < 300; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
        float y = Position(s_b[1]).y < Position(s_b[2]).y ? Position(s_b[1]).y : Position(s_b[2]).y;
        lowest = y < lowest ? y : lowest;
    }
    Still();
    CHECK(Position(s_b[2]).x < -0.5f, "mirrored");
    // The length after the push lets a falling tail through by a little
    // (1.6 cm here at the first touch, 1 cm at rest).
    CHECK(lowest > kFloor + kHit - 2e-2f && fabsf(Position(s_b[2]).y - kFloor - kHit) < 1e-2f,
          "the floor holds the chain");
}

static void TestTiltedFloor(void)
{
    // A floor and gravity turned with the character: a quarter about x
    // (y to z) turns the floor's normal with b0, the chain alike.
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 2, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / 60.0f, 40), "frames");
    maniVec3 b1 = Position(s_b[1]);
    maniVec3 b2 = Position(s_b[2]);
    c.gravityDirection = (maniVec3){0.0f, 0.0f, -1.0f};
    MakeSet(&c);
    s_rootLocal.rotation = (maniQuat){0.70710678f, 0.0f, 0.0f, 0.70710678f};
    state = NewState(0);
    CHECK(Frames(state, 1.0f / 60.0f, 40), "frames");
    CHECK(Distance(Position(s_b[1]), (maniVec3){b1.x, -b1.z, b1.y}) < 1e-4f &&
              Distance(Position(s_b[2]), (maniVec3){b2.x, -b2.z, b2.y}) < 1e-4f && b2.y < 0.8f,
          "a tilted floor holds alike");
    Still();
}

static void TestShell(void)
{
    // An inside sphere about b0 folds the falling chain into it.
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 3, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    float farthest = 0.0f;
    for (int i = 0; i < 120; ++i)
    {
        CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
        float d = Distance(Position(s_b[2]), Position(s_b[0]));
        farthest = i >= 5 && d > farthest ? d : farthest;
    }
    CHECK(farthest < kShell - kHit + 2e-2f, "the shell keeps the tail in");
}

static void TestSpherical(void)
{
    // A spherical limit of a wide pitch and a narrow yaw: from +x, the
    // yaw is toward -y, so gravity drops the chain by the yaw alone.
    Config c = {0.0f, 2.0f,         0.4f, maniSpringLimitSpherical,
                1.5f, MANI_NO_BONE, 0,    {0.0f, 0.0f, 0.0f},
                0.2f, 0.0f};
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / 60.0f, 120), "frames");
    maniVec3 b0 = Position(s_b[0]);
    maniVec3 b1 = Position(s_b[1]);
    float drop = asinf((b0.y - b1.y) / Distance(b0, b1));
    CHECK(drop < 0.2f + 2e-3f && drop > 0.19f, "the yaw holds gravity at its edge");
}

static void TestExactRate(void)
{
    // A host at a step rate a power of two divides: one step each call,
    // the clock never off zero.
    Config c = kSwing;
    c.rate = 64.0f;
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    bool zero = true;
    for (int i = 0; i < 10; ++i)
    {
        CHECK(Frame(state, 1.0f / 64.0f, NULL), "a frame");
        zero = zero && SavedFloat(state, 2) == 0.0f && SavedFloat(state, 3) == 0.0f;
    }
    CHECK(zero && Position(s_b[2]).y < 0.999f, "a step a call");
}

static void TestApplied(void)
{
    // At an exact rate the output is the last step: the applied pose puts
    // each tail there, the chain swinging out of its plane, its root
    // tilting and b1 turned about its bone.
    Config c = kSwing;
    c.rate = 64.0f;
    MakeSet(&c);
    s_b1Rotation = (maniQuat){0.34289781f, 0.0f, 0.0f, 0.93937271f};
    maniSpringState* state = NewState(0);
    float worst = 0.0f;
    for (uint32_t i = 1; i <= 64; ++i)
    {
        float a = sinf(6.2831853f * (float)i / 64.0f);
        maniTransform place = {
            {0.0f, 0.0f, 0.0f}, {0.0f, sinf(0.25f * a), 0.0f, cosf(0.25f * a)}, {1, 1, 1}};
        s_rootLocal.rotation = (maniQuat){0.0f, 0.0f, sinf(0.2f * a), cosf(0.2f * a)};
        CHECK(Frame(state, 1.0f / 64.0f, &place), "a frame");
        for (uint32_t j = 0; j < 2; ++j)
        {
            uint64_t at = 18 + 17 * (uint64_t)j;
            maniVec3 before = {SavedFloat(state, at), SavedFloat(state, at + 1),
                               SavedFloat(state, at + 2)};
            float d = Distance(Position(s_b[j + 1]), before);
            worst = d > worst ? d : worst;
        }
    }
    Still();
    CHECK(worst < 2e-5f, "the pose's tails are the steps'");
}

// The last tail after a second at a host rate, the root moving at a
// steady speed under the chain resting on the ball.
static maniVec3 AfterGlide(uint32_t rate)
{
    maniSpringState* state = NewState(0);
    for (uint32_t i = 1; i <= rate; ++i)
    {
        s_rootLocal.translation.x = 0.5f * (float)i / (float)rate;
        CHECK(Frame(state, 1.0f / (float)rate, NULL), "a frame");
    }
    Still();
    return Position(s_b[2]);
}

static void TestColliderRates(void)
{
    Config c = {0.0f, 2.0f, 0.4f, maniSpringLimitNone, 0.0f, MANI_NO_BONE, 1, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniVec3 at60 = AfterGlide(60);
    // Inputs moving at a steady speed interpolate exactly.
    CHECK(Distance(AfterGlide(30), at60) < 1e-5f && Distance(AfterGlide(144), at60) < 1e-5f,
          "a collider moving with the chain at every host rate");
}

static void TestLyingMoving(void)
{
    // A character on its back moving in the world, its tails between
    // steps (240 Hz): as one upright moving likewise in its own space
    // with gravity along +z.
    maniQuat back = {1.41421356f, 0.0f, 0.0f, 1.41421356f};
    Config side = kSwing;
    side.gravityDirection = (maniVec3){0.0f, 0.0f, 1.0f};
    maniVec3 want[2];
    for (int run = 0; run < 2; ++run)
    {
        MakeSet(run == 0 ? &side : &kSwing);
        maniSpringState* state = NewState(0);
        for (uint32_t i = 1; i <= 240; ++i)
        {
            float t = (float)i / 240.0f;
            maniVec3 world = {0.3f * sinf(6.2831853f * t), 0.2f * t, 0.1f * cosf(6.2831853f * t)};
            // On its back, the model's (x, y, z) is the world's (x, -z, y).
            maniTransform place = {
                {world.x, world.z, -world.y}, {0.0f, 0.0f, 0.0f, 1.0f}, {1, 1, 1}};
            if (run == 1)
            {
                place = (maniTransform){world, back, {1, 1, 1}};
            }
            CHECK(Frame(state, 1.0f / 240.0f, &place), "a frame");
        }
        want[run] = Position(s_b[2]);
    }
    CHECK(Distance(want[0], want[1]) < 1e-5f, "moving on its back");
}

int main(void)
{
    Setup();
    TestReference();
    TestRest();
    TestGravity();
    TestHostRates();
    TestPlacement();
    TestCenter();
    TestCollider();
    TestLimit();
    TestTransformed();
    TestLying();
    TestTurningRates();
    TestFloor();
    TestTiltedFloor();
    TestShell();
    TestSpherical();
    TestExactRate();
    TestApplied();
    TestColliderRates();
    TestLyingMoving();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
