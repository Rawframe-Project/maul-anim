// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Trajectory prediction: the springs against Holden's
// formulas in binary64 over random states, goals and steps; one advance
// against many shorter ones; a trajectory settling on its goal; the
// refusals, leaving it unchanged; a sample's values as the database
// measures its own (the walk's root half a second ahead), turned with
// the root, and on the ground of a database of up z and forward -x.

#include "graph_match_fixture.h"
#include "vecmath.h"

#include "maul-anim/trajectory.h"

typedef struct D3
{
    double x, y, z;
} D3;

static D3 D(maniVec3 v)
{
    return (D3){(double)v.x, (double)v.y, (double)v.z};
}

static D3 Comb(D3 a, double sa, D3 b, double sb)
{
    return (D3){a.x * sa + b.x * sb, a.y * sa + b.y * sb, a.z * sa + b.z * sb};
}

static bool Close(maniVec3 got, D3 want, double tolerance)
{
    double scale = 1.0 + fabs(want.x) + fabs(want.y) + fabs(want.z);
    return fabs((double)got.x - want.x) <= tolerance * scale &&
           fabs((double)got.y - want.y) <= tolerance * scale &&
           fabs((double)got.z - want.z) <= tolerance * scale;
}

// Holden's simulation_positions_update in binary64, his
// halflife_to_damping's eps.
static bool PositionsMatch(const maniTrajectory* before, const maniTrajectory* after,
                           const maniTrajectoryGoal* g, double dt)
{
    double y = 2.0 * 0.6931471805599453 / ((double)g->positionHalfLife + 1e-5);
    D3 j0 = Comb(D(before->velocity), 1.0, D(g->velocity), -1.0);
    D3 j1 = Comb(D(before->acceleration), 1.0, j0, y);
    double e = exp(-y * dt);
    D3 x = Comb(j1, -e / (y * y) + 1.0 / (y * y), j0, -e / y + 1.0 / y);
    x = Comb(x, 1.0, j1, -e * dt / y);
    x = Comb(Comb(x, 1.0, D(g->velocity), dt), 1.0, D(before->position), 1.0);
    D3 v = Comb(Comb(j0, e, j1, e * dt), 1.0, D(g->velocity), 1.0);
    D3 a = Comb(D(before->acceleration), e, j1, -e * y * dt);
    return Close(after->position, x, 1e-5) && Close(after->velocity, v, 1e-5) &&
           Close(after->acceleration, a, 1e-5);
}

static uint64_t s_state = 0x2545F4914F6CDD1Dull;

static float Random(float low, float high)
{
    s_state ^= s_state << 13;
    s_state ^= s_state >> 7;
    s_state ^= s_state << 17;
    return low + (high - low) * (float)(s_state >> 40) / 16777216.0f;
}

static maniVec3 RandomVec(float r)
{
    return (maniVec3){Random(-r, r), Random(-r, r), Random(-r, r)};
}

static maniQuat RandomTurn(void)
{
    maniVec3 axis = maniVec3Normalize(RandomVec(1.0f), (maniVec3){0, 1, 0});
    return maniQuatFromAxisAngle(axis, Random(-3.0f, 3.0f));
}

static maniTrajectory RandomTrajectory(void)
{
    return (maniTrajectory){RandomVec(10.0f), RandomVec(3.0f), RandomVec(5.0f), RandomTurn(),
                            RandomVec(2.0f)};
}

static maniTrajectoryGoal RandomGoal(void)
{
    return (maniTrajectoryGoal){RandomVec(3.0f), RandomTurn(), Random(0.05f, 1.0f),
                                Random(0.05f, 1.0f)};
}

static void TestHolden(void)
{
    bool positions = true;
    bool rotations = true;
    for (int k = 0; k < 2000; ++k)
    {
        maniTrajectory t = RandomTrajectory();
        maniTrajectoryGoal g = RandomGoal();
        float dt = Random(0.0f, 0.5f);
        maniTrajectory after = t;
        CHECK(maniAdvanceTrajectory(&after, &g, dt) == mani_success, "advanced");
        positions = positions && PositionsMatch(&t, &after, &g, (double)dt);
        // The rotation's offset from the goal decays as the position's
        // does with no acceleration: its scaled axis-angle e (j0 + j1 t).
        double y = 2.0 * 0.6931471805599453 / ((double)g.rotationHalfLife + 1e-5);
        double e = exp(-y * (double)dt);
        maniVec3 j0 =
            maniQuatToRotationVector(maniQuatMul(t.rotation, maniQuatConjugate(g.rotation)));
        D3 j1 = Comb(D(t.angularVelocity), 1.0, D(j0), y);
        D3 offset = Comb(Comb(D(j0), e, j1, e * (double)dt), 1.0, (D3){0, 0, 0}, 0.0);
        maniQuat want = maniQuatMul(maniQuatFromRotationVector((maniVec3){
                                        (float)offset.x, (float)offset.y, (float)offset.z}),
                                    g.rotation);
        D3 w = Comb(D(t.angularVelocity), e, j1, -e * y * (double)dt);
        rotations = rotations && fabsf(maniQuatDot(after.rotation, want)) >= 1.0f - 1e-5f &&
                    Close(after.angularVelocity, w, 1e-5);
    }
    CHECK(positions, "the positions as Holden's springs");
    CHECK(rotations, "the rotations as Holden's springs");
}

static void TestSettles(void)
{
    maniTrajectory t = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}};
    maniQuat left = maniQuatFromAxisAngle((maniVec3){0, 1, 0}, 1.5f);
    maniTrajectoryGoal g = {{1.0f, 0.0f, 2.0f}, left, 0.27f, 0.27f};
    maniTrajectory once = t;
    CHECK(maniAdvanceTrajectory(&once, &g, 0.5f) == mani_success, "half a second at once");
    maniTrajectory steps = t;
    for (int k = 0; k < 10; ++k)
    {
        CHECK(maniAdvanceTrajectory(&steps, &g, 0.05f) == mani_success, "a twentieth");
    }
    CHECK(Close(once.position, D(steps.position), 1e-5) &&
              Close(once.velocity, D(steps.velocity), 1e-5) &&
              fabsf(maniQuatDot(once.rotation, steps.rotation)) >= 1.0f - 1e-6f,
          "one advance as ten");
    CHECK(maniAdvanceTrajectory(&t, &g, 5.0f) == mani_success &&
              Close(t.velocity, D(g.velocity), 1e-5) &&
              Close(t.acceleration, (D3){0, 0, 0}, 1e-5) &&
              fabsf(maniQuatDot(t.rotation, left)) >= 1.0f - 1e-6f,
          "settled on the goal");
    maniTrajectory unit = once;
    maniTrajectory scaled = once;
    maniTrajectoryGoal doubled = g;
    doubled.rotation = (maniQuat){2.0f * left.x, 2.0f * left.y, 2.0f * left.z, 2.0f * left.w};
    CHECK(maniAdvanceTrajectory(&unit, &g, 0.2f) == mani_success &&
              maniAdvanceTrajectory(&scaled, &doubled, 0.2f) == mani_success &&
              memcmp(&unit, &scaled, sizeof unit) == 0,
          "a goal's rotation normalized");
    maniTrajectory still = t;
    CHECK(maniAdvanceTrajectory(&still, &g, 0.0f) == mani_success &&
              Close(still.position, D(t.position), 1e-6),
          "no time, no move");
}

static void TestRefusals(void)
{
    maniTrajectory t = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}};
    maniTrajectoryGoal g = {{1, 0, 0}, {0, 0, 0, 1}, 0.27f, 0.27f};
    maniTrajectory bad[3] = {t, t, t};
    bad[0].position.x = NAN;
    bad[1].rotation = (maniQuat){0, 0, 0, 2};
    bad[2].angularVelocity.y = INFINITY;
    for (int k = 0; k < 3; ++k)
    {
        maniTrajectory kept = bad[k];
        CHECK(maniAdvanceTrajectory(&bad[k], &g, 0.1f) == mani_errorInvalid &&
                  memcmp(&kept, &bad[k], sizeof kept) == 0,
              "a trajectory out of form, unchanged");
    }
    maniTrajectoryGoal goals[4] = {g, g, g, g};
    goals[0].rotation = (maniQuat){0, 0, 0, 0};
    goals[1].positionHalfLife = -1.0f;
    goals[2].rotationHalfLife = NAN;
    goals[3].velocity.z = INFINITY;
    for (int k = 0; k < 4; ++k)
    {
        CHECK(maniAdvanceTrajectory(&t, &goals[k], 0.1f) == mani_errorInvalid,
              "a goal out of form");
    }
    CHECK(maniAdvanceTrajectory(&t, &g, -0.1f) == mani_errorInvalid &&
              maniAdvanceTrajectory(&t, &g, NAN) == mani_errorInvalid &&
              maniAdvanceTrajectory(NULL, &g, 0.1f) == mani_errorInvalid &&
              maniAdvanceTrajectory(&t, NULL, 0.1f) == mani_errorInvalid,
          "seconds out of range, NULL");
    float out[2];
    CHECK(maniTrajectoryFeatures(NULL, &t, &t, out, out) == mani_errorInvalid &&
              maniTrajectoryFeatures(&s_database, NULL, &t, out, out) == mani_errorInvalid &&
              maniTrajectoryFeatures(&s_database, &t, NULL, out, out) == mani_errorInvalid &&
              maniTrajectoryFeatures(&s_database, &t, &t, NULL, NULL) == mani_success,
          "features' NULL arguments");
}

// The walk's root half a second ahead: 0.5 along x, as the database
// measured it; turned with the root.
static void TestFeatures(void)
{
    float features[MANI_MAX_MATCH_FEATURES];
    float offsets[MANI_MAX_MATCH_FEATURES];
    float scales[MANI_MAX_MATCH_FEATURES];
    CHECK(maniMotionDatabaseFeatures(&s_database, 3, features) == mani_success &&
              maniMotionDatabaseNormalization(&s_database, offsets, scales) == mani_success,
          "the walk's fourth frame");
    maniTrajectory root = {{2, 0, 3}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}};
    maniTrajectory ahead = root;
    ahead.position.x += 0.5f;
    float position[2];
    float facing[2];
    CHECK(maniTrajectoryFeatures(&s_database, &root, &ahead, position, facing) == mani_success &&
              NearF(position[0], (double)features[0] * (double)scales[0] + (double)offsets[0]) &&
              NearF(position[1], (double)features[1] * (double)scales[1] + (double)offsets[1]) &&
              NearF(facing[0], 0.0) && NearF(facing[1], 1.0),
          "the database's own values");
    maniQuat quarter = maniQuatFromAxisAngle((maniVec3){0, 1, 0}, 1.5707963f);
    root.rotation = quarter;
    ahead = root;
    // A quarter turn about +y takes +z to +x: forward is now +x.
    ahead.position = maniVec3Add(root.position, (maniVec3){0.5f, 0.0f, 0.0f});
    CHECK(maniTrajectoryFeatures(&s_database, &root, &ahead, position, facing) == mani_success &&
              fabs((double)position[0]) <= 1e-6 && fabs((double)position[1] - 0.5) <= 1e-6 &&
              fabs((double)facing[0]) <= 1e-6 && fabs((double)facing[1] - 1.0) <= 1e-6,
          "turned with the root: half a second straight ahead");
}

// A database of up z and forward -x: the ground plane x and y, forward
// -x the facing of no turn.
static _Alignas(16) uint8_t s_zFile[1 << 15];

static void TestAxes(void)
{
    static const maniMatchChannelSource kChannel = {
        maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f};
    const maniClip* clips[1] = {&s_clips[0]};
    const maniMatchEntrySource entry = {0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 0u};
    maniMotionDatabaseSource source = {
        &s_skeleton, clips, &kChannel, &entry,         1,
        1,           1,     10.0f,     maniMatchAxisZ, maniMatchAxisNegativeX,
        NULL,        0};
    maniMotionDatabase d;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_zFile, sizeof s_zFile, &size) == mani_success &&
              maniOpenMotionDatabase(s_zFile, size, &d).result == mani_success,
          "a database of up z");
    maniDestroyBuilder(builder);
    maniTrajectory root = {{1, 2, 3}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0}};
    maniTrajectory ahead = root;
    ahead.position = (maniVec3){1.5f, 2.25f, 12.0f};
    float position[2];
    float facing[2];
    CHECK(maniTrajectoryFeatures(&d, &root, &ahead, position, facing) == mani_success &&
              position[0] == 0.5f && position[1] == 0.25f && facing[0] == -1.0f &&
              facing[1] == 0.0f,
          "on the ground of up z, facing -x");
}

int main(void)
{
    Open();
    OpenDatabase();
    TestHolden();
    TestSettles();
    TestRefusals();
    TestFeatures();
    TestAxes();
    return s_failures == 0 ? 0 : 1;
}
