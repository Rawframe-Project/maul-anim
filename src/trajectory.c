// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Holden's springs (Motion-Matching's controller and spring.h): with
// y = 2 ln 2 / (h + 1e-5), j0 = v - v*, j1 = a + j0 y and e = e^(-y t),
// the position moves by e (-j1 / y^2 + (-j0 - j1 t) / y) + j1 / y^2 +
// j0 / y + v* t, the velocity becomes e (j0 + j1 t) + v* and the
// acceleration e (a - j1 y t); the rotation's offset from its goal as a
// scaled axis-angle on the shorter arc, j0, with j1 = w + j0 y, becomes
// e (j0 + j1 t) and the angular velocity e (w - j1 y t). The exponential
// is the library's, so the bits are the same everywhere. A sample's
// values are its offset and turned forward axis in the root's frame,
// projected on the database's ground plane as its sampling projects
// them.

#include "maul-anim/trajectory.h"

#include "elementary_float.h"
#include "fpenv.h"
#include "match_move.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/match.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#define LN2 0.693147180559945309f

static bool Finite3(maniVec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool Finite4(maniQuat q)
{
    return isfinite(q.x) && isfinite(q.y) && isfinite(q.z) && isfinite(q.w);
}

static bool Unit(maniQuat q)
{
    return Finite4(q) && fabsf(maniQuatDot(q, q) - 1.0f) <= 2e-3f;
}

static bool Valid(const maniTrajectory* t, const maniTrajectoryGoal* g, float seconds)
{
    return Finite3(t->position) && Finite3(t->velocity) && Finite3(t->acceleration) &&
           Unit(t->rotation) && Finite3(t->angularVelocity) && Finite3(g->velocity) &&
           Finite4(g->rotation) && maniQuatDot(g->rotation, g->rotation) > 0.0f &&
           isfinite(g->positionHalfLife) && g->positionHalfLife >= 0.0f &&
           isfinite(g->rotationHalfLife) && g->rotationHalfLife >= 0.0f && isfinite(seconds) &&
           seconds >= 0.0f;
}

// Half the damping of a half-life: 2 ln 2 / (h + eps), Holden's.
static float Rate(float halfLife)
{
    return 2.0f * LN2 / (halfLife + 1e-5f);
}

static maniVec3 Mix(maniVec3 a, float sa, maniVec3 b, float sb)
{
    return maniVec3Add(maniVec3Scale(a, sa), maniVec3Scale(b, sb));
}

static void Positions(maniTrajectory* t, const maniTrajectoryGoal* g, float dt)
{
    float y = Rate(g->positionHalfLife);
    maniVec3 j0 = maniVec3Sub(t->velocity, g->velocity);
    maniVec3 j1 = Mix(t->acceleration, 1.0f, j0, y);
    float e = maniExpF(-y * dt);
    float yy = y * y;
    maniVec3 moved = Mix(maniVec3Scale(j1, -1.0f / yy), e, Mix(j0, -1.0f, j1, -dt), e / y);
    moved = maniVec3Add(moved, Mix(j1, 1.0f / yy, j0, 1.0f / y));
    t->position = maniVec3Add(t->position, maniVec3Add(moved, maniVec3Scale(g->velocity, dt)));
    t->velocity = maniVec3Add(maniVec3Scale(Mix(j0, 1.0f, j1, dt), e), g->velocity);
    t->acceleration = maniVec3Scale(Mix(t->acceleration, 1.0f, j1, -y * dt), e);
}

static void Rotations(maniTrajectory* t, const maniTrajectoryGoal* g, float dt)
{
    float y = Rate(g->rotationHalfLife);
    maniQuat goal = maniQuatNormalize(g->rotation);
    maniVec3 j0 = maniQuatToRotationVector(maniQuatMul(t->rotation, maniQuatConjugate(goal)));
    maniVec3 j1 = Mix(t->angularVelocity, 1.0f, j0, y);
    float e = maniExpF(-y * dt);
    t->rotation = maniQuatNormalize(
        maniQuatMul(maniQuatFromRotationVector(maniVec3Scale(Mix(j0, 1.0f, j1, dt), e)), goal));
    t->angularVelocity = maniVec3Scale(Mix(t->angularVelocity, 1.0f, j1, -y * dt), e);
}

static MANI_NOINLINE maniResult AdvanceTrajectoryWork(maniTrajectory* trajectory,
                                                      const maniTrajectoryGoal* goal, float seconds)
{
    if (trajectory == NULL || goal == NULL || !Valid(trajectory, goal, seconds))
    {
        return mani_errorInvalid;
    }
    Positions(trajectory, goal, seconds);
    Rotations(trajectory, goal, seconds);
    return mani_success;
}

maniResult maniAdvanceTrajectory(maniTrajectory* trajectory, const maniTrajectoryGoal* goal,
                                 float seconds)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = AdvanceTrajectoryWork(trajectory, goal, seconds);
    maniFloatEnvLeave(env);
    return result;
}

static MANI_NOINLINE maniResult TrajectoryFeaturesWork(const maniMotionDatabase* database,
                                                       const maniTrajectory* root,
                                                       const maniTrajectory* sample,
                                                       float* positionOut, float* facingOut)
{
    if (database == NULL || root == NULL || sample == NULL)
    {
        return mani_errorInvalid;
    }
    maniQuat inverse = maniQuatConjugate(root->rotation);
    if (positionOut != NULL)
    {
        maniVec3 offset = maniVec3Sub(sample->position, root->position);
        maniMatchGround(maniQuatRotate(inverse, offset), database->up, positionOut);
    }
    if (facingOut != NULL)
    {
        maniVec3 forward = maniQuatRotate(sample->rotation, maniMatchAxisVector(database->forward));
        maniMatchGround(maniQuatRotate(inverse, forward), database->up, facingOut);
    }
    return mani_success;
}

maniResult maniMotionDatabase_ComputeTrajectoryFeatures(const maniMotionDatabase* database,
                                                        const maniTrajectory* root,
                                                        const maniTrajectory* sample,
                                                        float* positionOut, float* facingOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = TrajectoryFeaturesWork(database, root, sample, positionOut, facingOut);
    maniFloatEnvLeave(env);
    return result;
}
