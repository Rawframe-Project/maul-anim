// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A character's trajectory predicted for motion matching:
// Holden's critically damped springs, in closed form, take a simulated
// root toward a desired velocity and rotation; its samples, predicted or
// recorded, become the query values of a database's trajectory channels.

#ifndef MAUL_ANIM_TRAJECTORY_H
#define MAUL_ANIM_TRAJECTORY_H

#include "maul-anim/base.h"
#include "maul-anim/match.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // A simulated root: its position, velocity and acceleration, and its
    // rotation (unit) and angular velocity (radians a second about an
    // axis, scaled), in the world.
    typedef struct maniTrajectory
    {
        maniVec3 position;
        maniVec3 velocity;
        maniVec3 acceleration;
        maniQuat rotation;
        maniVec3 angularVelocity;
    } maniTrajectory;

    // Where a trajectory is going: a desired velocity and rotation, and
    // the seconds in which each spring halves its distance to them
    // (Holden's 0.27 his defaults).
    typedef struct maniTrajectoryGoal
    {
        maniVec3 velocity;
        maniQuat rotation;
        float positionHalfLife;
        float rotationHalfLife;
    } maniTrajectoryGoal;

    /// Moves a trajectory by seconds toward its goal, each spring in
    /// closed form (y = 2 ln 2 / (half-life + 1e-5); the position's with
    /// its acceleration, Holden's simulation_positions_update; the
    /// rotation's on the shorter arc, his simple_spring_damper_exact), in
    /// binary32: one advance is many shorter ones up to rounding, so a
    /// prediction t seconds ahead is a copy advanced by t.
    ///
    /// @param trajectory  The trajectory; finite, its rotation unit
    ///                    within 1e-3.
    /// @param goal        Finite, its rotation not zero (normalized),
    ///                    its half-lives not negative.
    /// @param seconds     Finite, not negative.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// one out of range (the trajectory then unchanged).
    /// @par Thread safety
    /// Safe from any thread; the trajectory is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniAdvanceTrajectory(maniTrajectory* trajectory,
                                                             const maniTrajectoryGoal* goal,
                                                             float seconds);

    /// Gives a sample's trajectory values as a database measures its
    /// trajectories: the sample's offset from the root and the
    /// database's forward axis turned by the sample's rotation, both in
    /// the root's frame, on the ground plane (the two axes besides the
    /// database's up axis, in order); the values of a trajectory
    /// position and a trajectory facing query parameter.
    ///
    /// @param database     An opened database.
    /// @param root         The character's root now (its position and
    ///                     rotation; the rest unread).
    /// @param sample       A sample predicted ahead or recorded behind.
    /// @param positionOut  Receives 2 values, or NULL.
    /// @param facingOut    Receives 2 values, or NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL database,
    /// root or sample.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMotionDatabase_ComputeTrajectoryFeatures(
        const maniMotionDatabase* database, const maniTrajectory* root,
        const maniTrajectory* sample, float* positionOut, float* facingOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_TRAJECTORY_H
