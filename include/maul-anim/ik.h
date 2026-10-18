// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inverse kinematics on a pose (mani-0029): a two-bone chain bent to
// reach a goal and turned toward a pole; a bone aimed at a goal, or a
// chain of up to MANI_AIM_MAX_BONES sharing the turn (a look-at over
// spine, neck and head), its up toward a pole. A solver reads the pose's
// locals, composes the model frames of its bones' root path itself (the
// pose's model need not be current), writes the corrected locals of the
// bones it turns and leaves the model stale. Goals and poles are in the
// model space of the pose; axes fixed to a bone are in that bone's local
// frame. Stateless: the result is a function of the pose and the inputs.

#ifndef MAUL_ANIM_IK_H
#define MAUL_ANIM_IK_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A two-bone solve: the start, middle and end bones, each an ancestor
    // of the next (not necessarily its parent).
    typedef struct maniTwoBoneSolve
    {
        uint32_t start;
        uint32_t middle;
        uint32_t end;
        uint32_t reserved;
        // Where the end bone's origin should be, in model space.
        maniVec3 goal;
        // A model-space direction the chain's bend turns toward, from the
        // start; zero for none (the chain keeps its bend plane). Near the
        // start-to-goal line the plane blends to the chain's own, so it
        // never flips.
        maniVec3 pole;
        // The middle bone's hinge in its local frame, a positive turn
        // about it opening the chain; zero for the chain's own bend
        // plane.
        maniVec3 hinge;
        // A turn of the bend plane about the start-to-goal line, radians.
        float twist;
        // In [0, 1]: the share of the chain's length past which a goal
        // is reached only gradually (ozz's soften); 1 for none.
        float soften;
        // In [0, 1]: the correction's share, 1 for all of it.
        float weight;
    } maniTwoBoneSolve;

    /// Bends a two-bone chain of a pose toward a goal: the middle bone
    /// turns about its hinge so the chain spans the goal's distance (a
    /// goal out of reach straightens it, one closer than the bones'
    /// difference folds it), then the start bone turns the chain onto the
    /// goal, its bend toward the pole, twisted. A goal at the start turns
    /// nothing at the start; a bone of no length turns nothing at the
    /// middle. The solve on its own result changes nothing.
    ///
    /// @param pose       A pose of the skeleton (not a difference); the
    ///                   start and middle bones' locals are written.
    /// @param skeleton   Its skeleton.
    /// @param solve      The solve.
    /// @param reachedOut Receives whether the goal is reached at full
    ///                   weight (within the softened reach and further
    ///                   than the bones' difference); may be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL pose,
    /// skeleton or solve, a pose of another skeleton or a difference,
    /// bones out of range or not each an ancestor of the next, a goal,
    /// pole, hinge or twist not finite, a soften or weight outside [0, 1].
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSolveTwoBone(maniPose* pose,
                                                        const maniSkeleton* skeleton,
                                                        const maniTwoBoneSolve* solve,
                                                        bool* reachedOut);

// The most bones an aim spreads its turn over.
#define MANI_AIM_MAX_BONES 8u
// The widest cone an aim takes, pi radians.
#define MANI_AIM_MAX_CONE 3.14159265358979f

    // An aim: count bones, an ancestor first and the aiming bone last,
    // each an ancestor of the next; one for an aim at a single bone.
    typedef struct maniAimSolve
    {
        uint32_t bones[MANI_AIM_MAX_BONES];
        // Each bone's share of the turn, at least 0 and not all 0: each,
        // root first, turns its share of what is still needed among the
        // bones from it down (the last all of it).
        float shares[MANI_AIM_MAX_BONES];
        uint32_t count;
        uint32_t reserved;
        // Where to aim, in model space.
        maniVec3 goal;
        // A model-space direction the up axis turns toward, from the
        // aiming bone; zero for none. Near the aim line its plane blends
        // to the bone's own, so it never flips.
        maniVec3 pole;
        // In the aiming bone's local frame: the unit direction that aims,
        // the point it aims from (relative to the bone's origin), and the
        // axis kept toward the pole (zero for none).
        maniVec3 forward;
        maniVec3 offset;
        maniVec3 up;
        // A turn about the aim line, radians.
        float twist;
        // In (0, MANI_AIM_MAX_CONE]: the largest turn of the forward
        // from the input pose's; 0 for none.
        float cone;
        // In [0, 1]: the corrections' share, 1 for all of them.
        float weight;
    } maniAimSolve;

    /// Aims a bone of a pose at a goal: its forward from its offset point
    /// turned onto the goal by the shortest arc, then about that line so
    /// its up axis lies in the plane of the line and the pole, then by
    /// the twist; with more bones each turns its share. A cone stops the
    /// forward at its edge. A goal at the aiming bone, or nearer to it
    /// than the offset lies from the forward line, turns nothing.
    ///
    /// @param pose       A pose of the skeleton (not a difference); the
    ///                   aim's bones' locals are written.
    /// @param skeleton   Its skeleton.
    /// @param solve      The aim.
    /// @param reachedOut Receives whether the forward points at the goal
    ///                   at full weight (not stopped by the cone, the goal
    ///                   reachable); may be NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL pose,
    /// skeleton or solve, a pose of another skeleton or a difference, a
    /// count of 0 or past MANI_AIM_MAX_BONES, bones out of range or not
    /// each an ancestor of the next, shares negative, not finite or all
    /// 0, a forward not of length 1 within a thousandth, a goal, pole,
    /// offset, up or twist not finite, a cone outside [0,
    /// MANI_AIM_MAX_CONE], a weight
    /// outside [0, 1].
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSolveAim(maniPose* pose, const maniSkeleton* skeleton,
                                                    const maniAimSolve* solve, bool* reachedOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_IK_H
