// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inverse kinematics on a pose (mani-0029): a two-bone chain bent to
// reach a goal and turned toward a pole. A solver reads the pose's
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

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_IK_H
