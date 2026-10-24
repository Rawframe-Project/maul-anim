// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Cyclic coordinate descent, Wang and Chen's: each iteration
// sweeps from the end's parent to the root, each bone turning the rest
// of the chain by the shortest arc that takes its direction to the end
// onto its direction to the goal, cut to the step bound, then limited;
// the frames below the bone are recomposed before the next. Stops once
// the end lies within the tolerance.

#include "fpenv.h"
#include "ik_chain.h"
#include "ik_frame.h"
#include "scalar.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The shortest arc from a onto b, cut to step radians (0 for no cut);
// the arc's w is the cosine of half its angle, at least 0.
static maniQuat Arc(maniVec3 a, maniVec3 b, float step)
{
    maniQuat turn = maniQuatBetween(a, b);
    if (step > 0.0f)
    {
        maniVec3 v = {turn.x, turn.y, turn.z};
        float angle = 2.0f * maniAtan2(maniVec3Length(v), turn.w);
        turn = angle > step ? maniQuatSlerp(maniQuatIdentity(), turn, step / angle) : turn;
    }
    return turn;
}

static MANI_NOINLINE maniResult SolveCcdWork(maniPose* pose, const maniSkeleton* skeleton,
                                             const maniChainSolve* solve, bool* reachedOut)
{
    if (pose == NULL || skeleton == NULL || solve == NULL ||
        !maniIkChainValid(pose, skeleton, solve, false, true))
    {
        return mani_errorInvalid;
    }
    maniIkChain chain;
    maniIkChainBegin(&chain, pose, skeleton, solve);
    for (uint32_t i = 0; i < solve->iterations; ++i)
    {
        if (maniVec3Length(maniVec3Sub(maniIkChainEnd(&chain), solve->goal)) <= solve->tolerance)
        {
            break;
        }
        for (uint32_t k = chain.count - 1; k-- > 0;)
        {
            maniVec3 head = maniIkOrigin(&chain.frames[k]);
            maniVec3 toEnd = maniVec3Sub(maniIkChainEnd(&chain), head);
            maniVec3 toGoal = maniVec3Sub(solve->goal, head);
            if (!(maniVec3Dot(toEnd, toEnd) > 0.0f) || !(maniVec3Dot(toGoal, toGoal) > 0.0f))
            {
                continue;
            }
            maniIkChainTurn(&chain, pose, skeleton, solve, k, Arc(toEnd, toGoal, solve->step));
            maniIkChainFrames(&chain, pose, k);
        }
    }
    bool reached = maniIkChainFinish(&chain, pose, solve);
    if (reachedOut != NULL)
    {
        *reachedOut = reached;
    }
    return mani_success;
}

maniResult maniSolveCcd(maniPose* pose, const maniSkeleton* skeleton, const maniChainSolve* solve,
                        bool* reachedOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = SolveCcdWork(pose, skeleton, solve, reachedOut);
    maniFloatEnvLeave(env);
    return result;
}
