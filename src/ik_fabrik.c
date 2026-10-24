// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FABRIK, Aristidou and Lasenby's: each iteration places the
// end on the goal and pulls each joint back to its bone's length from
// the one below it (backward), then from the root turns each bone by
// the shortest arc from its direction onto the one toward its joint's
// backward place, limits it and recomposes the frames below, so the next
// joint stands where the turned bone puts it (forward). Twist is kept,
// as each turn is a shortest arc; a pole turns the whole chain about the
// root-to-goal line first. Stops once the end lies within the tolerance.

#include "maul-anim/ik.h"

#include "fpenv.h"
#include "ik_chain.h"
#include "ik_frame.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The chain turned about the root-to-goal line so its bend (its middle
// joints' offsets from that line) faces the pole, blended near the line
// as the two-bone solve's; nothing without a pole or a bend.
static void Pole(maniIkChain* chain, maniPose* pose, const maniSkeleton* skeleton,
                 const maniChainSolve* solve)
{
    maniVec3 root = maniIkOrigin(&chain->frames[0]);
    maniVec3 u = maniVec3Sub(solve->goal, root);
    float length = maniVec3Length(u);
    if (!(length > 0.0f) || maniVec3Dot(solve->pole, solve->pole) == 0.0f)
    {
        return;
    }
    u = maniVec3Scale(u, 1.0f / length);
    maniVec3 bend = maniVec3Make(0.0f, 0.0f, 0.0f);
    for (uint32_t k = 1; k + 1 < chain->count; ++k)
    {
        bend = maniVec3Add(bend, maniVec3Sub(maniIkOrigin(&chain->frames[k]), root));
    }
    maniQuat turn = maniIkPlane(u, maniVec3Cross(u, bend), solve->pole);
    maniIkChainTurn(chain, pose, skeleton, solve, 0, turn);
    maniIkChainFrames(chain, pose, 0);
}

// The backward pass: places[k] each joint's place, the end's the goal.
static void Backward(const maniIkChain* chain, const float* lengths, maniVec3 goal,
                     maniVec3* places)
{
    uint32_t last = chain->count - 1;
    places[last] = goal;
    for (uint32_t k = last; k-- > 0;)
    {
        maniVec3 from = maniVec3Sub(maniIkOrigin(&chain->frames[k]), places[k + 1]);
        maniVec3 d = maniVec3Normalize(from, maniVec3Make(0.0f, 0.0f, 0.0f));
        places[k] = maniVec3Add(places[k + 1], maniVec3Scale(d, lengths[k]));
    }
}

static MANI_NOINLINE maniResult SolveFabrikWork(maniPose* pose, const maniSkeleton* skeleton,
                                                const maniChainSolve* solve, bool* reachedOut)
{
    if (pose == NULL || skeleton == NULL || solve == NULL ||
        !maniIkChainValid(pose, skeleton, solve, true, false))
    {
        return mani_errorInvalid;
    }
    maniIkChain chain;
    maniIkChainBegin(&chain, pose, skeleton, solve);
    float lengths[MANI_IK_CHAIN_MAX_BONES] = {0};
    for (uint32_t k = 0; k + 1 < chain.count; ++k)
    {
        lengths[k] = maniVec3Length(
            maniVec3Sub(maniIkOrigin(&chain.frames[k + 1]), maniIkOrigin(&chain.frames[k])));
    }
    Pole(&chain, pose, skeleton, solve);
    maniVec3 places[MANI_IK_CHAIN_MAX_BONES] = {{0}};
    for (uint32_t i = 0; i < solve->iterations; ++i)
    {
        if (maniVec3Length(maniVec3Sub(maniIkChainEnd(&chain), solve->goal)) <= solve->tolerance)
        {
            break;
        }
        Backward(&chain, lengths, solve->goal, places);
        for (uint32_t k = 0; k + 1 < chain.count; ++k)
        {
            maniVec3 head = maniIkOrigin(&chain.frames[k]);
            maniVec3 now = maniVec3Sub(maniIkOrigin(&chain.frames[k + 1]), head);
            maniVec3 wanted = maniVec3Sub(places[k + 1], head);
            if (!(maniVec3Dot(now, now) > 0.0f) || !(maniVec3Dot(wanted, wanted) > 0.0f))
            {
                continue;
            }
            maniIkChainTurn(&chain, pose, skeleton, solve, k, maniQuatBetween(now, wanted));
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

maniResult maniSolveFabrik(maniPose* pose, const maniSkeleton* skeleton,
                           const maniChainSolve* solve, bool* reachedOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = SolveFabrikWork(pose, skeleton, solve, reachedOut);
    maniFloatEnvLeave(&env);
    return result;
}
