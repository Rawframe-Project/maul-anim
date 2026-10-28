// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the chain solvers share. A bone's limit applies to
// its local rotation relative to its rest local: the rest's rotation
// then a turn r, r limited and the local rebuilt from it.

#include "ik_chain.h"

#include "ik_frame.h"
#include "ik_limit.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PI_F 3.14159265358979f

uint32_t maniIkChainPath(const maniSkeleton* skeleton, uint32_t root, uint32_t end, uint32_t* bones)
{
    uint32_t reversed[MANI_MAX_IK_CHAIN_BONES];
    uint32_t n = 0;
    for (uint32_t b = end; b != MANI_NO_BONE; b = maniSkeletonParent(skeleton, b))
    {
        if (n == MANI_MAX_IK_CHAIN_BONES)
        {
            return 0;
        }
        reversed[n++] = b;
        if (b == root)
        {
            for (uint32_t k = 0; k < n; ++k)
            {
                bones[k] = reversed[n - 1 - k];
            }
            return n > 1 ? n : 0;
        }
    }
    return 0;
}

bool maniIkChainPathPast(const maniSkeleton* skeleton, uint32_t root, uint32_t end)
{
    uint32_t n = 0;
    for (uint32_t b = end; b != MANI_NO_BONE; b = maniSkeletonParent(skeleton, b))
    {
        ++n;
        if (b == root)
        {
            return n > MANI_MAX_IK_CHAIN_BONES;
        }
    }
    return false;
}

static bool Unit(float v)
{
    return v >= 0.0f && v <= 1.0f;
}

bool maniIkChainLimitsValid(const maniSkeleton* skeleton, const uint32_t* bones, uint32_t count,
                            const maniJointLimit* limits)
{
    for (uint32_t k = 0; k + 1 < count; ++k)
    {
        const maniJointLimit* limit = &limits[k];
        maniVec3 axis = maniSkeletonRest(skeleton, bones[k + 1]).translation;
        float along = maniVec3Length(axis);
        // A hinge along the bone's own axis is no hinge.
        bool parallel = limit->kind == maniJointLimitHinge && along > 0.0f &&
                        !(maniVec3Length(maniVec3Cross(limit->axis, axis)) >= 1e-3f * along);
        if (!maniIkLimitValid(limit) || parallel)
        {
            return false;
        }
    }
    return true;
}

maniResult maniIkChainCheck(const maniPose* pose, const maniSkeleton* skeleton,
                            const maniChainSolve* solve, bool poles, bool steps)
{
    uint32_t bones[MANI_MAX_IK_CHAIN_BONES];
    if (pose == NULL || skeleton == NULL || solve == NULL || !maniIkPoseUsable(pose, skeleton) ||
        solve->root >= skeleton->boneCount || solve->end >= skeleton->boneCount)
    {
        return mani_errorInvalid;
    }
    uint32_t count = maniIkChainPath(skeleton, solve->root, solve->end, bones);
    if (solve->iterations > MANI_MAX_IK_ITERATIONS ||
        (count == 0 && maniIkChainPathPast(skeleton, solve->root, solve->end)))
    {
        return mani_errorLimit;
    }
    if (solve->limits != NULL && !maniIkChainLimitsValid(skeleton, bones, count, solve->limits))
    {
        return mani_errorInvalid;
    }
    maniQuat q = solve->rotation;
    float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    bool noPole = solve->pole.x == 0.0f && solve->pole.y == 0.0f && solve->pole.z == 0.0f;
    bool valid = count > 0 && maniIkFiniteVec(solve->goal) && maniIkFiniteVec(solve->pole) &&
                 (poles || noPole) && length > 0.0f && isfinite(length) && solve->iterations >= 1 &&
                 solve->tolerance >= 0.0f && isfinite(solve->tolerance) && solve->step >= 0.0f &&
                 solve->step <= PI_F && (steps || solve->step == 0.0f) &&
                 Unit(solve->rotationWeight) && Unit(solve->weight);
    return valid ? mani_success : mani_errorInvalid;
}

void maniIkChainBegin(maniIkChain* chain, const maniPose* pose, const maniSkeleton* skeleton,
                      const maniChainSolve* solve)
{
    chain->count = maniIkChainPath(skeleton, solve->root, solve->end, chain->bones);
    chain->top = maniIkParentModel(pose, skeleton, solve->root);
    for (uint32_t k = 0; k < chain->count; ++k)
    {
        chain->before[k] = maniPoseLocal(pose, chain->bones[k]);
        maniVec3 next = k + 1 < chain->count
                            ? maniSkeletonRest(skeleton, chain->bones[k + 1]).translation
                            : maniVec3Make(0.0f, 0.0f, 0.0f);
        chain->axes[k] = maniVec3Normalize(next, maniVec3Make(0.0f, 0.0f, 0.0f));
    }
    maniIkChainFrames(chain, pose, 0);
}

void maniIkChainFrames(maniIkChain* chain, const maniPose* pose, uint32_t from)
{
    for (uint32_t k = from; k < chain->count; ++k)
    {
        maniIkFrame local = maniIkLocal(pose, chain->bones[k]);
        const maniIkFrame* parent = k == 0 ? &chain->top : &chain->frames[k - 1];
        chain->frames[k] = maniIkMul(parent, &local);
    }
}

void maniIkChainTurn(maniIkChain* chain, maniPose* pose, const maniSkeleton* skeleton,
                     const maniChainSolve* solve, uint32_t k, maniQuat turn)
{
    uint32_t bone = chain->bones[k];
    const maniIkFrame* parent = k == 0 ? &chain->top : &chain->frames[k - 1];
    maniIkCorrect(pose, bone, maniIkInParent(turn, parent));
    if (solve->limits == NULL || solve->limits[k].kind == maniJointLimitNone)
    {
        return;
    }
    maniTransform t = maniPoseLocal(pose, bone);
    maniQuat rest = maniSkeletonRest(skeleton, bone).rotation;
    maniQuat r = maniQuatNormalize(maniQuatMul(maniQuatConjugate(rest), t.rotation));
    t.rotation =
        maniQuatNormalize(maniQuatMul(rest, maniIkLimit(&solve->limits[k], chain->axes[k], r)));
    maniPoseSetLocal(pose, bone, t);
}

maniVec3 maniIkChainEnd(const maniIkChain* chain)
{
    return maniIkOrigin(&chain->frames[chain->count - 1]);
}

bool maniIkChainFinish(maniIkChain* chain, maniPose* pose, const maniChainSolve* solve)
{
    maniVec3 miss = maniVec3Sub(maniIkChainEnd(chain), solve->goal);
    bool reached = maniVec3Length(miss) <= solve->tolerance;
    uint32_t last = chain->count - 1;
    if (solve->rotationWeight > 0.0f)
    {
        maniQuat now = maniIkRotation(&chain->frames[last]);
        maniQuat wanted = maniQuatNormalize(solve->rotation);
        maniQuat turn = maniQuatMul(wanted, maniQuatConjugate(now));
        turn = maniIkWeighted(turn, solve->rotationWeight);
        maniIkCorrect(pose, chain->bones[last], maniIkInParent(turn, &chain->frames[last - 1]));
    }
    if (solve->weight < 1.0f)
    {
        // Each correction weighted on the input pose.
        for (uint32_t k = 0; k < chain->count; ++k)
        {
            maniQuat now = maniPoseLocal(pose, chain->bones[k]).rotation;
            maniQuat correction = maniQuatMul(now, maniQuatConjugate(chain->before[k].rotation));
            maniPoseSetLocal(pose, chain->bones[k], chain->before[k]);
            maniIkCorrect(pose, chain->bones[k], maniIkWeighted(correction, solve->weight));
        }
    }
    return reached && solve->weight >= 1.0f;
}
