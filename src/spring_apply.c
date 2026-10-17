// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Turning spring joints toward their tails (mani-0026). Each spring's
// tails are taken between the last two steps, at the time the last call
// ran past the last step (so the motion is smooth whatever the host's
// rate, a step behind), and carried into the last call's placement.
// Joints turn from the root of each chain: a joint's turn in model space
// q, with its model rotation R, is its local rotation times R* q R, so
// the parent's rotation need not be known; the joints below turn with it
// by the rigid motion carried down the chain, as the steps do.

#include "bytes.h"
#include "spring_file.h"
#include "spring_state.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stdint.h>

static const maniVec3 kUp = {0.0f, 1.0f, 0.0f};

static maniVec3 Lerp(maniVec3 a, maniVec3 b, float f)
{
    return maniVec3Add(a, maniVec3Scale(maniVec3Sub(b, a), f));
}

// Model space at the step's placement carried into the call's.
typedef struct Carry
{
    maniQuat rotation;
    maniVec3 translation;
} Carry;

static Carry CarryOf(const uint32_t* block)
{
    maniVec3 sp = maniSpringVec(block, maniSpringStepPlacement);
    maniQuat sr = maniSpringQuat(block, maniSpringStepPlacement + 3);
    maniVec3 cp = maniSpringVec(block, maniSpringPlacement);
    maniQuat back = maniQuatConjugate(maniSpringQuat(block, maniSpringPlacement + 3));
    return (Carry){maniQuatNormalize(maniQuatMul(back, sr)),
                   maniQuatRotate(back, maniVec3Sub(sp, cp))};
}

static maniVec3 PosePosition(const maniPose* pose, uint32_t bone)
{
    maniAffine m = maniPoseModel(pose, bone);
    return maniAffineTransformPoint(&m, maniVec3Make(0.0f, 0.0f, 0.0f));
}

// The chain's rigid motion so far: x' = rotation x + translation.
typedef struct Chain
{
    maniQuat rotation;
    maniVec3 translation;
} Chain;

static maniVec3 Move(const Chain* c, maniVec3 x)
{
    return maniVec3Add(maniQuatRotate(c->rotation, x), c->translation);
}

static void ApplySpring(const maniSpringSet* set, const uint32_t* block, maniPose* pose,
                        uint32_t spring, float alpha)
{
    const uint8_t* e = set->springs + (size_t)spring * MANI_SPRING_SPRING_SIZE;
    uint32_t center = maniLoadU32(e + 16);
    maniAffine space = maniPoseModel(pose, center);
    Carry carry = CarryOf(block);
    Chain chain = {maniQuatIdentity(), maniVec3Make(0.0f, 0.0f, 0.0f)};
    uint32_t first = maniLoadU32(e);
    for (uint32_t j = first; j < first + maniLoadU32(e + 4); ++j)
    {
        const uint8_t* r = set->joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
        uint32_t bone = maniLoadU32(r + maniSpringJointHead);
        maniTransform head = maniPoseModelTransform(pose, bone);
        maniVec3 p = Move(&chain, head.translation);
        maniQuat rotation = maniQuatNormalize(maniQuatMul(chain.rotation, head.rotation));
        maniVec3 tail = Move(&chain, PosePosition(pose, maniLoadU32(r + maniSpringJointTail)));
        maniVec3 from = maniVec3Normalize(maniVec3Sub(tail, p), kUp);
        uint64_t at = maniSpringJointAt(j);
        maniVec3 sim = Lerp(maniSpringVec(block, at + maniSpringTailBefore),
                            maniSpringVec(block, at + maniSpringTailNow), alpha);
        sim = center != MANI_NO_BONE
                  ? maniAffineTransformPoint(&space, sim)
                  : maniVec3Add(maniQuatRotate(carry.rotation, sim), carry.translation);
        maniQuat turn = maniQuatBetween(from, maniVec3Normalize(maniVec3Sub(sim, p), from));
        maniTransform local = maniPoseLocal(pose, bone);
        maniQuat inBone = maniQuatMul(maniQuatMul(maniQuatConjugate(rotation), turn), rotation);
        local.rotation = maniQuatNormalize(maniQuatMul(local.rotation, inBone));
        maniPoseSetLocal(pose, bone, local);
        maniVec3 moved = maniVec3Add(maniQuatRotate(turn, maniVec3Sub(chain.translation, p)), p);
        chain = (Chain){maniQuatNormalize(maniQuatMul(turn, chain.rotation)), moved};
    }
}

void maniSpringApplyBlock(const maniSpringSet* set, const uint32_t* block, maniPose* pose)
{
    if (block[maniSpringHas] == 0)
    {
        return;
    }
    float alpha = (float)(maniSpringClockOf(block) * (double)set->stepRate);
    alpha = alpha < 1.0f ? alpha : 1.0f;
    for (uint32_t i = 0; i < set->springCount; ++i)
    {
        ApplySpring(set, block, pose, i, alpha);
    }
}
