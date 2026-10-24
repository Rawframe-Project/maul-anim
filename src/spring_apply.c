// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Turning spring joints toward their tails. Each spring's
// tails are the last step's moved on at their velocity (the last two
// steps') for the time the last call ran past it, and carried into the
// last call's placement: the last step's own at a host rate the step
// rate divides, so the output does not lag the steps, and continuous
// to the second difference across a step at faster hosts. Between
// steps the tails go on with their heads, which the pose has moved.
// Each tail is put at its bone's length toward that point and pushed
// out of its spring's colliders as they stand at the last call's end,
// as a step pushes, so a tail moved on does not cross one; the state is
// only read.
// Joints turn from the root of each chain: a joint's turn in model space
// q, with its parent's model rotation P (turned with the chain), turns
// its local rotation by P* q P, mirrored when the parent's scale is a
// mirror; the joints below turn with it by the rigid motion carried down
// the chain, as the steps do.

#include "bytes.h"
#include "pose_memory.h"
#include "spring_file.h"
#include "spring_shape.h"
#include "spring_state.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
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

// What applying a spring reads.
typedef struct Apply
{
    const maniSpringSet* set;
    const maniSkeleton* skeleton;
    const uint32_t* block;
    maniPose* pose;
    float alpha;
} Apply;

// A joint's head bone turned by q in model space, its parent's rotation
// the chain's turn of the pose's.
static void Turn(const Apply* a, const Chain* chain, uint32_t bone, maniQuat q)
{
    maniTransform parent =
        maniPoseModelTransformEntered(a->pose, maniSkeletonParent(a->skeleton, bone));
    maniQuat above = maniQuatNormalize(maniQuatMul(chain->rotation, parent.rotation));
    maniQuat inParent = maniQuatMul(maniQuatMul(maniQuatConjugate(above), q), above);
    maniTransform local = maniPoseLocal(a->pose, bone);
    local.rotation =
        maniQuatNormalize(maniQuatMul(maniSpringMirror(inParent, parent.scale), local.rotation));
    maniPoseSetLocal(a->pose, bone, local);
}

// A spring's colliders as they stand at the last call's end, the first
// SHAPES made once for all its joints.
#define SHAPES 16u

typedef struct Shapes
{
    const Apply* a;
    const uint8_t* spring;
    maniSpringShape first[SHAPES];
} Shapes;

static maniSpringShape LinkShape(const Shapes* s, uint32_t k)
{
    const maniSpringSet* set = s->a->set;
    uint32_t c = maniLoadU32(set->links + 4 * (size_t)(maniLoadU32(s->spring + 8) + k));
    return maniSpringShapeOf(set->colliders + (size_t)c * MANI_SPRING_COLLIDER_SIZE,
                             maniSpringBoneWords(s->a->block, maniSpringColliderAt(set, c)));
}

static void MakeShapes(Shapes* s)
{
    uint32_t count = maniLoadU32(s->spring + 12);
    for (uint32_t k = 0; k < count && k < SHAPES; ++k)
    {
        s->first[k] = LinkShape(s, k);
    }
}

// A tail of the joint's radius pushed out of the spring's colliders (or
// back into inside ones), in the order the steps take them.
static maniVec3 PushOut(const Shapes* s, maniVec3 tail, float radius)
{
    for (uint32_t k = 0; k < maniLoadU32(s->spring + 12); ++k)
    {
        maniSpringShape shape = k < SHAPES ? s->first[k] : LinkShape(s, k);
        maniVec3 direction;
        float distance = 0.0f;
        if (maniSpringCollide(&shape, tail, radius, &direction, &distance))
        {
            tail = maniVec3Sub(tail, maniVec3Scale(direction, distance));
        }
    }
    return tail;
}

static void ApplySpring(const Apply* a, uint32_t spring)
{
    const maniSpringSet* set = a->set;
    const uint32_t* block = a->block;
    maniPose* pose = a->pose;
    float alpha = a->alpha;
    const uint8_t* e = set->springs + (size_t)spring * MANI_SPRING_SPRING_SIZE;
    uint32_t center = maniLoadU32(e + 16);
    maniAffine space = maniPoseModel(pose, center);
    Carry carry = CarryOf(block);
    Chain chain = {maniQuatIdentity(), maniVec3Make(0.0f, 0.0f, 0.0f)};
    Shapes shapes = {.a = a, .spring = e};
    MakeShapes(&shapes);
    uint32_t first = maniLoadU32(e);
    for (uint32_t j = first; j < first + maniLoadU32(e + 4); ++j)
    {
        const uint8_t* r = set->joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
        uint32_t bone = maniLoadU32(r + maniSpringJointHead);
        maniVec3 p = Move(&chain, PosePosition(pose, bone));
        maniVec3 tail = Move(&chain, PosePosition(pose, maniLoadU32(r + maniSpringJointTail)));
        maniVec3 from = maniVec3Normalize(maniVec3Sub(tail, p), kUp);
        uint64_t at = maniSpringJointAt(j);
        maniVec3 sim = Lerp(maniSpringVec(block, at + maniSpringTailBefore),
                            maniSpringVec(block, at + maniSpringTailNow), 1.0f + alpha);
        sim = center != MANI_NO_BONE
                  ? maniAffineTransformPoint(&space, sim)
                  : maniVec3Add(maniQuatRotate(carry.rotation, sim), carry.translation);
        // At the bone's length toward the moved tail, then out of the
        // colliders: the steps' tails are, but a tail moved on between
        // steps can cross a collider by as far as it moves.
        maniVec3 to = maniVec3Normalize(maniVec3Sub(sim, p), from);
        maniVec3 shown = maniVec3Add(p, maniVec3Scale(to, maniVec3Length(maniVec3Sub(tail, p))));
        float radius = maniSpringFileFloat(r + maniSpringJointHitRadius) *
                       maniSpringWord(block, at + maniSpringHeadScale);
        shown = PushOut(&shapes, shown, radius);
        maniQuat turn = maniQuatBetween(from, maniVec3Normalize(maniVec3Sub(shown, p), to));
        Turn(a, &chain, bone, turn);
        maniVec3 moved = maniVec3Add(maniQuatRotate(turn, maniVec3Sub(chain.translation, p)), p);
        chain = (Chain){maniQuatNormalize(maniQuatMul(turn, chain.rotation)), moved};
    }
}

void maniSpringApplyBlock(const maniSpringSet* set, const maniSkeleton* skeleton,
                          const uint32_t* block, maniPose* pose)
{
    if (block[maniSpringHas] == 0)
    {
        return;
    }
    // The clock is below a step, so the fraction at most 1.
    Apply a = {set, skeleton, block, pose,
               (float)(maniSpringClockOf(block) * (double)set->stepRate)};
    for (uint32_t i = 0; i < set->springCount; ++i)
    {
        ApplySpring(&a, i);
    }
}
