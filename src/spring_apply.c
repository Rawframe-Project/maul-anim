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
#include "spring_view.h"
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
    maniAffine m = maniPose_GetModel(pose, bone);
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
        maniPoseModelTransformEntered(a->pose, maniSkeleton_GetParent(a->skeleton, bone));
    maniQuat above = maniQuatNormalize(maniQuatMul(chain->rotation, parent.rotation));
    maniQuat inParent = maniQuatMul(maniQuatMul(maniQuatConjugate(above), q), above);
    maniTransform local = maniPose_GetLocal(a->pose, bone);
    local.rotation =
        maniQuatNormalize(maniQuatMul(maniSpringMirror(inParent, parent.scale), local.rotation));
    maniPose_SetLocal(a->pose, bone, local);
}

// A spring's colliders as they stand at the last call's end, the first
// SHAPES made once for all its joints.
#define SHAPES 16u

typedef struct Shapes
{
    const Apply* a;
    // The spring's links, [first, end), within the set's.
    uint32_t first;
    uint32_t end;
    maniSpringShape shapes[SHAPES];
    // Each of the first SHAPES links' shape in shapes; NULL for one
    // skipped.
    const maniSpringShape* made[SHAPES];
} Shapes;

// Link k's collider's shape; false for a collider past the colliders,
// in bytes changed since the open, which is skipped.
MANI_NODISCARD static bool LinkShape(const Shapes* s, uint32_t k, maniSpringShape* out)
{
    const maniSpringSet* set = s->a->set;
    uint32_t c = maniSpringLinkCollider(set, k);
    if (c == UINT32_MAX)
    {
        return false;
    }
    *out = maniSpringShapeOf(maniSpringCollider(set, c),
                             maniSpringBoneWords(s->a->block, maniSpringColliderAt(set, c)));
    return true;
}

static void MakeShapes(Shapes* s)
{
    for (uint32_t k = s->first; k < s->end && k - s->first < SHAPES; ++k)
    {
        maniSpringShape* shape = &s->shapes[k - s->first];
        s->made[k - s->first] = LinkShape(s, k, shape) ? shape : NULL;
    }
}

// A tail of the joint's radius pushed out of the spring's colliders (or
// back into inside ones), in the order the steps take them.
static maniVec3 PushOut(const Shapes* s, maniVec3 tail, float radius)
{
    for (uint32_t k = s->first; k < s->end; ++k)
    {
        maniSpringShape spare;
        const maniSpringShape* shape = NULL;
        if (k - s->first < SHAPES)
        {
            shape = s->made[k - s->first];
        }
        else if (LinkShape(s, k, &spare))
        {
            shape = &spare;
        }
        if (shape == NULL)
        {
            continue;
        }
        maniVec3 direction;
        float distance = 0.0f;
        if (maniSpringCollide(shape, tail, radius, &direction, &distance))
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
    const uint8_t* e = maniSpringRecord(set, spring);
    uint32_t center = maniLoadU32(e + 16);
    maniAffine space = maniPose_GetModel(pose, center);
    Carry carry = CarryOf(block);
    Chain chain = {maniQuatIdentity(), maniVec3Make(0.0f, 0.0f, 0.0f)};
    Shapes shapes = {.a = a};
    maniSpringLinksOf(set, e, &shapes.first, &shapes.end);
    MakeShapes(&shapes);
    uint32_t first = 0;
    uint32_t end = 0;
    maniSpringJointsOf(set, e, &first, &end);
    for (uint32_t j = first; j < end; ++j)
    {
        const uint8_t* r = maniSpringJoint(set, j);
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
