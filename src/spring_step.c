// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Stepping spring tails and turning joints toward them (mani-0026). A
// call runs the steps due at the set's rate, each with its inputs (the
// joints' heads and tails, the colliders' and centers' bones in model
// space, and the character's placement) interpolated between the last
// call's and this one's at the step's time. A step moves each spring's
// joints from the root to the leaves, as VRM orders them:
//
//   next = now + (now - before)(1 - drag) + stiffness * the pose's
//          direction + gravity, both times the head's scale,
//
// then the bone's length, the limit, and for each collider a push and
// the length and limit again. The joints below a turned joint turn with
// it: a rigid motion, carried down the chain, maps the pose's model
// space to the turned one. Tails keep in the simulation's space: the
// center's, or model space as the last step placed it, carried into the
// next step's placement so the character's motion in the world sways
// them. The pose's direction and length come from the pose itself, so a
// joint the animation moves springs toward its animated direction.

#include "bytes.h"
#include "spring_file.h"
#include "spring_shape.h"
#include "spring_state.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const maniVec3 kUp = {0.0f, 1.0f, 0.0f};

static float FileFloat(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static maniVec3 FileVec(const uint8_t* p)
{
    return maniVec3Make(FileFloat(p), FileFloat(p + 4), FileFloat(p + 8));
}

static maniVec3 Lerp(maniVec3 a, maniVec3 b, float f)
{
    return maniVec3Add(a, maniVec3Scale(maniVec3Sub(b, a), f));
}

// A rigid motion: x' = rotation x + translation.
typedef struct Rigid
{
    maniQuat rotation;
    maniVec3 translation;
} Rigid;

static maniVec3 Move(const Rigid* r, maniVec3 x)
{
    return maniVec3Add(maniQuatRotate(r->rotation, x), r->translation);
}

// The motion r followed by a turn q about the point p.
static Rigid TurnAbout(const Rigid* r, maniQuat q, maniVec3 p)
{
    maniVec3 t = maniVec3Add(maniQuatRotate(q, maniVec3Sub(r->translation, p)), p);
    return (Rigid){maniQuatNormalize(maniQuatMul(q, r->rotation)), t};
}

static float LargestScale(maniVec3 s)
{
    float x = fabsf(s.x);
    float y = fabsf(s.y);
    float z = fabsf(s.z);
    float m = x > y ? x : y;
    return m > z ? m : z;
}

// A placement: translation and rotation.
typedef struct Place
{
    maniVec3 position;
    maniQuat rotation;
} Place;

static Place LoadPlace(const uint32_t* block, uint64_t at)
{
    return (Place){maniSpringVec(block, at), maniSpringQuat(block, at + 3)};
}

static void StorePlace(uint32_t* block, uint64_t at, Place p)
{
    maniSpringSetVec(block, at, p.position);
    maniSpringSetQuat(block, at + 3, p.rotation);
}

// A point of model space under placement from carried into placement to.
static maniVec3 Carry(Place from, Place to, maniVec3 x)
{
    maniVec3 world = maniVec3Add(from.position, maniQuatRotate(from.rotation, x));
    return maniQuatRotate(maniQuatConjugate(to.rotation), maniVec3Sub(world, to.position));
}

static maniTransform BoneWords(const uint32_t* block, uint64_t at)
{
    return (maniTransform){maniSpringVec(block, at), maniSpringQuat(block, at + 3),
                           maniSpringVec(block, at + 7)};
}

static void SetBoneWords(uint32_t* block, uint64_t at, maniTransform t)
{
    maniSpringSetVec(block, at, t.translation);
    maniSpringSetQuat(block, at + 3, t.rotation);
    maniSpringSetVec(block, at + 7, t.scale);
}

static maniTransform LerpTransform(maniTransform a, maniTransform b, float f)
{
    return (maniTransform){Lerp(a.translation, b.translation, f),
                           maniQuatNlerp(a.rotation, b.rotation, f), Lerp(a.scale, b.scale, f)};
}

// What a step reads: the set, the block (last call's inputs), the pose
// (this call's), the fraction of the way between them, the placements.
typedef struct Step
{
    const maniSpringSet* set;
    uint32_t* block;
    const maniPose* pose;
    float fraction;
    Place place;
    Place last;
} Step;

static maniVec3 PosePosition(const maniPose* pose, uint32_t bone)
{
    maniAffine m = maniPoseModel(pose, bone);
    return maniAffineTransformPoint(&m, maniVec3Make(0.0f, 0.0f, 0.0f));
}

// A joint's inputs at the step: its head and the pose's tail.
typedef struct Head
{
    maniVec3 position;
    maniQuat rotation;
    float scale;
    maniVec3 tail;
    // The pose's scale, for its mirror.
    maniVec3 mirror;
} Head;

static Head HeadAt(const Step* s, uint32_t joint)
{
    const uint8_t* e = s->set->joints + (size_t)joint * MANI_SPRING_JOINT_SIZE;
    uint64_t at = maniSpringJointAt(joint);
    maniTransform now = maniPoseModelTransform(s->pose, maniLoadU32(e + maniSpringJointHead));
    float f = s->fraction;
    return (Head){
        Lerp(maniSpringVec(s->block, at + maniSpringHeadPosition), now.translation, f),
        maniQuatNlerp(maniSpringQuat(s->block, at + maniSpringHeadRotation), now.rotation, f),
        maniSpringWord(s->block, at + maniSpringHeadScale) +
            f * (LargestScale(now.scale) - maniSpringWord(s->block, at + maniSpringHeadScale)),
        Lerp(maniSpringVec(s->block, at + maniSpringTailPosition),
             PosePosition(s->pose, maniLoadU32(e + maniSpringJointTail)), f),
        now.scale};
}

// A collider in model space at the step.
static maniSpringShape ShapeAt(const Step* s, uint32_t collider)
{
    const uint8_t* e = s->set->colliders + (size_t)collider * MANI_SPRING_COLLIDER_SIZE;
    maniTransform now = maniPoseModelTransform(s->pose, maniLoadU32(e));
    maniTransform t = LerpTransform(BoneWords(s->block, maniSpringColliderAt(s->set, collider)),
                                    now, s->fraction);
    maniAffine m = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    uint32_t shape = maniLoadU32(e + 4);
    maniVec3 tail = FileVec(e + 28);
    // A normal turns with the inverse transpose: the cofactors' signs
    // kept by the determinant's, so no division by a zero scale.
    maniVec3 k = t.scale;
    float sign = k.x * k.y * k.z < 0.0f ? -1.0f : 1.0f;
    maniVec3 normal = maniVec3Scale(
        maniVec3Make(tail.x * k.y * k.z, tail.y * k.x * k.z, tail.z * k.x * k.y), sign);
    return (maniSpringShape){maniAffineTransformPoint(&m, FileVec(e + 16)),
                             shape == maniColliderPlane
                                 ? maniVec3Normalize(maniQuatRotate(t.rotation, normal), kUp)
                                 : maniAffineTransformPoint(&m, tail),
                             FileFloat(e + 40) * LargestScale(t.scale), shape, maniLoadU32(e + 8)};
}

// The tail kept at length from the head, within the limit.
static maniVec3 Keep(const uint8_t* joint, maniQuat frame, maniVec3 head, maniVec3 tail,
                     maniVec3 pull, float length)
{
    maniVec3 direction = maniVec3Normalize(maniVec3Sub(tail, head), pull);
    uint32_t limit = maniLoadU32(joint + maniSpringJointLimit);
    if (limit != maniSpringLimitNone)
    {
        maniVec3 local = maniQuatRotate(maniQuatConjugate(frame), direction);
        local = maniSpringLimitDirection(limit, FileFloat(joint + maniSpringJointAngle),
                                         FileFloat(joint + maniSpringJointYaw), local);
        direction = maniQuatRotate(frame, local);
    }
    return maniVec3Add(head, maniVec3Scale(direction, length));
}

// Joint j of a spring whose colliders begin at link: its tail moved from
// now and before (model space at the step), the chain's motion so far.
static maniVec3 MoveTail(const Step* s, uint32_t joint, const uint8_t* spring, const Rigid* chain,
                         maniVec3 now, maniVec3 before, maniVec3* pullOut, maniVec3* headOut)
{
    const uint8_t* e = s->set->joints + (size_t)joint * MANI_SPRING_JOINT_SIZE;
    Head h = HeadAt(s, joint);
    maniVec3 head = Move(chain, h.position);
    maniVec3 tail = Move(chain, h.tail);
    float length = maniVec3Length(maniVec3Sub(tail, head));
    maniVec3 pull = maniVec3Normalize(maniVec3Sub(tail, head), kUp);
    // The limit's frame in the head's space, mirrored with the head: the
    // limits are alike under the x mirror a model scale holds.
    maniQuat local = {FileFloat(e + maniSpringJointFrame), FileFloat(e + maniSpringJointFrame + 4),
                      FileFloat(e + maniSpringJointFrame + 8),
                      FileFloat(e + maniSpringJointFrame + 12)};
    maniQuat frame =
        maniQuatMul(maniQuatMul(chain->rotation, h.rotation), maniSpringMirror(local, h.mirror));
    maniVec3 gravity =
        maniQuatRotate(maniQuatConjugate(s->place.rotation), FileVec(e + maniSpringJointGravity));
    float drag = FileFloat(e + maniSpringJointDrag);
    float stiffness = FileFloat(e + maniSpringJointStiffness);
    maniVec3 next = maniVec3Add(now, maniVec3Scale(maniVec3Sub(now, before), 1.0f - drag));
    next = maniVec3Add(next, maniVec3Scale(pull, stiffness * h.scale));
    next = maniVec3Add(next, maniVec3Scale(gravity, h.scale));
    next = Keep(e, frame, head, next, pull, length);
    float radius = FileFloat(e + maniSpringJointHitRadius) * h.scale;
    uint32_t first = maniLoadU32(spring + 8);
    for (uint32_t k = 0; k < maniLoadU32(spring + 12); ++k)
    {
        maniSpringShape shape = ShapeAt(s, maniLoadU32(s->set->links + 4 * (size_t)(first + k)));
        maniVec3 direction;
        float distance = 0.0f;
        if (maniSpringCollide(&shape, next, radius, &direction, &distance))
        {
            next = maniVec3Sub(next, maniVec3Scale(direction, distance));
            next = Keep(e, frame, head, next, pull, length);
        }
    }
    *pullOut = pull;
    *headOut = head;
    return next;
}

// A spring's center at the step, or none.
static bool CenterOf(const Step* s, uint32_t spring, maniAffine* centerOut)
{
    const uint8_t* e = s->set->springs + (size_t)spring * MANI_SPRING_SPRING_SIZE;
    uint32_t center = maniLoadU32(e + 16);
    if (center == MANI_NO_BONE)
    {
        return false;
    }
    maniTransform now = maniPoseModelTransform(s->pose, center);
    maniTransform t =
        LerpTransform(BoneWords(s->block, maniSpringCenterAt(s->set, spring)), now, s->fraction);
    *centerOut = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    return true;
}

static void StepSpring(const Step* s, uint32_t spring)
{
    const uint8_t* e = s->set->springs + (size_t)spring * MANI_SPRING_SPRING_SIZE;
    maniAffine center;
    maniAffine inverse;
    bool centered = CenterOf(s, spring, &center) && maniAffineInverse(&center, &inverse);
    Rigid chain = {maniQuatIdentity(), maniVec3Make(0.0f, 0.0f, 0.0f)};
    uint32_t first = maniLoadU32(e);
    for (uint32_t j = first; j < first + maniLoadU32(e + 4); ++j)
    {
        uint64_t at = maniSpringJointAt(j);
        maniVec3 now = maniSpringVec(s->block, at + maniSpringTailNow);
        maniVec3 before = maniSpringVec(s->block, at + maniSpringTailBefore);
        now = centered ? maniAffineTransformPoint(&center, now) : Carry(s->last, s->place, now);
        before =
            centered ? maniAffineTransformPoint(&center, before) : Carry(s->last, s->place, before);
        maniVec3 pull;
        maniVec3 head;
        maniVec3 next = MoveTail(s, j, e, &chain, now, before, &pull, &head);
        maniVec3 turned = maniVec3Normalize(maniVec3Sub(next, head), pull);
        chain = TurnAbout(&chain, maniQuatBetween(pull, turned), head);
        maniSpringSetVec(s->block, at + maniSpringTailBefore,
                         centered ? maniAffineTransformPoint(&inverse, now) : now);
        maniSpringSetVec(s->block, at + maniSpringTailNow,
                         centered ? maniAffineTransformPoint(&inverse, next) : next);
    }
}

static Place PlaceAt(const uint32_t* block, Place now, float f)
{
    Place last = LoadPlace(block, maniSpringPlacement);
    return (Place){Lerp(last.position, now.position, f),
                   maniQuatNlerp(last.rotation, now.rotation, f)};
}

// This call's inputs kept as the next call's last.
static void KeepInputs(const maniSpringSet* set, uint32_t* block, const maniPose* pose, Place now)
{
    for (uint32_t j = 0; j < set->jointCount; ++j)
    {
        const uint8_t* e = set->joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
        maniTransform head = maniPoseModelTransform(pose, maniLoadU32(e + maniSpringJointHead));
        uint64_t at = maniSpringJointAt(j);
        maniSpringSetVec(block, at + maniSpringHeadPosition, head.translation);
        maniSpringSetQuat(block, at + maniSpringHeadRotation, head.rotation);
        maniSpringSetWord(block, at + maniSpringHeadScale, LargestScale(head.scale));
        maniSpringSetVec(block, at + maniSpringTailPosition,
                         PosePosition(pose, maniLoadU32(e + maniSpringJointTail)));
    }
    for (uint32_t c = 0; c < set->colliderCount; ++c)
    {
        uint32_t bone = maniLoadU32(set->colliders + (size_t)c * MANI_SPRING_COLLIDER_SIZE);
        SetBoneWords(block, maniSpringColliderAt(set, c), maniPoseModelTransform(pose, bone));
    }
    for (uint32_t i = 0; i < set->springCount; ++i)
    {
        uint32_t center = maniLoadU32(set->springs + (size_t)i * MANI_SPRING_SPRING_SIZE + 16);
        if (center != MANI_NO_BONE)
        {
            SetBoneWords(block, maniSpringCenterAt(set, i), maniPoseModelTransform(pose, center));
        }
    }
    StorePlace(block, maniSpringPlacement, now);
}

// The first call: every tail where the pose puts it, at rest.
static void Begin(const maniSpringSet* set, uint32_t* block, const maniPose* pose, Place now)
{
    KeepInputs(set, block, pose, now);
    StorePlace(block, maniSpringStepPlacement, now);
    for (uint32_t i = 0; i < set->springCount; ++i)
    {
        const uint8_t* e = set->springs + (size_t)i * MANI_SPRING_SPRING_SIZE;
        uint32_t center = maniLoadU32(e + 16);
        maniAffine inverse = {
            {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}};
        if (center != MANI_NO_BONE)
        {
            maniAffine m = maniPoseModel(pose, center);
            (void)maniAffineInverse(&m, &inverse);
        }
        for (uint32_t j = maniLoadU32(e); j < maniLoadU32(e) + maniLoadU32(e + 4); ++j)
        {
            const uint8_t* r = set->joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
            maniVec3 tail = PosePosition(pose, maniLoadU32(r + maniSpringJointTail));
            tail = maniAffineTransformPoint(&inverse, tail);
            maniSpringSetVec(block, maniSpringJointAt(j) + maniSpringTailNow, tail);
            maniSpringSetVec(block, maniSpringJointAt(j) + maniSpringTailBefore, tail);
        }
    }
    block[maniSpringHas] = 1;
}

static void SetClock(uint32_t* block, double v)
{
    uint64_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    block[maniSpringClock] = (uint32_t)bits;
    block[maniSpringClock + 1] = (uint32_t)(bits >> 32);
}

void maniSpringStepBlock(const maniSpringSet* set, uint32_t* block, const maniPose* pose,
                         const maniTransform* placement, double step)
{
    Place now = {placement->translation, placement->rotation};
    if (block[maniSpringHas] == 0)
    {
        Begin(set, block, pose, now);
    }
    double seconds = 1.0 / (double)set->stepRate;
    double most = MANI_SPRING_MAX_STEPS * seconds;
    double e = step < most ? step : most;
    double clock = maniSpringClockOf(block);
    uint32_t steps = 0;
    for (double t = seconds - clock; t <= e && steps < MANI_SPRING_MAX_STEPS;
         t = (double)(steps + 1) * seconds - clock)
    {
        float f = e > 0.0 ? (float)(t / e) : 1.0f;
        Step s = {
            set, block, pose, f, PlaceAt(block, now, f), LoadPlace(block, maniSpringStepPlacement)};
        for (uint32_t i = 0; i < set->springCount; ++i)
        {
            StepSpring(&s, i);
        }
        StorePlace(block, maniSpringStepPlacement, s.place);
        ++steps;
    }
    clock = clock + e - (double)steps * seconds;
    SetClock(block, clock >= 0.0 && clock < seconds ? clock : 0.0);
    KeepInputs(set, block, pose, now);
}
