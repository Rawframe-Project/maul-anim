// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ragdoll bodies and poses: bodies placed from a pose's model
// frames (scale left out) under the placement, with velocities as the
// differences from the pose before; joints' motor targets from bodies;
// a pose from bodies, the bones without bodies keeping a reference's
// locals. Frames are rigid: a translation t and a unit rotation r, a
// frame A after B being (A.t + A.r B.t, A.r B.r).

#include "bytes.h"
#include "decompose.h"
#include "fpenv.h"
#include "pose_memory.h"
#include "ragdoll_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/ragdoll.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct Rigid
{
    maniVec3 t;
    maniQuat r;
} Rigid;

static const Rigid kIdentity = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};

// a after b, its rotation not normalized: each result is, once.
static inline Rigid Compose(Rigid a, Rigid b)
{
    return (Rigid){maniVec3Add(a.t, maniQuatRotate(a.r, b.t)), maniQuatMul(a.r, b.r)};
}

static Rigid Inverse(Rigid a)
{
    maniQuat r = maniQuatConjugate(a.r);
    return (Rigid){maniVec3Scale(maniQuatRotate(r, a.t), -1.0f), r};
}

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static Rigid LoadRigid(const uint8_t* p)
{
    return (Rigid){{LoadF32(p), LoadF32(p + 4), LoadF32(p + 8)},
                   {LoadF32(p + 12), LoadF32(p + 16), LoadF32(p + 20), LoadF32(p + 24)}};
}

static const uint8_t* Record(const maniRagdoll* ragdoll, uint32_t body)
{
    return ragdoll->bodies + (size_t)body * MANI_RAGDOLL_BODY_SIZE;
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

static bool UnitQuat(maniQuat q)
{
    float l = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return l >= 0.998f && l <= 1.002f;
}

// A placement's rigid part: the identity for NULL; false when not
// finite or its rotation not of length 1.
static bool Placement(const maniTransform* placement, Rigid* out)
{
    *out = kIdentity;
    if (placement == NULL)
    {
        return true;
    }
    maniVec3 t = placement->translation;
    maniQuat r = placement->rotation;
    bool finite = Finite(t.x) && Finite(t.y) && Finite(t.z) && Finite(r.x) && Finite(r.y) &&
                  Finite(r.z) && Finite(r.w);
    *out = (Rigid){t, maniQuatNormalize(r)};
    return finite && UnitQuat(r);
}

// A bone's model frame from a matrix, its scale left out.
static Rigid Split(const maniAffine* m)
{
    Rigid out;
    maniDecomposeRigid(m, &out.t, &out.r);
    return out;
}

// Body i's frame in the world from a pose.
static Rigid BodyFrame(const maniRagdoll* ragdoll, const maniPose* pose, Rigid place, uint32_t i)
{
    const uint8_t* e = Record(ragdoll, i);
    maniAffine model = maniPose_GetModel(pose, maniLoadU32(e + maniRagdollBodyBone));
    Rigid body = Compose(Compose(place, Split(&model)), LoadRigid(e + maniRagdollBodyFrame));
    return (Rigid){body.t, maniQuatNormalize(body.r)};
}

// The turn with w at least 0: q and -q turn alike.
static maniQuat Shortest(maniQuat q)
{
    return q.w < 0.0f ? (maniQuat){-q.x, -q.y, -q.z, -q.w} : q;
}

static bool PoseUsable(const maniRagdoll* ragdoll, const maniPose* pose)
{
    return maniPoseOfSkeleton(pose, ragdoll->skeletonIdentity, ragdoll->boneCount) &&
           maniPose_IsModelCurrent(pose);
}

static MANI_NOINLINE maniResult RagdollBodiesFromPoseWork(
    const maniRagdoll* ragdoll, const maniPose* pose, const maniTransform* placement,
    const maniPose* previous, const maniTransform* previousPlacement, float seconds,
    maniRagdollBodyState* bodiesOut, uint32_t capacity, uint32_t* countOut)
{
    Rigid place;
    Rigid before;
    if (ragdoll == NULL || (bodiesOut == NULL && capacity > 0) || !PoseUsable(ragdoll, pose) ||
        !Placement(placement, &place) || !Placement(previousPlacement, &before) ||
        (previous != NULL &&
         (!PoseUsable(ragdoll, previous) || !(seconds > 0.0f) || !Finite(seconds))))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = ragdoll->bodyCount;
    }
    if (bodiesOut == NULL || capacity < ragdoll->bodyCount)
    {
        return mani_errorCapacity;
    }
    for (uint32_t i = 0; i < ragdoll->bodyCount; ++i)
    {
        Rigid now = BodyFrame(ragdoll, pose, place, i);
        maniRagdollBodyState b = {now.t, now.r, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
        if (previous != NULL)
        {
            Rigid then = BodyFrame(ragdoll, previous, before, i);
            float rate = 1.0f / seconds;
            b.linearVelocity = maniVec3Scale(maniVec3Sub(now.t, then.t), rate);
            // The rotation vector is the shorter turn's (at most half a
            // turn).
            maniQuat turn = maniQuatMul(now.r, maniQuatConjugate(then.r));
            b.angularVelocity = maniVec3Scale(maniQuatToRotationVector(turn), rate);
        }
        bodiesOut[i] = b;
    }
    return mani_success;
}

maniResult maniRagdoll_ComputeBodiesFromPose(const maniRagdoll* ragdoll, const maniPose* pose,
                                             const maniTransform* placement,
                                             const maniPose* previous,
                                             const maniTransform* previousPlacement, float seconds,
                                             maniRagdollBodyState* bodiesOut, uint32_t capacity,
                                             uint32_t* countOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result =
        RagdollBodiesFromPoseWork(ragdoll, pose, placement, previous, previousPlacement, seconds,
                                  bodiesOut, capacity, countOut);
    maniFloatEnvLeave(env);
    return result;
}

static MANI_NOINLINE maniResult RagdollJointTargetsWork(const maniRagdoll* ragdoll,
                                                        const maniRagdollBodyState* bodies,
                                                        maniRagdollJointTarget* jointsOut,
                                                        uint32_t capacity, uint32_t* countOut)
{
    if (ragdoll == NULL || bodies == NULL || (jointsOut == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = ragdoll->bodyCount;
    }
    if (jointsOut == NULL || capacity < ragdoll->bodyCount)
    {
        return mani_errorCapacity;
    }
    jointsOut[0] = (maniRagdollJointTarget){{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
    for (uint32_t i = 1; i < ragdoll->bodyCount; ++i)
    {
        const uint8_t* e = Record(ragdoll, i);
        const maniRagdollBodyState* child = &bodies[i];
        const maniRagdollBodyState* parent = &bodies[maniLoadU32(e + maniRagdollBodyParent)];
        Rigid inChild =
            Compose((Rigid){child->position, child->rotation}, LoadRigid(e + maniRagdollBodyJoint));
        Rigid inParent = Compose((Rigid){parent->position, parent->rotation},
                                 LoadRigid(e + maniRagdollBodyParentJoint));
        maniQuat back = maniQuatConjugate(inParent.r);
        maniQuat turn = Shortest(maniQuatNormalize(maniQuatMul(back, inChild.r)));
        maniVec3 relative = maniVec3Sub(child->angularVelocity, parent->angularVelocity);
        jointsOut[i] = (maniRagdollJointTarget){turn, maniQuatRotate(back, relative)};
    }
    return mani_success;
}

maniResult maniRagdoll_ComputeJointTargets(const maniRagdoll* ragdoll,
                                           const maniRagdollBodyState* bodies,
                                           maniRagdollJointTarget* jointsOut, uint32_t capacity,
                                           uint32_t* countOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = RagdollJointTargetsWork(ragdoll, bodies, jointsOut, capacity, countOut);
    maniFloatEnvLeave(env);
    return result;
}

static bool BodiesValid(const maniRagdollBodyState* bodies, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        maniVec3 p = bodies[i].position;
        maniQuat r = bodies[i].rotation;
        if (!(Finite(p.x) && Finite(p.y) && Finite(p.z) && Finite(r.x) && Finite(r.y) &&
              Finite(r.z) && Finite(r.w) && UnitQuat(r)))
        {
            return false;
        }
    }
    return true;
}

// Bone's local transform taking it to the model frame want under its
// parent's model matrix, its scale the reference's.
static maniTransform LocalFor(const maniAffine* parent, Rigid want, maniVec3 scale)
{
    if (parent == NULL)
    {
        return (maniTransform){want.t, want.r, scale};
    }
    maniAffine inverse;
    maniVec3 t = want.t;
    if (maniAffineInverse(parent, &inverse))
    {
        t = maniAffineTransformPoint(&inverse, want.t);
    }
    maniQuat r = maniQuatNormalize(maniQuatMul(maniQuatConjugate(Split(parent).r), want.r));
    return (maniTransform){t, r, scale};
}

static MANI_NOINLINE maniResult RagdollPoseFromBodiesWork(
    const maniRagdoll* ragdoll, const maniSkeleton* skeleton, const maniRagdollBodyState* bodies,
    uint32_t count, const maniTransform* placement, const maniPose* reference, maniPose* poseOut)
{
    Rigid place;
    if (ragdoll == NULL || bodies == NULL || skeleton == NULL ||
        maniSkeleton_GetIdentity(skeleton) != ragdoll->skeletonIdentity ||
        !maniPoseOfSkeleton(reference, ragdoll->skeletonIdentity, ragdoll->boneCount) ||
        !maniPoseOfSkeleton(poseOut, ragdoll->skeletonIdentity, ragdoll->boneCount) ||
        !Placement(placement, &place))
    {
        return mani_errorInvalid;
    }
    if (count < ragdoll->bodyCount)
    {
        return mani_errorCapacity;
    }
    if (!BodiesValid(bodies, ragdoll->bodyCount))
    {
        return mani_errorInvalid;
    }
    Rigid back = Inverse(place);
    uint32_t next = 0;
    for (uint32_t bone = 0; bone < ragdoll->boneCount; ++bone)
    {
        maniTransform local = maniPose_GetLocal(reference, bone);
        uint32_t parent = maniSkeleton_GetParent(skeleton, bone);
        maniAffine parentModel =
            parent != MANI_NO_BONE ? maniPose_GetModel(poseOut, parent) : (maniAffine){{0}};
        const uint8_t* e = next < ragdoll->bodyCount ? Record(ragdoll, next) : NULL;
        if (e != NULL && maniLoadU32(e + maniRagdollBodyBone) == bone)
        {
            // The bone's frame: the body's with the body's frame taken off.
            Rigid body = {bodies[next].position, maniQuatNormalize(bodies[next].rotation)};
            Rigid want = Compose(back, Compose(body, Inverse(LoadRigid(e + maniRagdollBodyFrame))));
            local = LocalFor(parent != MANI_NO_BONE ? &parentModel : NULL, want, local.scale);
            ++next;
        }
        maniPose_SetLocal(poseOut, bone, local);
        maniAffine own = maniAffineFromTrs(local.translation, local.rotation, local.scale);
        maniAffine model = parent != MANI_NO_BONE ? maniAffineMul(&parentModel, &own) : own;
        maniPoseStoreModel(poseOut, bone, &model);
    }
    maniPoseMarkModelCurrent(poseOut);
    return mani_success;
}

maniResult maniRagdoll_ComputePoseFromBodies(const maniRagdoll* ragdoll,
                                             const maniSkeleton* skeleton,
                                             const maniRagdollBodyState* bodies, uint32_t count,
                                             const maniTransform* placement,
                                             const maniPose* reference, maniPose* poseOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result =
        RagdollPoseFromBodiesWork(ragdoll, skeleton, bodies, count, placement, reference, poseOut);
    maniFloatEnvLeave(env);
    return result;
}
