// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pose transfer through a retargeting map: the source's
// model rotations, then each target bone from the root down by its plan
// (its local against its parent as just made), the pelvis placed in the
// model and taken into its parent's frame. Model rotations here are the
// products of the root paths' local rotations, as the map's references
// are; the pelvis's positions are the full transforms'.

#include "bytes.h"
#include "fpenv.h"
#include "pose_memory.h"
#include "retarget_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static maniVec3 LoadVec(const uint8_t* p)
{
    return maniVec3Make(LoadF32(p), LoadF32(p + 4), LoadF32(p + 8));
}

static maniQuat LoadQuat(const uint8_t* p)
{
    return (maniQuat){LoadF32(p), LoadF32(p + 4), LoadF32(p + 8), LoadF32(p + 12)};
}

static const uint8_t* SourceBone(const maniRetargetMap* map, uint32_t b)
{
    return map->sourceBones + (size_t)b * MANI_RETARGET_SOURCE_SIZE;
}

static const uint8_t* TargetBone(const maniRetargetMap* map, uint32_t b)
{
    return map->targetBones + (size_t)b * MANI_RETARGET_TARGET_SIZE;
}

// A reference model rotation, the identity for no bone.
static maniQuat SourceModel(const maniRetargetMap* map, uint32_t b)
{
    return b == MANI_NO_BONE ? maniQuatIdentity() : LoadQuat(SourceBone(map, b));
}

static maniQuat TargetModel(const maniRetargetMap* map, uint32_t b)
{
    return b == MANI_NO_BONE ? maniQuatIdentity() : LoadQuat(TargetBone(map, b));
}

size_t maniRetargetScratchSize(const maniRetargetMap* map)
{
    if (map == NULL)
    {
        return 0;
    }
    return sizeof(maniQuat) * ((size_t)map->sourceBoneCount + map->targetBoneCount);
}

static MANI_NOINLINE maniVec3 RetargetRootMotionWork(const maniRetargetMap* map, maniVec3 delta)
{
    if (map == NULL)
    {
        return delta;
    }
    maniVec3 up = LoadVec(map->header + maniRetargetHeaderUp);
    maniVec3 vertical = maniVec3Scale(up, maniVec3Dot(up, delta));
    maniVec3 horizontal = maniVec3Sub(delta, vertical);
    return maniVec3Add(
        maniVec3Scale(horizontal, LoadF32(map->header + maniRetargetHeaderHorizontal)),
        maniVec3Scale(vertical, LoadF32(map->header + maniRetargetHeaderVertical)));
}

maniVec3 maniRetargetRootMotion(const maniRetargetMap* map, maniVec3 delta)
{
    uint64_t env = maniFloatEnvEnter();
    maniVec3 result = RetargetRootMotionWork(map, delta);
    maniFloatEnvLeave(env);
    return result;
}

typedef struct Transfer
{
    const maniRetargetMap* map;
    const maniSkeleton* source;
    const maniPose* sourcePose;
    const maniSkeleton* target;
    const maniPose* targetPose;
    maniQuat* sourceModels;
    maniQuat* targetModels;
} Transfer;

// Source bone a's turn from its reference, in the model.
static maniQuat Delta(const Transfer* x, uint32_t a)
{
    return maniQuatMul(x->sourceModels[a], maniQuatConjugate(SourceModel(x->map, a)));
}

static float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

// A bone's model matrix: the full transforms of its root path in a pose.
static maniAffine PathMatrix(const maniSkeleton* skeleton, const maniPose* pose, uint32_t bone)
{
    maniTransform t = maniPoseLocal(pose, bone);
    maniAffine m = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    for (uint32_t p = maniSkeletonParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeletonParent(skeleton, p))
    {
        t = maniPoseLocal(pose, p);
        maniAffine parent = maniAffineFromTrs(t.translation, t.rotation, t.scale);
        m = maniAffineMul(&parent, &m);
    }
    return m;
}

// A source bone's model position.
static maniVec3 SourcePosition(const Transfer* x, uint32_t bone)
{
    maniAffine m = PathMatrix(x->source, x->sourcePose, bone);
    return maniVec3Make(m.m[9], m.m[10], m.m[11]);
}

// The pelvis's model position: the target reference's plus the source's
// change, scaled by parts, blended per axis toward the unscaled change
// and then the source's own position, weighted from the reference.
static maniVec3 PelvisPosition(const Transfer* x, uint32_t sourcePelvis)
{
    const uint8_t* h = x->map->header;
    maniVec3 reference = LoadVec(h + maniRetargetHeaderTargetPelvis);
    maniVec3 now = SourcePosition(x, sourcePelvis);
    maniVec3 change = maniVec3Sub(now, LoadVec(h + maniRetargetHeaderSourcePelvis));
    maniVec3 scaled = maniRetargetRootMotion(x->map, change);
    maniVec3 absolute = LoadVec(h + maniRetargetHeaderAbsolute);
    maniVec3 toSource = LoadVec(h + maniRetargetHeaderToSource);
    float weight = LoadF32(h + maniRetargetHeaderTranslationWeight);
    const float s[3] = {scaled.x, scaled.y, scaled.z};
    const float c[3] = {change.x, change.y, change.z};
    const float r[3] = {reference.x, reference.y, reference.z};
    const float n[3] = {now.x, now.y, now.z};
    const float a[3] = {absolute.x, absolute.y, absolute.z};
    const float o[3] = {toSource.x, toSource.y, toSource.z};
    float out[3];
    for (int k = 0; k < 3; ++k)
    {
        float placed = Lerp(r[k] + Lerp(s[k], c[k], a[k]), n[k], o[k]);
        out[k] = Lerp(r[k], placed, weight);
    }
    return maniVec3Make(out[0], out[1], out[2]);
}

// A planned bone's local translation and scale from source bone a.
static void Carry(const Transfer* x, uint32_t bone, uint32_t a, uint32_t translation, bool scale,
                  maniTransform* local)
{
    if (translation == maniRetargetTranslationScaled ||
        translation == maniRetargetTranslationAbsolute)
    {
        maniVec3 from = maniPoseLocal(x->sourcePose, a).translation;
        if (translation == maniRetargetTranslationScaled)
        {
            float ratio = LoadF32(x->map->header + maniRetargetHeaderRatio);
            from = maniVec3Scale(maniVec3Sub(from, LoadVec(SourceBone(x->map, a) + 16)), ratio);
        }
        // From the source parent's reference model frame to the target
        // parent's.
        maniQuat frame =
            maniQuatMul(maniQuatConjugate(TargetModel(x->map, maniSkeletonParent(x->target, bone))),
                        SourceModel(x->map, maniSkeletonParent(x->source, a)));
        maniVec3 turned = maniQuatRotate(frame, from);
        local->translation = translation == maniRetargetTranslationScaled
                                 ? maniVec3Add(local->translation, turned)
                                 : turned;
    }
    if (scale)
    {
        maniVec3 now = maniPoseLocal(x->sourcePose, a).scale;
        maniVec3 was = LoadVec(SourceBone(x->map, a) + 28);
        local->scale.x *= was.x != 0.0f ? now.x / was.x : 1.0f;
        local->scale.y *= was.y != 0.0f ? now.y / was.y : 1.0f;
        local->scale.z *= was.z != 0.0f ? now.z / was.z : 1.0f;
    }
}

// A bone in its source's frame: its source's local as it is (the
// translation when absolute, or its reference plus the scaled change;
// the scale when carried) over its reference local.
static maniTransform SameFrame(const Transfer* x, uint32_t b, uint32_t a, maniQuat above,
                               uint32_t translation, bool scale, maniTransform local)
{
    const maniTransform given = maniPoseLocal(x->sourcePose, a);
    local.rotation = given.rotation;
    x->targetModels[b] = maniQuatMul(above, given.rotation);
    if (translation == maniRetargetTranslationAbsolute)
    {
        local.translation = given.translation;
    }
    else if (translation == maniRetargetTranslationScaled)
    {
        float ratio = LoadF32(x->map->header + maniRetargetHeaderRatio);
        local.translation =
            maniVec3Add(local.translation,
                        maniVec3Scale(maniVec3Sub(given.translation, local.translation), ratio));
    }
    local.scale = scale ? given.scale : local.scale;
    return local;
}

// Target bone b's local by its plan; its model rotation and matrix kept.
static maniTransform Bone(const Transfer* x, uint32_t b)
{
    const uint8_t* e = x->map->plan + (size_t)b * MANI_RETARGET_PLAN_SIZE;
    const uint8_t* t = TargetBone(x->map, b);
    uint32_t parent = maniSkeletonParent(x->target, b);
    maniQuat above = parent == MANI_NO_BONE ? maniQuatIdentity() : x->targetModels[parent];
    maniTransform local = {LoadVec(t + 32), LoadQuat(t + 16), LoadVec(t + 44)};
    uint32_t kind = maniLoadU32(e + maniRetargetPlanKind);
    if (kind == maniRetargetPlanReference)
    {
        x->targetModels[b] = maniQuatMul(above, local.rotation);
        return local;
    }
    uint32_t a = maniLoadU32(e + maniRetargetPlanA);
    uint32_t translation = maniLoadU32(e + maniRetargetPlanTranslation);
    uint32_t flags = maniLoadU32(e + maniRetargetPlanFlags);
    bool scale = (flags & MANI_RETARGET_PLAN_SCALE) != 0;
    if ((flags & MANI_RETARGET_PLAN_SAME) != 0)
    {
        return SameFrame(x, b, a, above, translation, scale, local);
    }
    maniQuat delta = Delta(x, a);
    if (kind == maniRetargetPlanTwo)
    {
        delta = maniQuatSlerp(delta, Delta(x, maniLoadU32(e + maniRetargetPlanB)),
                              LoadF32(e + maniRetargetPlanT));
    }
    float weight = LoadF32(e + maniRetargetPlanWeight);
    delta = weight < 1.0f ? maniQuatSlerp(maniQuatIdentity(), delta, weight) : delta;
    x->targetModels[b] = maniQuatNormalize(maniQuatMul(delta, LoadQuat(t)));
    local.rotation = maniQuatNormalize(maniQuatMul(maniQuatConjugate(above), x->targetModels[b]));
    if (translation != maniRetargetPlanPelvis)
    {
        Carry(x, b, a, translation, scale, &local);
        return local;
    }
    maniVec3 placed = PelvisPosition(x, a);
    // Its parent's matrix from the locals made so far.
    maniAffine matrix =
        parent != MANI_NO_BONE ? PathMatrix(x->target, x->targetPose, parent) : (maniAffine){{0}};
    maniAffine inverse;
    if (parent != MANI_NO_BONE && maniAffineInverse(&matrix, &inverse))
    {
        placed = maniAffineTransformPoint(&inverse, placed);
    }
    local.translation = placed;
    return local;
}

static bool Usable(const maniRetargetMap* map, const maniSkeleton* source,
                   const maniPose* sourcePose, const maniSkeleton* target,
                   const maniPose* targetPose, const void* scratch, size_t scratchSize)
{
    return map != NULL && source != NULL && target != NULL && sourcePose != NULL &&
           targetPose != NULL && scratch != NULL && (uintptr_t)scratch % 16 == 0 &&
           scratchSize >= maniRetargetScratchSize(map) &&
           maniSkeletonIdentity(source) == map->sourceIdentity &&
           maniSkeletonIdentity(target) == map->targetIdentity &&
           maniSkeletonBoneCount(source) == map->sourceBoneCount &&
           maniSkeletonBoneCount(target) == map->targetBoneCount &&
           maniPoseMatches(sourcePose, source) && !maniPoseIsDifference(sourcePose) &&
           maniPoseMatches(targetPose, target);
}

static MANI_NOINLINE maniResult RetargetPoseWork(const maniRetargetMap* map,
                                                 const maniSkeleton* source,
                                                 const maniPose* sourcePose,
                                                 const maniSkeleton* target, maniPose* targetPose,
                                                 void* scratch, size_t scratchSize)
{
    if (!Usable(map, source, sourcePose, target, targetPose, scratch, scratchSize))
    {
        return mani_errorInvalid;
    }
    maniQuat* sourceModels = scratch;
    Transfer x = {map,
                  source,
                  sourcePose,
                  target,
                  targetPose,
                  sourceModels,
                  sourceModels + map->sourceBoneCount};
    for (uint32_t b = 0; b < map->sourceBoneCount; ++b)
    {
        maniQuat q = maniPoseLocal(sourcePose, b).rotation;
        uint32_t parent = maniSkeletonParent(source, b);
        sourceModels[b] = parent == MANI_NO_BONE ? q : maniQuatMul(sourceModels[parent], q);
    }
    for (uint32_t b = 0; b < map->targetBoneCount; ++b)
    {
        maniPoseSetLocal(targetPose, b, Bone(&x, b));
    }
    maniPoseSetKind(targetPose, false);
    return mani_success;
}

maniResult maniRetargetPose(const maniRetargetMap* map, const maniSkeleton* source,
                            const maniPose* sourcePose, const maniSkeleton* target,
                            maniPose* targetPose, void* scratch, size_t scratchSize)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result =
        RetargetPoseWork(map, source, sourcePose, target, targetPose, scratch, scratchSize);
    maniFloatEnvLeave(env);
    return result;
}
