// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A pose's memory: a 64-byte header, then the local transforms per
// component in groups of eight bones (translation x, y, z, rotation x, y,
// z, w, scale x, y, z: ten runs of eight floats a group), then the model
// matrices, twelve floats a bone. Padding bones in the last group hold
// the identity, so passes over whole groups read finite values.

#include "maul-anim/pose.h"

#include "bytes.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <string.h>

enum
{
    kHeaderSize = 64,
    kGroupBones = 8,
    kComponents = 10,
    kGroupFloats = kGroupBones * kComponents,
    kModelFloats = 12,
    // "MPOS": marks memory maniInitPose made.
    kPoseTag = 0x534F504Du,
};

struct maniPose
{
    uint32_t tag;
    uint32_t boneCount;
    uint32_t groups;
    uint32_t modelCurrent;
    uint64_t identity;
    uint8_t reserved[kHeaderSize - 24];
};

static_assert(sizeof(struct maniPose) == kHeaderSize, "the header is 64 bytes");

static const maniTransform s_identity = {
    {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};

static float* Locals(maniPose* pose)
{
    return (float*)((uint8_t*)pose + kHeaderSize);
}

static const float* ConstLocals(const maniPose* pose)
{
    return (const float*)((const uint8_t*)pose + kHeaderSize);
}

static float* Models(maniPose* pose)
{
    return Locals(pose) + (size_t)pose->groups * kGroupFloats;
}

static const float* ConstModels(const maniPose* pose)
{
    return ConstLocals(pose) + (size_t)pose->groups * kGroupFloats;
}

static void StoreLocal(float* locals, uint32_t bone, const maniTransform* t)
{
    float* group = locals + (size_t)(bone / kGroupBones) * kGroupFloats + bone % kGroupBones;
    const float values[kComponents] = {
        t->translation.x, t->translation.y, t->translation.z, t->rotation.x, t->rotation.y,
        t->rotation.z,    t->rotation.w,    t->scale.x,       t->scale.y,    t->scale.z};
    for (uint32_t c = 0; c < kComponents; ++c)
    {
        group[c * kGroupBones] = values[c];
    }
}

static maniTransform LoadLocal(const float* locals, uint32_t bone)
{
    const float* g = locals + (size_t)(bone / kGroupBones) * kGroupFloats + bone % kGroupBones;
    return (maniTransform){
        {g[0], g[8], g[16]}, {g[24], g[32], g[40], g[48]}, {g[56], g[64], g[72]}};
}

size_t maniPoseSize(const maniSkeleton* skeleton)
{
    size_t bones = maniSkeletonBoneCount(skeleton);
    size_t groups = (bones + kGroupBones - 1) / kGroupBones;
    size_t size =
        kHeaderSize + groups * kGroupFloats * sizeof(float) + bones * kModelFloats * sizeof(float);
    return (size + MANI_POSE_ALIGNMENT - 1) / MANI_POSE_ALIGNMENT * MANI_POSE_ALIGNMENT;
}

bool maniPoseMatches(const maniPose* pose, const maniSkeleton* skeleton)
{
    return pose != NULL && skeleton != NULL && pose->tag == kPoseTag &&
           pose->identity == skeleton->identity && pose->boneCount == skeleton->boneCount;
}

maniResult maniInitPose(const maniSkeleton* skeleton, void* memory, size_t size, maniPose** poseOut)
{
    if (skeleton == NULL || memory == NULL || poseOut == NULL ||
        (uintptr_t)memory % MANI_POSE_ALIGNMENT != 0)
    {
        return mani_errorInvalid;
    }
    if (size < maniPoseSize(skeleton))
    {
        return mani_errorCapacity;
    }
    maniPose* pose = memory;
    uint32_t bones = maniSkeletonBoneCount(skeleton);
    *pose = (maniPose){
        .tag = kPoseTag,
        .boneCount = bones,
        .groups = (bones + kGroupBones - 1) / kGroupBones,
        .identity = maniSkeletonIdentity(skeleton),
    };
    // The padding bones of the last group.
    for (uint32_t bone = bones; bone < pose->groups * kGroupBones; ++bone)
    {
        StoreLocal(Locals(pose), bone, &s_identity);
    }
    maniResult result = maniPoseSetRest(pose, skeleton);
    *poseOut = pose;
    return result;
}

uint32_t maniPoseBoneCount(const maniPose* pose)
{
    return pose->boneCount;
}

maniTransform maniPoseLocal(const maniPose* pose, uint32_t bone)
{
    return bone < pose->boneCount ? LoadLocal(ConstLocals(pose), bone) : s_identity;
}

void maniPoseSetLocal(maniPose* pose, uint32_t bone, maniTransform transform)
{
    if (bone < pose->boneCount)
    {
        StoreLocal(Locals(pose), bone, &transform);
        pose->modelCurrent = 0;
    }
}

static bool RangeFits(const maniPose* pose, uint32_t first, uint32_t count)
{
    return first <= pose->boneCount && count <= pose->boneCount - first;
}

maniResult maniPoseReadLocals(const maniPose* pose, uint32_t first, uint32_t count,
                              maniTransform* transforms)
{
    if (pose == NULL || (transforms == NULL && count > 0) || !RangeFits(pose, first, count))
    {
        return mani_errorInvalid;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        transforms[i] = LoadLocal(ConstLocals(pose), first + i);
    }
    return mani_success;
}

maniResult maniPoseWriteLocals(maniPose* pose, uint32_t first, uint32_t count,
                               const maniTransform* transforms)
{
    if (pose == NULL || (transforms == NULL && count > 0) || !RangeFits(pose, first, count))
    {
        return mani_errorInvalid;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        StoreLocal(Locals(pose), first + i, &transforms[i]);
    }
    if (count > 0)
    {
        pose->modelCurrent = 0;
    }
    return mani_success;
}

maniResult maniPoseSetRest(maniPose* pose, const maniSkeleton* skeleton)
{
    if (!maniPoseMatches(pose, skeleton))
    {
        return mani_errorInvalid;
    }
    for (uint32_t bone = 0; bone < pose->boneCount; ++bone)
    {
        maniTransform rest = maniSkeletonRest(skeleton, bone);
        StoreLocal(Locals(pose), bone, &rest);
    }
    return maniComputeModel(pose, skeleton);
}

maniResult maniComputeModel(maniPose* pose, const maniSkeleton* skeleton)
{
    if (!maniPoseMatches(pose, skeleton))
    {
        return mani_errorInvalid;
    }
    const float* locals = Locals(pose);
    float* models = Models(pose);
    for (uint32_t bone = 0; bone < pose->boneCount; ++bone)
    {
        maniTransform t = LoadLocal(locals, bone);
        maniAffine local = maniAffineFromTrs(t.translation, t.rotation, t.scale);
        uint32_t parent = maniLoadU16(skeleton->parents + 2 * (size_t)bone);
        maniAffine model = local;
        if (parent != MANI_NO_BONE)
        {
            maniAffine parentModel;
            memcpy(parentModel.m, models + (size_t)parent * kModelFloats, sizeof parentModel.m);
            model = maniAffineMul(&parentModel, &local);
        }
        memcpy(models + (size_t)bone * kModelFloats, model.m, sizeof model.m);
    }
    pose->modelCurrent = 1;
    return mani_success;
}

const float* maniPoseModelData(const maniPose* pose)
{
    return ConstModels(pose);
}

bool maniPoseModelCurrent(const maniPose* pose)
{
    return pose->modelCurrent != 0;
}

maniAffine maniPoseModel(const maniPose* pose, uint32_t bone)
{
    maniAffine model = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}};
    if (bone < pose->boneCount)
    {
        memcpy(model.m, ConstModels(pose) + (size_t)bone * kModelFloats, sizeof model.m);
    }
    return model;
}

maniResult maniPoseReadModels(const maniPose* pose, uint32_t first, uint32_t count,
                              maniAffine* matrices)
{
    if (pose == NULL || (matrices == NULL && count > 0) || !RangeFits(pose, first, count))
    {
        return mani_errorInvalid;
    }
    if (count > 0)
    {
        memcpy(matrices, ConstModels(pose) + (size_t)first * kModelFloats,
               (size_t)count * sizeof(maniAffine));
    }
    return mani_success;
}
