// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a skin file and composing its palette. A skin records the
// skeleton it was built for by identity and bone count, so its joints are
// checked against that count at open and a palette needs one comparison
// to know the skeleton is the right one.

#include "maul-anim/skin.h"

#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "pose_memory.h"
#include "skin_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"

#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

uint64_t maniSkinComputeIdentity(const uint8_t* binding, const uint8_t* joints,
                                 const uint8_t* inverseBinds, uint32_t jointCount)
{
    uint64_t hash = maniHash64(MANI_HASH_INIT, binding, MANI_SKIN_BINDING_SIZE);
    hash = maniHashBytes(hash, joints, 2 * (uint64_t)jointCount);
    return maniHashBytes(hash, inverseBinds, MANI_SKIN_MATRIX_SIZE * (uint64_t)jointCount);
}

static MANI_NOINLINE maniFileResult OpenSkinWork(const void* bytes, size_t size, maniSkin* skinOut)
{
    if (skinOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindSkin, MANI_SKIN_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    if (c.sectionCount != MANI_SKIN_SECTIONS)
    {
        return Fail(12);
    }
    const uint8_t* data[MANI_SKIN_SECTIONS];
    uint64_t sizes[MANI_SKIN_SECTIONS];
    uint64_t offsets[MANI_SKIN_SECTIONS];
    for (uint32_t i = 0; i < MANI_SKIN_SECTIONS; ++i)
    {
        if (!maniContainerSection(&c, i + 1, &data[i], &sizes[i], &offsets[i]))
        {
            return Fail(MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE);
        }
    }
    if (sizes[0] != MANI_SKIN_BINDING_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + 16);
    }
    uint32_t bones = maniLoadU32(data[0] + 8);
    uint32_t joints = maniLoadU32(data[0] + 12);
    if (bones == 0 || bones > MANI_MAX_BONES)
    {
        return Fail(offsets[0] + 8);
    }
    if (joints == 0 || joints > MANI_MAX_BONES)
    {
        return Fail(offsets[0] + 12);
    }
    if (sizes[1] != 2 * (uint64_t)joints)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + MANI_CONTAINER_ENTRY_SIZE + 16);
    }
    if (sizes[2] != MANI_SKIN_MATRIX_SIZE * (uint64_t)joints)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + 2 * MANI_CONTAINER_ENTRY_SIZE + 16);
    }
    for (uint32_t j = 0; j < joints; ++j)
    {
        if (maniLoadU16(data[1] + 2 * (uint64_t)j) >= bones)
        {
            return Fail(offsets[1] + 2 * (uint64_t)j);
        }
    }
    for (uint64_t k = 0; k < 12 * (uint64_t)joints; ++k)
    {
        float value = 0.0f;
        memcpy(&value, data[2] + 4 * k, sizeof value);
        if (!Finite(value))
        {
            return Fail(offsets[2] + 4 * k);
        }
    }
    if (maniSkinComputeIdentity(data[0], data[1], data[2], joints) != c.identity)
    {
        return Fail(40);
    }
    *skinOut = (maniSkin){
        .joints = data[1],
        .inverseBinds = data[2],
        .identity = c.identity,
        .skeletonIdentity = maniLoadU64(data[0]),
        .skeletonBoneCount = bones,
        .jointCount = joints,
    };
    return (maniFileResult){mani_success, 0, 0};
}

maniFileResult maniOpenSkin(const void* bytes, size_t size, maniSkin* skinOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniFileResult result = OpenSkinWork(bytes, size, skinOut);
    maniFloatEnvLeave(&env);
    return result;
}

uint32_t maniSkinJointCount(const maniSkin* skin)
{
    return skin->jointCount;
}

uint64_t maniSkinSkeletonIdentity(const maniSkin* skin)
{
    return skin->skeletonIdentity;
}

uint64_t maniSkinIdentity(const maniSkin* skin)
{
    return skin->identity;
}

uint32_t maniSkinJoint(const maniSkin* skin, uint32_t joint)
{
    return joint < skin->jointCount ? maniLoadU16(skin->joints + 2 * (size_t)joint) : MANI_NO_BONE;
}

maniAffine maniSkinInverseBind(const maniSkin* skin, uint32_t joint)
{
    maniAffine m = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}};
    if (joint < skin->jointCount)
    {
        memcpy(m.m, skin->inverseBinds + MANI_SKIN_MATRIX_SIZE * (size_t)joint, sizeof m.m);
    }
    return m;
}

static MANI_NOINLINE maniResult ComputePaletteWork(const maniPose* pose,
                                                   const maniSkeleton* skeleton,
                                                   const maniSkin* skin, maniAffine* palette,
                                                   uint32_t capacity, uint32_t* countOut)
{
    if (skin == NULL || (palette == NULL && capacity > 0) || !maniPoseMatches(pose, skeleton) ||
        skin->skeletonIdentity != skeleton->identity ||
        skin->skeletonBoneCount != skeleton->boneCount || !maniPoseModelCurrent(pose) ||
        maniPoseIsDifference(pose))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = skin->jointCount;
    }
    if (capacity < skin->jointCount)
    {
        return mani_errorCapacity;
    }
    const float* models = maniPoseModelData(pose);
    for (uint32_t j = 0; j < skin->jointCount; ++j)
    {
        uint32_t bone = maniLoadU16(skin->joints + 2 * (size_t)j);
        maniAffine model;
        maniAffine inverseBind;
        memcpy(model.m, models + 12 * (size_t)bone, sizeof model.m);
        memcpy(inverseBind.m, skin->inverseBinds + MANI_SKIN_MATRIX_SIZE * (size_t)j,
               sizeof inverseBind.m);
        palette[j] = maniAffineMul(&model, &inverseBind);
    }
    return mani_success;
}

maniResult maniComputePalette(const maniPose* pose, const maniSkeleton* skeleton,
                              const maniSkin* skin, maniAffine* palette, uint32_t capacity,
                              uint32_t* countOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = ComputePaletteWork(pose, skeleton, skin, palette, capacity, countOut);
    maniFloatEnvLeave(&env);
    return result;
}
