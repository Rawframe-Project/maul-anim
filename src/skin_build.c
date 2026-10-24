// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a skin file: every joint and matrix checked, the
// skeleton's identity and bone count recorded.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "skin_file.h"

#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item)
{
    return (maniBuildResult){result, reason, item, 0};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static maniBuildResult Check(const maniSkinSource* source)
{
    uint32_t n = source->jointCount;
    if (n == 0 || n > MANI_MAX_BONES || source->joints == NULL || source->inverseBinds == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonCount, 0);
    }
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    for (uint32_t j = 0; j < n; ++j)
    {
        if (source->joints[j] >= bones)
        {
            return Result(mani_errorInvalid, maniBuildReasonJoint, j);
        }
        for (uint32_t k = 0; k < 12; ++k)
        {
            if (!Finite(source->inverseBinds[j].m[k]))
            {
                return Result(mani_errorInvalid, maniBuildReasonNotFinite, j);
            }
        }
    }
    return Result(mani_success, maniBuildReasonNone, 0);
}

static MANI_NOINLINE maniBuildResult BuildSkinWork(maniBuilder* builder,
                                                   const maniSkinSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0);
    }
    if (source == NULL || source->skeleton == NULL)
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0);
    }
    maniBuildResult result = Check(source);
    if (result.result != mani_success)
    {
        return result;
    }
    uint32_t n = source->jointCount;
    uint8_t binding[MANI_SKIN_BINDING_SIZE];
    maniStoreU64(binding, maniSkeletonIdentity(source->skeleton));
    maniStoreU32(binding + 8, maniSkeletonBoneCount(source->skeleton));
    maniStoreU32(binding + 12, n);
    size_t jointBytes = 2 * (size_t)n;
    size_t matrixBytes = MANI_SKIN_MATRIX_SIZE * (size_t)n;
    uint8_t* block = maniAllocate(&builder->allocator, jointBytes + matrixBytes, 1, 8);
    if (block == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0);
    }
    for (uint32_t j = 0; j < n; ++j)
    {
        maniStoreU16(block + 2 * (size_t)j, (uint16_t)source->joints[j]);
    }
    memcpy(block + jointBytes, source->inverseBinds, matrixBytes);
    maniSectionSource sections[MANI_SKIN_SECTIONS] = {
        {maniSkinTagBinding, binding, sizeof binding},
        {maniSkinTagJoints, block, jointBytes},
        {maniSkinTagInverseBinds, block + jointBytes, matrixBytes},
    };
    uint64_t identity = maniSkinComputeIdentity(binding, block, block + jointBytes, n);
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, MANI_SKIN_SECTIONS, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindSkin, MANI_SKIN_FORMAT_VERSION, identity, sections,
                           MANI_SKIN_SECTIONS);
    }
    maniRelease(&builder->allocator, block, jointBytes + matrixBytes, 1, 8);
    if (file == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0);
    }
    maniBuilderKeep(builder, maniKindSkin, file, (size_t)fileSize, NULL, 0);
    return result;
}

maniBuildResult maniBuildSkin(maniBuilder* builder, const maniSkinSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildSkinWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
