// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "builder.h"

#include "allocator.h"
#include "container.h"

#include "maul-anim/build.h"

#include <string.h>

maniResult maniCreateBuilder(const maniBuilderDef* def, maniBuilder** builderOut)
{
    if (builderOut == NULL)
    {
        return mani_errorInvalid;
    }
    *builderOut = NULL;
    if (def == NULL)
    {
        return mani_errorInvalid;
    }
    if (def->cookie != MANI_BUILDER_DEF_COOKIE)
    {
        return mani_errorInvalid;
    }
    if (def->version != MANI_ABI_VERSION)
    {
        return mani_errorVersion;
    }
    if (!maniAllocatorValid(&def->allocator))
    {
        return mani_errorInvalid;
    }
    maniBuilder* builder =
        maniAllocate(&def->allocator, 1, sizeof(maniBuilder), alignof(maniBuilder));
    if (builder == NULL)
    {
        return mani_errorCapacity;
    }
    *builder = (maniBuilder){0};
    maniHostCodeInit(&builder->host, &def->allocator, NULL);
    builder->allocator = maniHostAllocator(&builder->host);
    *builderOut = builder;
    return mani_success;
}

void maniBuilderClear(maniBuilder* builder)
{
    // Builds allocate files and orders 8-byte aligned.
    maniRelease(&builder->allocator, builder->file, builder->fileSize, 1, 8);
    maniRelease(&builder->allocator, builder->order, builder->orderCount, sizeof(uint32_t), 8);
    builder->file = NULL;
    builder->fileSize = 0;
    builder->kind = 0;
    builder->order = NULL;
    builder->orderCount = 0;
}

void maniBuilderKeep(maniBuilder* builder, uint32_t kind, uint8_t* file, size_t fileSize,
                     uint32_t* order, uint32_t orderCount)
{
    maniBuilderClear(builder);
    builder->file = file;
    builder->fileSize = fileSize;
    builder->kind = kind;
    builder->order = order;
    builder->orderCount = orderCount;
}

void maniDestroyBuilder(maniBuilder* builder)
{
    if (builder == NULL || maniBuilderCalledBack(builder))
    {
        return;
    }
    // A call back in from any free below, the block's own among them,
    // finds the builder gone.
    maniHostRetire(&builder->host);
    maniBuilderClear(builder);
    // The block itself straight to the host's: the library's allocator
    // marks the builder after the host's call returns.
    maniAllocator allocator = builder->host.allocator;
    maniRelease(&allocator, builder, 1, sizeof(maniBuilder), alignof(maniBuilder));
}

uint32_t maniBuilder_GetMisuse(const maniBuilder* builder)
{
    return maniHostMisuses(&builder->host);
}

static maniResult CopyBuilt(maniBuilder* builder, void* bufferOut, size_t capacity, size_t* sizeOut)
{
    if (sizeOut != NULL)
    {
        *sizeOut = 0;
    }
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if ((bufferOut == NULL && capacity > 0) || builder->kind == 0)
    {
        maniBuilderMisuse(builder);
        return mani_errorInvalid;
    }
    if (sizeOut != NULL)
    {
        *sizeOut = builder->fileSize;
    }
    if (capacity < builder->fileSize)
    {
        return mani_errorCapacity;
    }
    if (builder->fileSize > 0)
    {
        memcpy(bufferOut, builder->file, builder->fileSize);
    }
    return mani_success;
}

maniResult maniBuilder_CopyBuilt(maniBuilder* builder, void* bufferOut, size_t capacity,
                                 size_t* sizeOut)
{
    if (maniBuilderCalledBack(builder))
    {
        return mani_errorBusy;
    }
    maniResult r = CopyBuilt(builder, bufferOut, capacity, sizeOut);
    if (r != mani_success && bufferOut != NULL)
    {
        memset(bufferOut, 0, capacity);
    }
    return r;
}

static maniResult CopySkeletonOrder(maniBuilder* builder, uint32_t* boneOut, uint32_t capacity,
                                    uint32_t* countOut)
{
    if (countOut != NULL)
    {
        *countOut = 0;
    }
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if ((boneOut == NULL && capacity > 0) || builder->kind != maniKindSkeleton)
    {
        maniBuilderMisuse(builder);
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = builder->orderCount;
    }
    if (capacity < builder->orderCount)
    {
        return mani_errorCapacity;
    }
    if (builder->orderCount > 0)
    {
        memcpy(boneOut, builder->order, (size_t)builder->orderCount * sizeof(uint32_t));
    }
    return mani_success;
}

maniResult maniBuilder_CopySkeletonOrder(maniBuilder* builder, uint32_t* boneOut, uint32_t capacity,
                                         uint32_t* countOut)
{
    if (maniBuilderCalledBack(builder))
    {
        return mani_errorBusy;
    }
    maniResult r = CopySkeletonOrder(builder, boneOut, capacity, countOut);
    if (r != mani_success && boneOut != NULL)
    {
        memset(boneOut, 0, (size_t)capacity * sizeof(uint32_t));
    }
    return r;
}
