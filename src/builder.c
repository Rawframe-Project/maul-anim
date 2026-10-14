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
    if (def->version != MANI_ABI_VERSION)
    {
        return mani_errorVersion;
    }
    if (def->cookie != MANI_BUILDER_DEF_COOKIE || !maniAllocatorValid(&def->allocator))
    {
        return mani_errorInvalid;
    }
    maniBuilder* builder =
        maniAllocate(&def->allocator, 1, sizeof(maniBuilder), alignof(maniBuilder));
    if (builder == NULL)
    {
        return mani_errorCapacity;
    }
    *builder = (maniBuilder){.allocator = def->allocator};
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
    if (builder == NULL)
    {
        return;
    }
    maniBuilderClear(builder);
    maniAllocator allocator = builder->allocator;
    maniRelease(&allocator, builder, 1, sizeof(maniBuilder), alignof(maniBuilder));
}

uint32_t maniBuilderMisuses(const maniBuilder* builder)
{
    return builder->misuses;
}

maniResult maniCopyBuilt(maniBuilder* builder, void* bufferOut, size_t capacity, size_t* sizeOut)
{
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if ((bufferOut == NULL && capacity > 0) || builder->kind == 0)
    {
        ++builder->misuses;
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

maniResult maniCopySkeletonOrder(maniBuilder* builder, uint32_t* boneOut, uint32_t capacity,
                                 uint32_t* countOut)
{
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if ((boneOut == NULL && capacity > 0) || builder->kind != maniKindSkeleton)
    {
        ++builder->misuses;
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = builder->orderCount;
    }
    uint32_t copied = capacity < builder->orderCount ? capacity : builder->orderCount;
    if (copied > 0)
    {
        memcpy(boneOut, builder->order, (size_t)copied * sizeof(uint32_t));
    }
    return copied < builder->orderCount ? mani_errorCapacity : mani_success;
}
