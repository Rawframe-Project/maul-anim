// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Running a trainer's chunks on the host's task hooks (mani-0025).

#include "train_tasks.h"

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

void maniTrainRun(const maniTaskHooks* hooks, maniTaskFn* task, int32_t count, void* context)
{
    if (count <= 0)
    {
        return;
    }
    if (hooks == NULL || hooks->enqueueTask == NULL)
    {
        task(0, count, context);
        return;
    }
    void* handle = hooks->enqueueTask(task, count, 1, context, hooks->userContext);
    if (handle != NULL && hooks->finishTask != NULL)
    {
        hooks->finishTask(handle, hooks->userContext);
    }
}

void maniTrainSum(const float* chunks, uint32_t chunkCount, uint64_t count, float* sum)
{
    memcpy(sum, chunks, sizeof(float) * (size_t)count);
    for (uint32_t c = 1; c < chunkCount; ++c)
    {
        const float* chunk = chunks + (size_t)c * count;
        for (uint64_t k = 0; k < count; ++k)
        {
            sum[k] += chunk[k];
        }
    }
}
