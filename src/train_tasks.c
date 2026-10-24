// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Running a trainer's chunks on the host's task hooks.

#include "train_tasks.h"

#include "fpenv.h"

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// A task and its context, run by InEnvironment.
typedef struct Wrapped
{
    maniTaskFn* task;
    void* context;
} Wrapped;

// One range of a wrapped task.
typedef struct Range
{
    const Wrapped* wrapped;
    int32_t start;
    int32_t end;
} Range;

static void RunRange(void* context)
{
    const Range* range = context;
    range->wrapped->task(range->start, range->end, range->wrapped->context);
}

// A task body on a host's thread, which keeps the host's floating-point
// environment: the library's entered around it.
static void InEnvironment(int32_t startIndex, int32_t endIndex, void* taskContext)
{
    Range range = {taskContext, startIndex, endIndex};
    maniFloatEnvRun(RunRange, &range);
}

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
    // On this thread the caller entered the environment already; on the
    // hooks each range enters it. The wrapper lives until finishTask
    // returns, which waits for every range.
    Wrapped wrapped = {task, context};
    void* handle = hooks->enqueueTask(InEnvironment, count, 1, &wrapped, hooks->userContext);
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
