// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The host's code behind the library's own functions (mani-0028): each
// marks its object running for the host's call and restores the mark
// after, so a call back in sees it and nested host calls keep it.

#include "host_code.h"

#include "allocator.h"

#include "maul-anim/base.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static bool Mark(maniHostCode* host)
{
    return atomic_exchange_explicit(&host->running, true, memory_order_relaxed);
}

static void Restore(maniHostCode* host, bool was)
{
    atomic_store_explicit(&host->running, was, memory_order_relaxed);
}

static void* Alloc(size_t size, size_t alignment, void* context)
{
    maniHostCode* host = context;
    bool was = Mark(host);
    void* block = maniAllocate(&host->allocator, 1, size, alignment);
    Restore(host, was);
    return block;
}

static void Free(void* memory, size_t size, size_t alignment, void* context)
{
    maniHostCode* host = context;
    bool was = Mark(host);
    maniRelease(&host->allocator, memory, 1, size, alignment);
    Restore(host, was);
}

static void* Enqueue(maniTaskFn* task, int32_t itemCount, int32_t minRange, void* taskContext,
                     void* userContext)
{
    maniHostCode* host = userContext;
    if (host->tasks.enqueueTask == NULL)
    {
        task(0, itemCount, taskContext);
        return NULL;
    }
    bool was = Mark(host);
    void* handle =
        host->tasks.enqueueTask(task, itemCount, minRange, taskContext, host->tasks.userContext);
    Restore(host, was);
    return handle;
}

static void Finish(void* userTask, void* userContext)
{
    maniHostCode* host = userContext;
    if (host->tasks.finishTask == NULL)
    {
        return;
    }
    bool was = Mark(host);
    host->tasks.finishTask(userTask, host->tasks.userContext);
    Restore(host, was);
}

void maniHostCodeInit(maniHostCode* host, const maniAllocator* allocator,
                      const maniTaskHooks* tasks)
{
    host->allocator = allocator != NULL ? *allocator : (maniAllocator){0};
    host->tasks = tasks != NULL ? *tasks : (maniTaskHooks){0};
    atomic_init(&host->running, false);
    atomic_init(&host->misuses, 0u);
}

maniAllocator maniHostAllocator(maniHostCode* host)
{
    return (maniAllocator){Alloc, Free, host};
}

maniTaskHooks maniHostTasks(maniHostCode* host)
{
    return (maniTaskHooks){Enqueue, Finish, host};
}

bool maniHostCalledBack(maniHostCode* host)
{
    if (host == NULL || !atomic_load_explicit(&host->running, memory_order_relaxed))
    {
        return false;
    }
    maniHostCountMisuse(host);
    return true;
}

bool maniHostAllocatorCalledBack(const maniAllocator* allocator)
{
    return allocator->alloc == Alloc && maniHostCalledBack(allocator->context);
}
