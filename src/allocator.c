// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "allocator.h"

#include "fpenv.h"
#include "invariant.h"

#include <stdckdint.h>
#include <stdlib.h>

// The host's allocator runs in the host's floating-point environment:
// each call is the only work of a helper of its own, so none of the
// caller's arithmetic moves into it.
static MANI_NOINLINE void* HostAlloc(const maniAllocator* allocator, size_t bytes, size_t alignment)
{
    maniFloatHost host = maniFloatHostEnter();
    void* block = allocator->alloc(bytes, alignment, allocator->context);
    maniFloatHostLeave(host);
    return block;
}

static MANI_NOINLINE void HostFree(const maniAllocator* allocator, void* block, size_t bytes,
                                   size_t alignment)
{
    maniFloatHost host = maniFloatHostEnter();
    allocator->free(block, bytes, alignment, allocator->context);
    maniFloatHostLeave(host);
}

bool maniAllocatorValid(const maniAllocator* allocator)
{
    return (allocator->alloc == NULL) == (allocator->free == NULL);
}

void* maniAllocate(const maniAllocator* allocator, size_t count, size_t size, size_t alignment)
{
    // The library asks for the alignment of a type, a power of two; the
    // host's allocator is promised one.
    MANI_ASSERT(alignment != 0 && (alignment & (alignment - 1)) == 0);
    size_t bytes = 0;
    if (ckd_mul(&bytes, count, size) || bytes == 0)
    {
        return NULL;
    }
    if (allocator->alloc != NULL)
    {
        return HostAlloc(allocator, bytes, alignment);
    }
    return alignment <= alignof(max_align_t) ? malloc(bytes) : NULL;
}

void maniRelease(const maniAllocator* allocator, void* block, size_t count, size_t size,
                 size_t alignment)
{
    if (block == NULL)
    {
        return;
    }
    // A block is released with the count, size and alignment it was
    // allocated with, which the host's free is promised.
    MANI_ASSERT(count != 0 && size != 0 && (alignment & (alignment - 1)) == 0);
    if (allocator->free != NULL)
    {
        HostFree(allocator, block, count * size, alignment);
        return;
    }
    free(block);
}
