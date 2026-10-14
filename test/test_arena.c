// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The arena (src/arena.c): blocks zeroed and 8-aligned, apart from each
// other across chunks, a block larger than a chunk, every chunk given
// back, and sizes that overflow refused.

#include "arena.h"
#include "test_harness.h"

#include "maul-anim/base.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Counts what the arena holds: allocations minus releases, in bytes.
static size_t s_held;

static void* Allocate(size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    s_held += size;
    return malloc(size);
}

static void Release(void* memory, size_t size, size_t alignment, void* context)
{
    (void)alignment;
    (void)context;
    s_held -= size;
    free(memory);
}

int main(void)
{
    maniAllocator allocator = {Allocate, Release, NULL};
    maniArena arena = maniArenaMake(&allocator);
    // Three blocks of 30000 bytes: the third needs a second chunk.
    uint8_t* blocks[3];
    for (int i = 0; i < 3; ++i)
    {
        blocks[i] = maniArenaTake(&arena, 30000, 1);
        CHECK(blocks[i] != NULL && ((uintptr_t)blocks[i] & 7u) == 0, "8-aligned");
        bool zero = true;
        for (size_t k = 0; blocks[i] != NULL && k < 30000; ++k)
        {
            zero = zero && blocks[i][k] == 0;
        }
        CHECK(zero, "zeroed");
        if (blocks[i] != NULL)
        {
            memset(blocks[i], 'a' + i, 30000);
        }
    }
    uint8_t* odd = maniArenaTake(&arena, 3, 1);
    uint8_t* next = maniArenaTake(&arena, 1, 8);
    CHECK(odd != NULL && next != NULL && ((uintptr_t)next & 7u) == 0, "sizes rounded to 8");
    // Larger than a chunk.
    uint8_t* large = maniArenaTake(&arena, 100000, 1);
    CHECK(large != NULL, "a large block");
    if (large != NULL)
    {
        memset(large, 'z', 100000);
    }
    bool kept = true;
    for (int i = 0; i < 3; ++i)
    {
        for (size_t k = 0; blocks[i] != NULL && k < 30000; ++k)
        {
            kept = kept && blocks[i][k] == (uint8_t)('a' + i);
        }
    }
    CHECK(kept, "no block overlaps another");
    CHECK(maniArenaTake(&arena, SIZE_MAX / 2, 4) == NULL, "an overflowing size");
    CHECK(maniArenaTake(&arena, SIZE_MAX - 8, 1) == NULL, "a size near the limit");
    maniArenaRelease(&arena);
    CHECK(s_held == 0 && arena.chunks == NULL, "every chunk released");
    return s_failures == 0 ? 0 : 1;
}
