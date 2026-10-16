// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The builders' arena: a list of chunks, each at least 64 KiB, filled in
// order, every block at a multiple of 8 (the most any builder array
// needs, and what every platform's C library serves). Under
// AddressSanitizer a chunk is poisoned but for the blocks taken, each
// followed by a redzone, so reading or writing past a block is reported
// as it is for a block of its own.

#include "arena.h"

#include "allocator.h"

#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

#define MANI_ARENA_CHUNK 65536u

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define MANI_ARENA_ASAN 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) && !defined(MANI_ARENA_ASAN)
#define MANI_ARENA_ASAN 1
#endif

#ifdef MANI_ARENA_ASAN
#include <sanitizer/asan_interface.h>
#define MANI_ARENA_REDZONE        16u
#define MANI_ARENA_POISON(p, n)   ASAN_POISON_MEMORY_REGION((p), (n))
#define MANI_ARENA_UNPOISON(p, n) ASAN_UNPOISON_MEMORY_REGION((p), (n))
#else
#define MANI_ARENA_REDZONE        0u
#define MANI_ARENA_POISON(p, n)   ((void)(p), (void)(n))
#define MANI_ARENA_UNPOISON(p, n) ((void)(p), (void)(n))
#endif

struct maniArenaChunk
{
    maniArenaChunk* next;
    size_t size;
    size_t used;
    // Keeps the data after the header at a multiple of 8 on 32-bit
    // targets too.
    size_t pad;
};

void* maniArenaTake(maniArena* arena, size_t count, size_t size)
{
    size_t bytes = 0;
    if (ckd_mul(&bytes, count, size) || bytes > SIZE_MAX - 64)
    {
        return NULL;
    }
    size_t given = bytes;
    bytes = (bytes + 7) / 8 * 8 + MANI_ARENA_REDZONE;
    maniArenaChunk* chunk = arena->chunks;
    if (chunk == NULL || chunk->size - chunk->used < bytes)
    {
        size_t data = bytes > MANI_ARENA_CHUNK ? bytes : MANI_ARENA_CHUNK;
        size_t total = sizeof(maniArenaChunk) + data;
        chunk = maniAllocate(arena->allocator, total, 1, 8);
        if (chunk == NULL)
        {
            return NULL;
        }
        chunk->next = arena->chunks;
        chunk->size = data;
        chunk->used = 0;
        arena->chunks = chunk;
        MANI_ARENA_POISON(chunk + 1, data);
    }
    uint8_t* block = (uint8_t*)(chunk + 1) + chunk->used;
    chunk->used += bytes;
    MANI_ARENA_UNPOISON(block, given);
    memset(block, 0, given);
    return block;
}

void maniArenaRelease(maniArena* arena)
{
    while (arena->chunks != NULL)
    {
        maniArenaChunk* next = arena->chunks->next;
        MANI_ARENA_UNPOISON(arena->chunks + 1, arena->chunks->size);
        maniRelease(arena->allocator, arena->chunks, sizeof(maniArenaChunk) + arena->chunks->size,
                    1, 8);
        arena->chunks = next;
    }
}
