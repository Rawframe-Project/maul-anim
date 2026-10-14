// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The builders' arena: a list of chunks, each at least 64 KiB, filled in
// order, every block at a multiple of 8 (the most any builder array
// needs, and what every platform's C library serves).

#include "arena.h"

#include "allocator.h"

#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

#define MANI_ARENA_CHUNK 65536u

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
    bytes = (bytes + 7) / 8 * 8;
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
    }
    uint8_t* block = (uint8_t*)(chunk + 1) + chunk->used;
    chunk->used += bytes;
    memset(block, 0, bytes);
    return block;
}

void maniArenaRelease(maniArena* arena)
{
    while (arena->chunks != NULL)
    {
        maniArenaChunk* next = arena->chunks->next;
        maniRelease(arena->allocator, arena->chunks, sizeof(maniArenaChunk) + arena->chunks->size,
                    1, 8);
        arena->chunks = next;
    }
}
