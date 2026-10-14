// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An arena for the offline builders: blocks taken from an allocator in
// chunks and given back all at once, so a build with many short-lived
// arrays frees them in one place.

#ifndef MAUL_ANIM_SRC_ARENA_H
#define MAUL_ANIM_SRC_ARENA_H

#include "maul-anim/base.h"

#include <stddef.h>

typedef struct maniArenaChunk maniArenaChunk;

typedef struct maniArena
{
    const maniAllocator* allocator;
    maniArenaChunk* chunks;
} maniArena;

static inline maniArena maniArenaMake(const maniAllocator* allocator)
{
    maniArena arena = {allocator, NULL};
    return arena;
}

// count * size bytes, zeroed, aligned to 8; NULL when the size does not
// fit or the allocator fails. A count of 0 gives a valid empty block.
void* maniArenaTake(maniArena* arena, size_t count, size_t size);

// Gives every block back.
void maniArenaRelease(maniArena* arena);

#endif // MAUL_ANIM_SRC_ARENA_H
