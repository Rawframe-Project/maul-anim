// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Allocation through an owner object's allocator (conventions section 7);
// the one place a zeroed allocator reaches the C library.

#ifndef MAUL_ANIM_SRC_ALLOCATOR_H
#define MAUL_ANIM_SRC_ALLOCATOR_H

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>

// True when the allocator is zeroed or has both functions.
bool maniAllocatorValid(const maniAllocator* allocator);

// count * size bytes aligned to alignment (at most that of max_align_t
// for a zeroed allocator), or NULL when the size does not fit or the
// allocator fails. Zero bytes give NULL.
void* maniAllocate(const maniAllocator* allocator, size_t count, size_t size, size_t alignment);

// Frees what maniAllocate gave, with the same count, size and alignment.
void maniRelease(const maniAllocator* allocator, void* block, size_t count, size_t size,
                 size_t alignment);

#endif // MAUL_ANIM_SRC_ALLOCATOR_H
