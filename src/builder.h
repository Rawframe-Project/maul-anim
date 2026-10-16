// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The builder's state, shared by the builds of each kind of file.

#ifndef MAUL_ANIM_SRC_BUILDER_H
#define MAUL_ANIM_SRC_BUILDER_H

#include "maul-anim/build.h"

#include <stddef.h>
#include <stdint.h>

// The kind of a kept text (a JSON form), not a file: no container kind.
#define MANI_BUILT_TEXT 0x100u

struct maniBuilder
{
    maniAllocator allocator;
    // The last built file or text, and its kind (0 for none).
    uint8_t* file;
    size_t fileSize;
    uint32_t kind;
    uint32_t misuses;
    // For a skeleton: where each source bone went.
    uint32_t* order;
    uint32_t orderCount;
};

// Drops the file and order the builder holds.
void maniBuilderClear(maniBuilder* builder);

// Takes ownership of a file of a kind (and of an order, which may be
// NULL), in place of what the builder held.
void maniBuilderKeep(maniBuilder* builder, uint32_t kind, uint8_t* file, size_t fileSize,
                     uint32_t* order, uint32_t orderCount);

#endif // MAUL_ANIM_SRC_BUILDER_H
