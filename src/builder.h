// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The builder's state, shared by the builds of each kind of file.

#ifndef MAUL_ANIM_SRC_BUILDER_H
#define MAUL_ANIM_SRC_BUILDER_H

#include "host_code.h"

#include "maul-anim/build.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The kind of a kept text (a JSON form), not a file: no container kind.
#define MANI_BUILT_TEXT 0x100u

struct maniBuilder
{
    // The allocator the builds use: the host's, the builder marked while
    // it runs.
    maniAllocator allocator;
    // The host's allocator, the mark and the misuses.
    maniHostCode host;
    // The last built file or text, and its kind (0 for none).
    uint8_t* file;
    size_t fileSize;
    uint32_t kind;
    // For a skeleton: where each source bone went.
    uint32_t* order;
    uint32_t orderCount;
};

// Counts one misuse on the builder.
static inline void maniBuilderMisuse(maniBuilder* builder)
{
    maniHostCountMisuse(&builder->host);
}

// Whether a builder function is called back in from the builder's
// allocator: then one misuse is counted and the function refuses with
// mani_errorState. A NULL builder is not.
static inline bool maniBuilderCalledBack(maniBuilder* builder)
{
    return builder != NULL && maniHostCalledBack(&builder->host);
}

// A build refused for a call back in.
static inline maniBuildResult maniBuildCalledBack(void)
{
    return (maniBuildResult){mani_errorState, maniBuildReasonNone, 0, 0};
}

// Drops the file and order the builder holds.
void maniBuilderClear(maniBuilder* builder);

// Takes ownership of a file of a kind (and of an order, which may be
// NULL), in place of what the builder held.
void maniBuilderKeep(maniBuilder* builder, uint32_t kind, uint8_t* file, size_t fileSize,
                     uint32_t* order, uint32_t orderCount);

#endif // MAUL_ANIM_SRC_BUILDER_H
