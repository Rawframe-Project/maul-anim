// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inlining a graph source's subgraphs at build (mani-0023): each
// subgraph node replaced by the subgraph's output, its other nodes
// appended, its parameters read through the node's map, its clips and
// databases joined by identity and its sync groups by name.

#ifndef MAUL_ANIM_SRC_GRAPH_INLINE_H
#define MAUL_ANIM_SRC_GRAPH_INLINE_H

#include "allocator.h"

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stddef.h>
#include <stdint.h>

// The deepest subgraphs nest.
#define MANI_GRAPH_MAX_DEPTH 16u

// A source with its subgraphs inlined, and the block holding its arrays
// (NULL when the source had none, the source then used as it is).
typedef struct maniGraphFlat
{
    maniGraphSource source;
    void* block;
    size_t blockSize;
} maniGraphFlat;

maniBuildResult maniInlineSubgraphs(const maniGraphSource* source, const maniAllocator* allocator,
                                    maniGraphFlat* out);

void maniReleaseInlined(maniGraphFlat* flat, const maniAllocator* allocator);

#endif // MAUL_ANIM_SRC_GRAPH_INLINE_H
