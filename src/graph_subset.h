// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A subset plan's layout (mani-0035): the bones the plan computes and
// writes (the listed ones, their ancestors and what the graph's nodes
// read to write them, strictly increasing), then per group of a pose's
// bones whether it holds one of them.

#ifndef MAUL_ANIM_SRC_GRAPH_SUBSET_H
#define MAUL_ANIM_SRC_GRAPH_SUBSET_H

#include "maul-anim/graph.h"

#include <stdint.h>

#define MANI_GRAPH_SUBSET_STAMP 0x4D475353u

// The plan; its bones (u32 each) follow it, then a byte per group.
struct maniGraphSubset
{
    uint32_t stamp;
    uint32_t count;
    uint32_t groups;
    uint32_t reserved;
    const maniGraphBinding* binding;
    uint64_t identity;
};

static inline const uint32_t* maniGraphSubsetList(const maniGraphSubset* plan)
{
    return (const uint32_t*)(plan + 1);
}

// A byte per group: 1 where the group holds a planned bone.
static inline const uint8_t* maniGraphSubsetGroups(const maniGraphSubset* plan)
{
    return (const uint8_t*)(maniGraphSubsetList(plan) + plan->count);
}

#endif // MAUL_ANIM_SRC_GRAPH_SUBSET_H
