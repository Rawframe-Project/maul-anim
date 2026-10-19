// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A subset plan's layout (mani-0035): the bones the plan computes and
// writes (the listed ones, their ancestors and what the graph's nodes
// read to write them, strictly increasing), per group of a pose's bones
// whether it holds one of them, and per node whether it runs (a node that
// writes bones from others runs when it writes a planned bone, and
// otherwise passes its input on).

#ifndef MAUL_ANIM_SRC_GRAPH_SUBSET_H
#define MAUL_ANIM_SRC_GRAPH_SUBSET_H

#include "maul-anim/graph.h"

#include <stdbool.h>
#include <stdint.h>

#define MANI_GRAPH_SUBSET_STAMP 0x4D475353u

// The plan; its bones (u32 each) follow it, then a byte per group; a byte
// per node at nodesAt bytes from its start.
struct maniGraphSubset
{
    uint32_t stamp;
    uint32_t count;
    uint32_t groups;
    uint32_t nodesAt;
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

// Whether node p runs.
static inline bool maniGraphSubsetRuns(const maniGraphSubset* plan, uint32_t p)
{
    return ((const uint8_t*)plan)[plan->nodesAt + p] != 0;
}

#endif // MAUL_ANIM_SRC_GRAPH_SUBSET_H
