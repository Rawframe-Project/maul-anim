// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Checking a graph source as its build does: every
// parameter, clip slot, sync group and node on its own; the order and
// the kinds of what nodes give are the build's.

#ifndef MAUL_ANIM_SRC_GRAPH_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_CHECK_H

#include "allocator.h"

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// What a checked source needs written: its inputs, data words, masks and
// names' bytes.
typedef struct maniGraphTally
{
    uint64_t namesSize;
    uint64_t dataWords;
    // The words of every state machine's block in an instance.
    uint64_t machineWords;
    uint32_t inputCount;
    uint32_t maskCount;
    // The most transitions of one machine, the machines that listen and
    // the one-shots, and the longest of their clip lists.
    uint32_t maxTransitions;
    uint32_t listeners;
    uint32_t longestList;
    // The machines with an inertial transition and the motion matching
    // nodes.
    uint32_t inertial;
    // The map slots: one per distinct map the clips name.
    uint32_t mapCount;
    // The clips the databases list.
    uint64_t databaseClips;
} maniGraphTally;

maniBuildResult maniGraphRefuse(maniBuildReason reason, uint32_t item, uint32_t other);

// A source refused for passing a fixed bound (mani_errorLimit), with
// the reason and the item.
maniBuildResult maniGraphPastLimit(maniBuildReason reason, uint32_t item, uint32_t other);

// A map's slot among the count identities of ids, added when new (count
// grows); MANI_MAX_GRAPH_RETARGET_MAPS when ids are full.
uint32_t maniGraphMapSlotIn(uint64_t* ids, uint32_t* count, uint64_t identity);

// Whether a source's node count or a list's count passes its bound.
bool maniGraphListsPast(const maniGraphSource* source);

// The source's parameter, clip, sync group, database, model and spring
// set lists: each within its limit, and given when it is not empty.
bool maniGraphListsValid(const maniGraphSource* source);

// Checks the source's counts, parameters, clips, sync groups, databases
// and nodes;
// Capacity when the allocator fails.
maniBuildResult maniCheckGraphSource(const maniGraphSource* source, const maniAllocator* allocator,
                                     maniGraphTally* tallyOut);

#endif // MAUL_ANIM_SRC_GRAPH_CHECK_H
