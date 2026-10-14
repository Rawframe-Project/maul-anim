// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node forest for the importers (mani-0009, mani-0014): depths, common
// ancestors, and the skeletons that seed sets sharing nodes make. Every
// step is linear or n log n in the forest's size, whatever its shape.

#ifndef MAUL_ANIM_SRC_HIERARCHY_H
#define MAUL_ANIM_SRC_HIERARCHY_H

#include "allocator.h"

#include "maul-anim/base.h"

#include <stdint.h>

// No node: a top node's parent, or nodes of different trees' ancestor.
#define MANI_NO_NODE 0xFFFFFFFFu

typedef struct maniHierarchy
{
    const uint32_t* parents;
    uint32_t count;
    uint32_t* depths;
    // Binary lifting: ancestors[k * count + v] is v's 2^k-th ancestor.
    uint32_t* ancestors;
    uint32_t levels;
} maniHierarchy;

// Depths and ancestor tables for the forest the parents give (each below
// count or MANI_NO_NODE). mani_errorFormat, with *cycleOut a node on it,
// for a cycle; mani_errorCapacity when the allocator fails.
maniResult maniHierarchyInit(maniHierarchy* h, const maniAllocator* allocator,
                             const uint32_t* parents, uint32_t count, uint32_t* cycleOut);

void maniHierarchyRelease(maniHierarchy* h, const maniAllocator* allocator);

// The deepest common ancestor of two nodes (either may be the other), or
// MANI_NO_NODE for nodes of different trees.
uint32_t maniCommonAncestor(const maniHierarchy* h, uint32_t a, uint32_t b);

// Units' seeds: unit u's are seeds[starts[u]] to seeds[starts[u + 1]],
// its members the nodes on the paths from them up to roots[u] (an
// ancestor of each).
typedef struct maniHierarchyUnits
{
    const uint32_t* seeds;
    const uint32_t* starts;
    const uint32_t* roots;
    uint32_t count;
} maniHierarchyUnits;

// Groups units whose members share a node, each node claimed once and a
// walk meeting another unit's node jumping past that unit's root. Groups
// are numbered by their lowest unit: groupOf[u] is u's, groupRoots[g]
// the common ancestor of its units' roots, *groupCountOut their count
// (groupOf and groupRoots hold a word per unit).
maniResult maniHierarchyGroup(const maniHierarchy* h, const maniAllocator* allocator,
                              const maniHierarchyUnits* units, uint32_t* groupOf,
                              uint32_t* groupRoots, uint32_t* groupCountOut);

#endif // MAUL_ANIM_SRC_HIERARCHY_H
