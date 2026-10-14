// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node forest (mani-0009, mani-0014): depths by walks that stop at
// known nodes, common ancestors by binary lifting, and units grouped by
// walks in which each node is claimed once and a walk that meets another
// unit's node jumps past that unit's root, whose path from there that
// unit already holds.

#include "hierarchy.h"

#include "allocator.h"

#include <string.h>

#define UNKNOWN_DEPTH 0xFFFFFFFEu
#define WALKING       0xFFFFFFFDu

// count words, at least one so that 0 still allocates.
static uint32_t* Words(const maniAllocator* a, size_t count)
{
    return maniAllocate(a, count + 1, sizeof(uint32_t), 8);
}

static void Free(const maniAllocator* a, uint32_t* words, size_t count)
{
    maniRelease(a, words, count + 1, sizeof(uint32_t), 8);
}

static maniResult ReadDepths(maniHierarchy* h, uint32_t* stack, uint32_t* maxDepthOut,
                             uint32_t* cycleOut)
{
    uint32_t* depth = h->depths;
    uint32_t maxDepth = 0;
    for (uint32_t i = 0; i < h->count; ++i)
    {
        depth[i] = UNKNOWN_DEPTH;
    }
    for (uint32_t i = 0; i < h->count; ++i)
    {
        uint32_t height = 0;
        uint32_t v = i;
        while (v != MANI_NO_NODE && depth[v] == UNKNOWN_DEPTH)
        {
            depth[v] = WALKING;
            stack[height++] = v;
            v = h->parents[v];
        }
        if (v != MANI_NO_NODE && depth[v] == WALKING)
        {
            *cycleOut = v;
            return mani_errorFormat;
        }
        uint32_t d = v == MANI_NO_NODE ? 0 : depth[v] + 1;
        while (height > 0)
        {
            depth[stack[--height]] = d++;
        }
        maxDepth = d > 0 && d - 1 > maxDepth ? d - 1 : maxDepth;
    }
    *maxDepthOut = maxDepth;
    return mani_success;
}

static maniResult BuildAncestors(maniHierarchy* h, const maniAllocator* allocator,
                                 uint32_t maxDepth)
{
    uint32_t levels = 1;
    while (levels < 32 && ((uint64_t)1 << levels) <= maxDepth)
    {
        ++levels;
    }
    size_t n = h->count;
    h->ancestors = Words(allocator, (size_t)levels * n);
    if (h->ancestors == NULL)
    {
        return mani_errorCapacity;
    }
    h->levels = levels;
    uint32_t* up = h->ancestors;
    if (n > 0)
    {
        memcpy(up, h->parents, n * sizeof(uint32_t));
    }
    for (uint32_t k = 1; k < levels; ++k)
    {
        for (size_t v = 0; v < n; ++v)
        {
            uint32_t half = up[(k - 1) * n + v];
            up[k * n + v] = half == MANI_NO_NODE ? MANI_NO_NODE : up[(k - 1) * n + half];
        }
    }
    return mani_success;
}

maniResult maniHierarchyInit(maniHierarchy* h, const maniAllocator* allocator,
                             const uint32_t* parents, uint32_t count, uint32_t* cycleOut)
{
    *h = (maniHierarchy){parents, count, Words(allocator, count), NULL, 0};
    uint32_t* stack = Words(allocator, count);
    maniResult r = mani_errorCapacity;
    if (h->depths != NULL && stack != NULL)
    {
        uint32_t maxDepth = 0;
        r = ReadDepths(h, stack, &maxDepth, cycleOut);
        r = r == mani_success ? BuildAncestors(h, allocator, maxDepth) : r;
    }
    Free(allocator, stack, count);
    return r;
}

void maniHierarchyRelease(maniHierarchy* h, const maniAllocator* allocator)
{
    Free(allocator, h->depths, h->count);
    Free(allocator, h->ancestors, (size_t)h->levels * h->count);
    *h = (maniHierarchy){0};
}

uint32_t maniCommonAncestor(const maniHierarchy* h, uint32_t a, uint32_t b)
{
    const uint32_t* up = h->ancestors;
    size_t n = h->count;
    if (h->depths[a] < h->depths[b])
    {
        uint32_t swap = a;
        a = b;
        b = swap;
    }
    uint32_t lift = h->depths[a] - h->depths[b];
    for (uint32_t k = 0; lift != 0; ++k, lift >>= 1)
    {
        a = (lift & 1u) != 0 ? up[k * n + a] : a;
    }
    if (a == b)
    {
        return a;
    }
    for (uint32_t k = h->levels; k-- > 0;)
    {
        if (up[k * n + a] != up[k * n + b])
        {
            a = up[k * n + a];
            b = up[k * n + b];
        }
    }
    return h->parents[a];
}

static uint32_t Find(uint32_t* sets, uint32_t s)
{
    while (sets[s] != s)
    {
        sets[s] = sets[sets[s]];
        s = sets[s];
    }
    return s;
}

// Joins two sets under the lower unit, so a set's representative is its
// lowest unit.
static void Join(uint32_t* sets, uint32_t a, uint32_t b)
{
    a = Find(sets, a);
    b = Find(sets, b);
    if (a != b)
    {
        sets[a > b ? a : b] = a < b ? a : b;
    }
}

// One unit's walks from its seeds up to its root: unclaimed nodes are
// claimed; a node another unit claimed joins the two and the walk jumps
// past that unit's root, whose path from there it already holds.
static void GroupUnit(const maniHierarchy* h, const maniHierarchyUnits* units, uint32_t unit,
                      uint32_t* owner, uint32_t* stamp, uint32_t* sets)
{
    uint32_t top = h->depths[units->roots[unit]];
    for (uint32_t s = units->starts[unit]; s < units->starts[unit + 1]; ++s)
    {
        uint32_t v = units->seeds[s];
        while (v != MANI_NO_NODE && h->depths[v] >= top && stamp[v] != unit)
        {
            stamp[v] = unit;
            uint32_t other = owner[v];
            if (other == MANI_NO_NODE)
            {
                owner[v] = unit;
                v = h->parents[v];
                continue;
            }
            Join(sets, unit, other);
            uint32_t otherRoot = units->roots[other];
            v = h->depths[otherRoot] <= top ? MANI_NO_NODE : h->parents[otherRoot];
        }
    }
}

maniResult maniHierarchyGroup(const maniHierarchy* h, const maniAllocator* allocator,
                              const maniHierarchyUnits* units, uint32_t* groupOf,
                              uint32_t* groupRoots, uint32_t* groupCountOut)
{
    uint32_t* sets = Words(allocator, units->count);
    uint32_t* owner = Words(allocator, h->count);
    uint32_t* stamp = Words(allocator, h->count);
    maniResult r = mani_errorCapacity;
    *groupCountOut = 0;
    if (sets != NULL && owner != NULL && stamp != NULL)
    {
        for (uint32_t u = 0; u < units->count; ++u)
        {
            sets[u] = u;
        }
        for (uint32_t v = 0; v < h->count; ++v)
        {
            owner[v] = MANI_NO_NODE;
            stamp[v] = MANI_NO_NODE;
        }
        for (uint32_t u = 0; u < units->count; ++u)
        {
            GroupUnit(h, units, u, owner, stamp, sets);
        }
        // Groups in the order of their lowest units; each root the
        // common ancestor of its units' roots.
        for (uint32_t u = 0; u < units->count; ++u)
        {
            uint32_t lowest = Find(sets, u);
            if (lowest == u)
            {
                groupRoots[*groupCountOut] = units->roots[u];
                groupOf[u] = (*groupCountOut)++;
                continue;
            }
            uint32_t g = groupOf[lowest];
            groupOf[u] = g;
            groupRoots[g] = maniCommonAncestor(h, groupRoots[g], units->roots[u]);
        }
        r = mani_success;
    }
    Free(allocator, sets, units->count);
    Free(allocator, owner, h->count);
    Free(allocator, stamp, h->count);
    return r;
}
