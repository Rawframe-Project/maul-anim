// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The node forest (mani-0009, mani-0014): depths and common ancestors of
// random forests against walks, cycles found, and groups of random units
// against groups made from their member sets' intersections; plus the
// walk that must jump past another unit's root to claim what lies above.

#include "allocator.h"
#include "hierarchy.h"
#include "test_harness.h"

#include "maul-anim/base.h"

#include <stdlib.h>
#include <string.h>

static uint64_t s_seed = 0x2545F4914F6CDD1Du;

static uint32_t Random(uint32_t n)
{
    s_seed ^= s_seed << 13;
    s_seed ^= s_seed >> 7;
    s_seed ^= s_seed << 17;
    return (uint32_t)(s_seed % n);
}

// No functions: the library's default, malloc.
static const maniAllocator* Allocator(void)
{
    static const maniAllocator a = {NULL, NULL, NULL};
    return &a;
}

// A random forest: each node's parent an earlier node, or none.
static void Forest(uint32_t* parents, uint32_t n)
{
    for (uint32_t v = 0; v < n; ++v)
    {
        parents[v] = v == 0 || Random(8) == 0 ? MANI_NO_NODE : Random(v);
    }
}

static uint32_t Depth(const uint32_t* parents, uint32_t v)
{
    uint32_t d = 0;
    for (; parents[v] != MANI_NO_NODE; v = parents[v])
    {
        ++d;
    }
    return d;
}

static uint32_t SlowAncestor(const uint32_t* parents, uint32_t a, uint32_t b)
{
    for (uint32_t x = a; x != MANI_NO_NODE; x = parents[x])
    {
        for (uint32_t y = b; y != MANI_NO_NODE; y = parents[y])
        {
            if (x == y)
            {
                return x;
            }
        }
    }
    return MANI_NO_NODE;
}

static void TestAncestors(void)
{
    enum
    {
        kNodes = 300
    };
    uint32_t parents[kNodes];
    bool depths = true;
    bool ancestors = true;
    for (int round = 0; round < 20; ++round)
    {
        Forest(parents, kNodes);
        maniHierarchy h;
        uint32_t cycle = 0;
        CHECK(maniHierarchyInit(&h, Allocator(), parents, kNodes, &cycle) == mani_success, "init");
        for (uint32_t v = 0; v < kNodes; ++v)
        {
            depths = depths && h.depths[v] == Depth(parents, v);
        }
        for (int k = 0; k < 2000; ++k)
        {
            uint32_t a = Random(kNodes);
            uint32_t b = Random(kNodes);
            ancestors = ancestors && maniCommonAncestor(&h, a, b) == SlowAncestor(parents, a, b);
        }
        maniHierarchyRelease(&h, Allocator());
    }
    CHECK(depths, "depths as walks give them");
    CHECK(ancestors, "common ancestors as walks find them");
    // A chain 2^k long, the binary lifting's every level used.
    uint32_t chain[1025];
    for (uint32_t v = 0; v < 1025; ++v)
    {
        chain[v] = v == 0 ? MANI_NO_NODE : v - 1;
    }
    maniHierarchy h;
    uint32_t cycle = 0;
    CHECK(maniHierarchyInit(&h, Allocator(), chain, 1025, &cycle) == mani_success &&
              maniCommonAncestor(&h, 1024, 1023) == 1023 && maniCommonAncestor(&h, 1, 1024) == 1 &&
              h.depths[1024] == 1024,
          "a long chain");
    maniHierarchyRelease(&h, Allocator());
}

static void TestCycles(void)
{
    uint32_t loop[4] = {MANI_NO_NODE, 2, 3, 1};
    maniHierarchy h;
    uint32_t cycle = 99;
    CHECK(maniHierarchyInit(&h, Allocator(), loop, 4, &cycle) == mani_errorFormat && cycle >= 1 &&
              cycle <= 3,
          "a cycle, and a node on it");
    maniHierarchyRelease(&h, Allocator());
    uint32_t self[1] = {0};
    CHECK(maniHierarchyInit(&h, Allocator(), self, 1, &cycle) == mani_errorFormat && cycle == 0,
          "its own parent");
    maniHierarchyRelease(&h, Allocator());
    CHECK(maniHierarchyInit(&h, Allocator(), NULL, 0, &cycle) == mani_success, "no nodes");
    maniHierarchyRelease(&h, Allocator());
}

// Groups from members: units whose member sets intersect, transitively.
static void SlowGroups(const uint32_t* parents, uint32_t n, const maniHierarchyUnits* units,
                       uint32_t* groupOf, uint32_t* roots, uint32_t* countOut)
{
    uint32_t u = units->count;
    uint8_t* members = calloc((size_t)u * n, 1);
    uint32_t* sets = malloc(u * sizeof(uint32_t));
    for (uint32_t k = 0; k < u; ++k)
    {
        sets[k] = k;
        for (uint32_t s = units->starts[k]; s < units->starts[k + 1]; ++s)
        {
            for (uint32_t v = units->seeds[s];; v = parents[v])
            {
                members[(size_t)k * n + v] = 1;
                if (v == units->roots[k])
                {
                    break;
                }
            }
        }
    }
    for (bool changed = true; changed;)
    {
        changed = false;
        for (uint32_t a = 0; a < u; ++a)
        {
            for (uint32_t b = a + 1; b < u; ++b)
            {
                bool share = false;
                for (uint32_t v = 0; v < n && !share; ++v)
                {
                    share = members[(size_t)a * n + v] && members[(size_t)b * n + v];
                }
                if (share && sets[a] != sets[b])
                {
                    uint32_t low = sets[a] < sets[b] ? sets[a] : sets[b];
                    uint32_t high = sets[a] < sets[b] ? sets[b] : sets[a];
                    for (uint32_t k = 0; k < u; ++k)
                    {
                        sets[k] = sets[k] == high ? low : sets[k];
                    }
                    changed = true;
                }
            }
        }
    }
    *countOut = 0;
    for (uint32_t k = 0; k < u; ++k)
    {
        if (sets[k] == k)
        {
            roots[*countOut] = units->roots[k];
            groupOf[k] = (*countOut)++;
        }
        else
        {
            groupOf[k] = groupOf[sets[k]];
            roots[groupOf[k]] = SlowAncestor(parents, roots[groupOf[k]], units->roots[k]);
        }
    }
    free(members);
    free(sets);
}

static void TestGroups(void)
{
    enum
    {
        kNodes = 60,
        kUnits = 8
    };
    uint32_t parents[kNodes];
    uint32_t seeds[kUnits * 4];
    uint32_t starts[kUnits + 1];
    uint32_t roots[kUnits];
    bool same = true;
    for (int round = 0; round < 300; ++round)
    {
        // One tree, so every unit has a root.
        for (uint32_t v = 0; v < kNodes; ++v)
        {
            parents[v] = v == 0 ? MANI_NO_NODE : Random(v);
        }
        maniHierarchy h;
        uint32_t cycle = 0;
        (void)maniHierarchyInit(&h, Allocator(), parents, kNodes, &cycle);
        uint32_t at = 0;
        for (uint32_t k = 0; k < kUnits; ++k)
        {
            starts[k] = at;
            uint32_t count = 1 + Random(4);
            for (uint32_t s = 0; s < count; ++s)
            {
                seeds[at] = Random(kNodes);
                roots[k] = s == 0 ? seeds[at] : maniCommonAncestor(&h, roots[k], seeds[at]);
                ++at;
            }
            // Sometimes a root higher than the seeds need.
            roots[k] =
                Random(3) == 0 && parents[roots[k]] != MANI_NO_NODE ? parents[roots[k]] : roots[k];
        }
        starts[kUnits] = at;
        maniHierarchyUnits units = {seeds, starts, roots, kUnits};
        uint32_t groupOf[kUnits];
        uint32_t groupRoots[kUnits];
        uint32_t count = 0;
        uint32_t slowOf[kUnits];
        uint32_t slowRoots[kUnits];
        uint32_t slowCount = 0;
        CHECK(maniHierarchyGroup(&h, Allocator(), &units, groupOf, groupRoots, &count) ==
                  mani_success,
              "groups");
        SlowGroups(parents, kNodes, &units, slowOf, slowRoots, &slowCount);
        same = same && count == slowCount && memcmp(groupOf, slowOf, sizeof groupOf) == 0 &&
               memcmp(groupRoots, slowRoots, count * sizeof(uint32_t)) == 0;
        maniHierarchyRelease(&h, Allocator());
    }
    CHECK(same, "groups as member sets' intersections make them");
}

// 0 - 1 - 2 - 3. Unit 0 holds 3 and 2; unit 1 from 3 up to 0 meets
// unit 0's nodes and must claim 1 and 0 past unit 0's root; unit 2,
// holding 1 alone, then joins them.
static void TestJump(void)
{
    uint32_t parents[4] = {MANI_NO_NODE, 0, 1, 2};
    uint32_t seeds[3] = {3, 3, 1};
    uint32_t starts[4] = {0, 1, 2, 3};
    uint32_t roots[3] = {2, 0, 1};
    maniHierarchy h;
    uint32_t cycle = 0;
    (void)maniHierarchyInit(&h, Allocator(), parents, 4, &cycle);
    maniHierarchyUnits units = {seeds, starts, roots, 3};
    uint32_t groupOf[3];
    uint32_t groupRoots[3];
    uint32_t count = 0;
    CHECK(maniHierarchyGroup(&h, Allocator(), &units, groupOf, groupRoots, &count) ==
                  mani_success &&
              count == 1 && groupOf[2] == 0 && groupRoots[0] == 0,
          "a walk jumping past another unit's root claims the nodes above");
    maniHierarchyRelease(&h, Allocator());
}

int main(void)
{
    TestAncestors();
    TestCycles();
    TestGroups();
    TestJump();
    return s_failures == 0 ? 0 : 1;
}
