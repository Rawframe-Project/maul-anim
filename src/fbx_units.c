// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skeletons. A skin's unit is the models its clusters
// link, rooted at their common ancestor; a limb chain's is every limb
// below a limb with no limb above it (a limb: a model of subclass
// LimbNode, Limb or Root, or with a node attribute of one, as ufbx reads
// bones). Units sharing a model are one skeleton (hierarchy.c), numbered
// by their lowest unit: skins first, in object order, then chains in
// model order.

#include "fbx_units.h"

#include "allocator.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "hierarchy.h"

#include <stdbool.h>
#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

static uint32_t* Words(const maniFbx* fbx, size_t count)
{
    return maniAllocate(&fbx->allocator, count + 1, sizeof(uint32_t), 8);
}

static void FreeWords(const maniFbx* fbx, uint32_t* words, size_t count)
{
    maniRelease(&fbx->allocator, words, count + 1, sizeof(uint32_t), 8);
}

// Links: the models a skin's clusters link, counted or written.
typedef struct Links
{
    uint32_t* out;
    uint32_t count;
} Links;

static void VisitLink(const maniFbx* fbx, uint32_t child, void* context)
{
    Links* links = context;
    uint32_t model = maniFbxModelOf(fbx, child);
    if (model != MANI_FBX_NONE)
    {
        if (links->out != NULL)
        {
            links->out[links->count] = model;
        }
        ++links->count;
    }
}

static void VisitCluster(const maniFbx* fbx, uint32_t child, void* context)
{
    if (maniFbxDeformerIs(fbx, child, "Cluster", 7))
    {
        maniFbxEachChild(fbx, child, VisitLink, context);
    }
}

static bool LimbClass(const maniFbx* fbx, uint32_t object)
{
    return maniFbxObjectSubclassIs(fbx, object, "LimbNode", 8) ||
           maniFbxObjectSubclassIs(fbx, object, "Limb", 4) ||
           maniFbxObjectSubclassIs(fbx, object, "Root", 4);
}

static void VisitAttribute(const maniFbx* fbx, uint32_t child, void* context)
{
    bool* limb = context;
    *limb =
        *limb || (child < fbx->objectCount &&
                  maniFbxObjectKindIs(fbx, child, "NodeAttribute", 13) && LimbClass(fbx, child));
}

static bool IsLimb(const maniFbx* fbx, uint32_t model)
{
    uint32_t object = fbx->models[model];
    bool limb = LimbClass(fbx, object);
    maniFbxEachChild(fbx, object, VisitAttribute, &limb);
    return limb;
}

// Each model's topmost limb at or above it (MANI_FBX_NONE when none) and
// whether it is a limb, by a walk of the children lists, parents first.
static void TopLimbs(const maniFbx* fbx, uint32_t* top, uint8_t* limb, uint32_t* stack)
{
    uint32_t n = fbx->modelCount;
    uint32_t height = 0;
    stack[height++] = n;
    while (height > 0)
    {
        uint32_t v = stack[--height];
        if (v != n)
        {
            uint32_t p = fbx->modelParents[v];
            uint32_t above = p != MANI_FBX_NONE ? top[p] : MANI_FBX_NONE;
            limb[v] = IsLimb(fbx, v) ? 1 : 0;
            top[v] = above != MANI_FBX_NONE ? above : limb[v] != 0 ? v : MANI_FBX_NONE;
        }
        const uint32_t* children = NULL;
        uint32_t count = maniFbxModelChildren(fbx, v, &children);
        for (uint32_t k = 0; k < count; ++k)
        {
            stack[height++] = children[k];
        }
    }
}

// The skins, each one's links' count; their total.
static maniImportResult ListSkins(maniFbx* fbx, uint32_t* totalOut)
{
    uint32_t n = 0;
    for (uint32_t o = 0; o < fbx->objectCount; ++o)
    {
        n += maniFbxDeformerIs(fbx, o, "Skin", 4) ? 1 : 0;
    }
    fbx->skinCount = n;
    fbx->skins = Words(fbx, n);
    fbx->skinSkeletons = Words(fbx, n);
    if (fbx->skins == NULL || fbx->skinSkeletons == NULL)
    {
        return Capacity();
    }
    uint32_t s = 0;
    *totalOut = 0;
    for (uint32_t o = 0; o < fbx->objectCount; ++o)
    {
        if (maniFbxDeformerIs(fbx, o, "Skin", 4))
        {
            Links links = {NULL, 0};
            maniFbxEachChild(fbx, o, VisitCluster, &links);
            fbx->skins[s] = o;
            fbx->skinSkeletons[s++] = MANI_FBX_NONE;
            *totalOut += links.count;
        }
    }
    return maniFbxSuccess();
}

// Skin units: skins with links, seeds their links, root their common
// ancestor; their seeds from the start.
static maniImportResult SkinUnits(maniFbx* fbx, uint32_t* unitSkin, uint32_t* atOut)
{
    uint32_t at = 0;
    uint32_t u = 0;
    for (uint32_t s = 0; s < fbx->skinCount; ++s)
    {
        Links links = {fbx->unitSeeds + at, 0};
        maniFbxEachChild(fbx, fbx->skins[s], VisitCluster, &links);
        if (links.count == 0)
        {
            continue;
        }
        uint32_t root = links.out[0];
        for (uint32_t k = 1; k < links.count && root != MANI_NO_NODE; ++k)
        {
            root = maniCommonAncestor(&fbx->modelHierarchy, root, links.out[k]);
        }
        if (root == MANI_NO_NODE)
        {
            uint64_t offset = fbx->nodes[fbx->objects[fbx->skins[s]].node].nameOffset;
            return maniFbxFail(maniImportReasonHierarchy, offset, s);
        }
        fbx->unitStarts[u] = at;
        fbx->unitRoots[u] = root;
        unitSkin[u++] = s;
        at += links.count;
    }
    fbx->unitCount = u;
    *atOut = at;
    return maniFbxSuccess();
}

// Chain units after the skins': a unit per top limb in model order, its
// seeds its limbs in model order, by a counting sort on their tops.
static void ChainUnits(maniFbx* fbx, const uint32_t* top, const uint8_t* limb, uint32_t at,
                       uint32_t* unitSkin, uint32_t* chainOf)
{
    uint32_t first = fbx->unitCount;
    uint32_t u = first;
    for (uint32_t r = 0; r < fbx->modelCount; ++r)
    {
        if (top[r] == r)
        {
            fbx->unitRoots[u] = r;
            unitSkin[u] = MANI_FBX_NONE;
            chainOf[r] = u++;
        }
    }
    // unitStarts[c + 1] counts chain c's limbs, then sums to starts.
    for (uint32_t c = first; c <= u; ++c)
    {
        fbx->unitStarts[c] = 0;
    }
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        if (limb[m] != 0)
        {
            ++fbx->unitStarts[chainOf[top[m]] + 1];
        }
    }
    fbx->unitStarts[first] = at;
    for (uint32_t c = first; c < u; ++c)
    {
        fbx->unitStarts[c + 1] += fbx->unitStarts[c];
    }
    // Filled with chainOf reused as each chain's cursor.
    for (uint32_t c = first; c < u; ++c)
    {
        chainOf[fbx->unitRoots[c]] = fbx->unitStarts[c];
    }
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        if (limb[m] != 0)
        {
            fbx->unitSeeds[chainOf[top[m]]++] = m;
        }
    }
    fbx->unitCount = u;
}

// Groups the units; each skin's skeleton is its unit's.
static maniImportResult Group(maniFbx* fbx, const uint32_t* unitSkin)
{
    // At the units' capacity, as the other unit arrays, which their
    // release counts on.
    fbx->skeletonRoots = Words(fbx, fbx->unitCapacity);
    if (fbx->skeletonRoots == NULL)
    {
        return Capacity();
    }
    maniHierarchyUnits units = {fbx->unitSeeds, fbx->unitStarts, fbx->unitRoots, fbx->unitCount};
    maniResult r = maniHierarchyGroup(&fbx->modelHierarchy, &fbx->allocator, &units,
                                      fbx->unitSkeletons, fbx->skeletonRoots, &fbx->skeletonCount);
    for (uint32_t u = 0; r == mani_success && u < fbx->unitCount; ++u)
    {
        if (unitSkin[u] != MANI_FBX_NONE)
        {
            fbx->skinSkeletons[unitSkin[u]] = fbx->unitSkeletons[u];
        }
    }
    return r == mani_success ? maniFbxSuccess() : Capacity();
}

maniImportResult maniFbxReadUnits(maniFbx* fbx)
{
    uint32_t n = fbx->modelCount;
    uint32_t links = 0;
    maniImportResult r = ListSkins(fbx, &links);
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t* top = Words(fbx, n);
    uint32_t* scratch = Words(fbx, n);
    uint8_t* limb = maniAllocate(&fbx->allocator, (size_t)n + 1, 1, 8);
    // Units: at most a skin each and a chain per model.
    size_t units = (size_t)fbx->skinCount + n;
    uint32_t* unitSkin = Words(fbx, units);
    fbx->unitCapacity = (uint32_t)units;
    fbx->seedCapacity = links + n;
    fbx->unitSeeds = Words(fbx, fbx->seedCapacity);
    fbx->unitStarts = Words(fbx, units + 1);
    fbx->unitRoots = Words(fbx, units);
    fbx->unitSkeletons = Words(fbx, units);
    r = Capacity();
    if (top != NULL && scratch != NULL && limb != NULL && unitSkin != NULL &&
        fbx->unitSeeds != NULL && fbx->unitStarts != NULL && fbx->unitRoots != NULL &&
        fbx->unitSkeletons != NULL)
    {
        TopLimbs(fbx, top, limb, scratch);
        uint32_t at = 0;
        r = SkinUnits(fbx, unitSkin, &at);
        if (r.result == mani_success)
        {
            ChainUnits(fbx, top, limb, at, unitSkin, scratch);
            r = Group(fbx, unitSkin);
        }
    }
    FreeWords(fbx, top, n);
    FreeWords(fbx, scratch, n);
    maniRelease(&fbx->allocator, limb, (size_t)n + 1, 1, 8);
    FreeWords(fbx, unitSkin, units);
    return r;
}

void maniFbxReleaseUnits(maniFbx* fbx)
{
    size_t units = fbx->unitCapacity;
    FreeWords(fbx, fbx->skins, fbx->skinCount);
    FreeWords(fbx, fbx->skinSkeletons, fbx->skinCount);
    FreeWords(fbx, fbx->unitSeeds, fbx->seedCapacity);
    FreeWords(fbx, fbx->unitStarts, units + 1);
    FreeWords(fbx, fbx->unitRoots, units);
    FreeWords(fbx, fbx->unitSkeletons, units);
    FreeWords(fbx, fbx->skeletonRoots, units);
}

void maniFbxMarkSkeleton(const maniFbx* fbx, uint32_t skeleton, uint8_t* marked)
{
    marked[fbx->skeletonRoots[skeleton]] = 1;
    for (uint32_t u = 0; u < fbx->unitCount; ++u)
    {
        if (fbx->unitSkeletons[u] != skeleton)
        {
            continue;
        }
        for (uint32_t s = fbx->unitStarts[u]; s < fbx->unitStarts[u + 1]; ++s)
        {
            for (uint32_t v = fbx->unitSeeds[s]; marked[v] == 0; v = fbx->modelParents[v])
            {
                marked[v] = 1;
            }
        }
    }
}

uint32_t maniFbxSkeletonCount(const maniFbx* fbx)
{
    return fbx != NULL ? fbx->skeletonCount : 0;
}

uint32_t maniFbxSkinCount(const maniFbx* fbx)
{
    return fbx != NULL ? fbx->skinCount : 0;
}

uint32_t maniFbxSkinSkeleton(const maniFbx* fbx, uint32_t skin)
{
    return fbx != NULL && skin < fbx->skinCount ? fbx->skinSkeletons[skin] : UINT32_MAX;
}
