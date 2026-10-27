// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The node hierarchy and skins: parents from children
// lists, then the shared forest (hierarchy.c) for depths, common roots
// and the skeletons skins sharing nodes make, each step linear or
// n log n in the document's size, whatever its shape.

#include "gltf_node.h"

#include "allocator.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "hierarchy.h"
#include "json.h"

#include "maul-anim/skeleton.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

// count words, at least one so that 0 still allocates.
static uint32_t* Words(const maniGltf* gltf, size_t count)
{
    return maniAllocate(&gltf->allocator, count + 1, sizeof(uint32_t), 8);
}

static void Free(const maniGltf* gltf, uint32_t* words, size_t count)
{
    maniRelease(&gltf->allocator, words, count + 1, sizeof(uint32_t), 8);
}

// An index element: a whole number below count. A whole number past it is
// an Index refusal, anything else a Property one.
static maniImportResult ReadIndexElement(const maniGltf* gltf, uint32_t element, uint32_t count,
                                         uint32_t item, uint32_t* indexOut)
{
    int64_t value = -1;
    if (maniJsonInteger(&gltf->json, element, 0, (int64_t)count - 1, &value))
    {
        *indexOut = (uint32_t)value;
        return maniImportSuccess();
    }
    bool whole = maniJsonInteger(&gltf->json, element, INT64_MIN, INT64_MAX, &value);
    return maniImportFail(whole && value >= 0 ? maniImportReasonIndex : maniImportReasonProperty,
                          maniGltfOffset(&gltf->g, element), item);
}

// Each child's parent, refusing a node given two parents.
static maniImportResult ReadChildren(maniGltf* gltf)
{
    const maniGltfJson* g = &gltf->g;
    const maniJsonValue* values = gltf->json.values;
    for (uint32_t i = 0; i < gltf->nodeCount; ++i)
    {
        uint32_t children = MANI_JSON_NONE;
        maniImportResult r =
            maniGltfMember(g, gltf->nodeValues[i], "children", maniJsonArray, false, i, &children);
        if (r.result != mani_success)
        {
            return r;
        }
        uint32_t element = children + 1;
        uint32_t count = children != MANI_JSON_NONE ? values[children].count : 0;
        for (uint32_t c = 0; c < count; ++c)
        {
            uint32_t child = 0;
            r = ReadIndexElement(gltf, element, gltf->nodeCount, i, &child);
            if (r.result != mani_success)
            {
                return r;
            }
            if (gltf->nodeParents[child] != MANI_JSON_NONE)
            {
                return maniImportFail(maniImportReasonHierarchy, maniGltfOffset(g, element),
                                      (uint32_t)child);
            }
            gltf->nodeParents[child] = i;
            element = values[element].next;
        }
    }
    return maniImportSuccess();
}

// Depths and common ancestors, refusing a cycle.
static maniImportResult ReadHierarchy(maniGltf* gltf)
{
    uint32_t cycle = 0;
    maniResult r = maniHierarchyInit(&gltf->hierarchy, &gltf->allocator, gltf->nodeParents,
                                     gltf->nodeCount, &cycle);
    if (r == mani_errorFormat)
    {
        return maniImportFail(maniImportReasonHierarchy,
                              maniGltfOffset(&gltf->g, gltf->nodeValues[cycle]), cycle);
    }
    return r == mani_success ? maniImportSuccess() : Capacity();
}

uint32_t maniGltfSkinJoints(const maniGltf* gltf, uint32_t skin, uint32_t* countOut)
{
    uint32_t joints = maniJsonFind(&gltf->json, gltf->skinValues[skin], "joints", 6);
    *countOut = gltf->json.values[joints].count;
    return joints;
}

uint32_t maniGltfJointNode(const maniGltf* gltf, uint32_t element)
{
    int64_t node = 0;
    (void)maniJsonInteger(&gltf->json, element, 0, (int64_t)gltf->nodeCount - 1, &node);
    return (uint32_t)node;
}

// A skin's inverse binds: a MAT4 float accessor of at least one per
// joint, from a packed view (a stride of 64 reads as none).
static maniImportResult CheckInverseBinds(const maniGltf* gltf, uint32_t skin, uint32_t joints)
{
    const maniGltfJson* g = &gltf->g;
    uint32_t accessor = MANI_JSON_NONE;
    maniImportResult r = maniGltfIndex(g, gltf->skinValues[skin], "inverseBindMatrices", false,
                                       gltf->accessorCount, skin, &accessor);
    if (r.result != mani_success || accessor == MANI_JSON_NONE)
    {
        return r;
    }
    const maniGltfAccessor* a = &gltf->accessors[accessor];
    uint32_t at = maniJsonFind(g->json, gltf->skinValues[skin], "inverseBindMatrices", 19);
    if (a->rows != 4 || a->columns != 4 || a->component != maniGltfFloat || a->count < joints)
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, at), skin);
    }
    if (!maniGltfPacked(gltf, a))
    {
        return maniImportFail(maniImportReasonLayout, maniGltfOffset(g, at), skin);
    }
    return maniImportSuccess();
}

// A skin's joints (indices, none repeated: stamp marks them) and their
// common root.
static maniImportResult ReadSkin(maniGltf* gltf, uint32_t skin, uint32_t* stamp)
{
    const maniGltfJson* g = &gltf->g;
    uint32_t value = gltf->skinValues[skin];
    uint32_t joints = MANI_JSON_NONE;
    uint32_t root = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, value, "joints", maniJsonArray, true, skin, &joints);
    if (r.result == mani_success)
    {
        r = maniGltfIndex(g, value, "skeleton", false, gltf->nodeCount, skin, &root);
    }
    uint32_t count = r.result == mani_success ? gltf->json.values[joints].count : 0;
    if (r.result == mani_success && (count == 0 || count > MANI_MAX_BONES))
    {
        r = maniImportFail(maniImportReasonProperty, maniGltfOffset(g, joints), skin);
    }
    uint32_t element = joints + 1;
    root = MANI_JSON_NONE;
    for (uint32_t j = 0; r.result == mani_success && j < count; ++j)
    {
        uint32_t node = 0;
        r = ReadIndexElement(gltf, element, gltf->nodeCount, skin, &node);
        if (r.result != mani_success)
        {
            return r;
        }
        if (stamp[node] == skin)
        {
            return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, element), skin);
        }
        stamp[node] = skin;
        root = j == 0 ? node : maniCommonAncestor(&gltf->hierarchy, root, node);
        if (root == MANI_NO_NODE)
        {
            return maniImportFail(maniImportReasonHierarchy, maniGltfOffset(g, joints), skin);
        }
        element = gltf->json.values[element].next;
    }
    if (r.result == mani_success)
    {
        r = CheckInverseBinds(gltf, skin, count);
    }
    gltf->skinRoots[skin] = root;
    return r;
}

// Skeletons: skins whose joints' paths to their roots share a node, as
// one (hierarchy.c).
static maniImportResult GroupSkins(maniGltf* gltf)
{
    uint32_t total = 0;
    for (uint32_t k = 0; k < gltf->skinCount; ++k)
    {
        uint32_t count = 0;
        (void)maniGltfSkinJoints(gltf, k, &count);
        total += count;
    }
    uint32_t* seeds = Words(gltf, total);
    uint32_t* starts = Words(gltf, (size_t)gltf->skinCount + 1);
    gltf->skinSkeletons = Words(gltf, gltf->skinCount);
    gltf->skeletonRoots = Words(gltf, gltf->skinCount);
    maniResult r = mani_errorCapacity;
    if (seeds != NULL && starts != NULL && gltf->skinSkeletons != NULL &&
        gltf->skeletonRoots != NULL)
    {
        uint32_t at = 0;
        for (uint32_t k = 0; k < gltf->skinCount; ++k)
        {
            uint32_t count = 0;
            uint32_t element = maniGltfSkinJoints(gltf, k, &count) + 1;
            starts[k] = at;
            for (uint32_t j = 0; j < count; ++j)
            {
                seeds[at++] = maniGltfJointNode(gltf, element);
                element = gltf->json.values[element].next;
            }
        }
        starts[gltf->skinCount] = at;
        maniHierarchyUnits units = {seeds, starts, gltf->skinRoots, gltf->skinCount};
        r = maniHierarchyGroup(&gltf->hierarchy, &gltf->allocator, &units, gltf->skinSkeletons,
                               gltf->skeletonRoots, &gltf->skeletonCount);
    }
    Free(gltf, seeds, total);
    Free(gltf, starts, (size_t)gltf->skinCount + 1);
    return r == mani_success ? maniImportSuccess() : Capacity();
}

static maniImportResult ReadSkins(maniGltf* gltf, uint32_t* stamp)
{
    uint32_t array = MANI_JSON_NONE;
    uint32_t count = 0;
    maniImportResult r = maniGltfArray(&gltf->g, "skins", maniJsonObject, &array, &count);
    if (r.result != mani_success)
    {
        return r;
    }
    gltf->skinCount = count;
    gltf->skinValues = Words(gltf, count);
    gltf->skinRoots = Words(gltf, count);
    if (gltf->skinValues == NULL || gltf->skinRoots == NULL)
    {
        return Capacity();
    }
    for (uint32_t v = 0; v < gltf->nodeCount; ++v)
    {
        stamp[v] = MANI_JSON_NONE;
    }
    uint32_t value = array + 1;
    for (uint32_t s = 0; s < count; ++s)
    {
        gltf->skinValues[s] = value;
        r = ReadSkin(gltf, s, stamp);
        if (r.result != mani_success)
        {
            return r;
        }
        value = gltf->json.values[value].next;
    }
    return GroupSkins(gltf);
}

maniImportResult maniGltfReadNodes(maniGltf* gltf)
{
    uint32_t array = MANI_JSON_NONE;
    uint32_t count = 0;
    maniImportResult r = maniGltfArray(&gltf->g, "nodes", maniJsonObject, &array, &count);
    if (r.result != mani_success)
    {
        return r;
    }
    // The count first: a release frees with it, whatever succeeded.
    gltf->nodeCount = count;
    gltf->nodeValues = Words(gltf, count);
    gltf->nodeParents = Words(gltf, count);
    uint32_t* stamp = Words(gltf, count);
    if (gltf->nodeValues == NULL || gltf->nodeParents == NULL || stamp == NULL)
    {
        r = Capacity();
    }
    else
    {
        uint32_t value = array + 1;
        for (uint32_t i = 0; i < count; ++i)
        {
            gltf->nodeValues[i] = value;
            gltf->nodeParents[i] = MANI_JSON_NONE;
            value = gltf->json.values[value].next;
        }
        r = ReadChildren(gltf);
        if (r.result == mani_success)
        {
            r = ReadHierarchy(gltf);
        }
        if (r.result == mani_success)
        {
            r = ReadSkins(gltf, stamp);
        }
    }
    Free(gltf, stamp, count);
    return r;
}

void maniGltfReleaseNodes(maniGltf* gltf)
{
    size_t n = gltf->nodeCount;
    size_t skins = gltf->skinCount;
    Free(gltf, gltf->nodeValues, n);
    Free(gltf, gltf->nodeParents, n);
    maniHierarchyRelease(&gltf->hierarchy, &gltf->allocator);
    Free(gltf, gltf->skinValues, skins);
    Free(gltf, gltf->skinRoots, skins);
    Free(gltf, gltf->skinSkeletons, skins);
    Free(gltf, gltf->skeletonRoots, skins);
}
