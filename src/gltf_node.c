// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The node hierarchy and skins (mani-0009). Every step is
// linear or n log n in the document's size, whatever its shape: depths
// by walks that stop at known nodes, common ancestors by binary lifting,
// and skins grouped by walks in which each node is claimed once and a
// walk that meets another skin's node jumps past that skin's root.

#include "gltf_node.h"

#include "allocator.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"

#include "maul-anim/skeleton.h"

#include <string.h>

#define UNKNOWN_DEPTH 0xFFFFFFFEu
#define WALKING       0xFFFFFFFDu

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
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

// Depths, refusing a cycle: each walk goes up to a node of known depth or
// a root, then numbers its way back down.
static maniImportResult ReadDepths(maniGltf* gltf, uint32_t* stack, uint32_t* maxDepthOut)
{
    uint32_t* depth = gltf->nodeDepths;
    uint32_t maxDepth = 0;
    for (uint32_t i = 0; i < gltf->nodeCount; ++i)
    {
        depth[i] = UNKNOWN_DEPTH;
    }
    for (uint32_t i = 0; i < gltf->nodeCount; ++i)
    {
        uint32_t height = 0;
        uint32_t v = i;
        while (v != MANI_JSON_NONE && depth[v] == UNKNOWN_DEPTH)
        {
            depth[v] = WALKING;
            stack[height++] = v;
            v = gltf->nodeParents[v];
        }
        if (v != MANI_JSON_NONE && depth[v] == WALKING)
        {
            return maniImportFail(maniImportReasonHierarchy,
                                  maniGltfOffset(&gltf->g, gltf->nodeValues[v]), v);
        }
        uint32_t d = v == MANI_JSON_NONE ? 0 : depth[v] + 1;
        while (height > 0)
        {
            depth[stack[--height]] = d++;
        }
        maxDepth = d > 0 && d - 1 > maxDepth ? d - 1 : maxDepth;
    }
    *maxDepthOut = maxDepth;
    return maniImportSuccess();
}

static maniImportResult BuildAncestors(maniGltf* gltf, uint32_t maxDepth)
{
    uint32_t levels = 1;
    while (levels < 32 && ((uint64_t)1 << levels) <= maxDepth)
    {
        ++levels;
    }
    size_t n = gltf->nodeCount;
    gltf->ancestors = Words(gltf, (size_t)levels * n);
    if (gltf->ancestors == NULL)
    {
        return Capacity();
    }
    gltf->levels = levels;
    uint32_t* up = gltf->ancestors;
    memcpy(up, gltf->nodeParents, n * sizeof(uint32_t));
    for (uint32_t k = 1; k < levels; ++k)
    {
        for (size_t v = 0; v < n; ++v)
        {
            uint32_t half = up[(k - 1) * n + v];
            up[k * n + v] = half == MANI_JSON_NONE ? MANI_JSON_NONE : up[(k - 1) * n + half];
        }
    }
    return maniImportSuccess();
}

uint32_t maniGltfCommonAncestor(const maniGltf* gltf, uint32_t a, uint32_t b)
{
    const uint32_t* up = gltf->ancestors;
    size_t n = gltf->nodeCount;
    if (gltf->nodeDepths[a] < gltf->nodeDepths[b])
    {
        uint32_t swap = a;
        a = b;
        b = swap;
    }
    uint32_t lift = gltf->nodeDepths[a] - gltf->nodeDepths[b];
    for (uint32_t k = 0; lift != 0; ++k, lift >>= 1)
    {
        a = (lift & 1u) != 0 ? up[k * n + a] : a;
    }
    if (a == b)
    {
        return a;
    }
    for (uint32_t k = gltf->levels; k-- > 0;)
    {
        if (up[k * n + a] != up[k * n + b])
        {
            a = up[k * n + a];
            b = up[k * n + b];
        }
    }
    return gltf->nodeParents[a];
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
// joint, from a view without a stride.
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
    if (a->view != MANI_JSON_NONE && gltf->views[a->view].stride != 0)
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
        root = j == 0 ? node : maniGltfCommonAncestor(gltf, root, node);
        if (root == MANI_JSON_NONE)
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

static uint32_t Find(uint32_t* sets, uint32_t s)
{
    while (sets[s] != s)
    {
        sets[s] = sets[sets[s]];
        s = sets[s];
    }
    return s;
}

// Joins two sets under the lower skin, so a set's representative is its
// lowest skin.
static void Join(uint32_t* sets, uint32_t a, uint32_t b)
{
    a = Find(sets, a);
    b = Find(sets, b);
    if (a != b)
    {
        sets[a > b ? a : b] = a < b ? a : b;
    }
}

// One skin's walks from its joints up to its root: unclaimed nodes are
// claimed; a node another skin claimed joins the two and the walk jumps
// past that skin's root, whose path from there it already holds.
static void GroupSkin(const maniGltf* gltf, uint32_t skin, uint32_t* owner, uint32_t* stamp,
                      uint32_t* sets)
{
    uint32_t count = 0;
    uint32_t element = maniGltfSkinJoints(gltf, skin, &count) + 1;
    uint32_t top = gltf->nodeDepths[gltf->skinRoots[skin]];
    for (uint32_t j = 0; j < count; ++j)
    {
        uint32_t v = maniGltfJointNode(gltf, element);
        element = gltf->json.values[element].next;
        while (v != MANI_JSON_NONE && gltf->nodeDepths[v] >= top && stamp[v] != skin)
        {
            stamp[v] = skin;
            uint32_t other = owner[v];
            if (other == MANI_JSON_NONE)
            {
                owner[v] = skin;
                v = gltf->nodeParents[v];
                continue;
            }
            Join(sets, skin, other);
            uint32_t otherRoot = gltf->skinRoots[other];
            v = gltf->nodeDepths[otherRoot] <= top ? MANI_JSON_NONE : gltf->nodeParents[otherRoot];
        }
    }
}

static maniImportResult GroupSkins(maniGltf* gltf, uint32_t* owner, uint32_t* stamp)
{
    uint32_t* sets = Words(gltf, gltf->skinCount);
    gltf->skinSkeletons = Words(gltf, gltf->skinCount);
    gltf->skeletonRoots = Words(gltf, gltf->skinCount);
    maniImportResult r = maniImportSuccess();
    if (sets == NULL || gltf->skinSkeletons == NULL || gltf->skeletonRoots == NULL)
    {
        r = Capacity();
    }
    for (uint32_t s = 0; r.result == mani_success && s < gltf->skinCount; ++s)
    {
        sets[s] = s;
    }
    for (uint32_t v = 0; r.result == mani_success && v < gltf->nodeCount; ++v)
    {
        owner[v] = MANI_JSON_NONE;
        stamp[v] = MANI_JSON_NONE;
    }
    for (uint32_t s = 0; r.result == mani_success && s < gltf->skinCount; ++s)
    {
        GroupSkin(gltf, s, owner, stamp, sets);
    }
    // Skeletons in the order of their lowest skins; each root the common
    // ancestor of its skins' roots.
    for (uint32_t s = 0; r.result == mani_success && s < gltf->skinCount; ++s)
    {
        uint32_t lowest = Find(sets, s);
        if (lowest == s)
        {
            gltf->skeletonRoots[gltf->skeletonCount] = gltf->skinRoots[s];
            gltf->skinSkeletons[s] = gltf->skeletonCount++;
            continue;
        }
        uint32_t k = gltf->skinSkeletons[lowest];
        gltf->skinSkeletons[s] = k;
        gltf->skeletonRoots[k] =
            maniGltfCommonAncestor(gltf, gltf->skeletonRoots[k], gltf->skinRoots[s]);
    }
    Free(gltf, sets, gltf->skinCount);
    return r;
}

static maniImportResult ReadSkins(maniGltf* gltf, uint32_t* owner, uint32_t* stamp)
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
    return GroupSkins(gltf, owner, stamp);
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
    gltf->nodeDepths = Words(gltf, count);
    uint32_t* stack = Words(gltf, count);
    uint32_t* owner = Words(gltf, count);
    if (gltf->nodeValues == NULL || gltf->nodeParents == NULL || gltf->nodeDepths == NULL ||
        stack == NULL || owner == NULL)
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
        uint32_t maxDepth = 0;
        r = ReadChildren(gltf);
        if (r.result == mani_success)
        {
            r = ReadDepths(gltf, stack, &maxDepth);
        }
        if (r.result == mani_success)
        {
            r = BuildAncestors(gltf, maxDepth);
        }
        if (r.result == mani_success)
        {
            r = ReadSkins(gltf, owner, stack);
        }
    }
    Free(gltf, stack, count);
    Free(gltf, owner, count);
    return r;
}

void maniGltfReleaseNodes(maniGltf* gltf)
{
    size_t n = gltf->nodeCount;
    size_t skins = gltf->skinCount;
    Free(gltf, gltf->nodeValues, n);
    Free(gltf, gltf->nodeParents, n);
    Free(gltf, gltf->nodeDepths, n);
    Free(gltf, gltf->ancestors, (size_t)gltf->levels * n);
    Free(gltf, gltf->skinValues, skins);
    Free(gltf, gltf->skinRoots, skins);
    Free(gltf, gltf->skinSkeletons, skins);
    Free(gltf, gltf->skeletonRoots, skins);
}
