// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skeletons and skins from a glTF document:
// bones from skins' joints or a node's subtree, rests from TRS or
// decomposed matrices, names made unique by a fixed rule, the placement
// above the root reported or folded, the basis change applied, skins
// bound by bone name.

#include "allocator.h"
#include "builder.h"
#include "fpenv.h"
#include "gltf_basis.h"
#include "gltf_document.h"
#include "gltf_layout.h"
#include "gltf_names.h"
#include "gltf_node.h"
#include "gltf_property.h"
#include "gltf_rest.h"
#include "json.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <string.h>

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static uint32_t* Words(const maniGltf* gltf, size_t count)
{
    return maniAllocate(&gltf->allocator, count + 1, sizeof(uint32_t), 8);
}

static void Free(const maniGltf* gltf, void* block, size_t count, size_t size)
{
    maniRelease(&gltf->allocator, block, count + 1, size, 8);
}

// The builder's refusal as the import's: a name collision on the bone's
// node, anything else with the build's reason.
static maniImportResult FromBuild(const maniGltf* gltf, maniBuildResult b, const uint32_t* nodes)
{
    if (b.result == mani_success || b.result == mani_errorCapacity)
    {
        return (maniImportResult){b.result, maniImportReasonNone, 0, 0, 0};
    }
    bool name = b.reason == maniBuildReasonDuplicateName || b.reason == maniBuildReasonNameHash;
    uint32_t node = nodes != NULL ? nodes[b.item] : 0;
    return maniImportFail(name ? maniImportReasonName : maniImportReasonBuild,
                          nodes != NULL ? maniGltfOffset(&gltf->g, gltf->nodeValues[node]) : 0,
                          name ? node : (uint32_t)b.reason);
}

// The bones: marked nodes in node order, with each node's bone index in
// boneOf (MANI_JSON_NONE for others).
static uint32_t CollectBones(const maniGltf* gltf, const uint32_t* marked, uint32_t* nodes,
                             uint32_t* boneOf)
{
    uint32_t count = 0;
    for (uint32_t v = 0; v < gltf->nodeCount; ++v)
    {
        boneOf[v] = marked[v] != 0 ? count : MANI_JSON_NONE;
        if (marked[v] != 0)
        {
            nodes[count++] = v;
        }
    }
    return count;
}

// A skeleton's bones: each of its skins' joints and the nodes up to its
// root.
static void MarkSkinBones(const maniGltf* gltf, uint32_t skeleton, uint32_t* marked)
{
    uint32_t root = gltf->skeletonRoots[skeleton];
    marked[root] = 1;
    for (uint32_t s = 0; s < gltf->skinCount; ++s)
    {
        if (gltf->skinSkeletons[s] != skeleton)
        {
            continue;
        }
        uint32_t count = 0;
        uint32_t element = maniGltfSkinJoints(gltf, s, &count) + 1;
        for (uint32_t j = 0; j < count; ++j)
        {
            for (uint32_t v = maniGltfJointNode(gltf, element); marked[v] == 0;
                 v = gltf->nodeParents[v])
            {
                marked[v] = 1;
            }
            element = gltf->json.values[element].next;
        }
    }
}

// A node's subtree, by an explicit stack of child arrays' next elements.
static void MarkSubtree(const maniGltf* gltf, uint32_t node, uint32_t* marked, uint32_t* stack)
{
    uint32_t height = 0;
    stack[height++] = node;
    while (height > 0)
    {
        uint32_t v = stack[--height];
        marked[v] = 1;
        uint32_t children = maniJsonFind(&gltf->json, gltf->nodeValues[v], "children", 8);
        if (children == MANI_JSON_NONE)
        {
            continue;
        }
        uint32_t element = children + 1;
        for (uint32_t c = 0; c < gltf->json.values[children].count; ++c)
        {
            stack[height++] = maniGltfJointNode(gltf, element);
            element = gltf->json.values[element].next;
        }
    }
}

// Builds the marked bones rooted at `root` into the builder.
static maniImportResult BuildBones(maniBuilder* builder, const maniGltf* gltf, uint32_t root,
                                   const uint32_t* marked, const maniGltfOptions* options)
{
    size_t n = gltf->nodeCount;
    uint32_t* nodes = Words(gltf, n);
    uint32_t* boneOf = Words(gltf, n);
    maniBoneSource* bones = maniAllocate(&gltf->allocator, n + 1, sizeof(maniBoneSource), 8);
    maniImportResult r = Capacity();
    if (nodes != NULL && boneOf != NULL && bones != NULL)
    {
        uint32_t count = CollectBones(gltf, marked, nodes, boneOf);
        r = maniImportSuccess();
        for (uint32_t b = 0; r.result == mani_success && b < count; ++b)
        {
            uint32_t v = nodes[b];
            bool isRoot = v == root;
            bones[b].name = maniGltfNodeName(gltf, v, &bones[b].nameLength);
            bones[b].parent = isRoot ? -1 : (int32_t)boneOf[gltf->nodeParents[v]];
            r = isRoot ? maniGltfRootRest(gltf, root, options, &bones[b].rest)
                       : maniGltfNodeRest(gltf, v, options, &bones[b].rest);
        }
        if (r.result == mani_success)
        {
            maniSkeletonSource source = {bones, count, 0};
            r = FromBuild(gltf, maniBuildSkeleton(builder, &source), nodes);
        }
    }
    Free(gltf, bones, n, sizeof(maniBoneSource));
    Free(gltf, nodes, n, sizeof(uint32_t));
    Free(gltf, boneOf, n, sizeof(uint32_t));
    return r;
}

// The checks every entry point makes: a NULL builder is refused alone, any
// other bad argument is a misuse.
static bool Usable(maniBuilder* builder, const maniGltf* gltf, const maniGltfOptions* options,
                   bool inRange)
{
    if (builder == NULL)
    {
        return false;
    }
    if (gltf == NULL || options == NULL || !maniGltfOptionsValid(options) || !inRange)
    {
        maniBuilderMisuse(builder);
        return false;
    }
    return true;
}

static MANI_NOINLINE maniImportResult GltfBuildSkeletonWork(maniBuilder* builder,
                                                            const maniGltf* gltf, uint32_t skeleton,
                                                            const maniGltfOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorState, maniImportReasonNone, 0, 0, 0};
    }
    if (!Usable(builder, gltf, options, gltf != NULL && skeleton < gltf->skeletonCount))
    {
        return Invalid();
    }
    uint32_t* marked = Words(gltf, gltf->nodeCount);
    if (marked == NULL)
    {
        return Capacity();
    }
    memset(marked, 0, ((size_t)gltf->nodeCount + 1) * sizeof(uint32_t));
    MarkSkinBones(gltf, skeleton, marked);
    maniImportResult r = BuildBones(builder, gltf, gltf->skeletonRoots[skeleton], marked, options);
    Free(gltf, marked, gltf->nodeCount, sizeof(uint32_t));
    return r;
}

maniImportResult maniGltfBuildSkeleton(maniBuilder* builder, const maniGltf* gltf,
                                       uint32_t skeleton, const maniGltfOptions* options)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniImportResult result = GltfBuildSkeletonWork(builder, gltf, skeleton, options);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniImportResult GltfBuildNodeSkeletonWork(maniBuilder* builder,
                                                                const maniGltf* gltf, uint32_t node,
                                                                const maniGltfOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorState, maniImportReasonNone, 0, 0, 0};
    }
    if (!Usable(builder, gltf, options, gltf != NULL && node < gltf->nodeCount))
    {
        return Invalid();
    }
    size_t n = gltf->nodeCount;
    uint32_t* marked = Words(gltf, n);
    uint32_t* stack = Words(gltf, n);
    maniImportResult r = Capacity();
    if (marked != NULL && stack != NULL)
    {
        memset(marked, 0, (n + 1) * sizeof(uint32_t));
        MarkSubtree(gltf, node, marked, stack);
        r = BuildBones(builder, gltf, node, marked, options);
    }
    Free(gltf, marked, n, sizeof(uint32_t));
    Free(gltf, stack, n, sizeof(uint32_t));
    return r;
}

maniImportResult maniGltfBuildNodeSkeleton(maniBuilder* builder, const maniGltf* gltf,
                                           uint32_t node, const maniGltfOptions* options)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniImportResult result = GltfBuildNodeSkeletonWork(builder, gltf, node, options);
    maniFloatEnvLeave(&env);
    return result;
}

static MANI_NOINLINE maniImportResult GltfSkeletonPlacementWork(const maniGltf* gltf,
                                                                uint32_t skeleton,
                                                                const maniGltfOptions* options,
                                                                maniAffine* placementOut)
{
    if (gltf == NULL || options == NULL || placementOut == NULL || !maniGltfOptionsValid(options) ||
        skeleton >= gltf->skeletonCount)
    {
        return Invalid();
    }
    *placementOut = maniGltfIdentityAffine();
    if (options->foldPlacement != 0)
    {
        return maniImportSuccess();
    }
    return maniGltfPlacement(gltf, gltf->skeletonRoots[skeleton], options, placementOut);
}

maniImportResult maniGltfSkeletonPlacement(const maniGltf* gltf, uint32_t skeleton,
                                           const maniGltfOptions* options, maniAffine* placementOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniImportResult result = GltfSkeletonPlacementWork(gltf, skeleton, options, placementOut);
    maniFloatEnvLeave(&env);
    return result;
}

// The skin's joints as bones of the skeleton, found by name.
static maniImportResult FindJoints(const maniGltf* gltf, uint32_t skin,
                                   const maniSkeleton* skeleton, uint32_t* joints)
{
    uint32_t count = 0;
    uint32_t element = maniGltfSkinJoints(gltf, skin, &count) + 1;
    for (uint32_t j = 0; j < count; ++j)
    {
        uint32_t length = 0;
        const char* name = maniGltfNodeName(gltf, maniGltfJointNode(gltf, element), &length);
        joints[j] = maniSkeletonFindBone(skeleton, name, length);
        if (joints[j] == MANI_NO_BONE)
        {
            return maniImportFail(maniImportReasonBone, maniGltfOffset(&gltf->g, element), skin);
        }
        element = gltf->json.values[element].next;
    }
    return maniImportSuccess();
}

// The inverse binds, one per joint, identity when the skin has none.
static maniImportResult ReadInverseBinds(const maniGltf* gltf, uint32_t skin,
                                         const maniGltfOptions* options, uint32_t jointCount,
                                         maniAffine* binds)
{
    uint32_t accessor = MANI_JSON_NONE;
    maniImportResult r = maniGltfIndex(&gltf->g, gltf->skinValues[skin], "inverseBindMatrices",
                                       false, gltf->accessorCount, skin, &accessor);
    if (r.result != mani_success || accessor == MANI_JSON_NONE)
    {
        for (uint32_t j = 0; j < jointCount; ++j)
        {
            binds[j] = maniGltfIdentityAffine();
        }
        return r;
    }
    size_t floats = (size_t)gltf->accessors[accessor].count * 16;
    float* m = maniAllocate(&gltf->allocator, floats, sizeof(float), 8);
    if (m == NULL)
    {
        return Capacity();
    }
    r = maniGltfReadFloats(gltf, accessor, m);
    for (uint32_t j = 0; r.result == mani_success && j < jointCount; ++j)
    {
        r = maniGltfMatrixAffine(gltf, m + 16 * (size_t)j, options,
                                 maniGltfOffset(&gltf->g, gltf->accessors[accessor].value), skin,
                                 &binds[j]);
    }
    maniRelease(&gltf->allocator, m, floats, sizeof(float), 8);
    return r;
}

static MANI_NOINLINE maniImportResult GltfBuildSkinWork(maniBuilder* builder, const maniGltf* gltf,
                                                        uint32_t skin, const maniSkeleton* skeleton,
                                                        const maniGltfOptions* options)
{
    if (maniBuilderCalledBack(builder))
    {
        return (maniImportResult){mani_errorState, maniImportReasonNone, 0, 0, 0};
    }
    if (!Usable(builder, gltf, options, gltf != NULL && skin < gltf->skinCount && skeleton != NULL))
    {
        return Invalid();
    }
    uint32_t jointCount = 0;
    (void)maniGltfSkinJoints(gltf, skin, &jointCount);
    uint32_t* joints = Words(gltf, jointCount);
    maniAffine* binds =
        maniAllocate(&gltf->allocator, (size_t)jointCount + 1, sizeof(maniAffine), 8);
    maniImportResult r = Capacity();
    if (joints != NULL && binds != NULL)
    {
        r = FindJoints(gltf, skin, skeleton, joints);
        if (r.result == mani_success)
        {
            r = ReadInverseBinds(gltf, skin, options, jointCount, binds);
        }
        if (r.result == mani_success)
        {
            maniSkinSource source = {skeleton, joints, binds, jointCount, 0};
            r = FromBuild(gltf, maniBuildSkin(builder, &source), NULL);
        }
    }
    Free(gltf, joints, jointCount, sizeof(uint32_t));
    maniRelease(&gltf->allocator, binds, (size_t)jointCount + 1, sizeof(maniAffine), 8);
    return r;
}

maniImportResult maniGltfBuildSkin(maniBuilder* builder, const maniGltf* gltf, uint32_t skin,
                                   const maniSkeleton* skeleton, const maniGltfOptions* options)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniImportResult result = GltfBuildSkinWork(builder, gltf, skin, skeleton, options);
    maniFloatEnvLeave(&env);
    return result;
}
