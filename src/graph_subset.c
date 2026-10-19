// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Subset plans (mani-0035): the listed bones closed under their ancestors
// and under what the graph's nodes read to write them, kept as the bones
// to compute and write and the pose groups holding them. A graph with a node that
// writes a bone from others (mirror, IK, springs, full-body), a mirror map
// or retargeted clips plans every bone.

#include "graph_subset.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "pose_memory.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint32_t Groups(uint32_t bones)
{
    return (bones + MANI_POSE_GROUP_BONES - 1u) / MANI_POSE_GROUP_BONES;
}

// The plan, its list at its longest and a byte per bone (the marks, then
// the groups' flags), rounded to the alignment.
static size_t SizeOf(uint32_t bones)
{
    size_t size = sizeof(maniGraphSubset) + 4 * (size_t)bones + bones;
    return (size + MANI_GRAPH_SUBSET_ALIGNMENT - 1) / MANI_GRAPH_SUBSET_ALIGNMENT *
           MANI_GRAPH_SUBSET_ALIGNMENT;
}

size_t maniGraphSubsetSize(const maniGraphBinding* binding)
{
    return binding != NULL ? SizeOf(maniSkeletonBoneCount(binding->skeleton)) : 0;
}

// Whether a node of the graph writes a bone from others, so that every
// bone is planned.
static bool ReadsOthers(const maniGraph* graph)
{
    bool others = graph->mapCount > 0 || graph->mirrorMapIdentity != 0;
    for (uint32_t p = 0; p < graph->nodeCount && !others; ++p)
    {
        uint32_t kind = maniLoadU16(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE);
        others = kind == maniGraphNodeSpring || kind == maniGraphNodeMirror ||
                 (kind >= maniGraphNodeTwoBone && kind <= maniGraphNodeBodySdls);
    }
    return others;
}

// Whether the bones are in range and strictly increasing.
static bool BonesValid(const uint32_t* bones, uint32_t count, uint32_t boneCount)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        if (bones[i] >= boneCount || (i > 0 && bones[i] <= bones[i - 1]))
        {
            return false;
        }
    }
    return true;
}

maniResult maniPlanGraphSubset(const maniGraphBinding* binding, const uint32_t* bones,
                               uint32_t count, void* memory, size_t size, maniGraphSubset** planOut)
{
    if (binding == NULL || binding->stamp != MANI_GRAPH_BINDING_STAMP || memory == NULL ||
        (uintptr_t)memory % MANI_GRAPH_SUBSET_ALIGNMENT != 0 || planOut == NULL ||
        (bones == NULL && count > 0))
    {
        return mani_errorInvalid;
    }
    const maniSkeleton* skeleton = binding->skeleton;
    uint32_t boneCount = maniSkeletonBoneCount(skeleton);
    if (!BonesValid(bones, count, boneCount))
    {
        return mani_errorInvalid;
    }
    if (size < SizeOf(boneCount))
    {
        return mani_errorCapacity;
    }
    maniGraphSubset* plan = memory;
    *plan = (maniGraphSubset){MANI_GRAPH_SUBSET_STAMP, 0, Groups(boneCount), 0, binding,
                              binding->graph.identity};
    uint32_t* list = (uint32_t*)(plan + 1);
    // The marks sit past the longest list, where its unused end and the
    // groups' flags will be.
    uint8_t* marks = (uint8_t*)(list + boneCount);
    bool all = ReadsOthers(&binding->graph);
    memset(marks, all ? 1 : 0, boneCount);
    for (uint32_t i = 0; i < count; ++i)
    {
        for (uint32_t b = bones[i]; b != MANI_NO_BONE && marks[b] == 0;
             b = maniSkeletonParent(skeleton, b))
        {
            marks[b] = 1;
        }
    }
    for (uint32_t b = 0; b < boneCount; ++b)
    {
        if (marks[b] != 0)
        {
            list[plan->count++] = b;
        }
    }
    uint8_t* groups = (uint8_t*)(list + plan->count);
    memset(groups, 0, plan->groups);
    for (uint32_t i = 0; i < plan->count; ++i)
    {
        groups[list[i] / MANI_POSE_GROUP_BONES] = 1;
    }
    *planOut = plan;
    return mani_success;
}

const uint32_t* maniGraphSubsetBones(const maniGraphSubset* plan, uint32_t* countOut)
{
    bool valid = plan != NULL && plan->stamp == MANI_GRAPH_SUBSET_STAMP;
    if (countOut != NULL)
    {
        *countOut = valid ? plan->count : 0u;
    }
    return valid ? maniGraphSubsetList(plan) : NULL;
}
