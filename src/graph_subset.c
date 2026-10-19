// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Subset plans (mani-0035): the listed bones closed under their ancestors
// and under what the graph's nodes read to write them, to a fixed point:
// under a mirror map each bone's partner; an IK, spring or full-body node
// that writes a planned bone brings every bone it reads (its chain, its
// set's joints, tails, colliders and centers, its paths from the top to
// the effectors). Kept as the bones to compute and write, the pose groups
// holding them and whether each node runs.

#include "graph_subset.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_spring.h"
#include "pose_memory.h"
#include "spring_file.h"

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/mirror.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static uint32_t Groups(uint32_t bones)
{
    return (bones + MANI_POSE_GROUP_BONES - 1u) / MANI_POSE_GROUP_BONES;
}

// The plan, its list at its longest, a byte per bone (the marks, then the
// groups' flags) and a byte per node, rounded to the alignment.
static size_t NodesAt(uint32_t bones)
{
    return sizeof(maniGraphSubset) + 4 * (size_t)bones + bones;
}

static size_t SizeOf(uint32_t bones, uint32_t nodes)
{
    size_t size = NodesAt(bones) + nodes;
    return (size + MANI_GRAPH_SUBSET_ALIGNMENT - 1) / MANI_GRAPH_SUBSET_ALIGNMENT *
           MANI_GRAPH_SUBSET_ALIGNMENT;
}

size_t maniGraphSubsetSize(const maniGraphBinding* binding)
{
    return binding != NULL
               ? SizeOf(maniSkeletonBoneCount(binding->skeleton), binding->graph.nodeCount)
               : 0;
}

// The marks growing toward the closure.
typedef struct Closure
{
    const maniGraphBinding* binding;
    const maniSkeleton* skeleton;
    uint8_t* marks;
    uint32_t bones;
    bool grew;
} Closure;

// A bone and its ancestors marked.
static void Mark(Closure* c, uint32_t bone)
{
    for (uint32_t b = bone; b < c->bones && c->marks[b] == 0;
         b = maniSkeletonParent(c->skeleton, b))
    {
        c->marks[b] = 1;
        c->grew = true;
    }
}

static bool Marked(const Closure* c, uint32_t bone)
{
    return bone < c->bones && c->marks[bone] != 0;
}

static uint32_t Word(const maniGraph* graph, uint32_t at)
{
    return maniLoadU32(graph->data + 4 * (size_t)at);
}

// What a node does with a bone: asks whether it is marked, or marks it.
typedef struct Visit
{
    Closure* closure;
    bool marking;
    bool found;
} Visit;

static void See(Visit* v, uint32_t bone)
{
    if (v->marking)
    {
        Mark(v->closure, bone);
    }
    else
    {
        v->found = v->found || Marked(v->closure, bone);
    }
}

// The bones from one up to (and with) an ancestor.
static void Path(Visit* v, uint32_t from, uint32_t to)
{
    const maniSkeleton* skeleton = v->closure->skeleton;
    for (uint32_t b = from; b < v->closure->bones; b = maniSkeletonParent(skeleton, b))
    {
        See(v, b);
        if (b == to)
        {
            return;
        }
    }
}

// A spring node's set: its joints' heads (writing, writes false) and,
// reading, their tails, its colliders' bones and its springs' centers.
static void Springs(Visit* v, const maniSpringSet* set, bool reads)
{
    for (uint32_t j = 0; j < set->jointCount; ++j)
    {
        const uint8_t* joint = set->joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
        See(v, maniLoadU32(joint + maniSpringJointHead));
        if (reads)
        {
            See(v, maniLoadU32(joint + maniSpringJointTail));
        }
    }
    for (uint32_t k = 0; reads && k < set->colliderCount; ++k)
    {
        See(v, maniLoadU32(set->colliders + (size_t)k * MANI_SPRING_COLLIDER_SIZE));
    }
    for (uint32_t s = 0; reads && s < set->springCount; ++s)
    {
        See(v, maniLoadU32(set->springs + (size_t)s * MANI_SPRING_SPRING_SIZE + 16));
    }
}

// A node's bones (those it writes, or with reads those it reads too);
// false for a kind that writes each bone from the same bone.
static bool Bones(Visit* v, const maniGraph* graph, uint32_t p, bool reads)
{
    const uint8_t* e = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
    uint32_t kind = maniLoadU16(e);
    uint32_t d = maniLoadU32(e + 12);
    if (kind == maniGraphNodeTwoBone)
    {
        for (uint32_t k = 0; k < 3; ++k)
        {
            See(v, Word(graph, d + k));
        }
    }
    else if (kind == maniGraphNodeAim)
    {
        for (uint32_t k = 0; k < Word(graph, d); ++k)
        {
            See(v, Word(graph, d + maniGraphAimBones + k));
        }
    }
    else if (kind == maniGraphNodeCcd || kind == maniGraphNodeFabrik)
    {
        Path(v, Word(graph, d + 1), Word(graph, d));
    }
    else if (kind == maniGraphNodeBodyXpbd || kind == maniGraphNodeBodySdls)
    {
        uint32_t top = Word(graph, d + maniGraphBodyTop);
        for (uint32_t k = 0; k < Word(graph, d + maniGraphBodyEffectorCount); ++k)
        {
            Path(v, Word(graph, d + MANI_GRAPH_BODY_WORDS + k * MANI_GRAPH_EFFECTOR_WORDS), top);
        }
    }
    else if (kind == maniGraphNodeSpring)
    {
        Springs(v,
                maniGraphBoundSpringSet(v->closure->binding, Word(graph, d + maniGraphSpringSet)),
                reads);
    }
    else
    {
        return false;
    }
    return true;
}

// One pass: each node writing a marked bone marks what it reads; under a
// mirror map each marked bone's partner.
static void Grow(Closure* c)
{
    const maniGraph* graph = &c->binding->graph;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        Visit writes = {c, false, false};
        if (Bones(&writes, graph, p, false) && writes.found)
        {
            Visit reads = {c, true, false};
            (void)Bones(&reads, graph, p, true);
        }
    }
    const maniMirrorMap* map = c->binding->mirrorMap;
    for (uint32_t b = 0; map != NULL && b < c->bones; ++b)
    {
        if (c->marks[b] != 0)
        {
            Mark(c, maniMirrorPartner(map, b));
        }
    }
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

// Each node runs but one writing bones from others with none planned.
static void Runs(Closure* c, uint8_t* nodes)
{
    const maniGraph* graph = &c->binding->graph;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        Visit writes = {c, false, false};
        nodes[p] = !Bones(&writes, graph, p, false) || writes.found ? 1u : 0u;
    }
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
    if (size < SizeOf(boneCount, binding->graph.nodeCount))
    {
        return mani_errorCapacity;
    }
    maniGraphSubset* plan = memory;
    *plan = (maniGraphSubset){MANI_GRAPH_SUBSET_STAMP,      0,       Groups(boneCount),
                              (uint32_t)NodesAt(boneCount), binding, binding->graph.identity};
    uint32_t* list = (uint32_t*)(plan + 1);
    // The marks sit past the longest list, where its unused end and the
    // groups' flags will be.
    Closure c = {binding, skeleton, (uint8_t*)(list + boneCount), boneCount, false};
    memset(c.marks, 0, boneCount);
    for (uint32_t i = 0; i < count; ++i)
    {
        Mark(&c, bones[i]);
    }
    do
    {
        c.grew = false;
        Grow(&c);
    } while (c.grew);
    Runs(&c, (uint8_t*)memory + plan->nodesAt);
    for (uint32_t b = 0; b < boneCount; ++b)
    {
        if (c.marks[b] != 0)
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
