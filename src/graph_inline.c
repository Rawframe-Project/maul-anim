// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Subgraphs inlined at build (mani-0023): counted first, then emitted
// scope by scope, each scope mapping its nodes, parameters, clips and
// sync groups into the flat source.

#include "graph_inline.h"

#include "allocator.h"
#include "graph_check.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct Totals
{
    uint64_t nodes;
    uint64_t inputs;
    uint64_t weights;
    uint64_t clips;
    uint64_t groups;
} Totals;

static uint32_t WeightCount(const maniGraphNodeSource* node)
{
    if (node->kind == maniGraphNodeBlend)
    {
        return node->inputCount;
    }
    return node->kind == maniGraphNodeAdditive && node->inputCount > 0 ? node->inputCount - 1 : 0;
}

// What a source and its subgraphs hold; false past the depth or with a
// subgraph node naming no graph or a graph without nodes.
static bool Count(const maniGraphSource* source, uint32_t depth, Totals* t)
{
    if (depth > MANI_GRAPH_MAX_DEPTH || source->nodeCount == 0 || source->nodes == NULL)
    {
        return false;
    }
    t->clips += source->clipCount;
    t->groups += source->syncGroupCount;
    for (uint32_t i = 0; i < source->nodeCount; ++i)
    {
        const maniGraphNodeSource* node = &source->nodes[i];
        t->inputs += node->inputCount;
        t->weights += WeightCount(node);
        if (node->kind != maniGraphNodeSubgraph)
        {
            continue;
        }
        const maniGraphSource* sub = node->subgraph.graph;
        if (sub == NULL || sub->nodeCount == 0 || !Count(sub, depth + 1, t))
        {
            return false;
        }
        t->nodes += sub->nodeCount - 1;
    }
    return t->nodes <= MANI_GRAPH_MAX_NODES;
}

static bool HasSubgraph(const maniGraphSource* source)
{
    for (uint32_t i = 0; i < source->nodeCount; ++i)
    {
        if (source->nodes[i].kind == maniGraphNodeSubgraph)
        {
            return true;
        }
    }
    return false;
}

// A graph's scope while it is emitted: its own node j stands at Place.
typedef struct Scope
{
    const maniGraphSource* graph;
    const uint32_t* parameters;
    const struct Scope* outer;
    uint32_t place;
    uint32_t base;
} Scope;

typedef struct Flat
{
    maniGraphNodeSource* nodes;
    uint32_t* inputs;
    maniGraphScalar* weights;
    maniGraphClipSource* clips;
    maniGraphSyncGroupSource* groups;
    uint32_t nodeCount;
    uint32_t inputCount;
    uint32_t weightCount;
    uint32_t clipCount;
    uint32_t groupCount;
    uint32_t reserved;
} Flat;

static uint32_t Place(const Scope* s, uint32_t j)
{
    if (j >= s->graph->nodeCount)
    {
        return UINT32_MAX;
    }
    if (s->outer == NULL)
    {
        return j;
    }
    uint32_t out = s->graph->output;
    return j == out ? s->place : s->base + j - (j > out ? 1u : 0u);
}

// A parameter of the scope as the root's: through each map outward;
// MANI_NO_PARAMETER stays, one past a map becomes UINT32_MAX - 1.
static uint32_t Resolve(const Scope* s, uint32_t parameter)
{
    for (; s->outer != NULL && parameter != MANI_NO_PARAMETER; s = s->outer)
    {
        if (parameter >= s->graph->parameterCount)
        {
            return UINT32_MAX - 1;
        }
        parameter = s->parameters[parameter];
    }
    return parameter;
}

static uint32_t ClipSlot(const Scope* s, Flat* f, uint32_t slot)
{
    if (s->outer == NULL || slot >= s->graph->clipCount)
    {
        return slot;
    }
    const maniGraphClipSource* c = &s->graph->clips[slot];
    uint64_t identity = c->clip != NULL ? maniClipIdentity(c->clip) : 0;
    for (uint32_t k = 0; k < f->clipCount; ++k)
    {
        if (f->clips[k].clip != NULL && maniClipIdentity(f->clips[k].clip) == identity)
        {
            return k;
        }
    }
    f->clips[f->clipCount] = *c;
    return f->clipCount++;
}

static uint32_t GroupSlot(const Scope* s, Flat* f, uint32_t group)
{
    if (s->outer == NULL || group >= s->graph->syncGroupCount)
    {
        return group;
    }
    const maniGraphSyncGroupSource* g = &s->graph->syncGroups[group];
    for (uint32_t k = 0; k < f->groupCount; ++k)
    {
        const maniGraphSyncGroupSource* h = &f->groups[k];
        if (h->nameLength == g->nameLength && h->name != NULL && g->name != NULL &&
            memcmp(h->name, g->name, g->nameLength) == 0)
        {
            return k;
        }
    }
    f->groups[f->groupCount] = *g;
    return f->groupCount++;
}

static maniGraphScalar Scalar(const Scope* s, maniGraphScalar v)
{
    return (maniGraphScalar){Resolve(s, v.parameter), v.value};
}

// A node's parameters, clip, sync group and weights as the root reads
// them.
static void Remap(const Scope* s, Flat* f, maniGraphNodeSource* node)
{
    switch (node->kind)
    {
    case maniGraphNodeClip:
        node->clip.clip = ClipSlot(s, f, node->clip.clip);
        node->clip.syncGroup = GroupSlot(s, f, node->clip.syncGroup);
        node->clip.speed = Scalar(s, node->clip.speed);
        return;
    case maniGraphNodeBlend:
    case maniGraphNodeAdditive:
    {
        bool blend = node->kind == maniGraphNodeBlend;
        const maniGraphScalar* from = blend ? node->blend.weights : node->additive.weights;
        maniGraphScalar* to = f->weights + f->weightCount;
        for (uint32_t k = 0; k < WeightCount(node) && from != NULL; ++k)
        {
            to[k] = Scalar(s, from[k]);
        }
        f->weightCount += WeightCount(node);
        if (blend)
        {
            node->blend.weights = from != NULL ? to : NULL;
        }
        else
        {
            node->additive.weights = from != NULL ? to : NULL;
        }
        return;
    }
    case maniGraphNodeSelect:
        node->select.parameter = Resolve(s, node->select.parameter);
        return;
    case maniGraphNodeTimeScale:
        node->timeScale.scale = Scalar(s, node->timeScale.scale);
        return;
    case maniGraphNodeSeek:
        node->seek.trigger = Resolve(s, node->seek.trigger);
        node->seek.time = Scalar(s, node->seek.time);
        return;
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceTriangles:
    case maniGraphNodeBlendSpaceBands:
    {
        maniGraphPosition* p =
            node->kind == maniGraphNodeBlendSpace1D      ? &node->blendSpace1D.position
            : node->kind == maniGraphNodeBlendSpaceBands ? &node->bands.position
                                                         : &node->triangulated.position;
        p->vector = Resolve(s, p->vector);
        p->x = Scalar(s, p->x);
        p->y = Scalar(s, p->y);
        return;
    }
    default:
        return;
    }
}

// The subgraph a node names, of the skeleton, its parameters mapped to
// this scope's of their types, its output in range, taking no inputs.
static bool SubgraphValid(const Scope* s, const maniGraphNodeSource* node,
                          const maniSkeleton* skeleton)
{
    const maniGraphSource* sub = node->subgraph.graph;
    if (node->inputCount != 0 || sub->skeleton == NULL ||
        maniSkeletonIdentity(sub->skeleton) != maniSkeletonIdentity(skeleton) ||
        sub->output >= sub->nodeCount ||
        (sub->parameterCount > 0 && (node->subgraph.parameters == NULL || sub->parameters == NULL)))
    {
        return false;
    }
    for (uint32_t k = 0; k < sub->parameterCount; ++k)
    {
        uint32_t p = node->subgraph.parameters[k];
        if (p >= s->graph->parameterCount ||
            s->graph->parameters[p].type != sub->parameters[k].type)
        {
            return false;
        }
    }
    return true;
}

static maniBuildResult Emit(const Scope* s, Flat* f, const maniSkeleton* skeleton);

static maniBuildResult EmitNode(const Scope* s, Flat* f, const maniSkeleton* skeleton, uint32_t j)
{
    uint32_t place = Place(s, j);
    maniGraphNodeSource node = s->graph->nodes[j];
    if (node.kind == maniGraphNodeSubgraph)
    {
        if (!SubgraphValid(s, &node, skeleton))
        {
            return maniGraphRefuse(maniBuildReasonNode, place, 0);
        }
        const maniGraphSource* sub = node.subgraph.graph;
        Scope inner = {sub, node.subgraph.parameters, s, place, f->nodeCount};
        f->nodeCount += sub->nodeCount - 1;
        return Emit(&inner, f, skeleton);
    }
    uint32_t* inputs = f->inputs + f->inputCount;
    for (uint32_t k = 0; k < node.inputCount && node.inputs != NULL; ++k)
    {
        inputs[k] = Place(s, node.inputs[k]);
    }
    f->inputCount += node.inputCount;
    node.inputs = node.inputs != NULL ? inputs : NULL;
    Remap(s, f, &node);
    f->nodes[place] = node;
    return (maniBuildResult){mani_success, maniBuildReasonNone, 0, 0};
}

// Emits a scope's nodes; the counting before it bounds the nesting.
static maniBuildResult Emit(const Scope* s, Flat* f, const maniSkeleton* skeleton)
{
    maniBuildResult r = {mani_success, maniBuildReasonNone, 0, 0};
    for (uint32_t j = 0; j < s->graph->nodeCount && r.result == mani_success; ++j)
    {
        r = EmitNode(s, f, skeleton, j);
    }
    return r;
}

maniBuildResult maniInlineSubgraphs(const maniGraphSource* source, const maniAllocator* allocator,
                                    maniGraphFlat* out)
{
    *out = (maniGraphFlat){*source, NULL, 0};
    if (source->nodes == NULL || !HasSubgraph(source))
    {
        return (maniBuildResult){mani_success, maniBuildReasonNone, 0, 0};
    }
    Totals t = {source->nodeCount, 0, 0, 0, 0};
    if (!Count(source, 0, &t) || t.inputs > UINT32_MAX || t.weights > UINT32_MAX ||
        t.clips > MANI_GRAPH_MAX_CLIPS || t.groups > MANI_GRAPH_MAX_SYNC_GROUPS)
    {
        return maniGraphRefuse(maniBuildReasonCount, 0, 0);
    }
    size_t sizes[5] = {
        (size_t)t.nodes * sizeof(maniGraphNodeSource), (size_t)t.inputs * sizeof(uint32_t),
        (size_t)t.weights * sizeof(maniGraphScalar), (size_t)t.clips * sizeof(maniGraphClipSource),
        (size_t)t.groups * sizeof(maniGraphSyncGroupSource)};
    size_t total = 0;
    for (int k = 0; k < 5; ++k)
    {
        sizes[k] = (sizes[k] + 7u) & ~(size_t)7u;
        total += sizes[k];
    }
    uint8_t* block = maniAllocate(allocator, total, 1, 8);
    if (block == NULL)
    {
        return (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0};
    }
    void* parts[5];
    size_t at = 0;
    for (int k = 0; k < 5; ++k)
    {
        parts[k] = block + at;
        at += sizes[k];
    }
    Flat f = {parts[0], parts[1], parts[2], parts[3], parts[4], source->nodeCount, 0, 0, 0, 0, 0};
    for (uint32_t k = 0; k < source->clipCount && source->clips != NULL; ++k)
    {
        f.clips[f.clipCount++] = source->clips[k];
    }
    for (uint32_t k = 0; k < source->syncGroupCount && source->syncGroups != NULL; ++k)
    {
        f.groups[f.groupCount++] = source->syncGroups[k];
    }
    Scope root = {source, NULL, NULL, 0, 0};
    maniBuildResult r = Emit(&root, &f, source->skeleton);
    out->block = block;
    out->blockSize = total;
    out->source.nodes = f.nodes;
    out->source.nodeCount = f.nodeCount;
    out->source.clips = f.clips;
    out->source.clipCount = f.clipCount;
    out->source.syncGroups = f.groups;
    out->source.syncGroupCount = f.groupCount;
    return r;
}

void maniReleaseInlined(maniGraphFlat* flat, const maniAllocator* allocator)
{
    maniRelease(allocator, flat->block, flat->blockSize, 1, 8);
    flat->block = NULL;
}
