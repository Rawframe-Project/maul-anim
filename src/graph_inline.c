// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Subgraphs inlined at build (mani-0023): counted first, then emitted
// scope by scope, each scope mapping its nodes, parameters, clips,
// databases and sync groups into the flat source.

#include "graph_inline.h"

#include "allocator.h"
#include "graph_check.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

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
    uint64_t transitions;
    uint64_t conditions;
    uint64_t databases;
    uint64_t queries;
    uint64_t models;
    uint64_t springSets;
} Totals;

static uint32_t WeightCount(const maniGraphNodeSource* node)
{
    if (node->kind == maniGraphNodeBlend)
    {
        return node->inputCount;
    }
    return node->kind == maniGraphNodeAdditive && node->inputCount > 0 ? node->inputCount - 1 : 0;
}

// A motion matching or learned node's query, when its count is in range
// (out of range, the check refuses it, and Remap copies none).
static uint32_t QueryCount(const maniGraphNodeSource* node)
{
    bool match = node->kind == maniGraphNodeMatch;
    bool learned = node->kind == maniGraphNodeLearned;
    const uint32_t* query = match ? node->match.query : (learned ? node->learned.query : NULL);
    uint32_t count = match ? node->match.queryCount : (learned ? node->learned.queryCount : 0u);
    return query != NULL && count <= MANI_MATCH_MAX_FEATURES ? count : 0u;
}

// A machine's transitions and conditions, when its counts are in range
// (out of range, the check refuses them, and Remap copies none).
static bool MachineCopied(const maniGraphNodeSource* node)
{
    const maniGraphMachineNodeSource* m = &node->machine;
    bool ok = node->kind == maniGraphNodeMachine && m->transitions != NULL &&
              m->transitionCount <= MANI_GRAPH_MAX_TRANSITIONS;
    for (uint32_t k = 0; k < m->transitionCount && ok; ++k)
    {
        ok = m->transitions[k].conditionCount <= MANI_GRAPH_MAX_CONDITIONS &&
             (m->transitions[k].conditions != NULL || m->transitions[k].conditionCount == 0);
    }
    return ok;
}

static void Transitions(const maniGraphNodeSource* node, Totals* t)
{
    if (!MachineCopied(node))
    {
        return;
    }
    t->transitions += node->machine.transitionCount;
    for (uint32_t k = 0; k < node->machine.transitionCount; ++k)
    {
        t->conditions += node->machine.transitions[k].conditionCount;
    }
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
    t->databases += source->databaseCount;
    t->models += source->modelCount;
    t->springSets += source->springSetCount;
    for (uint32_t i = 0; i < source->nodeCount; ++i)
    {
        const maniGraphNodeSource* node = &source->nodes[i];
        t->inputs += node->inputCount;
        t->weights += WeightCount(node);
        t->queries += QueryCount(node);
        Transitions(node, t);
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
    maniGraphTransitionSource* transitions;
    maniGraphConditionSource* conditions;
    maniGraphDatabaseSource* databases;
    uint32_t* queries;
    maniGraphModelSource* models;
    maniGraphSpringSetSource* springSets;
    uint32_t nodeCount;
    uint32_t inputCount;
    uint32_t weightCount;
    uint32_t clipCount;
    uint32_t groupCount;
    uint32_t transitionCount;
    uint32_t conditionCount;
    uint32_t databaseCount;
    uint32_t queryCount;
    uint32_t modelCount;
    uint32_t springSetCount;
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

// A subgraph's slot joins the flat slot of its asset's identity, or the
// first flat slot without an asset if it has none (the checks refuse
// it): Totals counts a slot once for every subgraph node, so the slots
// joined never outnumber it.
static uint32_t ClipSlot(const Scope* s, Flat* f, uint32_t slot)
{
    if (s->outer == NULL || slot >= s->graph->clipCount)
    {
        return slot;
    }
    const maniGraphClipSource* c = &s->graph->clips[slot];
    for (uint32_t k = 0; k < f->clipCount; ++k)
    {
        const maniClip* other = f->clips[k].clip;
        if (other != NULL && c->clip != NULL ? maniClipIdentity(other) == maniClipIdentity(c->clip)
                                             : other == c->clip)
        {
            return k;
        }
    }
    f->clips[f->clipCount] = *c;
    return f->clipCount++;
}

static uint32_t DatabaseSlot(const Scope* s, Flat* f, uint32_t slot)
{
    if (s->outer == NULL || slot >= s->graph->databaseCount)
    {
        return slot;
    }
    const maniGraphDatabaseSource* d = &s->graph->databases[slot];
    for (uint32_t k = 0; k < f->databaseCount; ++k)
    {
        const maniMotionDatabase* other = f->databases[k].database;
        if (other != NULL && d->database != NULL
                ? maniMotionDatabaseIdentity(other) == maniMotionDatabaseIdentity(d->database)
                : other == d->database)
        {
            return k;
        }
    }
    // Its clips join the root's too, whether or not a clip node plays
    // them.
    for (uint32_t k = 0; k < s->graph->clipCount && d->database != NULL; ++k)
    {
        const maniClip* clip = s->graph->clips[k].clip;
        for (uint32_t c = 0; c < d->database->clipCount && clip != NULL; ++c)
        {
            if (maniMotionDatabaseClipIdentity(d->database, c) == maniClipIdentity(clip))
            {
                (void)ClipSlot(s, f, k);
            }
        }
    }
    f->databases[f->databaseCount] = *d;
    return f->databaseCount++;
}

static uint32_t ModelSlot(const Scope* s, Flat* f, uint32_t slot)
{
    if (s->outer == NULL || slot >= s->graph->modelCount)
    {
        return slot;
    }
    const maniGraphModelSource* m = &s->graph->models[slot];
    for (uint32_t k = 0; k < f->modelCount; ++k)
    {
        const maniLearnedModel* other = f->models[k].model;
        if (other != NULL && m->model != NULL
                ? maniLearnedModelIdentity(other) == maniLearnedModelIdentity(m->model)
                : other == m->model)
        {
            return k;
        }
    }
    f->models[f->modelCount] = *m;
    return f->modelCount++;
}

static uint32_t SpringSetSlot(const Scope* s, Flat* f, uint32_t slot)
{
    if (s->outer == NULL || slot >= s->graph->springSetCount)
    {
        return slot;
    }
    const maniGraphSpringSetSource* p = &s->graph->springSets[slot];
    for (uint32_t k = 0; k < f->springSetCount; ++k)
    {
        const maniSpringSet* other = f->springSets[k].set;
        if (other != NULL && p->set != NULL
                ? maniSpringSetIdentity(other) == maniSpringSetIdentity(p->set)
                : other == p->set)
        {
            return k;
        }
    }
    f->springSets[f->springSetCount] = *p;
    return f->springSetCount++;
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

// A machine's transitions copied, their conditions' parameters as the
// root reads them.
static void RemapMachine(const Scope* s, Flat* f, maniGraphNodeSource* node)
{
    if (!MachineCopied(node))
    {
        return;
    }
    maniGraphMachineNodeSource* m = &node->machine;
    maniGraphTransitionSource* to = f->transitions + f->transitionCount;
    for (uint32_t k = 0; k < m->transitionCount; ++k)
    {
        to[k] = m->transitions[k];
        maniGraphConditionSource* c = f->conditions + f->conditionCount;
        for (uint32_t j = 0; j < to[k].conditionCount; ++j)
        {
            c[j] = to[k].conditions[j];
            bool reads =
                c[j].kind == maniGraphConditionParameter || c[j].kind == maniGraphConditionTrigger;
            c[j].parameter = reads ? Resolve(s, c[j].parameter) : c[j].parameter;
        }
        to[k].conditions = to[k].conditionCount > 0 ? c : NULL;
        f->conditionCount += to[k].conditionCount;
    }
    m->transitions = to;
    f->transitionCount += m->transitionCount;
}

// A motion matching node's database, parameters and query as the root
// reads them.
static void RemapMatch(const Scope* s, Flat* f, maniGraphMatchNodeSource* m, uint32_t queries)
{
    m->database = DatabaseSlot(s, f, m->database);
    m->trigger = Resolve(s, m->trigger);
    m->requiredTags.parameter = Resolve(s, m->requiredTags.parameter);
    m->excludedTags.parameter = Resolve(s, m->excludedTags.parameter);
    m->duration = Scalar(s, m->duration);
    uint32_t* to = f->queries + f->queryCount;
    for (uint32_t q = 0; q < queries; ++q)
    {
        to[q] = Resolve(s, m->query[q]);
    }
    f->queryCount += queries;
    m->query = queries > 0 ? to : m->query;
}

// A learned node's model, parameters and query as the root reads them.
static void RemapLearned(const Scope* s, Flat* f, maniGraphLearnedNodeSource* l, uint32_t queries)
{
    l->model = ModelSlot(s, f, l->model);
    l->trigger = Resolve(s, l->trigger);
    l->requiredTags.parameter = Resolve(s, l->requiredTags.parameter);
    l->excludedTags.parameter = Resolve(s, l->excludedTags.parameter);
    l->duration = Scalar(s, l->duration);
    uint32_t* to = f->queries + f->queryCount;
    for (uint32_t q = 0; q < queries; ++q)
    {
        to[q] = Resolve(s, l->query[q]);
    }
    f->queryCount += queries;
    l->query = queries > 0 ? to : l->query;
}

// A node's parameters, clip, sync group and weights as the root reads
// them.
static void Remap(const Scope* s, Flat* f, maniGraphNodeSource* node)
{
    switch (node->kind)
    {
    case maniGraphNodeMatch:
        RemapMatch(s, f, &node->match, QueryCount(node));
        return;
    case maniGraphNodeLearned:
        RemapLearned(s, f, &node->learned, QueryCount(node));
        return;
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
    case maniGraphNodeMachine:
        RemapMachine(s, f, node);
        return;
    case maniGraphNodeInertialize:
        node->inertialize.trigger = Resolve(s, node->inertialize.trigger);
        node->inertialize.duration = Scalar(s, node->inertialize.duration);
        return;
    case maniGraphNodeSpring:
        node->spring.springSet = SpringSetSlot(s, f, node->spring.springSet);
        node->spring.position = Resolve(s, node->spring.position);
        node->spring.rotation = Resolve(s, node->spring.rotation);
        node->spring.reset = Resolve(s, node->spring.reset);
        return;
    case maniGraphNodeOneShot:
        node->oneShot.fire = Resolve(s, node->oneShot.fire);
        node->oneShot.fadeOutTrigger = Resolve(s, node->oneShot.fadeOutTrigger);
        node->oneShot.abort = Resolve(s, node->oneShot.abort);
        return;
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

// The totals within the lists' limits.
static bool TotalsFit(const Totals* t)
{
    return t->inputs <= UINT32_MAX && t->weights <= UINT32_MAX &&
           t->clips <= MANI_GRAPH_MAX_CLIPS && t->groups <= MANI_GRAPH_MAX_SYNC_GROUPS &&
           t->transitions <= UINT32_MAX && t->conditions <= UINT32_MAX &&
           t->databases <= MANI_GRAPH_MAX_DATABASES && t->queries <= UINT32_MAX &&
           t->models <= MANI_GRAPH_MAX_MODELS && t->springSets <= MANI_GRAPH_MAX_SPRING_SETS;
}

// The root's slots and sync groups first, as it lists them.
static void CopyRoot(const maniGraphSource* source, Flat* f)
{
    for (uint32_t k = 0; k < source->modelCount && source->models != NULL; ++k)
    {
        f->models[f->modelCount++] = source->models[k];
    }
    for (uint32_t k = 0; k < source->springSetCount && source->springSets != NULL; ++k)
    {
        f->springSets[f->springSetCount++] = source->springSets[k];
    }
    for (uint32_t k = 0; k < source->databaseCount && source->databases != NULL; ++k)
    {
        f->databases[f->databaseCount++] = source->databases[k];
    }
    for (uint32_t k = 0; k < source->clipCount && source->clips != NULL; ++k)
    {
        f->clips[f->clipCount++] = source->clips[k];
    }
    for (uint32_t k = 0; k < source->syncGroupCount && source->syncGroups != NULL; ++k)
    {
        f->groups[f->groupCount++] = source->syncGroups[k];
    }
}

maniBuildResult maniInlineSubgraphs(const maniGraphSource* source, const maniAllocator* allocator,
                                    maniGraphFlat* out)
{
    *out = (maniGraphFlat){*source, NULL, 0};
    if (source->nodes == NULL || !HasSubgraph(source))
    {
        return (maniBuildResult){mani_success, maniBuildReasonNone, 0, 0};
    }
    Totals t = {source->nodeCount, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!Count(source, 0, &t) || !TotalsFit(&t))
    {
        return maniGraphRefuse(maniBuildReasonCount, 0, 0);
    }
    size_t sizes[11] = {(size_t)t.nodes * sizeof(maniGraphNodeSource),
                        (size_t)t.inputs * sizeof(uint32_t),
                        (size_t)t.weights * sizeof(maniGraphScalar),
                        (size_t)t.clips * sizeof(maniGraphClipSource),
                        (size_t)t.groups * sizeof(maniGraphSyncGroupSource),
                        (size_t)t.transitions * sizeof(maniGraphTransitionSource),
                        (size_t)t.conditions * sizeof(maniGraphConditionSource),
                        (size_t)t.databases * sizeof(maniGraphDatabaseSource),
                        (size_t)t.queries * sizeof(uint32_t),
                        (size_t)t.models * sizeof(maniGraphModelSource),
                        (size_t)t.springSets * sizeof(maniGraphSpringSetSource)};
    size_t total = 0;
    for (int k = 0; k < 11; ++k)
    {
        sizes[k] = (sizes[k] + 7u) & ~(size_t)7u;
        total += sizes[k];
    }
    uint8_t* block = maniAllocate(allocator, total, 1, 8);
    if (block == NULL)
    {
        return (maniBuildResult){mani_errorCapacity, maniBuildReasonNone, 0, 0};
    }
    void* parts[11];
    size_t at = 0;
    for (int k = 0; k < 11; ++k)
    {
        parts[k] = block + at;
        at += sizes[k];
    }
    Flat f = {.nodes = parts[0],
              .inputs = parts[1],
              .weights = parts[2],
              .clips = parts[3],
              .groups = parts[4],
              .transitions = parts[5],
              .conditions = parts[6],
              .databases = parts[7],
              .queries = parts[8],
              .models = parts[9],
              .springSets = parts[10],
              .nodeCount = source->nodeCount};
    CopyRoot(source, &f);
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
    out->source.databases = f.databases;
    out->source.databaseCount = f.databaseCount;
    out->source.models = f.models;
    out->source.modelCount = f.modelCount;
    out->source.springSets = f.springSets;
    out->source.springSetCount = f.springSetCount;
    return r;
}

void maniReleaseInlined(maniGraphFlat* flat, const maniAllocator* allocator)
{
    maniRelease(allocator, flat->block, flat->blockSize, 1, 8);
    flat->block = NULL;
}
