// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binding a graph to its skeleton, clips and motion databases (mani-0023,
// mani-0024): each slot found by identity among the host's, in caller
// memory. A database's clips must be the slots the graph lists for it,
// its skeleton the graph's, and its channels must take the query of
// every node searching it, so a file naming a database by identity
// cannot pair it with a query or clips it was not built for.

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_learned.h"
#include "graph_match.h"
#include "graph_spring.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

size_t maniGraphBindingSize(const maniGraph* graph)
{
    if (graph == NULL)
    {
        return 0;
    }
    return offsetof(maniGraphBinding, clips) + (size_t)graph->clipCount * sizeof(const maniClip*) +
           (size_t)graph->databaseCount * sizeof(const maniMotionDatabase*) +
           (size_t)graph->modelCount * sizeof(const maniLearnedModel*) +
           (size_t)graph->springSetCount * sizeof(const maniSpringSet*);
}

// The clip of the slot's identity among clips, of the graph's skeleton
// and of the slot's kind; NULL for none.
static const maniClip* Find(const maniGraph* graph, uint32_t slot, const maniClip* const* clips,
                            uint32_t count)
{
    const uint8_t* e = graph->clips + (size_t)slot * MANI_GRAPH_CLIP_SIZE;
    uint64_t identity = maniLoadU64(e);
    bool difference = maniLoadU32(e + 24) != 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        const maniClip* clip = clips[i];
        if (clip != NULL && maniClipIdentity(clip) == identity &&
            maniClipSkeletonIdentity(clip) == graph->skeletonIdentity &&
            (maniClipAdditiveBasis(clip) != maniAdditiveNone) == difference)
        {
            return clip;
        }
    }
    return NULL;
}

// Whether a database takes the slot: its clips the slot's, in order, and
// the query of every node searching the slot.
static bool DatabaseFits(const maniGraph* graph, uint32_t slot, const maniMotionDatabase* d)
{
    const uint8_t* e = graph->databases + (size_t)slot * MANI_GRAPH_DATABASE_SIZE;
    uint32_t first = maniLoadU32(e + 24);
    uint32_t count = maniLoadU32(e + 28);
    if (maniMotionDatabaseSkeletonIdentity(d) != graph->skeletonIdentity || d->clipCount != count)
    {
        return false;
    }
    for (uint32_t c = 0; c < count; ++c)
    {
        uint32_t clip = maniLoadU32(graph->databaseClips + 4 * (size_t)(first + c));
        if (maniMotionDatabaseClipIdentity(d, c) != maniGraphClipIdentity(graph, clip))
        {
            return false;
        }
    }
    uint32_t types[MANI_MATCH_MAX_FEATURES];
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* record = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        const uint8_t* words = graph->data + 4 * (size_t)maniLoadU32(record + 12);
        if (maniLoadU16(record) != maniGraphNodeMatch ||
            maniLoadU32(words + 4 * maniGraphMatchDatabase) != slot)
        {
            continue;
        }
        uint32_t queries = maniLoadU32(words + 4 * maniGraphMatchQueryCount);
        for (uint32_t q = 0; q < queries; ++q)
        {
            uint32_t parameter = maniLoadU32(words + 4 * (size_t)(maniGraphMatchQuery + q));
            types[q] =
                maniLoadU32(graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE);
        }
        if (!maniGraphQueryFits(d, d->channelCount, maniGraphDatabaseChannel, types, queries))
        {
            return false;
        }
    }
    return true;
}

// The database of the slot's identity among databases that takes it;
// NULL for none.
static const maniMotionDatabase* FindDatabase(const maniGraph* graph, uint32_t slot,
                                              const maniMotionDatabase* const* databases,
                                              uint32_t count)
{
    uint64_t identity = maniGraphDatabaseIdentity(graph, slot);
    for (uint32_t i = 0; i < count; ++i)
    {
        const maniMotionDatabase* d = databases[i];
        if (d != NULL && maniMotionDatabaseIdentity(d) == identity)
        {
            return DatabaseFits(graph, slot, d) ? d : NULL;
        }
    }
    return NULL;
}

static size_t CursorSize(const maniGraphBinding* binding)
{
    size_t largest = 0;
    for (uint32_t s = 0; s < binding->graph.clipCount; ++s)
    {
        size_t size = maniClipCursorSize(binding->clips[s]);
        largest = size > largest ? size : largest;
    }
    return (largest + 7u) & ~(size_t)7u;
}

// The one bone the clips' root motion names, into the binding; false
// when two clips name different bones.
static bool RootMotionBone(maniGraphBinding* binding)
{
    binding->rootMotionBone = MANI_NO_BONE;
    for (uint32_t s = 0; s < binding->graph.clipCount; ++s)
    {
        maniRootMotionSource source;
        if (!maniClipRootMotionSource(binding->clips[s], &source))
        {
            continue;
        }
        if (binding->rootMotionBone != MANI_NO_BONE && binding->rootMotionBone != source.bone)
        {
            return false;
        }
        binding->rootMotionBone = source.bone;
    }
    return true;
}

// Each database slot bound; false with the first left unbound.
static bool BindDatabases(maniGraphBinding* binding, const maniMotionDatabase* const* databases,
                          uint32_t count, uint32_t* unboundOut)
{
    const maniGraph* graph = &binding->graph;
    const maniMotionDatabase** slots = maniGraphBoundDatabases(binding);
    for (uint32_t s = 0; s < graph->databaseCount; ++s)
    {
        slots[s] = FindDatabase(graph, s, databases, count);
        if (slots[s] == NULL)
        {
            if (unboundOut != NULL)
            {
                *unboundOut = graph->clipCount + s;
            }
            return false;
        }
    }
    return true;
}

// Whether a model takes the slot: of the graph's skeleton, and taking
// every learned node playing the slot.
static bool ModelFits(const maniGraph* graph, uint32_t slot, const maniLearnedModel* m)
{
    if (maniLearnedModelSkeletonIdentity(m) != graph->skeletonIdentity)
    {
        return false;
    }
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* record = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        uint32_t data = maniLoadU32(record + 12);
        if (maniLoadU16(record) == maniGraphNodeLearned &&
            maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphLearnedModel)) == slot &&
            !maniGraphLearnedFits(graph, data, m))
        {
            return false;
        }
    }
    return true;
}

// Each model slot bound to the model of its identity that takes it;
// false with the first left unbound.
static bool BindModels(maniGraphBinding* binding, const maniGraphAssets* assets,
                       uint32_t* unboundOut)
{
    const maniGraph* graph = &binding->graph;
    const maniLearnedModel** slots = maniGraphBoundModels(binding);
    for (uint32_t s = 0; s < graph->modelCount; ++s)
    {
        uint64_t identity = maniGraphModelIdentity(graph, s);
        slots[s] = NULL;
        for (uint32_t i = 0; i < assets->modelCount && slots[s] == NULL; ++i)
        {
            const maniLearnedModel* m = assets->models[i];
            bool fits =
                m != NULL && maniLearnedModelIdentity(m) == identity && ModelFits(graph, s, m);
            slots[s] = fits ? m : NULL;
        }
        if (slots[s] == NULL)
        {
            if (unboundOut != NULL)
            {
                *unboundOut = graph->clipCount + graph->databaseCount + s;
            }
            return false;
        }
    }
    return true;
}

// Whether a set takes the slot: of the graph's skeleton, and its state
// the words of every spring node over the slot.
static bool SpringSetFits(const maniGraph* graph, uint32_t slot, const maniSpringSet* set)
{
    if (maniSpringSetSkeletonIdentity(set) != graph->skeletonIdentity)
    {
        return false;
    }
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        const uint8_t* record = graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE;
        uint32_t data = maniLoadU32(record + 12);
        if (maniLoadU16(record) == maniGraphNodeSpring &&
            maniLoadU32(graph->data + 4 * (size_t)(data + maniGraphSpringSet)) == slot &&
            !maniGraphSpringFits(graph, data, set))
        {
            return false;
        }
    }
    return true;
}

// Each spring set slot bound to the set of its identity that takes it;
// false with the first left unbound.
static bool BindSpringSets(maniGraphBinding* binding, const maniGraphAssets* assets,
                           uint32_t* unboundOut)
{
    const maniGraph* graph = &binding->graph;
    const maniSpringSet** slots = maniGraphBoundSpringSets(binding);
    for (uint32_t s = 0; s < graph->springSetCount; ++s)
    {
        uint64_t identity = maniGraphSpringSetIdentity(graph, s);
        slots[s] = NULL;
        for (uint32_t i = 0; i < assets->springSetCount && slots[s] == NULL; ++i)
        {
            const maniSpringSet* p = assets->springSets[i];
            bool fits =
                p != NULL && maniSpringSetIdentity(p) == identity && SpringSetFits(graph, s, p);
            slots[s] = fits ? p : NULL;
        }
        if (slots[s] == NULL)
        {
            if (unboundOut != NULL)
            {
                *unboundOut = graph->clipCount + graph->databaseCount + graph->modelCount + s;
            }
            return false;
        }
    }
    return true;
}

static bool AssetsValid(const maniGraph* graph, const maniGraphAssets* a)
{
    return a != NULL && a->skeleton != NULL && (a->clips != NULL || a->clipCount == 0) &&
           (a->databases != NULL || a->databaseCount == 0) &&
           (a->models != NULL || a->modelCount == 0) &&
           (a->springSets != NULL || a->springSetCount == 0) &&
           maniSkeletonIdentity(a->skeleton) == graph->skeletonIdentity &&
           maniSkeletonBoneCount(a->skeleton) == graph->skeletonBoneCount;
}

maniResult maniBindGraph(const maniGraph* graph, const maniGraphAssets* assets, void* memory,
                         size_t size, maniGraphBinding** bindingOut, uint32_t* unboundOut)
{
    if (unboundOut != NULL)
    {
        *unboundOut = MANI_NO_TRACK;
    }
    if (graph == NULL || memory == NULL || bindingOut == NULL ||
        (uintptr_t)memory % MANI_GRAPH_BINDING_ALIGNMENT != 0 || !AssetsValid(graph, assets))
    {
        return mani_errorInvalid;
    }
    const maniClip* const* clips = assets->clips;
    uint32_t count = assets->clipCount;
    if (size < maniGraphBindingSize(graph))
    {
        return mani_errorCapacity;
    }
    maniGraphBinding* binding = memory;
    binding->graph = *graph;
    for (uint32_t s = 0; s < graph->clipCount; ++s)
    {
        binding->clips[s] = Find(graph, s, clips, count);
        if (binding->clips[s] == NULL)
        {
            if (unboundOut != NULL)
            {
                *unboundOut = s;
            }
            return mani_errorInvalid;
        }
    }
    if (!BindDatabases(binding, assets->databases, assets->databaseCount, unboundOut) ||
        !BindModels(binding, assets, unboundOut) || !BindSpringSets(binding, assets, unboundOut) ||
        !RootMotionBone(binding))
    {
        return mani_errorInvalid;
    }
    binding->stamp = MANI_GRAPH_BINDING_STAMP;
    binding->reserved = 0;
    binding->maxInputs = 0;
    for (uint32_t p = 0; p < graph->nodeCount; ++p)
    {
        uint32_t n = maniLoadU32(graph->nodes + (size_t)p * MANI_GRAPH_NODE_SIZE + 8);
        binding->maxInputs = n > binding->maxInputs ? n : binding->maxInputs;
    }
    binding->skeleton = assets->skeleton;
    binding->cursorSize = CursorSize(binding);
    *bindingOut = binding;
    return mani_success;
}
