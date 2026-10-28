// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binding a graph to its skeleton, clips and motion databases:
// each slot found by identity among the host's, in caller
// memory. A database's clips must be the slots the graph lists for it,
// its skeleton the graph's, and its channels must take the query of
// every node searching it, so a file naming a database by identity
// cannot pair it with a query or clips it was not built for.

#include "bytes.h"
#include "fpenv.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_learned.h"
#include "graph_match.h"
#include "graph_spring.h"
#include "retarget_motion.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"
#include "maul-anim/mirror.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
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
           (size_t)graph->springSetCount * sizeof(const maniSpringSet*) +
           (size_t)graph->mapCount * sizeof(maniGraphBoundMapPair);
}

// The clip of the slot's identity among clips, of the graph's skeleton
// (its map's source for a mapped slot) and of the slot's kind; NULL for
// none.
static const maniClip* Find(const maniGraphBinding* binding, uint32_t slot,
                            const maniClip* const* clips, uint32_t count)
{
    const maniGraph* graph = &binding->graph;
    const uint8_t* e = graph->clips + (size_t)slot * MANI_GRAPH_CLIP_SIZE;
    uint64_t identity = maniLoadU64(e);
    bool difference = maniLoadU32(e + 24) != 0;
    const maniGraphBoundMapPair* pair = maniGraphClipMapPair(binding, slot);
    const maniSkeleton* skeleton = pair != NULL ? pair->source : binding->skeleton;
    for (uint32_t i = 0; i < count; ++i)
    {
        const maniClip* clip = clips[i];
        if (clip != NULL && maniClipIdentity(clip) == identity &&
            maniClipSkeletonIdentity(clip) == maniSkeletonIdentity(skeleton) &&
            clip->skeletonBoneCount == maniSkeletonBoneCount(skeleton) &&
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
    uint64_t mirror = maniMotionDatabaseMirrorMapIdentity(d);
    if (maniMotionDatabaseSkeletonIdentity(d) != graph->skeletonIdentity || d->clipCount != count ||
        (mirror != 0 && mirror != graph->mirrorMapIdentity))
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
    uint32_t types[MANI_MAX_MATCH_FEATURES];
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

// The one bone the clips' root motion names (a mapped clip's, the target
// bone following its own alone), into the binding; false when two clips
// name different bones or a mapped clip's has no follower.
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
        const maniGraphBoundMapPair* pair = maniGraphClipMapPair(binding, s);
        uint32_t bone = pair != NULL ? maniRetargetFollower(pair->map, source.bone) : source.bone;
        if (bone == MANI_NO_BONE ||
            (binding->rootMotionBone != MANI_NO_BONE && binding->rootMotionBone != bone))
        {
            return false;
        }
        binding->rootMotionBone = bone;
    }
    return true;
}

static size_t PoseRound(size_t size)
{
    return (size + MANI_POSE_ALIGNMENT - 1) / MANI_POSE_ALIGNMENT * MANI_POSE_ALIGNMENT;
}

// The map of a slot's identity among the maps, made for the graph's
// skeleton; NULL for none.
static const maniRetargetMap* FindMap(const maniGraph* graph, uint32_t slot,
                                      const maniGraphAssets* assets)
{
    for (uint32_t i = 0; i < assets->retargetMapCount; ++i)
    {
        const maniRetargetMap* m = assets->retargetMaps[i];
        if (m != NULL && m->identity == maniGraphRetargetMapIdentity(graph, slot) &&
            m->targetIdentity == graph->skeletonIdentity &&
            m->targetBoneCount == graph->skeletonBoneCount)
        {
            return m;
        }
    }
    return NULL;
}

// A map's source skeleton among the source skeletons; NULL for none.
static const maniSkeleton* FindSource(const maniRetargetMap* map, const maniGraphAssets* assets)
{
    for (uint32_t i = 0; i < assets->sourceSkeletonCount; ++i)
    {
        const maniSkeleton* k = assets->sourceSkeletons[i];
        if (k != NULL && maniSkeletonIdentity(k) == map->sourceIdentity &&
            maniSkeletonBoneCount(k) == map->sourceBoneCount)
        {
            return k;
        }
    }
    return NULL;
}

// Each map slot bound to its map and the map's source skeleton, the
// scratch for carrying poses sized; false with the first left unbound.
static bool BindMaps(maniGraphBinding* binding, const maniGraphAssets* assets, uint32_t* unboundOut)
{
    const maniGraph* graph = &binding->graph;
    maniGraphBoundMapPair* pairs = (maniGraphBoundMapPair*)maniGraphBoundMap(binding, 0);
    binding->sourcePoseSize = 0;
    binding->retargetScratchSize = 0;
    for (uint32_t s = 0; s < graph->mapCount; ++s)
    {
        const maniRetargetMap* map = FindMap(graph, s, assets);
        const maniSkeleton* source = map != NULL ? FindSource(map, assets) : NULL;
        if (source == NULL)
        {
            if (unboundOut != NULL)
            {
                *unboundOut = graph->clipCount + graph->databaseCount + graph->modelCount +
                              graph->springSetCount + s;
            }
            return false;
        }
        pairs[s] = (maniGraphBoundMapPair){map, source};
        size_t pose = PoseRound(maniPoseSize(source));
        size_t scratch = PoseRound(maniRetargetScratchSize(map));
        binding->sourcePoseSize = pose > binding->sourcePoseSize ? pose : binding->sourcePoseSize;
        binding->retargetScratchSize =
            scratch > binding->retargetScratchSize ? scratch : binding->retargetScratchSize;
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

// The mirror map the graph names, of its identity and the graph's
// skeleton; false with it unbound.
static bool BindMirror(maniGraphBinding* binding, const maniMirrorMap* map, uint32_t* unboundOut)
{
    const maniGraph* graph = &binding->graph;
    binding->mirrorMap = NULL;
    if (graph->mirrorMapIdentity == 0)
    {
        return true;
    }
    if (map != NULL && map->identity == graph->mirrorMapIdentity &&
        map->skeletonIdentity == graph->skeletonIdentity &&
        map->boneCount == graph->skeletonBoneCount)
    {
        binding->mirrorMap = map;
        return true;
    }
    if (unboundOut != NULL)
    {
        *unboundOut = graph->clipCount + graph->databaseCount + graph->modelCount +
                      graph->springSetCount + graph->mapCount;
    }
    return false;
}

static bool AssetsValid(const maniGraph* graph, const maniGraphAssets* a)
{
    return a != NULL && a->skeleton != NULL && (a->clips != NULL || a->clipCount == 0) &&
           (a->databases != NULL || a->databaseCount == 0) &&
           (a->models != NULL || a->modelCount == 0) &&
           (a->springSets != NULL || a->springSetCount == 0) &&
           (a->retargetMaps != NULL || a->retargetMapCount == 0) &&
           (a->sourceSkeletons != NULL || a->sourceSkeletonCount == 0) &&
           maniSkeletonIdentity(a->skeleton) == graph->skeletonIdentity &&
           maniSkeletonBoneCount(a->skeleton) == graph->skeletonBoneCount;
}

static MANI_NOINLINE maniResult BindGraphWork(const maniGraph* graph, const maniGraphAssets* assets,
                                              void* memory, size_t size,
                                              maniGraphBinding** bindingOut, uint32_t* unboundOut)
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
    binding->skeleton = assets->skeleton;
    if (!BindMaps(binding, assets, unboundOut))
    {
        return mani_errorInvalid;
    }
    for (uint32_t s = 0; s < graph->clipCount; ++s)
    {
        binding->clips[s] = Find(binding, s, clips, count);
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
        !BindMirror(binding, assets->mirrorMap, unboundOut) || !RootMotionBone(binding))
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
    binding->cursorSize = CursorSize(binding);
    *bindingOut = binding;
    return mani_success;
}

maniResult maniBindGraph(const maniGraph* graph, const maniGraphAssets* assets, void* memory,
                         size_t size, maniGraphBinding** bindingOut, uint32_t* unboundOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = BindGraphWork(graph, assets, memory, size, bindingOut, unboundOut);
    maniFloatEnvLeave(env);
    return result;
}
