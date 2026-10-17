// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Checking a graph source (mani-0023): counts, names, parameters, clip
// slots, sync groups, database slots and each node on its own.

#include "graph_check.h"

#include "allocator.h"
#include "container.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_machine_check.h"
#include "graph_match.h"
#include "graph_match_check.h"
#include "graph_space_check.h"
#include "sort.h"
#include "text.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/learned.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Outcome(maniResult result)
{
    return (maniBuildResult){result, maniBuildReasonNone, 0, 0};
}

maniBuildResult maniGraphRefuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static bool Finite(float value)
{
    return isfinite(value);
}

// The lists whose items are named.
enum
{
    kParameters = 0,
    kClips = 1,
    kSyncGroups = 2,
    kDatabases = 3,
    kModels = 4,
};

static const maniBuildReason s_lists[5] = {maniBuildReasonParameterList, maniBuildReasonClipList,
                                           maniBuildReasonSyncGroupList,
                                           maniBuildReasonDatabaseList, maniBuildReasonModelList};

static uint32_t ListCount(const maniGraphSource* source, uint32_t list)
{
    return list == kParameters   ? source->parameterCount
           : list == kClips      ? source->clipCount
           : list == kSyncGroups ? source->syncGroupCount
           : list == kDatabases  ? source->databaseCount
                                 : source->modelCount;
}

static const char* ItemName(const maniGraphSource* source, uint32_t list, uint32_t i,
                            uint32_t* lengthOut)
{
    if (list == kParameters)
    {
        *lengthOut = source->parameters[i].nameLength;
        return source->parameters[i].name;
    }
    if (list == kClips)
    {
        *lengthOut = source->clips[i].nameLength;
        return source->clips[i].name;
    }
    if (list == kDatabases)
    {
        *lengthOut = source->databases[i].nameLength;
        return source->databases[i].name;
    }
    if (list == kModels)
    {
        *lengthOut = source->models[i].nameLength;
        return source->models[i].name;
    }
    *lengthOut = source->syncGroups[i].nameLength;
    return source->syncGroups[i].name;
}

// The first two equal keys in key order, lower index first: 1 when found,
// 0 when none, -1 when the allocator fails.
static int FindRepeat(const uint64_t* keys, uint32_t n, const maniAllocator* allocator,
                      uint32_t* lowOut, uint32_t* highOut)
{
    uint32_t* sorted = n > 1 ? maniAllocate(allocator, 2 * (size_t)n, sizeof(uint32_t), 4) : NULL;
    if (n > 1 && sorted == NULL)
    {
        return -1;
    }
    int found = 0;
    if (n > 1)
    {
        maniSortByKey(keys, n, sorted, sorted + n);
        for (uint32_t k = 1; k < n && found == 0; ++k)
        {
            uint32_t a = sorted[k - 1];
            uint32_t b = sorted[k];
            found = keys[a] == keys[b] ? 1 : 0;
            *lowOut = a < b ? a : b;
            *highOut = a < b ? b : a;
        }
        maniRelease(allocator, sorted, 2 * (size_t)n, sizeof(uint32_t), 4);
    }
    return found;
}

// A list's names: each UTF-8 and not empty, no two equal or of one hash.
static maniBuildResult CheckNames(const maniGraphSource* source, uint32_t list,
                                  const maniAllocator* allocator, uint64_t* sizeOut)
{
    uint32_t n = ListCount(source, list);
    uint64_t* keys = n > 0 ? maniAllocate(allocator, n, sizeof(uint64_t), 8) : NULL;
    if (n > 0 && keys == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    maniBuildResult r = Outcome(mani_success);
    for (uint32_t i = 0; i < n && r.result == mani_success; ++i)
    {
        uint32_t length = 0;
        const char* name = ItemName(source, list, i, &length);
        if (name == NULL || length == 0 || !maniValidUtf8((const uint8_t*)name, length))
        {
            r = maniGraphRefuse(s_lists[list] | maniBuildReasonName, i, 0);
            break;
        }
        keys[i] = maniHashBytes(MANI_HASH_INIT, (const uint8_t*)name, length);
        *sizeOut += length;
    }
    uint32_t low = 0;
    uint32_t high = 0;
    int found = r.result == mani_success ? FindRepeat(keys, n, allocator, &low, &high) : 0;
    if (found < 0)
    {
        r = Outcome(mani_errorCapacity);
    }
    else if (found > 0)
    {
        uint32_t lengthA = 0;
        uint32_t lengthB = 0;
        const char* a = ItemName(source, list, low, &lengthA);
        const char* b = ItemName(source, list, high, &lengthB);
        bool same = lengthA == lengthB && memcmp(a, b, lengthA) == 0;
        r = maniGraphRefuse(s_lists[list] |
                                (same ? maniBuildReasonDuplicateName : maniBuildReasonNameHash),
                            high, low);
    }
    maniRelease(allocator, keys, n, sizeof(uint64_t), 8);
    return r;
}

// A float value and bounds in form; an int's bounds.
static bool NumberInForm(const maniGraphParameterSource* p)
{
    if (p->type == maniGraphParameterInt)
    {
        return p->bounded == 0 ||
               (p->minimum.i <= p->maximum.i && p->defaultValue.i >= p->minimum.i &&
                p->defaultValue.i <= p->maximum.i);
    }
    float v = p->defaultValue.f[0];
    return Finite(v) && (p->bounded == 0 || (Finite(p->minimum.f[0]) && Finite(p->maximum.f[0]) &&
                                             p->minimum.f[0] <= p->maximum.f[0] &&
                                             v >= p->minimum.f[0] && v <= p->maximum.f[0]));
}

static maniBuildResult CheckParameter(const maniGraphParameterSource* p, uint32_t i)
{
    maniGraphParameterType t = p->type;
    bool number = t == maniGraphParameterInt || t == maniGraphParameterFloat;
    bool ok = t <= maniGraphParameterTrigger && p->bounded <= (number ? 1u : 0u);
    if (ok && number)
    {
        ok = NumberInForm(p);
    }
    else if (ok && (t == maniGraphParameterBool || t == maniGraphParameterTrigger))
    {
        ok = p->defaultValue.b <= 1u;
    }
    else if (ok)
    {
        uint32_t n = t == maniGraphParameterVec2 ? 2u : (t == maniGraphParameterVec3 ? 3u : 4u);
        double length = 0.0;
        for (uint32_t k = 0; k < n; ++k)
        {
            ok = ok && Finite(p->defaultValue.f[k]);
            length += (double)p->defaultValue.f[k] * (double)p->defaultValue.f[k];
        }
        ok = ok && (t != maniGraphParameterQuat || (length > 0.0 && isfinite(length)));
    }
    return ok ? Outcome(mani_success) : maniGraphRefuse(maniBuildReasonParameter, i, 0);
}

// Every clip slot: a clip of the skeleton, each clip once.
static maniBuildResult CheckClips(const maniGraphSource* source, const maniAllocator* allocator)
{
    uint32_t n = source->clipCount;
    uint64_t skeleton = maniSkeletonIdentity(source->skeleton);
    for (uint32_t i = 0; i < n; ++i)
    {
        const maniClip* clip = source->clips[i].clip;
        if (clip == NULL || maniClipSkeletonIdentity(clip) != skeleton)
        {
            return maniGraphRefuse(maniBuildReasonClip, i, 0);
        }
    }
    uint64_t* keys = n > 1 ? maniAllocate(allocator, n, sizeof(uint64_t), 8) : NULL;
    if (n > 1 && keys == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    for (uint32_t i = 0; i < n && keys != NULL; ++i)
    {
        keys[i] = maniClipIdentity(source->clips[i].clip);
    }
    uint32_t low = 0;
    uint32_t high = 0;
    int found = FindRepeat(keys, n > 1 ? n : 0, allocator, &low, &high);
    maniRelease(allocator, keys, n > 1 ? n : 0, sizeof(uint64_t), 8);
    return found < 0   ? Outcome(mani_errorCapacity)
           : found > 0 ? maniGraphRefuse(maniBuildReasonClip, high, low)
                       : Outcome(mani_success);
}

// Every model slot: a model of the skeleton, each model once.
static maniBuildResult CheckModels(const maniGraphSource* source)
{
    uint64_t skeleton = maniSkeletonIdentity(source->skeleton);
    for (uint32_t i = 0; i < source->modelCount; ++i)
    {
        const maniLearnedModel* m = source->models[i].model;
        if (m == NULL || maniLearnedModelSkeletonIdentity(m) != skeleton)
        {
            return maniGraphRefuse(maniBuildReasonModel, i, 0);
        }
        for (uint32_t k = 0; k < i; ++k)
        {
            if (maniLearnedModelIdentity(source->models[k].model) == maniLearnedModelIdentity(m))
            {
                return maniGraphRefuse(maniBuildReasonModel, i, k);
            }
        }
    }
    return Outcome(mani_success);
}

static maniBuildResult CheckSyncGroups(const maniGraphSource* source)
{
    for (uint32_t i = 0; i < source->syncGroupCount; ++i)
    {
        if ((source->syncGroups[i].flags & ~(uint32_t)maniGraphSyncAllEvents) != 0)
        {
            return maniGraphRefuse(maniBuildReasonSyncGroupList | maniBuildReasonKind, i, 0);
        }
    }
    return Outcome(mani_success);
}

// A number: a float parameter, or a finite literal from low to high.
static maniBuildResult CheckScalar(const maniGraphSource* source, maniGraphScalar s, uint32_t node,
                                   uint32_t place, float low, float high)
{
    bool ok = s.parameter == MANI_NO_PARAMETER
                  ? Finite(s.value) && s.value >= low && s.value <= high
                  : s.parameter < source->parameterCount &&
                        source->parameters[s.parameter].type == maniGraphParameterFloat;
    return ok ? Outcome(mani_success) : maniGraphRefuse(maniBuildReasonScalar, node, place);
}

static maniBuildResult CheckWeights(const maniGraphSource* source, const maniGraphScalar* weights,
                                    uint32_t count, uint32_t node, float high)
{
    if (weights == NULL)
    {
        return maniGraphRefuse(maniBuildReasonNode, node, 0);
    }
    maniBuildResult r = Outcome(mani_success);
    for (uint32_t k = 0; k < count && r.result == mani_success; ++k)
    {
        r = CheckScalar(source, weights[k], node, k, 0.0f, high);
    }
    return r;
}

static bool ParameterOf(const maniGraphSource* source, uint32_t parameter, maniGraphParameterType a,
                        maniGraphParameterType b)
{
    return parameter < source->parameterCount &&
           (source->parameters[parameter].type == a || source->parameters[parameter].type == b);
}

static maniBuildResult CheckClipNode(const maniGraphSource* source, const maniGraphNodeSource* n,
                                     uint32_t node)
{
    const maniGraphClipNodeSource* c = &n->clip;
    bool grouped = c->syncGroup < source->syncGroupCount;
    bool ok = c->clip < source->clipCount && (grouped || c->syncGroup == MANI_NO_SYNC_GROUP) &&
              c->loop <= maniGraphLoopRepeat && (c->flags & ~(uint32_t)maniGraphClipLeader) == 0 &&
              (grouped || c->flags == 0);
    return ok ? CheckScalar(source, c->speed, node, 0, -HUGE_VALF, HUGE_VALF)
              : maniGraphRefuse(maniBuildReasonNode, node, 0);
}

static maniBuildResult CheckMask(const maniGraphSource* source, const maniGraphMaskNodeSource* m,
                                 uint32_t node)
{
    if (m->chainCount > 0 && m->chains == NULL)
    {
        return maniGraphRefuse(maniBuildReasonNode, node, 0);
    }
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    for (uint32_t k = 0; k < m->chainCount; ++k)
    {
        float w = m->chains[k].weight;
        if (m->chains[k].bone >= bones)
        {
            return maniGraphRefuse(maniBuildReasonNode, node, 0);
        }
        if (!(w >= 0.0f && w <= 1.0f))
        {
            return maniGraphRefuse(maniBuildReasonWeight, node, k);
        }
    }
    bool ok = m->defaultWeight >= 0.0f && m->defaultWeight <= 1.0f;
    return ok ? Outcome(mani_success) : maniGraphRefuse(maniBuildReasonWeight, node, m->chainCount);
}

static bool Seconds(float value)
{
    return isfinite(value) && value >= 0.0f;
}

// A one-shot: a fire trigger, optional fade-out and abort triggers, a
// curve, fades in seconds, an optional mask.
static maniBuildResult CheckShotNode(const maniGraphSource* source, const maniGraphNodeSource* n,
                                     uint32_t node)
{
    const maniGraphOneShotNodeSource* s = &n->oneShot;
    maniGraphParameterType trigger = maniGraphParameterTrigger;
    bool ok = ParameterOf(source, s->fire, trigger, trigger) &&
              (s->fadeOutTrigger == MANI_NO_PARAMETER ||
               ParameterOf(source, s->fadeOutTrigger, trigger, trigger)) &&
              (s->abort == MANI_NO_PARAMETER || ParameterOf(source, s->abort, trigger, trigger)) &&
              s->curve <= maniGraphFadeSmoothstep && Seconds(s->fadeIn) && Seconds(s->fadeOut);
    if (!ok)
    {
        return maniGraphRefuse(maniBuildReasonNode, node, 0);
    }
    return s->mask != NULL ? CheckMask(source, s->mask, node) : Outcome(mani_success);
}

// An inertialize node: an optional trigger, a known method, half-lives
// as it wants them, a duration in seconds.
static maniBuildResult CheckInertialize(const maniGraphSource* source, const maniGraphNodeSource* n,
                                        uint32_t node)
{
    const maniGraphInertializeNodeSource* s = &n->inertialize;
    maniGraphParameterType trigger = maniGraphParameterTrigger;
    maniGraphInertia lives = {0, s->method, s->halfLifeScale, s->halfLifeMin, s->halfLifeMax};
    bool ok =
        (s->trigger == MANI_NO_PARAMETER || ParameterOf(source, s->trigger, trigger, trigger)) &&
        s->method <= maniGraphDeadBlending && maniGraphHalfLivesValid(&lives) == UINT32_MAX;
    return ok ? CheckScalar(source, s->duration, node, 0, 0.0f, HUGE_VALF)
              : maniGraphRefuse(maniBuildReasonNode, node, 0);
}

// A node's own fields, by kind.
static maniBuildResult CheckFields(const maniGraphSource* source, const maniGraphNodeSource* n,
                                   uint32_t node)
{
    uint32_t count = n->inputCount;
    switch (n->kind)
    {
    case maniGraphNodeClip:
        return CheckClipNode(source, n, node);
    case maniGraphNodeBlend:
        return n->blend.direct > 1u ? maniGraphRefuse(maniBuildReasonNode, node, 0)
                                    : CheckWeights(source, n->blend.weights, count, node,
                                                   n->blend.direct != 0 ? 1.0f : HUGE_VALF);
    case maniGraphNodeMask:
        return CheckMask(source, &n->mask, node);
    case maniGraphNodeOneShot:
        return CheckShotNode(source, n, node);
    case maniGraphNodeInertialize:
        return CheckInertialize(source, n, node);
    case maniGraphNodeMatch:
        return maniCheckGraphMatch(source, n, node);
    case maniGraphNodeAdditive:
        return CheckWeights(source, n->additive.weights, count - 1, node, 1.0f);
    case maniGraphNodeSelect:
        return ParameterOf(source, n->select.parameter, maniGraphParameterInt,
                           maniGraphParameterBool)
                   ? Outcome(mani_success)
                   : maniGraphRefuse(maniBuildReasonNode, node, 0);
    case maniGraphNodeTimeScale:
        return CheckScalar(source, n->timeScale.scale, node, 0, -HUGE_VALF, HUGE_VALF);
    case maniGraphNodeBlendSpace1D:
    case maniGraphNodeBlendSpaceTriangles:
    case maniGraphNodeBlendSpaceBands:
        return maniCheckGraphSpace(source, n, node);
    case maniGraphNodeMachine:
        return maniCheckGraphMachine(source, n, node);
    default:
        return ParameterOf(source, n->seek.trigger, maniGraphParameterTrigger,
                           maniGraphParameterTrigger)
                   ? CheckScalar(source, n->seek.time, node, 0, -HUGE_VALF, HUGE_VALF)
                   : maniGraphRefuse(maniBuildReasonNode, node, 0);
    }
}

// The words of the block a node keeps in an instance.
static uint32_t BlockWords(const maniGraphSource* source, const maniGraphNodeSource* n)
{
    switch (n->kind)
    {
    case maniGraphNodeMachine:
        return MANI_GRAPH_BLOCK_SIZE(n->inputCount, n->machine.transitionCount,
                                     maniGraphMachineConditions(n));
    case maniGraphNodeOneShot:
        return MANI_GRAPH_SHOT_BLOCK_WORDS;
    case maniGraphNodeInertialize:
        return MANI_GRAPH_INERTIA_BLOCK(maniSkeletonBoneCount(source->skeleton),
                                        n->inertialize.method);
    case maniGraphNodeMatch:
        return MANI_GRAPH_MATCH_BLOCK_WORDS;
    default:
        return 0;
    }
}

static maniBuildResult CheckNode(const maniGraphSource* source, uint32_t node,
                                 maniGraphTally* tally)
{
    const maniGraphNodeSource* n = &source->nodes[node];
    uint32_t low = 0;
    uint32_t high = maniGraphInputRange(n->kind, &low);
    if (n->inputCount < low || n->inputCount > high || (n->inputCount > 0 && n->inputs == NULL))
    {
        return maniGraphRefuse(maniBuildReasonNode, node, 0);
    }
    for (uint32_t k = 0; k < n->inputCount; ++k)
    {
        if (n->inputs[k] >= source->nodeCount || n->inputs[k] == node)
        {
            return maniGraphRefuse(maniBuildReasonInput, node, k);
        }
    }
    maniBuildResult r = CheckFields(source, n, node);
    if (r.result != mani_success)
    {
        return r;
    }
    bool machine = n->kind == maniGraphNodeMachine;
    bool shot = n->kind == maniGraphNodeOneShot;
    uint32_t transitions = machine ? n->machine.transitionCount : 0u;
    tally->inputCount += n->inputCount;
    bool match = n->kind == maniGraphNodeMatch;
    tally->dataWords +=
        machine ? maniGraphMachineSourceWords(n)
                : (shot ? MANI_GRAPH_SHOT_WORDS : maniGraphDataWords(n->kind, n->inputCount));
    tally->dataWords += match ? MANI_GRAPH_MATCH_WORDS + n->match.queryCount : 0u;
    tally->machineWords += BlockWords(source, n);
    tally->listeners += (machine && maniGraphMachineListens(n)) || shot ? 1u : 0u;
    tally->inertial += maniGraphFirstInertial(n) != UINT32_MAX || match ? 1u : 0u;
    tally->maskCount += shot && n->oneShot.mask != NULL ? 1u : 0u;
    tally->maxTransitions =
        transitions > tally->maxTransitions ? transitions : tally->maxTransitions;
    tally->maskCount += n->kind == maniGraphNodeMask ? 1u : 0u;
    return r;
}

// The clip lists of the machines that listen and of the one-shots: their
// words, and the longest list.
static maniBuildResult TallyLists(const maniGraphSource* source, const maniAllocator* allocator,
                                  maniGraphTally* tally)
{
    size_t n = source->nodeCount;
    uint32_t* work = maniAllocate(allocator, 2 * n, sizeof(uint32_t), 4);
    if (work == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    memset(work, 0, n * sizeof(uint32_t));
    maniGraphReach reach = {source, NULL, work, work + n, 0, 0};
    for (uint32_t v = 0; v < n; ++v)
    {
        const maniGraphNodeSource* node = &source->nodes[v];
        bool listens = node->kind == maniGraphNodeMachine && maniGraphMachineListens(node);
        bool shot = node->kind == maniGraphNodeOneShot;
        // A machine lists per state; a one-shot beneath its shot.
        for (uint32_t k = shot ? 1u : 0u; k < (listens || shot ? node->inputCount : 0u); ++k)
        {
            uint32_t count = maniGraphReachClips(&reach, node->inputs[k], NULL, NULL);
            tally->dataWords += count;
            tally->longestList = count > tally->longestList ? count : tally->longestList;
        }
    }
    maniRelease(allocator, work, 2 * n, sizeof(uint32_t), 4);
    return Outcome(mani_success);
}

// Every machine with an inertial transition below an inertialize node.
static maniBuildResult CheckInertial(const maniGraphSource* source, const maniAllocator* allocator)
{
    size_t n = source->nodeCount;
    uint32_t* work = maniAllocate(allocator, 2 * n, sizeof(uint32_t), 4);
    if (work == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    memset(work, 0, n * sizeof(uint32_t));
    maniBuildResult r = maniCheckGraphInertial(source, work, work + n);
    maniRelease(allocator, work, 2 * n, sizeof(uint32_t), 4);
    return r;
}

static bool CountsValid(const maniGraphSource* source)
{
    return source->nodeCount > 0 && source->nodeCount <= MANI_GRAPH_MAX_NODES &&
           source->nodes != NULL && source->parameterCount <= MANI_GRAPH_MAX_PARAMETERS &&
           (source->parameterCount == 0 || source->parameters != NULL) &&
           source->clipCount <= MANI_GRAPH_MAX_CLIPS &&
           (source->clipCount == 0 || source->clips != NULL) &&
           source->syncGroupCount <= MANI_GRAPH_MAX_SYNC_GROUPS &&
           (source->syncGroupCount == 0 || source->syncGroups != NULL) &&
           source->databaseCount <= MANI_GRAPH_MAX_DATABASES &&
           (source->databaseCount == 0 || source->databases != NULL) &&
           source->modelCount <= MANI_GRAPH_MAX_MODELS &&
           (source->modelCount == 0 || source->models != NULL);
}

maniBuildResult maniCheckGraphSource(const maniGraphSource* source, const maniAllocator* allocator,
                                     maniGraphTally* tallyOut)
{
    *tallyOut = (maniGraphTally){0};
    if (!CountsValid(source))
    {
        return maniGraphRefuse(maniBuildReasonCount, 0, 0);
    }
    maniBuildResult r = Outcome(mani_success);
    for (uint32_t list = kParameters; list <= kModels && r.result == mani_success; ++list)
    {
        r = CheckNames(source, list, allocator, &tallyOut->namesSize);
    }
    for (uint32_t i = 0; i < source->parameterCount && r.result == mani_success; ++i)
    {
        r = CheckParameter(&source->parameters[i], i);
    }
    r = r.result == mani_success ? CheckClips(source, allocator) : r;
    r = r.result == mani_success ? CheckSyncGroups(source) : r;
    r = r.result == mani_success ? maniCheckGraphDatabases(source, &tallyOut->databaseClips) : r;
    r = r.result == mani_success ? CheckModels(source) : r;
    for (uint32_t i = 0; i < source->nodeCount && r.result == mani_success; ++i)
    {
        r = CheckNode(source, i, tallyOut);
    }
    if (r.result == mani_success && source->output >= source->nodeCount)
    {
        r = maniGraphRefuse(maniBuildReasonInput, source->output, 0);
    }
    if (r.result == mani_success && tallyOut->listeners > 0)
    {
        r = TallyLists(source, allocator, tallyOut);
    }
    if (r.result == mani_success && tallyOut->inertial > 0)
    {
        r = CheckInertial(source, allocator);
    }
    if (r.result == mani_success &&
        (tallyOut->dataWords > UINT32_MAX || tallyOut->machineWords > UINT32_MAX))
    {
        r = maniGraphRefuse(maniBuildReasonCount, 0, 0);
    }
    return r;
}
