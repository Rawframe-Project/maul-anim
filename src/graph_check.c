// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Checking a graph source (mani-0023): counts, names, parameters, clip
// slots, sync groups, database slots and each node on its own.

#include "graph_check.h"

#include "allocator.h"
#include "container.h"
#include "graph_body_check.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "graph_learned.h"
#include "graph_learned_check.h"
#include "graph_machine_check.h"
#include "graph_match.h"
#include "graph_match_check.h"
#include "graph_space_check.h"
#include "graph_spring.h"
#include "ik_chain.h"
#include "ik_frame.h"
#include "sort.h"
#include "text.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/ik.h"
#include "maul-anim/learned.h"
#include "maul-anim/retarget.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

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
    kSpringSets = 5,
};

static const maniBuildReason s_lists[6] = {
    maniBuildReasonParameterList, maniBuildReasonClipList,  maniBuildReasonSyncGroupList,
    maniBuildReasonDatabaseList,  maniBuildReasonModelList, maniBuildReasonSpringSetList};

static uint32_t ListCount(const maniGraphSource* source, uint32_t list)
{
    return list == kParameters   ? source->parameterCount
           : list == kClips      ? source->clipCount
           : list == kSyncGroups ? source->syncGroupCount
           : list == kDatabases  ? source->databaseCount
           : list == kModels     ? source->modelCount
                                 : source->springSetCount;
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
    if (list == kSpringSets)
    {
        *lengthOut = source->springSets[i].nameLength;
        return source->springSets[i].name;
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
    bool ok = t <= maniGraphParameterPose && p->bounded <= (number ? 1u : 0u);
    if (ok && t == maniGraphParameterPose)
    {
        // A pose parameter's value is 0.
        uint32_t words[4];
        memcpy(words, &p->defaultValue, sizeof words);
        ok = (words[0] | words[1] | words[2] | words[3]) == 0;
    }
    else if (ok && number)
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

uint32_t maniGraphMapSlotIn(uint64_t* ids, uint32_t* count, uint64_t identity)
{
    for (uint32_t k = 0; k < *count; ++k)
    {
        if (ids[k] == identity)
        {
            return k;
        }
    }
    if (*count == MANI_GRAPH_MAX_MAPS)
    {
        return MANI_GRAPH_MAX_MAPS;
    }
    ids[*count] = identity;
    return (*count)++;
}

// A clip slot's clip and map: a clip of the skeleton, or of the map's
// source with a map made for the skeleton.
static bool ClipFits(const maniGraphSource* source, const maniGraphClipSource* c)
{
    uint64_t skeleton = maniSkeletonIdentity(source->skeleton);
    const maniRetargetMap* map = c->map;
    if (c->clip == NULL)
    {
        return false;
    }
    if (map == NULL)
    {
        return maniClipSkeletonIdentity(c->clip) == skeleton;
    }
    return map->targetIdentity == skeleton &&
           map->targetBoneCount == maniSkeletonBoneCount(source->skeleton) &&
           maniClipSkeletonIdentity(c->clip) == map->sourceIdentity &&
           c->clip->skeletonBoneCount == map->sourceBoneCount;
}

// Every clip slot: a clip of the skeleton or of its map's source, each
// clip once, at most MANI_GRAPH_MAX_MAPS maps.
static maniBuildResult CheckClips(const maniGraphSource* source, const maniAllocator* allocator,
                                  uint32_t* mapCountOut)
{
    uint32_t n = source->clipCount;
    uint64_t ids[MANI_GRAPH_MAX_MAPS] = {0};
    for (uint32_t i = 0; i < n; ++i)
    {
        const maniGraphClipSource* c = &source->clips[i];
        if (!ClipFits(source, c) ||
            (c->map != NULL &&
             maniGraphMapSlotIn(ids, mapCountOut, c->map->identity) == MANI_GRAPH_MAX_MAPS))
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

// Every spring set slot: a set of the skeleton, each set once.
static maniBuildResult CheckSpringSets(const maniGraphSource* source)
{
    uint64_t skeleton = maniSkeletonIdentity(source->skeleton);
    for (uint32_t i = 0; i < source->springSetCount; ++i)
    {
        const maniSpringSet* p = source->springSets[i].set;
        if (p == NULL || maniSpringSetSkeletonIdentity(p) != skeleton)
        {
            return maniGraphRefuse(maniBuildReasonSpringSetSlot, i, 0);
        }
        for (uint32_t k = 0; k < i; ++k)
        {
            if (maniSpringSetIdentity(source->springSets[k].set) == maniSpringSetIdentity(p))
            {
                return maniGraphRefuse(maniBuildReasonSpringSetSlot, i, k);
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

// A spring node: a spring set slot whose state is not past
// MANI_GRAPH_SPRING_MAX_WORDS, a vec3 position, a quat rotation and a
// reset trigger, each or none.
static maniBuildResult CheckSpring(const maniGraphSource* source, const maniGraphNodeSource* n,
                                   uint32_t node)
{
    const maniGraphSpringNodeSource* s = &n->spring;
    const uint32_t parameters[4] = {s->position, s->rotation, s->reset, s->force};
    const maniGraphParameterType types[4] = {maniGraphParameterVec3, maniGraphParameterQuat,
                                             maniGraphParameterTrigger, maniGraphParameterVec3};
    bool ok = s->springSet < source->springSetCount &&
              maniGraphSpringSetWords(source->springSets[s->springSet].set) <=
                  MANI_GRAPH_SPRING_MAX_WORDS;
    for (uint32_t k = 0; k < 4 && ok; ++k)
    {
        ok = parameters[k] == MANI_NO_PARAMETER ||
             ParameterOf(source, parameters[k], types[k], types[k]);
    }
    return ok ? Outcome(mani_success) : maniGraphRefuse(maniBuildReasonNode, node, 0);
}

// Bones of the skeleton, each an ancestor of the next.
static bool ChainValid(const maniSkeleton* skeleton, const uint32_t* bones, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (bones[k] >= maniSkeletonBoneCount(skeleton) ||
            (k > 0 && !maniIkAbove(skeleton, bones[k - 1], bones[k])))
        {
            return false;
        }
    }
    return true;
}

static bool FiniteVector(maniVec3 v)
{
    return Finite(v.x) && Finite(v.y) && Finite(v.z);
}

// A goal a vec3 parameter, a pole one or none.
static bool GoalValid(const maniGraphSource* source, uint32_t goal, uint32_t pole)
{
    maniGraphParameterType vec3 = maniGraphParameterVec3;
    return ParameterOf(source, goal, vec3, vec3) &&
           (pole == MANI_NO_PARAMETER || ParameterOf(source, pole, vec3, vec3));
}

// The twist, then a number from 0 to high, then the weight.
static maniBuildResult CheckIkScalars(const maniGraphSource* source, const maniGraphScalar* s,
                                      float high, uint32_t node)
{
    maniBuildResult r = CheckScalar(source, s[0], node, 0, -HUGE_VALF, HUGE_VALF);
    r = r.result == mani_success ? CheckScalar(source, s[1], node, 1, 0.0f, high) : r;
    return r.result == mani_success ? CheckScalar(source, s[2], node, 2, 0.0f, 1.0f) : r;
}

// A two-bone node: its chain, goal and pole, a finite hinge, a finite
// twist, soften and weight in [0, 1].
static maniBuildResult CheckTwoBone(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node)
{
    const maniGraphTwoBoneNodeSource* s = &n->twoBone;
    const uint32_t bones[3] = {s->start, s->middle, s->end};
    const maniGraphScalar scalars[3] = {s->twist, s->soften, s->weight};
    bool ok = ChainValid(source->skeleton, bones, 3) && GoalValid(source, s->goal, s->pole) &&
              FiniteVector(s->hinge);
    return ok ? CheckIkScalars(source, scalars, 1.0f, node)
              : maniGraphRefuse(maniBuildReasonNode, node, 0);
}

// Shares at least 0, finite, not all 0 and of a finite sum; NULL for
// equal ones.
static bool SharesValid(const float* shares, uint32_t count)
{
    if (shares == NULL)
    {
        return true;
    }
    float total = 0.0f;
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!(shares[k] >= 0.0f) || !Finite(shares[k]))
        {
            return false;
        }
        total += shares[k];
    }
    return total > 0.0f && Finite(total);
}

// An aim node: 1 to MANI_AIM_MAX_BONES bones of a chain, its shares,
// goal and pole, a forward of length 1 within a thousandth, offset and
// up finite, a finite twist, a cone in [0, MANI_AIM_MAX_CONE], a weight
// in [0, 1].
static maniBuildResult CheckAim(const maniGraphSource* source, const maniGraphNodeSource* n,
                                uint32_t node)
{
    const maniGraphAimNodeSource* s = &n->aim;
    const maniGraphScalar scalars[3] = {s->twist, s->cone, s->weight};
    bool ok = s->bones != NULL && s->count > 0 && s->count <= MANI_AIM_MAX_BONES &&
              ChainValid(source->skeleton, s->bones, s->count) &&
              SharesValid(s->shares, s->count) && GoalValid(source, s->goal, s->pole) &&
              fabsf(maniVec3Length(s->forward) - 1.0f) <= 1e-3f && FiniteVector(s->offset) &&
              FiniteVector(s->up);
    return ok ? CheckIkScalars(source, scalars, MANI_AIM_MAX_CONE, node)
              : maniGraphRefuse(maniBuildReasonNode, node, 0);
}

// The words of a chain node's limits: one limit per turning bone, or
// none.
static uint32_t ChainLimitWords(const maniGraphSource* source, const maniGraphNodeSource* n)
{
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t count = n->chain.limits != NULL
                         ? maniIkChainPath(source->skeleton, n->chain.root, n->chain.end, bones)
                         : 0u;
    return count > 0 ? MANI_GRAPH_LIMIT_WORDS * (count - 1) : 0u;
}

// The data words of a chain or full-body node, by its counts; 0 for
// other kinds.
static uint32_t IkWords(const maniGraphSource* source, const maniGraphNodeSource* n)
{
    if (n->kind == maniGraphNodeCcd || n->kind == maniGraphNodeFabrik)
    {
        return MANI_GRAPH_CHAIN_WORDS + ChainLimitWords(source, n);
    }
    bool body = n->kind == maniGraphNodeBodyXpbd || n->kind == maniGraphNodeBodySdls;
    return body ? maniGraphBodySourceWords(n) : 0u;
}

// A chain node: a chain of the skeleton, its limits one per turning bone
// (or none) and valid, a vec3 goal, a FABRIK node's pole a vec3 or none
// (a CCD node's none), a quat rotation or none, its iterations,
// tolerance and step (0 for FABRIK) in range, its weights in [0, 1].
static maniBuildResult CheckChain(const maniGraphSource* source, const maniGraphNodeSource* n,
                                  uint32_t node)
{
    const maniGraphChainNodeSource* s = &n->chain;
    bool fabrik = n->kind == maniGraphNodeFabrik;
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    uint32_t boneCount = maniSkeletonBoneCount(source->skeleton);
    uint32_t count = s->root < boneCount && s->end < boneCount
                         ? maniIkChainPath(source->skeleton, s->root, s->end, bones)
                         : 0u;
    maniGraphParameterType quat = maniGraphParameterQuat;
    bool ok =
        count > 0 &&
        (s->limits == NULL || maniIkChainLimitsValid(source->skeleton, bones, count, s->limits)) &&
        GoalValid(source, s->goal, fabrik ? s->pole : MANI_NO_PARAMETER) &&
        (fabrik || s->pole == MANI_NO_PARAMETER) &&
        (s->rotation == MANI_NO_PARAMETER || ParameterOf(source, s->rotation, quat, quat)) &&
        s->iterations >= 1 && s->iterations <= MANI_IK_MAX_ITERATIONS && s->tolerance >= 0.0f &&
        Finite(s->tolerance) && s->step >= 0.0f && s->step <= MANI_AIM_MAX_CONE &&
        (!fabrik || s->step == 0.0f);
    maniBuildResult r = ok ? CheckScalar(source, s->rotationWeight, node, 0, 0.0f, 1.0f)
                           : maniGraphRefuse(maniBuildReasonNode, node, 0);
    return r.result == mani_success ? CheckScalar(source, s->weight, node, 1, 0.0f, 1.0f) : r;
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
    case maniGraphNodeLearned:
        return maniCheckGraphLearned(source, n, node);
    case maniGraphNodeSpring:
        return CheckSpring(source, n, node);
    case maniGraphNodeTwoBone:
        return CheckTwoBone(source, n, node);
    case maniGraphNodeAim:
        return CheckAim(source, n, node);
    case maniGraphNodeCcd:
    case maniGraphNodeFabrik:
        return CheckChain(source, n, node);
    case maniGraphNodeBodyXpbd:
    case maniGraphNodeBodySdls:
        return maniCheckGraphBody(source, n, node);
    case maniGraphNodeAdditive:
        return CheckWeights(source, n->additive.weights, count - 1, node, 1.0f);
    case maniGraphNodeSelect:
        return ParameterOf(source, n->select.parameter, maniGraphParameterInt,
                           maniGraphParameterBool)
                   ? Outcome(mani_success)
                   : maniGraphRefuse(maniBuildReasonNode, node, 0);
    case maniGraphNodeTimeScale:
        return CheckScalar(source, n->timeScale.scale, node, 0, -HUGE_VALF, HUGE_VALF);
    case maniGraphNodePose:
        return ParameterOf(source, n->pose.parameter, maniGraphParameterPose,
                           maniGraphParameterPose)
                   ? Outcome(mani_success)
                   : maniGraphRefuse(maniBuildReasonNode, node, 0);
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
    case maniGraphNodeLearned:
    {
        const maniLearnedModel* m = source->models[n->learned.model].model;
        return MANI_GRAPH_LEARNED_BLOCK_WORDS(m->featureCount + m->latentCount, m->eventCount);
    }
    case maniGraphNodeSpring:
        return maniGraphSpringState +
               (uint32_t)maniGraphSpringSetWords(source->springSets[n->spring.springSet].set);
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
    bool learned = n->kind == maniGraphNodeLearned;
    tally->dataWords +=
        machine ? maniGraphMachineSourceWords(n)
                : (shot ? MANI_GRAPH_SHOT_WORDS : maniGraphDataWords(n->kind, n->inputCount));
    tally->dataWords += match ? MANI_GRAPH_MATCH_WORDS + n->match.queryCount : 0u;
    tally->dataWords += learned ? MANI_GRAPH_LEARNED_WORDS + n->learned.queryCount : 0u;
    tally->dataWords += IkWords(source, n);
    tally->machineWords += BlockWords(source, n);
    tally->listeners += (machine && maniGraphMachineListens(n)) || shot ? 1u : 0u;
    tally->inertial += maniGraphFirstInertial(n) != UINT32_MAX || match || learned ? 1u : 0u;
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
           (source->modelCount == 0 || source->models != NULL) &&
           source->springSetCount <= MANI_GRAPH_MAX_SPRING_SETS &&
           (source->springSetCount == 0 || source->springSets != NULL);
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
    for (uint32_t list = kParameters; list <= kSpringSets && r.result == mani_success; ++list)
    {
        r = CheckNames(source, list, allocator, &tallyOut->namesSize);
    }
    for (uint32_t i = 0; i < source->parameterCount && r.result == mani_success; ++i)
    {
        r = CheckParameter(&source->parameters[i], i);
    }
    r = r.result == mani_success ? CheckClips(source, allocator, &tallyOut->mapCount) : r;
    r = r.result == mani_success ? CheckSyncGroups(source) : r;
    r = r.result == mani_success ? maniCheckGraphDatabases(source, &tallyOut->databaseClips) : r;
    r = r.result == mani_success ? CheckModels(source) : r;
    r = r.result == mani_success ? CheckSpringSets(source) : r;
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
