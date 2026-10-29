// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A database slot holds an opened database of the graph's skeleton, no
// two the same database, each clip it lists the clip of a slot giving
// poses. A motion matching node names a slot, a trigger or none, finite
// costs, times not negative (the period above 0), at most
// MANI_MAX_MATCH_HISTORY picks, tags literal or from an int parameter, a
// duration as an inertialize node's, and a parameter per trajectory and
// float time of its database (vec2 and float, in channel order).

#include "graph_match_check.h"

#include "graph_match.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static maniBuildResult PastLimit(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorLimit, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

uint32_t maniGraphDatabaseClipSlot(const maniGraphSource* source,
                                   const maniMotionDatabase* database, uint32_t clip)
{
    uint64_t identity = maniMotionDatabase_GetClipIdentity(database, clip);
    for (uint32_t s = 0; s < source->clipCount; ++s)
    {
        const maniClip* c = source->clips[s].clip;
        if (c != NULL && maniClip_GetIdentity(c) == identity &&
            maniClip_GetAdditiveBasis(c) == maniAdditiveNone)
        {
            return s;
        }
    }
    return UINT32_MAX;
}

maniBuildResult maniCheckGraphDatabases(const maniGraphSource* source, uint64_t* clipsOut)
{
    uint64_t skeleton = maniSkeleton_GetIdentity(source->skeleton);
    *clipsOut = 0;
    for (uint32_t i = 0; i < source->databaseCount; ++i)
    {
        const maniMotionDatabase* d = source->databases[i].database;
        if (d == NULL || maniMotionDatabase_GetSkeletonIdentity(d) != skeleton)
        {
            return Refuse(maniBuildReasonDatabase, i, 0);
        }
        // Mirrored entries play through the graph's mirror map.
        uint64_t mirror = maniMotionDatabase_GetMirrorMapIdentity(d);
        if (mirror != 0 && (source->mirrorMap == NULL || source->mirrorMap->identity != mirror))
        {
            return Refuse(maniBuildReasonMirrorGraph, i, 3);
        }
        for (uint32_t k = 0; k < i; ++k)
        {
            if (maniMotionDatabase_GetIdentity(source->databases[k].database) ==
                maniMotionDatabase_GetIdentity(d))
            {
                return Refuse(maniBuildReasonDatabase, i, k);
            }
        }
        for (uint32_t c = 0; c < d->clipCount; ++c)
        {
            if (maniGraphDatabaseClipSlot(source, d, c) == UINT32_MAX)
            {
                return Refuse(maniBuildReasonDatabase, i, c);
            }
        }
        *clipsOut += d->clipCount;
    }
    return kOk;
}

static bool ParameterOf(const maniGraphSource* source, uint32_t parameter,
                        maniGraphParameterType type)
{
    return parameter < source->parameterCount && source->parameters[parameter].type == type;
}

// The query's parameters against the database's channels.
static bool QueryValid(const maniGraphSource* source, const maniGraphMatchNodeSource* m)
{
    // maniCheckGraphMatch refused a count past MANI_MAX_MATCH_FEATURES.
    if (m->database >= source->databaseCount || (m->queryCount > 0 && m->query == NULL))
    {
        return false;
    }
    uint32_t types[MANI_MAX_MATCH_FEATURES];
    for (uint32_t q = 0; q < m->queryCount; ++q)
    {
        uint32_t p = m->query[q];
        types[q] = p < source->parameterCount ? source->parameters[p].type : UINT32_MAX;
    }
    const maniMotionDatabase* d = source->databases[m->database].database;
    return maniGraphQueryFits(d, d->channelCount, maniGraphDatabaseChannel, types, m->queryCount);
}

static bool Seconds(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static bool TagsValid(const maniGraphSource* source, maniGraphTags tags)
{
    return tags.parameter == MANI_NO_PARAMETER ||
           (ParameterOf(source, tags.parameter, maniGraphParameterInt) && tags.value == 0);
}

maniBuildResult maniCheckGraphMatch(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node)
{
    const maniGraphMatchNodeSource* m = &n->match;
    if (m->queryCount > MANI_MAX_MATCH_FEATURES || m->historyCount > MANI_MAX_MATCH_HISTORY)
    {
        return PastLimit(maniBuildReasonNode, node, 0);
    }
    maniGraphScalar d = m->duration;
    bool ok = (m->trigger == MANI_NO_PARAMETER ||
               ParameterOf(source, m->trigger, maniGraphParameterTrigger)) &&
              isfinite(m->period) && m->period > 0.0f && Seconds(m->jumpThreshold) &&
              Seconds(m->endExclusion) && Seconds(m->historyTime) && isfinite(m->continuingBias) &&
              isfinite(m->transitionCost) && TagsValid(source, m->requiredTags) &&
              TagsValid(source, m->excludedTags) && QueryValid(source, m);
    bool duration = d.parameter == MANI_NO_PARAMETER
                        ? Seconds(d.value)
                        : ParameterOf(source, d.parameter, maniGraphParameterFloat);
    if (!ok)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    return duration ? kOk : Refuse(maniBuildReasonScalar, node, 0);
}
