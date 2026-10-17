// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned motion matching node names a model slot, a trigger or none, a
// period above 0, a transition cost not negative (finite), tags literal
// or from an int parameter (literal tags those of one of its model's
// Projectors), a duration as an inertialize node's, and a parameter per
// trajectory and float time of its model's database (vec2 and float, in
// channel order).

#include "graph_learned_check.h"

#include "graph_match.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static bool ParameterOf(const maniGraphSource* source, uint32_t parameter,
                        maniGraphParameterType type)
{
    return parameter < source->parameterCount && source->parameters[parameter].type == type;
}

static bool TagsValid(const maniGraphSource* source, maniGraphTags tags)
{
    return tags.parameter == MANI_NO_PARAMETER ||
           (ParameterOf(source, tags.parameter, maniGraphParameterInt) && tags.value == 0);
}

// The query's parameters against the model's database's channels.
static bool QueryValid(const maniGraphSource* source, const maniGraphLearnedNodeSource* l)
{
    if (l->queryCount > MANI_MATCH_MAX_FEATURES || (l->queryCount > 0 && l->query == NULL))
    {
        return false;
    }
    uint32_t types[MANI_MATCH_MAX_FEATURES];
    for (uint32_t q = 0; q < l->queryCount; ++q)
    {
        uint32_t p = l->query[q];
        types[q] = p < source->parameterCount ? source->parameters[p].type : UINT32_MAX;
    }
    const maniLearnedModel* m = source->models[l->model].model;
    return maniGraphQueryFits(m, m->channelCount, maniGraphModelChannel, types, l->queryCount);
}

// Literal tags (both) those of one of the model's Projectors.
static bool TagsKnown(const maniGraphSource* source, const maniGraphLearnedNodeSource* l)
{
    if (l->requiredTags.parameter != MANI_NO_PARAMETER ||
        l->excludedTags.parameter != MANI_NO_PARAMETER)
    {
        return true;
    }
    const maniLearnedModel* m = source->models[l->model].model;
    for (uint32_t p = 0; p < m->projectorCount; ++p)
    {
        uint32_t required = 0;
        uint32_t excluded = 0;
        (void)maniLearnedModelProjector(m, p, &required, &excluded);
        if (required == l->requiredTags.value && excluded == l->excludedTags.value)
        {
            return true;
        }
    }
    return false;
}

maniBuildResult maniCheckGraphLearned(const maniGraphSource* source, const maniGraphNodeSource* n,
                                      uint32_t node)
{
    const maniGraphLearnedNodeSource* l = &n->learned;
    maniGraphScalar d = l->duration;
    bool ok = l->model < source->modelCount &&
              (l->trigger == MANI_NO_PARAMETER ||
               ParameterOf(source, l->trigger, maniGraphParameterTrigger)) &&
              isfinite(l->period) && l->period > 0.0f && isfinite(l->transitionCost) &&
              l->transitionCost >= 0.0f && TagsValid(source, l->requiredTags) &&
              TagsValid(source, l->excludedTags) && QueryValid(source, l);
    if (!ok)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    bool duration = d.parameter == MANI_NO_PARAMETER
                        ? isfinite(d.value) && d.value >= 0.0f
                        : ParameterOf(source, d.parameter, maniGraphParameterFloat);
    if (!duration)
    {
        return Refuse(maniBuildReasonScalar, node, 0);
    }
    return TagsKnown(source, l) ? kOk : Refuse(maniBuildReasonTags, node, 0);
}
