// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The searching nodes read from the graph JSON form: a motion matching
// node by its database's name, a learned node by its model's name; each
// query an array of parameters by name, tags a number or an int
// parameter.

#include "graph_json_search.h"

#include "arena.h"
#include "graph_json_read.h"
#include "json.h"

#include "maul-anim/base.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef maniGraphReader Reader;
typedef maniGraphScope Scope;

#define ABSENT MANI_JSON_NONE

// Tags: a whole number from 0 to 2^32 - 1, or {"parameter": name}.
static bool Tags(Reader* r, const Scope* scope, uint32_t value, maniGraphTags* out)
{
    *out = (maniGraphTags){MANI_NO_PARAMETER, 0};
    if (r->json->values[value].kind == maniJsonNumber)
    {
        int64_t bits = 0;
        bool whole = maniJsonInteger(r->json, value, 0, UINT32_MAX, &bits);
        out->value = (uint32_t)bits;
        return whole || maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    static const char* const kKeys[] = {"parameter"};
    uint32_t found = 0;
    return maniGraphJsonMembers(r, value, kKeys, 1, 1u, &found) &&
           maniGraphJsonRefer(r, &scope->parameters, found, &out->parameter);
}

// A query: an array of parameters by name.
static bool Query(Reader* r, const Scope* scope, uint32_t value, const uint32_t** queryOut,
                  uint32_t* countOut)
{
    if (!maniGraphJsonArray(r, value, countOut))
    {
        return false;
    }
    uint32_t* query = maniArenaTake(&r->arena, *countOut, sizeof(uint32_t));
    if (query == NULL && *countOut > 0)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t q = 0; q < *countOut; ++q)
    {
        e = maniGraphJsonNext(r->json, e, q == 0);
        if (!maniGraphJsonRefer(r, &scope->parameters, e, &query[q]))
        {
            return false;
        }
    }
    *queryOut = query;
    return true;
}

// A motion matching node (f the members of its form): its database by
// name; its trigger none, query empty, period 0.1 s (Holden's and
// Unreal's), the other numbers, tags and duration 0 when left out.
bool maniGraphJsonMatch(Reader* r, const Scope* scope, const uint32_t* f,
                        maniGraphMatchNodeSource* m)
{
    *m = (maniGraphMatchNodeSource){.trigger = MANI_NO_PARAMETER,
                                    .period = 0.1f,
                                    .requiredTags = {MANI_NO_PARAMETER, 0},
                                    .excludedTags = {MANI_NO_PARAMETER, 0},
                                    .duration = {MANI_NO_PARAMETER, 0.0f}};
    float* numbers[6] = {&m->period,      &m->jumpThreshold,  &m->endExclusion,
                         &m->historyTime, &m->continuingBias, &m->transitionCost};
    bool ok = maniGraphJsonRefer(r, &scope->databases, f[1], &m->database) &&
              (f[2] == ABSENT || maniGraphJsonRefer(r, &scope->parameters, f[2], &m->trigger)) &&
              (f[3] == ABSENT || Query(r, scope, f[3], &m->query, &m->queryCount));
    for (uint32_t k = 0; k < 6 && ok; ++k)
    {
        ok = f[4 + k] == ABSENT || maniGraphJsonFloat(r, f[4 + k], numbers[k]);
    }
    return ok && (f[10] == ABSENT || maniGraphJsonIndex(r, f[10], &m->historyCount)) &&
           (f[11] == ABSENT || Tags(r, scope, f[11], &m->requiredTags)) &&
           (f[12] == ABSENT || Tags(r, scope, f[12], &m->excludedTags)) &&
           (f[13] == ABSENT || maniGraphJsonScalar(r, scope, f[13], &m->duration));
}

// A learned node (f the members of its form): its model by name; its
// trigger none, query empty, period 0.1 s (as a matching node's), its
// transition cost, tags and duration 0 when left out.
bool maniGraphJsonLearned(Reader* r, const Scope* scope, const uint32_t* f,
                          maniGraphLearnedNodeSource* l)
{
    *l = (maniGraphLearnedNodeSource){.trigger = MANI_NO_PARAMETER,
                                      .period = 0.1f,
                                      .requiredTags = {MANI_NO_PARAMETER, 0},
                                      .excludedTags = {MANI_NO_PARAMETER, 0},
                                      .duration = {MANI_NO_PARAMETER, 0.0f}};
    return maniGraphJsonRefer(r, &scope->models, f[1], &l->model) &&
           (f[2] == ABSENT || maniGraphJsonRefer(r, &scope->parameters, f[2], &l->trigger)) &&
           (f[3] == ABSENT || Query(r, scope, f[3], &l->query, &l->queryCount)) &&
           (f[4] == ABSENT || maniGraphJsonFloat(r, f[4], &l->period)) &&
           (f[5] == ABSENT || maniGraphJsonFloat(r, f[5], &l->transitionCost)) &&
           (f[6] == ABSENT || Tags(r, scope, f[6], &l->requiredTags)) &&
           (f[7] == ABSENT || Tags(r, scope, f[7], &l->excludedTags)) &&
           (f[8] == ABSENT || maniGraphJsonScalar(r, scope, f[8], &l->duration));
}
