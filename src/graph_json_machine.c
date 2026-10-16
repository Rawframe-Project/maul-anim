// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A transition: {"from": state or "any", "to", "priority", "interruption":
// "none" | "higher" | "any", "entry": "restart" | "keep" | "match",
// "fade": "linear" | "smoothstep", "duration", "cooldown", "conditions"},
// all but from and to optional. A condition: {"parameter": name,
// "compare": "==" | "!=" | "<" | "<=" | ">" | ">=", "value": of the
// parameter's type}, {"trigger": name}, {"timeInState": seconds},
// {"phase": number}, {"finished": true} or {"event": name}. A machine's
// finals: an array of states.

#include "graph_json_machine.h"

#include "arena.h"
#include "graph_json_read.h"
#include "json.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ABSENT MANI_JSON_NONE

// An optional word among count, its index in out (0 when absent).
static bool Choice(maniGraphReader* r, uint32_t value, const char* const* words, uint32_t count,
                   uint32_t* out)
{
    *out = value == ABSENT ? 0u : maniGraphJsonWord(r, value, words, count);
    return *out < count || maniGraphJsonFault(r, value, maniGraphFormValue);
}

// A parameter condition's value, of its parameter's type: true or false,
// a whole number, or a number.
static bool Compared(maniGraphReader* r, const maniGraphScope* scope, uint32_t value,
                     maniGraphConditionSource* c)
{
    maniGraphParameterType type = scope->sources[c->parameter].type;
    if (type == maniGraphParameterBool)
    {
        bool b = false;
        bool ok = maniGraphJsonBool(r, value, &b);
        c->value.b = b ? 1u : 0u;
        return ok;
    }
    if (type == maniGraphParameterInt)
    {
        int64_t whole = 0;
        bool ok = maniJsonInteger(r->json, value, INT32_MIN, INT32_MAX, &whole);
        c->value.i = (int32_t)whole;
        return ok || maniGraphJsonFault(r, value, maniGraphFormValue);
    }
    return maniGraphJsonFloat(r, value, &c->value.f[0]);
}

// The one member naming a condition's kind (its index among the kind
// keys), or a fault on a second or on none.
static bool KindOf(maniGraphReader* r, uint32_t object, const uint32_t* f, uint32_t* kindOut)
{
    // The keys' order is the members' from parameter on.
    static const uint32_t kKinds[] = {0, 3, 4, 5, 6, 7};
    *kindOut = UINT32_MAX;
    for (uint32_t k = 0; k < 6; ++k)
    {
        uint32_t at = f[kKinds[k]];
        if (at != ABSENT && *kindOut != UINT32_MAX)
        {
            return maniGraphJsonFault(r, at - 1, maniGraphFormMember);
        }
        *kindOut = at != ABSENT ? k : *kindOut;
    }
    if (*kindOut == UINT32_MAX)
    {
        return maniGraphJsonFault(r, object, maniGraphFormMissing);
    }
    uint32_t extra = *kindOut != 0 ? (f[1] != ABSENT ? f[1] : f[2]) : ABSENT;
    return extra == ABSENT || maniGraphJsonFault(r, extra - 1, maniGraphFormMember);
}

static bool Condition(maniGraphReader* r, const maniGraphScope* scope, uint32_t object,
                      maniGraphConditionSource* c)
{
    static const char* const kKeys[] = {"parameter",   "compare", "value",    "trigger",
                                        "timeInState", "phase",   "finished", "event"};
    static const char* const kCompares[] = {"==", "!=", "<", "<=", ">", ">="};
    static const maniGraphConditionKind kKind[] = {
        maniGraphConditionParameter, maniGraphConditionTrigger,  maniGraphConditionTimeInState,
        maniGraphConditionPhase,     maniGraphConditionFinished, maniGraphConditionEvent};
    uint32_t f[8];
    uint32_t kind = 0;
    *c = (maniGraphConditionSource){0};
    if (!maniGraphJsonMembers(r, object, kKeys, 8, 0u, f) || !KindOf(r, object, f, &kind))
    {
        return false;
    }
    c->kind = kKind[kind];
    c->parameter = kind < 2 ? 0u : MANI_NO_PARAMETER;
    bool finished = false;
    switch (c->kind)
    {
    case maniGraphConditionParameter:
        if (f[1] == ABSENT || f[2] == ABSENT)
        {
            return maniGraphJsonFault(r, object, maniGraphFormMissing);
        }
        return maniGraphJsonRefer(r, &scope->parameters, f[0], &c->parameter) &&
               Choice(r, f[1], kCompares, 6, &c->compare) && Compared(r, scope, f[2], c);
    case maniGraphConditionTrigger:
        return maniGraphJsonRefer(r, &scope->parameters, f[3], &c->parameter);
    case maniGraphConditionTimeInState:
    case maniGraphConditionPhase:
        return maniGraphJsonFloat(r, f[kind + 2], &c->value.f[0]);
    case maniGraphConditionFinished:
        // Only true: a condition that never holds is a mistake.
        return maniGraphJsonBool(r, f[6], &finished) &&
               (finished || maniGraphJsonFault(r, f[6], maniGraphFormValue));
    default:
        return maniGraphJsonName(r, f[7], &c->event, &c->eventLength);
    }
}

static bool Conditions(maniGraphReader* r, const maniGraphScope* scope, uint32_t value,
                       maniGraphTransitionSource* t)
{
    uint32_t count = 0;
    if (value == ABSENT)
    {
        return true;
    }
    if (!maniGraphJsonArray(r, value, &count))
    {
        return false;
    }
    maniGraphConditionSource* c =
        count > 0 ? maniArenaTake(&r->arena, count, sizeof(maniGraphConditionSource)) : NULL;
    if (count > 0 && c == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Condition(r, scope, e, &c[k]))
        {
            return false;
        }
    }
    t->conditions = c;
    t->conditionCount = count;
    return true;
}

// A state index, or "any" for a transition's source.
static bool State(maniGraphReader* r, uint32_t value, bool any, uint32_t* out)
{
    static const char* const kAny[] = {"any"};
    if (any && maniGraphJsonWord(r, value, kAny, 1) == 0)
    {
        *out = MANI_GRAPH_ANY_STATE;
        return true;
    }
    return maniGraphJsonIndex(r, value, out);
}

static bool Transition(maniGraphReader* r, const maniGraphScope* scope, uint32_t object,
                       maniGraphTransitionSource* t)
{
    static const char* const kKeys[] = {"from", "to",       "priority", "interruption", "entry",
                                        "fade", "duration", "cooldown", "conditions"};
    static const char* const kInterruptions[] = {"none", "higher", "any"};
    static const char* const kEntries[] = {"restart", "keep", "match"};
    static const char* const kFades[] = {"linear", "smoothstep"};
    uint32_t f[9];
    *t = (maniGraphTransitionSource){0};
    return maniGraphJsonMembers(r, object, kKeys, 9, 3u, f) && State(r, f[0], true, &t->from) &&
           State(r, f[1], false, &t->to) &&
           (f[2] == ABSENT || maniGraphJsonIndex(r, f[2], &t->priority)) &&
           Choice(r, f[3], kInterruptions, 3, &t->interruption) &&
           Choice(r, f[4], kEntries, 3, &t->entry) && Choice(r, f[5], kFades, 2, &t->fade) &&
           (f[6] == ABSENT || maniGraphJsonFloat(r, f[6], &t->duration)) &&
           (f[7] == ABSENT || maniGraphJsonFloat(r, f[7], &t->cooldown)) &&
           Conditions(r, scope, f[8], t);
}

// The final states: an array of state indices.
static bool Finals(maniGraphReader* r, uint32_t value, maniGraphMachineNodeSource* m)
{
    uint32_t count = 0;
    if (!maniGraphJsonArray(r, value, &count))
    {
        return false;
    }
    uint32_t* finals = count > 0 ? maniArenaTake(&r->arena, count, sizeof(uint32_t)) : NULL;
    if (count > 0 && finals == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = value;
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!maniGraphJsonIndex(r, e, &finals[k]))
        {
            return false;
        }
    }
    m->finals = finals;
    m->finalCount = count;
    return true;
}

bool maniGraphJsonMachine(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                          maniGraphNodeSource* n)
{
    maniGraphMachineNodeSource* m = &n->machine;
    *m = (maniGraphMachineNodeSource){0};
    if ((members[0] != ABSENT && !maniGraphJsonIndex(r, members[0], &m->entry)) ||
        (members[1] != ABSENT && !maniGraphJsonIndex(r, members[1], &m->chainLimit)))
    {
        return false;
    }
    if (members[3] != ABSENT && !Finals(r, members[3], m))
    {
        return false;
    }
    uint32_t count = 0;
    if (members[2] == ABSENT)
    {
        return true;
    }
    if (!maniGraphJsonArray(r, members[2], &count))
    {
        return false;
    }
    maniGraphTransitionSource* t =
        count > 0 ? maniArenaTake(&r->arena, count, sizeof(maniGraphTransitionSource)) : NULL;
    if (count > 0 && t == NULL)
    {
        return maniGraphJsonNoMemory(r);
    }
    uint32_t e = members[2];
    for (uint32_t k = 0; k < count; ++k)
    {
        e = maniGraphJsonNext(r->json, e, k == 0);
        if (!Transition(r, scope, e, &t[k]))
        {
            return false;
        }
    }
    m->transitions = t;
    m->transitionCount = count;
    return true;
}
