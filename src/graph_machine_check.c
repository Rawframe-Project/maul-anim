// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_machine_check.h"

#include "bytes.h"
#include "graph_file.h"
#include "sort.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static bool ParameterIs(const maniGraphSource* source, uint32_t parameter,
                        maniGraphParameterType type)
{
    return parameter < source->parameterCount && source->parameters[parameter].type == type;
}

// A bool compared for equality with 0 or 1; an int any way; a float in
// order with a finite value.
static bool ComparisonValid(const maniGraphSource* source, const maniGraphConditionSource* c)
{
    if (ParameterIs(source, c->parameter, maniGraphParameterBool))
    {
        return c->compare <= maniGraphCompareNotEqual && c->value.b <= 1u;
    }
    if (ParameterIs(source, c->parameter, maniGraphParameterInt))
    {
        return c->compare <= maniGraphCompareAtLeast;
    }
    return ParameterIs(source, c->parameter, maniGraphParameterFloat) &&
           c->compare >= maniGraphCompareLess && c->compare <= maniGraphCompareAtLeast &&
           isfinite(c->value.f[0]);
}

static bool ConditionValid(const maniGraphSource* source, const maniGraphConditionSource* c)
{
    switch (c->kind)
    {
    case maniGraphConditionParameter:
        return ComparisonValid(source, c);
    case maniGraphConditionTrigger:
        return ParameterIs(source, c->parameter, maniGraphParameterTrigger);
    case maniGraphConditionTimeInState:
    case maniGraphConditionPhase:
        return isfinite(c->value.f[0]) && c->value.f[0] >= 0.0f;
    case maniGraphConditionFinished:
        return true;
    case maniGraphConditionEvent:
        return c->event != NULL && c->eventLength > 0;
    default:
        return false;
    }
}

static bool TransitionValid(const maniGraphSource* source, const maniGraphTransitionSource* t,
                            uint32_t states)
{
    bool ok = (t->from < states || t->from == MANI_GRAPH_ANY_STATE) && t->to < states &&
              t->from != t->to && t->interruption <= maniGraphInterruptionAny &&
              t->entry <= maniGraphEntryMatch && t->fade <= maniGraphFadeSmoothstep &&
              isfinite(t->duration) && t->duration >= 0.0f && isfinite(t->cooldown) &&
              t->cooldown >= 0.0f && t->conditionCount <= MANI_GRAPH_MAX_CONDITIONS &&
              (t->conditions != NULL || t->conditionCount == 0);
    for (uint32_t k = 0; k < t->conditionCount && ok; ++k)
    {
        ok = ConditionValid(source, &t->conditions[k]);
    }
    return ok;
}

// Whether two transitions are ever candidates together: from one state,
// or either from any state.
static bool Rivals(const maniGraphTransitionSource* a, const maniGraphTransitionSource* b)
{
    return a->from == b->from || a->from == MANI_GRAPH_ANY_STATE || b->from == MANI_GRAPH_ANY_STATE;
}

maniBuildResult maniCheckGraphMachine(const maniGraphSource* source, const maniGraphNodeSource* n,
                                      uint32_t node)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    bool valid = !(m->transitions == NULL && m->transitionCount > 0) &&
                 m->transitionCount <= MANI_GRAPH_MAX_TRANSITIONS && m->entry < n->inputCount &&
                 m->chainLimit <= MANI_GRAPH_MAX_CHAIN && (m->finals != NULL || m->finalCount == 0);
    for (uint32_t k = 0; k < m->finalCount && valid; ++k)
    {
        valid = m->finals[k] < n->inputCount;
    }
    if (!valid)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    for (uint32_t t = 0; t < m->transitionCount; ++t)
    {
        const maniGraphTransitionSource* a = &m->transitions[t];
        bool ok = TransitionValid(source, a, n->inputCount);
        for (uint32_t u = 0; u < t && ok; ++u)
        {
            const maniGraphTransitionSource* b = &m->transitions[u];
            ok = !(Rivals(a, b) && a->priority == b->priority);
        }
        if (!ok)
        {
            return Refuse(maniBuildReasonTransition, node, t);
        }
    }
    return kOk;
}

bool maniGraphMachineListens(const maniGraphNodeSource* n)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    bool listens = false;
    for (uint32_t t = 0; t < m->transitionCount && !listens; ++t)
    {
        const maniGraphTransitionSource* a = &m->transitions[t];
        listens = a->entry == maniGraphEntryMatch;
        for (uint32_t c = 0; c < a->conditionCount && !listens; ++c)
        {
            listens = a->conditions[c].kind >= maniGraphConditionPhase;
        }
    }
    return listens;
}

uint32_t maniGraphMachineSourceWords(const maniGraphNodeSource* n)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    uint32_t words = MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * m->transitionCount +
                     MANI_GRAPH_STATE_WORDS * n->inputCount;
    return words + MANI_GRAPH_CONDITION_WORDS * maniGraphMachineConditions(n);
}

uint32_t maniGraphMachineConditions(const maniGraphNodeSource* n)
{
    uint32_t count = 0;
    for (uint32_t t = 0; t < n->machine.transitionCount; ++t)
    {
        count += n->machine.transitions[t].conditionCount;
    }
    return count;
}

uint32_t maniGraphReachClips(maniGraphReach* r, uint32_t from, uint32_t* clipsOut,
                             uint32_t* nearestOut)
{
    uint32_t mark = ++r->mark;
    uint32_t depth = 1;
    uint32_t count = 0;
    uint32_t nearest = MANI_GRAPH_NO_NODE;
    r->stack[0] = from;
    r->stamp[from] = mark;
    while (depth > 0)
    {
        uint32_t v = r->stack[--depth];
        const maniGraphNodeSource* node = &r->source->nodes[v];
        uint32_t at = r->position != NULL ? r->position[v] : v;
        if (node->kind == maniGraphNodeClip && clipsOut != NULL)
        {
            clipsOut[count] = at;
        }
        count += node->kind == maniGraphNodeClip ? 1u : 0u;
        bool machine = node->kind == maniGraphNodeMachine;
        nearest = machine && (nearest == MANI_GRAPH_NO_NODE || at > nearest) ? at : nearest;
        for (uint32_t k = 0; k < node->inputCount; ++k)
        {
            uint32_t u = node->inputs[k];
            if (r->stamp[u] != mark)
            {
                r->stamp[u] = mark;
                r->stack[depth++] = u;
            }
        }
    }
    if (nearestOut != NULL)
    {
        *nearestOut = nearest;
    }
    return count;
}

static uint32_t Put(uint8_t* data, uint32_t at, uint32_t word)
{
    maniStoreU32(data + 4 * (size_t)at, word);
    return at + 1;
}

static uint32_t Bits(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

// A condition's words: a parameter's value as its bits (bool, int or
// binary32 alike), a trigger's compare and value zero; the others read
// no parameter: a time's or phase's value as bits, an event's name hash
// in the parameter and compare words.
static uint32_t PutCondition(uint8_t* data, uint32_t at, const maniGraphConditionSource* c)
{
    uint32_t words[4] = {c->kind, MANI_NO_PARAMETER, 0u, 0u};
    if (c->kind == maniGraphConditionParameter || c->kind == maniGraphConditionTrigger)
    {
        words[1] = c->parameter;
        words[2] = c->kind == maniGraphConditionParameter ? c->compare : 0u;
        words[3] = c->kind == maniGraphConditionParameter ? c->value.b : 0u;
    }
    else if (c->kind == maniGraphConditionTimeInState || c->kind == maniGraphConditionPhase)
    {
        words[3] = Bits(c->value.f[0]);
    }
    else if (c->kind == maniGraphConditionEvent)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, c->event, (int32_t)c->eventLength);
        words[1] = (uint32_t)hash;
        words[2] = (uint32_t)(hash >> 32);
    }
    for (uint32_t w = 0; w < 4; ++w)
    {
        at = Put(data, at, words[w]);
    }
    return at;
}

static uint32_t PutTransitions(uint8_t* data, uint32_t at, const maniGraphNodeSource* n,
                               const uint32_t* order)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    uint32_t count = m->transitionCount;
    uint32_t conditions = at + MANI_GRAPH_TRANSITION_WORDS * count;
    uint32_t first = 0;
    for (uint32_t k = 0; k < count; ++k)
    {
        const maniGraphTransitionSource* t = &m->transitions[order[k]];
        const uint32_t words[8] = {t->from,
                                   t->to,
                                   t->priority,
                                   t->interruption + (t->entry << 4) + (t->fade << 8),
                                   Bits(t->duration),
                                   Bits(t->cooldown),
                                   first,
                                   t->conditionCount};
        for (uint32_t w = 0; w < 8; ++w)
        {
            at = Put(data, at, words[w]);
        }
        for (uint32_t c = 0; c < t->conditionCount; ++c)
        {
            conditions = PutCondition(data, conditions, &t->conditions[c]);
        }
        first += t->conditionCount;
    }
    return conditions;
}

uint32_t maniGraphSortedClips(maniGraphMachineWork* work, uint32_t from, uint32_t* nearestOut)
{
    uint32_t count = maniGraphReachClips(work->reach, from, work->clips, nearestOut);
    for (uint32_t c = 0; c < count; ++c)
    {
        work->keys[c] = work->clips[c];
    }
    maniSortByKey(work->keys, count, work->order, work->scratch);
    return count;
}

// Per state its final flag, nearest machine and clip list (a listening
// machine's), then the lists, each in node order.
static uint32_t PutStates(uint8_t* data, uint32_t at, const maniGraphNodeSource* n,
                          maniGraphMachineWork* work)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    bool listens = maniGraphMachineListens(n);
    uint32_t lists = at + MANI_GRAPH_STATE_WORDS * n->inputCount;
    uint32_t first = 0;
    for (uint32_t k = 0; k < n->inputCount; ++k)
    {
        bool final = false;
        for (uint32_t f = 0; f < m->finalCount; ++f)
        {
            final = final || m->finals[f] == k;
        }
        uint32_t nearest = MANI_GRAPH_NO_NODE;
        uint32_t count = listens ? maniGraphSortedClips(work, n->inputs[k], &nearest) : 0u;
        for (uint32_t c = 0; c < count; ++c)
        {
            (void)Put(data, lists + first + c, work->clips[work->order[c]]);
        }
        const uint32_t words[4] = {final ? 1u : 0u, nearest, first, count};
        for (uint32_t w = 0; w < 4; ++w)
        {
            at = Put(data, at, words[w]);
        }
        first += count;
    }
    return lists + first;
}

uint32_t maniPutGraphMachine(uint8_t* data, uint32_t at, const maniGraphNodeSource* n,
                             uint32_t block, maniGraphMachineWork* work)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    uint32_t count = m->transitionCount;
    for (uint32_t t = 0; t < count; ++t)
    {
        work->keys[t] = m->transitions[t].priority;
    }
    maniSortByKey(work->keys, count, work->order, work->scratch);
    at = Put(data, at, m->entry);
    at = Put(data, at, m->chainLimit != 0 ? m->chainLimit : 4u);
    at = Put(data, at, count);
    at = Put(data, at, block);
    at = Put(data, at, maniGraphMachineConditions(n));
    at = Put(data, at, maniGraphMachineListens(n) ? 1u : 0u);
    at = PutTransitions(data, at, n, work->order);
    return PutStates(data, at, n, work);
}
