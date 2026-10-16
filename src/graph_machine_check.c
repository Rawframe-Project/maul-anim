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
        return isfinite(c->value.f[0]) && c->value.f[0] >= 0.0f;
    default:
        return false;
    }
}

static bool TransitionValid(const maniGraphSource* source, const maniGraphTransitionSource* t,
                            uint32_t states)
{
    bool ok = (t->from < states || t->from == MANI_GRAPH_ANY_STATE) && t->to < states &&
              t->from != t->to && t->interruption <= maniGraphInterruptionAny &&
              t->entry <= maniGraphEntryKeep && t->fade <= maniGraphFadeSmoothstep &&
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
    if ((m->transitions == NULL && m->transitionCount > 0) ||
        m->transitionCount > MANI_GRAPH_MAX_TRANSITIONS || m->entry >= n->inputCount ||
        m->chainLimit > MANI_GRAPH_MAX_CHAIN)
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

uint32_t maniGraphMachineSourceWords(const maniGraphNodeSource* n)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    uint32_t words = MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * m->transitionCount;
    for (uint32_t t = 0; t < m->transitionCount; ++t)
    {
        words += MANI_GRAPH_CONDITION_WORDS * m->transitions[t].conditionCount;
    }
    return words;
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

// A condition's words: a trigger's compare and value zero, time in
// state's parameter none; a parameter's value as its bits (bool, int or
// binary32 alike).
static uint32_t PutCondition(uint8_t* data, uint32_t at, const maniGraphConditionSource* c)
{
    bool parameter = c->kind == maniGraphConditionParameter;
    bool time = c->kind == maniGraphConditionTimeInState;
    uint32_t value = parameter ? c->value.b : (time ? Bits(c->value.f[0]) : 0u);
    at = Put(data, at, c->kind);
    at = Put(data, at, time ? MANI_NO_PARAMETER : c->parameter);
    at = Put(data, at, parameter ? c->compare : 0u);
    return Put(data, at, value);
}

uint32_t maniPutGraphMachine(uint8_t* data, uint32_t at, const maniGraphNodeSource* n,
                             uint32_t block, uint32_t* order, uint32_t* scratch, uint64_t* keys)
{
    const maniGraphMachineNodeSource* m = &n->machine;
    uint32_t count = m->transitionCount;
    for (uint32_t t = 0; t < count; ++t)
    {
        keys[t] = m->transitions[t].priority;
    }
    maniSortByKey(keys, count, order, scratch);
    at = Put(data, at, m->entry);
    at = Put(data, at, m->chainLimit != 0 ? m->chainLimit : 4u);
    at = Put(data, at, count);
    at = Put(data, at, block);
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
