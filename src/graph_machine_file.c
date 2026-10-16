// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Priorities need only be in order here: the builder refuses equal ones
// among a state's candidates, and a crafted file with equals still plays
// the same everywhere (the first in order wins).

#include "graph_machine_file.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static float WordFloat(const maniGraph* g, uint32_t at)
{
    uint32_t bits = Word(g, at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

uint32_t maniGraphMachineDataWords(const maniGraph* g, uint32_t data, uint32_t states,
                                   uint32_t available)
{
    if (available < MANI_GRAPH_MACHINE_WORDS)
    {
        return UINT32_MAX;
    }
    uint64_t count = Word(g, data + 2);
    uint64_t conditions = Word(g, data + 4);
    uint64_t head = MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * count +
                    MANI_GRAPH_CONDITION_WORDS * conditions +
                    MANI_GRAPH_STATE_WORDS * (uint64_t)states;
    if (count > MANI_GRAPH_MAX_TRANSITIONS || conditions > MANI_GRAPH_MAX_CONDITIONS * count ||
        head > available)
    {
        return UINT32_MAX;
    }
    uint64_t words = head;
    uint32_t at = data + (uint32_t)head - MANI_GRAPH_STATE_WORDS * states;
    for (uint32_t k = 0; k < states; ++k)
    {
        words += Word(g, at + MANI_GRAPH_STATE_WORDS * k + 3);
    }
    return words <= available ? (uint32_t)words : UINT32_MAX;
}

static uint32_t ParameterType(const maniGraph* g, uint32_t parameter)
{
    return parameter < g->parameterCount
               ? maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE)
               : UINT32_MAX;
}

static bool Finite(float value, float low)
{
    return isfinite(value) && value >= low;
}

// A condition's four words, as the builder writes them.
static bool ConditionValid(const maniGraph* g, uint32_t at)
{
    uint32_t parameter = Word(g, at + 1);
    uint32_t compare = Word(g, at + 2);
    uint32_t value = Word(g, at + 3);
    uint32_t type = ParameterType(g, parameter);
    bool none = parameter == MANI_NO_PARAMETER && compare == 0;
    switch (Word(g, at))
    {
    case maniGraphConditionParameter:
        if (type == maniGraphParameterBool)
        {
            return compare <= maniGraphCompareNotEqual && value <= 1u;
        }
        if (type == maniGraphParameterInt)
        {
            return compare <= maniGraphCompareAtLeast;
        }
        return type == maniGraphParameterFloat && compare >= maniGraphCompareLess &&
               compare <= maniGraphCompareAtLeast && Finite(WordFloat(g, at + 3), -HUGE_VALF);
    case maniGraphConditionTrigger:
        return type == maniGraphParameterTrigger && compare == 0 && value == 0;
    case maniGraphConditionTimeInState:
    case maniGraphConditionPhase:
        return none && Finite(WordFloat(g, at + 3), 0.0f);
    case maniGraphConditionFinished:
        return none && value == 0;
    case maniGraphConditionEvent:
        return value == 0;
    default:
        return false;
    }
}

// A transition's eight words: states, priority in order, flags, times,
// and its conditions where the running count puts them.
static uint32_t TransitionValid(const maniGraph* g, uint32_t at, uint32_t states, uint32_t before,
                                uint32_t first)
{
    uint32_t from = Word(g, at);
    uint32_t to = Word(g, at + 1);
    uint32_t flags = Word(g, at + 3);
    if (!(from < states || from == MANI_GRAPH_ANY_STATE))
    {
        return at;
    }
    if (to >= states || to == from)
    {
        return at + 1;
    }
    if (Word(g, at + 2) < before)
    {
        return at + 2;
    }
    if ((flags & 0xFu) > maniGraphInterruptionAny || (flags >> 4 & 0xFu) > maniGraphEntryMatch ||
        (flags >> 8) > maniGraphFadeSmoothstep)
    {
        return at + 3;
    }
    for (uint32_t k = 4; k < 6; ++k)
    {
        if (!Finite(WordFloat(g, at + k), 0.0f))
        {
            return at + k;
        }
    }
    return Word(g, at + 6) == first ? UINT32_MAX : at + 6;
}

// Whether transition t's entry matches or a condition of it reads the
// leading clip.
static bool Listens(const maniGraph* g, uint32_t at, uint32_t conditions)
{
    bool listens = (Word(g, at + 3) >> 4 & 0xFu) == maniGraphEntryMatch;
    for (uint32_t c = 0; c < Word(g, at + 7) && !listens; ++c)
    {
        uint32_t where = conditions + MANI_GRAPH_CONDITION_WORDS * (Word(g, at + 6) + c);
        listens = Word(g, where) >= maniGraphConditionPhase;
    }
    return listens;
}

static uint32_t TransitionsValid(const maniGraph* g, uint32_t data, uint32_t states,
                                 bool* listensOut)
{
    uint32_t count = Word(g, data + 2);
    uint32_t conditions = data + MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * count;
    uint32_t first = 0;
    uint32_t priority = 0;
    bool listens = false;
    for (uint32_t t = 0; t < count; ++t)
    {
        uint32_t at = data + MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * t;
        uint32_t wrong = TransitionValid(g, at, states, priority, first);
        uint32_t n = Word(g, at + 7);
        if (wrong == UINT32_MAX && (n > MANI_GRAPH_MAX_CONDITIONS || n > Word(g, data + 4) - first))
        {
            wrong = at + 7;
        }
        if (wrong != UINT32_MAX)
        {
            return wrong;
        }
        priority = Word(g, at + 2);
        for (uint32_t c = 0; c < n; ++c)
        {
            uint32_t where = conditions + MANI_GRAPH_CONDITION_WORDS * (first + c);
            if (!ConditionValid(g, where))
            {
                return where;
            }
        }
        listens = listens || Listens(g, at, conditions);
        first += n;
    }
    *listensOut = listens;
    return first == Word(g, data + 4) ? UINT32_MAX : data + 4;
}

static uint32_t Kind(const maniGraph* g, uint32_t node)
{
    return maniLoadU16(g->nodes + (size_t)node * MANI_GRAPH_NODE_SIZE);
}

// A state's four words and its list: a listening machine's nearest a
// machine before it, its clips before it in node order; another's
// nearest none (its counts are 0, or its words would not add up).
static uint32_t StateValid(const maniGraph* g, uint32_t p, uint32_t at, uint32_t lists,
                           bool listens, uint32_t first)
{
    uint32_t nearest = Word(g, at + 1);
    uint32_t count = Word(g, at + 3);
    if (Word(g, at) > 1u)
    {
        return at;
    }
    if (nearest != MANI_GRAPH_NO_NODE &&
        (!listens || nearest >= p || Kind(g, nearest) != maniGraphNodeMachine))
    {
        return at + 1;
    }
    if (Word(g, at + 2) != first)
    {
        return at + 2;
    }
    for (uint32_t c = 0; c < count; ++c)
    {
        uint32_t clip = Word(g, lists + first + c);
        if (clip >= p || Kind(g, clip) != maniGraphNodeClip ||
            (c > 0 && clip <= Word(g, lists + first + c - 1)))
        {
            return lists + first + c;
        }
    }
    return UINT32_MAX;
}

uint32_t maniGraphMachineValid(const maniGraph* g, uint32_t p, uint32_t data, uint32_t states,
                               uint32_t block)
{
    uint32_t chain = Word(g, data + 1);
    if (Word(g, data) >= states)
    {
        return data;
    }
    if (chain == 0 || chain > MANI_GRAPH_MAX_CHAIN)
    {
        return data + 1;
    }
    if (Word(g, data + 3) != block)
    {
        return data + 3;
    }
    bool listens = false;
    uint32_t wrong = TransitionsValid(g, data, states, &listens);
    if (wrong != UINT32_MAX)
    {
        return wrong;
    }
    if (Word(g, data + 5) != (listens ? 1u : 0u))
    {
        return data + 5;
    }
    uint32_t at = data + MANI_GRAPH_MACHINE_WORDS +
                  MANI_GRAPH_TRANSITION_WORDS * Word(g, data + 2) +
                  MANI_GRAPH_CONDITION_WORDS * Word(g, data + 4);
    uint32_t lists = at + MANI_GRAPH_STATE_WORDS * states;
    uint32_t first = 0;
    for (uint32_t k = 0; k < states && wrong == UINT32_MAX; ++k)
    {
        wrong = StateValid(g, p, at + MANI_GRAPH_STATE_WORDS * k, lists, listens, first);
        first += Word(g, at + MANI_GRAPH_STATE_WORDS * k + 3);
    }
    return wrong;
}
