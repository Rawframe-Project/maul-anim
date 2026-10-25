// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A machine decides at the start of an update from what the updates
// before it left (its leading clip's record among them): its clocks
// advance, a fade whose time is up completes, then the first candidate
// in priority order whose conditions hold fires
// (a fade of 0 completing at once, so the machine looks again, up to its
// chain limit). A fade sets each state's weight from the weights when it
// began: the destination's rises to 1, every other falls to 0, so an
// interrupted fade goes on from where it was.

#include "graph_machine_step.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "graph_machine.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint32_t Transition(const maniGraphMachine* m, uint32_t t, uint32_t k)
{
    return maniGraphMachineTransition(m, t, k);
}

// Transition t's word k as a binary32.
static float TransitionFloat(const maniGraphMachine* m, uint32_t t, uint32_t k)
{
    return maniGraphMachineFloat(m, MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * t + k);
}

// Transition t's first condition's data word.
static uint32_t FirstCondition(const maniGraphMachine* m, uint32_t t)
{
    return maniGraphMachineCondition(m, Transition(m, t, 6));
}

// A parameter condition: its parameter's value compared with the
// condition's value.
static bool Compares(const maniGraphMachine* m, const maniGraphCell* cells, uint32_t at)
{
    uint32_t parameter = maniGraphMachineWord(m, at + 1);
    uint32_t compare = maniGraphMachineWord(m, at + 2);
    uint32_t word = maniGraphMachineWord(m, at + 3);
    const uint8_t* entry = m->graph->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE;
    const maniGraphValue* v = &cells[parameter].value;
    uint32_t type = maniLoadU32(entry);
    if (type == maniGraphParameterBool)
    {
        return compare == maniGraphCompareEqual ? v->b == word : v->b != word;
    }
    bool integer = type == maniGraphParameterInt;
    double a = integer ? (double)v->i : (double)v->f[0];
    double b = integer ? (double)(int32_t)word : (double)maniGraphMachineFloat(m, at + 3);
    switch (compare)
    {
    case maniGraphCompareEqual:
        return a == b;
    case maniGraphCompareNotEqual:
        return a != b;
    case maniGraphCompareLess:
        return a < b;
    case maniGraphCompareAtMost:
        return a <= b;
    case maniGraphCompareGreater:
        return a > b;
    default:
        return a >= b;
    }
}

// Whether the leading clip passed phase v in the update before: below 1
// its position, by the whole loops it wrapped and where it was before and
// after; 1 or more the loops since the state was entered.
static bool Passed(const uint32_t* block, double v)
{
    double before = maniGraphBlockDouble(block, maniGraphBlockBefore) - v;
    double after = maniGraphBlockDouble(block, maniGraphBlockPosition) - v;
    double wraps = maniGraphBlockDouble(block, maniGraphBlockWraps);
    double move = maniGraphRecordMove(block);
    if (v >= 1.0)
    {
        double loops = maniGraphBlockDouble(block, maniGraphBlockLoops);
        return loops < v && v <= loops + fabs(move);
    }
    if (move > 0.0)
    {
        return wraps + floor(after) - floor(before) > 0.0;
    }
    // Backward the count is ceil(before) - ceil(after) - wraps.
    return move < 0.0 && floor(-after) - floor(-before) - wraps > 0.0;
}

static bool Holds(const maniGraphMachine* m, const maniGraphCell* cells, const uint32_t* block,
                  uint32_t at, uint32_t c)
{
    switch (maniGraphMachineWord(m, at))
    {
    case maniGraphConditionParameter:
        return Compares(m, cells, at);
    case maniGraphConditionTrigger:
        return cells[maniGraphMachineWord(m, at + 1)].value.b != 0;
    case maniGraphConditionTimeInState:
        return maniGraphBlockDouble(block, maniGraphBlockTime) >=
               (double)maniGraphMachineFloat(m, at + 3);
    case maniGraphConditionPhase:
        return Passed(block, (double)maniGraphMachineFloat(m, at + 3));
    case maniGraphConditionFinished:
        return block[maniGraphBlockFinished] != 0;
    default:
        return maniGraphMachineHeard(m, block, c);
    }
}

// Whether transition t's conditions all hold.
static bool AllHold(const maniGraphMachine* m, const maniGraphCell* cells, const uint32_t* block,
                    uint32_t t)
{
    uint32_t first = Transition(m, t, 6);
    bool holds = true;
    for (uint32_t c = first; c < first + Transition(m, t, 7) && holds; ++c)
    {
        holds = Holds(m, cells, block, maniGraphMachineCondition(m, c), c);
    }
    return holds;
}

// Whether t may fire now: a candidate of the current state, past its
// cooldown, allowed by the running transition's interruption.
static bool Candidate(const maniGraphMachine* m, const uint32_t* block, uint32_t t)
{
    uint32_t current = block[maniGraphBlockCurrent];
    uint32_t from = Transition(m, t, 0);
    if ((from != current && from != MANI_GRAPH_ANY_STATE) || Transition(m, t, 1) == current ||
        maniGraphMachineSince(m, block, t) < TransitionFloat(m, t, 5))
    {
        return false;
    }
    uint32_t running = block[maniGraphBlockRunning];
    if (running == MANI_GRAPH_NO_TRANSITION)
    {
        return true;
    }
    uint32_t rule = Transition(m, running, 3) & 0xFu;
    return rule == maniGraphInterruptionAny ||
           (rule == maniGraphInterruptionHigher && Transition(m, t, 2) < Transition(m, running, 2));
}

// The running fade's weights, or at its end the destination's 1 and no
// transition running.
static void Fade(const maniGraphMachine* m, uint32_t* block)
{
    uint32_t running = block[maniGraphBlockRunning];
    float duration = TransitionFloat(m, running, 4);
    float elapsed = maniGraphBlockFloat(block, maniGraphBlockElapsed);
    // An inertial transition switches at once; its duration is the
    // inertialize node's.
    bool done = !(elapsed < duration) || Transition(m, running, 3) >> 8 == maniGraphFadeInertial;
    float c = done ? 1.0f : elapsed / duration;
    if (Transition(m, running, 3) >> 8 == maniGraphFadeSmoothstep)
    {
        c = c * c * (3.0f - 2.0f * c);
    }
    uint32_t to = block[maniGraphBlockCurrent];
    for (uint32_t k = 0; k < m->states; ++k)
    {
        float start = maniGraphMachineStartWeight(block, k);
        float weight = k == to ? start + (1.0f - start) * c : start * (1.0f - c);
        maniGraphSetMachineWeight(block, k, done ? (k == to ? 1.0f : 0.0f) : weight);
    }
    block[maniGraphBlockRunning] = done ? MANI_GRAPH_NO_TRANSITION : running;
}

// Fires t: its triggers consumed, its timer reset, the weights kept as
// the fade's start, its destination current (from its start or at the
// leading clip's phase when entered at weight 0 by a restart or a match),
// the record cleared but the position.
static void Fire(const maniGraphMachine* m, maniGraphCell* cells, uint32_t* block, uint32_t t,
                 maniGraphEntered* entered)
{
    uint32_t first = FirstCondition(m, t);
    for (uint32_t c = 0; c < Transition(m, t, 7); ++c)
    {
        uint32_t at = first + MANI_GRAPH_CONDITION_WORDS * c;
        if (maniGraphMachineWord(m, at) == maniGraphConditionTrigger)
        {
            cells[maniGraphMachineWord(m, at + 1)].value.b = 0;
        }
    }
    uint32_t to = Transition(m, t, 1);
    uint32_t entry = Transition(m, t, 3) >> 4 & 0xFu;
    float duration = TransitionFloat(m, t, 4);
    if (Transition(m, t, 3) >> 8 == maniGraphFadeInertial && duration < entered->inertia)
    {
        entered->inertia = duration;
    }
    if (entry != maniGraphEntryKeep && !(maniGraphMachineWeight(block, to) > 0.0f))
    {
        entered->states[entered->count] = to;
        entered->entries[entered->count++] = entry;
    }
    maniGraphKeepStartWeights(m, block);
    maniGraphSetMachineSince(m, block, t, 0.0f);
    block[maniGraphBlockRunning] = t;
    block[maniGraphBlockCurrent] = to;
    maniGraphSetBlockFloat(block, maniGraphBlockElapsed, 0.0f);
    maniGraphSetBlockDouble(block, maniGraphBlockTime, 0.0);
    maniGraphClearRecord(m, block);
    Fade(m, block);
}

static void Advance(const maniGraphMachine* m, uint32_t* block, double seconds)
{
    double time = maniGraphBlockDouble(block, maniGraphBlockTime) + seconds;
    maniGraphSetBlockDouble(block, maniGraphBlockTime, time);
    for (uint32_t t = 0; t < maniGraphMachineCount(m); ++t)
    {
        double since = (double)maniGraphMachineSince(m, block, t) + seconds;
        maniGraphSetMachineSince(m, block, t, (float)since);
    }
    if (block[maniGraphBlockRunning] != MANI_GRAPH_NO_TRANSITION)
    {
        double elapsed = (double)maniGraphBlockFloat(block, maniGraphBlockElapsed) + seconds;
        maniGraphSetBlockFloat(block, maniGraphBlockElapsed, maniGraphSeconds(elapsed));
        Fade(m, block);
    }
}

void maniGraphStepMachine(const maniGraphMachine* m, maniGraphCell* cells, uint32_t* block,
                          double seconds, maniGraphEntered* entered)
{
    entered->count = 0;
    entered->inertia = HUGE_VALF;
    Advance(m, block, seconds);
    uint32_t count = maniGraphMachineCount(m);
    uint32_t limit = maniGraphMachineWord(m, 1);
    for (uint32_t chain = 0; chain < limit; ++chain)
    {
        uint32_t t = 0;
        while (t < count && !(Candidate(m, block, t) && AllHold(m, cells, block, t)))
        {
            ++t;
        }
        if (t == count)
        {
            return;
        }
        Fire(m, cells, block, t, entered);
        if (block[maniGraphBlockRunning] != MANI_GRAPH_NO_TRANSITION)
        {
            return;
        }
    }
}
