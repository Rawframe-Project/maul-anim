// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A block's words: current state, running transition, elapsed fade
// seconds (binary32), seconds in state, the leading clip's positions
// before and after, its wraps and the loops since entry before the move
// (binary64, low word first), the finished flag; then per state its weight and fade-start weight;
// then per transition its seconds since it fired (+inf before it has); then the conditions' heard
// bits.

#include "graph_machine.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/graph.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float FloatOf(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static uint32_t BitsOf(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

static double DoubleOf(uint32_t low, uint32_t high)
{
    uint64_t bits = (uint64_t)low + ((uint64_t)high << 32);
    double value = 0.0;
    memcpy(&value, &bits, sizeof value);
    return value;
}

uint32_t maniGraphMachineWord(const maniGraphMachine* m, uint32_t at)
{
    return maniLoadU32(m->graph->data + 4 * (size_t)(m->data + at));
}

float maniGraphMachineFloat(const maniGraphMachine* m, uint32_t at)
{
    return FloatOf(maniGraphMachineWord(m, at));
}

uint32_t maniGraphMachineCount(const maniGraphMachine* m)
{
    return maniGraphMachineWord(m, 2);
}

uint32_t maniGraphMachineTransition(const maniGraphMachine* m, uint32_t t, uint32_t k)
{
    return maniGraphMachineWord(m, MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * t + k);
}

uint32_t maniGraphMachineConditionCount(const maniGraphMachine* m)
{
    return maniGraphMachineWord(m, 4);
}

uint32_t maniGraphMachineCondition(const maniGraphMachine* m, uint32_t c)
{
    return MANI_GRAPH_MACHINE_WORDS + MANI_GRAPH_TRANSITION_WORDS * maniGraphMachineCount(m) +
           MANI_GRAPH_CONDITION_WORDS * c;
}

bool maniGraphMachineListening(const maniGraphMachine* m)
{
    return maniGraphMachineWord(m, 5) != 0;
}

uint32_t maniGraphMachineState(const maniGraphMachine* m, uint32_t k, uint32_t w)
{
    uint32_t states = maniGraphMachineCondition(m, maniGraphMachineConditionCount(m));
    return maniGraphMachineWord(m, states + MANI_GRAPH_STATE_WORDS * k + w);
}

uint32_t maniGraphMachineClip(const maniGraphMachine* m, uint32_t k, uint32_t i)
{
    uint32_t lists = maniGraphMachineCondition(m, maniGraphMachineConditionCount(m)) +
                     MANI_GRAPH_STATE_WORDS * m->states;
    return maniGraphMachineWord(m, lists + maniGraphMachineState(m, k, 2) + i);
}

uint32_t maniGraphMachineBlockAt(const maniGraphMachine* m)
{
    return maniGraphMachineWord(m, 3);
}

uint32_t maniGraphMachineBlockWords(const maniGraphMachine* m)
{
    return MANI_GRAPH_BLOCK_SIZE(m->states, maniGraphMachineCount(m),
                                 maniGraphMachineConditionCount(m));
}

float maniGraphBlockFloat(const uint32_t* block, uint32_t at)
{
    return FloatOf(block[at]);
}

void maniGraphSetBlockFloat(uint32_t* block, uint32_t at, float value)
{
    block[at] = BitsOf(value);
}

double maniGraphBlockDouble(const uint32_t* block, uint32_t at)
{
    return DoubleOf(block[at], block[at + 1]);
}

void maniGraphSetBlockDouble(uint32_t* block, uint32_t at, double value)
{
    uint64_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    block[at] = (uint32_t)bits;
    block[at + 1] = (uint32_t)(bits >> 32);
}

float maniGraphMachineWeight(const uint32_t* block, uint32_t k)
{
    return FloatOf(block[MANI_GRAPH_BLOCK_WORDS + 2 * k]);
}

void maniGraphSetMachineWeight(uint32_t* block, uint32_t k, float weight)
{
    block[MANI_GRAPH_BLOCK_WORDS + 2 * k] = BitsOf(weight);
}

float maniGraphMachineStartWeight(const uint32_t* block, uint32_t k)
{
    return FloatOf(block[MANI_GRAPH_BLOCK_WORDS + 2 * k + 1]);
}

void maniGraphKeepStartWeights(const maniGraphMachine* m, uint32_t* block)
{
    for (uint32_t k = 0; k < m->states; ++k)
    {
        block[MANI_GRAPH_BLOCK_WORDS + 2 * k + 1] = block[MANI_GRAPH_BLOCK_WORDS + 2 * k];
    }
}

static uint32_t SinceAt(const maniGraphMachine* m, uint32_t t)
{
    return MANI_GRAPH_BLOCK_WORDS + 2 * m->states + t;
}

float maniGraphMachineSince(const maniGraphMachine* m, const uint32_t* block, uint32_t t)
{
    return FloatOf(block[SinceAt(m, t)]);
}

void maniGraphSetMachineSince(const maniGraphMachine* m, uint32_t* block, uint32_t t, float seconds)
{
    block[SinceAt(m, t)] = BitsOf(seconds);
}

static uint32_t HeardAt(const maniGraphMachine* m, uint32_t c)
{
    return SinceAt(m, maniGraphMachineCount(m)) + c / 32;
}

bool maniGraphMachineHeard(const maniGraphMachine* m, const uint32_t* block, uint32_t c)
{
    return (block[HeardAt(m, c)] >> (c % 32) & 1u) != 0;
}

void maniGraphSetMachineHeard(const maniGraphMachine* m, uint32_t* block, uint32_t c)
{
    block[HeardAt(m, c)] |= 1u << (c % 32);
}

double maniGraphRecordMove(const uint32_t* block)
{
    return maniGraphBlockDouble(block, maniGraphBlockWraps) +
           (maniGraphBlockDouble(block, maniGraphBlockPosition) -
            maniGraphBlockDouble(block, maniGraphBlockBefore));
}

void maniGraphClearRecord(const maniGraphMachine* m, uint32_t* block)
{
    block[maniGraphBlockBefore] = block[maniGraphBlockPosition];
    block[maniGraphBlockBefore + 1] = block[maniGraphBlockPosition + 1];
    maniGraphSetBlockDouble(block, maniGraphBlockWraps, 0.0);
    maniGraphSetBlockDouble(block, maniGraphBlockLoops, 0.0);
    block[maniGraphBlockFinished] = 0;
    uint32_t conditions = maniGraphMachineConditionCount(m);
    for (uint32_t w = 0; w < (conditions + 31) / 32; ++w)
    {
        block[HeardAt(m, 0) + w] = 0;
    }
}

void maniGraphResetMachine(const maniGraphMachine* m, uint32_t* block)
{
    uint32_t entry = maniGraphMachineWord(m, 0);
    block[maniGraphBlockCurrent] = entry;
    block[maniGraphBlockRunning] = MANI_GRAPH_NO_TRANSITION;
    maniGraphSetBlockFloat(block, maniGraphBlockElapsed, 0.0f);
    maniGraphSetBlockDouble(block, maniGraphBlockTime, 0.0);
    maniGraphSetBlockDouble(block, maniGraphBlockPosition, 0.0);
    maniGraphClearRecord(m, block);
    for (uint32_t k = 0; k < m->states; ++k)
    {
        maniGraphSetMachineWeight(block, k, k == entry ? 1.0f : 0.0f);
    }
    maniGraphKeepStartWeights(m, block);
    for (uint32_t t = 0; t < maniGraphMachineCount(m); ++t)
    {
        maniGraphSetMachineSince(m, block, t, HUGE_VALF);
    }
}

static bool UnitWeight(uint32_t bits)
{
    float value = FloatOf(bits);
    return isfinite(value) && value >= 0.0f && value <= 1.0f;
}

static double SavedDouble(const uint8_t* bytes, uint32_t at)
{
    return DoubleOf(maniLoadU32(bytes + 4 * at), maniLoadU32(bytes + 4 * at + 4));
}

static bool Unit(double value)
{
    return value >= 0.0 && value <= 1.0;
}

// The saved record: positions in [0, 1], whole wraps, loops not
// negative, a finished flag.
static uint32_t RecordValid(const uint8_t* bytes)
{
    double wraps = SavedDouble(bytes, maniGraphBlockWraps);
    double loops = SavedDouble(bytes, maniGraphBlockLoops);
    if (!Unit(SavedDouble(bytes, maniGraphBlockBefore)))
    {
        return maniGraphBlockBefore;
    }
    if (!Unit(SavedDouble(bytes, maniGraphBlockPosition)))
    {
        return maniGraphBlockPosition;
    }
    if (!(isfinite(wraps) && wraps == floor(wraps)))
    {
        return maniGraphBlockWraps;
    }
    if (!(isfinite(loops) && loops >= 0.0))
    {
        return maniGraphBlockLoops;
    }
    return maniLoadU32(bytes + 4 * maniGraphBlockFinished) <= 1u ? UINT32_MAX
                                                                 : (uint32_t)maniGraphBlockFinished;
}

// The saved header's words: a state, a running transition into it or
// none, a clock and a time not negative, then the record.
static uint32_t HeaderValid(const maniGraphMachine* m, const uint8_t* bytes)
{
    uint32_t current = maniLoadU32(bytes);
    uint32_t running = maniLoadU32(bytes + 4);
    float elapsed = FloatOf(maniLoadU32(bytes + 4 * maniGraphBlockElapsed));
    double time = SavedDouble(bytes, maniGraphBlockTime);
    if (current >= m->states)
    {
        return maniGraphBlockCurrent;
    }
    if (running != MANI_GRAPH_NO_TRANSITION &&
        (running >= maniGraphMachineCount(m) ||
         maniGraphMachineTransition(m, running, 1) != current))
    {
        return maniGraphBlockRunning;
    }
    if (!(isfinite(elapsed) && elapsed >= 0.0f))
    {
        return maniGraphBlockElapsed;
    }
    if (!(isfinite(time) && time >= 0.0))
    {
        return maniGraphBlockTime;
    }
    return RecordValid(bytes);
}

uint32_t maniGraphSavedBlockValid(const maniGraphMachine* m, const uint8_t* bytes)
{
    uint32_t wrong = HeaderValid(m, bytes);
    uint32_t weights = MANI_GRAPH_BLOCK_WORDS + 2 * m->states;
    for (uint32_t k = MANI_GRAPH_BLOCK_WORDS; k < weights && wrong == UINT32_MAX; ++k)
    {
        wrong = UnitWeight(maniLoadU32(bytes + 4 * (size_t)k)) ? UINT32_MAX : k;
    }
    for (uint32_t t = 0; t < maniGraphMachineCount(m) && wrong == UINT32_MAX; ++t)
    {
        float since = FloatOf(maniLoadU32(bytes + 4 * (size_t)(weights + t)));
        wrong = since >= 0.0f ? UINT32_MAX : weights + t;
    }
    // The bits past the last condition stay clear.
    uint32_t conditions = maniGraphMachineConditionCount(m);
    uint32_t last = HeardAt(m, conditions - (conditions > 0 ? 1u : 0u));
    uint32_t spare = conditions % 32;
    if (wrong == UINT32_MAX && spare != 0 && (maniLoadU32(bytes + 4 * (size_t)last) >> spare) != 0)
    {
        wrong = last;
    }
    return wrong;
}
