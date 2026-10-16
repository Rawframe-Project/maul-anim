// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A block's words: current state, running transition, elapsed fade
// seconds (binary32), seconds in state (binary64, low word first), zero;
// then per state its weight and fade-start weight; then per transition
// its seconds since it fired (+inf before it has).

#include "graph_machine.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/graph.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    kElapsed = 2,
    kTime = 3,
    kZero = 5,
};

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

uint32_t maniGraphMachineBlockAt(const maniGraphMachine* m)
{
    return maniGraphMachineWord(m, 3);
}

uint32_t maniGraphMachineBlockWords(const maniGraphMachine* m)
{
    return MANI_GRAPH_BLOCK_SIZE(m->states, maniGraphMachineCount(m));
}

float maniGraphBlockElapsed(const uint32_t* block)
{
    return FloatOf(block[kElapsed]);
}

void maniGraphSetBlockElapsed(uint32_t* block, float seconds)
{
    block[kElapsed] = BitsOf(seconds);
}

double maniGraphBlockTime(const uint32_t* block)
{
    uint64_t bits = (uint64_t)block[kTime] + ((uint64_t)block[kTime + 1] << 32);
    double value = 0.0;
    memcpy(&value, &bits, sizeof value);
    return value;
}

void maniGraphSetBlockTime(uint32_t* block, double seconds)
{
    uint64_t bits = 0;
    memcpy(&bits, &seconds, sizeof bits);
    block[kTime] = (uint32_t)bits;
    block[kTime + 1] = (uint32_t)(bits >> 32);
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

void maniGraphResetMachine(const maniGraphMachine* m, uint32_t* block)
{
    uint32_t entry = maniGraphMachineWord(m, 0);
    block[maniGraphBlockCurrent] = entry;
    block[maniGraphBlockRunning] = MANI_GRAPH_NO_TRANSITION;
    maniGraphSetBlockElapsed(block, 0.0f);
    maniGraphSetBlockTime(block, 0.0);
    block[kZero] = 0;
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

// The saved header's words: a state, a running transition into it or
// none, a clock and a time not negative, the zero word.
static uint32_t HeaderValid(const maniGraphMachine* m, const uint8_t* bytes)
{
    uint32_t current = maniLoadU32(bytes);
    uint32_t running = maniLoadU32(bytes + 4);
    float elapsed = FloatOf(maniLoadU32(bytes + 8));
    uint64_t timeBits = maniLoadU64(bytes + 12);
    double time = 0.0;
    memcpy(&time, &timeBits, sizeof time);
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
        return kElapsed;
    }
    if (!(isfinite(time) && time >= 0.0))
    {
        return kTime;
    }
    return maniLoadU32(bytes + 4 * kZero) == 0 ? UINT32_MAX : (uint32_t)kZero;
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
    return wrong;
}
