// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node's words name a model slot, a trigger or none, a period above 0
// and a transition cost not negative (finite), tags from an int parameter
// (its literal 0) or literal, a duration as an inertialize node's, its
// model's counts (a state of 2 to MANI_GRAPH_LEARNED_MAX_STATES values,
// at most MANI_MATCH_MAX_FEATURES features with a latent value at least,
// at most MANI_LEARNED_MAX_EVENTS events) and a vec2 or float parameter
// per query; binding checks the counts and query against the model. A
// saved block has a state or none (then every word 0), its clocks finite
// and not negative (the step's below a step's seconds), its states
// finite, its bits past the events 0.

#include "graph_learned.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_match.h"

#include "maul-anim/graph.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"

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

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static uint32_t ParameterType(const maniGraph* g, uint32_t parameter)
{
    return parameter < g->parameterCount
               ? maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE)
               : UINT32_MAX;
}

uint32_t maniGraphLearnedDataWords(const maniGraph* g, uint32_t data, uint32_t available)
{
    if (available < MANI_GRAPH_LEARNED_WORDS)
    {
        return 0;
    }
    uint32_t queries = Word(g, data + maniGraphLearnedQueryCount);
    return queries <= MANI_MATCH_MAX_FEATURES && queries <= available - MANI_GRAPH_LEARNED_WORDS
               ? MANI_GRAPH_LEARNED_WORDS + queries
               : 0;
}

uint32_t maniGraphLearnedBlockWords(const maniGraph* g, uint32_t data)
{
    return MANI_GRAPH_LEARNED_BLOCK_WORDS(Word(g, data + maniGraphLearnedStates),
                                          Word(g, data + maniGraphLearnedEvents));
}

// The slot, trigger, period and cost; the first wrong, or UINT32_MAX.
static uint32_t HeadValid(const maniGraph* g, uint32_t data)
{
    uint32_t trigger = Word(g, data + maniGraphLearnedTrigger);
    float period = FloatOf(Word(g, data + maniGraphLearnedPeriod));
    float cost = FloatOf(Word(g, data + maniGraphLearnedTransitionCost));
    if (Word(g, data + maniGraphLearnedModel) >= g->modelCount)
    {
        return data + maniGraphLearnedModel;
    }
    if (trigger != MANI_NO_PARAMETER && ParameterType(g, trigger) != maniGraphParameterTrigger)
    {
        return data + maniGraphLearnedTrigger;
    }
    if (!(isfinite(period) && period > 0.0f))
    {
        return data + maniGraphLearnedPeriod;
    }
    return isfinite(cost) && cost >= 0.0f ? UINT32_MAX : data + maniGraphLearnedTransitionCost;
}

// Tags: an int parameter and literal 0, or no parameter and any bits.
static bool TagsValid(const maniGraph* g, uint32_t at)
{
    uint32_t parameter = Word(g, at);
    return parameter == MANI_NO_PARAMETER ||
           (ParameterType(g, parameter) == maniGraphParameterInt && Word(g, at + 1) == 0);
}

// The tags, duration, block and counts; the first wrong, or UINT32_MAX.
static uint32_t MiddleValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    for (uint32_t k = maniGraphLearnedRequiredTags; k <= maniGraphLearnedExcludedTags; k += 2)
    {
        if (!TagsValid(g, data + k))
        {
            return data + k;
        }
    }
    uint32_t parameter = Word(g, data + maniGraphLearnedDurationScalar);
    uint32_t bits = Word(g, data + maniGraphLearnedDurationScalar + 1);
    bool literal =
        parameter == MANI_NO_PARAMETER && isfinite(FloatOf(bits)) && FloatOf(bits) >= 0.0f;
    if (!(literal || (ParameterType(g, parameter) == maniGraphParameterFloat && bits == 0)))
    {
        return data + maniGraphLearnedDurationScalar;
    }
    if (Word(g, data + maniGraphLearnedBlock) != block)
    {
        return data + maniGraphLearnedBlock;
    }
    uint32_t states = Word(g, data + maniGraphLearnedStates);
    uint32_t features = Word(g, data + maniGraphLearnedFeatures);
    if (states < 2 || states > MANI_GRAPH_LEARNED_MAX_STATES)
    {
        return data + maniGraphLearnedStates;
    }
    if (features < 1 || features > MANI_MATCH_MAX_FEATURES || features >= states ||
        states - features > MANI_LEARNED_MAX_LATENT)
    {
        return data + maniGraphLearnedFeatures;
    }
    return Word(g, data + maniGraphLearnedEvents) <= MANI_LEARNED_MAX_EVENTS
               ? UINT32_MAX
               : data + maniGraphLearnedEvents;
}

uint32_t maniGraphLearnedValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    uint32_t wrong = HeadValid(g, data);
    wrong = wrong == UINT32_MAX ? MiddleValid(g, data, block) : wrong;
    uint32_t queries = Word(g, data + maniGraphLearnedQueryCount);
    for (uint32_t q = 0; wrong == UINT32_MAX && q < queries; ++q)
    {
        uint32_t type = ParameterType(g, Word(g, data + maniGraphLearnedQuery + q));
        wrong = type == maniGraphParameterVec2 || type == maniGraphParameterFloat
                    ? UINT32_MAX
                    : data + maniGraphLearnedQuery + q;
    }
    return wrong;
}

bool maniGraphLearnedFits(const maniGraph* g, uint32_t data, const maniLearnedModel* model)
{
    if (Word(g, data + maniGraphLearnedStates) != model->featureCount + model->latentCount ||
        Word(g, data + maniGraphLearnedFeatures) != model->featureCount ||
        Word(g, data + maniGraphLearnedEvents) != model->eventCount)
    {
        return false;
    }
    uint32_t types[MANI_MATCH_MAX_FEATURES];
    uint32_t queries = Word(g, data + maniGraphLearnedQueryCount);
    for (uint32_t q = 0; q < queries; ++q)
    {
        types[q] = ParameterType(g, Word(g, data + maniGraphLearnedQuery + q));
    }
    return maniGraphQueryFits(model, model->channelCount, maniGraphModelChannel, types, queries);
}

void maniGraphResetLearned(const maniGraph* g, uint32_t data, uint32_t* block)
{
    memset(block, 0, sizeof(uint32_t) * maniGraphLearnedBlockWords(g, data));
}

static double DoubleOf(const uint8_t* bytes)
{
    uint64_t bits = maniLoadU64(bytes);
    double value = 0.0;
    memcpy(&value, &bits, sizeof value);
    return value;
}

uint32_t maniGraphSavedLearnedValid(const uint8_t* bytes, const maniGraph* g, uint32_t data,
                                    const maniLearnedModel* model)
{
    uint32_t words = maniGraphLearnedBlockWords(g, data);
    uint32_t has = maniLoadU32(bytes);
    if (has > 1)
    {
        return maniGraphLearnedHas;
    }
    uint32_t states = Word(g, data + maniGraphLearnedStates);
    uint32_t events = Word(g, data + maniGraphLearnedEvents);
    double clock = DoubleOf(bytes + 4 * maniGraphLearnedClock);
    float since = FloatOf(maniLoadU32(bytes + 4 * maniGraphLearnedSince));
    if (!(clock >= 0.0 && clock < 1.0 / (double)model->stepRate))
    {
        return maniGraphLearnedClock;
    }
    if (!(isfinite(since) && since >= 0.0f))
    {
        return maniGraphLearnedSince;
    }
    for (uint32_t k = maniGraphLearnedBefore; k < words; ++k)
    {
        uint32_t w = maniLoadU32(bytes + 4 * (size_t)k);
        bool state = k < maniGraphLearnedBefore + 2 * states;
        uint32_t bit = k - maniGraphLearnedBefore - 2 * states;
        uint32_t past = state || 32 * (bit + 1) <= events ? 0u : ~0u << (events - 32 * bit);
        bool ok = state ? isfinite(FloatOf(w)) : (w & past) == 0;
        if (!ok || (has == 0 && w != 0))
        {
            return k;
        }
    }
    if (has == 0 && clock != 0.0)
    {
        return maniGraphLearnedClock;
    }
    return has == 0 && since != 0.0f ? maniGraphLearnedSince : UINT32_MAX;
}

void maniGraphLearnedState(const uint32_t* block, uint32_t states, float rate, float* out)
{
    uint64_t bits =
        (uint64_t)block[maniGraphLearnedClock] | (uint64_t)block[maniGraphLearnedClock + 1] << 32;
    double clock = 0.0;
    memcpy(&clock, &bits, sizeof clock);
    float fraction = (float)(clock * (double)rate);
    const uint32_t* before = block + maniGraphLearnedBefore;
    for (uint32_t k = 0; k < states; ++k)
    {
        float b = FloatOf(before[k]);
        float a = FloatOf(before[states + k]);
        out[k] = b + fraction * (a - b);
    }
}
