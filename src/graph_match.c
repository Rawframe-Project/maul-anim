// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node's words name a database slot, a trigger or none, finite costs,
// seconds not negative (the period above 0), at most
// MANI_MAX_MATCH_HISTORY picks, tags from an int parameter (its literal
// 0) or literal, a duration as an inertialize node's, and a vec2 or
// float parameter per query; binding checks the query against the
// database. A saved block's entry is the database's or none, its clock
// finite and not negative, its picks frames of the database, the unused
// ones 0, and the node's playhead within its entry.

#include "graph_match.h"

#include "bytes.h"
#include "graph_file.h"

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

uint32_t maniGraphMatchDataWords(const maniGraph* g, uint32_t data, uint32_t available)
{
    if (available < MANI_GRAPH_MATCH_WORDS)
    {
        return 0;
    }
    uint32_t queries = Word(g, data + maniGraphMatchQueryCount);
    return queries <= MANI_MAX_MATCH_FEATURES && queries <= available - MANI_GRAPH_MATCH_WORDS
               ? MANI_GRAPH_MATCH_WORDS + queries
               : 0;
}

// The times of a node: the period above 0, the others not negative, the
// costs finite; the first wrong, or UINT32_MAX.
static uint32_t NumbersValid(const maniGraph* g, uint32_t data)
{
    for (uint32_t k = maniGraphMatchPeriod; k <= maniGraphMatchTransitionCost; ++k)
    {
        float v = FloatOf(Word(g, data + k));
        bool cost = k >= maniGraphMatchContinuingBias;
        bool ok = isfinite(v) && (cost || v >= 0.0f) && (k != maniGraphMatchPeriod || v > 0.0f);
        if (!ok)
        {
            return data + k;
        }
    }
    return UINT32_MAX;
}

// Tags: an int parameter and literal 0, or no parameter and any bits.
static bool TagsValid(const maniGraph* g, uint32_t at)
{
    uint32_t parameter = Word(g, at);
    return parameter == MANI_NO_PARAMETER ||
           (ParameterType(g, parameter) == maniGraphParameterInt && Word(g, at + 1) == 0);
}

// The fields before the numbers: the slot and trigger.
static uint32_t SlotsValid(const maniGraph* g, uint32_t data)
{
    uint32_t trigger = Word(g, data + maniGraphMatchTrigger);
    if (Word(g, data + maniGraphMatchDatabase) >= g->databaseCount)
    {
        return data + maniGraphMatchDatabase;
    }
    bool triggered =
        trigger == MANI_NO_PARAMETER || ParameterType(g, trigger) == maniGraphParameterTrigger;
    return triggered ? UINT32_MAX : data + maniGraphMatchTrigger;
}

// The fields after the numbers: the history count, the tags, the
// duration, the block and each query.
static uint32_t RestValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    if (Word(g, data + maniGraphMatchHistoryCount) > MANI_MAX_MATCH_HISTORY)
    {
        return data + maniGraphMatchHistoryCount;
    }
    for (uint32_t k = maniGraphMatchRequiredTags; k <= maniGraphMatchExcludedTags; k += 2)
    {
        if (!TagsValid(g, data + k))
        {
            return data + k;
        }
    }
    uint32_t parameter = Word(g, data + maniGraphMatchDurationScalar);
    float value = FloatOf(Word(g, data + maniGraphMatchDurationScalar + 1));
    bool literal = parameter == MANI_NO_PARAMETER && isfinite(value) && value >= 0.0f;
    if (!(literal || (ParameterType(g, parameter) == maniGraphParameterFloat &&
                      Word(g, data + maniGraphMatchDurationScalar + 1) == 0)))
    {
        return data + maniGraphMatchDurationScalar;
    }
    if (Word(g, data + maniGraphMatchBlock) != block)
    {
        return data + maniGraphMatchBlock;
    }
    uint32_t queries = Word(g, data + maniGraphMatchQueryCount);
    for (uint32_t q = 0; q < queries; ++q)
    {
        uint32_t type = ParameterType(g, Word(g, data + maniGraphMatchQuery + q));
        if (type != maniGraphParameterVec2 && type != maniGraphParameterFloat)
        {
            return data + maniGraphMatchQuery + q;
        }
    }
    return UINT32_MAX;
}

uint32_t maniGraphMatchValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    uint32_t wrong = SlotsValid(g, data);
    wrong = wrong == UINT32_MAX ? NumbersValid(g, data) : wrong;
    return wrong == UINT32_MAX ? RestValid(g, data, block) : wrong;
}

void maniGraphResetMatch(uint32_t* block)
{
    memset(block, 0, MANI_GRAPH_MATCH_BLOCK_WORDS * sizeof(uint32_t));
    block[maniGraphMatchEntry] = MANI_GRAPH_MATCH_NO_ENTRY;
}

// The playhead within the entry's span.
static bool TimeValid(const maniMotionDatabase* database, uint32_t entry, double time)
{
    maniMatchEntry e;
    return maniMotionDatabase_GetEntry(database, entry, &e) == mani_success &&
           time >= (double)e.start && time <= (double)e.end;
}

uint32_t maniGraphSavedMatchValid(const uint8_t* bytes, const maniGraph* g, uint32_t data,
                                  const maniMotionDatabase* database, double time)
{
    uint32_t entry = maniLoadU32(bytes + 4 * maniGraphMatchEntry);
    float clock = FloatOf(maniLoadU32(bytes + 4 * maniGraphMatchClock));
    uint32_t picks = maniLoadU32(bytes + 4 * maniGraphMatchPicks);
    uint32_t next = maniLoadU32(bytes + 4 * maniGraphMatchNext);
    uint32_t history = Word(g, data + maniGraphMatchHistoryCount);
    if (entry != MANI_GRAPH_MATCH_NO_ENTRY && !TimeValid(database, entry, time))
    {
        return maniGraphMatchEntry;
    }
    if (!(isfinite(clock) && clock >= 0.0f))
    {
        return maniGraphMatchClock;
    }
    if (picks > history)
    {
        return maniGraphMatchPicks;
    }
    // Kept in order until the history fills, then over the oldest.
    if (history > 0 ? next >= history || (picks < history && next != picks) : next != 0)
    {
        return maniGraphMatchNext;
    }
    for (uint32_t k = 0; k < MANI_MAX_MATCH_HISTORY; ++k)
    {
        uint32_t frame = maniLoadU32(bytes + 4 * (size_t)(maniGraphMatchHistory + k));
        if (k < picks ? frame >= database->frameCount : frame != 0)
        {
            return maniGraphMatchHistory + k;
        }
    }
    return UINT32_MAX;
}

maniResult maniGraphDatabaseChannel(const void* database, uint32_t channel,
                                    maniMatchChannel* channelOut)
{
    return maniMotionDatabase_GetChannel(database, channel, channelOut);
}

maniResult maniGraphModelChannel(const void* model, uint32_t channel, maniMatchChannel* channelOut)
{
    return maniLearnedModel_GetChannel(model, channel, channelOut);
}

bool maniGraphQueryFits(const void* owner, uint32_t channelCount, maniGraphChannelFn* channel,
                        const uint32_t* types, uint32_t count)
{
    uint32_t q = 0;
    for (uint32_t c = 0; c < channelCount; ++c)
    {
        maniMatchChannel ch;
        if (channel(owner, c, &ch) != mani_success)
        {
            return false;
        }
        bool vector =
            ch.kind == maniMatchTrajectoryPosition || ch.kind == maniMatchTrajectoryFacing;
        if (!vector && ch.kind != maniMatchFloatTrack)
        {
            continue;
        }
        uint32_t type = vector ? maniGraphParameterVec2 : maniGraphParameterFloat;
        for (uint32_t t = 0; t < ch.timeCount; ++t, ++q)
        {
            if (q >= count || types[q] != type)
            {
                return false;
            }
        }
    }
    return q == count;
}
