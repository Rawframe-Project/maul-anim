// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion matching nodes' words: a node's data words read and
// checked on open, and its block in an instance (its entry, the period's
// clock, its picks) reset and checked when a saved state restores it.

#ifndef MAUL_ANIM_SRC_GRAPH_MATCH_H
#define MAUL_ANIM_SRC_GRAPH_MATCH_H

#include "maul-anim/graph.h"
#include "maul-anim/match.h"

#include <stdbool.h>
#include <stdint.h>

// A node's data words (graph_file.h).
enum
{
    maniGraphMatchDatabase = 0,
    maniGraphMatchTrigger = 1,
    maniGraphMatchPeriod = 2,
    maniGraphMatchJumpThreshold = 3,
    maniGraphMatchEndExclusion = 4,
    maniGraphMatchHistoryTime = 5,
    maniGraphMatchContinuingBias = 6,
    maniGraphMatchTransitionCost = 7,
    maniGraphMatchHistoryCount = 8,
    maniGraphMatchRequiredTags = 9,
    maniGraphMatchExcludedTags = 11,
    maniGraphMatchDurationScalar = 13,
    maniGraphMatchBlock = 15,
    maniGraphMatchQueryCount = 16,
    maniGraphMatchQuery = 17,
};

// A block: the entry playing (MANI_GRAPH_MATCH_NO_ENTRY for none), the
// seconds since the last search (binary32), the picks kept (at most the
// history count) and the place the next is kept at (below the history
// count, 0 without one), then the picks (frames; 0 where none is kept).
enum
{
    maniGraphMatchEntry = 0,
    maniGraphMatchClock = 1,
    maniGraphMatchPicks = 2,
    maniGraphMatchNext = 3,
    maniGraphMatchHistory = 4,
};

#define MANI_GRAPH_MATCH_BLOCK_WORDS (4u + MANI_MATCH_MAX_HISTORY)
#define MANI_GRAPH_MATCH_NO_ENTRY    0xFFFFFFFFu

// The data words of the node whose words begin at data, as its query
// count gives them; 0 when they would pass available words.
uint32_t maniGraphMatchDataWords(const maniGraph* g, uint32_t data, uint32_t available);

// The first wrong data word of the node whose words begin at data and
// whose block starts at block, or UINT32_MAX.
uint32_t maniGraphMatchValid(const maniGraph* g, uint32_t data, uint32_t block);

// No entry, the clock at 0, no picks.
void maniGraphResetMatch(uint32_t* block);

// The first wrong word of a saved block (little-endian words at bytes) of
// the node whose words begin at data, searching database, whose saved
// playhead is time; or UINT32_MAX.
uint32_t maniGraphSavedMatchValid(const uint8_t* bytes, const maniGraph* g, uint32_t data,
                                  const maniMotionDatabase* database, double time);

// A list of channels: a database's or a learned model's (its database's).
typedef maniResult maniGraphChannelFn(const void* owner, uint32_t channel,
                                      maniMatchChannel* channelOut);

// maniMotionDatabaseChannel, and maniLearnedModelChannel, as such.
maniResult maniGraphDatabaseChannel(const void* database, uint32_t channel,
                                    maniMatchChannel* channelOut);
maniResult maniGraphModelChannel(const void* model, uint32_t channel, maniMatchChannel* channelOut);

// Whether channelCount channels take a node's query: per trajectory and
// float time in channel order, a parameter of the type given (vec2 or
// float). types holds the query's parameters' types.
bool maniGraphQueryFits(const void* owner, uint32_t channelCount, maniGraphChannelFn* channel,
                        const uint32_t* types, uint32_t count);

#endif // MAUL_ANIM_SRC_GRAPH_MATCH_H
