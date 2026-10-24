// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Learned motion matching nodes' words: a node's data words
// read and checked on open and against its model at binding, its block
// in an instance (whether it has a state, the clocks, the states before
// and after the last step, a bit per event) reset and checked when a
// saved state restores it, and the state it plays.

#ifndef MAUL_ANIM_SRC_GRAPH_LEARNED_H
#define MAUL_ANIM_SRC_GRAPH_LEARNED_H

#include "maul-anim/graph.h"
#include "maul-anim/learned.h"
#include "maul-anim/match.h"

#include <stdbool.h>
#include <stdint.h>

// A node's data words (graph_file.h): its model slot, trigger, period
// and transition cost (binary32), tags (parameter, literal: two words
// each), duration (a scalar), its block's first word, its model's state,
// feature and event counts, its query count, then the query.
enum
{
    maniGraphLearnedModel = 0,
    maniGraphLearnedTrigger = 1,
    maniGraphLearnedPeriod = 2,
    maniGraphLearnedTransitionCost = 3,
    maniGraphLearnedRequiredTags = 4,
    maniGraphLearnedExcludedTags = 6,
    maniGraphLearnedDurationScalar = 8,
    maniGraphLearnedBlock = 10,
    maniGraphLearnedStates = 11,
    maniGraphLearnedFeatures = 12,
    maniGraphLearnedEvents = 13,
    maniGraphLearnedQueryCount = 14,
    maniGraphLearnedQuery = 15,
};

#define MANI_GRAPH_LEARNED_WORDS 15u

// The most values a state holds: a database's features and a model's
// latent values.
#define MANI_GRAPH_LEARNED_MAX_STATES (MANI_MATCH_MAX_FEATURES + MANI_LEARNED_MAX_LATENT)

// A block: whether the node has a state (0 or 1), the seconds since the
// last step (binary64, low word first, below a step's seconds), the
// seconds since the last projection (binary32), the state before the last
// step and after it, then a bit per event (low bit first), set when its
// state was above 0.5 at the last update.
enum
{
    maniGraphLearnedHas = 0,
    maniGraphLearnedClock = 1,
    maniGraphLearnedSince = 3,
    maniGraphLearnedBefore = 4,
};

#define MANI_GRAPH_LEARNED_BLOCK_WORDS(states, events) (4u + 2u * (states) + ((events) + 31u) / 32u)

// The data words of the node whose words begin at data, as its query
// count gives them; 0 when they would pass available words.
uint32_t maniGraphLearnedDataWords(const maniGraph* g, uint32_t data, uint32_t available);

// The first wrong data word of the node whose words begin at data and
// whose block starts at block, or UINT32_MAX.
uint32_t maniGraphLearnedValid(const maniGraph* g, uint32_t data, uint32_t block);

// The words of the node's block.
uint32_t maniGraphLearnedBlockWords(const maniGraph* g, uint32_t data);

// Whether a model takes the node: its counts the node's, and its
// database's channels the node's query.
bool maniGraphLearnedFits(const maniGraph* g, uint32_t data, const maniLearnedModel* model);

// No state, the clocks at 0, no events heard.
void maniGraphResetLearned(const maniGraph* g, uint32_t data, uint32_t* block);

// The first wrong word of a saved block (little-endian words at bytes)
// of the node whose words begin at data, playing model; or UINT32_MAX.
uint32_t maniGraphSavedLearnedValid(const uint8_t* bytes, const maniGraph* g, uint32_t data,
                                    const maniLearnedModel* model);

// The state the node plays: the state before the last step plus the
// fraction of a step passed times the change, value by value.
void maniGraphLearnedState(const uint32_t* block, uint32_t states, float rate, float* out);

#endif // MAUL_ANIM_SRC_GRAPH_LEARNED_H
