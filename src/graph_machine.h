// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// State machines' words (mani-0023): a machine's data words read, and its
// block in an instance (current state, running transition, fade and
// state clocks, the record of its leading clip, per state weights, per
// transition timers, per condition heard bits) read, written, reset to
// its entry state and checked when a saved state restores it.

#ifndef MAUL_ANIM_SRC_GRAPH_MACHINE_H
#define MAUL_ANIM_SRC_GRAPH_MACHINE_H

#include "maul-anim/graph.h"

#include <stdbool.h>
#include <stdint.h>

// The machine whose data words begin at data in a graph, of states
// inputs.
typedef struct maniGraphMachine
{
    const maniGraph* graph;
    uint32_t data;
    uint32_t states;
} maniGraphMachine;

// Its data word at, as a word and as a binary32.
uint32_t maniGraphMachineWord(const maniGraphMachine* m, uint32_t at);
float maniGraphMachineFloat(const maniGraphMachine* m, uint32_t at);

// Its transition count, and transition t's word k (t in priority order).
uint32_t maniGraphMachineCount(const maniGraphMachine* m);
uint32_t maniGraphMachineTransition(const maniGraphMachine* m, uint32_t t, uint32_t k);
// Its condition count, and condition c's first data word.
uint32_t maniGraphMachineConditionCount(const maniGraphMachine* m);
uint32_t maniGraphMachineCondition(const maniGraphMachine* m, uint32_t c);
// Whether it listens (records its leading clip), and state k's word w:
// final, nearest machine, list's first, list's count.
bool maniGraphMachineListening(const maniGraphMachine* m);
uint32_t maniGraphMachineState(const maniGraphMachine* m, uint32_t k, uint32_t w);
// State k's i-th clip node.
uint32_t maniGraphMachineClip(const maniGraphMachine* m, uint32_t k, uint32_t i);

// Its block's first word in an instance's machine state, and its words.
uint32_t maniGraphMachineBlockAt(const maniGraphMachine* m);
uint32_t maniGraphMachineBlockWords(const maniGraphMachine* m);

// A block's first words; the binary64 ones take two.
enum
{
    maniGraphBlockCurrent = 0,
    maniGraphBlockRunning = 1,
    maniGraphBlockElapsed = 2,
    maniGraphBlockTime = 3,
    maniGraphBlockBefore = 5,
    maniGraphBlockPosition = 7,
    maniGraphBlockWraps = 9,
    maniGraphBlockLoops = 11,
    maniGraphBlockFinished = 13,
};

float maniGraphBlockFloat(const uint32_t* block, uint32_t at);
void maniGraphSetBlockFloat(uint32_t* block, uint32_t at, float value);
double maniGraphBlockDouble(const uint32_t* block, uint32_t at);
void maniGraphSetBlockDouble(uint32_t* block, uint32_t at, double value);

// State k's weight, and its weight when the running fade began.
float maniGraphMachineWeight(const uint32_t* block, uint32_t k);
void maniGraphSetMachineWeight(uint32_t* block, uint32_t k, float weight);
float maniGraphMachineStartWeight(const uint32_t* block, uint32_t k);
// Every state's weight kept as its fade-start weight.
void maniGraphKeepStartWeights(const maniGraphMachine* m, uint32_t* block);

// Transition t's seconds since it fired.
float maniGraphMachineSince(const maniGraphMachine* m, const uint32_t* block, uint32_t t);
void maniGraphSetMachineSince(const maniGraphMachine* m, uint32_t* block, uint32_t t,
                              float seconds);

// Condition c's heard bit.
bool maniGraphMachineHeard(const maniGraphMachine* m, const uint32_t* block, uint32_t c);
void maniGraphSetMachineHeard(const maniGraphMachine* m, uint32_t* block, uint32_t c);

// The record of the update before cleared but the position: no move, no
// loops, not finished, nothing heard.
void maniGraphClearRecord(const maniGraphMachine* m, uint32_t* block);

// The record's move in loops: whole wraps plus the position's change.
double maniGraphRecordMove(const uint32_t* block);

// Its entry state at weight 1, no transition running, its clocks and
// record at 0 and every transition never fired.
void maniGraphResetMachine(const maniGraphMachine* m, uint32_t* block);

// The first wrong word of a saved block (little-endian words at bytes),
// or UINT32_MAX.
uint32_t maniGraphSavedBlockValid(const maniGraphMachine* m, const uint8_t* bytes);

#endif // MAUL_ANIM_SRC_GRAPH_MACHINE_H
