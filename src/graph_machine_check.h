// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A state machine node's source checked and written (mani-0023): its
// entry, chain limit, final states, transitions (states in range, never
// to their own source, priorities unique among each state's candidates)
// and their conditions (parameters of the types they compare); the clip
// nodes beneath each state of a machine that listens. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_MACHINE_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_MACHINE_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

maniBuildResult maniCheckGraphMachine(const maniGraphSource* source, const maniGraphNodeSource* n,
                                      uint32_t node);

// Whether a machine listens: a transition with a phase, finished or
// event condition or a match entry.
bool maniGraphMachineListens(const maniGraphNodeSource* n);

// Its conditions, and the data words it takes before its clip lists.
uint32_t maniGraphMachineConditions(const maniGraphNodeSource* n);
uint32_t maniGraphMachineSourceWords(const maniGraphNodeSource* n);

// A search below a node: the source's nodes, their places in the order
// (NULL to count in source order), a stamp per node (0 before the first
// search) and a stack of as many.
typedef struct maniGraphReach
{
    const maniGraphSource* source;
    const uint32_t* position;
    uint32_t* stamp;
    uint32_t* stack;
    uint32_t mark;
    uint32_t reserved;
} maniGraphReach;

// The clip nodes from a node down (it included): their count, their
// places into clipsOut (unordered) and the greatest place of a machine
// node among them (MANI_GRAPH_NO_NODE for none) into nearestOut, each
// when not NULL.
uint32_t maniGraphReachClips(maniGraphReach* r, uint32_t from, uint32_t* clipsOut,
                             uint32_t* nearestOut);

// Writing a machine: sort keys, order and scratch for its transitions
// and for one state's clips, a buffer of as many clips, and the search.
typedef struct maniGraphMachineWork
{
    uint64_t* keys;
    uint32_t* order;
    uint32_t* scratch;
    uint32_t* clips;
    maniGraphReach* reach;
} maniGraphMachineWork;

// The clip nodes from a node down, as maniGraphReachClips finds them,
// their places in work's clips taken in node order by work's order.
uint32_t maniGraphSortedClips(maniGraphMachineWork* work, uint32_t from, uint32_t* nearestOut);

// Writes a checked machine node's words at data word at of data, its
// block at word block of an instance's machine state; returns the word
// after them.
uint32_t maniPutGraphMachine(uint8_t* data, uint32_t at, const maniGraphNodeSource* n,
                             uint32_t block, maniGraphMachineWork* work);

#endif // MAUL_ANIM_SRC_GRAPH_MACHINE_CHECK_H
