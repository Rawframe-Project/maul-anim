// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A state machine node's source checked and written (mani-0023): its
// entry, chain limit, transitions (states in range, never to their own
// source, priorities unique among each state's candidates) and their
// conditions (parameters of the types they compare). Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_MACHINE_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_MACHINE_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>

maniBuildResult maniCheckGraphMachine(const maniGraphSource* source, const maniGraphNodeSource* n,
                                      uint32_t node);

// The data words a checked machine node takes.
uint32_t maniGraphMachineSourceWords(const maniGraphNodeSource* n);

// Writes a checked machine node's words at data word at of data, its
// block at word block of an instance's machine state; returns the word
// after them. order (transitionCount) and scratch (as many) are working
// memory.
uint32_t maniPutGraphMachine(uint8_t* data, uint32_t at, const maniGraphNodeSource* n,
                             uint32_t block, uint32_t* order, uint32_t* scratch, uint64_t* keys);

#endif // MAUL_ANIM_SRC_GRAPH_MACHINE_CHECK_H
