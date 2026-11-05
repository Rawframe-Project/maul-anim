// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a graph's nodes: each record, its inputs (every
// one before it, in another slot) and its data, by kind.

#ifndef MAUL_ANIM_SRC_GRAPH_NODES_H
#define MAUL_ANIM_SRC_GRAPH_NODES_H

#include "maul-anim/base.h"
#include "maul-anim/graph.h"

#include <stdint.h>

// The words of the block a node of the kind, with count inputs and its
// data at data, keeps in an instance (0 for a kind without one), in 64
// bits so that no size wraps. Read at run time too, so every data word
// it reads is bounded (maniGraphWord).
uint64_t maniGraphBlockWords(const maniGraph* g, uint32_t kind, uint32_t count, uint32_t data);

// Checks the nodes, inputs and data of a graph whose other sections are
// checked; the offsets are the sections' in the file. Gives the words of
// every state machine's block in an instance.
maniFileResult maniCheckGraphNodes(const maniGraph* g, uint64_t nodesAt, uint64_t inputsAt,
                                   uint64_t dataAt, uint32_t* machineWordsOut);

#endif // MAUL_ANIM_SRC_GRAPH_NODES_H
