// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a graph's nodes (mani-0023): each record, its inputs (every
// one before it, in another slot) and its data, by kind.

#ifndef MAUL_ANIM_SRC_GRAPH_NODES_H
#define MAUL_ANIM_SRC_GRAPH_NODES_H

#include "maul-anim/base.h"
#include "maul-anim/graph.h"

#include <stdint.h>

// Checks the nodes, inputs and data of a graph whose other sections are
// checked; the offsets are the sections' in the file. Gives the words of
// every state machine's block in an instance.
maniFileResult maniCheckGraphNodes(const maniGraph* g, uint64_t nodesAt, uint64_t inputsAt,
                                   uint64_t dataAt, uint32_t* machineWordsOut);

#endif // MAUL_ANIM_SRC_GRAPH_NODES_H
