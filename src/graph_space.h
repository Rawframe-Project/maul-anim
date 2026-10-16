// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blend spaces' weights (mani-0023): where a position falls among a
// space's points, as the update shares weights and the evaluation blends
// by them, the same binary32 operations for both.

#ifndef MAUL_ANIM_SRC_GRAPH_SPACE_H
#define MAUL_ANIM_SRC_GRAPH_SPACE_H

#include "graph_instance.h"

#include "maul-anim/graph.h"

#include <stdint.h>

// The weights of a blend space node (its kind, first data word and input
// count) at the parameters' values in cells: count of them, each 0 to 1,
// summing to 1 but for rounding.
void maniGraphSpaceWeights(const maniGraph* graph, const maniGraphCell* cells, uint32_t kind,
                           uint32_t data, uint32_t count, float* weights);

#endif // MAUL_ANIM_SRC_GRAPH_SPACE_H
