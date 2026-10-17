// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A motion matching node's step in an update (mani-0024): its playhead
// moved through its entry, its events listed and its root motion added,
// then a search when one is due, a jump to the pick and a request of
// inertialization.

#ifndef MAUL_ANIM_SRC_GRAPH_MATCH_STEP_H
#define MAUL_ANIM_SRC_GRAPH_MATCH_STEP_H

#include "graph_emit.h"

#include <stdint.h>

// Moves motion matching node p. One no parent reached, or reached with
// no weight, stays where it is.
void maniGraphAdvanceMatch(maniGraphUpdate* u, uint32_t p);

#endif // MAUL_ANIM_SRC_GRAPH_MATCH_STEP_H
