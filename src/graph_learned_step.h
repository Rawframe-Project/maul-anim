// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned motion matching node's update: its clock moved
// and its state stepped, a projection when one is due, its root motion
// added and its events listed.

#ifndef MAUL_ANIM_SRC_GRAPH_LEARNED_STEP_H
#define MAUL_ANIM_SRC_GRAPH_LEARNED_STEP_H

#include "graph_emit.h"

#include <stdint.h>

// Moves learned node p. One no parent reached, or reached with no
// weight, stays where it is.
void maniGraphAdvanceLearned(maniGraphUpdate* u, uint32_t p);

#endif // MAUL_ANIM_SRC_GRAPH_LEARNED_STEP_H
