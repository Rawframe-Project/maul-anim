// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clip node's step in an update (graph_clip_step.c): its sync group's
// leader chosen, then its playhead moved, its events listed and its root
// motion added.

#ifndef MAUL_ANIM_SRC_GRAPH_CLIP_STEP_H
#define MAUL_ANIM_SRC_GRAPH_CLIP_STEP_H

#include "graph_emit.h"

#include <stdint.h>

// Each sync group's leader, chosen once the weights are shared, and how
// it moves.
void maniGraphLead(const maniGraphUpdate* u);

// Moves clip node p. One no parent reached and in no sync group stays
// where it is.
void maniGraphAdvanceClip(maniGraphUpdate* u, uint32_t p);

#endif // MAUL_ANIM_SRC_GRAPH_CLIP_STEP_H
