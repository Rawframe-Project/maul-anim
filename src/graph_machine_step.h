// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A state machine stepped by an update (mani-0023): its clocks advanced,
// a fade whose time is up completed, transitions chosen and fired, its
// weights set from the fade.

#ifndef MAUL_ANIM_SRC_GRAPH_MACHINE_STEP_H
#define MAUL_ANIM_SRC_GRAPH_MACHINE_STEP_H

#include "graph_instance.h"
#include "graph_machine.h"

#include "maul-anim/graph.h"

#include <stdint.h>

// The states one step entered from their start, at most
// MANI_GRAPH_MAX_CHAIN.
typedef struct maniGraphEntered
{
    uint32_t states[MANI_GRAPH_MAX_CHAIN];
    uint32_t count;
} maniGraphEntered;

// Steps a machine by seconds (not negative) with the parameters in cells,
// the triggers of the transitions it fires consumed.
void maniGraphStepMachine(const maniGraphMachine* m, maniGraphCell* cells, uint32_t* block,
                          double seconds, maniGraphEntered* entered);

#endif // MAUL_ANIM_SRC_GRAPH_MACHINE_STEP_H
