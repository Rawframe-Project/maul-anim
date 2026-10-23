// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A one-shot stepped at the start of an update: its fade
// advanced, then its triggers read.

#ifndef MAUL_ANIM_SRC_GRAPH_SHOT_STEP_H
#define MAUL_ANIM_SRC_GRAPH_SHOT_STEP_H

#include "graph_instance.h"
#include "graph_shot.h"

#include <stdbool.h>
#include <stdint.h>

// Steps a one-shot by seconds (not negative) with the triggers in cells:
// its fade advanced, then the first set of abort, fire and fade-out
// acting (each consumed), or else a fade-out begun when its leading clip
// had no more than the fade-out's seconds left. Whether fire restarted
// it.
bool maniGraphStepShot(const maniGraphShot* s, maniGraphCell* cells, uint32_t* block,
                       double seconds);

#endif // MAUL_ANIM_SRC_GRAPH_SHOT_STEP_H
