// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A motion matching node's step in an update: its playhead
// moved through its entry, its events listed and its root motion added,
// then a search when one is due, a jump to the pick and a request of
// inertialization.

#ifndef MAUL_ANIM_SRC_GRAPH_MATCH_STEP_H
#define MAUL_ANIM_SRC_GRAPH_MATCH_STEP_H

#include "graph_emit.h"
#include "match_search.h"

#include "maul-anim/match.h"

#include <stdint.h>

// The continuing frame's features (normalized, in here) blended toward
// the next frame's by w, as the query takes them at the playhead:
// a + w (b - a) a feature, bone facings made unit again.
static inline void maniGraphMatchBlend(const maniMotionDatabase* d, float* here, const float* next,
                                       float w)
{
    for (uint32_t j = 0; j < d->featureCount; ++j)
    {
        here[j] += w * (next[j] - here[j]);
    }
    maniMatchFacings(d, here);
}

// Moves motion matching node p. One no parent reached, or reached with
// no weight, stays where it is.
void maniGraphAdvanceMatch(maniGraphUpdate* u, uint32_t p);

#endif // MAUL_ANIM_SRC_GRAPH_MATCH_STEP_H
