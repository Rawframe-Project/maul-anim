// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A graph parameter's value as an instance keeps it (finite, a
// quaternion normalized, a bounded value clamped), and its four words in
// the bytes of saved state.

#ifndef MAUL_ANIM_SRC_GRAPH_VALUES_H
#define MAUL_ANIM_SRC_GRAPH_VALUES_H

#include "maul-anim/graph.h"

#include <stdbool.h>
#include <stdint.h>

// A value's four words at p, little-endian.
maniGraphValue maniGraphLoadValue(const uint8_t* p);
void maniGraphStoreValue(uint8_t* p, maniGraphValue v);

// The value of a parameter of a type and record (entry) as the parameter
// keeps it; false for one out of form (not finite, a zero quaternion, a
// flag past 1).
bool maniGraphCanonicalValue(maniGraphParameterType type, const uint8_t* entry, maniGraphValue* v);

#endif // MAUL_ANIM_SRC_GRAPH_VALUES_H
