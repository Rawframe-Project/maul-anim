// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A blend space node's JSON form read: its position, its
// points and a 1D space's period, a banded space's polar flag or a
// triangulated space's authored triangles. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_SPACE_H
#define MAUL_ANIM_SRC_GRAPH_JSON_SPACE_H

#include "graph_json_read.h"

#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// The space's members (values in members in the order kind, inputs,
// position, points and its third, MANI_JSON_NONE when absent) into n,
// whose kind and inputs are read.
bool maniGraphJsonSpace(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                        maniGraphNodeSource* n);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_SPACE_H
