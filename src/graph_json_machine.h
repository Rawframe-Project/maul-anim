// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A state machine node's JSON form read (mani-0023): its entry, chain
// limit and transitions, each condition an object of one of three forms.
// Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_MACHINE_H
#define MAUL_ANIM_SRC_GRAPH_JSON_MACHINE_H

#include "graph_json_read.h"

#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// The machine's members entry, chainLimit, transitions and finals (values in
// members, MANI_JSON_NONE when absent) into n.
bool maniGraphJsonMachine(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                          maniGraphNodeSource* n);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_MACHINE_H
