// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mirror parity of a graph's nodes at build: how many
// mirror nodes lie above each, odd or even, from the output down.

#ifndef MAUL_ANIM_SRC_GRAPH_MIRROR_H
#define MAUL_ANIM_SRC_GRAPH_MIRROR_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>

// Gives each node of a checked source its parity in mirrored (1 under an
// even number of mirror nodes, 2 an odd), its readers first: order lists
// the nodes each after its inputs, the output last. Refuses a node
// reached both ways (maniBuildReasonMirrorGraph, the node, 2).
maniBuildResult maniGraphMirrorParity(const maniGraphSource* source, const uint32_t* order,
                                      uint32_t* mirrored);

#endif // MAUL_ANIM_SRC_GRAPH_MIRROR_H
