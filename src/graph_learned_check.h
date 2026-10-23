// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Checking a graph source's learned motion matching nodes as its build
// does.

#ifndef MAUL_ANIM_SRC_GRAPH_LEARNED_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_LEARNED_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>

// A learned node's fields, its query against its model's database, its
// literal tags against its model's Projectors.
maniBuildResult maniCheckGraphLearned(const maniGraphSource* source, const maniGraphNodeSource* n,
                                      uint32_t node);

#endif // MAUL_ANIM_SRC_GRAPH_LEARNED_CHECK_H
