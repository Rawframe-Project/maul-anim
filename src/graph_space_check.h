// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A blend space node's source checked: its position, its
// points (finite, no two equal, within a period when it wraps, not all on
// one line for triangles), its flags and its authored triangles (exact
// tests). Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_SPACE_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_SPACE_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>

maniBuildResult maniCheckGraphSpace(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node);

#endif // MAUL_ANIM_SRC_GRAPH_SPACE_CHECK_H
