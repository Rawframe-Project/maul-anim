// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A blend space node's source checked (mani-0023): its position, its
// points (finite, no two equal, within a period when it wraps) and its
// flags. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_SPACE_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_SPACE_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>

maniBuildResult maniCheckGraphSpace(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node);

// A binary32's key in ascending order of value (equal values aside, -0
// before +0).
uint64_t maniGraphSpaceKey(float value);

#endif // MAUL_ANIM_SRC_GRAPH_SPACE_CHECK_H
