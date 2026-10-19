// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A full-body IK node's source checked (mani-0031): its parameters and
// scalars, and its bones, settings and numbers as the solve takes them
// on the source's skeleton. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_BODY_CHECK_H
#define MAUL_ANIM_SRC_GRAPH_BODY_CHECK_H

#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"

#include <stdint.h>

// An XPBD or SDLS node: 1 to MANI_BODY_IK_MAX_EFFECTORS effectors, each
// a vec3 position parameter and a quat rotation parameter or none (its
// rotation weight then the number 0), its weights numbers in [0, 1] or
// float parameters; stretch only for XPBD; the top, mode, effector
// bones, strengths, settings, iterations and tolerance a solve on the
// skeleton takes; a weight in [0, 1] or a float parameter. Scalars are
// refused by place: effector e's position weight 2e, its rotation
// weight 2e + 1, the node's weight 2 * effectorCount.
maniBuildResult maniCheckGraphBody(const maniGraphSource* source, const maniGraphNodeSource* n,
                                   uint32_t node);

// The data words of a checked body node.
uint32_t maniGraphBodySourceWords(const maniGraphNodeSource* n);

#endif // MAUL_ANIM_SRC_GRAPH_BODY_CHECK_H
