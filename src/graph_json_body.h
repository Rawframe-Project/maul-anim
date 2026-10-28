// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body IK nodes' JSON form, read and written: the
// top and bones by name, effectors' goals parameters by name, weights as
// numbers or {"parameter": name}, settings' limits as a chain node's.
// Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_BODY_H
#define MAUL_ANIM_SRC_GRAPH_JSON_BODY_H

#include "graph_json_read.h"
#include "graph_json_write.h"

#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// A full-body node's members (in the order kind, inputs, top, effectors,
// iterations, topMode, bones, stretch, tolerance, weight) into s: mode
// "pinned" ("free", "prePull"), no settings, no stretch, tolerance 0 and
// weight 1 when left out. Effectors are a list of 1 to
// MANI_MAX_BODY_IK_EFFECTORS objects {"bone", "position", "rotation",
// "positionWeight", "rotationWeight", "strength", "pullChain"} (bone and
// position required; no rotation, weights 1, strength 1 and pull chain 0
// when left out; a rotation weight only with a rotation); bones a list
// of at most MANI_MAX_BODY_IK_BONES objects {"bone", "limit",
// "preferred", "rotationStiffness", "positionStiffness"} (bone required;
// no limit, zero angles and stiffness 0 when left out).
bool maniGraphJsonBody(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                       maniGraphBodyNodeSource* s);

// A full-body node's members after its inputs: its settings when it has
// any, stretch when on, an effector's rotation and rotation weight when
// it has a rotation, every other member always.
void maniGraphJsonPutBody(maniGraphWriter* w, const maniGraphSource* source,
                          const maniGraphNodeSource* n);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_BODY_H
