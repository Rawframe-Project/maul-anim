// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The IK nodes' JSON form (mani-0029), read and written: bones by name,
// goal and pole parameters by name, axes as [x, y, z], twist, soften,
// cone and weight as numbers or {"parameter": name}. Offline.

#ifndef MAUL_ANIM_SRC_GRAPH_JSON_IK_H
#define MAUL_ANIM_SRC_GRAPH_JSON_IK_H

#include "graph_json_read.h"
#include "graph_json_write.h"

#include "maul-anim/graphbuild.h"

#include <stdbool.h>
#include <stdint.h>

// A two-bone node's members (values in members in the order kind,
// inputs, start, middle, end, goal, pole, hinge, twist, soften, weight,
// MANI_JSON_NONE when absent) into s: no pole, a zero hinge, twist 0,
// soften and weight 1 when left out.
bool maniGraphJsonTwoBone(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                          maniGraphTwoBoneNodeSource* s);

// An aim node's members (in the order kind, inputs, bones, goal,
// forward, shares, pole, offset, up, twist, cone, weight) into s: equal
// shares, no pole, zero offset and up, twist and cone 0 and weight 1 when
// left out. Bones are a list of 1 to MANI_AIM_MAX_BONES names, shares a
// list of as many numbers.
bool maniGraphJsonAim(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                      maniGraphAimNodeSource* s);

// A two-bone or aim node's members after its inputs: the shares and the
// pole when it has them, every other member always.
void maniGraphJsonPutIk(maniGraphWriter* w, const maniGraphSource* source,
                        const maniGraphNodeSource* n);

// A chain node's members (in the order kind, inputs, root, end, goal,
// iterations, a FABRIK node's pole or a CCD node's step, rotation,
// tolerance, rotationWeight, weight, limits) into s: no pole, step 0, no
// rotation, tolerance 0, weights 1 and no limits when left out; limits a
// list of one object per turning bone.
bool maniGraphJsonChain(maniGraphReader* r, const maniGraphScope* scope, const uint32_t* members,
                        bool fabrik, maniGraphChainNodeSource* s);

// A chain node's members after its inputs: its pole, rotation and
// limits when it has them, a CCD node's step, every other member always.
void maniGraphJsonPutChain(maniGraphWriter* w, const maniGraphSource* source,
                           const maniGraphNodeSource* n);

#endif // MAUL_ANIM_SRC_GRAPH_JSON_IK_H
