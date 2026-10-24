// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// IK and full-body nodes evaluated: each its
// input with its bones solved toward goals its parameters give.

#ifndef MAUL_ANIM_SRC_GRAPH_EVALUATE_IK_H
#define MAUL_ANIM_SRC_GRAPH_EVALUATE_IK_H

#include "graph_evaluation.h"

#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stdint.h>

// A two-bone node into out.
void maniGraphEvaluateTwoBone(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out);

// An aim node into out.
void maniGraphEvaluateAim(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out);

// A CCD or (fabrik) FABRIK node into out.
void maniGraphEvaluateChain(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                            bool fabrik);

// A full-body node, SDLS (sdls) or XPBD, into out.
void maniGraphEvaluateBody(const maniGraphEvaluation* e, const uint8_t* record, maniPose* out,
                           bool sdls);

#endif // MAUL_ANIM_SRC_GRAPH_EVALUATE_IK_H
