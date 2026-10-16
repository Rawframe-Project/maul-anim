// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An inertialize node evaluated (mani-0023): a pending request captured
// from its last two outputs and its input, the capture applied at its
// elapsed time, the output kept in its history.

#ifndef MAUL_ANIM_SRC_GRAPH_INERTIA_APPLY_H
#define MAUL_ANIM_SRC_GRAPH_INERTIA_APPLY_H

#include "maul-anim/pose.h"

#include <stdint.h>

// Bollo's quintic at t: from x0 at speed v0 to 0 at t1, 0 from t1 on.
float maniGraphQuintic(float x0, float v0, float t1, float t);

// Writes into output the input eased by the node's block (of bones
// bones): a pending request captured first, the output then kept.
void maniGraphInertiaEvaluate(uint32_t* block, uint32_t bones, const maniPose* input,
                              maniPose* output);

#endif // MAUL_ANIM_SRC_GRAPH_INERTIA_APPLY_H
