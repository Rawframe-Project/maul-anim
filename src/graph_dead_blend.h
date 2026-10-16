// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Dead blending (mani-0023): an inertialize node's request captured as
// its last output, speeds and decay rates per axis, and applied as that
// output extrapolated in closed form, cross-faded into the input.

#ifndef MAUL_ANIM_SRC_GRAPH_DEAD_BLEND_H
#define MAUL_ANIM_SRC_GRAPH_DEAD_BLEND_H

#include "graph_inertia.h"

#include <stdbool.h>
#include <stdint.h>

// Per bone, from the block's last output, the one before it when moving
// (dt seconds earlier) and the input in locals: the capture.
void maniGraphDeadCapture(uint32_t* block, const maniGraphInertia* node, float* locals, bool moving,
                          float dt);

// The input in locals blended from the capture's extrapolation at t of
// duration seconds (0 < t < duration).
void maniGraphDeadApply(const uint32_t* block, const maniGraphInertia* node, float* locals, float t,
                        float duration);

#endif // MAUL_ANIM_SRC_GRAPH_DEAD_BLEND_H
