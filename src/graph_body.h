// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Full-body IK nodes' words (mani-0031): their count from the effector
// and setting counts, checked on open, and a node's settings read back.

#ifndef MAUL_ANIM_SRC_GRAPH_BODY_H
#define MAUL_ANIM_SRC_GRAPH_BODY_H

#include "maul-anim/bodyik.h"
#include "maul-anim/graph.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The words of the node whose words begin at data, at most available;
// 0 when they do not fit or a count is out of range.
uint32_t maniGraphBodyDataWords(const maniGraph* g, uint32_t data, uint32_t available);

// The first wrong data word of an XPBD (sdls false) or SDLS node, or
// UINT32_MAX: a top of the skeleton and a known mode; stretch 0 or 1 (0
// for SDLS); iterations from 1 to MANI_IK_MAX_ITERATIONS; a tolerance
// finite and at least 0; a weight in [0, 1]; each effector a bone of the
// skeleton with a vec3 position, a quat rotation or none (its rotation
// weight then the number 0), weights in [0, 1], a strength and pull
// chain in [0, 1]; each setting a bone of the skeleton, a limit of a
// known kind with its numbers in range and its unused words 0,
// preferred angles finite, stiffness in [0, 1]. Whether the bones make a
// part the solve takes waits for the skeleton: a solve refusing them
// passes the input.
uint32_t maniGraphBodyValid(const maniGraph* g, uint32_t data, bool sdls);

// Setting k of the node whose words begin at data.
maniBodyBone maniGraphBodySetting(const maniGraph* g, uint32_t data, uint32_t k);

// The first word of effector e, of setting k.
uint32_t maniGraphBodyEffectorAt(uint32_t data, uint32_t e);
uint32_t maniGraphBodySettingAt(const maniGraph* g, uint32_t data, uint32_t k);

// The bytes of a checked node's room on the skeleton: 15 to reach a
// 16-byte boundary, its effectors and settings (each rounded up to 16
// bytes), then the scratch its solve takes (none when its bones make no
// part, the solve then refusing).
size_t maniGraphBodyRoom(const maniGraph* g, uint32_t data, const maniSkeleton* skeleton);

// A multiple of 16 bytes at least bytes.
size_t maniGraphBodyRounded(size_t bytes);

#endif // MAUL_ANIM_SRC_GRAPH_BODY_H
