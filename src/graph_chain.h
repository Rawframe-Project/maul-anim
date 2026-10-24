// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Chain IK nodes' words: their count from the limit count,
// checked on open, and a node's limits read back; the checks the
// full-body nodes share (a limit's words wherever they lie, parameters,
// weights).

#ifndef MAUL_ANIM_SRC_GRAPH_CHAIN_H
#define MAUL_ANIM_SRC_GRAPH_CHAIN_H

#include "maul-anim/graph.h"
#include "maul-anim/ik.h"

#include <stdint.h>

// The words of the node whose words begin at data, at most available;
// 0 when they do not fit or its limit count passes
// MANI_IK_CHAIN_MAX_BONES - 1.
uint32_t maniGraphChainDataWords(const maniGraph* g, uint32_t data, uint32_t available);

// The first wrong data word of a CCD (fabrik false) or FABRIK node, or
// UINT32_MAX: bones of the skeleton, a vec3 goal, a vec3 pole or none
// (none for CCD), a quat rotation or none, iterations from 1 to
// MANI_IK_MAX_ITERATIONS, a tolerance finite and at least 0, a step in
// [0, pi] (0 for FABRIK), weights in [0, 1], each limit of a known kind
// with its numbers in range and its unused words 0. Whether the bones
// make a chain, and its limits one per turning bone, waits for the
// skeleton: a solve refusing them passes the input.
uint32_t maniGraphChainValid(const maniGraph* g, uint32_t data, bool fabrik);

// Limit k of the node whose words begin at data.
maniJointLimit maniGraphChainLimit(const maniGraph* g, uint32_t data, uint32_t k);

// The limit whose MANI_GRAPH_LIMIT_WORDS begin at word at.
maniJointLimit maniGraphLimitAt(const maniGraph* g, uint32_t at);

// The first wrong word of the limit at word at, or UINT32_MAX: its kind
// known, its unused words 0, its numbers as a solve takes them.
uint32_t maniGraphLimitValid(const maniGraph* g, uint32_t at);

// Whether word at names a parameter of the type, or none when optional.
bool maniGraphIkParameterValid(const maniGraph* g, uint32_t at, uint32_t type, bool optional);

// Whether the scalar at words at is a float parameter with literal bits
// 0, or a literal in [0, 1].
bool maniGraphIkWeightValid(const maniGraph* g, uint32_t at);

#endif // MAUL_ANIM_SRC_GRAPH_CHAIN_H
