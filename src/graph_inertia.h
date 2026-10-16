// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inertialize nodes' words (mani-0023): a node's data words checked on
// open, and its block in an instance (flags, clocks, the pending request,
// its last two outputs and the capture) read, written, reset and checked
// when a saved state restores it.

#ifndef MAUL_ANIM_SRC_GRAPH_INERTIA_H
#define MAUL_ANIM_SRC_GRAPH_INERTIA_H

#include "graph_file.h"

#include "maul-anim/graph.h"

#include <stdint.h>

// A block's header words, and its flags.
enum
{
    maniGraphInertiaFlags = 0,
    maniGraphInertiaSerial = 1,
    maniGraphInertiaStep = 2,
    maniGraphInertiaElapsed = 3,
    maniGraphInertiaDuration = 4,
    maniGraphInertiaPending = 5,
    maniGraphInertiaLastSerial = 6,
    maniGraphInertiaLastStep = 7,
    maniGraphInertiaPrevSerial = 8,
};

enum
{
    maniGraphInertiaActive = 1,
    maniGraphInertiaLastKept = 2,
    maniGraphInertiaPrevKept = 4,
};

// Per bone: the last output at 0, the one before at 10, then per channel
// (translation, rotation, scale) six words: a direction or axis, the
// magnitude or angle, its speed, its duration.
#define MANI_GRAPH_INERTIA_LAST    0u
#define MANI_GRAPH_INERTIA_PREV    10u
#define MANI_GRAPH_INERTIA_CAPTURE 20u

float maniGraphInertiaGet(const uint32_t* block, uint32_t at);
void maniGraphInertiaSet(uint32_t* block, uint32_t at, float value);

// Bone b's first word in a block.
static inline uint32_t maniGraphInertiaBone(uint32_t b)
{
    return MANI_GRAPH_INERTIA_HEADER + MANI_GRAPH_INERTIA_BONE_WORDS * b;
}

// Nothing kept, nothing pending, inactive, its count at 0.
void maniGraphResetInertia(uint32_t* block, uint32_t bones);

// The first wrong word of a saved block of bones bones (little-endian
// words at bytes), or UINT32_MAX.
uint32_t maniGraphSavedInertiaValid(const uint8_t* bytes, uint32_t bones);

// The first wrong data word of the inertialize node whose words begin at
// data and whose block starts at block, or UINT32_MAX.
uint32_t maniGraphInertiaValid(const maniGraph* g, uint32_t data, uint32_t block);

#endif // MAUL_ANIM_SRC_GRAPH_INERTIA_H
