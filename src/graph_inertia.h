// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Inertialize nodes' words (mani-0023): a node's data words read and
// checked on open, and its block in an instance (flags, clocks, the
// pending request, its last two outputs and the capture) read, written,
// reset and checked when a saved state restores it.

#ifndef MAUL_ANIM_SRC_GRAPH_INERTIA_H
#define MAUL_ANIM_SRC_GRAPH_INERTIA_H

#include "graph_file.h"
#include "vecmath.h"

#include "maul-anim/graph.h"

#include <stdint.h>

// A node's data words.
enum
{
    maniGraphInertiaTrigger = 0,
    maniGraphInertiaMethod = 1,
    maniGraphInertiaDurationScalar = 2,
    maniGraphInertiaHalfLifeScale = 4,
    maniGraphInertiaHalfLifeMin = 5,
    maniGraphInertiaHalfLifeMax = 6,
    maniGraphInertiaBlock = 7,
};

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

// Per bone: the last output at 0, the one before at 10, then the
// capture (graph_file.h).
#define MANI_GRAPH_INERTIA_LAST    0u
#define MANI_GRAPH_INERTIA_PREV    10u
#define MANI_GRAPH_INERTIA_CAPTURE 20u

// An inertialize node as its words give it.
typedef struct maniGraphInertia
{
    uint32_t bones;
    uint32_t method;
    float halfLifeScale;
    float halfLifeMin;
    float halfLifeMax;
} maniGraphInertia;

// The node whose data words begin at data, of g's bones.
maniGraphInertia maniGraphInertiaOf(const maniGraph* g, uint32_t data);

float maniGraphInertiaGet(const uint32_t* block, uint32_t at);
void maniGraphInertiaSet(uint32_t* block, uint32_t at, float value);

// A bone's ten components: translation, rotation, scale.
typedef struct maniGraphTrs
{
    maniVec3 t;
    maniQuat r;
    maniVec3 s;
} maniGraphTrs;

// A bone's transform read from or written to its pose slot (a group's
// lane), or kept in a block at word at.
maniGraphTrs maniGraphTrsRead(const float* slot);
void maniGraphTrsWrite(float* slot, maniGraphTrs x);
maniGraphTrs maniGraphTrsKept(const uint32_t* block, uint32_t at);
void maniGraphTrsKeep(uint32_t* block, uint32_t at, maniGraphTrs x);

// Bone b's first word in a block of a method.
static inline uint32_t maniGraphInertiaBone(uint32_t b, uint32_t method)
{
    return MANI_GRAPH_INERTIA_HEADER + MANI_GRAPH_INERTIA_BONE_WORDS(method) * b;
}

// Nothing kept, nothing pending, inactive, its count at 0.
void maniGraphResetInertia(uint32_t* block, const maniGraphInertia* node);

// The first wrong word of a saved block of the node (little-endian words
// at bytes), or UINT32_MAX.
uint32_t maniGraphSavedInertiaValid(const uint8_t* bytes, const maniGraphInertia* node);

// The first wrong of a node's half-lives (scale 0, minimum 1, maximum
// 2) or UINT32_MAX: dead blending's 0 < minimum <= maximum, the scale
// not negative, all finite; inertialization's all 0.
uint32_t maniGraphHalfLivesValid(const maniGraphInertia* node);

// The first wrong data word of the inertialize node whose words begin at
// data and whose block starts at block, or UINT32_MAX.
uint32_t maniGraphInertiaValid(const maniGraph* g, uint32_t data, uint32_t block);

#endif // MAUL_ANIM_SRC_GRAPH_INERTIA_H
