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
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/graph.h"

#include <stdbool.h>
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
    // The last output in the second history, the one before in the first.
    maniGraphInertiaLastSecond = 8,
};

// A block: the header's ten words (flags, its update count, this
// update's step, the elapsed and requested seconds, the pending request,
// +inf for none, the last output's count and step, the one before's
// count, a zero word; binary32 for seconds), two histories laid out as a
// pose's locals (groups of eight bones, each component of the eight
// together), so keeping an output is one copy, then each bone's capture.
// Inertialization's per channel (translation, rotation, scale): a unit
// direction or axis, the offset's magnitude or angle, its speed and its
// duration (18 words). Dead blending's: the last output (10), its speeds
// per axis (translation, the rotation's scaled axis-angle, the scale's
// logarithm: 9) and their decay rates (9).
#define MANI_GRAPH_INERTIA_HEADER 10u
#define MANI_GRAPH_INERTIA_HISTORY(bones)                                                          \
    (((bones) + MANI_POSE_GROUP_BONES - 1u) / MANI_POSE_GROUP_BONES * MANI_POSE_GROUP_BONES *      \
     MANI_POSE_COMPONENTS)
#define MANI_GRAPH_INERTIA_CAPTURE_WORDS(method) ((method) != 0 ? 28u : 18u)
#define MANI_GRAPH_INERTIA_BLOCK(bones, method)                                                    \
    (MANI_GRAPH_INERTIA_HEADER + 2u * MANI_GRAPH_INERTIA_HISTORY(bones) +                          \
     MANI_GRAPH_INERTIA_CAPTURE_WORDS(method) * (bones))

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
// lane), kept in a block at word at (ten words), or held in a history
// whose first word is at (bone b's lane).
maniGraphTrs maniGraphTrsRead(const float* slot);
void maniGraphTrsWrite(float* slot, maniGraphTrs x);
maniGraphTrs maniGraphTrsKept(const uint32_t* block, uint32_t at);
void maniGraphTrsKeep(uint32_t* block, uint32_t at, maniGraphTrs x);
maniGraphTrs maniGraphTrsHeld(const uint32_t* block, uint32_t at, uint32_t b);
void maniGraphTrsHold(uint32_t* block, uint32_t at, uint32_t b, maniGraphTrs x);

// The first word of the history holding the last output (last true) or
// the one before, as the flags say.
static inline uint32_t maniGraphInertiaHistory(const uint32_t* block, uint32_t bones, bool last)
{
    bool second = (block[maniGraphInertiaFlags] & maniGraphInertiaLastSecond) != 0;
    return MANI_GRAPH_INERTIA_HEADER + (second == last ? MANI_GRAPH_INERTIA_HISTORY(bones) : 0u);
}

// Bone b's capture's first word.
static inline uint32_t maniGraphInertiaCapture(const maniGraphInertia* node, uint32_t b)
{
    return MANI_GRAPH_INERTIA_HEADER + 2u * MANI_GRAPH_INERTIA_HISTORY(node->bones) +
           MANI_GRAPH_INERTIA_CAPTURE_WORDS(node->method) * b;
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
