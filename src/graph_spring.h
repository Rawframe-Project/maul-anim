// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring bone nodes' words (mani-0026): a node's data words read and
// checked on open and against its set at binding, its block in an
// instance (the time its updates gave, then a spring state's block)
// checked when a saved state restores it, and its pose.

#ifndef MAUL_ANIM_SRC_GRAPH_SPRING_H
#define MAUL_ANIM_SRC_GRAPH_SPRING_H

#include "maul-anim/base.h"
#include "maul-anim/graph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stdint.h>

// A node's data words (graph_file.h): its spring set slot, its position
// (a vec3 parameter or MANI_NO_PARAMETER), rotation (a quat parameter
// or MANI_NO_PARAMETER) and reset trigger (or MANI_NO_PARAMETER), its
// block's first word, the words of its set's spring state, and its force
// (a vec3 parameter or MANI_NO_PARAMETER).
enum
{
    maniGraphSpringSet = 0,
    maniGraphSpringPosition = 1,
    maniGraphSpringRotation = 2,
    maniGraphSpringReset = 3,
    maniGraphSpringBlock = 4,
    maniGraphSpringStateWords = 5,
    maniGraphSpringForce = 6,
};

// The most words a node's spring state takes (64 MiB): a set past it is
// refused.
#define MANI_GRAPH_SPRING_MAX_WORDS (1u << 24)

// A block: the seconds its updates gave since it last stepped (binary64,
// low word first, not negative), then the spring state's block.
enum
{
    maniGraphSpringPending = 0,
    maniGraphSpringState = 2,
};

// The first wrong data word of the node whose words begin at data and
// whose block starts at block, or UINT32_MAX.
uint32_t maniGraphSpringValid(const maniGraph* g, uint32_t data, uint32_t block);

// The words of a set's spring state.
uint64_t maniGraphSpringSetWords(const maniSpringSet* set);

// The words of the node's block.
uint32_t maniGraphSpringBlockWords(const maniGraph* g, uint32_t data);

// Whether a set takes the node: its state the node's words.
bool maniGraphSpringFits(const maniGraph* g, uint32_t data, const maniSpringSet* set);

// The first wrong word of a saved block (little-endian words at bytes)
// of a node of set, or UINT32_MAX.
uint32_t maniGraphSavedSpringValid(const uint8_t* bytes, const maniSpringSet* set);

// Adds seconds to the time the node's block keeps.
void maniGraphSpringAdd(uint32_t* block, double seconds);

// Drops the motion: the next evaluation places every tail where its pose
// puts it.
void maniGraphResetSpring(const maniGraph* g, uint32_t data, uint32_t* block);

// The words a set's step works in (the instance's room holds them).
uint64_t maniGraphSpringWorkWords(const maniSpringSet* set);

// Steps the block by the time it keeps toward pose, a pose of the
// skeleton, at placement (its rotation unit) under force (finite, in the
// world), working in work, and turns the pose's joints toward their
// tails.
void maniGraphSpringPose(const maniSpringSet* set, const maniSkeleton* skeleton, uint32_t* block,
                         uint32_t* work, const maniTransform* placement, maniVec3 force,
                         maniPose* pose);

#endif // MAUL_ANIM_SRC_GRAPH_SPRING_H
