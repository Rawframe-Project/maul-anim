// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Turn limit nodes' words (mani-0037): a node's data words checked on
// open, its block in an instance (the time its updates gave, a rotation a
// bone) checked when a saved state restores it, and its step.

#ifndef MAUL_ANIM_SRC_GRAPH_TURN_H
#define MAUL_ANIM_SRC_GRAPH_TURN_H

#include "maul-anim/graph.h"
#include "maul-anim/pose.h"

#include <stdint.h>

// A node's data words (graph_file.h): its mask (MANI_GRAPH_NO_MASK for
// every bone at weight 1), its speed (a scalar, radians a second), its
// reset trigger (or MANI_NO_PARAMETER) and its block's first word.
enum
{
    maniGraphTurnMask = 0,
    maniGraphTurnSpeed = 1,
    maniGraphTurnReset = 3,
    maniGraphTurnBlock = 4,
};

// A block: the seconds its updates gave since it last stepped (binary64,
// low word first, not negative), then a rotation a bone (x, y, z, w
// binary32), the last output's, kept normalized: all four zero for a bone
// it holds nothing for (after a reset, before its first step, after an
// update that did not reach the node), which takes its input as it is.
enum
{
    maniGraphTurnPending = 0,
    maniGraphTurnRotations = 2,
};

// The words of the block of a node over bones bones.
uint32_t maniGraphTurnBlockWords(uint32_t bones);

// The first wrong data word of the node whose words begin at data and
// whose block starts at block, or UINT32_MAX.
uint32_t maniGraphTurnValid(const maniGraph* g, uint32_t data, uint32_t block);

// The first wrong word of a saved block (little-endian words at bytes)
// of a node over bones bones, or UINT32_MAX: the time finite and not
// negative, every rotation all zero or finite and of length 1 within
// binary32's drift.
uint32_t maniGraphSavedTurnValid(const uint8_t* bytes, uint32_t bones);

// Adds seconds to the time the node's block keeps.
void maniGraphTurnAdd(uint32_t* block, double seconds);

// Drops what the block holds for each of bones bones: their next step
// takes the input as it is.
void maniGraphResetTurn(uint32_t* block, uint32_t bones);

// Steps the block by the time it keeps for the bones listed (count of
// them, increasing; every bone of the pose when list is NULL): each
// bone's rotation in pose turned from the block's toward it by no more
// than its weight (weights, a bone each, or 1 for every bone when NULL)
// times speed (radians a second) times the time, a budget of 0 or below
// (or not a number) holding the rotation, and kept; a bone the block
// holds nothing for takes the pose as it is.
void maniGraphTurnPose(uint32_t* block, const float* weights, float speed, const uint32_t* list,
                       uint32_t count, maniPose* pose);

#endif // MAUL_ANIM_SRC_GRAPH_TURN_H
