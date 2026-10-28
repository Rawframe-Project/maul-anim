// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the full-body solvers share: the description
// checked, the solved part found (the top and every bone on a path from
// it to an effector, by bone index, so parents come first), each part
// bone's input frame and offset, the effectors' blended goals, the
// steps taken before the solve (preferred angles, pull chains,
// pre-pull) and the result written back as local corrections.

#ifndef MAUL_ANIM_SRC_BODY_IK_H
#define MAUL_ANIM_SRC_BODY_IK_H

#include "ik_frame.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// No part bone: the top's parent slot, a bone without settings.
#define MANI_BODY_NONE 0xFFFFFFFFu

// A bone of the solved part.
typedef struct maniBodyNode
{
    // Its input model frame and its parent's (the identity above a
    // root).
    maniIkFrame frame;
    maniIkFrame parentFrame;
    // Its origin and the model-space turn taken so far (the identity at
    // the input); a turn of a node moves no other node's.
    maniVec3 origin;
    maniQuat turn;
    // Its input local rotation.
    maniQuat local;
    // Its input origin less its parent's, in model space; zero for the
    // top.
    maniVec3 offset;
    // Its limit's axis in its rest frame: the direction of its one child
    // on the part at rest; zero for none.
    maniVec3 axis;
    uint32_t bone;
    // The parent's slot on the part, MANI_BODY_NONE for the top.
    uint32_t parent;
    // Its settings' index in the solve, MANI_BODY_NONE for none.
    uint32_t setting;
    // How many part bones lie below it, how many are its children.
    uint32_t below;
    uint32_t children;
    uint32_t reserved;
} maniBodyNode;

// An effector's goals, blended from the input by its weights.
typedef struct maniBodyGoal
{
    maniQuat rotation;
    maniVec3 position;
    // Its bone's slot on the part.
    uint32_t node;
    // Whether it wants a rotation (a rotation weight above 0).
    uint32_t turns;
    // The summed offsets from the top to its bone, no less than the
    // part's mean offset: what a rotation miss is a length by.
    float reach;
    float reserved;
} maniBodyGoal;

// The solved part in the caller's scratch.
typedef struct maniBodyPart
{
    maniBodyNode* nodes;
    maniBodyGoal* goals;
    // Each solver's own room: floats past the goals.
    float* room;
    uint32_t count;
    uint32_t effectors;
    // The mean of the part's nonzero offsets (1 for none).
    float mean;
    float reserved;
} maniBodyPart;

// The part's size in nodes for a solve valid on the skeleton, 0 for one
// refused, past set when the refusal is a bound passed (effectors,
// settings, iterations, or the part past MANI_MAX_BODY_IK_BONES); the
// pose is not looked at.
uint32_t maniBodyPartShape(const maniSkeleton* skeleton, const maniBodySolve* solve, bool* past);

// A solve checked on a usable pose of the skeleton with aligned scratch:
// mani_success with the part's size in countOut; mani_errorLimit for a
// count or a part past its bound; mani_errorCapacity for too little
// scratch; mani_errorInvalid for anything else refused.
maniResult maniBodyPartCheck(const maniPose* pose, const maniSkeleton* skeleton,
                             const maniBodySolve* solve, const void* scratch, size_t scratchSize,
                             uint32_t* countOut);

// Each solver's own floats for a part of count nodes.
size_t maniBodyXpbdRoom(uint32_t count, uint32_t effectors);
size_t maniBodySdlsRoom(uint32_t count, uint32_t effectors);

// Lays out the part in the scratch and reads the input: frames,
// offsets, axes, goals. The solve must be valid and the scratch at
// least maniBodyScratchBytes.
void maniBodyBegin(maniBodyPart* part, const maniPose* pose, const maniSkeleton* skeleton,
                   const maniBodySolve* solve, void* scratch, uint32_t count);

// A node's settings, NULL for none.
const maniBodyBone* maniBodySetting(const maniBodySolve* solve, const maniBodyNode* node);

// A node's model rotation now: its turn, then its input rotation.
maniQuat maniBodyRotation(const maniBodyNode* node);

// Places every node but the top from the turns: each origin its
// parent's origin plus the parent's turn of its offset.
void maniBodyPlace(maniBodyPart* part);

// Turns node k and every node below it by a model-space turn about k's
// origin, the nodes placed after.
void maniBodyTurn(maniBodyPart* part, uint32_t k, maniQuat turn);

// Node k's local rotation now, from its turn and its parent's.
maniQuat maniBodyLocal(const maniBodyPart* part, uint32_t k);

// Effector e's miss with its bone's origin at origin: the larger of its
// distance from the goal and, wanting a rotation, its rotation's angle
// from the goal's times its reach.
float maniBodyGoalMiss(const maniBodyPart* part, uint32_t e, maniVec3 origin);

// The largest effector miss.
float maniBodyMiss(const maniBodyPart* part);

// Takes the steps before the solve: preferred angles by compression,
// pull chains, pre-pull; the nodes placed after.
void maniBodyPrepare(maniBodyPart* part, const maniBodySolve* solve);

// The node's local rotation relative to its rest's limited by its
// settings, its turn changed to match.
void maniBodyLimit(maniBodyPart* part, const maniSkeleton* skeleton, const maniBodySolve* solve,
                   uint32_t k);

// Writes the part's turns (and, stretching, its origins) to the pose's
// locals by the weight; whether every effector lies within the
// tolerance at full weight.
bool maniBodyFinish(maniBodyPart* part, maniPose* pose, const maniBodySolve* solve);

#endif // MAUL_ANIM_SRC_BODY_IK_H
