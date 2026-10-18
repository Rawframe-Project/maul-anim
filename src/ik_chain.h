// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the chain solvers share (mani-0030): a chain's bones from its
// ends, its solve checked, its model frames recomposed from a bone down,
// a turn applied to a bone within its limit, and the end's rotation and
// the weight applied once the chain is solved.

#ifndef MAUL_ANIM_SRC_IK_CHAIN_H
#define MAUL_ANIM_SRC_IK_CHAIN_H

#include "ik_frame.h"

#include "maul-anim/base.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

// A chain being solved: its bones from the root to the end, the frame
// of the root's parent, each bone's model frame and axis (the next
// bone's rest offset, unit, zero for none), and the turning bones'
// input locals.
typedef struct maniIkChain
{
    uint32_t bones[MANI_IK_CHAIN_MAX_BONES];
    maniIkFrame frames[MANI_IK_CHAIN_MAX_BONES];
    maniVec3 axes[MANI_IK_CHAIN_MAX_BONES];
    maniTransform before[MANI_IK_CHAIN_MAX_BONES];
    maniIkFrame top;
    uint32_t count;
} maniIkChain;

// The chain's bones root first into bones; their count, or 0 when the
// root is not an ancestor of the end (or is the end) or the path passes
// MANI_IK_CHAIN_MAX_BONES.
uint32_t maniIkChainPath(const maniSkeleton* skeleton, uint32_t root, uint32_t end,
                         uint32_t* bones);

// Whether a chain's count - 1 limits, one per turning bone, are valid:
// known kinds, numbers in range, no hinge along its bone's axis.
bool maniIkChainLimitsValid(const maniSkeleton* skeleton, const uint32_t* bones, uint32_t count,
                            const maniJointLimit* limits);

// Whether the solve fits the pose: its bones a chain within
// MANI_IK_CHAIN_MAX_BONES, its limits valid, its numbers in range; a
// pole is allowed when poles is true, a step when steps is.
bool maniIkChainValid(const maniPose* pose, const maniSkeleton* skeleton,
                      const maniChainSolve* solve, bool poles, bool steps);

// The chain set up from the pose: bones, frames, axes, input locals.
void maniIkChainBegin(maniIkChain* chain, const maniPose* pose, const maniSkeleton* skeleton,
                      const maniChainSolve* solve);

// The frames of bones from k to the end recomposed from the pose.
void maniIkChainFrames(maniIkChain* chain, const maniPose* pose, uint32_t from);

// Bone k turned by a model-space turn, then limited; its frames below
// left stale.
void maniIkChainTurn(maniIkChain* chain, maniPose* pose, const maniSkeleton* skeleton,
                     const maniChainSolve* solve, uint32_t k, maniQuat turn);

// The end's origin.
maniVec3 maniIkChainEnd(const maniIkChain* chain);

// The end turned toward the wanted rotation by its weight, then every
// turned bone's correction weighted on its input local; whether the end
// lies within the tolerance at full weight.
bool maniIkChainFinish(maniIkChain* chain, maniPose* pose, const maniChainSolve* solve);

#endif // MAUL_ANIM_SRC_IK_CHAIN_H
