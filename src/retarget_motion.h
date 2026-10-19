// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion carried through a retargeting map (mani-0032), shared by
// the clip bake and graphs: the target bone that follows a source bone
// alone, and a move turned from the source bone's parent's reference
// model frame into the model, scaled by the map's horizontal and
// vertical ratios, and turned into the target bone's parent's frame.

#ifndef MAUL_ANIM_SRC_RETARGET_MOTION_H
#define MAUL_ANIM_SRC_RETARGET_MOTION_H

#include "maul-anim/base.h"
#include "maul-anim/retarget.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

// The frames a move carries between: each bone's parent's reference
// model rotation (the identity for a root), and the turn from the
// source's to the target's.
typedef struct maniRetargetMotion
{
    const maniRetargetMap* map;
    maniQuat sourceFrame;
    maniQuat targetFrame;
    maniQuat turn;
} maniRetargetMotion;

// The first target bone whose plan follows source bone a alone;
// MANI_NO_BONE for none.
uint32_t maniRetargetFollower(const maniRetargetMap* map, uint32_t a);

// The frames for a source bone and a target bone of the map's
// skeletons, both within them.
maniRetargetMotion maniRetargetMotionOf(const maniRetargetMap* map, const maniSkeleton* source,
                                        uint32_t sourceBone, const maniSkeleton* target,
                                        uint32_t targetBone);

// A translation carried: into the model, scaled, out.
maniVec3 maniRetargetMotionTranslation(const maniRetargetMotion* motion, maniVec3 t);

// A move carried: its translation as above, its rotation turned (and
// normalized).
maniRootMove maniRetargetMove(const maniRetargetMotion* motion, maniRootMove move);

#endif // MAUL_ANIM_SRC_RETARGET_MOTION_H
