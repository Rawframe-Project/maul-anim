// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What other modules read of a pose's memory (mani-0004).

#ifndef MAUL_ANIM_SRC_POSE_MEMORY_H
#define MAUL_ANIM_SRC_POSE_MEMORY_H

#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>

// True when pose is a pose of skeleton: both non-NULL, the pose made by
// maniInitPose, and the identity and bone count the same.
bool maniPoseMatches(const maniPose* pose, const maniSkeleton* skeleton);

// Sets every bone's local to the skeleton's rest and marks the model
// stale; mani_errorInvalid when the pose is not the skeleton's.
maniResult maniPoseSetRestLocals(maniPose* pose, const maniSkeleton* skeleton);

// The model matrices, twelve floats a bone in bone order.
const float* maniPoseModelData(const maniPose* pose);

#endif // MAUL_ANIM_SRC_POSE_MEMORY_H
