// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Blending and adding over some groups of a pose's bones: a
// subset's evaluation computes only the groups its plan holds, each bone
// as the whole pass computes it.

#ifndef MAUL_ANIM_SRC_BLEND_GROUPS_H
#define MAUL_ANIM_SRC_BLEND_GROUPS_H

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

// maniBlendPoses and maniPose_AddLayers over the groups of MANI_POSE_GROUP_BONES
// bones whose byte in groups is not 0 (every group when groups is NULL);
// the other groups' bones are left as they were.
maniResult maniBlendPosesIn(const maniSkeleton* skeleton, const maniBlendLayer* layers,
                            uint32_t count, float threshold, maniPose* out, const uint8_t* groups);
maniResult maniAddPosesIn(const maniSkeleton* skeleton, maniPose* pose,
                          const maniBlendLayer* layers, uint32_t count, const uint8_t* groups);

#endif // MAUL_ANIM_SRC_BLEND_GROUPS_H
