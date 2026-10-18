// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What the IK solvers share (mani-0029): model frames composed from a
// pose's locals along a root path, model-space turns as local
// corrections, turns from cosines and sines, the pole's plane blended to
// a chain's own near the aim line, weighting.

#ifndef MAUL_ANIM_SRC_IK_FRAME_H
#define MAUL_ANIM_SRC_IK_FRAME_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

// A bone's frame in its ancestor top's frame (the model's for
// MANI_NO_BONE): the locals below top composed; the identity for top
// itself.
maniAffine maniIkBelow(const maniPose* pose, const maniSkeleton* skeleton, uint32_t top,
                       uint32_t bone);

// The model frame of a bone's parent; the identity for a root.
maniAffine maniIkParentModel(const maniPose* pose, const maniSkeleton* skeleton, uint32_t bone);

// A bone's local matrix.
maniAffine maniIkLocal(const maniPose* pose, uint32_t bone);

// Whether ancestor is a strict ancestor of bone.
bool maniIkAbove(const maniSkeleton* skeleton, uint32_t ancestor, uint32_t bone);

// A frame's origin and rigid rotation.
maniVec3 maniIkOrigin(const maniAffine* m);
maniQuat maniIkRotation(const maniAffine* m);

// A model-space turn as the correction of a bone's local rotation under
// its parent's model frame: P* q P, mirrored through x under a mirrored
// parent.
maniQuat maniIkInParent(maniQuat q, const maniAffine* parent);

// The turn about unit axis u by the angle of cosine c and sine s (any
// common scale), without losing digits at small or near-half turns.
maniQuat maniIkHalf(maniVec3 u, float c, float s);

// The turn about unit axis u taking a onto b, both perpendicular to u.
maniQuat maniIkAbout(maniVec3 u, maniVec3 a, maniVec3 b);

// The turn about the unit aim line u taking normal n (made perpendicular
// to u) to the normal of the plane of u and the pole, cross(u, pole);
// blended to none as the pole's sine to the line falls below 0.05; none
// without a pole.
maniQuat maniIkPlane(maniVec3 u, maniVec3 n, maniVec3 pole);

// A correction, w kept at least 0, lerped from the identity by weight.
maniQuat maniIkWeighted(maniQuat q, float weight);

// A bone's local rotation turned by a correction before it.
void maniIkCorrect(maniPose* pose, uint32_t bone, maniQuat correction);

bool maniIkFiniteVec(maniVec3 v);

// A pose of the skeleton that is not a difference.
bool maniIkPoseUsable(const maniPose* pose, const maniSkeleton* skeleton);

#endif // MAUL_ANIM_SRC_IK_FRAME_H
