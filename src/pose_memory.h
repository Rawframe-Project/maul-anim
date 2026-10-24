// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What other modules read of a pose's memory.

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

// A pose's locals after its 64-byte header: groups of eight bones, each
// component (translation x, y, z, rotation x, y, z, w, scale x, y, z) of
// the eight together. Samplers write them in place, through these.
#define MANI_POSE_HEADER_SIZE 64u
#define MANI_POSE_GROUP_BONES 8u
#define MANI_POSE_COMPONENTS  10u

// The locals' first float, and the model marked stale once they change.
float* maniPoseLocals(maniPose* pose);
const float* maniPoseLocalsRead(const maniPose* pose);
void maniPoseMarkStale(maniPose* pose);

// Copies the locals and kind of a pose of the same skeleton, marking the
// model stale.
void maniPoseCopy(maniPose* to, const maniPose* from);

// Records what the locals hold: a difference or a pose of the skeleton.
void maniPoseSetKind(maniPose* pose, bool difference);

// A bone's slot in the locals: its component c at slot[c * 8].
static inline float* maniPoseSlot(float* locals, uint32_t bone)
{
    return locals +
           (size_t)(bone / MANI_POSE_GROUP_BONES) * (MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS) +
           bone % MANI_POSE_GROUP_BONES;
}

// count components from first into a slot.
static inline void maniPoseSlotPut(float* slot, uint32_t first, const float* values, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        slot[(first + k) * MANI_POSE_GROUP_BONES] = values[k];
    }
}

// The model matrices, twelve floats a bone in bone order.
const float* maniPoseModelData(const maniPose* pose);

// The pose is of a skeleton of this identity and bone count, not a
// difference (a profile or set built for it names both).
bool maniPoseOfSkeleton(const maniPose* pose, uint64_t identity, uint32_t boneCount);

// Writes a bone's model matrix; a pass writing every bone's, parents
// first, then marks the model current.
void maniPoseStoreModel(maniPose* pose, uint32_t bone, const maniAffine* model);
void maniPoseMarkModelCurrent(maniPose* pose);

// A pose made in memory, the model pass and a bone's model transform
// for a caller already in
// the library's floating-point environment (fpenv.h): the public calls'
// workers, without entering it again.
maniResult maniInitPoseEntered(const maniSkeleton* skeleton, void* memory, size_t size,
                               maniPose** poseOut);
maniResult maniComputeModelEntered(maniPose* pose, const maniSkeleton* skeleton);
maniTransform maniPoseModelTransformEntered(const maniPose* pose, uint32_t bone);

#endif // MAUL_ANIM_SRC_POSE_MEMORY_H
