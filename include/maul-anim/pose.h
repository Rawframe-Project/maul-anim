// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Poses: a skeleton's local transforms and the model matrices
// composed from them, in memory the caller owns. A pose holds no
// pointers: copying its bytes copies the pose, and it may move between
// threads. A pose is used by one thread at a time.

#ifndef MAUL_ANIM_POSE_H
#define MAUL_ANIM_POSE_H

#include "maul-anim/base.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The alignment a pose's memory needs.
#define MANI_POSE_ALIGNMENT 32u

    // A pose: the caller's memory, initialized by maniInitPose.
    typedef struct maniPose maniPose;

    /// Returns the bytes a pose of the skeleton needs.
    ///
    /// @param skeleton  An opened skeleton.
    /// @return The size in bytes.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniSkeleton_GetPoseSize(const maniSkeleton* skeleton);

    /// Makes a pose of the skeleton in caller memory, every bone at its
    /// rest, the model matrices composed.
    ///
    /// @param skeleton  An opened skeleton.
    /// @param memory    At least maniSkeleton_GetPoseSize bytes, MANI_POSE_ALIGNMENT
    ///                  aligned; the pose lives in it.
    /// @param size      The memory's size in bytes.
    /// @param poseOut   Receives the pose.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned memory; `mani_errorCapacity` when size is too small.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniInitPose(const maniSkeleton* skeleton, void* memory,
                                                    size_t size, maniPose** poseOut);

    /// Returns the number of bones of the pose's skeleton.
    ///
    /// @param pose  The pose.
    /// @return The bone count.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API uint32_t maniPose_GetBoneCount(const maniPose* pose);

    /// Returns a bone's local transform.
    ///
    /// @param pose  The pose.
    /// @param bone  The bone.
    /// @return The transform, or the identity for a bone past the count.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API maniTransform maniPose_GetLocal(const maniPose* pose, uint32_t bone);

    /// Sets a bone's local transform and marks the model matrices stale.
    /// A bone past the count is ignored.
    ///
    /// @param pose       The pose.
    /// @param bone       The bone.
    /// @param transform  The transform; its rotation is used as given and
    ///                   should be unit.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API void maniPose_SetLocal(maniPose* pose, uint32_t bone, maniTransform transform);

    /// Reads the local transforms of bones first to first + count - 1.
    ///
    /// @param pose        The pose.
    /// @param first       The first bone.
    /// @param count       How many bones; the range must lie within the
    ///                    bone count.
    /// @param transforms  Receives count transforms.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// range past the bone count.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_ReadLocals(const maniPose* pose, uint32_t first,
                                                           uint32_t count,
                                                           maniTransform* transforms);

    /// Writes the local transforms of bones first to first + count - 1
    /// and marks the model matrices stale.
    ///
    /// @param pose        The pose.
    /// @param first       The first bone.
    /// @param count       How many bones; the range must lie within the
    ///                    bone count.
    /// @param transforms  The count transforms.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// range past the bone count.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_WriteLocals(maniPose* pose, uint32_t first,
                                                            uint32_t count,
                                                            const maniTransform* transforms);

    /// Sets every bone to the skeleton's rest and composes the model.
    ///
    /// @param pose      The pose.
    /// @param skeleton  The pose's skeleton.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// skeleton of another identity.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_SetRest(maniPose* pose,
                                                        const maniSkeleton* skeleton);

    /// Composes every bone's model matrix: its parent's model matrix times
    /// its local transform's matrix (scale, then rotation, then
    /// translation), a root's being its local's.
    ///
    /// @param pose      The pose.
    /// @param skeleton  The pose's skeleton.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton of another identity or a difference (which has no model).
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_ComputeModel(maniPose* pose,
                                                             const maniSkeleton* skeleton);

    /// Returns whether the model matrices match the local transforms: true
    /// after maniPose_ComputeModel until a local is set.
    ///
    /// @param pose  The pose.
    /// @return True when current.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API bool maniPose_IsModelCurrent(const maniPose* pose);

    /// Returns whether the pose holds a difference: what
    /// maniComputePoseDifference writes and additive layers apply, never mixed
    /// with poses of the skeleton. Making a pose, setting its rest and
    /// sampling a whole clip into it make it a pose of the skeleton.
    ///
    /// @param pose  The pose.
    /// @return True for a difference.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API bool maniPose_IsDifference(const maniPose* pose);

    /// Returns a bone's model matrix as last composed.
    ///
    /// @param pose  The pose.
    /// @param bone  The bone.
    /// @return The matrix, or the identity for a bone past the count.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API maniAffine maniPose_GetModel(const maniPose* pose, uint32_t bone);

    /// Reads the model matrices of bones first to first + count - 1, as
    /// last composed.
    ///
    /// @param pose      The pose.
    /// @param first     The first bone.
    /// @param count     How many bones; the range must lie within the bone
    ///                  count.
    /// @param matrices  Receives count matrices.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// range past the bone count.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_ReadModels(const maniPose* pose, uint32_t first,
                                                           uint32_t count, maniAffine* matrices);

    /// Splits an affine matrix into translation, rotation and scale: the
    /// fourth column, the rotation of the polar decomposition (the
    /// rotation nearest the matrix, exact up to rounding when the matrix
    /// has no shear) as a quaternion with w >= 0, and the scale along the
    /// rotated axes. A mirror becomes a negative x scale. A matrix with a
    /// zero or flat axis still splits, its rotation built from the axes
    /// that are left.
    ///
    /// @param matrix        The matrix.
    /// @param transformOut  Receives the transform.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or a
    /// component that is not finite.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniDecomposeAffine(const maniAffine* matrix,
                                                           maniTransform* transformOut);

    /// Returns a bone's model matrix, as last composed, split as
    /// maniDecomposeAffine splits it.
    ///
    /// @param pose  The pose.
    /// @param bone  The bone.
    /// @return The transform, or the identity for a bone past the count or a
    /// matrix that is not finite.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_API maniTransform maniPose_GetModelTransform(const maniPose* pose, uint32_t bone);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_POSE_H
