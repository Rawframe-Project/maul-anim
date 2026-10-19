// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mirroring poses and root moves through a mirror map (mani-0033): each
// bone takes its partner's local, the translation's normal component
// negated and turned by the bone's A, the rotation mirrored between its A
// and B, the scale as it is. A pair's two bones are read before either is
// written, so a pose may be mirrored in place.

#include "bytes.h"
#include "mirror_file.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/mirror.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static maniQuat LoadQuat(const uint8_t* p)
{
    return (maniQuat){LoadF32(p), LoadF32(p + 4), LoadF32(p + 8), LoadF32(p + 12)};
}

// The normal's component negated.
static maniVec3 Reflect(maniVec3 v, maniMirrorAxis axis)
{
    return maniVec3Make(axis == maniMirrorAxisX ? -v.x : v.x, axis == maniMirrorAxisY ? -v.y : v.y,
                        axis == maniMirrorAxisZ ? -v.z : v.z);
}

// The two vector components off the normal negated.
static maniQuat Mirror(maniQuat q, maniMirrorAxis axis)
{
    return (maniQuat){axis == maniMirrorAxisX ? q.x : -q.x, axis == maniMirrorAxisY ? q.y : -q.y,
                      axis == maniMirrorAxisZ ? q.z : -q.z, q.w};
}

// A bone's local from the locals.
static void ReadLocal(const float* locals, uint32_t bone, float local[MANI_POSE_COMPONENTS])
{
    const float* slot =
        locals +
        (size_t)(bone / MANI_POSE_GROUP_BONES) * (MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS) +
        bone % MANI_POSE_GROUP_BONES;
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        local[c] = slot[c * MANI_POSE_GROUP_BONES];
    }
}

// A bone's mirrored local (its record e) from its partner's.
static void MirrorLocal(const uint8_t* e, maniMirrorAxis axis,
                        const float partner[MANI_POSE_COMPONENTS], float* slot)
{
    maniQuat a = LoadQuat(e + 8);
    maniQuat b = LoadQuat(e + 24);
    maniVec3 t = maniQuatRotate(a, Reflect(maniVec3Make(partner[0], partner[1], partner[2]), axis));
    maniQuat q = Mirror((maniQuat){partner[3], partner[4], partner[5], partner[6]}, axis);
    q = maniQuatMul(maniQuatMul(a, q), b);
    const float turned[7] = {t.x, t.y, t.z, q.x, q.y, q.z, q.w};
    maniPoseSlotPut(slot, 0, turned, 7);
    maniPoseSlotPut(slot, 7, partner + 7, 3);
}

maniResult maniMirrorPose(const maniMirrorMap* map, const maniSkeleton* skeleton,
                          const maniPose* in, maniPose* out)
{
    if (map == NULL || skeleton == NULL ||
        maniSkeletonIdentity(skeleton) != map->skeletonIdentity ||
        maniSkeletonBoneCount(skeleton) != map->boneCount || !maniPoseMatches(in, skeleton) ||
        !maniPoseMatches(out, skeleton) || maniPoseIsDifference(in))
    {
        return mani_errorInvalid;
    }
    const float* from = maniPoseLocalsRead(in);
    float* to = maniPoseLocals(out);
    for (uint32_t b = 0; b < map->boneCount; ++b)
    {
        uint32_t m = maniLoadU32(map->bones + (size_t)b * MANI_MIRROR_BONE_SIZE);
        if (m < b)
        {
            continue;
        }
        float own[MANI_POSE_COMPONENTS];
        float partner[MANI_POSE_COMPONENTS];
        ReadLocal(from, b, own);
        ReadLocal(from, m, partner);
        MirrorLocal(map->bones + (size_t)b * MANI_MIRROR_BONE_SIZE, map->axis, partner,
                    maniPoseSlot(to, b));
        if (m != b)
        {
            MirrorLocal(map->bones + (size_t)m * MANI_MIRROR_BONE_SIZE, map->axis, own,
                        maniPoseSlot(to, m));
        }
    }
    maniPoseSetKind(out, false);
    maniPoseMarkStale(out);
    return mani_success;
}

maniRootMove maniMirrorRootMotion(const maniMirrorMap* map, maniRootMove move)
{
    if (map == NULL)
    {
        return move;
    }
    return (maniRootMove){Reflect(move.translation, map->axis), Mirror(move.rotation, map->axis)};
}
