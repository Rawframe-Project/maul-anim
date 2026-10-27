// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ragdoll tests' fixture: a humanoid skeleton (pelvis,
// a spine scaled by 1.5, a turned chest, head, an arm turned out along
// x, a hand, and a second root, a prop) with its rest pose, and a
// profile's source of five bodies given out of the bones' order (the
// spine and hand without bodies).

#ifndef MAUL_ANIM_TEST_RAGDOLL_FIXTURE_H
#define MAUL_ANIM_TEST_RAGDOLL_FIXTURE_H

#include "test_harness.h"

#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/ragdoll.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[8192];
static _Alignas(16) uint8_t s_file[8192];
static size_t s_size;
static maniSkeleton s_skeleton;
static maniPose* s_pose;
static maniBuilder* s_builder;

enum
{
    kPelvis,
    kSpine,
    kChest,
    kHead,
    kUpperArm,
    kLowerArm,
    kHand,
    kProp,
    kBones
};

static uint32_t s_bone[kBones];

static maniBoneSource Bone(const char* name, int32_t parent, maniVec3 at, maniQuat turn,
                           float scale)
{
    return (maniBoneSource){
        name, (uint32_t)strlen(name), parent, {at, turn, {scale, scale, scale}}};
}

static uint32_t Find(const char* name)
{
    return maniSkeletonFindBone(&s_skeleton, name, (uint32_t)strlen(name));
}

static void OpenSkeleton(void)
{
    // The spine is scaled by 1.5, the upper arm turned a quarter about z
    // (pointing out along x), the lower arm turned a little about y; the
    // prop is a second root.
    const maniQuat none = {0.0f, 0.0f, 0.0f, 1.0f};
    const maniBoneSource bones[kBones] = {
        Bone("pelvis", -1, (maniVec3){0.0f, 1.0f, 0.0f}, none, 1.0f),
        Bone("spine", 0, (maniVec3){0.0f, 0.1f, 0.0f}, none, 1.5f),
        Bone("chest", 1, (maniVec3){0.0f, 0.2f, 0.02f}, (maniQuat){0.0998f, 0.0f, 0.0f, 0.995f},
             1.0f),
        Bone("head", 2, (maniVec3){0.0f, 0.3f, 0.0f}, none, 1.0f),
        Bone("upperArm", 2, (maniVec3){0.15f, 0.2f, 0.0f},
             (maniQuat){0.0f, 0.0f, -0.70710678f, 0.70710678f}, 1.0f),
        Bone("lowerArm", 4, (maniVec3){0.0f, 0.28f, 0.0f}, (maniQuat){0.0f, 0.2588f, 0.0f, 0.9659f},
             1.0f),
        Bone("hand", 5, (maniVec3){0.0f, 0.25f, 0.0f}, none, 1.0f),
        Bone("prop", -1, (maniVec3){1.0f, 0.0f, 0.0f}, none, 1.0f)};
    maniBuilderDef def = maniDefaultBuilderDef();
    maniSkeletonSource source = {bones, kBones};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success &&
              maniBuildSkeleton(s_builder, &source).result == mani_success &&
              maniCopyBuilt(s_builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "a skeleton");
    const char* names[kBones] = {"pelvis",   "spine",    "chest", "head",
                                 "upperArm", "lowerArm", "hand",  "prop"};
    for (int k = 0; k < kBones; ++k)
    {
        s_bone[k] = Find(names[k]);
    }
    CHECK(maniPoseSize(&s_skeleton) <= sizeof s_poseMemory &&
              maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &s_pose) ==
                  mani_success &&
              maniPoseSetRest(s_pose, &s_skeleton) == mani_success &&
              maniComputeModel(s_pose, &s_skeleton) == mani_success,
          "the rest pose");
}

// The source, kept here so a test may change any part.
static maniRagdollBodySource s_bodies[5];

static maniRagdollJointSource Joint(maniVec3 at, maniQuat turn)
{
    return (maniRagdollJointSource){.translation = at,
                                    .rotation = turn,
                                    .twistMin = -0.5f,
                                    .twistMax = 0.75f,
                                    .swingY = 0.6f,
                                    .swingZ = 0.4f,
                                    .driveStiffness = 200.0f,
                                    .driveDamping = 20.0f,
                                    .driveMaxTorque = 500.0f};
}

static maniRagdollBodySource Body(const char* name, uint32_t bone, maniRagdollShape shape,
                                  maniVec3 size, float mass)
{
    return (maniRagdollBodySource){.name = name,
                                   .nameLength = (uint32_t)strlen(name),
                                   .bone = bone,
                                   .frameTranslation = {0.0f, 0.05f, 0.0f},
                                   .frameRotation = {0.0f, 0.0f, 0.0f, 1.0f},
                                   .shape = shape,
                                   .size = size,
                                   .mass = mass,
                                   .joint =
                                       Joint((maniVec3){0.0f, -0.05f, 0.0f},
                                             (maniQuat){0.0f, 0.0f, 0.70710678f, 0.70710678f})};
}

static maniRagdollSource Source(void)
{
    // Out of the bones' order, so the build sorts them.
    s_bodies[0] =
        Body("head", s_bone[kHead], maniRagdollSphere, (maniVec3){0.1f, 0.0f, 0.0f}, 5.0f);
    s_bodies[1] = Body("lowerArm", s_bone[kLowerArm], maniRagdollCapsule,
                       (maniVec3){0.04f, 0.2f, 0.0f}, 1.5f);
    s_bodies[2] =
        Body("pelvis", s_bone[kPelvis], maniRagdollBox, (maniVec3){0.15f, 0.1f, 0.1f}, 12.0f);
    s_bodies[3] =
        Body("chest", s_bone[kChest], maniRagdollBox, (maniVec3){0.16f, 0.15f, 0.1f}, 15.0f);
    s_bodies[4] = Body("upperArm", s_bone[kUpperArm], maniRagdollCapsule,
                       (maniVec3){0.05f, 0.2f, 0.0f}, 2.0f);
    s_bodies[3].frameRotation = (maniQuat){0.0f, 0.38268343f, 0.0f, 0.92387953f};
    s_bodies[4].frameTranslation = (maniVec3){0.01f, 0.14f, -0.02f};
    s_bodies[1].joint.translation = (maniVec3){0.02f, -0.1f, 0.01f};
    s_bodies[2].joint = (maniRagdollJointSource){0}; // a root's joint is ignored
    return (maniRagdollSource){&s_skeleton, s_bodies, 5, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}};
}

static maniBuildResult Build(const maniRagdollSource* source)
{
    maniBuildResult r = maniBuildRagdoll(s_builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(s_builder, s_file, sizeof s_file, &s_size) == mani_success, "copied");
    }
    return r;
}

#endif // MAUL_ANIM_TEST_RAGDOLL_FIXTURE_H
