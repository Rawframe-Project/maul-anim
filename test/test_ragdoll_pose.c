// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Ragdoll bodies and poses (mani-0027) on the fixture's humanoid: bodies
// placed from a bent pose under a turned placement and a pose made back
// from them (every local within float rounding, the bones without bodies
// the reference's exactly, the model current); velocities from the pose
// before (a moved placement, a turned elbow); joint targets at rest (the
// identity), for a bent elbow (its angle) and from bodies' angular
// velocities; the refusals.

#include "pose_memory.h"
#include "ragdoll_fixture.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/ragdoll.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_memory[3][8192];
static maniPose* s_poses[3];
static maniRagdoll s_ragdoll;

static const maniTransform kPlace = {
    {2.0f, 0.5f, -1.0f}, {0.0f, 0.38268343f, 0.0f, 0.92387953f}, {1.0f, 1.0f, 1.0f}};

static maniQuat Turn(maniVec3 axis, float angle)
{
    float s = sinf(0.5f * angle);
    return (maniQuat){axis.x * s, axis.y * s, axis.z * s, cosf(0.5f * angle)};
}

static maniQuat Mul(maniQuat a, maniQuat b)
{
    return (maniQuat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// Pose k: the rest pose with the chest, upper arm and elbow bent by
// angle (and the spine and hand too, which have no bodies).
static maniPose* Bent(int k, float angle)
{
    maniPose* p = s_poses[k];
    CHECK(maniPoseSetRest(p, &s_skeleton) == mani_success, "rest");
    const int bones[5] = {kSpine, kChest, kUpperArm, kLowerArm, kHand};
    const maniVec3 axes[5] = {{1.0f, 0.0f, 0.0f},
                              {0.0f, 0.0f, 1.0f},
                              {0.6f, 0.0f, 0.8f},
                              {1.0f, 0.0f, 0.0f},
                              {0.0f, 1.0f, 0.0f}};
    for (int i = 0; i < 5; ++i)
    {
        maniTransform t = maniPoseLocal(p, s_bone[bones[i]]);
        t.rotation = Mul(t.rotation, Turn(axes[i], angle * (float)(i + 1) * 0.3f));
        maniPoseSetLocal(p, s_bone[bones[i]], t);
    }
    CHECK(maniComputeModel(p, &s_skeleton) == mani_success, "the model");
    return p;
}

static float QuatApart(maniQuat a, maniQuat b)
{
    return 1.0f - fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

static float VecApart(maniVec3 a, maniVec3 b)
{
    return fabsf(a.x - b.x) + fabsf(a.y - b.y) + fabsf(a.z - b.z);
}

static float Length(maniVec3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static void Setup(void)
{
    OpenSkeleton();
    maniRagdollSource source = Source();
    CHECK(Build(&source).result == mani_success &&
              maniOpenRagdoll(s_file, s_size, &s_ragdoll).result == mani_success,
          "a profile");
    for (int k = 0; k < 3; ++k)
    {
        CHECK(maniInitPose(&s_skeleton, s_memory[k], sizeof s_memory[k], &s_poses[k]) ==
                  mani_success,
              "a pose");
    }
}

static void TestRoundTrip(void)
{
    maniPose* bent = Bent(0, 0.7f);
    maniRagdollBodyState bodies[5];
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, bent, &kPlace, NULL, NULL, 0.0f, bodies, 5) ==
              mani_success,
          "bodies placed");
    // Back over the same pose as the reference: every local again.
    CHECK(maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, bodies, 5, &kPlace, bent,
                                    s_poses[1]) == mani_success &&
              maniPoseModelCurrent(s_poses[1]),
          "a pose from them");
    float worst = 0.0f;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform x = maniPoseLocal(bent, b);
        maniTransform y = maniPoseLocal(s_poses[1], b);
        worst = fmaxf(worst, VecApart(x.translation, y.translation) +
                                 QuatApart(x.rotation, y.rotation) + VecApart(x.scale, y.scale));
    }
    CHECK(worst < 2e-6f, "each local back within rounding");
    // Over the rest pose: the spine and hand the rest's exactly, and the
    // bodies where they were.
    maniPose* rest = s_poses[2];
    CHECK(maniPoseSetRest(rest, &s_skeleton) == mani_success, "rest");
    CHECK(maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, bodies, 5, &kPlace, rest,
                                    s_poses[1]) == mani_success,
          "a pose over the rest");
    maniTransform spine = maniPoseLocal(s_poses[1], s_bone[kSpine]);
    maniTransform rested = maniPoseLocal(rest, s_bone[kSpine]);
    maniTransform hand = maniPoseLocal(s_poses[1], s_bone[kHand]);
    maniTransform restHand = maniPoseLocal(rest, s_bone[kHand]);
    CHECK(memcmp(&spine, &rested, sizeof spine) == 0 &&
              memcmp(&hand, &restHand, sizeof hand) == 0 && spine.scale.x == 1.5f,
          "bones without bodies keep the reference's locals");
    maniRagdollBodyState again[5];
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, s_poses[1], &kPlace, NULL, NULL, 0.0f, again, 5) ==
              mani_success,
          "placed again");
    worst = 0.0f;
    for (int i = 0; i < 5; ++i)
    {
        worst = fmaxf(worst, VecApart(bodies[i].position, again[i].position) +
                                 QuatApart(bodies[i].rotation, again[i].rotation));
    }
    CHECK(worst < 4e-6f, "the bodies where they were");
    // In place: the reference the pose made.
    CHECK(maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, bodies, 5, &kPlace, s_poses[1],
                                    s_poses[1]) == mani_success,
          "a pose from bodies in place");
}

static void TestVelocities(void)
{
    maniPose* now = Bent(0, 0.5f);
    maniPose* then = Bent(1, 0.5f);
    // The elbow 0.02 radians less bent before, the character 0.03 m back.
    maniTransform elbow = maniPoseLocal(then, s_bone[kLowerArm]);
    elbow.rotation = Mul(elbow.rotation, Turn((maniVec3){1.0f, 0.0f, 0.0f}, -0.02f));
    maniPoseSetLocal(then, s_bone[kLowerArm], elbow);
    CHECK(maniComputeModel(then, &s_skeleton) == mani_success, "the model");
    maniTransform before = kPlace;
    before.translation.x -= 0.03f;
    maniRagdollBodyState bodies[5];
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, now, &kPlace, then, &before, 0.1f, bodies, 5) ==
              mani_success,
          "bodies with velocities");
    bool moving = true;
    for (int i = 0; i < 4; ++i)
    {
        moving = moving &&
                 VecApart(bodies[i].linearVelocity, (maniVec3){0.3f, 0.0f, 0.0f}) < 1e-4f &&
                 VecApart(bodies[i].angularVelocity, (maniVec3){0.0f, 0.0f, 0.0f}) < 1e-4f;
    }
    float spin = Length(bodies[4].angularVelocity);
    CHECK(moving && fabsf(spin - 0.2f) < 1e-3f, "moved with the placement, the elbow turning");
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, now, &kPlace, NULL, &before, 0.1f, bodies, 5) ==
                  mani_success &&
              bodies[4].angularVelocity.x == 0.0f && bodies[2].linearVelocity.x == 0.0f,
          "no velocities without the pose before");
}

static void TestJointTargets(void)
{
    maniRagdollBodyState bodies[5];
    maniRagdollJointTarget joints[5];
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, Bent(0, 0.0f), &kPlace, NULL, NULL, 0.0f, bodies,
                                    5) == mani_success &&
              maniRagdollJointTargets(&s_ragdoll, bodies, joints, 5) == mani_success,
          "targets at rest");
    float worst = 0.0f;
    for (int i = 0; i < 5; ++i)
    {
        worst = fmaxf(worst, QuatApart(joints[i].rotation, (maniQuat){0.0f, 0.0f, 0.0f, 1.0f}));
    }
    CHECK(worst < 1e-6f, "every joint at rest in the rest pose");
    // The elbow bent by 1.2 radians (0.3 * 4 of Bent's): its joint turned
    // by as much.
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, Bent(0, 1.0f), &kPlace, NULL, NULL, 0.0f, bodies,
                                    5) == mani_success &&
              maniRagdollJointTargets(&s_ragdoll, bodies, joints, 5) == mani_success,
          "targets bent");
    float angle = 2.0f * acosf(fminf(1.0f, joints[4].rotation.w));
    CHECK(fabsf(angle - 1.2f) < 1e-4f && joints[4].rotation.w >= 0.0f, "the elbow's angle");
    bodies[4].angularVelocity = (maniVec3){0.0f, 3.0f, 0.0f};
    bodies[3].angularVelocity = (maniVec3){0.0f, 1.0f, 0.0f};
    CHECK(maniRagdollJointTargets(&s_ragdoll, bodies, joints, 5) == mani_success, "spinning");
    maniVec3 w = joints[4].angularVelocity;
    CHECK(fabsf(Length(w) - 2.0f) < 1e-5f && joints[2].angularVelocity.y == 0.0f &&
              fabsf(Length(joints[3].angularVelocity) - 1.0f) < 1e-5f &&
              joints[0].rotation.w == 1.0f,
          "the child's spin less the parent's");
}

static void TestRefusals(void)
{
    maniPose* p = Bent(0, 0.3f);
    maniRagdollBodyState b[5];
    maniRagdollJointTarget j[5];
    maniTransform bad = kPlace;
    bad.translation.y = NAN;
    maniTransform turned = kPlace;
    turned.rotation.w = 2.0f;
    CHECK(maniRagdollBodiesFromPose(NULL, p, NULL, NULL, NULL, 0.0f, b, 5) == mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, NULL, NULL, NULL, NULL, 0.0f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, NULL, NULL, 0.0f, NULL, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, &bad, NULL, NULL, 0.0f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, &turned, NULL, NULL, 0.0f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, NULL, &bad, 0.0f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, p, NULL, 0.0f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, p, NULL, INFINITY, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, p, NULL, NAN, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, NULL, NULL, 0.0f, b, 4) ==
                  mani_errorCapacity,
          "bodies refused");
    maniPoseSetLocal(s_poses[1], 0, maniPoseLocal(s_poses[1], 0));
    maniPoseSetKind(s_poses[2], true);
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, s_poses[1], NULL, NULL, NULL, 0.0f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, s_poses[1], NULL, 0.1f, b, 5) ==
                  mani_errorInvalid &&
              maniRagdollBodiesFromPose(&s_ragdoll, s_poses[2], NULL, NULL, NULL, 0.0f, b, 5) ==
                  mani_errorInvalid,
          "a model not current, a difference");
    CHECK(maniRagdollJointTargets(NULL, b, j, 5) == mani_errorInvalid &&
              maniRagdollJointTargets(&s_ragdoll, NULL, j, 5) == mani_errorInvalid &&
              maniRagdollJointTargets(&s_ragdoll, b, NULL, 5) == mani_errorInvalid &&
              maniRagdollJointTargets(&s_ragdoll, b, j, 4) == mani_errorCapacity,
          "targets refused");
    CHECK(maniRagdollBodiesFromPose(&s_ragdoll, p, NULL, NULL, NULL, 0.0f, b, 5) == mani_success,
          "placed");
    maniPose* out = s_poses[1];
    CHECK(maniRagdollPoseFromBodies(NULL, &s_skeleton, b, 5, NULL, p, out) == mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, NULL, b, 5, NULL, p, out) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, NULL, 5, NULL, p, out) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, b, 5, NULL, NULL, out) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, b, 5, NULL, p, NULL) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, b, 5, NULL, s_poses[2], out) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, b, 5, NULL, p, s_poses[2]) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, b, 5, &bad, p, out) ==
                  mani_errorInvalid &&
              maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, b, 4, NULL, p, out) ==
                  mani_errorCapacity,
          "poses refused");
    const float kBad[3] = {NAN, INFINITY, 1.1f};
    bool refused = true;
    for (int k = 0; k < 3; ++k)
    {
        maniRagdollBodyState c[5];
        memcpy(c, b, sizeof c);
        c[k + 2].position.z = k < 2 ? kBad[k] : c[k + 2].position.z;
        c[k + 2].rotation.w = k == 2 ? kBad[k] : c[k + 2].rotation.w;
        refused = refused && maniRagdollPoseFromBodies(&s_ragdoll, &s_skeleton, c, 5, NULL, p,
                                                       out) == mani_errorInvalid;
    }
    CHECK(refused, "a body's place not finite or its rotation not of length 1");
    maniPoseSetKind(s_poses[2], false);
}

int main(void)
{
    Setup();
    TestRoundTrip();
    TestVelocities();
    TestJointTargets();
    TestRefusals();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
