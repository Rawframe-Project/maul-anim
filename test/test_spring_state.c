// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring states (mani-0026) on the fixture's chain: the colliders' and
// limits' cases, the step bound, reset, save and restore with every
// check of the saved bytes, the calls' refusals, the work area's size
// and the inputs a call keeps for the next.

#include "spring_fixture.h"
#include "spring_shape.h"
#include "spring_state.h"

#include "maul-anim/blend.h"

static _Alignas(32) uint8_t s_differenceMemory[4096];

static void TestStepBound(void)
{
    MakeSet(&kSwing);
    maniSpringState* a = NewState(0);
    maniSpringState* b = NewState(1);
    CHECK(Frame(a, 10.0f, NULL) && Frame(b, 20.0f / 60.0f, NULL), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveSpringState(b, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], maniSpringSavedSize(&s_set)) == 0,
          "a long delta plays as the bound's steps");
    // Either sign of delta moves on alike.
    a = NewState(0);
    b = NewState(1);
    CHECK(Frame(a, -0.05f, NULL) && Frame(b, 0.05f, NULL), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveSpringState(b, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], maniSpringSavedSize(&s_set)) == 0,
          "a negative delta moves on by its magnitude");
}

static void TestReset(void)
{
    MakeSet(&kSwing);
    maniSpringState* a = NewState(0);
    maniSpringState* b = NewState(1);
    CHECK(Frames(a, 1.0f / 60.0f, 50), "frames");
    maniResetSprings(a);
    maniResetSprings(NULL);
    CHECK(Frames(a, 1.0f / 60.0f, 1) && Frames(b, 1.0f / 60.0f, 1), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], sizeof s_saved[0]) == mani_success &&
              maniSaveSpringState(b, s_saved[1], sizeof s_saved[1]) == mani_success &&
              memcmp(s_saved[0], s_saved[1], maniSpringSavedSize(&s_set)) == 0,
          "a reset state starts over");
}

// Restoring the saved bytes with one word changed fails at want, and
// leaves the state as it was.
static bool RefusedAt(maniSpringState* state, uint64_t at, uint32_t value, uint64_t want)
{
    size_t size = maniSpringSavedSize(&s_set);
    static uint8_t s_before[4096];
    CHECK(maniSaveSpringState(state, s_before, sizeof s_before) == mani_success, "saved");
    memcpy(s_saved[1], s_saved[0], size);
    maniStoreU32(s_saved[1] + at, value);
    maniFileResult r = maniRestoreSpringState(state, s_saved[1], size);
    uint8_t after[4096];
    CHECK(maniSaveSpringState(state, after, sizeof after) == mani_success, "saved");
    return r.result == mani_errorFormat && r.offset == want && memcmp(after, s_before, size) == 0;
}

static uint32_t Bits(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

static void TestSaveRestore(void)
{
    Config c = {0.5f, 1.0f, 0.2f, maniSpringLimitNone, 0.0f, s_root, 0, {0.0f, 0.0f, 0.0f},
                0.0f, 0.0f};
    MakeSet(&c);
    maniSpringState* a = NewState(0);
    maniSpringState* b = NewState(1);
    size_t size = maniSpringSavedSize(&s_set);
    CHECK(size == 24 + 4 * (18 + 2 * 17 + 10 * 4) && maniSpringSavedSize(NULL) == 0,
          "the saved size");
    CHECK(Frames(a, 1.0f / 50.0f, 30), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], size - 1) == mani_errorCapacity &&
              maniSaveSpringState(a, s_saved[0], size) == mani_success &&
              maniRestoreSpringState(b, s_saved[0], size).result == mani_success,
          "saved and restored");
    CHECK(Frames(a, 1.0f / 50.0f, 30) && Frames(b, 1.0f / 50.0f, 30), "frames");
    uint8_t other[4096];
    CHECK(maniSaveSpringState(a, s_saved[0], size) == mani_success &&
              maniSaveSpringState(b, other, size) == mani_success &&
              memcmp(s_saved[0], other, size) == 0,
          "a restored state moves as the saved one");
    // The header, then the block's words (byte 24 + 4 * word).
    uint64_t joint1 = 24 + 4 * (18 + 17);
    uint64_t collider = 24 + 4 * (18 + 2 * 17);
    uint64_t center = collider + 4 * 30;
    CHECK(RefusedAt(b, 0, 0, 0) && RefusedAt(b, 4, 1, 4) && RefusedAt(b, 8, 1, 8) &&
              RefusedAt(b, 16, (uint32_t)size + 4, 16) && RefusedAt(b, 24, 2, 24) &&
              RefusedAt(b, 28, 1, 28) && RefusedAt(b, 36, 0x7FF00000u, 32) &&
              RefusedAt(b, 36, 0xBFF00000u, 32) && RefusedAt(b, 36, 0x3FA00000u, 32),
          "the header, motion and clock checked");
    CHECK(RefusedAt(b, 24 + 4 * 7, Bits(2.0f), 24 + 4 * 7) &&
              RefusedAt(b, 24 + 4 * 11, Bits(NAN), 24 + 4 * 11) &&
              RefusedAt(b, 24 + 4 * 17, Bits(0.5f), 24 + 4 * 14) &&
              RefusedAt(b, 24 + 4 * 8, Bits(NAN), 24 + 4 * 8) &&
              RefusedAt(b, joint1 + 4 * 13, Bits(INFINITY), joint1 + 4 * 13) &&
              RefusedAt(b, collider + 4 * 1, Bits(NAN), collider + 4 * 1) &&
              RefusedAt(b, collider + 4 * 13, Bits(NAN), collider + 4 * 13) &&
              RefusedAt(b, joint1, Bits(INFINITY), joint1) &&
              RefusedAt(b, joint1 + 4 * 9, Bits(0.5f), joint1 + 4 * 9) &&
              RefusedAt(b, joint1 + 4 * 13, Bits(-1.0f), joint1 + 4 * 13) &&
              RefusedAt(b, joint1 + 4 * 16, Bits(NAN), joint1 + 4 * 16) &&
              RefusedAt(b, center + 4 * 9, Bits(NAN), center + 4 * 9),
          "every value checked");
    // Without motion every word is 0.
    maniResetSprings(b);
    CHECK(maniSaveSpringState(b, s_saved[0], size) == mani_success &&
              RefusedAt(b, 24 + 4 * 30, 1, 24 + 4 * 30),
          "a state without motion holds only zeros");
    CHECK(maniRestoreSpringState(b, s_saved[0], 23).offset == 0 &&
              maniRestoreSpringState(b, s_saved[0], size - 1).offset == 16 &&
              maniRestoreSpringState(NULL, s_saved[0], size).result == mani_errorInvalid &&
              maniRestoreSpringState(b, NULL, size).result == mani_errorInvalid &&
              maniSaveSpringState(NULL, s_saved[0], size) == mani_errorInvalid &&
              maniSaveSpringState(b, NULL, size) == mani_errorInvalid,
          "the sizes and arguments checked");
    // A spring without a center keeps zeros in its center's words.
    MakeSet(&kSwing);
    a = NewState(0);
    CHECK(Frames(a, 1.0f / 60.0f, 2), "frames");
    CHECK(maniSaveSpringState(a, s_saved[0], size) == mani_success &&
              RefusedAt(a, center + 4 * 3, Bits(1.0f), center + 4 * 3),
          "no center, no transform");
}

static void TestRefusals(void)
{
    MakeSet(&kSwing);
    maniSpringState* state = NULL;
    size_t size = maniSpringStateSize(&s_set);
    maniSpringSet other = s_set;
    other.skeletonIdentity ^= 1u;
    maniSpringSet more = s_set;
    more.boneCount += 1;
    // An odd count of words still gives a size of whole alignments.
    maniSpringSet odd = s_set;
    odd.jointCount = 3;
    CHECK(maniSpringStateSize(&odd) % MANI_SPRING_STATE_ALIGNMENT == 0, "rounded up");
    CHECK(size > 0 && size % MANI_SPRING_STATE_ALIGNMENT == 0 && maniSpringStateSize(NULL) == 0,
          "the state's size, a multiple of its alignment");
    // The block, then the work area: per joint its head, per collider
    // and spring a bone, per collider a shape.
    uint64_t work = 14u * odd.jointCount + 10u * odd.colliderCount + 10u * odd.springCount +
                    9u * odd.colliderCount;
    CHECK(maniSpringWorkWords(&odd) == work &&
              maniSpringStateSize(&odd) >=
                  sizeof(maniSpringState) + 4 * (size_t)(maniSpringBlockWords(&odd) + work),
          "room for the block and the work area");
    CHECK(maniInitSpringState(&more, &s_skeleton, s_memory[0], size, &state) == mani_errorInvalid,
          "a set of more bones than the skeleton");
    CHECK(maniInitSpringState(NULL, &s_skeleton, s_memory[0], size, &state) == mani_errorInvalid &&
              maniInitSpringState(&s_set, NULL, s_memory[0], size, &state) == mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, NULL, size, &state) == mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[0], size, NULL) ==
                  mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[0] + 4, size, &state) ==
                  mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_lone, s_memory[0], size, &state) ==
                  mani_errorInvalid &&
              maniInitSpringState(&other, &s_skeleton, s_memory[0], size, &state) ==
                  mani_errorInvalid &&
              maniInitSpringState(&s_set, &s_skeleton, s_memory[0], size - 1, &state) ==
                  mani_errorCapacity &&
              state == NULL,
          "making a state checked");
    state = NewState(0);
    maniTransform bad = {{NAN, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
    maniTransform flat = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}};
    CHECK(maniStepSprings(NULL, s_pose, NULL, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, NULL, NULL, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, NULL, NAN) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, NULL, INFINITY) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, &bad, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, s_pose, &flat, 0.1f) == mani_errorInvalid &&
              maniStepSprings(state, s_lonePose, NULL, 0.1f) == mani_errorInvalid &&
              maniApplySprings(NULL, s_pose) == mani_errorInvalid &&
              maniApplySprings(state, NULL) == mani_errorInvalid &&
              maniApplySprings(state, s_lonePose) == mani_errorInvalid,
          "the calls' arguments checked");
    maniPose* difference = NULL;
    CHECK(maniInitPose(&s_skeleton, s_differenceMemory, sizeof s_differenceMemory, &difference) ==
                  mani_success &&
              maniPoseDifference(&s_skeleton, s_pose, s_pose, difference) == mani_success &&
              maniStepSprings(state, difference, NULL, 0.1f) == mani_errorInvalid &&
              maniApplySprings(state, difference) == mani_errorInvalid,
          "a difference refused");
    // Applying before any step leaves the pose; a stale model is composed.
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    maniTransform local = maniPoseLocal(s_pose, s_b[1]);
    maniPoseSetLocal(s_pose, s_root, maniPoseLocal(s_pose, s_root));
    CHECK(maniApplySprings(state, s_pose) == mani_success, "applied");
    maniTransform after = maniPoseLocal(s_pose, s_b[1]);
    CHECK(memcmp(&local, &after, sizeof local) == 0 && maniPoseModelCurrent(s_pose),
          "no motion, no change");
}

static bool Near3(maniVec3 a, maniVec3 b)
{
    return Distance(a, b) < 1e-6f;
}

// A collision's direction and distance, or no collision.
static bool Hits(maniSpringShape shape, maniVec3 tail, maniVec3 direction, float distance)
{
    maniVec3 d = {0.0f, 0.0f, 0.0f};
    float at = 0.0f;
    return maniSpringCollide(&shape, tail, 0.1f, &d, &at) && Near3(d, direction) &&
           fabsf(at - distance) < 1e-6f;
}

static bool Misses(maniSpringShape shape, maniVec3 tail)
{
    maniVec3 d = {0.0f, 0.0f, 0.0f};
    float at = 0.0f;
    return !maniSpringCollide(&shape, tail, 0.1f, &d, &at);
}

static void TestShapes(void)
{
    const maniVec3 o = {0.0f, 0.0f, 0.0f};
    maniSpringShape ball = {o, o, 1.0f, maniColliderSphere, 0};
    CHECK(Misses(ball, (maniVec3){1.5f, 0.0f, 0.0f}) &&
              Hits(ball, (maniVec3){0.5f, 0.0f, 0.0f}, (maniVec3){1.0f, 0.0f, 0.0f}, -0.6f) &&
              Hits(ball, o, (maniVec3){0.0f, 1.0f, 0.0f}, -1.1f),
          "a sphere pushes out, up from its center");
    ball.inside = 1;
    CHECK(Misses(ball, (maniVec3){0.5f, 0.0f, 0.0f}) &&
              Hits(ball, (maniVec3){0.0f, 0.0f, 0.95f}, (maniVec3){0.0f, 0.0f, -1.0f}, -0.05f),
          "an inside sphere pulls in");
    maniSpringShape capsule = {o, {2.0f, 0.0f, 0.0f}, 0.5f, maniColliderCapsule, 0};
    CHECK(Misses(capsule, (maniVec3){1.0f, 0.7f, 0.0f}) &&
              Hits(capsule, (maniVec3){1.0f, 0.3f, 0.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.3f) &&
              Hits(capsule, (maniVec3){-0.3f, 0.0f, 0.0f}, (maniVec3){-1.0f, 0.0f, 0.0f}, -0.3f) &&
              Hits(capsule, (maniVec3){2.0f, 0.0f, 0.4f}, (maniVec3){0.0f, 0.0f, 1.0f}, -0.2f) &&
              Hits(capsule, (maniVec3){2.3f, 0.0f, 0.0f}, (maniVec3){1.0f, 0.0f, 0.0f}, -0.3f),
          "a capsule pushes out from its segment");
    capsule.inside = 1;
    CHECK(Misses(capsule, (maniVec3){1.0f, 0.3f, 0.0f}) &&
              Hits(capsule, (maniVec3){1.0f, -0.45f, 0.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.05f),
          "an inside capsule pulls in");
    maniSpringShape point = {o, o, 0.5f, maniColliderCapsule, 0};
    CHECK(Hits(point, (maniVec3){0.0f, 0.0f, 0.3f}, (maniVec3){0.0f, 0.0f, 1.0f}, -0.3f),
          "a capsule of no length is a sphere");
    maniSpringShape plane = {{0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.0f, maniColliderPlane, 0};
    CHECK(Misses(plane, (maniVec3){0.0f, 1.2f, 0.0f}) &&
              Hits(plane, (maniVec3){3.0f, 1.05f, 2.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.05f) &&
              Hits(plane, (maniVec3){0.0f, 0.5f, 0.0f}, (maniVec3){0.0f, 1.0f, 0.0f}, -0.6f),
          "a plane pushes along its normal");
}

static bool Limited(maniSpringLimit limit, float angle, float yaw, maniVec3 in, maniVec3 out)
{
    maniVec3 got = maniSpringLimitDirection(limit, angle, yaw, in);
    return Distance(got, out) < 1e-5f;
}

static void TestLimitDirections(void)
{
    const maniVec3 x = {1.0f, 0.0f, 0.0f};
    const maniVec3 y = {0.0f, 1.0f, 0.0f};
    const maniVec3 z = {0.0f, 0.0f, 1.0f};
    const maniVec3 down = {0.0f, -1.0f, 0.0f};
    float c = cosf(0.5f);
    float s = sinf(0.5f);
    CHECK(Limited(maniSpringLimitNone, 0.5f, 0.0f, down, down) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, y, y) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, x, (maniVec3){s, c, 0.0f}) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, down, (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, (maniVec3){0.6f, -0.8f, 0.0f},
                      (maniVec3){s, c, 0.0f}) &&
              Limited(maniSpringLimitCone, 0.5f, 0.0f, (maniVec3){0.0f, 0.95f, 0.31224990f},
                      (maniVec3){0.0f, 0.95f, 0.31224990f}),
          "a cone");
    CHECK(Limited(maniSpringLimitHinge, 0.5f, 0.0f, x, y) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.6f, 0.8f, 0.0f}, y) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.0f, 0.0f, -1.0f},
                      (maniVec3){0.0f, c, -s}) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.6f, 0.0f, 0.8f},
                      (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitHinge, 0.5f, 0.0f, (maniVec3){0.0f, 0.95f, -0.31224990f},
                      (maniVec3){0.0f, 0.95f, -0.31224990f}),
          "a hinge in the yz plane");
    float cy = cosf(0.3f);
    CHECK(Limited(maniSpringLimitSpherical, 0.5f, 0.3f, y, y) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, z, (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, (maniVec3){0.0f, 0.0f, -1.0f},
                      (maniVec3){0.0f, c, -s}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, x, (maniVec3){sinf(0.3f), cy, 0.0f}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, (maniVec3){-1.0f, 0.0f, 0.0f},
                      (maniVec3){-sinf(0.3f), cy, 0.0f}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, down, (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitSpherical, 0.5f, 0.3f, (maniVec3){0.0f, -1.0f, -0.0f},
                      (maniVec3){0.0f, c, s}) &&
              Limited(maniSpringLimitSpherical, 3.0f, 1.5f, (maniVec3){0.6f, 0.8f, 0.0f},
                      (maniVec3){0.6f, 0.8f, 0.0f}),
          "a spherical limit's pitch and yaw");
}

static void TestScaleWords(void)
{
    // A head's scale is its model scale's largest, whichever axis.
    MakeSet(&kSwing);
    maniSpringState* state = NewState(0);
    s_rootLocal.scale = (maniVec3){1.0f, 2.0f, 3.0f};
    CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
    float z = SavedFloat(state, 18 + 13);
    s_rootLocal.scale = (maniVec3){1.0f, 3.0f, 2.0f};
    CHECK(Frame(state, 1.0f / 60.0f, NULL), "a frame");
    float y = SavedFloat(state, 18 + 13);
    Still();
    CHECK(fabsf(z - 3.0f) < 1e-5f && fabsf(y - 3.0f) < 1e-5f, "the largest scale");
}

static void TestClockRounding(void)
{
    // A clock and a delta whose sum rounds to a step though the step's
    // time is past the delta: the time is kept just short of the step,
    // which comes at the next call, not lost.
    MakeSet(&kSwing);
    maniSpringState* state = NewState(0);
    CHECK(Frames(state, 1.0f / 60.0f, 3), "frames");
    size_t size = maniSpringSavedSize(&s_set);
    CHECK(maniSaveSpringState(state, s_saved[0], size) == mani_success, "saved");
    double clock = 0.0020946870247522975;
    uint64_t bits = 0;
    memcpy(&bits, &clock, sizeof bits);
    maniStoreU64(s_saved[0] + 24 + 8, bits);
    CHECK(maniRestoreSpringState(state, s_saved[0], size).result == mani_success, "restored");
    CHECK(Frame(state, 0.014571979641914368f, NULL), "a frame");
    CHECK(maniSaveSpringState(state, s_saved[1], size) == mani_success, "saved");
    memcpy(&clock, s_saved[1] + 24 + 8, sizeof clock);
    CHECK(clock > 0.0166 && clock < 1.0 / 60.0, "the clock just short of a step");
    CHECK(maniRestoreSpringState(state, s_saved[1], size).result == mani_success, "and restorable");
}

// A call keeps the pose it read as the next call's last inputs: each
// joint's head position, rotation and largest scale and its tail's
// position, each collider's bone, as the pose's model space holds them.
static void TestKeptInputs(void)
{
    Config c = kSwing;
    c.collider = 1;
    MakeSet(&c);
    maniSpringState* state = NewState(0);
    CHECK(maniPoseSetRest(s_pose, &s_skeleton) == mani_success, "rest");
    maniTransform b1 = maniPoseLocal(s_pose, s_b[1]);
    b1.rotation = (maniQuat){0.0f, sinf(0.3f), 0.0f, cosf(0.3f)};
    b1.scale = (maniVec3){1.0f, 1.5f, 1.0f};
    maniPoseSetLocal(s_pose, s_b[1], b1);
    CHECK(maniStepSprings(state, s_pose, NULL, 1.0f / 60.0f) == mani_success, "stepped");
    const uint32_t* block = maniSpringConstBlock(state);
    bool same = true;
    for (uint32_t j = 0; j < s_set.jointCount; ++j)
    {
        const uint8_t* e = s_set.joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
        maniTransform head = maniPoseModelTransform(s_pose, maniLoadU32(e + maniSpringJointHead));
        maniAffine tail = maniPoseModel(s_pose, maniLoadU32(e + maniSpringJointTail));
        uint64_t at = maniSpringJointAt(j);
        maniVec3 p = maniSpringVec(block, at + maniSpringHeadPosition);
        maniQuat q = maniSpringQuat(block, at + maniSpringHeadRotation);
        maniVec3 t = maniSpringVec(block, at + maniSpringTailPosition);
        float largest =
            fabsf(head.scale.x) > fabsf(head.scale.y) ? fabsf(head.scale.x) : fabsf(head.scale.y);
        largest = fabsf(head.scale.z) > largest ? fabsf(head.scale.z) : largest;
        same = same && memcmp(&p, &head.translation, sizeof p) == 0 &&
               memcmp(&q, &head.rotation, sizeof q) == 0 &&
               maniSpringWord(block, at + maniSpringHeadScale) == largest && t.x == tail.m[9] &&
               t.y == tail.m[10] && t.z == tail.m[11];
    }
    maniTransform bone = maniPoseModelTransform(s_pose, maniLoadU32(s_set.colliders));
    maniVec3 kept = maniSpringVec(block, maniSpringColliderAt(&s_set, 0) + 7);
    CHECK(same && memcmp(&kept, &bone.scale, sizeof kept) == 0, "the inputs kept as read");
}

int main(void)
{
    Setup();
    TestShapes();
    TestLimitDirections();
    TestStepBound();
    TestReset();
    TestSaveRestore();
    TestRefusals();
    TestScaleWords();
    TestClockRounding();
    TestKeptInputs();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
