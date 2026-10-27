// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Spring sets: a set of hair (three joints, a cone and a
// hinge limit), a tail in its hips' space and four colliders (a sphere,
// a capsule, a plane and an inside sphere) built, opened and read back;
// the per-frame values converted to the step for each drag and against the rates; the limits'
// frames and clamped angles; the builder's refusals with their reasons; opening's checks of every
// record against a file changed at its byte. Run as test_spring --seed
// <file> it writes the fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "spring_file.h"
#include "test_harness.h"

#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;
static maniSkeleton s_skeleton;
static maniBuilder* s_builder;

// Bones by name, as the skeleton placed them.
static uint32_t s_hips;
static uint32_t s_head;
static uint32_t s_hair[3];
static uint32_t s_tail[3];
static uint32_t s_arm;

static maniBoneSource Bone(const char* name, int32_t parent, float x, float y, float z)
{
    return (maniBoneSource){name,
                            (uint32_t)strlen(name),
                            parent,
                            {{x, y, z}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
}

static uint32_t Find(const char* name)
{
    return maniSkeletonFindBone(&s_skeleton, name, (uint32_t)strlen(name));
}

static void OpenSkeleton(void)
{
    // The hair hangs straight down from the head; the tail runs back
    // along -z; the arm out along +x.
    const maniBoneSource bones[9] = {
        Bone("hips", -1, 0.0f, 1.0f, 0.0f),  Bone("head", 0, 0.0f, 0.6f, 0.0f),
        Bone("hair0", 1, 0.0f, 0.1f, -0.1f), Bone("hair1", 2, 0.0f, -0.2f, 0.0f),
        Bone("hair2", 3, 0.0f, -0.2f, 0.0f), Bone("tail0", 0, 0.0f, 0.0f, -0.2f),
        Bone("tail1", 5, 0.0f, 0.0f, -0.3f), Bone("tail2", 6, 0.0f, 0.0f, -0.3f),
        Bone("arm", 0, 0.3f, 0.4f, 0.0f)};
    maniBuilderDef def = maniDefaultBuilderDef();
    maniSkeletonSource source = {bones, 9};
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success &&
              maniBuildSkeleton(s_builder, &source).result == mani_success &&
              maniCopyBuilt(s_builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "a skeleton");
    s_hips = Find("hips");
    s_head = Find("head");
    s_arm = Find("arm");
    const char* hair[3] = {"hair0", "hair1", "hair2"};
    const char* tail[3] = {"tail0", "tail1", "tail2"};
    for (int k = 0; k < 3; ++k)
    {
        s_hair[k] = Find(hair[k]);
        s_tail[k] = Find(tail[k]);
    }
}

// The set's source, kept here so a test may change any part.
static maniColliderSource s_colliders[4];
static uint32_t s_groupLists[2][3];
static maniColliderGroupSource s_groups[2];
static maniSpringJointSource s_hairJoints[3];
static maniSpringJointSource s_tailJoints[3];
static uint32_t s_springGroups[2][2];
static maniSpringSource s_springs[2];

static maniSpringJointSource Joint(uint32_t bone, float drag)
{
    return (maniSpringJointSource){.bone = bone,
                                   .hitRadius = 0.02f,
                                   .stiffness = 1.0f,
                                   .gravityPower = 0.5f,
                                   .gravityDirection = {0.0f, -1.0f, 0.0f},
                                   .drag = drag,
                                   .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
}

static maniSpringSetSource Source(void)
{
    s_colliders[0] =
        (maniColliderSource){s_head, maniColliderSphere, {0, 0.1f, 0}, {0, 0, 0}, 0.12f, 0};
    s_colliders[1] =
        (maniColliderSource){s_arm, maniColliderCapsule, {0, 0, 0}, {0.3f, 0, 0}, 0.05f, 0};
    s_colliders[2] =
        (maniColliderSource){s_hips, maniColliderPlane, {0, -0.9f, 0}, {0, 1, 0}, 0, 0};
    s_colliders[3] =
        (maniColliderSource){s_hips, maniColliderSphere, {0, 0, 0}, {0, 0, 0}, 2.0f, 1};
    const uint32_t lists[2][3] = {{0, 1, 0}, {1, 2, 3}};
    memcpy(s_groupLists, lists, sizeof lists);
    s_groups[0] = (maniColliderGroupSource){"head", 4, 3, s_groupLists[0]};
    s_groups[1] = (maniColliderGroupSource){"body", 4, 3, s_groupLists[1]};
    for (int k = 0; k < 3; ++k)
    {
        s_hairJoints[k] = Joint(s_hair[k], 0.4f);
        s_tailJoints[k] = Joint(s_tail[k], (float)k * 0.5f);
    }
    s_hairJoints[0].limit = maniSpringLimitCone;
    s_hairJoints[0].limitAngle = 0.5f;
    s_hairJoints[1].limit = maniSpringLimitHinge;
    s_hairJoints[1].limitAngle = 4.0f;
    s_hairJoints[1].limitRotation = (maniQuat){0.0f, 0.0f, 0.70710678f, 0.70710678f};
    s_tailJoints[0].limit = maniSpringLimitSpherical;
    s_tailJoints[0].limitAngle = 1.0f;
    s_tailJoints[0].limitYaw = 3.0f;
    s_tailJoints[2].limit = 7; // the last joint's limit is ignored
    const uint32_t groups[2][2] = {{0, 1}, {1, 0}};
    memcpy(s_springGroups, groups, sizeof groups);
    s_springs[0] =
        (maniSpringSource){"hairs", 5, 3, s_hairJoints, s_springGroups[0], 2, MANI_NO_BONE};
    s_springs[1] = (maniSpringSource){"tail", 4, 3, s_tailJoints, s_springGroups[1], 1, s_hips};
    return (maniSpringSetSource){&s_skeleton, s_colliders, s_groups, s_springs, 4, 2, 2, 0, 0};
}

static maniBuildResult Build(const maniSpringSetSource* source)
{
    maniBuildResult r = maniBuildSpringSet(s_builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(s_builder, s_file, sizeof s_file, &s_size) == mani_success, "copied");
    }
    return r;
}

static float LoadFloat(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static const uint8_t* JointRecord(const maniSpringSet* set, uint32_t j)
{
    return set->joints + (size_t)j * MANI_SPRING_JOINT_SIZE;
}

static void TestBuild(void)
{
    maniSpringSetSource source = Source();
    maniSpringSet set;
    CHECK(Build(&source).result == mani_success &&
              maniOpenSpringSet(s_file, s_size, &set).result == mani_success,
          "built and opened");
    float reference = 0.0f;
    uint32_t length = 0;
    const char* name = maniSpringSetSpringName(&set, 1, &length);
    CHECK(maniSpringSetSpringCount(&set) == 2 && set.jointCount == 4 && set.colliderCount == 4 &&
              maniSpringSetStepRate(&set, &reference) == 60.0f && reference == 60.0f &&
              maniSpringSetSkeletonIdentity(&set) == maniSkeletonIdentity(&s_skeleton) &&
              maniSpringSetIdentity(&set) != 0 && length == 4 && memcmp(name, "tail", 4) == 0,
          "its counts, rates, identities and names");
    CHECK(maniSpringSetSpringCount(NULL) == 0 && maniSpringSetIdentity(NULL) == 0 &&
              maniSpringSetSkeletonIdentity(NULL) == 0 &&
              maniSpringSetStepRate(NULL, &reference) == 0.0f && reference == 0.0f &&
              maniSpringSetSpringName(&set, 2, &length) == NULL && length == 0 &&
              maniSpringSetSpringName(NULL, 0, NULL) == NULL,
          "past the springs, and NULL");
    // The hair's colliders: its groups' in order, the head's sphere once.
    const uint8_t* hair = set.springs;
    const uint32_t want[4] = {0, 1, 2, 3};
    bool links = maniLoadU32(hair + 8) == 0 && maniLoadU32(hair + 12) == 4;
    for (uint32_t k = 0; k < 4 && links; ++k)
    {
        links = maniLoadU32(set.links + 4 * k) == want[k];
    }
    const uint8_t* tail = set.springs + MANI_SPRING_SPRING_SIZE;
    CHECK(links && maniLoadU32(tail + 8) == 4 && maniLoadU32(tail + 12) == 3 &&
              maniLoadU32(tail + 16) == s_hips && maniLoadU32(hair + 16) == MANI_NO_BONE,
          "each spring's colliders without repeats, and its center");
    const uint8_t* j = JointRecord(&set, 1);
    CHECK(maniLoadU32(j + maniSpringJointHead) == s_hair[1] &&
              maniLoadU32(j + maniSpringJointTail) == s_hair[2] &&
              LoadFloat(j + maniSpringJointDrag) == 0.4f &&
              LoadFloat(j + maniSpringJointStiffness) == (float)(1.0 / 60.0) &&
              LoadFloat(j + maniSpringJointGravity + 4) == -(float)(0.5 / 60.0),
          "at the reference's own rate: its values, stiffness and gravity a frame's");
    const uint8_t* plane = set.colliders + 2 * MANI_SPRING_COLLIDER_SIZE;
    CHECK(maniLoadU32(plane + 4) == maniColliderPlane && LoadFloat(plane + 32) == 1.0f &&
              maniLoadU32(set.colliders + 3 * MANI_SPRING_COLLIDER_SIZE + 8) == 1,
          "a plane's normal, an inside sphere");
}

// The step's drag and forcing as docs/internals/physics-seam.md gives them, in binary64.
static void Expected(double drag, double h, double f0, double* dh, double* forcing)
{
    *dh = drag >= 1.0 ? 1.0 : 1.0 - pow(1.0 - drag, h * f0);
    *forcing = h * (drag > 0.0 ? *dh / drag : h * f0);
}

static bool Near(float got, double want)
{
    return fabs((double)got - want) <= 1e-6 * fabs(want) + 1e-12;
}

static void TestConversion(void)
{
    const float rates[3][2] = {{60.0f, 120.0f}, {50.0f, 30.0f}, {60.0f, 0.0f}};
    for (int r = 0; r < 3; ++r)
    {
        maniSpringSetSource source = Source();
        source.referenceRate = rates[r][0];
        source.stepRate = rates[r][1];
        maniSpringSet set;
        CHECK(Build(&source).result == mani_success &&
                  maniOpenSpringSet(s_file, s_size, &set).result == mani_success,
              "built at other rates");
        double f0 = (double)rates[r][0];
        double h = 1.0 / (rates[r][1] == 0.0f ? f0 : (double)rates[r][1]);
        // The tail's joints: drags 0 and 0.5 (the last, 1, only a tail).
        for (uint32_t k = 0; k < 2; ++k)
        {
            double dh = 0.0;
            double forcing = 0.0;
            Expected(0.5 * k, h, f0, &dh, &forcing);
            const uint8_t* j = JointRecord(&set, 2 + k);
            CHECK(Near(LoadFloat(j + maniSpringJointDrag), dh) &&
                      Near(LoadFloat(j + maniSpringJointStiffness), forcing) &&
                      Near(LoadFloat(j + maniSpringJointGravity + 4), -0.5 * forcing) &&
                      Near(LoadFloat(j + maniSpringJointForcing), forcing),
                  "the drag, stiffness, gravity and forcing factor a step");
        }
    }
    maniSpringSetSource source = Source();
    source.stepRate = 120.0f;
    s_tailJoints[1].drag = 1.0f;
    maniSpringSet set;
    CHECK(Build(&source).result == mani_success &&
              maniOpenSpringSet(s_file, s_size, &set).result == mani_success &&
              LoadFloat(JointRecord(&set, 3) + maniSpringJointDrag) == 1.0f &&
              Near(LoadFloat(JointRecord(&set, 3) + maniSpringJointStiffness), 1.0 / 120.0) &&
              Near(LoadFloat(JointRecord(&set, 3) + maniSpringJointForcing), 1.0 / 120.0),
          "a drag of 1: nothing carried over, the forcing a step's");
}

static void TestLimits(void)
{
    maniSpringSetSource source = Source();
    maniSpringSet set;
    CHECK(Build(&source).result == mani_success &&
              maniOpenSpringSet(s_file, s_size, &set).result == mani_success,
          "built");
    // The hair hangs along -y: its default orientation is 180° about x.
    const uint8_t* cone = JointRecord(&set, 0);
    CHECK(maniLoadU32(cone + maniSpringJointLimit) == maniSpringLimitCone &&
              LoadFloat(cone + maniSpringJointAngle) == 0.5f &&
              LoadFloat(cone + maniSpringJointYaw) == 0.0f &&
              LoadFloat(cone + maniSpringJointFrame) == 1.0f &&
              LoadFloat(cone + maniSpringJointFrame + 12) == 0.0f,
          "a cone's angle and frame");
    const uint8_t* hinge = JointRecord(&set, 1);
    CHECK(LoadFloat(hinge + maniSpringJointAngle) == 3.14159265358979f &&
              Near(LoadFloat(hinge + maniSpringJointFrame), 0.70710678) &&
              Near(LoadFloat(hinge + maniSpringJointFrame + 4), -0.70710678) &&
              LoadFloat(hinge + maniSpringJointFrame + 12) == 0.0f,
          "a hinge's angle clamped to π, its frame the turn times the rotation");
    // The tail runs along -z: the turn from +y is -90° about x.
    const uint8_t* spherical = JointRecord(&set, 2);
    CHECK(LoadFloat(spherical + maniSpringJointAngle) == 1.0f &&
              LoadFloat(spherical + maniSpringJointYaw) == 3.14159265358979f / 2.0f &&
              Near(LoadFloat(spherical + maniSpringJointFrame), -0.70710678) &&
              Near(LoadFloat(spherical + maniSpringJointFrame + 12), 0.70710678),
          "a spherical limit's yaw clamped to π/2, its frame");
    const uint8_t* none = JointRecord(&set, 3);
    CHECK(maniLoadU32(none + maniSpringJointLimit) == 0 &&
              LoadFloat(none + maniSpringJointAngle) == 0.0f &&
              LoadFloat(none + maniSpringJointFrame + 12) == 1.0f,
          "no limit: the identity frame");
}

static bool Refused(const maniSpringSetSource* source, maniBuildReason reason, uint32_t item,
                    uint32_t other)
{
    maniBuildResult r = maniBuildSpringSet(s_builder, source);
    bool ok =
        r.result == mani_errorInvalid && r.reason == reason && r.item == item && r.other == other;
    if (!ok)
    {
        printf("  refused %d: reason %u item %u other %u\n", (int)r.result, r.reason, r.item,
               r.other);
    }
    return ok;
}

static void TestColliderRefusals(void)
{
    maniSpringSetSource s = Source();
    s_colliders[1].bone = 9;
    CHECK(Refused(&s, maniBuildReasonCollider, 1, 0), "a bone past the skeleton");
    s = Source();
    s_colliders[1].shape = 3;
    CHECK(Refused(&s, maniBuildReasonCollider, 1, 0), "an unknown shape");
    s = Source();
    s_colliders[2].inside = 1;
    CHECK(Refused(&s, maniBuildReasonCollider, 2, 0), "an inside plane");
    s = Source();
    s_colliders[3].inside = 2;
    CHECK(Refused(&s, maniBuildReasonCollider, 3, 0), "inside neither 0 nor 1");
    s = Source();
    s_colliders[0].radius = -0.1f;
    CHECK(Refused(&s, maniBuildReasonCollider, 0, 0), "a negative radius");
    s = Source();
    s_colliders[0].offset.y = INFINITY;
    CHECK(Refused(&s, maniBuildReasonCollider, 0, 0), "an offset not finite");
    s = Source();
    s_colliders[1].tail.z = NAN;
    CHECK(Refused(&s, maniBuildReasonCollider, 1, 0), "a tail not finite");
    s = Source();
    s_colliders[2].tail = (maniVec3){0.0f, 0.9f, 0.0f};
    CHECK(Refused(&s, maniBuildReasonCollider, 2, 0), "a plane's normal not of length 1");
    s = Source();
    s_colliders[2].tail = (maniVec3){0.0f, 1.0005f, 0.0f};
    maniSpringSet set;
    CHECK(Build(&s).result == mani_success &&
              maniOpenSpringSet(s_file, s_size, &set).result == mani_success &&
              LoadFloat(set.colliders + 2 * MANI_SPRING_COLLIDER_SIZE + 32) == 1.0f,
          "a normal within a thousandth, made unit");
    s = Source();
    s_groupLists[1][2] = 4;
    CHECK(Refused(&s, maniBuildReasonColliderGroup, 1, 2), "a group's collider past the list");
    s = Source();
    s_groups[0].colliders = NULL;
    CHECK(Refused(&s, maniBuildReasonColliderGroup, 0, 0), "a group's count without its list");
}

static void TestSpringRefusals(void)
{
    maniSpringSetSource s = Source();
    s_springs[0].jointCount = 1;
    CHECK(Refused(&s, maniBuildReasonSpring, 0, 0), "one joint");
    s = Source();
    s_hairJoints[2].bone = 9;
    CHECK(Refused(&s, maniBuildReasonSpring, 0, 2), "a joint past the skeleton");
    s = Source();
    s_hairJoints[1].bone = s_arm;
    CHECK(Refused(&s, maniBuildReasonSpring, 0, 1), "a joint not below the one before");
    s = Source();
    s_springGroups[1][0] = 2;
    CHECK(Refused(&s, maniBuildReasonSpring, 1, 3), "a group past the groups");
    s = Source();
    s_tailJoints[0].bone = s_hair[2];
    CHECK(Refused(&s, maniBuildReasonRepeatedJoint, 1, 0), "a joint of another spring");
    s = Source();
    s_springs[1].center = s_tail[1];
    CHECK(Refused(&s, maniBuildReasonSpring, 1, 3), "a center below the first joint");
    s = Source();
    s_springs[1].center = s_tail[0];
    CHECK(maniBuildSpringSet(s_builder, &s).result == mani_success, "the first joint a center");
    s = Source();
    s_springs[1].center = 9;
    CHECK(Refused(&s, maniBuildReasonSpring, 1, 3), "a center past the skeleton");
    // The hair from its second joint, centered on its first, which the
    // tail (now the head and the hair's first) holds.
    s = Source();
    s_springs[0].joints = &s_hairJoints[1];
    s_springs[0].jointCount = 2;
    s_springs[0].center = s_hair[0];
    s_springs[1].jointCount = 2;
    s_tailJoints[0].bone = s_head;
    s_tailJoints[1].bone = s_hair[0];
    CHECK(Refused(&s, maniBuildReasonSpring, 0, 2), "a center on another spring's joint");
    s = Source();
    s_springs[0].name = "\xff";
    s_springs[0].nameLength = 1;
    CHECK(Refused(&s, maniBuildReasonSpringList | maniBuildReasonName, 0, 0), "a name not UTF-8");
    // The hair from the head to its last bone, three bones apart.
    s = Source();
    s_hairJoints[1].bone = s_head;
    s_springs[0].joints = &s_hairJoints[1];
    s_springs[0].jointCount = 2;
    CHECK(maniBuildSpringSet(s_builder, &s).result == mani_success,
          "a joint three bones below the one before");
    s = Source();
    s.referenceRate = 2000.0f;
    CHECK(Refused(&s, maniBuildReasonOption, 0, 0), "a reference rate past the limit");
    s = Source();
    s.referenceRate = -1.0f;
    CHECK(Refused(&s, maniBuildReasonOption, 0, 0), "a negative reference rate");
    s.referenceRate = 0.0f;
    s.stepRate = 2000.0f;
    CHECK(Refused(&s, maniBuildReasonOption, 1, 0), "a step rate past the limit");
}

static void TestJointRefusals(void)
{
    maniSpringJointSource* j = &s_hairJoints[1];
    float* fields[4] = {&j->hitRadius, &j->stiffness, &j->gravityPower, &j->drag};
    const float values[4] = {-0.1f, -1.0f, -0.5f, 1.5f};
    for (int k = 0; k < 4; ++k)
    {
        maniSpringSetSource s = Source();
        *fields[k] = values[k];
        CHECK(Refused(&s, maniBuildReasonSpringJoint, 0, 1), "a value out of range");
    }
    maniSpringSetSource s = Source();
    j->gravityDirection.x = INFINITY;
    CHECK(Refused(&s, maniBuildReasonSpringJoint, 0, 1), "a direction not finite");
    s = Source();
    j->limit = 4;
    CHECK(Refused(&s, maniBuildReasonSpringJoint, 0, 1), "an unknown limit");
    s = Source();
    j->limitAngle = -0.1f;
    CHECK(Refused(&s, maniBuildReasonSpringJoint, 0, 1), "a negative angle");
    s = Source();
    s_tailJoints[0].limitYaw = NAN;
    CHECK(Refused(&s, maniBuildReasonSpringJoint, 1, 0), "a yaw not finite");
    s = Source();
    j->limitRotation = (maniQuat){0.0f, 0.0f, 0.0f, 0.5f};
    CHECK(Refused(&s, maniBuildReasonSpringJoint, 0, 1), "a rotation not of length 1");
    s = Source();
    s_hairJoints[2].drag = 2.0f;
    CHECK(Refused(&s, maniBuildReasonSpringJoint, 0, 2), "the last joint's values checked");
    s = Source();
    s.springs = NULL;
    uint32_t misuses = maniBuilderMisuses(s_builder);
    CHECK(maniBuildSpringSet(s_builder, &s).result == mani_errorInvalid &&
              maniBuildSpringSet(s_builder, NULL).result == mani_errorInvalid &&
              maniBuildSpringSet(NULL, &s).result == mani_errorInvalid &&
              maniBuilderMisuses(s_builder) == misuses + 2,
          "springs NULL with a count, no source, no builder");
}

// The file built last with a word set, rehashed and reidentified;
// opened: refused at want.
static bool RefusedAt(uint64_t offset, uint32_t value, uint64_t want)
{
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + offset, value);
    maniContainer c;
    CHECK(maniOpenContainer(s_file, s_size, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION, &c)
                  .result == mani_success,
          "the container");
    static const uint8_t kEmpty = 0;
    const uint8_t* data[MANI_SPRING_SECTIONS];
    uint64_t sizes[MANI_SPRING_SECTIONS];
    for (uint32_t t = 1; t <= MANI_SPRING_SECTIONS; ++t)
    {
        uint64_t at = 0;
        data[t - 1] = &kEmpty;
        sizes[t - 1] = 0;
        if (maniContainerSection(&c, t, &data[t - 1], &sizes[t - 1], &at))
        {
            data[t - 1] = s_crafted + at;
        }
    }
    maniStoreU64(s_crafted + 40, maniSpringComputeIdentity(data, sizes));
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + 64, (uint64_t)s_size - 64));
    maniSpringSet set;
    maniFileResult r = maniOpenSpringSet(s_crafted, s_size, &set);
    if (r.result != mani_errorFormat || r.offset != want)
    {
        printf("  at %llu: refused %d at %llu, want %llu\n", (unsigned long long)offset,
               (int)r.result, (unsigned long long)r.offset, (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want;
}

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

static uint32_t Bits(float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return bits;
}

static void TestOpen(void)
{
    maniSpringSetSource source = Source();
    CHECK(Build(&source).result == mani_success, "built");
    uint64_t h = SectionAt(maniSpringTagHeader);
    uint64_t c = SectionAt(maniSpringTagColliders);
    uint64_t s = SectionAt(maniSpringTagSprings);
    uint64_t j = SectionAt(maniSpringTagJoints);
    uint64_t l = SectionAt(maniSpringTagLinks);
    uint64_t n = SectionAt(maniSpringTagNames);
    const uint64_t kWords[][3] = {
        {h + 8, 0, h + 8},
        {h + 16, 70000, h + 16},
        {n + 8, 0xFF6Cu, n + 9},
        {h + 32, 0, h + 32},
        {h + 36, Bits(2000.0f), h + 36},
        {h + 40, 1, h + 40},
        {h + 12, 3, h + 12},
        {h + 28, 13, h + 28},
        {c, 9, c},
        {c + 4, 3, c + 4},
        {c + 2 * 48 + 8, 1, c + 2 * 48 + 8},
        {c + 8, 2, c + 8},
        {c + 12, 1, c + 12},
        {c + 20, Bits(NAN), c + 20},
        {c + 2 * 48 + 32, Bits(0.5f), c + 2 * 48 + 28},
        {c + 40, Bits(-1.0f), c + 40},
        {c + 44, 1, c + 44},
        {s, 1, s},
        {s + 4, 0, s + 4},
        {s + 4, 5, s + 4},
        {s + 8, 1, s + 8},
        {s + 12, 9, s + 12},
        {s + 16, 9, s + 16},
        {s + 20, 1, s + 20},
        {s + 24, 10, s + 24},
        {s + 28, 1, s + 28},
        {s + 32 + 4, 1, h + 20},
        {j, 9, j},
        {j + 64, 0, j + 64},
        {j + 4, s_hair[0], j + 4},
        {j + 8, 4, j + 8},
        {j + 12, Bits(-1.0f), j + 12},
        {j + 12, Bits(INFINITY), j + 12},
        {j + 24, Bits(INFINITY), j + 24},
        {j + 16, Bits(-1.0f), j + 16},
        {j + 20, Bits(1.5f), j + 20},
        {j + 24, Bits(-1.0f), j + 24},
        {j + 40, Bits(4.0f), j + 40},
        {j + 3 * 64 + 40, Bits(0.1f), j + 3 * 64 + 40},
        {j + 44, Bits(0.1f), j + 44},
        {j + 2 * 64 + 44, Bits(2.0f), j + 2 * 64 + 44},
        {j + 48, 0, j + 48},
        {j + 3 * 64 + 48, Bits(1.0f), j + 3 * 64 + 48},
        {l + 4, 4, l + 4},
        {n, 0xFFu, n},
    };
    for (size_t k = 0; k < sizeof kWords / sizeof kWords[0]; ++k)
    {
        CHECK(RefusedAt(kWords[k][0], (uint32_t)kWords[k][1], kWords[k][2]),
              "a word refused at its offset");
    }
    maniSpringSet set;
    CHECK(maniOpenSpringSet(s_file + 1, s_size - 1, &set).result == mani_errorInvalid &&
              maniOpenSpringSet(s_file, s_size, NULL).result == mani_errorInvalid &&
              maniOpenSpringSet(NULL, s_size, &set).result == mani_errorInvalid,
          "misaligned, no view, no bytes");
    memcpy(s_crafted, s_file, s_size);
    maniStoreU64(s_crafted + 40, 1);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + 64, (uint64_t)s_size - 64));
    CHECK(maniOpenSpringSet(s_crafted, s_size, &set).offset == 40, "another identity");
}

// A file whose sections agree with a header of more springs than
// MANI_MAX_BONES (one joint each): refused at the count.
static void TestTooManySprings(void)
{
    uint32_t springs = MANI_MAX_BONES + 1;
    uint8_t header[MANI_SPRING_HEADER_SIZE] = {0};
    maniStoreU64(header, maniSkeletonIdentity(&s_skeleton));
    const uint32_t counts[6] = {9, 0, springs, springs, 0, 0};
    for (uint32_t k = 0; k < 6; ++k)
    {
        maniStoreU32(header + 8 + 4 * k, counts[k]);
    }
    maniStoreU32(header + 32, 0x42700000u); // 60.0f
    maniStoreU32(header + 36, 0x42700000u);
    size_t springBytes = (size_t)springs * MANI_SPRING_SPRING_SIZE;
    size_t jointBytes = (size_t)springs * MANI_SPRING_JOINT_SIZE;
    uint8_t* records = calloc(springBytes + jointBytes, 1);
    CHECK(records != NULL, "memory");
    if (records == NULL)
    {
        return;
    }
    for (uint32_t i = 0; i < springs; ++i)
    {
        uint8_t* e = records + (size_t)i * MANI_SPRING_SPRING_SIZE;
        maniStoreU32(e, i);
        maniStoreU32(e + 4, 1);
        maniStoreU32(e + 16, MANI_NO_BONE);
        uint8_t* j = records + springBytes + (size_t)i * MANI_SPRING_JOINT_SIZE;
        maniStoreU32(j + maniSpringJointTail, 1);
        maniStoreU32(j + maniSpringJointFrame + 12, 0x3F800000u); // 1.0f
    }
    const maniSectionSource sections[3] = {
        {maniSpringTagHeader, header, sizeof header},
        {maniSpringTagSprings, records, springBytes},
        {maniSpringTagJoints, records + springBytes, jointBytes}};
    const uint8_t* data[MANI_SPRING_SECTIONS] = {header,  records, records, records + springBytes,
                                                 records, records};
    const uint64_t sizes[MANI_SPRING_SECTIONS] = {sizeof header, 0, springBytes, jointBytes, 0, 0};
    uint64_t fileSize = 0;
    CHECK(maniContainerSize(sections, 3, &fileSize), "the size");
    uint8_t* block = malloc((size_t)fileSize + 16);
    uint8_t* file = block != NULL ? block + (16 - (uintptr_t)block % 16) % 16 : NULL;
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindSpringSet, MANI_SPRING_FORMAT_VERSION,
                           maniSpringComputeIdentity(data, sizes), sections, 3);
        maniSpringSet set;
        maniFileResult r = maniOpenSpringSet(file, (size_t)fileSize, &set);
        uint64_t at = 0;
        maniContainer c;
        const uint8_t* h = NULL;
        uint64_t size = 0;
        CHECK(maniOpenContainer(file, (size_t)fileSize, maniKindSpringSet,
                                MANI_SPRING_FORMAT_VERSION, &c)
                          .result == mani_success &&
                  maniContainerSection(&c, maniSpringTagHeader, &h, &size, &at) &&
                  r.result == mani_errorFormat && r.offset == at + 16,
              "more springs than a skeleton's bones");
    }
    free(block);
    free(records);
}

static int WriteSeed(const char* path)
{
    maniSpringSetSource source = Source();
    FILE* file = Build(&source).result == mani_success ? fopen(path, "wb") : NULL;
    bool ok = file != NULL && fwrite(s_file, 1, s_size, file) == s_size;
    ok = file != NULL && fclose(file) == 0 && ok;
    return ok ? 0 : 1;
}

int main(int argc, char** argv)
{
    OpenSkeleton();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        int r = WriteSeed(argv[2]);
        maniDestroyBuilder(s_builder);
        return r;
    }
    TestBuild();
    TestConversion();
    TestLimits();
    TestColliderRefusals();
    TestSpringRefusals();
    TestJointRefusals();
    TestOpen();
    TestTooManySprings();
    maniDestroyBuilder(s_builder);
    return s_failures == 0 ? 0 : 1;
}
