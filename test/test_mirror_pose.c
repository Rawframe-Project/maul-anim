// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mirrored poses and root moves (mani-0033) on a rig whose left and right
// clavicles, arms and hands have unmirrored axes, with a prop under the
// right hand: for each axis, a turned pose's mirror puts every bone but
// the prop at its partner's model position reflected, turned by its
// partner's model rotation mirrored and carried to its own reference
// axes; the rest's model rotations stay; mirroring twice gives the pose
// back; in place equals apart; the partner's scale; moves reflected; the
// refusals.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    kRoot,
    kHips,
    kSpine,
    kChest,
    kHead,
    kClavL,
    kArmL,
    kHandL,
    kClavR,
    kArmR,
    kHandR,
    kPropR,
    kLegL,
    kLegR,
    kBones
};

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

static const maniBoneSource kRig[kBones] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"hips", 4, kRoot, {{0, 1, 0}, REST, {1, 1, 1}}},
    {"spine", 5, kHips, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
    {"chest", 5, kSpine, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
    {"head", 4, kChest, {{0, 0.3f, 0}, REST, {1, 1, 1}}},
    {"clavL", 5, kChest, {{0.1f, 0.1f, 0}, {0, 0, 0.25881904f, 0.96592583f}, {1, 1, 1}}},
    {"armL", 4, kClavL, {{0.2f, 0, 0}, REST, {1, 1, 1}}},
    {"handL", 5, kArmL, {{0.3f, 0, 0}, {0.5f, 0.5f, 0.5f, 0.5f}, {1, 1, 1}}},
    {"clavR", 5, kChest, {{-0.1f, 0.1f, 0}, {0, 0.70710678f, 0, 0.70710678f}, {1, 1, 1}}},
    {"armR", 4, kClavR, {{0, 0, 0.2f}, REST, {1, 1, 1}}},
    {"handR", 5, kArmR, {{0, 0, 0.3f}, {0.70710678f, 0, 0, 0.70710678f}, {1, 1, 1}}},
    {"propR", 5, kHandR, {{0, 0, 0.1f}, REST, {1, 1, 1}}},
    {"legL", 4, kHips, {{0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
    {"legR", 4, kHips, {{-0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
};

static const maniMirrorRuleSource kRules[2] = {{"L", 1, maniMirrorSuffix, "R", 1, 0},
                                               {"R", 1, maniMirrorSuffix, "L", 1, 0}};

static _Alignas(16) uint8_t s_skeletonFile[2][4096];
static _Alignas(16) uint8_t s_file[3][4096];
static _Alignas(32) uint8_t s_poseMemory[5][2048];
static maniSkeleton s_skeleton;
static maniSkeleton s_other;
static maniMirrorMap s_maps[3];
static maniPose* s_in;
static maniPose* s_out;
static maniPose* s_back;
static maniPose* s_place;
static maniPose* s_otherPose;

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

static void OpenSkeleton(const maniBoneSource* bones, uint32_t count, uint8_t* file,
                         maniSkeleton* out)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, count, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, 4096, &size) == mani_success &&
              maniOpenSkeleton(file, size, out).result == mani_success,
          "a rig");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    OpenSkeleton(kRig, kBones, s_skeletonFile[0], &s_skeleton);
    OpenSkeleton(kRig, kBones - 1, s_skeletonFile[1], &s_other);
    for (uint32_t axis = 0; axis < 3; ++axis)
    {
        maniMirrorMapSource source = {
            .skeleton = &s_skeleton, .rules = kRules, .ruleCount = 2, .axis = axis};
        maniBuilder* builder = NewBuilder();
        size_t size = 0;
        CHECK(maniBuildMirrorMap(builder, &source).result == mani_success &&
                  maniCopyBuilt(builder, s_file[axis], sizeof s_file[axis], &size) ==
                      mani_success &&
                  maniOpenMirrorMap(s_file[axis], size, &s_maps[axis]).result == mani_success,
              "a map");
        maniDestroyBuilder(builder);
    }
    maniPose** poses[4] = {&s_in, &s_out, &s_back, &s_place};
    for (uint32_t i = 0; i < 4; ++i)
    {
        CHECK(maniInitPose(&s_skeleton, s_poseMemory[i], sizeof s_poseMemory[i], poses[i]) ==
                  mani_success,
              "a pose");
    }
    CHECK(maniInitPose(&s_other, s_poseMemory[4], sizeof s_poseMemory[4], &s_otherPose) ==
              mani_success,
          "the other rig's pose");
}

static maniQuat Mul(maniQuat a, maniQuat b)
{
    return (maniQuat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static maniQuat Conjugate(maniQuat q)
{
    return (maniQuat){-q.x, -q.y, -q.z, q.w};
}

static maniQuat MirrorQuat(maniQuat q, uint32_t axis)
{
    return (maniQuat){axis == 0 ? q.x : -q.x, axis == 1 ? q.y : -q.y, axis == 2 ? q.z : -q.z, q.w};
}

static maniVec3 Reflect(maniVec3 v, uint32_t axis)
{
    return (maniVec3){axis == 0 ? -v.x : v.x, axis == 1 ? -v.y : v.y, axis == 2 ? -v.z : v.z};
}

static bool SameTurn(maniQuat a, maniQuat b)
{
    return fabsf(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) > 1.0f - 1e-5f;
}

static bool Near(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

// Each bone's model rotation: its root path's local rotations.
static void ModelRotations(const maniPose* pose, maniQuat out[kBones])
{
    for (uint32_t b = 0; b < kBones; ++b)
    {
        uint32_t parent = maniSkeletonParent(&s_skeleton, b);
        maniQuat local = maniPoseLocal(pose, b).rotation;
        out[b] = parent == MANI_NO_BONE ? local : Mul(out[parent], local);
    }
}

static maniQuat Unit(float x, float y, float z, float w)
{
    float n = sqrtf(x * x + y * y + z * z + w * w);
    return (maniQuat){x / n, y / n, z / n, w / n};
}

// A turned pose: every bone's rotation and translation moved.
static void Turned(maniPose* pose)
{
    for (uint32_t b = 0; b < kBones; ++b)
    {
        maniTransform t = maniSkeletonRest(&s_skeleton, b);
        float k = (float)(b + 1);
        t.rotation = Mul(t.rotation, Unit(0.1f * k, -0.07f * k, 0.05f + 0.03f * k, 1.0f));
        t.translation.x += 0.01f * k;
        t.translation.y -= 0.02f;
        t.translation.z += 0.03f - 0.005f * k;
        maniPoseSetLocal(pose, b, t);
    }
}

static maniVec3 Position(maniPose* pose, uint32_t bone)
{
    maniAffine m = maniPoseModel(pose, bone);
    return (maniVec3){m.m[9], m.m[10], m.m[11]};
}

static uint32_t Partner(uint32_t axis, uint32_t bone)
{
    return maniMirrorPartner(&s_maps[axis], bone);
}

// Every bone but the prop at its partner's model position reflected and
// model rotation mirrored, carried to its reference axes:
// `mirror(M_m) mirror(R_m)^-1 R_b`, R the rest's.
static bool Mirrored(uint32_t axis)
{
    maniQuat rest[kBones];
    maniQuat in[kBones];
    maniQuat out[kBones];
    CHECK(maniPoseSetRest(s_back, &s_skeleton) == mani_success, "the rest");
    ModelRotations(s_back, rest);
    ModelRotations(s_in, in);
    ModelRotations(s_out, out);
    CHECK(maniComputeModel(s_in, &s_skeleton) == mani_success &&
              maniComputeModel(s_out, &s_skeleton) == mani_success,
          "the models");
    bool ok = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        if (b == kPropR)
        {
            continue;
        }
        uint32_t m = Partner(axis, b);
        maniQuat want =
            Mul(Mul(MirrorQuat(in[m], axis), Conjugate(MirrorQuat(rest[m], axis))), rest[b]);
        maniVec3 at = Position(s_out, b);
        maniVec3 from = Reflect(Position(s_in, m), axis);
        ok = ok && SameTurn(out[b], want) && Near(at.x, from.x) && Near(at.y, from.y) &&
             Near(at.z, from.z);
    }
    return ok;
}

static bool SameLocals(maniPose* a, maniPose* b, bool exact)
{
    bool ok = true;
    for (uint32_t bone = 0; bone < kBones; ++bone)
    {
        maniTransform x = maniPoseLocal(a, bone);
        maniTransform y = maniPoseLocal(b, bone);
        if (exact)
        {
            ok = ok && memcmp(&x, &y, sizeof x) == 0;
            continue;
        }
        ok = ok && Near(x.translation.x, y.translation.x) &&
             Near(x.translation.y, y.translation.y) && Near(x.translation.z, y.translation.z) &&
             SameTurn(x.rotation, y.rotation) && Near(x.scale.x, y.scale.x) &&
             Near(x.scale.y, y.scale.y) && Near(x.scale.z, y.scale.z);
    }
    return ok;
}

static void TestPose(void)
{
    CHECK(Partner(0, kArmL) == kArmR && Partner(0, kPropR) == kPropR &&
              maniMirrorMapUnmatchedBone(&s_maps[0]) == kPropR,
          "the partners");
    for (uint32_t axis = 0; axis < 3; ++axis)
    {
        Turned(s_in);
        CHECK(maniMirrorPose(&s_maps[axis], &s_skeleton, s_in, s_out) == mani_success &&
                  !maniPoseModelCurrent(s_out),
              "mirrored");
        CHECK(Mirrored(axis), "each bone at its partner's place reflected, turned mirrored");
        CHECK(maniMirrorPose(&s_maps[axis], &s_skeleton, s_out, s_back) == mani_success &&
                  SameLocals(s_back, s_in, false),
              "mirroring twice gives the pose back");
        Turned(s_place);
        CHECK(maniMirrorPose(&s_maps[axis], &s_skeleton, s_place, s_place) == mani_success &&
                  SameLocals(s_place, s_out, true),
              "in place as apart");
    }
    maniQuat rest[kBones];
    maniQuat out[kBones];
    CHECK(maniPoseSetRest(s_in, &s_skeleton) == mani_success &&
              maniMirrorPose(&s_maps[0], &s_skeleton, s_in, s_out) == mani_success,
          "the rest mirrored");
    ModelRotations(s_in, rest);
    ModelRotations(s_out, out);
    bool kept = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        kept = kept && (b == kPropR || SameTurn(rest[b], out[b]));
    }
    CHECK(kept, "the rest's model rotations stay");
    maniTransform hand = maniSkeletonRest(&s_skeleton, kHandL);
    hand.scale = (maniVec3){2.0f, 3.0f, 4.0f};
    maniPoseSetLocal(s_in, kHandL, hand);
    maniVec3 scale = (maniMirrorPose(&s_maps[0], &s_skeleton, s_in, s_out) == mani_success)
                         ? maniPoseLocal(s_out, kHandR).scale
                         : (maniVec3){0};
    CHECK(scale.x == 2.0f && scale.y == 3.0f && scale.z == 4.0f, "the partner's scale");
}

static void TestMoves(void)
{
    maniRootMove move = {{1.0f, 2.0f, 3.0f}, Unit(0.1f, 0.2f, 0.3f, 0.9f)};
    for (uint32_t axis = 0; axis < 3; ++axis)
    {
        maniRootMove m = maniMirrorRootMotion(&s_maps[axis], move);
        maniVec3 t = Reflect(move.translation, axis);
        maniQuat q = MirrorQuat(move.rotation, axis);
        CHECK(memcmp(&m.translation, &t, sizeof t) == 0 && memcmp(&m.rotation, &q, sizeof q) == 0,
              "a move reflected");
    }
    maniRootMove same = maniMirrorRootMotion(NULL, move);
    CHECK(memcmp(&same, &move, sizeof move) == 0, "no map leaves a move");
}

static void TestRefusals(void)
{
    const maniMirrorMap* map = &s_maps[0];
    CHECK(maniMirrorPose(NULL, &s_skeleton, s_in, s_out) == mani_errorInvalid &&
              maniMirrorPose(map, NULL, s_in, s_out) == mani_errorInvalid &&
              maniMirrorPose(map, &s_skeleton, NULL, s_out) == mani_errorInvalid &&
              maniMirrorPose(map, &s_skeleton, s_in, NULL) == mani_errorInvalid,
          "a NULL argument");
    CHECK(maniMirrorPose(map, &s_other, s_otherPose, s_otherPose) == mani_errorInvalid,
          "another skeleton");
    CHECK(maniMirrorPose(map, &s_skeleton, s_otherPose, s_out) == mani_errorInvalid &&
              maniMirrorPose(map, &s_skeleton, s_in, s_otherPose) == mani_errorInvalid,
          "a pose of another skeleton");
    maniMirrorMap counted = *map;
    counted.boneCount = kBones + 1;
    CHECK(maniMirrorPose(&counted, &s_skeleton, s_in, s_out) == mani_errorInvalid,
          "a map of another bone count");
    Turned(s_in);
    CHECK(maniPoseSetRest(s_back, &s_skeleton) == mani_success &&
              maniPoseDifference(&s_skeleton, s_in, s_back, s_place) == mani_success &&
              maniMirrorPose(map, &s_skeleton, s_place, s_out) == mani_errorInvalid,
          "a difference");
    CHECK(maniPoseDifference(&s_skeleton, s_in, s_back, s_out) == mani_success &&
              maniMirrorPose(map, &s_skeleton, s_in, s_out) == mani_success &&
              !maniPoseIsDifference(s_out),
          "the mirror a pose, not a difference");
}

int main(void)
{
    Setup();
    TestPose();
    TestMoves();
    TestRefusals();
    return s_failures == 0 ? 0 : 1;
}
