// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pose transfer through a retargeting map from a source
// of three spine bones and a two-bone arm to a target of four spine
// bones and a three-bone arm whose rests turn sp1 and the upper arm: the
// source's reference giving the target's; each mapped bone's model turn
// from its reference the source bone's (slerped between two for an
// interpolated chain's middle bones, halved by a weight of 0.5), checked
// in binary64; unmapped bones at their reference locals bit for bit;
// translations kept, scaled through the parents' reference frames, the
// scale carried; the pelvis's change scaled by parts with its absolute
// and source blends; root motion scaled; the refusals; a transfer's bits
// pinned, the same on every platform. The source's rests turn s1 and
// the arm and scale the hand; the target's root rests off the origin.

#include "pose_memory.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PINNED_HASH 0x0c1461ad38a9a3fdull

static _Alignas(16) uint8_t s_skeletonFiles[6][4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[4][4096];
static _Alignas(16) uint8_t s_scratch[8192];
static maniSkeleton s_source;
static maniSkeleton s_target;
static maniSkeleton s_otherSource;
static maniSkeleton s_otherTarget;
static maniRetargetMap s_map;
static maniPose* s_in;
static maniPose* s_out;

typedef struct Bone
{
    const char* name;
    int32_t parent;
    maniVec3 offset;
    maniQuat rotation;
} Bone;

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

// s1 rests turned 20 degrees about x, the arm 30 about z; the hand's
// rest scale is (0.5, 1, 1) (Skeleton below).
static const Bone kSource[8] = {
    {"root", -1, {0.0f, 0.0f, 0.0f}, REST},
    {"hips", 0, {0.0f, 1.0f, 0.0f}, REST},
    {"s1", 1, {0.0f, 0.2f, 0.0f}, {0.17364818f, 0.0f, 0.0f, 0.98480775f}},
    {"s2", 2, {0.0f, 0.2f, 0.0f}, REST},
    {"chest", 3, {0.0f, 0.2f, 0.0f}, REST},
    {"armL", 4, {0.2f, 0.1f, 0.0f}, {0.0f, 0.0f, 0.25881904f, 0.96592583f}},
    {"handL", 5, {0.3f, 0.0f, 0.0f}, REST},
    {"head", 4, {0.0f, 0.3f, 0.0f}, REST}};

// The root rests off the origin; sp1 turned 30 degrees about z, the upper
// arm 90 about y; the offsets below them are along their own turned
// frames.
static const Bone kTarget[10] = {
    {"root", -1, {0.1f, 0.0f, 0.0f}, REST},
    {"pelvis", 0, {0.0f, 0.9f, 0.0f}, REST},
    {"sp1", 1, {0.0f, 0.15f, 0.0f}, {0.0f, 0.0f, 0.25881904f, 0.96592583f}},
    {"sp2", 2, {0.0f, 0.15f, 0.0f}, REST},
    {"sp3", 3, {0.0f, 0.15f, 0.0f}, REST},
    {"chest", 4, {0.0f, 0.15f, 0.0f}, REST},
    {"upperarm", 5, {0.2f, 0.1f, 0.0f}, {0.0f, 0.70710678f, 0.0f, 0.70710678f}},
    {"forearm", 6, {0.0f, 0.0f, -0.15f}, REST},
    {"hand", 7, {0.0f, 0.0f, -0.15f}, REST},
    {"head", 5, {0.0f, 0.3f, 0.0f}, REST}};

static const maniRetargetChainSource kChains[2] = {
    {2, 4, 2, 5, maniRetargetRotationInterpolated, 1.0f, maniRetargetTranslationOwn, false},
    {5, 6, 6, 8, maniRetargetRotationOneToOne, 0.5f, maniRetargetTranslationScaled, true}};
static const maniRetargetPelvisSource kPelvis = {
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 1.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 0.75f, 1.0f};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

static void Skeleton(const Bone* bones, uint32_t count, int side, maniSkeleton* out)
{
    maniBoneSource sources[16];
    for (uint32_t b = 0; b < count; ++b)
    {
        sources[b] = (maniBoneSource){bones[b].name,
                                      (uint32_t)strlen(bones[b].name),
                                      bones[b].parent,
                                      {bones[b].offset, bones[b].rotation, {1, 1, 1}}};
        // The source's hand rests scaled, and the invariant rigs' (sides 4
        // and 5).
        sources[b].rest.scale.x = (side == 0 || side >= 4) && b == 6 ? 0.5f : 1.0f;
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {sources, count};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFiles[side], sizeof s_skeletonFiles[side],
                                    &size) == mani_success &&
              maniOpenSkeleton(s_skeletonFiles[side], size, out).result == mani_success,
          "a skeleton");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    Skeleton(kSource, 8, 0, &s_source);
    Skeleton(kTarget, 10, 1, &s_target);
    // Rigs of the same counts and other identities (their last bone
    // moved).
    Bone sourceBones[8];
    Bone targetBones[10];
    memcpy(sourceBones, kSource, sizeof sourceBones);
    memcpy(targetBones, kTarget, sizeof targetBones);
    sourceBones[7].offset.y = 0.4f;
    targetBones[9].offset.y = 0.4f;
    Skeleton(sourceBones, 8, 2, &s_otherSource);
    Skeleton(targetBones, 10, 3, &s_otherTarget);
    maniRetargetMapSource source = {.source = &s_source,
                                    .target = &s_target,
                                    .chains = kChains,
                                    .pelvis = &kPelvis,
                                    .chainCount = 2,
                                    .matchNames = true};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenRetargetMap(s_file, size, &s_map).result == mani_success,
          "the map");
    maniDestroyBuilder(builder);
    CHECK(maniInitPose(&s_source, s_poseMemory[0], sizeof s_poseMemory[0], &s_in) == mani_success &&
              maniInitPose(&s_target, s_poseMemory[1], sizeof s_poseMemory[1], &s_out) ==
                  mani_success,
          "the poses");
}

static bool Transfer(void)
{
    return maniRetargetMap_RetargetPose(&s_map, &s_source, s_in, &s_target, s_out, s_scratch,
                                        sizeof s_scratch) == mani_success;
}

// Quaternions in binary64.
typedef struct Q
{
    double x;
    double y;
    double z;
    double w;
} Q;

static Q FromQuat(maniQuat q)
{
    return (Q){(double)q.x, (double)q.y, (double)q.z, (double)q.w};
}

static Q Mul(Q a, Q b)
{
    return (Q){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
               a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
               a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
               a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static Q Conj(Q q)
{
    return (Q){-q.x, -q.y, -q.z, q.w};
}

static Q Slerp(Q a, Q b, double t)
{
    double d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0)
    {
        b = (Q){-b.x, -b.y, -b.z, -b.w};
        d = -d;
    }
    double theta = acos(d > 1.0 ? 1.0 : d);
    if (theta < 1e-9)
    {
        return a;
    }
    double wa = sin((1.0 - t) * theta) / sin(theta);
    double wb = sin(t * theta) / sin(theta);
    return (Q){wa * a.x + wb * b.x, wa * a.y + wb * b.y, wa * a.z + wb * b.z, wa * a.w + wb * b.w};
}

// The same rotation, within tol of their normalized dot product's 1
// (either sign).
static bool Near(Q a, Q b, double tol)
{
    double na = sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w);
    double nb = sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w);
    double d = fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w) / (na * nb);
    return d >= 1.0 - tol;
}

static void Rotate(Q q, const double* v, double* out)
{
    Q p = Mul(Mul(q, (Q){v[0], v[1], v[2], 0.0}), Conj(q));
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
}

// A bone's model position from its root path's rotations and
// translations (their scales 1), from the pose or, for NULL, the rest.
static void Point(const maniSkeleton* skeleton, const maniPose* pose, uint32_t bone, double* out)
{
    double p[3] = {0.0, 0.0, 0.0};
    for (uint32_t b = bone; b != MANI_NO_BONE; b = maniSkeleton_GetParent(skeleton, b))
    {
        maniTransform t =
            pose != NULL ? maniPose_GetLocal(pose, b) : maniSkeleton_GetRest(skeleton, b);
        double turned[3];
        Rotate(FromQuat(t.rotation), p, turned);
        p[0] = turned[0] + (double)t.translation.x;
        p[1] = turned[1] + (double)t.translation.y;
        p[2] = turned[2] + (double)t.translation.z;
    }
    memcpy(out, p, sizeof p);
}

// A bone's model rotation: its root path's local rotations' product.
static Q Model(const maniPose* pose, const maniSkeleton* skeleton, uint32_t bone)
{
    Q q = FromQuat(maniPose_GetLocal(pose, bone).rotation);
    for (uint32_t p = maniSkeleton_GetParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeleton_GetParent(skeleton, p))
    {
        q = Mul(FromQuat(maniPose_GetLocal(pose, p).rotation), q);
    }
    return q;
}

static Q RestModel(const maniSkeleton* skeleton, uint32_t bone)
{
    Q q = FromQuat(maniSkeleton_GetRest(skeleton, bone).rotation);
    for (uint32_t p = maniSkeleton_GetParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeleton_GetParent(skeleton, p))
    {
        q = Mul(FromQuat(maniSkeleton_GetRest(skeleton, p).rotation), q);
    }
    return q;
}

// A bone's model turn from its rest.
static Q Turn(const maniPose* pose, const maniSkeleton* skeleton, uint32_t bone)
{
    return Mul(Model(pose, skeleton, bone), Conj(RestModel(skeleton, bone)));
}

static maniQuat AxisAngle(double x, double y, double z, double angle)
{
    double s = sin(0.5 * angle);
    return (maniQuat){(float)(x * s), (float)(y * s), (float)(z * s), (float)cos(0.5 * angle)};
}

static void Turned(uint32_t bone, maniQuat turn)
{
    maniTransform t = maniPose_GetLocal(s_in, bone);
    Q q = Mul(FromQuat(turn), FromQuat(t.rotation));
    t.rotation = (maniQuat){(float)q.x, (float)q.y, (float)q.z, (float)q.w};
    maniPose_SetLocal(s_in, bone, t);
}

// The source's reference gives the target's, but for the pelvis's
// translation: its source blend pulls it toward the source pelvis's own
// place, which the target's root offset puts elsewhere (TestTranslations
// checks it by its formula).
static void TestReference(void)
{
    CHECK(maniPose_SetRest(s_in, &s_source) == mani_success && Transfer(), "transferred");
    bool same = true;
    for (uint32_t b = 0; b < 10; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_out, b);
        maniTransform r = maniSkeleton_GetRest(&s_target, b);
        bool placed = b == 1 || (fabsf(t.translation.x - r.translation.x) <= 1e-6f &&
                                 fabsf(t.translation.y - r.translation.y) <= 1e-6f &&
                                 fabsf(t.translation.z - r.translation.z) <= 1e-6f);
        same = same && Near(FromQuat(t.rotation), FromQuat(r.rotation), 1e-12) && placed;
    }
    CHECK(same, "the source's reference gives the target's");
}

// A walk-like source pose: the hips, s1, s2, the chest, the arm and the
// hand turned about different axes.
static void Posed(void)
{
    CHECK(maniPose_SetRest(s_in, &s_source) == mani_success, "rest");
    // The root moved and turned: the hips' model place moves with it.
    maniTransform root = maniPose_GetLocal(s_in, 0);
    root.translation = (maniVec3){0.05f, 0.0f, 0.02f};
    maniPose_SetLocal(s_in, 0, root);
    Turned(0, AxisAngle(0.0, 1.0, 0.0, 0.3));
    Turned(1, AxisAngle(0.0, 1.0, 0.0, 0.4));
    Turned(2, AxisAngle(1.0, 0.0, 0.0, 0.3));
    Turned(3, AxisAngle(0.0, 0.0, 1.0, -0.5));
    Turned(4, AxisAngle(0.6, 0.8, 0.0, 0.2));
    Turned(5, AxisAngle(0.0, 0.0, 1.0, 0.9));
    Turned(6, AxisAngle(0.0, 1.0, 0.0, -0.7));
}

static void TestTurns(void)
{
    Posed();
    CHECK(Transfer(), "transferred");
    // The target's pelvis, sp1, chest, upper arm and forearm from one
    // source bone each; sp2 and sp3 between two at 2/3 and 1/3.
    Q pelvis = Turn(s_in, &s_source, 1);
    Q s1 = Turn(s_in, &s_source, 2);
    Q s2 = Turn(s_in, &s_source, 3);
    Q chest = Turn(s_in, &s_source, 4);
    Q arm = Turn(s_in, &s_source, 5);
    Q hand = Turn(s_in, &s_source, 6);
    Q none = {0.0, 0.0, 0.0, 1.0};
    CHECK(Near(Turn(s_out, &s_target, 1), pelvis, 1e-11) &&
              Near(Turn(s_out, &s_target, 2), s1, 1e-11) &&
              Near(Turn(s_out, &s_target, 5), chest, 1e-11) &&
              Near(Turn(s_out, &s_target, 9), chest, 1e-11),
          "one-source bones: the source's model turns, across turned rests");
    CHECK(Near(Turn(s_out, &s_target, 3), Slerp(s1, s2, 2.0 / 3.0), 1e-11) &&
              Near(Turn(s_out, &s_target, 4), Slerp(s2, chest, 1.0 / 3.0), 1e-11),
          "an interpolated chain's middle bones: the turns slerped at their places");
    CHECK(Near(Turn(s_out, &s_target, 6), Slerp(none, arm, 0.5), 1e-11) &&
              Near(Turn(s_out, &s_target, 7), Slerp(none, hand, 0.5), 1e-11),
          "a chain of weight 0.5: half the turns");
    maniTransform extra = maniPose_GetLocal(s_out, 8);
    maniTransform rest = maniSkeleton_GetRest(&s_target, 8);
    CHECK(memcmp(&extra, &rest, sizeof extra) == 0,
          "an extra bone: its reference local bit for bit");
    maniVec3 sp2 = maniPose_GetLocal(s_out, 3).translation;
    maniVec3 own = maniSkeleton_GetRest(&s_target, 3).translation;
    CHECK(memcmp(&sp2, &own, sizeof sp2) == 0, "their own translations kept");
}

static void TestTranslations(void)
{
    Posed();
    // The hand moved 0.1 along its parent's x and scaled 2 along x.
    maniTransform t = maniPose_GetLocal(s_in, 6);
    t.translation.x += 0.1f;
    t.scale.x = 2.0f;
    maniPose_SetLocal(s_in, 6, t);
    // The hips up 0.1 and along x 0.2 (their parent's frame is the
    // model's).
    maniTransform hips = maniPose_GetLocal(s_in, 1);
    hips.translation.x += 0.2f;
    hips.translation.y += 0.1f;
    maniPose_SetLocal(s_in, 1, hips);
    CHECK(Transfer(), "transferred");
    // The forearm (from the hand, scaled): the source's change times 0.9,
    // turned from the source arm's reference model frame into the upper
    // arm's (sp1's 30 degrees about z, then 90 about y), then the scale
    // carried.
    maniTransform forearm = maniPose_GetLocal(s_out, 7);
    maniVec3 rest = maniSkeleton_GetRest(&s_target, 7).translation;
    const double change[3] = {0.09, 0.0, 0.0};
    double turned[3];
    Rotate(Mul(Conj(RestModel(&s_target, 6)), RestModel(&s_source, 5)), change, turned);
    CHECK(fabs((double)forearm.translation.x - ((double)rest.x + turned[0])) <= 1e-6 &&
              fabs((double)forearm.translation.y - ((double)rest.y + turned[1])) <= 1e-6 &&
              fabs((double)forearm.translation.z - ((double)rest.z + turned[2])) <= 1e-6 &&
              fabsf(forearm.scale.x - 4.0f) <= 1e-6f && forearm.scale.y == 1.0f,
          "a scaled translation through the parents' frames, the scale's ratio carried");
    // The pelvis in binary64: the source's change from its reference,
    // horizontal by 0.9 and vertical unscaled (its absolute blend 1),
    // x a quarter of the way to the source's own, weighted 0.75 from the
    // target's reference, then into the target root's frame as posed.
    double now[3];
    double was[3];
    double reference[3];
    Point(&s_source, s_in, 1, now);
    Point(&s_source, NULL, 1, was);
    Point(&s_target, NULL, 1, reference);
    double placed[3] = {reference[0] + 0.9 * (now[0] - was[0]), reference[1] + (now[1] - was[1]),
                        reference[2] + 0.9 * (now[2] - was[2])};
    placed[0] += 0.25 * (now[0] - placed[0]);
    double model[3];
    for (int k = 0; k < 3; ++k)
    {
        model[k] = reference[k] + 0.75 * (placed[k] - reference[k]);
    }
    maniTransform root = maniPose_GetLocal(s_out, 0);
    double offset[3] = {model[0] - (double)root.translation.x,
                        model[1] - (double)root.translation.y,
                        model[2] - (double)root.translation.z};
    double local[3];
    Rotate(Conj(FromQuat(root.rotation)), offset, local);
    maniTransform pelvis = maniPose_GetLocal(s_out, 1);
    CHECK(fabs((double)pelvis.translation.x - local[0]) <= 1e-6 &&
              fabs((double)pelvis.translation.y - local[1]) <= 1e-6 &&
              fabs((double)pelvis.translation.z - local[2]) <= 1e-6 && fabs(now[0] - was[0]) > 0.1,
          "the pelvis: scaled by parts, blended, weighted, under the turned root");
    maniVec3 motion = maniRetargetMap_RetargetRootMotion(&s_map, (maniVec3){1.0f, 0.5f, -2.0f});
    CHECK(fabsf(motion.x - 0.9f) <= 1e-6f && fabsf(motion.y - 0.225f) <= 1e-6f &&
              fabsf(motion.z + 1.8f) <= 1e-6f,
          "root motion: horizontal by 0.9, vertical by 0.45");
    maniVec3 same = maniRetargetMap_RetargetRootMotion(NULL, (maniVec3){1.0f, 2.0f, 3.0f});
    CHECK(same.x == 1.0f && same.y == 2.0f && same.z == 3.0f, "no map: unchanged");
}

static void TestRefusals(void)
{
    CHECK(maniRetargetMap_GetScratchSize(&s_map) > 0 && maniRetargetMap_GetScratchSize(NULL) == 0,
          "the scratch");
    maniResult wrong = maniRetargetMap_RetargetPose(&s_map, &s_target, s_in, &s_source, s_out,
                                                    s_scratch, sizeof s_scratch);
    maniResult small =
        maniRetargetMap_RetargetPose(&s_map, &s_source, s_in, &s_target, s_out, s_scratch,
                                     maniRetargetMap_GetScratchSize(&s_map) - 1);
    maniResult aligned = maniRetargetMap_RetargetPose(&s_map, &s_source, s_in, &s_target, s_out,
                                                      s_scratch + 8, sizeof s_scratch - 8);
    maniResult swapped = maniRetargetMap_RetargetPose(
        &s_map, &s_source, (const maniPose*)s_out, &s_target, s_out, s_scratch, sizeof s_scratch);
    maniResult none = maniRetargetMap_RetargetPose(NULL, &s_source, s_in, &s_target, s_out,
                                                   s_scratch, sizeof s_scratch);
    maniPose* otherIn = NULL;
    maniPose* otherOut = NULL;
    CHECK(maniInitPose(&s_otherSource, s_poseMemory[2], sizeof s_poseMemory[2], &otherIn) ==
                  mani_success &&
              maniInitPose(&s_otherTarget, s_poseMemory[3], sizeof s_poseMemory[3], &otherOut) ==
                  mani_success,
          "poses of the other rigs");
    maniResult otherSource = maniRetargetMap_RetargetPose(
        &s_map, &s_otherSource, otherIn, &s_target, s_out, s_scratch, sizeof s_scratch);
    maniResult otherTarget = maniRetargetMap_RetargetPose(&s_map, &s_source, s_in, &s_target,
                                                          otherOut, s_scratch, sizeof s_scratch);
    maniPoseSetKind(s_in, true);
    maniResult difference = maniRetargetMap_RetargetPose(&s_map, &s_source, s_in, &s_target, s_out,
                                                         s_scratch, sizeof s_scratch);
    maniPoseSetKind(s_in, false);
    CHECK(otherSource == mani_errorInvalid && otherTarget == mani_errorInvalid &&
              difference == mani_errorInvalid,
          "a source of another identity, a target pose of another rig, a source of differences");
    maniPoseSetKind(s_out, true);
    CHECK(Transfer() && !maniPose_IsDifference(s_out), "the output a pose, not differences");
    CHECK(wrong == mani_errorInvalid && small == mani_errorInvalid &&
              aligned == mani_errorInvalid && swapped == mani_errorInvalid &&
              none == mani_errorInvalid,
          "skeletons swapped, scratch short or misaligned, a pose of the other, no map");
}

// A map from the source onto itself, its pelvis scaled: the pose
// retargeted apart, and the same pose as both source and target refused
// and left, since its locals are written while it is still read.
static void TestSamePose(void)
{
    maniRetargetMapSource source = {
        .source = &s_source, .target = &s_source, .pelvis = &kPelvis, .matchNames = true};
    static _Alignas(16) uint8_t file[16384];
    maniRetargetMap map;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenRetargetMap(file, size, &map).result == mani_success,
          "a map from a skeleton onto itself");
    maniDestroyBuilder(builder);
    Posed();
    maniPose* apart = NULL;
    CHECK(maniInitPose(&s_source, s_poseMemory[2], sizeof s_poseMemory[2], &apart) ==
                  mani_success &&
              maniRetargetMap_RetargetPose(&map, &s_source, s_in, &s_source, apart, s_scratch,
                                           sizeof s_scratch) == mani_success,
          "retargeted into another pose");
    uint32_t bones = maniSkeleton_GetBoneCount(&s_source);
    maniTransform before[16];
    bool moved = false;
    for (uint32_t b = 0; b < bones; ++b)
    {
        before[b] = maniPose_GetLocal(s_in, b);
        maniTransform t = maniPose_GetLocal(apart, b);
        moved = moved || memcmp(&t, &before[b], sizeof t) != 0;
    }
    CHECK(bones <= 16 && moved, "the retargeted pose differs from its source");
    CHECK(maniRetargetMap_RetargetPose(&map, &s_source, s_in, &s_source, s_in, s_scratch,
                                       sizeof s_scratch) == mani_errorInvalid,
          "the same pose as source and target refused");
    bool left = true;
    for (uint32_t b = 0; b < bones; ++b)
    {
        maniTransform t = maniPose_GetLocal(s_in, b);
        left = left && memcmp(&t, &before[b], sizeof t) == 0;
    }
    CHECK(left, "the pose left as it was");
}

// The arm chain reversed: the upper arm at its reference between the
// turned chest and the mapped forearm, which still takes the source
// arm's turn.
static void TestReversed(void)
{
    maniRetargetChainSource chains[2] = {kChains[0], kChains[1]};
    chains[1].rotation = maniRetargetRotationOneToOneReversed;
    chains[1].rotationWeight = 1.0f;
    maniRetargetMapSource source = {.source = &s_source,
                                    .target = &s_target,
                                    .chains = chains,
                                    .pelvis = &kPelvis,
                                    .chainCount = 2,
                                    .matchNames = true};
    static _Alignas(16) uint8_t file[16384];
    maniRetargetMap map;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenRetargetMap(file, size, &map).result == mani_success,
          "a reversed map");
    maniDestroyBuilder(builder);
    Posed();
    CHECK(maniRetargetMap_RetargetPose(&map, &s_source, s_in, &s_target, s_out, s_scratch,
                                       sizeof s_scratch) == mani_success,
          "transferred");
    maniTransform upper = maniPose_GetLocal(s_out, 6);
    maniTransform rest = maniSkeleton_GetRest(&s_target, 6);
    CHECK(memcmp(&upper, &rest, sizeof upper) == 0 &&
              Near(Turn(s_out, &s_target, 7), Turn(s_in, &s_source, 5), 1e-11) &&
              Near(Turn(s_out, &s_target, 8), Turn(s_in, &s_source, 6), 1e-11),
          "a reference bone between mapped ones: the mapped ones' turns kept");
}

static void TestPinned(void)
{
    Posed();
    CHECK(Transfer(), "transferred");
    maniTransform locals[10];
    CHECK(maniPose_ReadLocals(s_out, 0, 10, locals) == mani_success, "read");
    uint64_t hash = maniHash64(MANI_HASH_INIT, locals, (int64_t)sizeof locals);
    CHECK(hash == PINNED_HASH, "the transfer's bits the same on every platform");
    if (hash != PINNED_HASH)
    {
        printf("hash 0x%016llx\n", (unsigned long long)hash);
    }
}

// The hips as a plain pelvis: its height ratio scales translations.
static const maniRetargetPelvisSource kHips = {
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 1.0f, 1.0f};

// A map of every source bone to the bone of its place, from source to
// target, the hips by the pelvis when one is given.
static void PairMap(const maniSkeleton* from, const maniSkeleton* to,
                    maniRetargetTranslation translation, const maniRetargetPelvisSource* pelvis,
                    uint8_t* file, size_t capacity, maniRetargetMap* out)
{
    maniRetargetPairSource pairs[8];
    uint32_t count = 0;
    for (uint32_t b = 0; b < 8; ++b)
    {
        if (pelvis == NULL || b != pelvis->target)
        {
            pairs[count++] = (maniRetargetPairSource){b, b, 1.0f, translation, true};
        }
    }
    maniRetargetMapSource source = {
        .source = from, .target = to, .pairs = pairs, .pelvis = pelvis, .pairCount = count};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, capacity, &size) == mani_success &&
              maniOpenRetargetMap(file, size, out).result == mani_success,
          "a map of pairs");
    maniDestroyBuilder(builder);
}

// Whether two poses of a skeleton's first eight bones agree: turns within
// tol, translations and scales within tol times scale.
static bool Agree(const maniPose* a, const maniPose* b, float scale, double tol)
{
    bool same = true;
    for (uint32_t k = 0; k < 8; ++k)
    {
        maniTransform x = maniPose_GetLocal(a, k);
        maniTransform y = maniPose_GetLocal(b, k);
        same = same && Near(FromQuat(x.rotation), FromQuat(y.rotation), tol) &&
               fabs((double)(x.translation.x * scale - y.translation.x)) <= tol &&
               fabs((double)(x.translation.y * scale - y.translation.y)) <= tol &&
               fabs((double)(x.translation.z * scale - y.translation.z)) <= tol &&
               fabs((double)(x.scale.x - y.scale.x)) <= tol &&
               fabs((double)(x.scale.y - y.scale.y)) <= tol &&
               fabs((double)(x.scale.z - y.scale.z)) <= tol;
    }
    return same;
}

// Whether two poses of the source hold the same locals, bit for bit.
static bool SameLocals(const maniPose* a, const maniPose* b)
{
    bool same = true;
    for (uint32_t k = 0; k < 8; ++k)
    {
        maniTransform x = maniPose_GetLocal(a, k);
        maniTransform y = maniPose_GetLocal(b, k);
        uint32_t u[10];
        uint32_t v[10];
        static_assert(sizeof x == sizeof u, "ten words");
        memcpy(u, &x, sizeof u);
        memcpy(v, &y, sizeof v);
        same = same && memcmp(u, v, sizeof u) == 0;
    }
    return same;
}

static _Alignas(16) uint8_t s_pairFiles[3][4096];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_invariantPoses[2][4096];

// The transfer's invariants: a rig to itself is the identity (each bone in
// its source's frame carries its local as it is); to the rig
// scaled uniformly, the same turns with translations scaled (by the
// pelvis's height ratio, so the map has one); a map and
// its inverse between rigs whose rests turn differently return the pose.
static void TestInvariants(void)
{
    Bone doubled[8];
    Bone turned[8];
    memcpy(doubled, kSource, sizeof doubled);
    memcpy(turned, kSource, sizeof turned);
    for (uint32_t b = 0; b < 8; ++b)
    {
        doubled[b].offset = (maniVec3){2.0f * kSource[b].offset.x, 2.0f * kSource[b].offset.y,
                                       2.0f * kSource[b].offset.z};
    }
    turned[2].rotation = (maniQuat)REST;
    turned[5].rotation = (maniQuat){0.0f, 0.70710678f, 0.0f, 0.70710678f};
    turned[6].rotation = (maniQuat){0.5f, 0.5f, 0.5f, 0.5f};
    maniSkeleton twice;
    maniSkeleton other;
    Skeleton(doubled, 8, 4, &twice);
    Skeleton(turned, 8, 5, &other);
    maniRetargetMap self;
    maniRetargetMap scaled;
    maniRetargetMap there;
    maniRetargetMap back;
    PairMap(&s_source, &s_source, maniRetargetTranslationAbsolute, NULL, s_pairFiles[0], 4096,
            &self);
    PairMap(&s_source, &twice, maniRetargetTranslationScaled, &kHips, s_pairFiles[1], 4096,
            &scaled);
    PairMap(&s_source, &other, maniRetargetTranslationScaled, NULL, s_pairFiles[2], 4096, &there);
    static _Alignas(16) uint8_t backFile[4096];
    PairMap(&other, &s_source, maniRetargetTranslationScaled, NULL, backFile, sizeof backFile,
            &back);
    maniPose* same = NULL;
    maniPose* mid = NULL;
    CHECK(maniInitPose(&s_source, s_invariantPoses[0], sizeof s_invariantPoses[0], &same) ==
              mani_success,
          "a pose of the source");
    Posed();
    CHECK(maniRetargetMap_RetargetPose(&self, &s_source, s_in, &s_source, same, s_scratch,
                                       sizeof s_scratch) == mani_success &&
              SameLocals(s_in, same),
          "a rig to itself: the identity, bit for bit");
    CHECK(maniInitPose(&twice, s_invariantPoses[1], sizeof s_invariantPoses[1], &mid) ==
                  mani_success &&
              maniRetargetMap_RetargetPose(&scaled, &s_source, s_in, &twice, mid, s_scratch,
                                           sizeof s_scratch) == mani_success &&
              Agree(s_in, mid, 2.0f, 1e-6),
          "to the rig scaled twice: the same turns, translations doubled");
    CHECK(maniInitPose(&other, s_invariantPoses[1], sizeof s_invariantPoses[1], &mid) ==
                  mani_success &&
              maniRetargetMap_RetargetPose(&there, &s_source, s_in, &other, mid, s_scratch,
                                           sizeof s_scratch) == mani_success &&
              maniRetargetMap_RetargetPose(&back, &other, mid, &s_source, same, s_scratch,
                                           sizeof s_scratch) == mani_success &&
              Agree(s_in, same, 1.0f, 1e-5),
          "there and back between rigs whose rests turn otherwise: the pose");
}

int main(void)
{
    Setup();
    TestReference();
    TestInvariants();
    TestTurns();
    TestTranslations();
    TestRefusals();
    TestReversed();
    TestSamePose();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
