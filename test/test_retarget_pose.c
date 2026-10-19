// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The pose transfer through a retargeting map (mani-0032) from a source
// of three spine bones and a two-bone arm to a target of four spine
// bones and a three-bone arm whose rests turn sp1 and the upper arm: the
// source's reference giving the target's; each mapped bone's model turn
// from its reference the source bone's (slerped between two for an
// interpolated chain's middle bones, halved by a weight of 0.5), checked
// in binary64; unmapped bones at their reference locals bit for bit;
// translations kept, scaled through the parents' reference frames, the
// scale carried; the pelvis's change scaled by parts with its absolute
// and source blends; root motion scaled; the refusals; a transfer's bits
// pinned, the same on every platform.

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

#define PINNED_HASH 0x7fdf238cad7b91f1ull

static _Alignas(16) uint8_t s_skeletonFiles[2][4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(MANI_POSE_ALIGNMENT) uint8_t s_poseMemory[2][4096];
static _Alignas(16) uint8_t s_scratch[8192];
static maniSkeleton s_source;
static maniSkeleton s_target;
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

static const Bone kSource[8] = {
    {"root", -1, {0.0f, 0.0f, 0.0f}, REST}, {"hips", 0, {0.0f, 1.0f, 0.0f}, REST},
    {"s1", 1, {0.0f, 0.2f, 0.0f}, REST},    {"s2", 2, {0.0f, 0.2f, 0.0f}, REST},
    {"chest", 3, {0.0f, 0.2f, 0.0f}, REST}, {"armL", 4, {0.2f, 0.1f, 0.0f}, REST},
    {"handL", 5, {0.3f, 0.0f, 0.0f}, REST}, {"head", 4, {0.0f, 0.3f, 0.0f}, REST}};

// sp1 rests turned 30 degrees about z, the upper arm 90 about y; the
// offsets below them are along their own turned frames.
static const Bone kTarget[10] = {
    {"root", -1, {0.0f, 0.0f, 0.0f}, REST},
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
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 1.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 1.0f, 1.0f};

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
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {sources, count, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFiles[side], sizeof s_skeletonFiles[side], &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFiles[side], size, out).result == mani_success,
          "a skeleton");
    maniDestroyBuilder(builder);
}

static void Setup(void)
{
    Skeleton(kSource, 8, 0, &s_source);
    Skeleton(kTarget, 10, 1, &s_target);
    maniRetargetMapSource source = {.source = &s_source,
                                    .target = &s_target,
                                    .chains = kChains,
                                    .pelvis = &kPelvis,
                                    .chainCount = 2,
                                    .matchNames = true};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
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
    return maniRetargetPose(&s_map, &s_source, s_in, &s_target, s_out, s_scratch,
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

// A bone's model rotation: its root path's local rotations' product.
static Q Model(const maniPose* pose, const maniSkeleton* skeleton, uint32_t bone)
{
    Q q = FromQuat(maniPoseLocal(pose, bone).rotation);
    for (uint32_t p = maniSkeletonParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeletonParent(skeleton, p))
    {
        q = Mul(FromQuat(maniPoseLocal(pose, p).rotation), q);
    }
    return q;
}

static Q RestModel(const maniSkeleton* skeleton, uint32_t bone)
{
    Q q = FromQuat(maniSkeletonRest(skeleton, bone).rotation);
    for (uint32_t p = maniSkeletonParent(skeleton, bone); p != MANI_NO_BONE;
         p = maniSkeletonParent(skeleton, p))
    {
        q = Mul(FromQuat(maniSkeletonRest(skeleton, p).rotation), q);
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
    maniTransform t = maniPoseLocal(s_in, bone);
    Q q = Mul(FromQuat(turn), FromQuat(t.rotation));
    t.rotation = (maniQuat){(float)q.x, (float)q.y, (float)q.z, (float)q.w};
    maniPoseSetLocal(s_in, bone, t);
}

static void TestReference(void)
{
    CHECK(maniPoseSetRest(s_in, &s_source) == mani_success && Transfer(), "transferred");
    bool same = true;
    for (uint32_t b = 0; b < 10; ++b)
    {
        maniTransform t = maniPoseLocal(s_out, b);
        maniTransform r = maniSkeletonRest(&s_target, b);
        same = same && Near(FromQuat(t.rotation), FromQuat(r.rotation), 1e-12) &&
               fabsf(t.translation.x - r.translation.x) <= 1e-6f &&
               fabsf(t.translation.y - r.translation.y) <= 1e-6f &&
               fabsf(t.translation.z - r.translation.z) <= 1e-6f;
    }
    CHECK(same, "the source's reference gives the target's");
}

// A walk-like source pose: the hips, s1, s2, the chest, the arm and the
// hand turned about different axes.
static void Posed(void)
{
    CHECK(maniPoseSetRest(s_in, &s_source) == mani_success, "rest");
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
    maniTransform extra = maniPoseLocal(s_out, 8);
    maniTransform rest = maniSkeletonRest(&s_target, 8);
    CHECK(memcmp(&extra, &rest, sizeof extra) == 0 && Near(Turn(s_out, &s_target, 0), none, 0.0),
          "unmapped and extra bones: their reference locals bit for bit");
    maniVec3 sp2 = maniPoseLocal(s_out, 3).translation;
    maniVec3 own = maniSkeletonRest(&s_target, 3).translation;
    CHECK(memcmp(&sp2, &own, sizeof sp2) == 0, "their own translations kept");
}

static void TestTranslations(void)
{
    Posed();
    // The hand moved 0.1 along its parent's x and scaled 2 along x.
    maniTransform t = maniPoseLocal(s_in, 6);
    t.translation.x += 0.1f;
    t.scale.x = 2.0f;
    maniPoseSetLocal(s_in, 6, t);
    // The hips up 0.1 and along x 0.2 (their parent's frame is the
    // model's).
    maniTransform hips = maniPoseLocal(s_in, 1);
    hips.translation.x += 0.2f;
    hips.translation.y += 0.1f;
    maniPoseSetLocal(s_in, 1, hips);
    CHECK(Transfer(), "transferred");
    // The forearm (from the hand, scaled): the source's change times 0.9,
    // turned from the source arm's reference model frame into the upper
    // arm's (sp1's 30 degrees about z, then 90 about y), then the scale
    // carried.
    maniTransform forearm = maniPoseLocal(s_out, 7);
    maniVec3 rest = maniSkeletonRest(&s_target, 7).translation;
    const double change[3] = {0.09, 0.0, 0.0};
    double turned[3];
    Rotate(Mul(Conj(RestModel(&s_target, 6)), RestModel(&s_source, 5)), change, turned);
    CHECK(fabs((double)forearm.translation.x - ((double)rest.x + turned[0])) <= 1e-6 &&
              fabs((double)forearm.translation.y - ((double)rest.y + turned[1])) <= 1e-6 &&
              fabs((double)forearm.translation.z - ((double)rest.z + turned[2])) <= 1e-6 &&
              fabsf(forearm.scale.x - 2.0f) <= 1e-6f && forearm.scale.y == 1.0f,
          "a scaled translation through the parents' frames, the scale carried");
    // The pelvis: x scaled by 0.9 (0.18), then a quarter of the way to
    // the source's 0.2; y unscaled (its absolute blend 1): 0.1.
    maniTransform pelvis = maniPoseLocal(s_out, 1);
    double x = 0.18 + 0.25 * (0.2 - 0.18);
    CHECK(fabs((double)pelvis.translation.x - x) <= 1e-6 &&
              fabsf(pelvis.translation.y - 1.0f) <= 1e-6f && fabsf(pelvis.translation.z) <= 1e-6f,
          "the pelvis: scaled by parts, blended to the absolute and the source");
    maniVec3 motion = maniRetargetRootMotion(&s_map, (maniVec3){1.0f, 0.5f, -2.0f});
    CHECK(fabsf(motion.x - 0.9f) <= 1e-6f && fabsf(motion.y - 0.225f) <= 1e-6f &&
              fabsf(motion.z + 1.8f) <= 1e-6f,
          "root motion: horizontal by 0.9, vertical by 0.45");
    maniVec3 same = maniRetargetRootMotion(NULL, (maniVec3){1.0f, 2.0f, 3.0f});
    CHECK(same.x == 1.0f && same.y == 2.0f && same.z == 3.0f, "no map: unchanged");
}

static void TestRefusals(void)
{
    CHECK(maniRetargetScratchSize(&s_map) == 16u * 18u + 48u * 10u &&
              maniRetargetScratchSize(NULL) == 0,
          "the scratch: a rotation for each bone, a matrix for each target bone");
    maniResult wrong =
        maniRetargetPose(&s_map, &s_target, s_in, &s_source, s_out, s_scratch, sizeof s_scratch);
    maniResult small = maniRetargetPose(&s_map, &s_source, s_in, &s_target, s_out, s_scratch,
                                        maniRetargetScratchSize(&s_map) - 1);
    maniResult aligned = maniRetargetPose(&s_map, &s_source, s_in, &s_target, s_out, s_scratch + 8,
                                          sizeof s_scratch - 8);
    maniResult swapped = maniRetargetPose(&s_map, &s_source, (const maniPose*)s_out, &s_target,
                                          s_out, s_scratch, sizeof s_scratch);
    maniResult none =
        maniRetargetPose(NULL, &s_source, s_in, &s_target, s_out, s_scratch, sizeof s_scratch);
    CHECK(wrong == mani_errorInvalid && small == mani_errorInvalid &&
              aligned == mani_errorInvalid && swapped == mani_errorInvalid &&
              none == mani_errorInvalid,
          "skeletons swapped, scratch short or misaligned, a pose of the other, no map");
}

static void TestPinned(void)
{
    Posed();
    CHECK(Transfer(), "transferred");
    maniTransform locals[10];
    CHECK(maniPoseReadLocals(s_out, 0, 10, locals) == mani_success, "read");
    uint64_t hash = maniHash64(MANI_HASH_INIT, locals, (int32_t)sizeof locals);
    CHECK(hash == PINNED_HASH, "the transfer's bits the same on every platform");
    if (hash != PINNED_HASH)
    {
        printf("hash 0x%016llx\n", (unsigned long long)hash);
    }
}

int main(void)
{
    Setup();
    TestReference();
    TestTurns();
    TestTranslations();
    TestRefusals();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
