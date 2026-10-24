// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking a clip through a retargeting map: the bake core
// with each sample carried by the pose transfer into the target and the
// root motion's moves scaled and turned into the target bone's parent's
// frame.

#include "clip_bake.h"
#include "clip_view.h"
#include "fpenv.h"
#include "retarget_file.h"
#include "retarget_motion.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

typedef struct Bake
{
    const maniRetargetClipSource* s;
    // The target bone taking the root motion, MANI_NO_BONE without it.
    uint32_t root;
    // The frames its moves carry between.
    maniRetargetMotion motion;
} Bake;

static maniResult Carry(const void* context, const maniPose* in, maniPose* out, void* scratch,
                        size_t scratchSize)
{
    const maniRetargetClipSource* s = ((const Bake*)context)->s;
    return maniRetargetPose(s->map, s->source, in, s->target, out, scratch, scratchSize);
}

static maniRootMove Move(const void* context, maniRootMove move)
{
    return maniRetargetMove(&((const Bake*)context)->motion, move);
}

// The root motion's target bone and frames; a refusal for a bone past
// the target's or none following the source's.
static maniBuildResult PlaceRoot(Bake* b)
{
    const maniRetargetClipSource* s = b->s;
    maniRootMotionSource from;
    b->root = MANI_NO_BONE;
    if (!maniClipRootMotionSource(s->clip, &from))
    {
        return Result(mani_success, maniBuildReasonNone, 0, 0);
    }
    b->root = s->rootMotionBone != MANI_NO_BONE ? s->rootMotionBone
                                                : maniRetargetFollower(s->map, from.bone);
    if (b->root >= s->map->targetBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRootMotion, s->rootMotionBone, 0);
    }
    b->motion = maniRetargetMotionOf(s->map, s->source, from.bone, s->target, b->root);
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

// The target's root motion source: the source's on the target bone, its
// axes carried (a move's image along each, or the axis turned where the
// scales take it to nothing), all axes or the whole rotation where one
// leaves the axes.
static maniRootMotionSource Declared(const Bake* b, maniRootMotionSource s)
{
    uint32_t axes = 0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (((s.translationAxes >> i) & 1u) == 0)
        {
            continue;
        }
        maniVec3 v = maniRetargetMotionTranslation(&b->motion, maniBakeUnit(i));
        v = maniVec3Length(v) < 1e-6f ? maniQuatRotate(b->motion.turn, maniBakeUnit(i)) : v;
        uint32_t j = maniBakeAxisOf(v);
        axes |= j < 3 ? 1u << j : 7u;
    }
    s.translationAxes = axes;
    if (s.rotation == maniRootRotationTwist)
    {
        uint32_t j = maniBakeAxisOf(maniQuatRotate(b->motion.turn, maniBakeUnit(s.twistAxis)));
        s.rotation = j < 3 ? maniRootRotationTwist : maniRootRotationWhole;
        s.twistAxis = j < 3 ? j : s.twistAxis;
    }
    s.bone = b->root;
    return s;
}

static bool Usable(const maniRetargetClipSource* s)
{
    return s != NULL && s->map != NULL && s->source != NULL && s->target != NULL &&
           s->clip != NULL && s->reserved == 0;
}

// The skeletons the map's, the clip the source's.
static maniBuildResult Matches(const maniRetargetClipSource* s)
{
    const maniRetargetMap* m = s->map;
    if (maniSkeletonIdentity(s->source) != m->sourceIdentity ||
        maniSkeletonBoneCount(s->source) != m->sourceBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRetargetClip, 0, 0);
    }
    if (maniSkeletonIdentity(s->target) != m->targetIdentity ||
        maniSkeletonBoneCount(s->target) != m->targetBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRetargetClip, 0, 1);
    }
    if (s->clip->skeletonIdentity != m->sourceIdentity ||
        s->clip->skeletonBoneCount != m->sourceBoneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonRetargetClip, 0, 2);
    }
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

static MANI_NOINLINE maniBuildResult BuildRetargetedClipWork(maniBuilder* builder,
                                                             const maniRetargetClipSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (!Usable(source))
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    Bake b = {source, MANI_NO_BONE, {0}};
    maniBuildResult r = Matches(source);
    r = r.result == mani_success ? PlaceRoot(&b) : r;
    if (r.result != mani_success)
    {
        return r;
    }
    maniClipBakeKind kind = {.context = &b,
                             .clip = source->clip,
                             .from = source->source,
                             .to = source->target,
                             .scratchSize = maniRetargetScratchSize(source->map),
                             .carry = Carry,
                             .move = Move};
    maniRootMotionSource from;
    if (maniClipRootMotionSource(source->clip, &from))
    {
        kind.declared = Declared(&b, from);
    }
    return maniBakeClip(builder, &kind);
}

maniBuildResult maniBuildRetargetedClip(maniBuilder* builder, const maniRetargetClipSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildRetargetedClipWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
