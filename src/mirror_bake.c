// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking a mirrored clip through a mirror map (mani-0033): the bake core
// with each sample mirrored, the names renamed by the map's name pairs,
// the root motion on the root motion bone's partner, its moves (in that
// bone's parent's frame) reflected and carried by the partner's A.

#include "bytes.h"
#include "clip_bake.h"
#include "clip_view.h"
#include "mirror_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

typedef struct Bake
{
    const maniMirrorClipSource* s;
    // The root motion bone's partner's A: from its parent's frame to the
    // mirrored frame of the root motion bone's parent.
    maniQuat a;
} Bake;

static maniResult Carry(const void* context, const maniPose* in, maniPose* out, void* scratch,
                        size_t scratchSize)
{
    (void)scratch;
    (void)scratchSize;
    const maniMirrorClipSource* s = ((const Bake*)context)->s;
    return maniMirrorPose(s->map, s->skeleton, in, out);
}

// A move reflected, then carried by A: `A t`, `A r A^-1`.
static maniRootMove Move(const void* context, maniRootMove move)
{
    const Bake* b = context;
    maniRootMove m = maniMirrorRootMotion(b->s->map, move);
    m.translation = maniQuatRotate(b->a, m.translation);
    m.rotation = maniQuatMul(maniQuatMul(b->a, m.rotation), maniQuatConjugate(b->a));
    return m;
}

static const char* Rename(const void* context, const char* name, uint32_t* length)
{
    size_t made = 0;
    const char* partner = maniMirrorName(((const Bake*)context)->s->map, name, *length, &made);
    *length = (uint32_t)made;
    return partner;
}

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

// The root motion's source on the partner, its axes carried by A (all
// axes, and the whole rotation, where A takes an axis off the axes).
static maniRootMotionSource Declared(Bake* b, maniRootMotionSource s)
{
    const maniMirrorMap* map = b->s->map;
    uint32_t partner = maniMirrorPartner(map, s.bone);
    const uint8_t* e = map->bones + (size_t)partner * MANI_MIRROR_BONE_SIZE + 8;
    b->a = (maniQuat){LoadF32(e), LoadF32(e + 4), LoadF32(e + 8), LoadF32(e + 12)};
    uint32_t axes = 0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        uint32_t j = maniBakeAxisOf(maniQuatRotate(b->a, maniBakeUnit(i)));
        axes |= ((s.translationAxes >> i) & 1u) == 0 ? 0u : j < 3 ? 1u << j : 7u;
    }
    s.translationAxes = axes;
    if (s.rotation == maniRootRotationTwist)
    {
        uint32_t j = maniBakeAxisOf(maniQuatRotate(b->a, maniBakeUnit(s.twistAxis)));
        s.rotation = j < 3 ? maniRootRotationTwist : maniRootRotationWhole;
        s.twistAxis = j < 3 ? j : s.twistAxis;
    }
    s.bone = partner;
    return s;
}

static bool Usable(const maniMirrorClipSource* s)
{
    return s != NULL && s->map != NULL && s->skeleton != NULL && s->clip != NULL &&
           s->reserved == 0;
}

// The skeleton the map's, the clip the skeleton's.
static maniBuildResult Matches(const maniMirrorClipSource* s)
{
    const maniMirrorMap* m = s->map;
    if (maniSkeletonIdentity(s->skeleton) != m->skeletonIdentity ||
        maniSkeletonBoneCount(s->skeleton) != m->boneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonMirrorClip, 0, 0);
    }
    if (s->clip->skeletonIdentity != m->skeletonIdentity ||
        s->clip->skeletonBoneCount != m->boneCount)
    {
        return Result(mani_errorInvalid, maniBuildReasonMirrorClip, 0, 1);
    }
    return Result(mani_success, maniBuildReasonNone, 0, 0);
}

maniBuildResult maniBuildMirroredClip(maniBuilder* builder, const maniMirrorClipSource* source)
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
    maniBuildResult r = Matches(source);
    if (r.result != mani_success)
    {
        return r;
    }
    Bake b = {source, {0.0f, 0.0f, 0.0f, 1.0f}};
    maniClipBakeKind kind = {.context = &b,
                             .clip = source->clip,
                             .from = source->skeleton,
                             .to = source->skeleton,
                             .carry = Carry,
                             .move = Move,
                             .rename = Rename};
    maniRootMotionSource from;
    if (maniClipRootMotionSource(source->clip, &from))
    {
        kind.declared = Declared(&b, from);
    }
    return maniBakeClip(builder, &kind);
}
