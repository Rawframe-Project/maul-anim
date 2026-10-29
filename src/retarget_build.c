// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a retargeting map: each side's reference pose
// checked and taken to the model (rotations as the product of the root
// path's local rotations, positions from the full transforms); then
// pairs, chains, the pelvis and names, in that order, each target bone
// planned once. A chain's bones are placed by their distance from its
// start along the reference pose over the chain's length (by index when
// the chain has none); an interpolated chain's target bone takes the
// two source bones around its place, or one when it falls on one.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "fpenv.h"
#include "ik_chain.h"
#include "retarget_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/ik.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorInvalid, reason, item, other);
}

static bool Finite(float v)
{
    return fabsf(v) < HUGE_VALF;
}

static bool FiniteVec(maniVec3 v)
{
    return Finite(v.x) && Finite(v.y) && Finite(v.z);
}

static bool Share(float v)
{
    return v >= 0.0f && v <= 1.0f;
}

static bool Shares(maniVec3 v)
{
    return Share(v.x) && Share(v.y) && Share(v.z);
}

// Within a thousandth of length 1, as exporters' rounding leaves it.
static bool Unit(float lengthSquared)
{
    return lengthSquared >= 0.998f && lengthSquared <= 1.002f;
}

typedef struct Bone
{
    maniTransform local;
    maniQuat model;
    maniAffine affine;
} Bone;

typedef struct Plan
{
    uint32_t kind;
    uint32_t a;
    uint32_t b;
    float t;
    float weight;
    uint32_t translation;
    uint32_t flags;
    bool mapped;
} Plan;

typedef struct Work
{
    Bone* source;
    Bone* target;
    Plan* plan;
    uint32_t sourceCount;
    uint32_t targetCount;
    float ratio;
} Work;

static maniVec3 Position(const Bone* bone)
{
    return maniVec3Make(bone->affine.m[9], bone->affine.m[10], bone->affine.m[11]);
}

// A side's reference pose: each local checked, its rotation normalized,
// taken to the model from its parent's.
static maniBuildResult Side(const maniSkeleton* skeleton, const maniTransform* given, uint32_t side,
                            Bone* bones)
{
    uint32_t count = maniSkeleton_GetBoneCount(skeleton);
    for (uint32_t b = 0; b < count; ++b)
    {
        maniTransform t = given != NULL ? given[b] : maniSkeleton_GetRest(skeleton, b);
        maniQuat q = t.rotation;
        bool finite = FiniteVec(t.translation) && FiniteVec(t.scale) && Finite(q.x) &&
                      Finite(q.y) && Finite(q.z) && Finite(q.w);
        if (!finite || !Unit(maniQuatDot(q, q)))
        {
            return Refuse(maniBuildReasonRetargetReference, b, side);
        }
        t.rotation = maniQuatNormalize(q);
        maniAffine local = maniAffineFromTrs(t.translation, t.rotation, t.scale);
        uint32_t parent = maniSkeleton_GetParent(skeleton, b);
        bones[b].local = t;
        bones[b].model =
            parent == MANI_NO_BONE ? t.rotation : maniQuatMul(bones[parent].model, t.rotation);
        bones[b].affine =
            parent == MANI_NO_BONE ? local : maniAffineMul(&bones[parent].affine, &local);
    }
    return kOk;
}

static Plan One(uint32_t a, float weight, uint32_t translation, bool scale)
{
    return (Plan){maniRetargetPlanOne,
                  a,
                  MANI_NO_BONE,
                  0.0f,
                  weight,
                  translation,
                  scale ? MANI_RETARGET_PLAN_SCALE : 0u,
                  true};
}

static Plan Reference(void)
{
    return (Plan){maniRetargetPlanReference, MANI_NO_BONE, MANI_NO_BONE, 0.0f, 0.0f, 0u, 0u, true};
}

static maniBuildResult Pairs(const maniRetargetMapSource* s, Work* w)
{
    for (uint32_t i = 0; i < s->pairCount; ++i)
    {
        const maniRetargetPairSource* p = &s->pairs[i];
        bool ok = p->target < w->targetCount && p->source < w->sourceCount &&
                  !w->plan[p->target].mapped && Share(p->rotationWeight) &&
                  p->translation <= maniRetargetTranslationAbsolute;
        if (!ok)
        {
            return Refuse(maniBuildReasonRetargetPair, i, p->target);
        }
        w->plan[p->target] = One(p->source, p->rotationWeight, p->translation, p->scale);
    }
    return kOk;
}

// Each of a chain's count bones' place from 0 at its start to 1 at its
// end: its distance along the chain over the chain's length, or its
// index over count - 1 when the chain has no length.
static void Places(const Bone* bones, const uint32_t* chain, uint32_t count, float* places)
{
    float along = 0.0f;
    places[0] = 0.0f;
    for (uint32_t k = 1; k < count; ++k)
    {
        along +=
            maniVec3Length(maniVec3Sub(Position(&bones[chain[k]]), Position(&bones[chain[k - 1]])));
        places[k] = along;
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        places[k] = along > 0.0f ? places[k] / along : (float)k / (float)(count - 1);
    }
}

// The plan of an interpolated chain's target bone at place u, over the
// source bones' places v.
static Plan Interpolated(const maniRetargetChainSource* c, const uint32_t* source, const float* v,
                         uint32_t count, float u)
{
    uint32_t j = 0;
    while (j + 1 < count && v[j + 1] <= u)
    {
        ++j;
    }
    float span = j + 1 < count ? v[j + 1] - v[j] : 0.0f;
    float t = span > 0.0f ? (u - v[j]) / span : 0.0f;
    if (t <= 0.0f || t >= 1.0f)
    {
        return One(source[t >= 1.0f ? j + 1 : j], c->rotationWeight, c->translation, c->scale);
    }
    Plan p = One(source[j], c->rotationWeight, c->translation, c->scale);
    p.kind = maniRetargetPlanTwo;
    p.b = source[j + 1];
    p.t = t;
    return p;
}

// Target bone k of nt's plan in a chain of ns source bones.
static Plan ChainBone(const maniRetargetChainSource* c, const uint32_t* source, const float* v,
                      uint32_t ns, uint32_t nt, uint32_t k, float u)
{
    uint32_t fromEnd = nt - 1 - k;
    switch (c->rotation)
    {
    case maniRetargetRotationInterpolated:
        return Interpolated(c, source, v, ns, u);
    case maniRetargetRotationOneToOne:
        return k < ns ? One(source[k], c->rotationWeight, c->translation, c->scale) : Reference();
    case maniRetargetRotationOneToOneReversed:
        return fromEnd < ns
                   ? One(source[ns - 1 - fromEnd], c->rotationWeight, c->translation, c->scale)
                   : Reference();
    default:
        return Reference();
    }
}

static maniBuildResult Chain(const maniRetargetMapSource* s, Work* w, uint32_t i)
{
    const maniRetargetChainSource* c = &s->chains[i];
    uint32_t source[MANI_MAX_IK_CHAIN_BONES];
    uint32_t target[MANI_MAX_IK_CHAIN_BONES];
    bool inRange = c->sourceStart < w->sourceCount && c->sourceEnd < w->sourceCount &&
                   c->targetStart < w->targetCount && c->targetEnd < w->targetCount;
    if (inRange && (maniIkChainPathPast(s->source, c->sourceStart, c->sourceEnd) ||
                    maniIkChainPathPast(s->target, c->targetStart, c->targetEnd)))
    {
        return (maniBuildResult){mani_errorLimit, maniBuildReasonRetargetChain, i, 0};
    }
    uint32_t ns = inRange ? maniIkChainPath(s->source, c->sourceStart, c->sourceEnd, source) : 0u;
    uint32_t nt = inRange ? maniIkChainPath(s->target, c->targetStart, c->targetEnd, target) : 0u;
    if (ns == 0 || nt == 0 || c->rotation > maniRetargetRotationOneToOneReversed ||
        c->translation > maniRetargetTranslationAbsolute || !Share(c->rotationWeight))
    {
        return Refuse(maniBuildReasonRetargetChain, i, 0);
    }
    for (uint32_t k = 0; k < nt; ++k)
    {
        if (w->plan[target[k]].mapped)
        {
            return Refuse(maniBuildReasonRetargetChain, i, target[k]);
        }
    }
    float v[MANI_MAX_IK_CHAIN_BONES];
    float u[MANI_MAX_IK_CHAIN_BONES];
    Places(w->source, source, ns, v);
    Places(w->target, target, nt, u);
    for (uint32_t k = 0; k < nt; ++k)
    {
        w->plan[target[k]] = ChainBone(c, source, v, ns, nt, k, u[k]);
    }
    return kOk;
}

static maniBuildResult Pelvis(const maniRetargetMapSource* s, Work* w)
{
    const maniRetargetPelvisSource* p = s->pelvis;
    w->ratio = 1.0f;
    if (p == NULL)
    {
        return kOk;
    }
    bool ok = p->source < w->sourceCount && p->target < w->targetCount &&
              !w->plan[p->target].mapped && FiniteVec(p->up) && Unit(maniVec3Dot(p->up, p->up)) &&
              Finite(p->horizontal) && p->horizontal >= 0.0f && Finite(p->vertical) &&
              p->vertical >= 0.0f && Shares(p->absolute) && Shares(p->toSource) &&
              Share(p->translationWeight) && Share(p->rotationWeight);
    if (!ok)
    {
        return Refuse(maniBuildReasonRetargetPelvis, 0, 0);
    }
    float hs = maniVec3Dot(p->up, Position(&w->source[p->source]));
    float ht = maniVec3Dot(p->up, Position(&w->target[p->target]));
    if (!(hs > 0.0f) || !(ht > 0.0f) || !Finite(ht / hs))
    {
        return Refuse(maniBuildReasonRetargetPelvis, 0, 0);
    }
    w->ratio = ht / hs;
    w->plan[p->target] = One(p->source, p->rotationWeight, maniRetargetPlanPelvis, false);
    return kOk;
}

static void Names(const maniRetargetMapSource* s, Work* w)
{
    for (uint32_t b = 0; s->matchNames && b < w->targetCount; ++b)
    {
        size_t length = 0;
        const char* name = maniSkeleton_GetBoneName(s->target, b, &length);
        uint32_t found =
            w->plan[b].mapped ? MANI_NO_BONE : maniSkeleton_FindBone(s->source, name, length);
        if (found != MANI_NO_BONE)
        {
            w->plan[b] = One(found, 1.0f, maniRetargetTranslationOwn, false);
        }
    }
}

static maniBuildResult Check(const maniRetargetMapSource* s, Work* w)
{
    maniBuildResult r = Side(s->source, s->sourceReference, 0, w->source);
    r = r.result == mani_success ? Side(s->target, s->targetReference, 1, w->target) : r;
    r = r.result == mani_success ? Pairs(s, w) : r;
    for (uint32_t i = 0; r.result == mani_success && i < s->chainCount; ++i)
    {
        r = Chain(s, w, i);
    }
    r = r.result == mani_success ? Pelvis(s, w) : r;
    if (r.result == mani_success)
    {
        Names(s, w);
    }
    return r;
}

static void StoreF32(uint8_t* p, float v)
{
    uint32_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    maniStoreU32(p, bits);
}

static void StoreVec(uint8_t* p, maniVec3 v)
{
    StoreF32(p, v.x);
    StoreF32(p + 4, v.y);
    StoreF32(p + 8, v.z);
}

static void StoreQuat(uint8_t* p, maniQuat q)
{
    StoreF32(p, q.x);
    StoreF32(p + 4, q.y);
    StoreF32(p + 8, q.z);
    StoreF32(p + 12, q.w);
}

static void LayHeader(const maniRetargetMapSource* s, const Work* w, uint8_t* h)
{
    const maniRetargetPelvisSource* p = s->pelvis;
    maniStoreU64(h + maniRetargetHeaderSourceIdentity, maniSkeleton_GetIdentity(s->source));
    maniStoreU64(h + maniRetargetHeaderTargetIdentity, maniSkeleton_GetIdentity(s->target));
    maniStoreU32(h + maniRetargetHeaderSourceBones, w->sourceCount);
    maniStoreU32(h + maniRetargetHeaderTargetBones, w->targetCount);
    maniStoreU32(h + maniRetargetHeaderPelvisSource, p != NULL ? p->source : MANI_NO_BONE);
    maniStoreU32(h + maniRetargetHeaderPelvisTarget, p != NULL ? p->target : MANI_NO_BONE);
    StoreVec(h + maniRetargetHeaderUp,
             p != NULL ? maniVec3Normalize(p->up, maniVec3Make(0.0f, 1.0f, 0.0f))
                       : maniVec3Make(0.0f, 1.0f, 0.0f));
    StoreF32(h + maniRetargetHeaderHorizontal, p != NULL ? w->ratio * p->horizontal : 1.0f);
    StoreF32(h + maniRetargetHeaderVertical, p != NULL ? w->ratio * p->vertical : 1.0f);
    StoreF32(h + maniRetargetHeaderRatio, w->ratio);
    if (p != NULL)
    {
        StoreVec(h + maniRetargetHeaderAbsolute, p->absolute);
        StoreVec(h + maniRetargetHeaderToSource, p->toSource);
        StoreF32(h + maniRetargetHeaderTranslationWeight, p->translationWeight);
        StoreVec(h + maniRetargetHeaderSourcePelvis, Position(&w->source[p->source]));
        StoreVec(h + maniRetargetHeaderTargetPelvis, Position(&w->target[p->target]));
    }
}

static void LayBones(const Work* w, uint8_t* source, uint8_t* target, uint8_t* plan)
{
    for (uint32_t b = 0; b < w->sourceCount; ++b)
    {
        uint8_t* e = source + (size_t)b * MANI_RETARGET_SOURCE_SIZE;
        StoreQuat(e, w->source[b].model);
        StoreVec(e + 16, w->source[b].local.translation);
        StoreVec(e + 28, w->source[b].local.scale);
    }
    for (uint32_t b = 0; b < w->targetCount; ++b)
    {
        uint8_t* e = target + (size_t)b * MANI_RETARGET_TARGET_SIZE;
        StoreQuat(e, w->target[b].model);
        StoreQuat(e + 16, w->target[b].local.rotation);
        StoreVec(e + 32, w->target[b].local.translation);
        StoreVec(e + 44, w->target[b].local.scale);
        const Plan* p = &w->plan[b];
        uint8_t* r = plan + (size_t)b * MANI_RETARGET_PLAN_SIZE;
        maniStoreU32(r + maniRetargetPlanKind, p->mapped ? p->kind : maniRetargetPlanReference);
        maniStoreU32(r + maniRetargetPlanA, p->mapped ? p->a : MANI_NO_BONE);
        maniStoreU32(r + maniRetargetPlanB, p->mapped ? p->b : MANI_NO_BONE);
        StoreF32(r + maniRetargetPlanT, p->t);
        StoreF32(r + maniRetargetPlanWeight, p->weight);
        maniStoreU32(r + maniRetargetPlanTranslation, p->translation);
        maniStoreU32(r + maniRetargetPlanFlags, p->flags);
    }
}

static bool SameBits(const maniTransform* a, const maniTransform* b)
{
    static_assert(sizeof *a == 10 * sizeof(uint32_t), "ten words");
    uint32_t x[10];
    uint32_t y[10];
    memcpy(x, a, sizeof x);
    memcpy(y, b, sizeof y);
    return memcmp(x, y, sizeof x) == 0;
}

// Each target bone whose frame is its source bone's: one source at weight
// 1, the same reference local bit for bit, and a parent likewise (parents
// come first) or both roots.
static void MarkSame(const maniRetargetMapSource* s, Work* w)
{
    for (uint32_t b = 0; b < w->targetCount; ++b)
    {
        Plan* p = &w->plan[b];
        if (!p->mapped || p->kind != maniRetargetPlanOne || p->weight != 1.0f ||
            p->translation == maniRetargetPlanPelvis ||
            !SameBits(&w->source[p->a].local, &w->target[b].local))
        {
            continue;
        }
        uint32_t pb = maniSkeleton_GetParent(s->target, b);
        uint32_t pa = maniSkeleton_GetParent(s->source, p->a);
        bool parents = pb == MANI_NO_BONE
                           ? pa == MANI_NO_BONE
                           : pa != MANI_NO_BONE &&
                                 (w->plan[pb].flags & MANI_RETARGET_PLAN_SAME) != 0 &&
                                 w->plan[pb].a == pa;
        p->flags |= parents ? MANI_RETARGET_PLAN_SAME : 0u;
    }
}

static maniBuildResult Write(maniBuilder* builder, const maniRetargetMapSource* s, const Work* w)
{
    const uint64_t sizes[MANI_RETARGET_SECTIONS] = {
        MANI_RETARGET_HEADER_SIZE, (uint64_t)w->sourceCount * MANI_RETARGET_SOURCE_SIZE,
        (uint64_t)w->targetCount * MANI_RETARGET_TARGET_SIZE,
        (uint64_t)w->targetCount * MANI_RETARGET_PLAN_SIZE};
    size_t own = (size_t)(sizes[0] + sizes[1] + sizes[2] + sizes[3]);
    uint8_t* staging = maniAllocate(&builder->allocator, own, 1, 8);
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(staging, 0, own);
    uint8_t* data[MANI_RETARGET_SECTIONS] = {staging, staging + sizes[0],
                                             staging + sizes[0] + sizes[1],
                                             staging + sizes[0] + sizes[1] + sizes[2]};
    const uint8_t* views[MANI_RETARGET_SECTIONS] = {data[0], data[1], data[2], data[3]};
    maniSectionSource sections[MANI_RETARGET_SECTIONS];
    for (uint32_t i = 0; i < MANI_RETARGET_SECTIONS; ++i)
    {
        sections[i] = (maniSectionSource){i + 1, data[i], sizes[i]};
    }
    LayHeader(s, w, data[0]);
    LayBones(w, data[1], data[2], data[3]);
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, MANI_RETARGET_SECTIONS, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindRetargetMap, MANI_RETARGET_FORMAT_VERSION,
                           maniRetargetComputeIdentity(views, sizes), sections,
                           MANI_RETARGET_SECTIONS);
        maniBuilderKeep(builder, maniKindRetargetMap, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, own, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

static bool SourceUsable(const maniRetargetMapSource* s)
{
    return s != NULL && s->source != NULL && s->target != NULL &&
           (s->pairs != NULL || s->pairCount == 0) && (s->chains != NULL || s->chainCount == 0);
}

static MANI_NOINLINE maniBuildResult BuildRetargetMapWork(maniBuilder* builder,
                                                          const maniRetargetMapSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    if (!SourceUsable(source))
    {
        maniBuilderMisuse(builder);
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    Work w = {NULL,
              NULL,
              NULL,
              maniSkeleton_GetBoneCount(source->source),
              maniSkeleton_GetBoneCount(source->target),
              1.0f};
    size_t bytes = ((size_t)w.sourceCount + w.targetCount) * sizeof(Bone) +
                   (size_t)w.targetCount * sizeof(Plan);
    uint8_t* block = maniAllocate(&builder->allocator, bytes, 1, 8);
    if (block == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    memset(block, 0, bytes);
    w.source = (void*)block;
    w.target = w.source + w.sourceCount;
    w.plan = (void*)(w.target + w.targetCount);
    maniBuildResult r = Check(source, &w);
    if (r.result == mani_success)
    {
        MarkSame(source, &w);
    }
    r = r.result == mani_success ? Write(builder, source, &w) : r;
    maniRelease(&builder->allocator, block, bytes, 1, 8);
    return r;
}

maniBuildResult maniBuildRetargetMap(maniBuilder* builder, const maniRetargetMapSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildRetargetMapWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}
