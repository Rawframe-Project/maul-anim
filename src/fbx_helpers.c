// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Helper bones for animation (mani-0014, mani-0016). An RrSs or Rrs
// model compensated at rest stays exact while its parent's inherited
// scale is uniform; where a stack makes it otherwise at a sample of its
// default clip grid, the parent needs a helper bone as at rest. Only the
// children still exact are checked, and only while some are. A shear
// needs a scale that is not uniform: a child none of whose models up to
// the root can have one in a stack stays exact at every sample
// (rotations, translations and uniform scales compose without one), so
// a stack samples only for the others, and not at all when none is
// left.

#include "fbx_helpers.h"

#include "allocator.h"
#include "clip_file.h"
#include "decompose.h"
#include "fbx_anim.h"
#include "fbx_clip_state.h"
#include "fbx_curve.h"
#include "fbx_document.h"
#include "fbx_sample.h"
#include "fbx_take.h"
#include "fbx_transform.h"
#include "fbx_world.h"

#include <stdbool.h>
#include <string.h>

bool maniFbxCompensate(const maniFbxWorld* worlds, uint32_t child, uint32_t parent,
                       maniTransform* out)
{
    maniFbxMatrix inverse;
    if (!maniFbxMatrixInverse(&worlds[parent].world, &inverse))
    {
        return false;
    }
    maniFbxMatrix local = maniFbxMatrixMul(&inverse, &worlds[child].world);
    maniAffine affine = maniFbxMatrixAffine(&local);
    return maniDecomposeExact(&affine, out);
}

// The children still exact (their parents without a helper) at the
// worlds given: those that are not get their parents' helpers. Whether
// any is left to check.
static bool CheckAt(const maniFbx* fbx, const maniFbxWorld* worlds, const uint32_t* children,
                    uint32_t count, uint8_t* helper)
{
    bool left = false;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t c = children[i];
        uint32_t p = fbx->modelParents[c];
        maniTransform unused;
        if (helper[p] == 0 && !maniFbxCompensate(worlds, c, p, &unused))
        {
            helper[p] = 1;
        }
        left = left || helper[p] == 0;
    }
    return left;
}

static bool SameKey(const maniFbxKey* a, const maniFbxKey* b)
{
    return a->time == b->time && a->value == b->value && a->leftDx == b->leftDx &&
           a->leftDy == b->leftDy && a->rightDx == b->rightDx && a->rightDy == b->rightDy &&
           a->interpolation == b->interpolation;
}

// Whether two components evaluate alike at every time: no curve and the
// same static value, or the same curve key for key.
static bool SameComponent(const maniFbxChannel* c, uint32_t i, uint32_t j)
{
    const maniFbxCurve* a = &c->curves[i];
    const maniFbxCurve* b = &c->curves[j];
    if (a->count != b->count)
    {
        return false;
    }
    if (a->count == 0)
    {
        return c->values[i] == c->values[j];
    }
    bool same = a->pre.mode == b->pre.mode && a->pre.repeat == b->pre.repeat &&
                a->post.mode == b->post.mode && a->post.repeat == b->post.repeat &&
                a->ticksPerSecond == b->ticksPerSecond && a->firstTick == b->firstTick &&
                a->lastTick == b->lastTick;
    for (uint32_t k = 0; same && k < a->count; ++k)
    {
        same = SameKey(&a->keys[k], &b->keys[k]);
    }
    return same;
}

// Whether a model may carry a scale that is not uniform in a stack: its
// own at rest, or a Lcl Scaling channel whose three components are not
// one curve (layers combine uniform scales into uniform ones). No other
// property scales: offsets and pivots translate, rotations turn,
// geometric transforms are not inherited, and an inherit type only
// chooses which scales above reach a model, uniform when all are.
static bool MayScale(const maniFbx* fbx, const maniFbxAnimation* animation, uint32_t object,
                     const maniFbxLocalInputs* rest)
{
    const double* s = rest->vectors[maniFbxLocalScaling];
    if (s[0] != s[1] || s[1] != s[2])
    {
        return true;
    }
    uint32_t first = 0;
    uint32_t run = maniFbxAnimatedRun(animation, object, &first);
    for (uint32_t i = first; i < first + run; ++i)
    {
        const maniFbxChannel* c = &animation->channels[i];
        if (maniFbxChannelNameIs(fbx, c, "Lcl Scaling", 11) &&
            !(SameComponent(c, 0, 1) && SameComponent(c, 0, 2)))
        {
            return true;
        }
    }
    return false;
}

// The children still exact that a stack could make otherwise (a model
// up to the root that may scale unevenly), into risky; their count.
static uint32_t Risky(const maniFbx* fbx, const maniFbxAnimation* animation,
                      const maniFbxSampler* sampler, const uint32_t* children, uint32_t count,
                      const uint8_t* helper, uint8_t* mark, uint32_t* risky)
{
    // Parents come first, so a model's mark follows its parent's.
    for (uint32_t i = 0; i < sampler->count; ++i)
    {
        uint32_t v = sampler->order[i];
        uint32_t p = fbx->modelParents[v];
        bool above = p != MANI_FBX_NONE && mark[p] != 0;
        mark[v] = above || MayScale(fbx, animation, fbx->models[v], &sampler->rest[i]) ? 1 : 0;
    }
    uint32_t n = 0;
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t c = children[i];
        if (helper[fbx->modelParents[c]] == 0 && mark[c] != 0)
        {
            risky[n++] = c;
        }
    }
    return n;
}

// The children a stack's Lcl Scaling channels make risky, from a read
// of those channels alone: none whatever the rest of the stack holds
// (a refusal anywhere in it would skip it), so the whole stack is read
// only when there are some. Capacity when the allocator fails, else
// success.
static maniImportResult AnyRisky(const maniFbx* fbx, uint32_t k, const uint8_t* needed,
                                 const uint32_t* children, uint32_t count, const uint8_t* helper,
                                 uint8_t* mark, uint32_t* risky, uint32_t* left)
{
    *left = 0;
    maniFbxAnimation scales;
    maniImportResult r = maniFbxReadStackOf(fbx, k, "Lcl Scaling", 11, &scales);
    if (r.result != mani_success)
    {
        return r.result == mani_errorCapacity ? r : maniFbxSuccess();
    }
    maniFbxSampler sampler;
    r = maniFbxStartSampler(fbx, &scales, needed, &sampler);
    *left = r.result == mani_success
                ? Risky(fbx, &scales, &sampler, children, count, helper, mark, risky)
                : 0;
    maniFbxReleaseSampler(&sampler);
    maniFbxReleaseAnimation(fbx, &scales);
    return r.result == mani_errorCapacity ? r : maniFbxSuccess();
}

// One stack's grid; Capacity when the allocator fails, else success
// (a refusal skips the stack).
static maniImportResult CheckStack(const maniFbx* fbx, uint32_t k, const uint8_t* needed,
                                   maniFbxWorld* worlds, const uint32_t* children, uint32_t count,
                                   uint8_t* helper, uint8_t* mark, uint32_t* risky)
{
    uint32_t left = 0;
    maniImportResult r = AnyRisky(fbx, k, needed, children, count, helper, mark, risky, &left);
    if (r.result != mani_success || left == 0)
    {
        return r;
    }
    maniFbxAnimation animation;
    r = maniFbxReadStack(fbx, k, &animation);
    if (r.result != mani_success)
    {
        return r.result == mani_errorCapacity ? r : maniFbxSuccess();
    }
    maniFbxSampler sampler;
    r = maniFbxStartSampler(fbx, &animation, needed, &sampler);
    left = r.result == mani_success
               ? Risky(fbx, &animation, &sampler, children, count, helper, mark, risky)
               : 0;
    double rate = maniFbxDefaultRate(fbx);
    double tps = (double)fbx->settings.ticksPerSecond;
    double span = (double)animation.stop - (double)animation.start;
    uint32_t samples = 0;
    bool timed =
        maniClipSampleCountFor(span > 0.0 ? (float)(span / tps) : 0.0f, (float)rate, &samples);
    for (uint32_t s = 0; r.result == mani_success && timed && left > 0 && s < samples; ++s)
    {
        double time = maniFbxGridTime(fbx, &animation, rate, samples, s);
        r = maniFbxSampleWorlds(&sampler, time, worlds);
        left = r.result == mani_success && CheckAt(fbx, worlds, risky, left, helper) ? left : 0;
    }
    // Safe after a refused start too: it released itself.
    maniFbxReleaseSampler(&sampler);
    maniFbxReleaseAnimation(fbx, &animation);
    return r.result == mani_errorCapacity ? r : maniFbxSuccess();
}

maniImportResult maniFbxAnimatedHelpers(const maniFbx* fbx, const uint8_t* needed,
                                        const uint32_t* children, uint32_t count, uint8_t* helper)
{
    if (count == 0 || fbx->stackCount == 0)
    {
        return maniFbxSuccess();
    }
    size_t n = (size_t)fbx->modelCount + 1;
    maniFbxWorld* worlds = maniAllocate(&fbx->allocator, n, sizeof(maniFbxWorld), 8);
    uint8_t* mark = maniAllocate(&fbx->allocator, n, 1, 1);
    uint32_t* risky = maniAllocate(&fbx->allocator, count, sizeof(uint32_t), 4);
    maniImportResult r =
        worlds != NULL && mark != NULL && risky != NULL
            ? maniFbxSuccess()
            : (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    for (uint32_t k = 0; r.result == mani_success && k < fbx->stackCount; ++k)
    {
        r = CheckStack(fbx, k, needed, worlds, children, count, helper, mark, risky);
    }
    maniRelease(&fbx->allocator, risky, count, sizeof(uint32_t), 4);
    maniRelease(&fbx->allocator, mark, n, 1, 1);
    maniRelease(&fbx->allocator, worlds, n, sizeof(maniFbxWorld), 8);
    return r;
}
