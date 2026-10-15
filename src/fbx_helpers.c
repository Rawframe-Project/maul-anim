// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Helper bones for animation (mani-0014, mani-0016). An RrSs or Rrs
// model compensated at rest stays exact while its parent's inherited
// scale is uniform; where a stack makes it otherwise at a sample of its
// default clip grid, the parent needs a helper bone as at rest. Only the
// children still exact are checked, and only while some are.

#include "fbx_helpers.h"

#include "allocator.h"
#include "clip_file.h"
#include "decompose.h"
#include "fbx_anim.h"
#include "fbx_clip_state.h"
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

// One stack's grid; Capacity when the allocator fails, else success
// (a refusal skips the stack).
static maniImportResult CheckStack(const maniFbx* fbx, uint32_t k, const uint8_t* needed,
                                   maniFbxWorld* worlds, const uint32_t* children, uint32_t count,
                                   uint8_t* helper)
{
    maniFbxAnimation animation;
    maniImportResult r = maniFbxReadStack(fbx, k, &animation);
    if (r.result != mani_success)
    {
        return r.result == mani_errorCapacity ? r : maniFbxSuccess();
    }
    maniFbxSampler sampler;
    r = maniFbxStartSampler(fbx, &animation, needed, &sampler);
    double rate = maniFbxDefaultRate(fbx);
    double tps = (double)fbx->settings.ticksPerSecond;
    double span = (double)animation.stop - (double)animation.start;
    uint32_t samples = 0;
    bool timed =
        maniClipSampleCountFor(span > 0.0 ? (float)(span / tps) : 0.0f, (float)rate, &samples);
    bool left = true;
    for (uint32_t s = 0; r.result == mani_success && timed && left && s < samples; ++s)
    {
        double time = maniFbxGridTime(fbx, &animation, rate, samples, s);
        r = maniFbxSampleWorlds(&sampler, time, worlds);
        left = r.result == mani_success && CheckAt(fbx, worlds, children, count, helper);
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
    if (worlds == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    maniImportResult r = maniFbxSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < fbx->stackCount; ++k)
    {
        r = CheckStack(fbx, k, needed, worlds, children, count, helper);
    }
    maniRelease(&fbx->allocator, worlds, n, sizeof(maniFbxWorld), 8);
    return r;
}
