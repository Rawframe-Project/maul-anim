// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX models at a time of a stack (mani-0016). The needed models are
// ordered parents first once, with their inputs at rest; at each time a
// model's inputs take what the stack animates (the rotation order first,
// since composed rotations need it; integers truncated as ufbx truncates
// them), compose, and evaluate under the parent as at rest.

#include "fbx_sample.h"

#include "allocator.h"
#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_transform.h"
#include "fbx_world.h"

#include <math.h>
#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

void maniFbxReleaseSampler(maniFbxSampler* sampler)
{
    const maniFbx* fbx = sampler->fbx;
    size_t n = fbx != NULL ? (size_t)fbx->modelCount + 1 : 0;
    if (fbx != NULL)
    {
        maniRelease(&fbx->allocator, sampler->order, n, sizeof(uint32_t), 4);
        maniRelease(&fbx->allocator, sampler->rest, n, sizeof(maniFbxLocalInputs), 8);
    }
    memset(sampler, 0, sizeof *sampler);
}

maniImportResult maniFbxStartSampler(const maniFbx* fbx, const maniFbxAnimation* animation,
                                     const uint8_t* needed, maniFbxSampler* out)
{
    memset(out, 0, sizeof *out);
    out->fbx = fbx;
    out->animation = animation;
    size_t n = (size_t)fbx->modelCount + 1;
    out->order = maniAllocate(&fbx->allocator, n, sizeof(uint32_t), 4);
    out->rest = maniAllocate(&fbx->allocator, n, sizeof(maniFbxLocalInputs), 8);
    uint32_t* stack = maniAllocate(&fbx->allocator, n, sizeof(uint32_t), 4);
    maniImportResult r =
        out->order != NULL && out->rest != NULL && stack != NULL ? maniFbxSuccess() : Capacity();
    // Each model is pushed once, by its parent, from the root (the
    // hierarchy is a forest, mani-0014).
    uint32_t height = 0;
    if (stack != NULL)
    {
        stack[height++] = fbx->modelCount;
    }
    while (r.result == mani_success && height > 0)
    {
        uint32_t v = stack[--height];
        if (v != fbx->modelCount)
        {
            out->order[out->count] = v;
            r = maniFbxReadLocalInputs(fbx, fbx->models[v], &out->rest[out->count]);
            ++out->count;
        }
        const uint32_t* children = NULL;
        uint32_t count = maniFbxModelChildren(fbx, v, &children);
        for (uint32_t k = 0; k < count; ++k)
        {
            if (needed[children[k]] != 0)
            {
                stack[height++] = children[k];
            }
        }
    }
    maniRelease(&fbx->allocator, stack, n, sizeof(uint32_t), 4);
    if (r.result != mani_success)
    {
        maniFbxReleaseSampler(out);
    }
    return r;
}

// A model's inputs at a time; false for an animated integer out of range.
static bool Animate(const maniFbxSampler* s, uint32_t object, double time, maniFbxLocalInputs* in)
{
    for (uint32_t k = 0; k < maniFbxLocalIntegers; ++k)
    {
        double v[3] = {(double)in->integers[k], 0.0, 0.0};
        const char* name = maniFbxLocalIntegerNames[k];
        if (!maniFbxAnimateProperty(s->animation, s->fbx, object, name, strlen(name), time, 0, v))
        {
            continue;
        }
        double limit = (double)maniFbxLocalIntegerLimits[k];
        // Truncated toward zero as ufbx converts; NaN fails both tests.
        if (!(v[0] > -1.0 && v[0] < limit + 1.0))
        {
            return false;
        }
        in->integers[k] = (int64_t)v[0];
    }
    int64_t order = in->integers[maniFbxLocalOrder];
    uint32_t compose = order == 6 ? 0 : (uint32_t)order;
    for (uint32_t k = 0; k < maniFbxLocalVectors; ++k)
    {
        const char* name = maniFbxLocalVectorNames[k];
        (void)maniFbxAnimateProperty(s->animation, s->fbx, object, name, strlen(name), time,
                                     compose, in->vectors[k]);
    }
    return true;
}

maniImportResult maniFbxSampleWorlds(const maniFbxSampler* sampler, double time,
                                     maniFbxWorld* worlds)
{
    const maniFbx* fbx = sampler->fbx;
    for (uint32_t i = 0; i < sampler->count; ++i)
    {
        uint32_t v = sampler->order[i];
        uint32_t object = fbx->models[v];
        maniFbxLocalInputs in = sampler->rest[i];
        uint32_t first = 0;
        if (maniFbxAnimatedRun(sampler->animation, object, &first) > 0 &&
            !Animate(sampler, object, time, &in))
        {
            return maniFbxFail(maniImportReasonProperty,
                               fbx->nodes[fbx->objects[object].node].nameOffset, v);
        }
        maniFbxLocal local;
        maniFbxComposeLocal(&in, &local);
        maniFbxEvaluateModel(worlds, fbx->modelParents[v], &local, &worlds[v]);
    }
    return maniFbxSuccess();
}
