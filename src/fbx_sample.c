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
#include "fbx_value.h"
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
        maniRelease(&fbx->allocator, sampler->runs, 2 * n, sizeof(uint32_t), 4);
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
    out->runs = maniAllocate(&fbx->allocator, 2 * n, sizeof(uint32_t), 4);
    uint32_t* stack = maniAllocate(&fbx->allocator, n, sizeof(uint32_t), 4);
    maniImportResult r =
        out->order != NULL && out->rest != NULL && out->runs != NULL && stack != NULL
            ? maniFbxSuccess()
            : Capacity();
    for (uint32_t k = 0; k < maniFbxLocalVectors; ++k)
    {
        const char* name = maniFbxLocalVectorNames[k];
        out->hashes[k] = maniFbxNameHash(name, strlen(name));
    }
    for (uint32_t k = 0; k < maniFbxLocalIntegers; ++k)
    {
        const char* name = maniFbxLocalIntegerNames[k];
        out->hashes[maniFbxLocalVectors + k] = maniFbxNameHash(name, strlen(name));
    }
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
            uint32_t* run = &out->runs[2 * out->count];
            run[1] = maniFbxAnimatedRun(animation, fbx->models[v], &run[0]);
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

// Model i's inputs at a time, through its run of channels; false for an
// animated integer out of range. A model without channels keeps its
// inputs at rest.
static bool Animate(const maniFbxSampler* s, uint32_t i, double time, maniFbxLocalInputs* in)
{
    uint32_t first = s->runs[2 * i];
    uint32_t count = s->runs[2 * i + 1];
    if (count == 0)
    {
        return true;
    }
    for (uint32_t k = 0; k < maniFbxLocalIntegers; ++k)
    {
        double v[3] = {(double)in->integers[k], 0.0, 0.0};
        const char* name = maniFbxLocalIntegerNames[k];
        if (!maniFbxAnimateInRun(s->animation, s->fbx, first, count, name, strlen(name),
                                 s->hashes[maniFbxLocalVectors + k], time, 0, v))
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
        (void)maniFbxAnimateInRun(s->animation, s->fbx, first, count, name, strlen(name),
                                  s->hashes[k], time, compose, in->vectors[k]);
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
        if (!Animate(sampler, i, time, &in))
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
