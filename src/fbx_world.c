// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX worlds, ufbx's evaluation in binary64: a normal model's
// world is its parent's times its local; an RrSs or Rrs model's is its
// parent's unscaled world times its local with the translation scaled by
// the parent's inherited scale and the scale by its scale source's.
// Models are evaluated parents first, by a walk of the children lists.

#include "fbx_world.h"

#include "allocator.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_transform.h"

#include <string.h>

static const maniFbxWorld kRoot = {
    {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}},
    {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}},
    {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}},
    {1, 1, 1},
    maniFbxInheritRSrs,
    MANI_FBX_NONE,
};

// The transform without its scale.
static maniFbxMatrix Unscaled(const maniFbxTrs* t)
{
    maniFbxTrs r = *t;
    r.scale[0] = r.scale[1] = r.scale[2] = 1.0;
    return maniFbxTrsMatrix(&r);
}

void maniFbxEvaluateModel(const maniFbxWorld* worlds, uint32_t parent, const maniFbxLocal* local,
                          maniFbxWorld* out)
{
    const maniFbxWorld* p = parent != MANI_FBX_NONE ? &worlds[parent] : &kRoot;
    out->transform = local->trs;
    out->inherit = local->inherit;
    out->scaleSource = local->inherit == maniFbxInheritRrSs  ? parent
                       : local->inherit == maniFbxInheritRrs ? p->scaleSource
                                                             : MANI_FBX_NONE;
    maniFbxMatrix linear = maniFbxTrsMatrix(&out->transform);
    maniFbxMatrix rigid = Unscaled(&out->transform);
    if (local->inherit == maniFbxInheritRSrs)
    {
        out->world = maniFbxMatrixMul(&p->world, &linear);
        out->unscaled = maniFbxMatrixMul(&p->world, &rigid);
    }
    else
    {
        const double* source = out->scaleSource != MANI_FBX_NONE
                                   ? worlds[out->scaleSource].inheritScale
                                   : kRoot.inheritScale;
        for (uint32_t i = 0; i < 3; ++i)
        {
            out->transform.scale[i] *= source[i];
            out->transform.translation[i] *= p->inheritScale[i];
        }
        linear = maniFbxTrsMatrix(&out->transform);
        rigid = Unscaled(&out->transform);
        out->world = maniFbxMatrixMul(&p->unscaled, &linear);
        out->unscaled = maniFbxMatrixMul(&p->unscaled, &rigid);
    }
    memcpy(out->inheritScale, out->transform.scale, sizeof out->inheritScale);
}

maniImportResult maniFbxEvaluateWorlds(const maniFbx* fbx, const uint8_t* needed,
                                       maniFbxWorld* worlds)
{
    uint32_t n = fbx->modelCount;
    uint32_t* stack = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(uint32_t), 8);
    if (stack == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    // Each model is pushed once, by its parent: the hierarchy is a forest,
    // every model under the root.
    uint32_t height = 0;
    stack[height++] = n;
    maniFbxTemplateCache cache;
    maniFbxEmptyTemplateCache(&cache);
    maniImportResult r = maniFbxSuccess();
    while (r.result == mani_success && height > 0)
    {
        uint32_t v = stack[--height];
        if (v != n)
        {
            maniFbxLocal local;
            r = maniFbxModelLocal(fbx, fbx->models[v], &cache, &local);
            uint32_t parent = fbx->modelParents[v];
            if (r.result == mani_success)
            {
                maniFbxEvaluateModel(worlds, parent, &local, &worlds[v]);
            }
        }
        const uint32_t* children = NULL;
        uint32_t count = maniFbxModelChildren(fbx, v, &children);
        for (uint32_t k = 0; r.result == mani_success && k < count; ++k)
        {
            if (needed == NULL || needed[children[k]] != 0)
            {
                stack[height++] = children[k];
            }
        }
    }
    maniRelease(&fbx->allocator, stack, (size_t)n + 1, sizeof(uint32_t), 8);
    return r;
}
