// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX models. A model's parent is the model (or the root)
// it is the child of through an `OO` connection; another parent is
// refused, as is a cycle, found as the shared forest finds them.
// Children are listed per parent in connection order, models with no
// parent connection under the root after those connected to it.

#include "fbx_model.h"

#include "allocator.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_value.h"
#include "hierarchy.h"
#include "names.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

static maniImportResult Refuse(const maniFbx* fbx, uint32_t model)
{
    uint64_t at = fbx->nodes[fbx->objects[fbx->models[model]].node].nameOffset;
    return maniFbxFail(maniImportReasonHierarchy, at, model);
}

static uint32_t* Words(const maniFbx* fbx, size_t count)
{
    return maniAllocate(&fbx->allocator, count + 1, sizeof(uint32_t), 8);
}

static void FreeWords(const maniFbx* fbx, uint32_t* words, size_t count)
{
    maniRelease(&fbx->allocator, words, count + 1, sizeof(uint32_t), 8);
}

// Whether a connection joins two objects, not properties.
static bool ObjectToObject(const maniFbxConnection* c)
{
    return c->childProperty == MANI_FBX_NONE && c->parentProperty == MANI_FBX_NONE;
}

// The parent end of a connection as a model, the root (modelCount), or
// MANI_FBX_NONE for an object of another kind.
static uint32_t ParentModel(const maniFbx* fbx, const maniFbxConnection* c)
{
    return c->parent == fbx->objectCount ? fbx->modelCount : maniFbxModelOf(fbx, c->parent);
}

// Each model's parent: of several parent models the last in object
// order, the root when none, as ufbx links them (each node in turn
// taking its children).
static void ReadParents(maniFbx* fbx)
{
    for (uint32_t m = 0; m < fbx->modelCount; ++m)
    {
        const uint32_t* list = NULL;
        uint32_t n = maniFbxParentConnections(fbx, fbx->models[m], &list);
        uint32_t parent = MANI_FBX_NONE;
        for (uint32_t k = 0; k < n; ++k)
        {
            const maniFbxConnection* c = &fbx->connections[list[k]];
            uint32_t p = ObjectToObject(c) ? ParentModel(fbx, c) : MANI_FBX_NONE;
            bool later = p < fbx->modelCount && (parent == MANI_FBX_NONE || p > parent);
            parent = later ? p : parent;
        }
        fbx->modelParents[m] = parent;
    }
}

// Depths and common ancestors, refusing a cycle.
static maniImportResult ReadHierarchy(maniFbx* fbx)
{
    uint32_t cycle = 0;
    maniResult r = maniHierarchyInit(&fbx->modelHierarchy, &fbx->allocator, fbx->modelParents,
                                     fbx->modelCount, &cycle);
    if (r == mani_errorFormat)
    {
        return Refuse(fbx, cycle);
    }
    return r == mani_success ? maniFbxSuccess() : Capacity();
}

// Children per parent, in connection order: the root's (the parent
// modelCount) last, then the models with no parent connection.
static void ListChildren(maniFbx* fbx, uint8_t* placed)
{
    uint32_t n = fbx->modelCount;
    uint32_t at = 0;
    memset(placed, 0, (size_t)n + 1);
    for (uint32_t p = 0; p <= n; ++p)
    {
        fbx->childStartsOfModel[p] = at;
        uint32_t object = p < n ? fbx->models[p] : fbx->objectCount;
        const uint32_t* list = NULL;
        uint32_t count = maniFbxChildConnections(fbx, object, &list);
        for (uint32_t k = 0; k < count; ++k)
        {
            const maniFbxConnection* c = &fbx->connections[list[k]];
            uint32_t child = maniFbxModelOf(fbx, c->child);
            // Listed under the parent ReadParents chose for it only.
            bool mine =
                child != MANI_FBX_NONE && fbx->modelParents[child] == (p < n ? p : MANI_FBX_NONE);
            if (ObjectToObject(c) && mine && placed[child] == 0)
            {
                placed[child] = 1;
                fbx->modelChildren[at++] = child;
            }
        }
    }
    for (uint32_t m = 0; m < n; ++m)
    {
        if (placed[m] == 0)
        {
            fbx->modelChildren[at++] = m;
        }
    }
    fbx->childStartsOfModel[n + 1] = at;
}

// Every model's name, decoded and without its class, then named.
static maniImportResult ReadNames(maniFbx* fbx)
{
    uint32_t n = fbx->modelCount;
    size_t raw = 0;
    for (uint32_t m = 0; m < n; ++m)
    {
        raw += maniFbxObjectName(fbx, fbx->models[m], NULL, 0);
    }
    char* original = maniAllocate(&fbx->allocator, raw + 1, 1, 8);
    uint32_t* spans = Words(fbx, n);
    uint32_t* indices = Words(fbx, n);
    maniImportResult r = Capacity();
    if (original != NULL && spans != NULL && indices != NULL)
    {
        uint32_t at = 0;
        for (uint32_t m = 0; m < n; ++m)
        {
            spans[m] = at;
            indices[m] = m;
            at +=
                (uint32_t)maniFbxObjectName(fbx, fbx->models[m], (uint8_t*)original + at, raw - at);
        }
        spans[n] = at;
        r = maniNameBones(&fbx->allocator, original, spans, indices, n, &fbx->modelNames);
    }
    maniRelease(&fbx->allocator, original, raw + 1, 1, 8);
    FreeWords(fbx, spans, n);
    FreeWords(fbx, indices, n);
    return r;
}

static maniImportResult ListModels(maniFbx* fbx)
{
    uint32_t n = 0;
    for (uint32_t o = 0; o < fbx->objectCount; ++o)
    {
        n += maniFbxObjectKindIs(fbx, o, "Model", 5) ? 1 : 0;
    }
    fbx->modelCount = n;
    fbx->models = Words(fbx, n);
    fbx->modelOfObject = Words(fbx, fbx->objectCount);
    fbx->modelParents = Words(fbx, n);
    fbx->modelChildren = Words(fbx, n);
    fbx->childStartsOfModel = Words(fbx, (size_t)n + 1);
    if (fbx->models == NULL || fbx->modelOfObject == NULL || fbx->modelParents == NULL ||
        fbx->modelChildren == NULL || fbx->childStartsOfModel == NULL)
    {
        return Capacity();
    }
    uint32_t m = 0;
    for (uint32_t o = 0; o < fbx->objectCount; ++o)
    {
        bool model = maniFbxObjectKindIs(fbx, o, "Model", 5);
        fbx->modelOfObject[o] = model ? m : MANI_FBX_NONE;
        if (model)
        {
            fbx->models[m++] = o;
        }
    }
    return maniFbxSuccess();
}

maniImportResult maniFbxReadModels(maniFbx* fbx)
{
    maniImportResult r = ListModels(fbx);
    if (r.result == mani_success)
    {
        ReadParents(fbx);
        r = ReadHierarchy(fbx);
    }
    if (r.result == mani_success)
    {
        uint8_t* placed = maniAllocate(&fbx->allocator, (size_t)fbx->modelCount + 1, 1, 8);
        if (placed == NULL)
        {
            return Capacity();
        }
        ListChildren(fbx, placed);
        maniRelease(&fbx->allocator, placed, (size_t)fbx->modelCount + 1, 1, 8);
        r = ReadNames(fbx);
    }
    return r;
}

void maniFbxReleaseModels(maniFbx* fbx)
{
    uint32_t n = fbx->modelCount;
    maniReleaseBoneNames(&fbx->allocator, &fbx->modelNames);
    maniHierarchyRelease(&fbx->modelHierarchy, &fbx->allocator);
    FreeWords(fbx, fbx->childStartsOfModel, (size_t)n + 1);
    FreeWords(fbx, fbx->modelChildren, n);
    FreeWords(fbx, fbx->modelParents, n);
    FreeWords(fbx, fbx->modelOfObject, fbx->objectCount);
    FreeWords(fbx, fbx->models, n);
}

uint32_t maniFbxModelOf(const maniFbx* fbx, uint32_t object)
{
    return object < fbx->objectCount ? fbx->modelOfObject[object] : MANI_FBX_NONE;
}

// The first model among an object's parents through `OO` connections,
// looking one geometry up on request.
static uint32_t DeformerParent(const maniFbx* fbx, uint32_t object, bool throughGeometry)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxParentConnections(fbx, object, &list);
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniFbxConnection* c = &fbx->connections[list[k]];
        if (c->childProperty != MANI_FBX_NONE || c->parentProperty != MANI_FBX_NONE ||
            c->parent >= fbx->objectCount)
        {
            continue;
        }
        uint32_t model = maniFbxModelOf(fbx, c->parent);
        if (model == MANI_FBX_NONE && throughGeometry &&
            maniFbxObjectKindIs(fbx, c->parent, "Geometry", 8))
        {
            model = DeformerParent(fbx, c->parent, false);
        }
        if (model != MANI_FBX_NONE)
        {
            return model;
        }
    }
    return MANI_FBX_NONE;
}

uint32_t maniFbxDeformerModel(const maniFbx* fbx, uint32_t object)
{
    return DeformerParent(fbx, object, true);
}

uint32_t maniFbxModelChildren(const maniFbx* fbx, uint32_t model, const uint32_t** out)
{
    *out = fbx->modelChildren + fbx->childStartsOfModel[model];
    return fbx->childStartsOfModel[model + 1] - fbx->childStartsOfModel[model];
}

const char* maniFbxModelName(const maniFbx* fbx, uint32_t model, uint32_t* lengthOut)
{
    return maniBoneName(&fbx->modelNames, model, lengthOut);
}

uint32_t maniFbxModelCount(const maniFbx* fbx)
{
    return fbx != NULL ? fbx->modelCount : 0;
}

const char* maniFbxModelBoneName(const maniFbx* fbx, uint32_t model, uint32_t* lengthOut)
{
    bool inRange = fbx != NULL && model < fbx->modelCount;
    uint32_t length = 0;
    const char* name = inRange ? maniFbxModelName(fbx, model, &length) : NULL;
    if (lengthOut != NULL)
    {
        *lengthOut = length;
    }
    return name;
}

uint32_t maniFbxRenamedCount(const maniFbx* fbx)
{
    return fbx != NULL ? fbx->modelNames.renamedCount : 0;
}

uint32_t maniFbxRenamedModel(const maniFbx* fbx, uint32_t index)
{
    return fbx != NULL && index < fbx->modelNames.renamedCount ? fbx->modelNames.renamed[index]
                                                               : UINT32_MAX;
}
