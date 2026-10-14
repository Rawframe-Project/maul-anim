// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening and closing FBX documents (mani-0012): the binary form told
// apart by its magic, read into one tree.

#include "allocator.h"
#include "builder.h"
#include "fbx_ascii.h"
#include "fbx_binary.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"

#include "maul-anim/import.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

bool maniFbxNameIs(const maniFbx* fbx, uint32_t node, const char* name, size_t length)
{
    const maniFbxNode* n = &fbx->nodes[node];
    return n->nameLength == length && memcmp(fbx->bytes + n->nameOffset, name, length) == 0;
}

uint32_t maniFbxFindChild(const maniFbx* fbx, uint32_t node, const char* name, size_t length)
{
    for (uint32_t c = fbx->nodes[node].firstChild; c != MANI_FBX_NONE; c = fbx->nodes[c].next)
    {
        if (maniFbxNameIs(fbx, c, name, length))
        {
            return c;
        }
    }
    return MANI_FBX_NONE;
}

static maniImportResult Open(maniFbx* fbx)
{
    maniImportResult r =
        maniFbxIsBinary(fbx->bytes, fbx->size) ? maniFbxReadBinary(fbx) : maniFbxReadAscii(fbx);
    r = r.result == mani_success ? maniFbxReadObjects(fbx) : r;
    return r.result == mani_success ? maniFbxReadModels(fbx) : r;
}

maniImportResult maniOpenFbx(maniBuilder* builder, const void* bytes, size_t size, maniFbx** fbxOut)
{
    if (fbxOut != NULL)
    {
        *fbxOut = NULL;
    }
    if (builder == NULL)
    {
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    if (bytes == NULL || fbxOut == NULL)
    {
        ++builder->misuses;
        return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
    }
    maniFbx* fbx = maniAllocate(&builder->allocator, 1, sizeof(maniFbx), 8);
    if (fbx == NULL)
    {
        return Capacity();
    }
    memset(fbx, 0, sizeof *fbx);
    fbx->allocator = builder->allocator;
    fbx->bytes = bytes;
    fbx->size = size;
    maniImportResult r = Open(fbx);
    if (r.result != mani_success)
    {
        maniCloseFbx(fbx);
        return r;
    }
    *fbxOut = fbx;
    return r;
}

void maniCloseFbx(maniFbx* fbx)
{
    if (fbx == NULL)
    {
        return;
    }
    maniAllocator allocator = fbx->allocator;
    maniFbxReleaseModels(fbx);
    maniFbxReleaseObjects(fbx);
    maniRelease(&allocator, fbx->nodes, fbx->nodeCount, sizeof(maniFbxNode), 8);
    maniRelease(&allocator, fbx->properties, (size_t)fbx->propertyCount + 1,
                sizeof(maniFbxProperty), 8);
    maniRelease(&allocator, fbx, 1, sizeof(maniFbx), 8);
}

uint32_t maniFbxVersion(const maniFbx* fbx)
{
    return fbx != NULL ? fbx->version : 0;
}
