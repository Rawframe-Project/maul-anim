// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Node names, fixed per document: each node's name decoded,
// then named as every importer names its bones (names.c).

#include "gltf_names.h"

#include "allocator.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"
#include "names.h"

#include <string.h>

// Each node's decoded name, back to back, and where each starts.
static maniImportResult Decode(const maniGltf* gltf, char* original, uint32_t* spans)
{
    uint32_t at = 0;
    for (uint32_t v = 0; v < gltf->nodeCount; ++v)
    {
        uint32_t name = MANI_JSON_NONE;
        maniImportResult r =
            maniGltfMember(&gltf->g, gltf->nodeValues[v], "name", maniJsonString, false, v, &name);
        if (r.result != mani_success)
        {
            return r;
        }
        spans[v] = at;
        if (name != MANI_JSON_NONE)
        {
            size_t raw = gltf->json.values[name].length;
            at += (uint32_t)maniJsonStringDecode(&gltf->json, name, (uint8_t*)original + at, raw);
        }
    }
    spans[gltf->nodeCount] = at;
    return maniImportSuccess();
}

maniImportResult maniGltfReadNames(maniGltf* gltf)
{
    const maniAllocator* a = &gltf->allocator;
    uint32_t count = gltf->nodeCount;
    size_t n = (size_t)count + 1;
    // Decoded names are no longer than their JSON text.
    size_t raw = 0;
    for (uint32_t v = 0; v < count; ++v)
    {
        uint32_t name = maniJsonFind(&gltf->json, gltf->nodeValues[v], "name", 4);
        raw += name != MANI_JSON_NONE ? gltf->json.values[name].length : 0;
    }
    char* original = maniAllocate(a, raw + 1, 1, 8);
    uint32_t* spans = maniAllocate(a, n, sizeof(uint32_t), 8);
    uint32_t* indices = maniAllocate(a, n, sizeof(uint32_t), 8);
    maniImportResult r = (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    if (original != NULL && spans != NULL && indices != NULL)
    {
        for (uint32_t v = 0; v < count; ++v)
        {
            indices[v] = v;
        }
        r = Decode(gltf, original, spans);
        if (r.result == mani_success)
        {
            r = maniNameBones(a, original, spans, indices, count, &gltf->names);
        }
    }
    maniRelease(a, original, raw + 1, 1, 8);
    maniRelease(a, spans, n, sizeof(uint32_t), 8);
    maniRelease(a, indices, n, sizeof(uint32_t), 8);
    return r;
}

void maniGltfReleaseNames(maniGltf* gltf)
{
    maniReleaseBoneNames(&gltf->allocator, &gltf->names);
}

const char* maniGltfNodeName(const maniGltf* gltf, uint32_t node, uint32_t* lengthOut)
{
    return maniBoneName(&gltf->names, node, lengthOut);
}

const char* maniGltfNodeBoneName(const maniGltf* gltf, uint32_t node, size_t* lengthOut)
{
    bool inRange = gltf != NULL && node < gltf->names.count;
    uint32_t length = 0;
    const char* name = inRange ? maniGltfNodeName(gltf, node, &length) : NULL;
    if (lengthOut != NULL)
    {
        *lengthOut = length;
    }
    return name;
}

uint32_t maniGltfRenamedCount(const maniGltf* gltf)
{
    return gltf != NULL ? gltf->names.renamedCount : 0;
}

uint32_t maniGltfRenamedNode(const maniGltf* gltf, uint32_t index)
{
    return gltf != NULL && index < gltf->names.renamedCount ? gltf->names.renamed[index]
                                                            : UINT32_MAX;
}

uint32_t maniGltfNodeBone(const maniGltf* gltf, const maniSkeleton* skeleton, uint32_t node)
{
    uint32_t length = 0;
    const char* name = maniGltfNodeName(gltf, node, &length);
    return maniSkeletonFindBone(skeleton, name, length);
}
