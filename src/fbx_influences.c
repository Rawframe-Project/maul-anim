// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX skins' vertex weights (mani-0021), as ufbx reads them: a vertex a
// control point of the skin's mesh, each cluster's Indexes and Weights
// its joint's, indices past the control points skipped, a control point
// given twice by one cluster summed (ufbx keeps both, so its skinning
// sum adds them), negative weights refused; SkinningType and the
// per-vertex blend weights (BlendIndexes, BlendWeights) for the method.

#include "arena.h"
#include "fbx_document.h"
#include "fbx_object.h"
#include "fbx_skin.h"
#include "fbx_value.h"
#include "sort.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniImportResult Invalid(void)
{
    return (maniImportResult){mani_errorInvalid, maniImportReasonNone, 0, 0, 0};
}

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

static uint32_t SkinNode(const maniFbx* fbx, uint32_t skin)
{
    return fbx->objects[fbx->skins[skin]].node;
}

// A child node's value count (its one array's, or its scalars, 6.x's
// form); 0 when absent or unreadable.
static uint32_t ValueCount(const maniFbx* fbx, uint32_t node, const char* name)
{
    uint32_t child = maniFbxFindChild(fbx, node, name, strlen(name));
    uint32_t count = 0;
    if (child == MANI_FBX_NONE ||
        maniFbxReadValues(fbx, child, NULL, &count).result != mani_success)
    {
        return 0;
    }
    return count;
}

// Whether the skin gives per-vertex blend weights.
static bool GivesBlend(const maniFbx* fbx, uint32_t skin)
{
    uint32_t node = SkinNode(fbx, skin);
    return ValueCount(fbx, node, "Indexes") > 0 && ValueCount(fbx, node, "BlendWeights") > 0;
}

// SkinningType is this word.
static bool TypeIs(const maniFbx* fbx, uint32_t skin, const char* word)
{
    uint32_t child = maniFbxFindChild(fbx, SkinNode(fbx, skin), "SkinningType", 12);
    return child != MANI_FBX_NONE &&
           maniFbxStringEquals(fbx, maniFbxNodeProperty(fbx, child, 0), word, strlen(word));
}

maniSkinningMethod maniFbxSkinMethod(const maniFbx* fbx, uint32_t skin)
{
    if (fbx == NULL || skin >= fbx->skinCount)
    {
        return maniSkinningLinear;
    }
    bool blend = GivesBlend(fbx, skin);
    if (TypeIs(fbx, skin, "DualQuaternion") && !blend)
    {
        return maniSkinningDualQuaternion;
    }
    return blend || TypeIs(fbx, skin, "Blend") || TypeIs(fbx, skin, "DualQuaternion")
               ? maniSkinningBlended
               : maniSkinningLinear;
}

// The control points of the skin's mesh: the Vertices of the geometry
// (7.x) or model (6.x) it deforms; Property when there are none.
static maniImportResult ControlPoints(const maniFbx* fbx, uint32_t skin, uint32_t* countOut)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxParentConnections(fbx, fbx->skins[skin], &list);
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniFbxConnection* c = &fbx->connections[list[k]];
        if (c->childProperty != MANI_FBX_NONE || c->parentProperty != MANI_FBX_NONE ||
            c->parent >= fbx->objectCount)
        {
            continue;
        }
        uint32_t node = fbx->objects[c->parent].node;
        uint32_t floats = ValueCount(fbx, node, "Vertices");
        if (floats > 0)
        {
            *countOut = floats / 3;
            uint32_t child = maniFbxFindChild(fbx, node, "Vertices", 8);
            return floats % 3 == 0
                       ? maniFbxSuccess()
                       : maniFbxFail(maniImportReasonProperty, fbx->nodes[child].nameOffset, skin);
        }
    }
    return maniFbxFail(maniImportReasonProperty, fbx->nodes[SkinNode(fbx, skin)].nameOffset, skin);
}

// One weighted control point of a joint.
typedef struct Entry
{
    uint32_t vertex;
    uint32_t joint;
    double weight;
} Entry;

typedef struct Read
{
    const maniFbx* fbx;
    maniArena* arena;
    uint32_t skin;
    uint32_t vertices;
    Entry* entries;
    uint32_t count;
} Read;

// A child node's values as binary64 into the arena (NULL and 0 when
// absent); its node out for offsets.
static maniImportResult Values(Read* r, uint32_t node, const char* name, double** out,
                               uint32_t* countOut, uint32_t* childOut)
{
    const maniFbx* fbx = r->fbx;
    *out = NULL;
    *countOut = 0;
    *childOut = maniFbxFindChild(fbx, node, name, strlen(name));
    if (*childOut == MANI_FBX_NONE)
    {
        return maniFbxSuccess();
    }
    maniImportResult result = maniFbxReadValues(fbx, *childOut, NULL, countOut);
    if (result.result != mani_success)
    {
        return result;
    }
    *out = maniArenaTake(r->arena, *countOut, sizeof(double));
    if (*out == NULL)
    {
        return Capacity();
    }
    return maniFbxReadValues(fbx, *childOut, *out, countOut);
}

// A whole index, or -1 for one that is not.
static int64_t Index(double value)
{
    return value >= 0.0 && value < 4294967296.0 && value == (double)(int64_t)value ? (int64_t)value
                                                                                   : -1;
}

// A cluster's Indexes and Weights as entries of its joint: indices past
// the control points skipped, zero weights dropped, negative refused.
static maniImportResult ReadCluster(Read* r, uint32_t cluster, uint32_t joint)
{
    const maniFbx* fbx = r->fbx;
    uint32_t node = fbx->objects[cluster].node;
    double* index = NULL;
    double* weight = NULL;
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t indexes = 0;
    uint32_t weights = 0;
    maniImportResult result = Values(r, node, "Indexes", &index, &a, &indexes);
    result =
        result.result == mani_success ? Values(r, node, "Weights", &weight, &b, &weights) : result;
    if (result.result != mani_success)
    {
        return result;
    }
    if (a != b)
    {
        return maniFbxFail(maniImportReasonProperty, fbx->nodes[node].nameOffset, r->skin);
    }
    for (uint32_t i = 0; index != NULL && weight != NULL && i < a; ++i)
    {
        int64_t v = Index(index[i]);
        bool inside = v >= 0 && v < (int64_t)r->vertices;
        if (weight[i] < 0.0)
        {
            return maniFbxFail(maniImportReasonInfluence, fbx->nodes[weights].nameOffset,
                               inside ? (uint32_t)v : 0);
        }
        if (inside && weight[i] > 0.0)
        {
            r->entries[r->count++] = (Entry){(uint32_t)v, joint, weight[i]};
        }
    }
    return maniFbxSuccess();
}

// The per-vertex blend weights: the method's default, then BlendIndexes
// and BlendWeights (the shorter's count, later entries winning).
static maniImportResult ReadBlend(Read* r, maniSkinningMethod method, float** out)
{
    const maniFbx* fbx = r->fbx;
    *out = NULL;
    if (method == maniSkinningLinear)
    {
        return maniFbxSuccess();
    }
    float* blend = maniArenaTake(r->arena, r->vertices, sizeof(float));
    if (blend == NULL)
    {
        return Capacity();
    }
    float initial = TypeIs(fbx, r->skin, "DualQuaternion") ? 1.0f : 0.0f;
    for (uint32_t v = 0; v < r->vertices; ++v)
    {
        blend[v] = initial;
    }
    uint32_t node = SkinNode(fbx, r->skin);
    double* index = NULL;
    double* weight = NULL;
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t indexes = 0;
    uint32_t weights = 0;
    maniImportResult result = Values(r, node, "Indexes", &index, &a, &indexes);
    result = result.result == mani_success ? Values(r, node, "BlendWeights", &weight, &b, &weights)
                                           : result;
    uint32_t n = a < b ? a : b;
    for (uint32_t i = 0; result.result == mani_success && index != NULL && weight != NULL && i < n;
         ++i)
    {
        int64_t v = Index(index[i]);
        if (v < 0 || v >= (int64_t)r->vertices)
        {
            continue;
        }
        if (!(weight[i] >= 0.0 && weight[i] <= 1.0))
        {
            return maniFbxFail(maniImportReasonInfluence, fbx->nodes[weights].nameOffset,
                               (uint32_t)v);
        }
        blend[v] = (float)weight[i];
    }
    *out = blend;
    return result;
}

// The entries sorted by control point (stably, so a cluster's own
// repeats stay together) into the builder's lists, a vertex's repeats
// of one joint summed.
static maniImportResult Lists(Read* r, uint32_t* offsets, uint32_t* joints, float* weights)
{
    uint64_t* keys = maniArenaTake(r->arena, r->count, sizeof(uint64_t));
    uint32_t* order = maniArenaTake(r->arena, r->count, sizeof(uint32_t));
    uint32_t* scratch = maniArenaTake(r->arena, r->count, sizeof(uint32_t));
    if (keys == NULL || order == NULL || scratch == NULL)
    {
        return Capacity();
    }
    for (uint32_t i = 0; i < r->count; ++i)
    {
        keys[i] = r->entries[i].vertex;
    }
    maniSortByKey(keys, r->count, order, scratch);
    uint32_t n = 0;
    uint32_t i = 0;
    for (uint32_t v = 0; v < r->vertices; ++v)
    {
        offsets[v] = n;
        while (i < r->count && r->entries[order[i]].vertex == v)
        {
            const Entry* e = &r->entries[order[i++]];
            double sum = e->weight;
            while (i < r->count && r->entries[order[i]].vertex == v &&
                   r->entries[order[i]].joint == e->joint)
            {
                sum += r->entries[order[i++]].weight;
            }
            joints[n] = e->joint;
            weights[n++] = (float)sum;
        }
    }
    offsets[r->vertices] = n;
    return maniFbxSuccess();
}

static maniImportResult BuildInfluences(maniBuilder* builder, Read* r, const maniSkin* built)
{
    const maniFbx* fbx = r->fbx;
    maniImportResult result = ControlPoints(fbx, r->skin, &r->vertices);
    uint32_t clusters = maniFbxSkinClusters(fbx, r->skin, NULL);
    uint32_t* cluster = maniArenaTake(r->arena, clusters, sizeof(uint32_t));
    if (result.result == mani_success && cluster == NULL)
    {
        return Capacity();
    }
    uint64_t given = 0;
    if (result.result == mani_success)
    {
        (void)maniFbxSkinClusters(fbx, r->skin, cluster);
        for (uint32_t j = 0; j < clusters; ++j)
        {
            given += ValueCount(fbx, fbx->objects[cluster[j]].node, "Indexes");
        }
        r->entries = given <= UINT32_MAX ? maniArenaTake(r->arena, given, sizeof(Entry)) : NULL;
        result = r->entries != NULL ? result : Capacity();
    }
    for (uint32_t j = 0; result.result == mani_success && j < clusters; ++j)
    {
        result = ReadCluster(r, cluster[j], j);
    }
    float* blend = NULL;
    if (result.result == mani_success)
    {
        result = ReadBlend(r, maniFbxSkinMethod(fbx, r->skin), &blend);
    }
    uint32_t* offsets = maniArenaTake(r->arena, (size_t)r->vertices + 1, sizeof(uint32_t));
    uint32_t* joints = maniArenaTake(r->arena, r->count, sizeof(uint32_t));
    float* weights = maniArenaTake(r->arena, r->count, sizeof(float));
    if (result.result == mani_success && (offsets == NULL || joints == NULL || weights == NULL))
    {
        return Capacity();
    }
    if (result.result == mani_success)
    {
        result = Lists(r, offsets, joints, weights);
    }
    if (result.result != mani_success)
    {
        return result;
    }
    maniInfluenceSource source = {built, offsets, joints, weights, blend, r->vertices, 0};
    maniBuildResult b = maniBuildInfluences(builder, &source);
    if (b.result == mani_success || b.result == mani_errorCapacity)
    {
        return (maniImportResult){b.result, maniImportReasonNone, 0, 0, 0};
    }
    return maniFbxFail(maniImportReasonBuild, fbx->nodes[SkinNode(fbx, r->skin)].nameOffset,
                       (uint32_t)b.reason);
}

maniImportResult maniFbxBuildInfluences(maniBuilder* builder, const maniFbx* fbx, uint32_t skin,
                                        const maniSkin* built)
{
    if (builder == NULL || fbx == NULL || built == NULL || skin >= fbx->skinCount)
    {
        return Invalid();
    }
    maniArena arena = maniArenaMake(&fbx->allocator);
    Read r = {fbx, &arena, skin, 0, NULL, 0};
    maniImportResult result = BuildInfluences(builder, &r, built);
    maniArenaRelease(&arena);
    return result;
}
