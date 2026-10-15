// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX blend shape channels (mani-0017), as ufbx reads them. A channel
// (a BlendShapeChannel deformer under a BlendShape) holds Shape
// geometries; FullWeights gives each its target weight in percent, but
// Blender's binary exporter writes per-vertex weights there, so in its
// files every target is 1. At a channel weight, the two shapes around it
// share the weight linearly, a missing end standing for the rest.

#include "fbx_blend.h"

#include "allocator.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_value.h"
#include "sort.h"

#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
}

bool maniFbxIsBlendChannel(const maniFbx* fbx, uint32_t object)
{
    return maniFbxDeformerIs(fbx, object, "BlendShapeChannel", 17);
}

// The BlendShape a channel hangs from (OO), or MANI_FBX_NONE.
static uint32_t BlendShape(const maniFbx* fbx, uint32_t channel)
{
    const uint32_t* list = NULL;
    uint32_t n = maniFbxParentConnections(fbx, channel, &list);
    for (uint32_t k = 0; k < n; ++k)
    {
        const maniFbxConnection* c = &fbx->connections[list[k]];
        if (c->childProperty == MANI_FBX_NONE && c->parentProperty == MANI_FBX_NONE &&
            c->parent < fbx->objectCount && maniFbxDeformerIs(fbx, c->parent, "BlendShape", 10))
        {
            return c->parent;
        }
    }
    return MANI_FBX_NONE;
}

uint32_t maniFbxBlendChannelModel(const maniFbx* fbx, uint32_t channel)
{
    uint32_t shape = BlendShape(fbx, channel);
    return shape != MANI_FBX_NONE ? maniFbxDeformerModel(fbx, shape) : MANI_FBX_NONE;
}

// One character of ufbx's version-string match: a space skips blanks,
// '-' skips past the next '-', '?' takes a run of digits, a letter
// matches either case and '.' itself.
static bool Step(char c, const uint8_t* s, uint32_t length, uint32_t* at)
{
    if (c == ' ')
    {
        while (*at < length && (s[*at] == ' ' || s[*at] == '\t'))
        {
            ++*at;
        }
        return true;
    }
    if (c == '-')
    {
        while (*at < length && s[*at] != '-')
        {
            ++*at;
        }
        return (*at)++ < length;
    }
    if (c == '?')
    {
        uint32_t start = *at;
        while (*at < length && s[*at] >= '0' && s[*at] <= '9')
        {
            ++*at;
        }
        return *at > start;
    }
    bool letter = c >= 'a' && c <= 'z';
    bool same = *at < length && (s[*at] == (uint8_t)c || (letter && s[*at] == (uint8_t)(c - 32)));
    *at += same ? 1u : 0u;
    return same;
}

// ufbx's version-string match for its two Blender patterns; a prefix
// suffices.
static bool Matches(const char* format, const uint8_t* s, uint32_t length)
{
    uint32_t at = 0;
    for (const char* f = format; *f != '\0'; ++f)
    {
        if (!Step(*f, s, length, &at))
        {
            return false;
        }
    }
    return true;
}

// Whether the file is from Blender's binary exporter, by its Creator
// alone as ufbx decides (the ASCII exporter's Creator differs).
static bool FromBlender(const maniFbx* fbx)
{
    uint32_t header = maniFbxFindChild(fbx, 0, "FBXHeaderExtension", 18);
    uint32_t creator =
        header != MANI_FBX_NONE ? maniFbxFindChild(fbx, header, "Creator", 7) : MANI_FBX_NONE;
    const uint8_t* s = NULL;
    uint32_t length = 0;
    if (creator == MANI_FBX_NONE ||
        !maniFbxString(fbx, maniFbxNodeProperty(fbx, creator, 0), &s, &length))
    {
        return false;
    }
    return Matches("blender-- ?.?.?", s, length) || Matches("blender- ?.?", s, length);
}

// The shapes of a channel: counted (out NULL) or listed.
typedef struct Shapes
{
    maniFbxBlendShape* out;
    uint32_t count;
} Shapes;

static void VisitShape(const maniFbx* fbx, uint32_t child, void* context)
{
    Shapes* s = context;
    // ufbx takes a Shape as a blend shape only with its vertices and
    // indices; FullWeights counts those.
    uint32_t node = child < fbx->objectCount ? fbx->objects[child].node : MANI_FBX_NONE;
    if (node != MANI_FBX_NONE && maniFbxObjectKindIs(fbx, child, "Geometry", 8) &&
        maniFbxObjectSubclassIs(fbx, child, "Shape", 5) &&
        maniFbxFindChild(fbx, node, "Vertices", 8) != MANI_FBX_NONE &&
        maniFbxFindChild(fbx, node, "Indexes", 7) != MANI_FBX_NONE)
    {
        if (s->out != NULL)
        {
            s->out[s->count] = (maniFbxBlendShape){child, 0, 1.0};
        }
        ++s->count;
    }
}

// FullWeights' values divided by 100 into the shapes' targets.
static maniImportResult Targets(const maniFbx* fbx, uint32_t channel, maniFbxBlendShape* shapes,
                                uint32_t count)
{
    uint32_t node = maniFbxFindChild(fbx, fbx->objects[channel].node, "FullWeights", 11);
    uint32_t property = node != MANI_FBX_NONE ? maniFbxNodeProperty(fbx, node, 0) : MANI_FBX_NONE;
    uint32_t n = maniFbxArrayCount(fbx, property);
    if (n == 0 || FromBlender(fbx))
    {
        return maniFbxSuccess();
    }
    double* weights = maniAllocate(&fbx->allocator, (size_t)n + 1, sizeof(double), 8);
    if (weights == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniFbxReadDoubles(fbx, property, weights);
    for (uint32_t i = 0; r.result == mani_success && i < count && i < n; ++i)
    {
        shapes[i].target = weights[i] / 100.0;
    }
    maniRelease(&fbx->allocator, weights, (size_t)n + 1, sizeof(double), 8);
    r.item = r.result == mani_errorFormat ? channel : r.item;
    return r;
}

// A finite double's bits ordered as the doubles are.
static uint64_t OrderedBits(double d)
{
    uint64_t bits = 0;
    memcpy(&bits, &d, sizeof bits);
    return (bits >> 63) != 0 ? ~bits : bits + 0x8000000000000000u;
}

static maniImportResult SortShapes(const maniFbx* fbx, maniFbxBlendShape* shapes, uint32_t count)
{
    size_t n = (size_t)count + 1;
    uint64_t* keys = maniAllocate(&fbx->allocator, n, sizeof(uint64_t), 8);
    uint32_t* order = maniAllocate(&fbx->allocator, n, sizeof(uint32_t), 4);
    uint32_t* scratch = maniAllocate(&fbx->allocator, n, sizeof(uint32_t), 4);
    maniFbxBlendShape* sorted = maniAllocate(&fbx->allocator, n, sizeof(maniFbxBlendShape), 8);
    bool fine = keys != NULL && order != NULL && scratch != NULL && sorted != NULL;
    if (fine)
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            keys[i] = OrderedBits(shapes[i].target);
        }
        maniSortByKey(keys, count, order, scratch);
        for (uint32_t i = 0; i < count; ++i)
        {
            sorted[i] = shapes[order[i]];
        }
        memcpy(shapes, sorted, (size_t)count * sizeof *shapes);
    }
    maniRelease(&fbx->allocator, keys, n, sizeof(uint64_t), 8);
    maniRelease(&fbx->allocator, order, n, sizeof(uint32_t), 4);
    maniRelease(&fbx->allocator, scratch, n, sizeof(uint32_t), 4);
    maniRelease(&fbx->allocator, sorted, n, sizeof(maniFbxBlendShape), 8);
    return fine ? maniFbxSuccess() : Capacity();
}

maniImportResult maniFbxReadBlendShapes(const maniFbx* fbx, uint32_t channel,
                                        maniFbxBlendShape* shapes, uint32_t* countOut)
{
    Shapes s = {shapes, 0};
    maniFbxEachChild(fbx, channel, VisitShape, &s);
    *countOut = s.count;
    if (shapes == NULL)
    {
        return maniFbxSuccess();
    }
    maniImportResult r = Targets(fbx, channel, shapes, s.count);
    return r.result == mani_success ? SortShapes(fbx, shapes, s.count) : r;
}

double maniFbxBlendRestPercent(const maniFbx* fbx, uint32_t channel)
{
    double percent = 0.0;
    uint32_t entry = maniFbxFindProperty(fbx, channel, "DeformPercent", 13);
    if (entry != MANI_FBX_NONE)
    {
        return maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, 0), &percent) ? percent : 0.0;
    }
    uint32_t node = maniFbxFindChild(fbx, fbx->objects[channel].node, "DeformPercent", 13);
    if (fbx->objects[channel].properties == MANI_FBX_NONE && node != MANI_FBX_NONE)
    {
        percent = 100.0;
        (void)maniFbxNumber(fbx, maniFbxNodeProperty(fbx, node, 0), &percent);
    }
    return percent;
}

// A target, the rest's 0 for MANI_FBX_NONE.
static double Target(const maniFbxBlendShape* shapes, uint32_t k)
{
    return k != MANI_FBX_NONE ? shapes[k].target : 0.0;
}

void maniFbxBlendWeights(const maniFbxBlendShape* shapes, uint32_t count, double weight,
                         double* effective)
{
    uint32_t lastNegative = MANI_FBX_NONE;
    for (uint32_t i = 0; i < count; ++i)
    {
        effective[i] = 0.0;
        lastNegative = shapes[i].target < 0.0 ? i : lastNegative;
    }
    if (count == 0)
    {
        return;
    }
    // The shapes around the weight, walking away from zero (the rest is
    // MANI_FBX_NONE), as ufbx walks them.
    uint32_t prev = MANI_FBX_NONE;
    uint32_t next = MANI_FBX_NONE;
    uint32_t firstUp = lastNegative == MANI_FBX_NONE ? 0 : lastNegative + 1;
    if (weight > 0.0)
    {
        prev = lastNegative;
        for (uint32_t i = firstUp; i < count; ++i)
        {
            prev = next;
            next = i;
            if (shapes[next].target > weight)
            {
                break;
            }
        }
    }
    else
    {
        prev = firstUp < count ? firstUp : MANI_FBX_NONE;
        for (uint32_t i = firstUp; i-- > 0;)
        {
            prev = next;
            next = i;
            if (shapes[next].target < weight)
            {
                break;
            }
        }
    }
    double delta = Target(shapes, next) - Target(shapes, prev);
    if (delta != 0.0)
    {
        double t = (weight - Target(shapes, prev)) / delta;
        if (prev != MANI_FBX_NONE)
        {
            effective[prev] = 1.0 - t;
        }
        if (next != MANI_FBX_NONE)
        {
            effective[next] = t;
        }
    }
}

bool maniFbxIsShapeOwner(const maniFbx* fbx, uint32_t object)
{
    if (fbx->version >= 7000)
    {
        return fbx->version <= 7100 && maniFbxObjectKindIs(fbx, object, "Geometry", 8) &&
               maniFbxObjectSubclassIs(fbx, object, "Mesh", 4);
    }
    // An object not a model has no mesh model, so makes no tracks.
    return maniFbxModelIsMesh(fbx, object);
}

uint32_t maniFbxShapeOwnerModel(const maniFbx* fbx, uint32_t owner)
{
    return fbx->version < 7000 ? maniFbxModelOf(fbx, owner) : maniFbxDeformerModel(fbx, owner);
}
