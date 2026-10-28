// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// An FBX clip's float tracks: the other numbers a
// bone's model animates, named bone/property (with /component for
// vectors), and blend shape weights named mesh/shape, from BlendShape
// deformers or, before 7.2, from meshes' Shape children.

#include "fbx_tracks.h"

#include "arena.h"
#include "fbx_anim.h"
#include "fbx_blend.h"
#include "fbx_clip_state.h"
#include "fbx_document.h"
#include "fbx_model.h"
#include "fbx_object.h"
#include "fbx_skin.h"
#include "fbx_transform.h"
#include "fbx_value.h"

#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <stdckdint.h>
#include <stddef.h>
#include <string.h>

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

// Whether a property is one of the local transform's inputs.
static bool InChain(const maniFbx* fbx, const maniFbxChannel* channel)
{
    for (uint32_t k = 0; k < maniFbxLocalVectors; ++k)
    {
        const char* n = maniFbxLocalVectorNames[k];
        if (maniFbxChannelNameIs(fbx, channel, n, strlen(n)))
        {
            return true;
        }
    }
    for (uint32_t k = 0; k < maniFbxLocalIntegers; ++k)
    {
        const char* n = maniFbxLocalIntegerNames[k];
        if (maniFbxChannelNameIs(fbx, channel, n, strlen(n)))
        {
            return true;
        }
    }
    return false;
}

// A property's static value and its component count: the entry's
// numbers, or 0 and the channel's keyed components when the model has
// no entry.
static uint32_t Static(const maniFbx* fbx, uint32_t object, const char* name, size_t length,
                       const maniFbxChannel* channel, double value[3])
{
    value[0] = value[1] = value[2] = 0.0;
    uint32_t entry = maniFbxFindProperty(fbx, object, name, length);
    if (entry == MANI_FBX_NONE)
    {
        return channel->curves[1].count > 0 || channel->curves[2].count > 0 ? 3 : 1;
    }
    uint32_t n = 0;
    while (n < 3 && maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, n), &value[n]))
    {
        ++n;
    }
    return n > 0 ? n : 1;
}

// The float track's name: the bone's, '/', the property's, and '/'
// with the component for vectors.
static char* TrackName(maniFbxClipState* c, uint32_t b, const char* property, size_t length,
                       uint32_t width, uint32_t k, uint32_t* lengthOut)
{
    size_t boneLength = 0;
    const char* bone = maniSkeletonBoneName(c->skeleton, b, &boneLength);
    size_t n = (size_t)boneLength + 1 + length + (width > 1 ? 2 : 0);
    char* name = maniArenaTake(&c->arena, n, 1);
    if (name != NULL)
    {
        memcpy(name, bone, boneLength);
        name[boneLength] = '/';
        memcpy(name + boneLength + 1, property, length);
        if (width > 1)
        {
            name[n - 2] = '/';
            name[n - 1] = (char)('0' + k);
        }
    }
    *lengthOut = (uint32_t)n;
    return name;
}

// The float tracks of one property of a bone's model.
static maniImportResult PropertyTracks(maniFbxClipState* c, uint32_t b,
                                       const maniFbxChannel* channel)
{
    const maniFbx* fbx = c->fbx;
    uint32_t object = fbx->models[c->modelOf[b]];
    size_t needed = maniFbxChannelName(fbx, channel, NULL, 0);
    char* property = maniArenaTake(&c->arena, needed + 1, 1);
    if (property == NULL)
    {
        return Capacity();
    }
    size_t length = maniFbxChannelName(fbx, channel, (uint8_t*)property, needed);
    double rest[3];
    uint32_t width = Static(fbx, object, property, length, channel, rest);
    float* values = maniArenaTake(&c->arena, (size_t)c->sampleCount * width, sizeof(float));
    // The property's channels found once, not at every sample.
    uint32_t first = 0;
    uint32_t run = maniFbxAnimatedRun(&c->animation, object, &first);
    uint32_t* matched = maniArenaTake(&c->arena, (size_t)run + 1, sizeof(uint32_t));
    if (values == NULL || matched == NULL)
    {
        return Capacity();
    }
    uint32_t n = maniFbxMatchChannels(&c->animation, fbx, first, run, property, length, matched);
    for (uint32_t s = 0; s < c->sampleCount; ++s)
    {
        double v[3] = {rest[0], rest[1], rest[2]};
        (void)maniFbxAnimateChannels(&c->animation, fbx, matched, n, maniFbxClipStackTime(c, s), v);
        for (uint32_t k = 0; k < width; ++k)
        {
            values[(size_t)k * c->sampleCount + s] = (float)v[k];
        }
    }
    for (uint32_t k = 0; k < width; ++k)
    {
        maniFloatTrackSource* f = &c->floats[c->floatCount];
        f->name = TrackName(c, b, property, length, width, k, &f->nameLength);
        if (f->name == NULL)
        {
            return Capacity();
        }
        f->keys = (maniChannelSource){c->times, values + (size_t)k * c->sampleCount, c->sampleCount,
                                      maniInterpolationLinear};
        c->floatModel[c->floatCount++] = c->modelOf[b];
    }
    return maniFbxSuccess();
}

// Whether an earlier channel of the run names the same property.
static bool Seen(const maniFbxClipState* c, uint32_t first, uint32_t i)
{
    const maniFbxChannel* channels = c->animation.channels;
    for (uint32_t j = first; j < i; ++j)
    {
        if (channels[j].nameHash == channels[i].nameHash &&
            maniFbxSameChannelName(c->fbx, &channels[j], &channels[i]))
        {
            return true;
        }
    }
    return false;
}

// An object's name without its class, in the arena.
static char* ObjectName(maniFbxClipState* c, uint32_t object, size_t* lengthOut)
{
    const maniFbx* fbx = c->fbx;
    size_t length = maniFbxObjectName(fbx, object, NULL, 0);
    char* name = maniArenaTake(&c->arena, length + 1, 1);
    *lengthOut = name != NULL ? maniFbxObjectName(fbx, object, (uint8_t*)name, length) : 0;
    return name;
}

// '/' and k in decimal into out (12 bytes); the count written.
static size_t Decimal(uint32_t k, char* out)
{
    char digits[10];
    size_t n = 0;
    do
    {
        digits[n++] = (char)('0' + k % 10);
        k /= 10;
    } while (k > 0);
    out[0] = '/';
    for (size_t i = 0; i < n; ++i)
    {
        out[1 + i] = digits[n - 1 - i];
    }
    return n + 1;
}

// A shape's track name: the mesh's, '/', the shape's; or for a shape
// without a name, its channel's, with '/' and its index past the first.
static char* ShapeName(maniFbxClipState* c, uint32_t mesh, uint32_t channel, uint32_t shape,
                       uint32_t k, uint32_t* lengthOut)
{
    uint32_t meshLength = 0;
    const char* meshName = maniFbxModelName(c->fbx, mesh, &meshLength);
    size_t length = 0;
    const char* own = ObjectName(c, shape, &length);
    bool named = own != NULL && length > 0;
    const char* base = named ? own : ObjectName(c, channel, &length);
    char index[12];
    size_t digits = named || k == 0 ? 0 : Decimal(k, index);
    size_t n = (size_t)meshLength + 1 + length + digits;
    char* name = base != NULL ? maniArenaTake(&c->arena, n, 1) : NULL;
    if (name != NULL)
    {
        memcpy(name, meshName, meshLength);
        name[meshLength] = '/';
        memcpy(name + meshLength + 1, base, length);
        memcpy(name + meshLength + 1 + length, index, digits);
    }
    *lengthOut = (uint32_t)n;
    return name;
}

// Whether a mesh model belongs to the skeleton: a bone of it, or carrying
// a skin within it.
static bool MeshInSkeleton(const maniFbxClipState* c, uint32_t model)
{
    uint32_t length = 0;
    const char* name = maniFbxModelName(c->fbx, model, &length);
    if (maniSkeletonFindBone(c->skeleton, name, length) != MANI_NO_BONE)
    {
        return true;
    }
    for (uint32_t s = 0; s < c->fbx->skinCount; ++s)
    {
        if (maniFbxSkinModel(c->fbx, s) == model && maniFbxSkinWithin(c->fbx, s, c->skeleton))
        {
            return true;
        }
    }
    return false;
}

// Each shape's effective weight over the samples.
static maniImportResult ShapeWeights(maniFbxClipState* c, uint32_t channel,
                                     const maniFbxBlendShape* shapes, uint32_t count, float* values)
{
    double* effective = maniArenaTake(&c->arena, count, sizeof(double));
    if (effective == NULL)
    {
        return Capacity();
    }
    double rest = maniFbxBlendRestPercent(c->fbx, channel);
    for (uint32_t s = 0; s < c->sampleCount; ++s)
    {
        double v[3] = {rest, 0.0, 0.0};
        (void)maniFbxAnimateProperty(&c->animation, c->fbx, channel, "DeformPercent", 13,
                                     maniFbxClipStackTime(c, s), 0, v);
        maniFbxBlendWeights(shapes, count, v[0] / 100.0, effective);
        for (uint32_t k = 0; k < count; ++k)
        {
            values[(size_t)k * c->sampleCount + s] = (float)effective[k];
        }
    }
    return maniFbxSuccess();
}

// The tracks of a blend shape channel whose mesh is the skeleton's.
static maniImportResult ChannelTracks(maniFbxClipState* c, uint32_t channel)
{
    uint32_t mesh = maniFbxBlendChannelModel(c->fbx, channel);
    if (mesh == MANI_FBX_NONE || !MeshInSkeleton(c, mesh))
    {
        return maniFbxSuccess();
    }
    uint32_t first = 0;
    c->blendBound += maniFbxAnimatedRun(&c->animation, channel, &first);
    uint32_t count = 0;
    (void)maniFbxReadBlendShapes(c->fbx, channel, NULL, &count);
    maniFbxBlendShape* shapes = maniArenaTake(&c->arena, count, sizeof(maniFbxBlendShape));
    size_t floats = 0;
    float* values = !ckd_mul(&floats, (size_t)count, (size_t)c->sampleCount)
                        ? maniArenaTake(&c->arena, floats, sizeof(float))
                        : NULL;
    if (shapes == NULL || values == NULL)
    {
        return Capacity();
    }
    maniImportResult r = maniFbxReadBlendShapes(c->fbx, channel, shapes, &count);
    r = r.result == mani_success ? ShapeWeights(c, channel, shapes, count, values) : r;
    for (uint32_t k = 0; r.result == mani_success && k < count; ++k)
    {
        maniFloatTrackSource* f = &c->floats[c->floatCount];
        f->name = ShapeName(c, mesh, channel, shapes[k].object, k, &f->nameLength);
        f->keys = (maniChannelSource){c->times, values + (size_t)k * c->sampleCount, c->sampleCount,
                                      maniInterpolationLinear};
        c->floatModel[c->floatCount++] = mesh;
        r = f->name != NULL ? r : Capacity();
    }
    return r;
}

// Whether a run's channels include its target's DeformPercent: the first
// of the run's blend shape channel.
static bool DrivesWeight(const maniFbxClipState* c, uint32_t first, uint32_t count)
{
    for (uint32_t i = first; i < first + count; ++i)
    {
        if (maniFbxChannelNameIs(c->fbx, &c->animation.channels[i], "DeformPercent", 13))
        {
            return true;
        }
    }
    return false;
}

// The run's channel on an owner's property of a name: the attribute's
// for a 6.x model, the object's own for a 7.x geometry; NULL for none.
static const maniFbxChannel* OwnerChannel(const maniFbxClipState* c, uint32_t first, uint32_t count,
                                          const char* name, size_t length)
{
    bool legacy = c->fbx->version < 7000;
    for (uint32_t i = first; i < first + count; ++i)
    {
        const maniFbxChannel* channel = &c->animation.channels[i];
        if (channel->attribute == legacy && maniFbxChannelNameIs(c->fbx, channel, name, length))
        {
            return channel;
        }
    }
    return NULL;
}

// A Shape child's track: its weight (the owner's property of its name /
// 100) at every sample, when that property is a number (before 6.0, any
// name) the stack drives.
static maniImportResult OwnerShape(maniFbxClipState* c, uint32_t owner, uint32_t mesh,
                                   uint32_t shape, uint32_t first, uint32_t count)
{
    const maniFbx* fbx = c->fbx;
    uint32_t property = maniFbxNodeProperty(fbx, shape, 0);
    const uint8_t* raw = NULL;
    uint32_t rawLength = 0;
    if (!maniFbxString(fbx, property, &raw, &rawLength))
    {
        return maniFbxFail(maniImportReasonProperty, fbx->nodes[shape].nameOffset, owner);
    }
    size_t decoded = maniFbxStringDecode(fbx, property, NULL, 0);
    char* name = maniArenaTake(&c->arena, decoded + 1, 1);
    if (name == NULL)
    {
        return Capacity();
    }
    size_t length = maniFbxStringDecode(fbx, property, (uint8_t*)name, decoded);
    uint32_t entry = maniFbxFindProperty(fbx, owner, name, length);
    // Before 6.0 the weight is connected with no property.
    bool weighted =
        fbx->version < 6000 || (entry != MANI_FBX_NONE && maniFbxEntryNumeric(fbx, entry));
    if (maniFbxFindChild(fbx, shape, "Indexes", 7) == MANI_FBX_NONE ||
        maniFbxFindChild(fbx, shape, "Vertices", 8) == MANI_FBX_NONE || !weighted ||
        OwnerChannel(c, first, count, name, length) == NULL)
    {
        return maniFbxSuccess();
    }
    double rest = 0.0;
    (void)maniFbxNumber(fbx, maniFbxPropertyValue(fbx, entry, 0), &rest);
    float* values = maniArenaTake(&c->arena, c->sampleCount, sizeof(float));
    uint32_t meshLength = 0;
    const char* meshName = maniFbxModelName(fbx, mesh, &meshLength);
    size_t n = (size_t)meshLength + 1 + length;
    char* track = maniArenaTake(&c->arena, n, 1);
    if (values == NULL || track == NULL)
    {
        return Capacity();
    }
    // One shape at target 1: its weight is the channel's.
    static const maniFbxBlendShape kWhole = {0, 0, 1.0};
    for (uint32_t s = 0; s < c->sampleCount; ++s)
    {
        double v[3] = {rest, 0.0, 0.0};
        double t = maniFbxClipStackTime(c, s);
        (void)(fbx->version < 7000
                   ? maniFbxAnimateAttribute(&c->animation, fbx, owner, name, length, t, v)
                   : maniFbxAnimateProperty(&c->animation, fbx, owner, name, length, t, 0, v));
        double effective = 0.0;
        maniFbxBlendWeights(&kWhole, 1, v[0] / 100.0, &effective);
        values[s] = (float)effective;
    }
    memcpy(track, meshName, meshLength);
    track[meshLength] = '/';
    memcpy(track + meshLength + 1, name, length);
    maniFloatTrackSource* f = &c->floats[c->floatCount];
    f->name = track;
    f->nameLength = (uint32_t)n;
    f->keys = (maniChannelSource){c->times, values, c->sampleCount, maniInterpolationLinear};
    c->floatModel[c->floatCount++] = mesh;
    c->blendBound += 1;
    return maniFbxSuccess();
}

// The Shape children of an owner the stack drives (before 7.2): counted
// into shapesOut, and their tracks made unless counting, for a mesh in
// the skeleton.
static maniImportResult OwnerTracks(maniFbxClipState* c, uint32_t owner, uint32_t first,
                                    uint32_t count, bool counting, size_t* shapesOut)
{
    const maniFbx* fbx = c->fbx;
    uint32_t mesh = maniFbxShapeOwnerModel(fbx, owner);
    bool kept = mesh != MANI_FBX_NONE && MeshInSkeleton(c, mesh);
    maniImportResult r = maniFbxSuccess();
    uint32_t node = fbx->objects[owner].node;
    for (uint32_t k = fbx->nodes[node].firstChild; r.result == mani_success && k != MANI_FBX_NONE;
         k = fbx->nodes[k].next)
    {
        if (!maniFbxNameIs(fbx, k, "Shape", 5))
        {
            continue;
        }
        *shapesOut += 1;
        r = counting || !kept ? r : OwnerShape(c, owner, mesh, k, first, count);
    }
    return r;
}

maniImportResult maniFbxBlendTracks(maniFbxClipState* c, bool count, size_t* shapesOut)
{
    maniImportResult r = maniFbxSuccess();
    const maniFbxChannel* channels = c->animation.channels;
    for (uint32_t i = 0; r.result == mani_success && i < c->animation.channelCount;)
    {
        uint32_t first = 0;
        uint32_t target = channels[i].target;
        uint32_t n = maniFbxAnimatedRun(&c->animation, target, &first);
        if (maniFbxIsBlendChannel(c->fbx, target) && DrivesWeight(c, first, n))
        {
            uint32_t shapes = 0;
            (void)maniFbxReadBlendShapes(c->fbx, target, NULL, &shapes);
            *shapesOut += shapes;
            r = count ? r : ChannelTracks(c, target);
        }
        else if (maniFbxIsShapeOwner(c->fbx, target))
        {
            r = OwnerTracks(c, target, first, n, count, shapesOut);
        }
        i = first + n;
    }
    return r;
}

maniImportResult maniFbxBoneFloatTracks(maniFbxClipState* c, uint32_t b)
{
    maniImportResult r = maniFbxSuccess();
    uint32_t first = 0;
    uint32_t n = maniFbxAnimatedRun(&c->animation, c->fbx->models[c->modelOf[b]], &first);
    for (uint32_t i = first; r.result == mani_success && i < first + n; ++i)
    {
        const maniFbxChannel* channel = &c->animation.channels[i];
        // A 6.x attribute's properties are no bone's.
        if (!channel->attribute && !InChain(c->fbx, channel) && !Seen(c, first, i))
        {
            r = PropertyTracks(c, b, channel);
        }
    }
    return r;
}
