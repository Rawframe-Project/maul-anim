// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF's meshopt buffer compression: both extensions' JSON
// on buffer views and buffers checked by their specifications' rules,
// fallback buffers, and each view's stream decoded by gltf_meshopt.c
// into bytes the document owns.

#include "gltf_compression.h"

#include "allocator.h"
#include "gltf_document.h"
#include "gltf_meshopt.h"
#include "gltf_property.h"
#include "json.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const char* const kExtensions[2] = {"EXT_meshopt_compression", "KHR_meshopt_compression"};
static const char* const kModes[3] = {"ATTRIBUTES", "TRIANGLES", "INDICES"};
// COLOR is KHR's alone.
static const char* const kFilters[5] = {"NONE", "OCTAHEDRAL", "QUATERNION", "EXPONENTIAL", "COLOR"};

// The most bytes a valid stream decodes into a byte of its own: version
// 1 attributes whose every byte position is all zero cost a block of up
// to 256 elements a byte of control header per 4 bytes of stride.
#define MAX_EXPANSION 1024u

static maniImportResult Capacity(void)
{
    return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
}

// The object of the extension the value carries, MANI_JSON_NONE for
// neither, *khrOut true for KHR's; both at once refused (Property).
static maniImportResult FindExtension(const maniGltfJson* g, uint32_t value, uint32_t item,
                                      uint32_t* objectOut, bool* khrOut)
{
    uint32_t extensions = MANI_JSON_NONE;
    uint32_t objects[2] = {MANI_JSON_NONE, MANI_JSON_NONE};
    maniImportResult r =
        maniGltfMember(g, value, "extensions", maniJsonObject, false, item, &extensions);
    for (uint32_t k = 0; r.result == mani_success && k < 2; ++k)
    {
        r = maniGltfMember(g, extensions, kExtensions[k], maniJsonObject, false, item, &objects[k]);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (objects[0] != MANI_JSON_NONE && objects[1] != MANI_JSON_NONE)
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, objects[1]), item);
    }
    *khrOut = objects[1] != MANI_JSON_NONE;
    *objectOut = *khrOut ? objects[1] : objects[0];
    return r;
}

// A string member's place among the first count names; unchanged when
// absent and not required; any other string refused (Property).
static maniImportResult ReadName(const maniGltfJson* g, uint32_t object, const char* name,
                                 const char* const* names, uint32_t count, bool required,
                                 uint32_t item, uint8_t* out)
{
    uint32_t value = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, object, name, maniJsonString, required, item, &value);
    if (r.result != mani_success || value == MANI_JSON_NONE)
    {
        return r;
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        if (maniJsonStringEquals(g->json, value, names[k], strlen(names[k])))
        {
            *out = (uint8_t)k;
            return r;
        }
    }
    return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, value), item);
}

maniImportResult maniGltfReadFallback(maniGltf* gltf, uint32_t value, uint32_t buffer)
{
    uint32_t object = MANI_JSON_NONE;
    bool khr = false;
    bool fallback = false;
    maniImportResult r = FindExtension(&gltf->g, value, buffer, &object, &khr);
    if (r.result == mani_success)
    {
        r = maniGltfBool(&gltf->g, object, "fallback", buffer, &fallback);
    }
    gltf->buffers[buffer].fallback = fallback ? (khr ? 2 : 1) : 0;
    return r;
}

static bool FilterFits(const maniGltfView* v)
{
    uint32_t s = v->elementStride;
    switch ((maniMeshoptFilter)v->filter)
    {
    case maniMeshoptNone:
        return true;
    case maniMeshoptOctahedral:
    case maniMeshoptColor:
        return v->mode == maniMeshoptAttributes && (s == 4 || s == 8);
    case maniMeshoptQuaternion:
        return v->mode == maniMeshoptAttributes && s == 8;
    case maniMeshoptExponential:
        return v->mode == maniMeshoptAttributes;
    }
    return false;
}

// The specification's rules on the stream read into v, each refused
// (Layout) at the member that breaks it; a view longer than its stream
// could decode, or a stream in a fallback buffer, too.
static maniImportResult CheckStream(const maniGltf* gltf, uint32_t object, uint32_t i)
{
    const maniGltfView* v = &gltf->views[i];
    uint32_t s = v->elementStride;
    const char* member = NULL;
    if (v->mode == maniMeshoptAttributes ? s % 4 != 0 : (s != 2 && s != 4))
    {
        member = "byteStride";
    }
    else if (v->mode == maniMeshoptTriangles && v->count % 3 != 0)
    {
        member = "count";
    }
    else if (!FilterFits(v))
    {
        member = "filter";
    }
    else if ((uint64_t)v->count * s != v->length)
    {
        member = "count";
    }
    else if (v->sourceOffset + v->sourceLength > gltf->buffers[v->source].byteLength ||
             (v->length - 1) / MAX_EXPANSION >= v->sourceLength)
    {
        member = "byteLength";
    }
    else if (gltf->buffers[v->source].fallback != 0)
    {
        member = "buffer";
    }
    if (member == NULL)
    {
        return maniImportSuccess();
    }
    uint32_t at = maniJsonFind(gltf->g.json, object, member, strlen(member));
    return maniImportFail(maniImportReasonLayout, maniGltfOffset(&gltf->g, at), i);
}

maniImportResult maniGltfReadCompression(maniGltf* gltf, uint32_t value, uint32_t i)
{
    const maniGltfJson* g = &gltf->g;
    maniGltfView* v = &gltf->views[i];
    uint32_t object = MANI_JSON_NONE;
    bool khr = false;
    v->source = MANI_JSON_NONE;
    maniImportResult r = FindExtension(g, value, i, &object, &khr);
    if (r.result != mani_success)
    {
        return r;
    }
    // Only views compressed by the buffer's own extension use a fallback.
    uint8_t fallback = gltf->buffers[v->buffer].fallback;
    if (fallback != 0 && (object == MANI_JSON_NONE || khr != (fallback == 2)))
    {
        uint32_t at = maniJsonFind(g->json, value, "buffer", 6);
        return maniImportFail(maniImportReasonLayout, maniGltfOffset(g, at), i);
    }
    if (object == MANI_JSON_NONE)
    {
        return r;
    }
    uint32_t source = MANI_JSON_NONE;
    int64_t offset = 0;
    int64_t length = 0;
    int64_t stride = 0;
    int64_t count = 0;
    uint8_t mode = 0;
    uint8_t filter = maniMeshoptNone;
    r = maniGltfIndex(g, object, "buffer", true, gltf->bufferCount, i, &source);
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, object, "byteOffset", false, 0, INT64_MAX / 4, i, &offset);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, object, "byteLength", true, 1, INT64_MAX / 4, i, &length);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, object, "byteStride", true, 1, 256, i, &stride);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, object, "count", true, 1, UINT32_MAX, i, &count);
    }
    if (r.result == mani_success)
    {
        r = ReadName(g, object, "mode", kModes, 3, true, i, &mode);
    }
    if (r.result == mani_success)
    {
        r = ReadName(g, object, "filter", kFilters, khr ? 5 : 4, false, i, &filter);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    v->source = source;
    v->count = (uint32_t)count;
    v->sourceOffset = (uint64_t)offset;
    v->sourceLength = (uint64_t)length;
    v->elementStride = (uint16_t)stride;
    v->mode = mode;
    v->filter = filter;
    v->khr = khr;
    return CheckStream(gltf, object, i);
}

static void Release(maniGltf* gltf, maniGltfView* v)
{
    maniRelease(&gltf->allocator, v->decoded, v->decoded != NULL ? (size_t)v->length : 0, 1, 8);
    v->decoded = NULL;
}

static maniImportResult DecodeView(maniGltf* gltf, uint32_t i)
{
    maniGltfView* v = &gltf->views[i];
    const uint8_t* in = gltf->buffers[v->source].data + v->sourceOffset;
    Release(gltf, v);
    if (v->length > SIZE_MAX)
    {
        return Capacity();
    }
    uint8_t* out = maniAllocate(&gltf->allocator, (size_t)v->length, 1, 8);
    if (out == NULL)
    {
        return Capacity();
    }
    if (!maniMeshoptDecode(out, v->count, v->elementStride, (maniMeshoptMode)v->mode, v->khr, in,
                           (size_t)v->sourceLength))
    {
        maniRelease(&gltf->allocator, out, (size_t)v->length, 1, 8);
        return maniImportFail(maniImportReasonData, v->sourceOffset, i);
    }
    maniMeshoptApplyFilter(out, v->count, v->elementStride, (maniMeshoptFilter)v->filter);
    v->decoded = out;
    return maniImportSuccess();
}

// Each buffer's uses: by a view without a stream (1), by a compressed
// view (2), as a stream's source (4).
static maniImportResult MarkPlaceholders(maniGltf* gltf)
{
    if (gltf->bufferCount == 0)
    {
        return maniImportSuccess();
    }
    uint8_t* uses = maniAllocate(&gltf->allocator, gltf->bufferCount, 1, 8);
    if (uses == NULL)
    {
        return Capacity();
    }
    memset(uses, 0, gltf->bufferCount);
    for (uint32_t i = 0; i < gltf->viewCount; ++i)
    {
        const maniGltfView* v = &gltf->views[i];
        uses[v->buffer] |= v->source == MANI_JSON_NONE ? 1u : 2u;
        if (v->source != MANI_JSON_NONE)
        {
            uses[v->source] |= 4u;
        }
    }
    for (uint32_t b = 0; b < gltf->bufferCount; ++b)
    {
        maniGltfBuffer* buffer = &gltf->buffers[b];
        if (buffer->external && buffer->uri == NULL && uses[b] == 2u)
        {
            buffer->external = false;
        }
    }
    maniRelease(&gltf->allocator, uses, gltf->bufferCount, 1, 8);
    return maniImportSuccess();
}

maniImportResult maniGltfFinishCompression(maniGltf* gltf)
{
    maniImportResult r = MarkPlaceholders(gltf);
    for (uint32_t i = 0; r.result == mani_success && i < gltf->viewCount; ++i)
    {
        const maniGltfView* v = &gltf->views[i];
        if (v->source != MANI_JSON_NONE && gltf->buffers[v->source].data != NULL)
        {
            r = DecodeView(gltf, i);
        }
    }
    return r;
}

maniImportResult maniGltfDecodeBuffer(maniGltf* gltf, uint32_t buffer)
{
    maniImportResult r = maniImportSuccess();
    for (uint32_t i = 0; r.result == mani_success && i < gltf->viewCount; ++i)
    {
        if (gltf->views[i].source == buffer)
        {
            r = DecodeView(gltf, i);
        }
    }
    for (uint32_t i = 0; r.result != mani_success && i < gltf->viewCount; ++i)
    {
        if (gltf->views[i].source == buffer)
        {
            Release(gltf, &gltf->views[i]);
        }
    }
    return r;
}

void maniGltfReleaseDecoded(maniGltf* gltf)
{
    for (uint32_t i = 0; i < gltf->viewCount; ++i)
    {
        Release(gltf, &gltf->views[i]);
    }
}
