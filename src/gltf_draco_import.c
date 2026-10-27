// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// KHR_draco_mesh_compression's primitives: the
// extension's bufferView decoded, each attribute it names one of the
// primitive's, its unique id in the stream, and its accessor the decoded
// attribute's point count, component type and count, not sparse; the
// accessor's normalized reads the values, as for any accessor.

#include "gltf_draco_import.h"

#include "arena.h"
#include "gltf_document.h"
#include "gltf_draco.h"
#include "gltf_layout.h"
#include "gltf_property.h"
#include "json.h"

#include "maul-anim/import.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The longest attribute name compared (glTF's are short: JOINTS_n).
#define MAX_NAME 64u

static maniImportResult Fail(const maniGltf* gltf, maniImportReason reason, uint32_t value,
                             uint32_t item)
{
    return maniImportFail(reason, maniGltfOffset(&gltf->g, value), item);
}

// The glTF component of a Draco data type, or none (Draco's int32 and
// the 64-bit and boolean types have no accessor component).
static int32_t Component(uint8_t dataType)
{
    static const int8_t kComponents[12] = {-1,
                                           maniGltfByte,
                                           maniGltfUnsignedByte,
                                           maniGltfShort,
                                           maniGltfUnsignedShort,
                                           -1,
                                           maniGltfUnsignedInt,
                                           -1,
                                           -1,
                                           maniGltfFloat,
                                           -1,
                                           -1};
    return dataType < 12 ? kComponents[dataType] : -1;
}

static const maniDracoAttribute* ByUniqueId(const maniDracoMesh* m, int64_t id)
{
    for (uint32_t i = 0; i < m->attributeCount; ++i)
    {
        if (m->attributes[i].uniqueId == id)
        {
            return &m->attributes[i];
        }
    }
    return NULL;
}

// One name the extension gives: the primitive's attribute of that name,
// whose accessor matches the decoded attribute of that unique id.
static maniImportResult Match(const maniGltf* gltf, const maniGltfDraco* d, uint32_t attributes,
                              uint32_t key)
{
    uint32_t id = gltf->json.values[key].next;
    char name[MAX_NAME + 1];
    size_t length = maniJsonStringDecode(&gltf->json, key, (uint8_t*)name, MAX_NAME);
    name[length] = '\0';
    uint32_t accessor = MANI_JSON_NONE;
    int64_t unique = -1;
    if (gltf->json.values[key].length > MAX_NAME ||
        maniJsonFind(&gltf->json, attributes, name, length) == MANI_JSON_NONE ||
        !maniJsonInteger(&gltf->json, id, 0, UINT32_MAX, &unique))
    {
        return Fail(gltf, maniImportReasonProperty, id, 0);
    }
    maniImportResult r =
        maniGltfIndex(&gltf->g, attributes, name, true, gltf->accessorCount, 0, &accessor);
    const maniDracoAttribute* t = ByUniqueId(&d->mesh, unique);
    if (r.result != mani_success || t == NULL)
    {
        return r.result != mani_success ? r : Fail(gltf, maniImportReasonProperty, id, 0);
    }
    const maniGltfAccessor* a = &gltf->accessors[accessor];
    if (a->sparseCount != 0 || a->columns != 1 || a->rows != t->components ||
        a->count != d->mesh.pointCount || (int32_t)a->component != Component(t->dataType))
    {
        return Fail(gltf, maniImportReasonProperty, a->value, accessor);
    }
    return r;
}

// The stream's bytes, its view's (Pending for an external buffer not
// given).
static maniImportResult Stream(const maniGltf* gltf, uint32_t view, uint32_t value,
                               const uint8_t** bytes)
{
    const maniGltfView* v = &gltf->views[view];
    const maniGltfBuffer* b = &gltf->buffers[v->buffer];
    const uint8_t* data = v->source != MANI_JSON_NONE ? v->decoded : b->data;
    if (data == NULL)
    {
        return Fail(gltf, maniImportReasonPending, value, view);
    }
    *bytes = v->source != MANI_JSON_NONE ? data : data + v->offset;
    return maniImportSuccess();
}

maniImportResult maniGltfDracoDecode(const maniGltf* gltf, maniArena* arena, uint32_t primitive,
                                     uint32_t attributes, uint32_t vertices, maniGltfDraco* out)
{
    *out = (maniGltfDraco){0};
    uint32_t extensions = maniJsonFind(&gltf->json, primitive, "extensions", 10);
    uint32_t ext = maniJsonFind(&gltf->json, extensions, "KHR_draco_mesh_compression", 26);
    if (ext == MANI_JSON_NONE)
    {
        return maniImportSuccess();
    }
    out->present = true;
    maniImportResult r =
        maniGltfIndex(&gltf->g, ext, "bufferView", true, gltf->viewCount, 0, &out->view);
    if (r.result == mani_success)
    {
        r = maniGltfMember(&gltf->g, ext, "attributes", maniJsonObject, true, 0, &out->attributes);
    }
    uint32_t indices = MANI_JSON_NONE;
    if (r.result == mani_success)
    {
        r = maniGltfIndex(&gltf->g, primitive, "indices", false, gltf->accessorCount, 0, &indices);
    }
    const uint8_t* bytes = NULL;
    if (r.result == mani_success)
    {
        r = Stream(gltf, out->view, ext, &bytes);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    // The faces the primitive draws: at most its indices (a list's a third,
    // a strip's two fewer), or without them its vertices.
    uint32_t faces = indices != MANI_JSON_NONE ? gltf->accessors[indices].count : vertices;
    const maniGltfView* v = &gltf->views[out->view];
    maniDracoStatus s =
        maniDracoDecode(arena, bytes, (size_t)v->length, vertices, faces, &out->mesh);
    if (s != maniDracoDone)
    {
        return s == maniDracoCapacity
                   ? (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0}
                   : maniImportFail(maniImportReasonData, v->offset, out->view);
    }
    uint32_t key = out->attributes + 1;
    for (uint32_t i = 0; r.result == mani_success && i < gltf->json.values[out->attributes].count;
         ++i)
    {
        r = Match(gltf, out, attributes, key);
        key = gltf->json.values[gltf->json.values[key].next].next;
    }
    return r;
}

maniImportResult maniGltfDracoFloats(const maniGltf* gltf, const maniGltfDraco* draco,
                                     const char* name, size_t length, uint32_t accessor, float* out)
{
    uint32_t id = draco->present ? maniJsonFind(&gltf->json, draco->attributes, name, length)
                                 : MANI_JSON_NONE;
    int64_t unique = -1;
    if (id == MANI_JSON_NONE || !maniJsonInteger(&gltf->json, id, 0, UINT32_MAX, &unique))
    {
        return maniGltfReadFloats(gltf, accessor, out);
    }
    const maniDracoAttribute* t = ByUniqueId(&draco->mesh, unique);
    return maniGltfReadMapped(gltf, accessor, t->values, t->pointToValue, t->valueCount,
                              gltf->views[draco->view].offset, out);
}
