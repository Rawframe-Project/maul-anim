// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Typed reads of a glTF document's properties (mani-0008).

#include "gltf_property.h"

#include "json.h"

#include <string.h>

maniImportResult maniImportFail(maniImportReason reason, uint64_t offset, uint32_t item)
{
    return (maniImportResult){mani_errorFormat, reason, offset, item, 0};
}

maniImportResult maniImportSuccess(void)
{
    return (maniImportResult){mani_success, maniImportReasonNone, 0, 0, 0};
}

uint64_t maniGltfOffset(const maniGltfJson* g, uint32_t value)
{
    return g->base + g->json->values[value].start;
}

maniImportResult maniGltfMember(const maniGltfJson* g, uint32_t object, const char* name,
                                maniJsonKind kind, bool required, uint32_t item, uint32_t* valueOut)
{
    uint32_t value = maniJsonFind(g->json, object, name, strlen(name));
    *valueOut = value;
    if (value == MANI_JSON_NONE)
    {
        return required ? maniImportFail(maniImportReasonProperty, maniGltfOffset(g, object), item)
                        : maniImportSuccess();
    }
    if (g->json->values[value].kind != (uint32_t)kind)
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, value), item);
    }
    return maniImportSuccess();
}

maniImportResult maniGltfInteger(const maniGltfJson* g, uint32_t object, const char* name,
                                 bool required, int64_t minimum, int64_t maximum, uint32_t item,
                                 int64_t* valueOut)
{
    uint32_t value = MANI_JSON_NONE;
    maniImportResult r = maniGltfMember(g, object, name, maniJsonNumber, required, item, &value);
    if (r.result != mani_success || value == MANI_JSON_NONE)
    {
        return r;
    }
    if (!maniJsonInteger(g->json, value, minimum, maximum, valueOut))
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, value), item);
    }
    return maniImportSuccess();
}

maniImportResult maniGltfIndex(const maniGltfJson* g, uint32_t object, const char* name,
                               bool required, uint32_t count, uint32_t item, uint32_t* indexOut)
{
    *indexOut = MANI_JSON_NONE;
    int64_t index = -1;
    maniImportResult r =
        maniGltfInteger(g, object, name, required, 0, UINT32_MAX - 1, item, &index);
    if (r.result != mani_success || index < 0)
    {
        return r;
    }
    if (index >= (int64_t)count)
    {
        uint32_t value = maniJsonFind(g->json, object, name, strlen(name));
        return maniImportFail(maniImportReasonIndex, maniGltfOffset(g, value), item);
    }
    *indexOut = (uint32_t)index;
    return maniImportSuccess();
}

maniImportResult maniGltfBool(const maniGltfJson* g, uint32_t object, const char* name,
                              uint32_t item, bool* valueOut)
{
    uint32_t value = maniJsonFind(g->json, object, name, strlen(name));
    if (value == MANI_JSON_NONE)
    {
        return maniImportSuccess();
    }
    uint32_t kind = g->json->values[value].kind;
    if (kind != maniJsonTrue && kind != maniJsonFalse)
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, value), item);
    }
    *valueOut = kind == maniJsonTrue;
    return maniImportSuccess();
}

maniImportResult maniGltfArray(const maniGltfJson* g, const char* name, maniJsonKind kind,
                               uint32_t* arrayOut, uint32_t* countOut)
{
    *countOut = 0;
    maniImportResult r = maniGltfMember(g, 0, name, maniJsonArray, false, 0, arrayOut);
    if (r.result != mani_success || *arrayOut == MANI_JSON_NONE)
    {
        return r;
    }
    const maniJsonValue* values = g->json->values;
    uint32_t element = *arrayOut + 1;
    for (uint32_t i = 0; i < values[*arrayOut].count; ++i)
    {
        if (values[element].kind != (uint32_t)kind)
        {
            return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, element), i);
        }
        element = values[element].next;
    }
    *countOut = values[*arrayOut].count;
    return maniImportSuccess();
}
