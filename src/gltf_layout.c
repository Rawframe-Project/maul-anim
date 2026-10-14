// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Buffer views and accessors (mani-0008): every offset, length, stride
// and alignment the specification requires, checked with 64-bit sums
// that cannot wrap (sizes stay below 2^33 x 252); then accessors read
// into binary32, sparse substitution included.

#include "gltf_layout.h"

#include "allocator.h"
#include "bytes.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"

#include <string.h>

static uint8_t ComponentSize(int64_t code)
{
    static const uint8_t kSizes[7] = {1, 1, 2, 2, 0, 4, 4};
    return code >= 5120 && code <= 5126 ? kSizes[code - 5120] : 0;
}

static maniImportResult ReadView(maniGltf* gltf, uint32_t value, uint32_t i)
{
    const maniGltfJson* g = &gltf->g;
    maniGltfView* v = &gltf->views[i];
    int64_t offset = 0;
    int64_t length = 0;
    int64_t stride = 0;
    maniImportResult r = maniGltfIndex(g, value, "buffer", true, gltf->bufferCount, i, &v->buffer);
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, value, "byteOffset", false, 0, INT64_MAX / 4, i, &offset);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, value, "byteLength", true, 1, INT64_MAX / 4, i, &length);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, value, "byteStride", false, 4, 252, i, &stride);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t at = maniJsonFind(g->json, value, "byteStride", 10);
    if (stride % 4 != 0 || stride > length)
    {
        return maniImportFail(maniImportReasonLayout, maniGltfOffset(g, at), i);
    }
    if ((uint64_t)offset + (uint64_t)length > gltf->buffers[v->buffer].byteLength)
    {
        at = maniJsonFind(g->json, value, "byteLength", 10);
        return maniImportFail(maniImportReasonLayout, maniGltfOffset(g, at), i);
    }
    *v = (maniGltfView){v->buffer, (uint32_t)stride, (uint64_t)offset, (uint64_t)length};
    return maniImportSuccess();
}

// A type's rows and columns: SCALAR, VEC2 to VEC4, MAT2 to MAT4.
static bool TypeShape(const maniGltfJson* g, uint32_t type, uint8_t* rows, uint8_t* columns)
{
    static const char* const kTypes[7] = {"SCALAR", "VEC2", "VEC3", "VEC4", "MAT2", "MAT3", "MAT4"};
    static const uint8_t kRows[7] = {1, 2, 3, 4, 2, 3, 4};
    static const uint8_t kColumns[7] = {1, 1, 1, 1, 2, 3, 4};
    for (uint32_t k = 0; k < 7; ++k)
    {
        if (maniJsonStringEquals(g->json, type, kTypes[k], strlen(kTypes[k])))
        {
            *rows = kRows[k];
            *columns = kColumns[k];
            return true;
        }
    }
    return false;
}

// The view a span reads must hold it, at an aligned offset, with no
// stride when `strideAllowed` is false; `at` is the offset to blame.
static maniImportResult CheckSpan(const maniGltf* gltf, uint32_t view, uint64_t offset,
                                  uint64_t span, uint32_t alignment, bool strideAllowed,
                                  uint64_t at, uint32_t item)
{
    const maniGltfView* v = &gltf->views[view];
    if (alignment == 0 || (v->offset + offset) % alignment != 0 ||
        (!strideAllowed && v->stride != 0) || offset > v->length || span > v->length - offset)
    {
        return maniImportFail(maniImportReasonLayout, at, item);
    }
    return maniImportSuccess();
}

static maniImportResult ReadSparse(maniGltf* gltf, uint32_t sparse, uint32_t i)
{
    const maniGltfJson* g = &gltf->g;
    maniGltfAccessor* a = &gltf->accessors[i];
    int64_t count = 0;
    int64_t code = 0;
    int64_t indexOffset = 0;
    int64_t valueOffset = 0;
    uint32_t indices = MANI_JSON_NONE;
    uint32_t values = MANI_JSON_NONE;
    maniImportResult r = maniGltfInteger(g, sparse, "count", true, 1, a->count, i, &count);
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, sparse, "indices", maniJsonObject, true, i, &indices);
    }
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, sparse, "values", maniJsonObject, true, i, &values);
    }
    if (r.result == mani_success)
    {
        r = maniGltfIndex(g, indices, "bufferView", true, gltf->viewCount, i, &a->sparseIndexView);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, indices, "byteOffset", false, 0, INT64_MAX / 4, i, &indexOffset);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, indices, "componentType", true, 5121, 5125, i, &code);
    }
    if (r.result == mani_success && code != 5121 && code != 5123 && code != 5125)
    {
        uint32_t at = maniJsonFind(g->json, indices, "componentType", 13);
        r = maniImportFail(maniImportReasonProperty, maniGltfOffset(g, at), i);
    }
    if (r.result == mani_success)
    {
        r = maniGltfIndex(g, values, "bufferView", true, gltf->viewCount, i, &a->sparseValueView);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, values, "byteOffset", false, 0, INT64_MAX / 4, i, &valueOffset);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    a->sparseCount = (uint32_t)count;
    a->sparseIndexSize = ComponentSize(code);
    a->sparseIndexOffset = (uint64_t)indexOffset;
    a->sparseValueOffset = (uint64_t)valueOffset;
    r = CheckSpan(gltf, a->sparseIndexView, a->sparseIndexOffset,
                  (uint64_t)count * a->sparseIndexSize, a->sparseIndexSize, false,
                  maniGltfOffset(g, indices), i);
    if (r.result == mani_success)
    {
        r = CheckSpan(gltf, a->sparseValueView, a->sparseValueOffset,
                      (uint64_t)count * a->elementSize, a->componentSize, false,
                      maniGltfOffset(g, values), i);
    }
    return r;
}

// componentType, normalized, type and count: the element's shape.
static maniImportResult ReadShape(maniGltf* gltf, uint32_t value, uint32_t i)
{
    const maniGltfJson* g = &gltf->g;
    maniGltfAccessor* a = &gltf->accessors[i];
    int64_t code = 0;
    int64_t count = 0;
    uint32_t type = MANI_JSON_NONE;
    maniImportResult r = maniGltfInteger(g, value, "componentType", true, 5120, 5126, i, &code);
    if (r.result == mani_success)
    {
        r = maniGltfBool(g, value, "normalized", i, &a->normalized);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, value, "count", true, 1, UINT32_MAX, i, &count);
    }
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, value, "type", maniJsonString, true, i, &type);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t at = maniJsonFind(g->json, value, "componentType", 13);
    if (ComponentSize(code) == 0 || (a->normalized && code >= 5125))
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, at), i);
    }
    if (!TypeShape(g, type, &a->rows, &a->columns))
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, type), i);
    }
    a->component = (uint8_t)(code - 5120);
    a->componentSize = ComponentSize(code);
    a->count = (uint32_t)count;
    // Matrix columns start at multiples of 4 bytes.
    uint32_t column = (uint32_t)a->rows * a->componentSize;
    a->columnStride = a->columns > 1 ? (column + 3) / 4 * 4 : column;
    a->elementSize = a->columnStride * a->columns;
    return maniImportSuccess();
}

static maniImportResult ReadAccessor(maniGltf* gltf, uint32_t value, uint32_t i)
{
    const maniGltfJson* g = &gltf->g;
    maniGltfAccessor* a = &gltf->accessors[i];
    a->value = value;
    int64_t offset = 0;
    uint32_t sparse = MANI_JSON_NONE;
    maniImportResult r = ReadShape(gltf, value, i);
    if (r.result == mani_success)
    {
        r = maniGltfIndex(g, value, "bufferView", false, gltf->viewCount, i, &a->view);
    }
    if (r.result == mani_success)
    {
        r = maniGltfInteger(g, value, "byteOffset", false, 0, INT64_MAX / 4, i, &offset);
    }
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, value, "sparse", maniJsonObject, false, i, &sparse);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    uint32_t offsetAt = maniJsonFind(g->json, value, "byteOffset", 10);
    if (a->view == MANI_JSON_NONE && offsetAt != MANI_JSON_NONE)
    {
        return maniImportFail(maniImportReasonProperty, maniGltfOffset(g, offsetAt), i);
    }
    // Without a view the elements are zeros the file does not pay for: a
    // few bytes could ask for gigabytes. Bounded as clips' keys are.
    if (a->view == MANI_JSON_NONE && a->count > MANI_GLTF_MAX_IMPLICIT)
    {
        uint32_t countAt = maniJsonFind(g->json, value, "count", 5);
        return maniImportFail(maniImportReasonLayout, maniGltfOffset(g, countAt), i);
    }
    a->offset = (uint64_t)offset;
    if (a->view != MANI_JSON_NONE)
    {
        const maniGltfView* v = &gltf->views[a->view];
        uint64_t stride = v->stride != 0 ? v->stride : a->elementSize;
        uint64_t span = (uint64_t)(a->count - 1) * stride + a->elementSize;
        if (stride < a->elementSize)
        {
            return maniImportFail(maniImportReasonLayout, maniGltfOffset(g, value), i);
        }
        r = CheckSpan(gltf, a->view, a->offset, span, a->componentSize, true,
                      maniGltfOffset(g, value), i);
    }
    if (r.result == mani_success && sparse != MANI_JSON_NONE)
    {
        r = ReadSparse(gltf, sparse, i);
    }
    return r;
}

maniImportResult maniGltfReadLayout(maniGltf* gltf)
{
    uint32_t array = MANI_JSON_NONE;
    uint32_t count = 0;
    maniImportResult r = maniGltfArray(&gltf->g, "bufferViews", maniJsonObject, &array, &count);
    if (r.result != mani_success)
    {
        return r;
    }
    gltf->views = maniAllocate(&gltf->allocator, count, sizeof(maniGltfView), 8);
    if (count > 0 && gltf->views == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    gltf->viewCount = count;
    uint32_t value = array + 1;
    for (uint32_t i = 0; i < count; ++i)
    {
        r = ReadView(gltf, value, i);
        if (r.result != mani_success)
        {
            return r;
        }
        value = gltf->json.values[value].next;
    }
    r = maniGltfArray(&gltf->g, "accessors", maniJsonObject, &array, &count);
    if (r.result != mani_success)
    {
        return r;
    }
    gltf->accessors = maniAllocate(&gltf->allocator, count, sizeof(maniGltfAccessor), 8);
    if (count > 0 && gltf->accessors == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    if (count > 0)
    {
        memset(gltf->accessors, 0, (size_t)count * sizeof(maniGltfAccessor));
    }
    gltf->accessorCount = count;
    value = array + 1;
    for (uint32_t i = 0; i < count; ++i)
    {
        r = ReadAccessor(gltf, value, i);
        if (r.result != mani_success)
        {
            return r;
        }
        value = gltf->json.values[value].next;
    }
    return maniImportSuccess();
}

// One component as binary32.
static float Component(uint8_t component, bool normalized, const uint8_t* p)
{
    switch (component)
    {
    case maniGltfByte:
    {
        int32_t c = p[0] < 128 ? (int32_t)p[0] : (int32_t)p[0] - 256;
        return normalized ? (float)(c < -127 ? -127 : c) / 127.0f : (float)c;
    }
    case maniGltfUnsignedByte:
        return normalized ? (float)p[0] / 255.0f : (float)p[0];
    case maniGltfShort:
    {
        int32_t c = (int16_t)maniLoadU16(p);
        return normalized ? (float)(c < -32767 ? -32767 : c) / 32767.0f : (float)c;
    }
    case maniGltfUnsignedShort:
        return normalized ? (float)maniLoadU16(p) / 65535.0f : (float)maniLoadU16(p);
    case maniGltfUnsignedInt:
        return (float)maniLoadU32(p);
    default:
    {
        float f = 0.0f;
        memcpy(&f, p, sizeof f);
        return f;
    }
    }
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// One element at p (its first byte, at `at` in its buffer) into out.
static maniImportResult ReadElement(const maniGltfAccessor* a, const uint8_t* p, uint64_t at,
                                    uint32_t item, float* out)
{
    for (uint32_t c = 0; c < a->columns; ++c)
    {
        for (uint32_t r = 0; r < a->rows; ++r)
        {
            uint64_t offset = (uint64_t)c * a->columnStride + (uint64_t)r * a->componentSize;
            float f = Component(a->component, a->normalized, p + offset);
            if (!Finite(f))
            {
                return maniImportFail(maniImportReasonData, at + offset, item);
            }
            *out++ = f;
        }
    }
    return maniImportSuccess();
}

// A view's bytes, or a Pending refusal (at the accessor) for an external
// buffer not given.
static maniImportResult ViewBytes(const maniGltf* gltf, uint32_t view, uint32_t item,
                                  const uint8_t** bytesOut, uint64_t* atOut)
{
    const maniGltfView* v = &gltf->views[view];
    const maniGltfBuffer* b = &gltf->buffers[v->buffer];
    *bytesOut = NULL;
    *atOut = 0;
    if (b->data == NULL)
    {
        uint64_t at = maniGltfOffset(&gltf->g, gltf->accessors[item].value);
        return maniImportFail(maniImportReasonPending, at, item);
    }
    *bytesOut = b->data + v->offset;
    *atOut = v->offset;
    return maniImportSuccess();
}

static uint32_t SparseIndex(uint8_t size, const uint8_t* p)
{
    return size == 1 ? p[0] : (size == 2 ? maniLoadU16(p) : maniLoadU32(p));
}

static maniImportResult ApplySparse(const maniGltf* gltf, uint32_t accessor, float* out)
{
    const maniGltfAccessor* a = &gltf->accessors[accessor];
    const uint8_t* indices = NULL;
    const uint8_t* values = NULL;
    uint64_t indicesAt = 0;
    uint64_t valuesAt = 0;
    maniImportResult r = ViewBytes(gltf, a->sparseIndexView, accessor, &indices, &indicesAt);
    if (r.result == mani_success)
    {
        r = ViewBytes(gltf, a->sparseValueView, accessor, &values, &valuesAt);
    }
    if (r.result != mani_success || indices == NULL || values == NULL)
    {
        return r;
    }
    uint32_t floats = (uint32_t)a->rows * a->columns;
    uint64_t previous = 0;
    for (uint32_t j = 0; r.result == mani_success && j < a->sparseCount; ++j)
    {
        uint64_t at = a->sparseIndexOffset + (uint64_t)j * a->sparseIndexSize;
        uint32_t index = SparseIndex(a->sparseIndexSize, indices + at);
        if (index >= a->count || (j > 0 && index <= previous))
        {
            return maniImportFail(maniImportReasonData, indicesAt + at, accessor);
        }
        previous = index;
        uint64_t element = a->sparseValueOffset + (uint64_t)j * a->elementSize;
        r = ReadElement(a, values + element, valuesAt + element, accessor,
                        out + (size_t)index * floats);
    }
    return r;
}

// The elements from the accessor's view.
static maniImportResult ReadDense(const maniGltf* gltf, uint32_t accessor, float* out)
{
    const maniGltfAccessor* a = &gltf->accessors[accessor];
    const uint8_t* bytes = NULL;
    uint64_t at = 0;
    maniImportResult r = ViewBytes(gltf, a->view, accessor, &bytes, &at);
    if (r.result != mani_success || bytes == NULL)
    {
        return r;
    }
    const maniGltfView* v = &gltf->views[a->view];
    uint64_t stride = v->stride != 0 ? v->stride : a->elementSize;
    size_t floats = (size_t)a->rows * a->columns;
    for (uint32_t i = 0; r.result == mani_success && i < a->count; ++i)
    {
        uint64_t element = a->offset + (uint64_t)i * stride;
        r = ReadElement(a, bytes + element, at + element, accessor, out + i * floats);
    }
    return r;
}

maniImportResult maniGltfReadFloats(const maniGltf* gltf, uint32_t accessor, float* out)
{
    const maniGltfAccessor* a = &gltf->accessors[accessor];
    maniImportResult r = maniImportSuccess();
    if (a->view == MANI_JSON_NONE)
    {
        memset(out, 0, (size_t)a->count * a->rows * a->columns * sizeof(float));
    }
    else
    {
        r = ReadDense(gltf, accessor, out);
    }
    if (r.result == mani_success && a->sparseCount > 0)
    {
        r = ApplySparse(gltf, accessor, out);
    }
    return r;
}
