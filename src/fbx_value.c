// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX property values read on request (mani-0012). Scalars and raw
// arrays are read where they lie in the file, in its byte order; a zlib
// array is inflated by the family's inflate into exactly its decoded
// size first, so a stream that gives more or less, or fails its
// checksum, is refused at the array.

#include "fbx_value.h"

#include "allocator.h"
#include "fbx_ascii.h"
#include "fbx_binary.h"
#include "fbx_document.h"
#include "inflate.h"

#include <stdint.h>
#include <string.h>

uint32_t maniFbxNodeProperty(const maniFbx* fbx, uint32_t node, uint32_t k)
{
    const maniFbxNode* n = &fbx->nodes[node];
    return k < n->propertyCount ? n->firstProperty + k : MANI_FBX_NONE;
}

static bool Finite(double v)
{
    uint64_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    return (bits & 0x7FF0000000000000u) != 0x7FF0000000000000u;
}

static float AsFloat(uint32_t bits)
{
    float f = 0.0f;
    memcpy(&f, &bits, sizeof f);
    return f;
}

static double AsDouble(uint64_t bits)
{
    double d = 0.0;
    memcpy(&d, &bits, sizeof d);
    return d;
}

// A word of the file at `at` in its byte order, `bytes` long, sign
// extended for the signed integer types.
static int64_t LoadSigned(const maniFbx* fbx, uint64_t at, uint32_t bytes)
{
    if (bytes == 8)
    {
        return (int64_t)maniFbxLoad64(fbx, at);
    }
    if (bytes == 4)
    {
        return (int32_t)maniFbxLoad32(fbx, at);
    }
    const uint8_t* p = fbx->bytes + at;
    uint16_t v = fbx->bigEndian ? (uint16_t)(p[0] << 8 | p[1]) : (uint16_t)(p[1] << 8 | p[0]);
    return (int16_t)v;
}

bool maniFbxNumber(const maniFbx* fbx, uint32_t property, double* out)
{
    if (property >= fbx->propertyCount)
    {
        return false;
    }
    const maniFbxProperty* p = &fbx->properties[property];
    double v = 0.0;
    switch (p->type)
    {
    case 'N':
        if (!maniFbxTextDouble(fbx, p->offset, p->encodedSize, &v))
        {
            return false;
        }
        break;
    case 'C':
    case 'B':
    case 'Z':
        v = fbx->bytes[p->offset];
        break;
    case 'Y':
        v = (double)LoadSigned(fbx, p->offset, 2);
        break;
    case 'I':
        v = (double)LoadSigned(fbx, p->offset, 4);
        break;
    case 'L':
        v = (double)LoadSigned(fbx, p->offset, 8);
        break;
    case 'F':
        v = (double)AsFloat(maniFbxLoad32(fbx, p->offset));
        break;
    case 'D':
        v = AsDouble(maniFbxLoad64(fbx, p->offset));
        break;
    default:
        return false;
    }
    if (!Finite(v))
    {
        return false;
    }
    *out = v;
    return true;
}

bool maniFbxInteger(const maniFbx* fbx, uint32_t property, int64_t minimum, int64_t maximum,
                    int64_t* out)
{
    if (property >= fbx->propertyCount)
    {
        return false;
    }
    const maniFbxProperty* p = &fbx->properties[property];
    int64_t v = 0;
    if (p->type == 'N')
    {
        return maniFbxTextInteger(fbx, p->offset, p->encodedSize, minimum, maximum, out);
    }
    if (p->type == 'L' || p->type == 'I' || p->type == 'Y')
    {
        v = LoadSigned(fbx, p->offset, p->type == 'L' ? 8 : (p->type == 'I' ? 4 : 2));
    }
    else
    {
        double d = 0.0;
        // The range first: converting past it is undefined.
        if (!maniFbxNumber(fbx, property, &d) || !(d >= -0x1p63 && d < 0x1p63) ||
            d != (double)(int64_t)d)
        {
            return false;
        }
        v = (int64_t)d;
    }
    if (v < minimum || v > maximum)
    {
        return false;
    }
    *out = v;
    return true;
}

static bool IsText(uint8_t type)
{
    return type == 'S' || type == 'R' || type == 'W';
}

bool maniFbxString(const maniFbx* fbx, uint32_t property, const uint8_t** bytesOut,
                   uint32_t* lengthOut)
{
    if (property >= fbx->propertyCount || !IsText(fbx->properties[property].type))
    {
        return false;
    }
    const maniFbxProperty* p = &fbx->properties[property];
    *bytesOut = fbx->bytes + p->offset;
    *lengthOut = p->count;
    return true;
}

// The next decoded byte of an ASCII string at *at: FBX's entities
// "&quot;", "&cr;" and "&lf;"; any other '&' stands for itself.
static uint8_t NextByte(const uint8_t* p, uint32_t length, uint32_t* at)
{
    static const char* const kEntities[3] = {"&quot;", "&cr;", "&lf;"};
    static const uint8_t kBytes[3] = {'"', '\r', '\n'};
    if (p[*at] == '&')
    {
        for (uint32_t k = 0; k < 3; ++k)
        {
            uint32_t n = (uint32_t)strlen(kEntities[k]);
            if (length - *at >= n && memcmp(p + *at, kEntities[k], n) == 0)
            {
                *at += n;
                return kBytes[k];
            }
        }
    }
    return p[(*at)++];
}

size_t maniFbxStringDecode(const maniFbx* fbx, uint32_t property, uint8_t* out, size_t capacity)
{
    const uint8_t* p = NULL;
    uint32_t length = 0;
    if (!maniFbxString(fbx, property, &p, &length))
    {
        return 0;
    }
    bool entities = fbx->ascii && fbx->properties[property].type == 'S';
    size_t written = 0;
    for (uint32_t at = 0; at < length && written < capacity;)
    {
        out[written++] = entities ? NextByte(p, length, &at) : p[at++];
    }
    return written;
}

bool maniFbxStringEquals(const maniFbx* fbx, uint32_t property, const char* bytes, size_t length)
{
    const uint8_t* p = NULL;
    uint32_t raw = 0;
    if (!maniFbxString(fbx, property, &p, &raw))
    {
        return false;
    }
    bool entities = fbx->ascii && fbx->properties[property].type == 'S';
    size_t matched = 0;
    for (uint32_t at = 0; at < raw;)
    {
        uint8_t c = entities ? NextByte(p, raw, &at) : p[at++];
        if (matched == length || (uint8_t)bytes[matched] != c)
        {
            return false;
        }
        ++matched;
    }
    return matched == length;
}

static uint32_t ElementSize(uint8_t type)
{
    switch (type)
    {
    case 'b':
    case 'c':
        return 1;
    case 'i':
    case 'f':
        return 4;
    case 'l':
    case 'd':
        return 8;
    default:
        return 0;
    }
}

uint32_t maniFbxArrayCount(const maniFbx* fbx, uint32_t property)
{
    bool array =
        property < fbx->propertyCount &&
        (ElementSize(fbx->properties[property].type) != 0 || fbx->properties[property].type == 'A');
    return array ? fbx->properties[property].count : 0;
}

// The array's decoded elements: where they lie in the file, or a block
// inflated into (*ownedOut, released by the caller).
typedef struct Elements
{
    const uint8_t* bytes;
    uint8_t* owned;
    size_t size;
    // Where the n-th element's refusal points: into the file when raw,
    // at the array when inflated.
    bool raw;
} Elements;

static maniImportResult Decode(const maniFbx* fbx, const maniFbxProperty* p, Elements* e)
{
    uint64_t size = (uint64_t)p->count * ElementSize(p->type);
    *e = (Elements){fbx->bytes + p->offset, NULL, (size_t)size, true};
    if (p->encoding == 0)
    {
        return maniFbxSuccess();
    }
    if (size > SIZE_MAX / 2)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    e->owned = maniAllocate(&fbx->allocator, size + 1, 1, 8);
    if (e->owned == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0, 0};
    }
    e->bytes = e->owned;
    e->raw = false;
    if (maniInflateZlib(fbx->bytes + p->offset, (size_t)p->encodedSize, e->owned, (size_t)size) !=
        mani_success)
    {
        return maniFbxFail(maniImportReasonData, p->offset, 0);
    }
    return maniFbxSuccess();
}

// Element i of a decoded array, as the file's byte order has it.
static uint64_t ElementBits(const maniFbx* fbx, const Elements* e, uint32_t size, size_t i)
{
    const uint8_t* q = e->bytes + i * size;
    uint64_t v = 0;
    for (uint32_t k = 0; k < size; ++k)
    {
        uint32_t byte = fbx->bigEndian ? k : size - 1 - k;
        v = v << 8 | q[byte];
    }
    return v;
}

typedef enum Want
{
    WantDoubles,
    WantIntegers,
    // Binary32 bit patterns: float and int32 arrays as they are.
    WantBits,
} Want;

// Element i into out, as the caller wants it.
static maniImportResult Store(const maniFbx* fbx, const maniFbxProperty* p, const Elements* e,
                              size_t i, Want want, void* out)
{
    uint32_t size = ElementSize(p->type);
    uint64_t bits = ElementBits(fbx, e, size, i);
    if (want == WantBits)
    {
        ((uint32_t*)out)[i] = (uint32_t)bits;
        return maniFbxSuccess();
    }
    if (p->type == 'f' || p->type == 'd')
    {
        double d = p->type == 'f' ? (double)AsFloat((uint32_t)bits) : AsDouble(bits);
        if (!Finite(d))
        {
            return maniFbxFail(maniImportReasonData, e->raw ? p->offset + i * size : p->offset, 0);
        }
        ((double*)out)[i] = d;
        return maniFbxSuccess();
    }
    int64_t n = p->type == 'i' ? (int32_t)(uint32_t)bits : (int64_t)bits;
    if (want == WantDoubles)
    {
        ((double*)out)[i] = (double)n;
    }
    else
    {
        ((int64_t*)out)[i] = n;
    }
    return maniFbxSuccess();
}

// A text number as binary32 bits: from 7.2 on, the bits written as an
// integer (FBX's KeyAttrDataFloat packs integers into floats); before,
// a float's value.
static bool TextBits(const maniFbx* fbx, uint64_t start, uint64_t length, uint32_t* out)
{
    if (fbx->version >= 7200)
    {
        int64_t n = 0;
        bool fine = maniFbxTextInteger(fbx, start, length, INT32_MIN, UINT32_MAX, &n);
        *out = (uint32_t)n;
        return fine;
    }
    double d = 0.0;
    float f = 0.0f;
    bool fine = maniFbxTextDouble(fbx, start, length, &d);
    f = (float)d;
    memcpy(out, &f, sizeof f);
    return fine;
}

// An ASCII array's numbers converted one by one, each refusal at its
// text.
static maniImportResult ReadText(const maniFbx* fbx, const maniFbxProperty* p, Want want, void* out)
{
    uint64_t at = p->offset;
    uint64_t end = p->offset + p->encodedSize;
    for (uint32_t i = 0; i < p->count; ++i)
    {
        uint64_t start = 0;
        uint64_t length = 0;
        bool read = maniFbxAsciiNextNumber(fbx, &at, end, &start, &length);
        bool fine =
            read && (want == WantBits      ? TextBits(fbx, start, length, (uint32_t*)out + i)
                     : want == WantDoubles ? maniFbxTextDouble(fbx, start, length, (double*)out + i)
                                           : maniFbxTextInteger(fbx, start, length, INT64_MIN,
                                                                INT64_MAX, (int64_t*)out + i));
        if (!fine)
        {
            return maniFbxFail(maniImportReasonData, read ? start : p->offset, 0);
        }
    }
    return maniFbxSuccess();
}

static maniImportResult Read(const maniFbx* fbx, uint32_t property, Want want, void* out)
{
    if (property < fbx->propertyCount && fbx->properties[property].type == 'A')
    {
        return ReadText(fbx, &fbx->properties[property], want, out);
    }
    const maniFbxProperty* p = property < fbx->propertyCount ? &fbx->properties[property] : NULL;
    uint32_t size = p != NULL ? ElementSize(p->type) : 0;
    bool floats = p != NULL && (p->type == 'f' || p->type == 'd');
    bool words = p != NULL && (p->type == 'f' || p->type == 'i');
    if (size == 0 || (want == WantIntegers && floats) || (want == WantBits && !words))
    {
        return maniFbxFail(maniImportReasonProperty, p != NULL ? p->offset : 0, property);
    }
    Elements e;
    maniImportResult r = Decode(fbx, p, &e);
    for (size_t i = 0; r.result == mani_success && i < p->count; ++i)
    {
        r = Store(fbx, p, &e, i, want, out);
    }
    maniRelease(&fbx->allocator, e.owned, e.size + 1, 1, 8);
    return r;
}

maniImportResult maniFbxReadDoubles(const maniFbx* fbx, uint32_t property, double* out)
{
    return Read(fbx, property, WantDoubles, out);
}

maniImportResult maniFbxReadIntegers(const maniFbx* fbx, uint32_t property, int64_t* out)
{
    return Read(fbx, property, WantIntegers, out);
}

maniImportResult maniFbxReadFloatBits(const maniFbx* fbx, uint32_t property, uint32_t* out)
{
    return Read(fbx, property, WantBits, out);
}
