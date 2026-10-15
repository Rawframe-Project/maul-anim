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

#include "maul-anim/base.h"

#include <stdbool.h>
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

// A string property's decoded bytes, one at a time.
typedef struct Reader
{
    const uint8_t* p;
    uint32_t raw;
    uint32_t at;
    bool entities;
} Reader;

static bool Open(const maniFbx* fbx, uint32_t property, Reader* r)
{
    r->at = 0;
    r->entities = false;
    if (!maniFbxString(fbx, property, &r->p, &r->raw))
    {
        return false;
    }
    r->entities = fbx->ascii && fbx->properties[property].type == 'S';
    return true;
}

static uint8_t Next(Reader* r)
{
    return r->entities ? NextByte(r->p, r->raw, &r->at) : r->p[r->at++];
}

// The length of the UTF-8 sequence starting b (n bytes at hand), or 0
// for none, by ufbx's rules (ufbxi_utf8_valid_length): no NUL, no
// overlong form, no surrogate, nothing past U+10FFFF.
static uint32_t SequenceLength(const uint8_t* b, uint32_t n)
{
    uint32_t c = b[0];
    if (c < 0x80)
    {
        return c != 0 ? 1 : 0;
    }
    if ((c & 0xe0) == 0xc0 && n >= 2)
    {
        uint32_t code = c << 8 | b[1];
        return (code & 0xc0) == 0x80 && code >= 0xc280 ? 2 : 0;
    }
    if ((c & 0xf0) == 0xe0 && n >= 3)
    {
        uint32_t code = c << 16 | (uint32_t)b[1] << 8 | b[2];
        bool valid =
            (code & 0xc0c0) == 0x8080 && code >= 0xe0a080 && (code < 0xeda080 || code >= 0xee8080);
        return valid ? 3 : 0;
    }
    if ((c & 0xf8) == 0xf0 && n >= 4)
    {
        uint32_t code = c << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
        return (code & 0xc0c0c0) == 0x808080 && code >= 0xf0908080u && code <= 0xf48fbfbfu ? 4 : 0;
    }
    return 0;
}

// A string's characters made valid UTF-8 as ufbx makes them
// (mani-0012): each valid sequence whole, each other byte U+FFFD; over
// the decoded bytes from the start or a given position to the end or a
// given one.
typedef struct Text
{
    Reader r;
    uint64_t decoded;
    uint64_t end;
    uint8_t ahead[4];
    uint32_t held;
} Text;

static bool OpenText(const maniFbx* fbx, uint32_t property, uint64_t from, uint64_t to, Text* t)
{
    t->decoded = 0;
    t->end = to;
    t->held = 0;
    if (!Open(fbx, property, &t->r))
    {
        return false;
    }
    while (t->decoded < from && t->r.at < t->r.raw)
    {
        (void)Next(&t->r);
        ++t->decoded;
    }
    return true;
}

// The next character's bytes into out; 0 at the end.
static uint32_t NextCharacter(Text* t, uint8_t out[4])
{
    while (t->held < 4 && t->decoded < t->end && t->r.at < t->r.raw)
    {
        t->ahead[t->held++] = Next(&t->r);
        ++t->decoded;
    }
    if (t->held == 0)
    {
        return 0;
    }
    static const uint8_t kReplacement[3] = {0xef, 0xbf, 0xbd};
    uint32_t n = SequenceLength(t->ahead, t->held);
    uint32_t used = n != 0 ? n : 1;
    memcpy(out, n != 0 ? t->ahead : kReplacement, n != 0 ? n : 3);
    memmove(t->ahead, t->ahead + used, t->held - used);
    t->held -= used;
    return n != 0 ? n : 3;
}

// A range of a string's decoded bytes made valid into out (NULL to
// count): whole characters up to capacity.
static size_t DecodeRange(const maniFbx* fbx, uint32_t property, uint64_t from, uint64_t to,
                          uint8_t* out, size_t capacity)
{
    Text t;
    if (!OpenText(fbx, property, from, to, &t))
    {
        return 0;
    }
    size_t written = 0;
    uint8_t c[4];
    for (uint32_t n = NextCharacter(&t, c); n != 0; n = NextCharacter(&t, c))
    {
        if (out != NULL && capacity - written < n)
        {
            break;
        }
        if (out != NULL)
        {
            memcpy(out + written, c, n);
        }
        written += n;
    }
    return written;
}

size_t maniFbxStringDecode(const maniFbx* fbx, uint32_t property, uint8_t* out, size_t capacity)
{
    return DecodeRange(fbx, property, 0, UINT64_MAX, out, capacity);
}

size_t maniFbxStringNameRange(const maniFbx* fbx, uint32_t property, bool binary, uint8_t* out,
                              size_t capacity)
{
    // Binary: Name\x00\x01Class. ASCII: Class::Name. Found in the bytes
    // before they are made valid, as ufbx splits them.
    Reader r;
    if (!Open(fbx, property, &r))
    {
        return 0;
    }
    uint64_t from = 0;
    uint64_t to = UINT64_MAX;
    uint8_t previous = 0;
    for (uint64_t k = 0; r.at < r.raw && to == UINT64_MAX && from == 0; ++k)
    {
        uint8_t c = Next(&r);
        to = binary && k > 0 && previous == 0 && c == 1 ? k - 1 : to;
        from = !binary && k > 0 && previous == ':' && c == ':' ? k + 1 : from;
        previous = c;
    }
    return DecodeRange(fbx, property, from, to, out, capacity);
}

bool maniFbxStringEquals(const maniFbx* fbx, uint32_t property, const char* bytes, size_t length)
{
    Text t;
    if (!OpenText(fbx, property, 0, UINT64_MAX, &t))
    {
        return false;
    }
    size_t matched = 0;
    uint8_t c[4];
    for (uint32_t n = NextCharacter(&t, c); n != 0; n = NextCharacter(&t, c))
    {
        if (length - matched < n || memcmp(bytes + matched, c, n) != 0)
        {
            return false;
        }
        matched += n;
    }
    return matched == length;
}

uint64_t maniFbxNameHash(const void* bytes, size_t length)
{
    const uint8_t* p = bytes;
    uint64_t h = MANI_HASH_INIT;
    for (size_t i = 0; i < length; ++i)
    {
        h = maniHash64(h, p + i, 1);
    }
    return h;
}

uint64_t maniFbxStringHash(const maniFbx* fbx, uint32_t property)
{
    uint64_t h = MANI_HASH_INIT;
    Text t;
    if (!OpenText(fbx, property, 0, UINT64_MAX, &t))
    {
        return h;
    }
    uint8_t c[4];
    for (uint32_t n = NextCharacter(&t, c); n != 0; n = NextCharacter(&t, c))
    {
        for (uint32_t i = 0; i < n; ++i)
        {
            h = maniHash64(h, c + i, 1);
        }
    }
    return h;
}

bool maniFbxSameString(const maniFbx* fbx, uint32_t a, uint32_t b)
{
    Text x;
    Text y;
    if (!OpenText(fbx, a, 0, UINT64_MAX, &x) || !OpenText(fbx, b, 0, UINT64_MAX, &y))
    {
        return false;
    }
    uint8_t cx[4];
    uint8_t cy[4];
    uint32_t n = 0;
    do
    {
        n = NextCharacter(&x, cx);
        if (n != NextCharacter(&y, cy) || memcmp(cx, cy, n) != 0)
        {
            return false;
        }
    } while (n != 0);
    return true;
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

bool maniFbxCharacter(const maniFbx* fbx, uint32_t property, int64_t* out)
{
    if (property < fbx->propertyCount && fbx->properties[property].type == 'W')
    {
        *out = fbx->bytes[fbx->properties[property].offset];
        return true;
    }
    return maniFbxInteger(fbx, property, INT64_MIN, INT64_MAX, out);
}

// Whether a word is one ufbx reads as infinite or not a number: inf,
// infinity or nan in any case, with an optional sign.
static bool NonFiniteWord(const uint8_t* bytes, uint64_t length)
{
    static const char* const kWords[3] = {"inf", "infinity", "nan"};
    for (uint32_t w = 0; w < 3; ++w)
    {
        size_t n = strlen(kWords[w]);
        bool same = length == n;
        for (size_t i = 0; same && i < n; ++i)
        {
            same = (bytes[i] | 0x20) == (uint8_t)kWords[w][i];
        }
        if (same)
        {
            return true;
        }
    }
    return false;
}

// One scalar of a value list.
static maniImportResult ListValue(const maniFbx* fbx, uint32_t property, double* out)
{
    const maniFbxProperty* p = &fbx->properties[property];
    if (p->type == 'W')
    {
        const uint8_t* bytes = fbx->bytes + p->offset;
        *out = bytes[0];
        return NonFiniteWord(bytes, p->encodedSize)
                   ? maniFbxFail(maniImportReasonData, p->offset, 0)
                   : maniFbxSuccess();
    }
    if (p->type == 'S' || p->type == 'R' || maniFbxArrayCount(fbx, property) > 0)
    {
        return maniFbxFail(maniImportReasonProperty, p->offset, 0);
    }
    return maniFbxNumber(fbx, property, out) ? maniFbxSuccess()
                                             : maniFbxFail(maniImportReasonData, p->offset, 0);
}

maniImportResult maniFbxReadValues(const maniFbx* fbx, uint32_t node, double* out,
                                   uint32_t* countOut)
{
    const maniFbxNode* n = &fbx->nodes[node];
    uint32_t first = maniFbxNodeProperty(fbx, node, 0);
    bool array = n->propertyCount == 1 && maniFbxArrayCount(fbx, first) > 0;
    *countOut = array ? maniFbxArrayCount(fbx, first) : n->propertyCount;
    if (out == NULL)
    {
        return maniFbxSuccess();
    }
    if (array)
    {
        return maniFbxReadDoubles(fbx, first, out);
    }
    maniImportResult r = maniFbxSuccess();
    for (uint32_t k = 0; r.result == mani_success && k < n->propertyCount; ++k)
    {
        r = ListValue(fbx, n->firstProperty + k, &out[k]);
    }
    return r;
}
