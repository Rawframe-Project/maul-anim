// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A glTF document's buffers: the GLB's BIN chunk for the
// first buffer without a URI; `data:` URIs (RFC 2397, base64 by RFC 4648
// or percent-encoded) decoded into the document, their media type one
// of the two the specification allows; any other URI, or none, left for
// the host to supply; a meshopt fallback buffer never read.

#include "gltf_buffer.h"

#include "allocator.h"
#include "gltf_compression.h"
#include "gltf_document.h"
#include "gltf_property.h"
#include "json.h"

#include <string.h>

static uint8_t Lower(uint8_t c)
{
    return c >= 'A' && c <= 'Z' ? (uint8_t)(c + 32) : c;
}

static bool StartsWithFolded(const uint8_t* text, size_t length, const char* prefix)
{
    size_t n = strlen(prefix);
    if (length < n)
    {
        return false;
    }
    for (size_t i = 0; i < n; ++i)
    {
        if (Lower(text[i]) != (uint8_t)prefix[i])
        {
            return false;
        }
    }
    return true;
}

static bool EqualsFolded(const uint8_t* text, size_t length, const char* word)
{
    return length == strlen(word) && StartsWithFolded(text, length, word);
}

// Each byte's base64 digit, 255 for none (RFC 4648's alphabet).
static const uint8_t kDigits[256] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 62,  255, 255, 255, 63,  52,  53,  54,  55,  56,  57,  58,  59,  60,
    61,  255, 255, 255, 255, 255, 255, 255, 0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,
    11,  12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  255, 255, 255, 255,
    255, 255, 26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,
    43,  44,  45,  46,  47,  48,  49,  50,  51,  255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255,
};

static int32_t Base64Digit(uint8_t c)
{
    return kDigits[c] == 255 ? -1 : (int32_t)kDigits[c];
}

// The groups before the last, none padded: four digits, three bytes, a
// wrong digit found by the top bits of their union (a digit is below
// 64); out holds at least 3 bytes for each 4 read (its caller sizes it
// by the text); returns the bytes written, *atOut where it stopped.
static size_t DecodeGroups(const uint8_t* text, size_t length, uint8_t* out, size_t* atOut)
{
    size_t written = 0;
    size_t i = 0;
    for (; i + 4 < length; i += 4)
    {
        uint32_t a = kDigits[text[i]];
        uint32_t b = kDigits[text[i + 1]];
        uint32_t c = kDigits[text[i + 2]];
        uint32_t d = kDigits[text[i + 3]];
        if (((a | b | c | d) & 0xC0u) != 0)
        {
            break;
        }
        uint32_t group = a << 18 | b << 12 | c << 6 | d;
        out[written] = (uint8_t)(group >> 16);
        out[written + 1] = (uint8_t)(group >> 8);
        out[written + 2] = (uint8_t)group;
        written += 3;
    }
    *atOut = i;
    return written;
}

// Decodes base64 with its padding (RFC 4648 section 4); returns the byte
// count, or SIZE_MAX with *badOut at the first wrong character.
static size_t DecodeBase64(const uint8_t* text, size_t length, uint8_t* out, size_t* badOut)
{
    if (length % 4 != 0)
    {
        *badOut = length;
        return SIZE_MAX;
    }
    size_t i = 0;
    size_t written = DecodeGroups(text, length, out, &i);
    for (; i < length; i += 4)
    {
        bool last = i + 4 == length;
        size_t pad = last ? (text[i + 3] == '=' ? (text[i + 2] == '=' ? 2 : 1) : 0) : 0;
        uint32_t group = 0;
        for (size_t k = 0; k < 4; ++k)
        {
            int32_t digit = k < 4 - pad ? Base64Digit(text[i + k]) : 0;
            if (digit < 0)
            {
                *badOut = i + k;
                return SIZE_MAX;
            }
            group = group << 6 | (uint32_t)digit;
        }
        out[written++] = (uint8_t)(group >> 16);
        if (pad < 2)
        {
            out[written++] = (uint8_t)(group >> 8);
        }
        if (pad < 1)
        {
            out[written++] = (uint8_t)group;
        }
    }
    return written;
}

static int32_t HexDigit(uint8_t c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    c = Lower(c);
    return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

// Decodes percent escapes (RFC 3986 section 2.1); other bytes as they are.
static size_t DecodePercent(const uint8_t* text, size_t length, uint8_t* out, size_t* badOut)
{
    size_t written = 0;
    for (size_t i = 0; i < length; ++i)
    {
        if (text[i] != '%')
        {
            out[written++] = text[i];
            continue;
        }
        int32_t high = i + 2 < length ? HexDigit(text[i + 1]) : -1;
        int32_t low = i + 2 < length ? HexDigit(text[i + 2]) : -1;
        if (high < 0 || low < 0)
        {
            *badOut = i;
            return SIZE_MAX;
        }
        out[written++] = (uint8_t)(high * 16 + low);
        i += 2;
    }
    return written;
}

// The header between "data:" and the comma: the media type, one of the
// two glTF allows, then parameters; whether the last is ";base64".
static bool ReadHeader(const uint8_t* header, size_t length, bool* base64Out)
{
    size_t end = 0;
    while (end < length && header[end] != ';')
    {
        ++end;
    }
    if (!EqualsFolded(header, end, "application/octet-stream") &&
        !EqualsFolded(header, end, "application/gltf-buffer"))
    {
        return false;
    }
    *base64Out = length >= 7 && EqualsFolded(header + length - 7, 7, ";base64");
    return true;
}

// A `data:` URI's bytes into the buffer; `at` is the URI's offset.
static maniImportResult DecodeDataUri(maniGltf* gltf, const uint8_t* uri, size_t length,
                                      uint64_t at, uint32_t i)
{
    maniGltfBuffer* b = &gltf->buffers[i];
    size_t comma = 5;
    while (comma < length && uri[comma] != ',')
    {
        ++comma;
    }
    bool base64 = false;
    if (comma == length || !ReadHeader(uri + 5, comma - 5, &base64))
    {
        return maniImportFail(maniImportReasonBuffer, at, i);
    }
    // Decoding never lengthens: base64 gives 3 bytes for 4, a percent
    // escape 1 for 3.
    size_t capacity = length - comma;
    b->owned = maniAllocate(&gltf->allocator, capacity, 1, 8);
    if (b->owned == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    b->ownedSize = capacity;
    size_t bad = 0;
    const uint8_t* data = uri + comma + 1;
    size_t dataLength = length - comma - 1;
    size_t n = base64 ? DecodeBase64(data, dataLength, b->owned, &bad)
                      : DecodePercent(data, dataLength, b->owned, &bad);
    if (n == SIZE_MAX || n < b->byteLength)
    {
        return maniImportFail(maniImportReasonBuffer, at, i);
    }
    b->data = b->owned;
    return maniImportSuccess();
}

static maniImportResult ReadBuffer(maniGltf* gltf, uint32_t value, uint32_t i, uint8_t* scratch)
{
    const maniGltfJson* g = &gltf->g;
    maniGltfBuffer* b = &gltf->buffers[i];
    int64_t byteLength = 0;
    uint32_t uri = MANI_JSON_NONE;
    maniImportResult r =
        maniGltfInteger(g, value, "byteLength", true, 1, INT64_MAX / 4, i, &byteLength);
    if (r.result == mani_success)
    {
        r = maniGltfMember(g, value, "uri", maniJsonString, false, i, &uri);
    }
    if (r.result == mani_success)
    {
        r = maniGltfReadFallback(gltf, value, i);
    }
    b->byteLength = (uint64_t)byteLength;
    if (r.result != mani_success || b->fallback != 0)
    {
        return r;
    }
    if (uri == MANI_JSON_NONE)
    {
        if (i == 0 && gltf->bin != NULL)
        {
            if (gltf->binSize < b->byteLength)
            {
                return maniImportFail(maniImportReasonBuffer, maniGltfOffset(g, value), i);
            }
            b->data = gltf->bin;
            return maniImportSuccess();
        }
        // A GLB's first buffer needs its BIN chunk; any other buffer
        // without a URI is the host's to supply.
        b->external = true;
        return i == 0 && gltf->isGlb
                   ? maniImportFail(maniImportReasonBuffer, maniGltfOffset(g, value), i)
                   : maniImportSuccess();
    }
    size_t length = maniJsonStringDecode(&gltf->json, uri, scratch, gltf->json.values[uri].length);
    if (StartsWithFolded(scratch, length, "data:"))
    {
        return DecodeDataUri(gltf, scratch, length, maniGltfOffset(g, uri), i);
    }
    b->external = true;
    b->uri = maniAllocate(&gltf->allocator, length + 1, 1, 8);
    if (b->uri == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    memcpy(b->uri, scratch, length);
    b->uri[length] = '\0';
    b->uriLength = (uint32_t)length;
    return maniImportSuccess();
}

maniImportResult maniGltfReadBuffers(maniGltf* gltf)
{
    uint32_t array = MANI_JSON_NONE;
    uint32_t count = 0;
    maniImportResult r = maniGltfArray(&gltf->g, "buffers", maniJsonObject, &array, &count);
    if (r.result != mani_success || count == 0)
    {
        return r;
    }
    gltf->buffers = maniAllocate(&gltf->allocator, count, sizeof(maniGltfBuffer), 8);
    if (gltf->buffers == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    // Counted (and empty) at once, so a release after a later failure
    // frees the list whole.
    memset(gltf->buffers, 0, (size_t)count * sizeof(maniGltfBuffer));
    gltf->bufferCount = count;
    // A URI decodes into at most its own length: one scratch the JSON's
    // size serves every buffer.
    size_t scratchSize = gltf->json.values[0].length + 1;
    uint8_t* scratch = maniAllocate(&gltf->allocator, scratchSize, 1, 8);
    if (scratch == NULL)
    {
        return (maniImportResult){mani_errorCapacity, maniImportReasonNone, 0, 0};
    }
    uint32_t value = array + 1;
    for (uint32_t i = 0; r.result == mani_success && i < count; ++i)
    {
        r = ReadBuffer(gltf, value, i, scratch);
        value = gltf->json.values[value].next;
    }
    maniRelease(&gltf->allocator, scratch, scratchSize, 1, 8);
    return r;
}

void maniGltfReleaseBuffers(maniGltf* gltf)
{
    for (uint32_t i = 0; i < gltf->bufferCount; ++i)
    {
        maniGltfBuffer* b = &gltf->buffers[i];
        maniRelease(&gltf->allocator, b->owned, b->ownedSize, 1, 8);
        maniRelease(&gltf->allocator, b->uri, b->uri != NULL ? (size_t)b->uriLength + 1 : 0, 1, 8);
    }
    maniRelease(&gltf->allocator, gltf->buffers, gltf->bufferCount, sizeof(maniGltfBuffer), 8);
}
