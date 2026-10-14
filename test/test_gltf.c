// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The glTF document (mani-0008): JSON and GLB containers, the asset
// version, required extensions, `data:` and external buffers, buffer
// views, accessors read into binary32 (normalized integers, matrix
// column padding, strides, sparse substitution), and every refusal with
// its reason and offset. Run as test_gltf --seed <file> it writes the
// fuzz target's seed, a GLB.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "gltf_document.h"
#include "gltf_layout.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/import.h"

#include <stdio.h>
#include <string.h>

static char s_json[16384];
static _Alignas(16) uint8_t s_glb[16384];
static size_t s_glbSize;

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void Base64(const uint8_t* bytes, size_t size, char* out)
{
    static const char kDigits[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t o = 0;
    for (size_t i = 0; i < size; i += 3)
    {
        uint32_t group = (uint32_t)bytes[i] << 16;
        group |= i + 1 < size ? (uint32_t)bytes[i + 1] << 8 : 0;
        group |= i + 2 < size ? bytes[i + 2] : 0;
        out[o++] = kDigits[group >> 18];
        out[o++] = kDigits[(group >> 12) & 63];
        out[o++] = i + 1 < size ? kDigits[(group >> 6) & 63] : '=';
        out[o++] = i + 2 < size ? kDigits[group & 63] : '=';
    }
    out[o] = '\0';
}

// The test buffer, 64 bytes:
//   0: three floats 1, 2, 3 and three floats -4, 5.5, 6 (24 bytes)
//  24: bytes 127, -128, 0, 64 (snorm8), then 255, 0 (unorm8) and pad
//  32: MAT2 of bytes, columns padded to 4: 1 2 _ _ 3 4 _ _
//  40: shorts 32767, -32768 (snorm16)
//  44: sparse indices (u8) 1, then values 9, 8, 7 (floats) at 48
//  60: a NaN
static uint8_t s_buffer[64];

static void FillBuffer(void)
{
    memset(s_buffer, 0, sizeof s_buffer);
    const float floats[6] = {1, 2, 3, -4, 5.5f, 6};
    memcpy(s_buffer, floats, sizeof floats);
    const int8_t snorm[4] = {127, -128, 0, 64};
    memcpy(s_buffer + 24, snorm, 4);
    s_buffer[28] = 255;
    s_buffer[29] = 0;
    const uint8_t mat2[8] = {1, 2, 0, 0, 3, 4, 0, 0};
    memcpy(s_buffer + 32, mat2, 8);
    maniStoreU16(s_buffer + 40, 32767);
    maniStoreU16(s_buffer + 42, 0x8000);
    s_buffer[44] = 1;
    const float values[3] = {9, 8, 7};
    memcpy(s_buffer + 48, values, sizeof values);
    const uint32_t nan = 0x7FC00000u;
    memcpy(s_buffer + 60, &nan, 4);
}

// A document over the test buffer with these views and accessors.
static void Document(const char* uri, const char* views, const char* accessors, const char* extra)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},%s\"buffers\":[{\"byteLength\":64%s%s%s}],"
             "\"bufferViews\":[%s],\"accessors\":[%s]}",
             extra, uri[0] != '\0' ? ",\"uri\":\"" : "", uri, uri[0] != '\0' ? "\"" : "", views,
             accessors);
}

static char s_dataUri[256];

static const char* DataUri(void)
{
    char digits[200];
    FillBuffer();
    Base64(s_buffer, sizeof s_buffer, digits);
    snprintf(s_dataUri, sizeof s_dataUri, "data:application/octet-stream;base64,%s", digits);
    return s_dataUri;
}

static const char kViews[] = "{\"buffer\":0,\"byteLength\":64}";

static maniImportResult OpenJson(maniBuilder* builder, maniGltf** gltf)
{
    return maniOpenGltf(builder, s_json, strlen(s_json), gltf);
}

static void ExpectRefusal(maniImportReason reason, const char* at, uint32_t item, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniImportResult r = OpenJson(builder, &gltf);
    uint64_t offset = at != NULL ? (uint64_t)(strstr(s_json, at) - s_json) : UINT64_MAX;
    bool right = r.result == mani_errorFormat && r.reason == reason && r.item == item &&
                 (at == NULL || r.offset == offset) && gltf == NULL;
    CHECK(right, what);
    if (!right)
    {
        printf("  reason %u offset %llu (want %llu) item %u\n", r.reason,
               (unsigned long long)r.offset, (unsigned long long)offset, r.item);
    }
    maniDestroyBuilder(builder);
}

static void TestMinimal(void)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    const char* json = "{\"asset\":{\"version\":\"2.1\",\"generator\":\"x\"}}";
    CHECK(maniOpenGltf(builder, json, strlen(json), &gltf).result == mani_success, "opens");
    CHECK(maniGltfBufferCount(gltf) == 0, "no buffers");
    maniCloseGltf(gltf);
    maniCloseGltf(NULL);
    CHECK(maniOpenGltf(builder, NULL, 0, &gltf).result == mani_errorInvalid, "null bytes");
    CHECK(maniOpenGltf(builder, json, strlen(json), NULL).result == mani_errorInvalid, "null out");
    CHECK(maniBuilderMisuses(builder) == 2, "misuses");
    CHECK(maniOpenGltf(NULL, json, strlen(json), &gltf).result == mani_errorInvalid, "no builder");
    maniDestroyBuilder(builder);
}

static void TestVersionsAndExtensions(void)
{
    snprintf(s_json, sizeof s_json, "{\"asset\":{\"version\":\"1.0\"}}");
    ExpectRefusal(maniImportReasonVersion, "1.0\"", 0, "version 1");
    snprintf(s_json, sizeof s_json, "{\"asset\":{\"version\":\"2\"}}");
    ExpectRefusal(maniImportReasonVersion, "2\"", 0, "no minor");
    snprintf(s_json, sizeof s_json, "{\"asset\":{\"version\":\"2.0.1\"}}");
    ExpectRefusal(maniImportReasonVersion, "2.0.1\"", 0, "three parts");
    snprintf(s_json, sizeof s_json, "{\"asset\":{\"version\":\"2.0\",\"minVersion\":\"2.1\"}}");
    ExpectRefusal(maniImportReasonVersion, "2.1\"}", 0, "a newer minimum");
    snprintf(s_json, sizeof s_json, "{\"asset\":{}}");
    ExpectRefusal(maniImportReasonProperty, "{}", 0, "no version");
    snprintf(s_json, sizeof s_json, "{\"assets\":{}}");
    ExpectRefusal(maniImportReasonProperty, NULL, 0, "no asset");
    snprintf(s_json, sizeof s_json, "[1]");
    ExpectRefusal(maniImportReasonProperty, NULL, 0, "not an object");
    snprintf(s_json, sizeof s_json, "{\"asset\":{\"version\":\"2.0\"} x");
    ExpectRefusal(maniImportReasonJson, "x", maniJsonReasonSyntax, "the JSON's reason");
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"extensionsRequired\":"
             "[\"KHR_mesh_quantization\",\"EXT_meshopt_compression\"]}");
    ExpectRefusal(maniImportReasonExtension, "EXT_meshopt", 1, "an unread extension");
    snprintf(
        s_json, sizeof s_json,
        "{\"asset\":{\"version\":\"2.0\"},\"extensionsRequired\":[\"KHR_mesh_quantization\"]}");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(OpenJson(builder, &gltf).result == mani_success, "a harmless extension");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static maniGltf* OpenReading(maniBuilder* builder, const char* accessors)
{
    Document(DataUri(), kViews, accessors, "");
    maniGltf* gltf = NULL;
    maniImportResult r = OpenJson(builder, &gltf);
    CHECK(r.result == mani_success, "opens");
    if (r.result != mani_success)
    {
        printf("  reason %u offset %llu item %u: %s\n", r.reason, (unsigned long long)r.offset,
               r.item, s_json + (r.offset < strlen(s_json) ? r.offset : 0));
    }
    return gltf;
}

static void TestAccessors(void)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = OpenReading(
        builder,
        "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"},"
        "{\"bufferView\":0,\"byteOffset\":24,\"componentType\":5120,\"normalized\":true,"
        "\"count\":4,\"type\":\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":28,\"componentType\":5121,\"normalized\":true,"
        "\"count\":2,\"type\":\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":32,\"componentType\":5121,\"count\":1,\"type\":\"MAT2\"},"
        "{\"bufferView\":0,\"byteOffset\":40,\"componentType\":5122,\"normalized\":true,"
        "\"count\":2,\"type\":\"SCALAR\"},"
        "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"SCALAR\",\"sparse\":"
        "{\"count\":1,\"indices\":{\"bufferView\":0,\"byteOffset\":44,\"componentType\":5121},"
        "\"values\":{\"bufferView\":0,\"byteOffset\":48}}},"
        "{\"componentType\":5126,\"count\":2,\"type\":\"VEC2\"},"
        "{\"bufferView\":0,\"byteOffset\":60,\"componentType\":5126,\"count\":1,\"type\":"
        "\"SCALAR\"},"
        "{\"bufferView\":0,\"byteOffset\":40,\"componentType\":5123,\"normalized\":true,"
        "\"count\":2,\"type\":\"SCALAR\"},"
        "{\"bufferView\":0,\"componentType\":5126,\"count\":1,\"type\":\"SCALAR\",\"sparse\":"
        "{\"count\":1,\"indices\":{\"bufferView\":0,\"byteOffset\":44,\"componentType\":5121},"
        "\"values\":{\"bufferView\":0,\"byteOffset\":48}}}");
    if (gltf == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    float out[16];
    CHECK(maniGltfReadFloats(gltf, 0, out).result == mani_success, "floats");
    CHECK(out[0] == 1 && out[2] == 3 && out[3] == -4 && out[4] == 5.5f, "VEC3 floats");
    CHECK(maniGltfReadFloats(gltf, 1, out).result == mani_success, "snorm8");
    CHECK(out[0] == 1.0f && out[1] == -1.0f && out[2] == 0.0f && out[3] == 64.0f / 127.0f,
          "snorm8 as the table says, -128 is -1");
    CHECK(maniGltfReadFloats(gltf, 2, out).result == mani_success && out[0] == 1.0f &&
              out[1] == 0.0f,
          "unorm8");
    CHECK(maniGltfReadFloats(gltf, 3, out).result == mani_success, "MAT2 bytes");
    CHECK(out[0] == 1 && out[1] == 2 && out[2] == 3 && out[3] == 4, "padded columns");
    CHECK(maniGltfReadFloats(gltf, 4, out).result == mani_success && out[0] == 1.0f &&
              out[1] == -1.0f,
          "snorm16");
    CHECK(maniGltfReadFloats(gltf, 5, out).result == mani_success, "sparse");
    CHECK(out[0] == 1 && out[1] == 9 && out[2] == 3, "element 1 replaced");
    CHECK(maniGltfReadFloats(gltf, 6, out).result == mani_success && out[0] == 0 && out[3] == 0,
          "no view: zeros");
    CHECK(maniGltfReadFloats(gltf, 8, out).result == mani_success &&
              out[0] == 32767.0f / 65535.0f && out[1] == 32768.0f / 65535.0f,
          "unorm16");
    maniImportResult r = maniGltfReadFloats(gltf, 9, out);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData && r.offset == 44,
          "a sparse index past the count");
    r = maniGltfReadFloats(gltf, 7, out);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData && r.offset == 60 &&
              r.item == 7,
          "a NaN, at its buffer offset");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestStrideAndSparseOrder(void)
{
    maniBuilder* builder = NewBuilder();
    Document(DataUri(), "{\"buffer\":0,\"byteLength\":24,\"byteStride\":12}",
             "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC2\"}", "");
    maniGltf* gltf = NULL;
    CHECK(OpenJson(builder, &gltf).result == mani_success, "a strided view");
    float out[4];
    CHECK(maniGltfReadFloats(gltf, 0, out).result == mani_success && out[0] == 1 && out[1] == 2 &&
              out[2] == -4 && out[3] == 5.5f,
          "every twelfth byte");
    maniCloseGltf(gltf);
    // Sparse indices 1 then 1: not increasing.
    FillBuffer();
    s_buffer[45] = 1;
    char digits[200];
    Base64(s_buffer, sizeof s_buffer, digits);
    char uri[256];
    snprintf(uri, sizeof uri, "data:application/gltf-buffer;base64,%s", digits);
    Document(uri, kViews,
             "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"SCALAR\",\"sparse\":"
             "{\"count\":2,\"indices\":{\"bufferView\":0,\"byteOffset\":44,\"componentType\":5121},"
             "\"values\":{\"bufferView\":0,\"byteOffset\":48}}}",
             "");
    CHECK(OpenJson(builder, &gltf).result == mani_success, "opens");
    maniImportResult r = maniGltfReadFloats(gltf, 0, out);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData && r.offset == 45,
          "sparse indices not increasing");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestLayoutRefusals(void)
{
    const char* uri = DataUri();
    Document(uri, "{\"buffer\":0,\"byteOffset\":8,\"byteLength\":57}", "", "");
    ExpectRefusal(maniImportReasonLayout, "57", 0, "a view past its buffer");
    Document(uri, "{\"buffer\":1,\"byteLength\":8}", "", "");
    ExpectRefusal(maniImportReasonIndex, "1,", 0, "a buffer past the count");
    Document(uri, "{\"buffer\":0,\"byteLength\":8,\"byteStride\":6}", "", "");
    ExpectRefusal(maniImportReasonLayout, "6}", 0, "a stride not a multiple of 4");
    Document(uri, "{\"buffer\":0,\"byteLength\":8,\"byteStride\":2}", "", "");
    ExpectRefusal(maniImportReasonProperty, "2}", 0, "a stride below 4");
    Document(uri, kViews,
             "{\"bufferView\":0,\"byteOffset\":2,\"componentType\":5126,\"count\":1,"
             "\"type\":\"SCALAR\"}",
             "");
    ExpectRefusal(maniImportReasonLayout, "{\"bufferView\":0,\"byteOffset\":2", 0, "misaligned");
    Document(uri, kViews, "{\"bufferView\":0,\"componentType\":5126,\"count\":6,\"type\":\"VEC3\"}",
             "");
    ExpectRefusal(maniImportReasonLayout, "{\"bufferView\":0,\"componentType\"", 0,
                  "past its view");
    Document(uri, kViews,
             "{\"bufferView\":0,\"componentType\":5126,\"normalized\":true,"
             "\"count\":1,\"type\":\"SCALAR\"}",
             "");
    ExpectRefusal(maniImportReasonProperty, "5126", 0, "normalized floats");
    Document(uri, kViews,
             "{\"bufferView\":0,\"componentType\":5124,\"count\":1,\"type\":\"SCALAR\"}", "");
    ExpectRefusal(maniImportReasonProperty, "5124", 0, "a component type glTF lacks");
    Document(uri, kViews,
             "{\"bufferView\":0,\"componentType\":5126,\"normalized\":1,\"count\":1,"
             "\"type\":\"SCALAR\"}",
             "");
    ExpectRefusal(maniImportReasonProperty, "1,\"count", 0, "normalized not a boolean");
    Document(uri, "{\"buffer\":0,\"byteLength\":64,\"byteStride\":8}",
             "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}", "");
    ExpectRefusal(maniImportReasonLayout, "{\"bufferView\":0,\"componentType\"", 0,
                  "a stride below the element");
    Document(uri, kViews, "", "\"extensionsRequired\":[1],");
    ExpectRefusal(maniImportReasonProperty, "1]", 0, "an extension name not a string");
    Document(uri, kViews, "{\"bufferView\":0,\"componentType\":5126,\"count\":1,\"type\":\"VEC5\"}",
             "");
    ExpectRefusal(maniImportReasonProperty, "VEC5\"", 0, "a type glTF lacks");
    Document(uri, kViews, "{\"byteOffset\":0,\"componentType\":5126,\"count\":1,\"type\":\"VEC2\"}",
             "");
    ExpectRefusal(maniImportReasonProperty, "0,\"componentType", 0, "an offset without a view");
    Document(uri, kViews, "{\"bufferView\":0,\"componentType\":5126,\"count\":0,\"type\":\"VEC2\"}",
             "");
    ExpectRefusal(maniImportReasonProperty, "0,\"type", 0, "count 0");
    Document(uri, kViews, "{\"componentType\":5126,\"count\":16777217,\"type\":\"VEC2\"}", "");
    ExpectRefusal(maniImportReasonLayout, "16777217", 0, "too many implicit zeros");
    Document(uri, kViews, "{\"componentType\":5126,\"count\":16777216,\"type\":\"SCALAR\"}", "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(OpenJson(builder, &gltf).result == mani_success, "the most implicit zeros");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    Document(
        uri, "{\"buffer\":0,\"byteLength\":64,\"byteStride\":4}",
        "{\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\",\"sparse\":{\"count\":1,"
        "\"indices\":{\"bufferView\":0,\"componentType\":5121},\"values\":{\"bufferView\":0}}}",
        "");
    ExpectRefusal(maniImportReasonLayout, "{\"bufferView\":0,\"componentType\":5121", 0,
                  "sparse indices from a strided view");
    Document(
        uri, kViews,
        "{\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\",\"sparse\":{\"count\":3,"
        "\"indices\":{\"bufferView\":0,\"componentType\":5121},\"values\":{\"bufferView\":0}}}",
        "");
    ExpectRefusal(maniImportReasonProperty, "3,", 0, "more replacements than elements");
    Document(
        uri, kViews,
        "{\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\",\"sparse\":{\"count\":1,"
        "\"indices\":{\"bufferView\":0,\"componentType\":5122},\"values\":{\"bufferView\":0}}}",
        "");
    ExpectRefusal(maniImportReasonProperty, "5122", 0, "signed sparse indices");
}

// A data URI over the test buffer, its first base64 digit replaced, or
// one digit appended.
static void BadBase64(char replace, bool append)
{
    char uri[256];
    snprintf(uri, sizeof uri, "%s%s", DataUri(), append ? "A" : "");
    char* digits = strchr(uri, ',') + 1;
    digits[0] = replace != 0 ? replace : digits[0];
    Document(uri, "", "", "");
}

static void TestBufferRefusals(void)
{
    BadBase64('*', false);
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "a digit not base64, enough of them");
    BadBase64(0, true);
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "digits not in fours");
    // 64 bytes end in "==": one byte short of a buffer of 65.
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":65,\"uri\":\"%s\"}]}",
             DataUri());
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "padding counts no byte");
    {
        char uri[400] = "data:application/octet-stream,";
        for (int i = 0; i < 63; ++i)
        {
            strcat(uri, "%00");
        }
        strcat(uri, "%0G");
        Document(uri, "", "", "");
        ExpectRefusal(maniImportReasonBuffer, "data:", 0, "a percent escape's second digit");
    }
    Document("data:text/plain;base64,AAAA", "", "", "");
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "a media type glTF does not allow");
    Document("data:application/octet-stream;base64,AA*A", "", "", "");
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "not base64");
    Document("data:application/octet-stream;base64,AAAA", "", "", "");
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "fewer bytes than byteLength");
    Document("data:application/octet-stream", "", "", "");
    ExpectRefusal(maniImportReasonBuffer, "data:", 0, "no comma");
    // Percent-encoded data: 64 bytes, "%00" each.
    char uri[400] = "data:application/octet-stream,";
    for (int i = 0; i < 64; ++i)
    {
        strcat(uri, "%00");
    }
    Document(uri, "", "", "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(OpenJson(builder, &gltf).result == mani_success, "percent-encoded data");
    CHECK(maniGltfBufferUri(gltf, 0, NULL) == NULL, "held, no URI");
    maniCloseGltf(gltf);
    // The URI's scheme and media type compare without case.
    char upper[256];
    snprintf(upper, sizeof upper, "DATA:Application/Octet-Stream;BASE64,%s", DataUri() + 37);
    Document(upper, "", "", "");
    CHECK(OpenJson(builder, &gltf).result == mani_success &&
              maniGltfBufferUri(gltf, 0, NULL) == NULL,
          "case folded: data the document holds");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestExternalBuffer(void)
{
    Document("my%20data/\\u00e9.bin", kViews,
             "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}", "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(OpenJson(builder, &gltf).result == mani_success, "opens");
    uint32_t length = 0;
    const char* uri = maniGltfBufferUri(gltf, 0, &length);
    CHECK(uri != NULL && length == 16 && memcmp(uri, "my%20data/\xC3\xA9.bin", 16) == 0,
          "the URI, JSON escapes decoded, percent escapes left");
    CHECK(maniGltfBufferUri(gltf, 1, &length) == NULL && length == 0, "past the count");
    float out[6];
    maniImportResult r = maniGltfReadFloats(gltf, 0, out);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonPending, "not yet given");
    FillBuffer();
    r = maniGltfSupplyBuffer(gltf, 0, s_buffer, 63);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBuffer && r.offset == 63,
          "too few bytes");
    CHECK(maniGltfSupplyBuffer(gltf, 0, NULL, 64).result == mani_errorInvalid, "null bytes");
    CHECK(maniGltfSupplyBuffer(gltf, 1, s_buffer, 64).result == mani_errorInvalid, "past");
    CHECK(maniGltfSupplyBuffer(gltf, 0, s_buffer, 64).result == mani_success, "given");
    CHECK(maniGltfReadFloats(gltf, 0, out).result == mani_success && out[5] == 6, "read");
    maniCloseGltf(gltf);
    // A held buffer takes no bytes from the host.
    Document(DataUri(), "", "", "");
    CHECK(OpenJson(builder, &gltf).result == mani_success, "opens");
    CHECK(maniGltfSupplyBuffer(gltf, 0, s_buffer, 64).result == mani_errorInvalid, "held");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// A GLB of the document in s_json and the test buffer.
static void Glb(bool withBin)
{
    size_t jsonSize = strlen(s_json);
    size_t jsonPadded = (jsonSize + 3) / 4 * 4;
    size_t total = 12 + 8 + jsonPadded + (withBin ? 8 + sizeof s_buffer : 0);
    maniStoreU32(s_glb, 0x46546C67u);
    maniStoreU32(s_glb + 4, 2);
    maniStoreU32(s_glb + 8, (uint32_t)total);
    maniStoreU32(s_glb + 12, (uint32_t)jsonPadded);
    maniStoreU32(s_glb + 16, 0x4E4F534Au);
    memcpy(s_glb + 20, s_json, jsonSize);
    memset(s_glb + 20 + jsonSize, ' ', jsonPadded - jsonSize);
    if (withBin)
    {
        FillBuffer();
        maniStoreU32(s_glb + 20 + jsonPadded, sizeof s_buffer);
        maniStoreU32(s_glb + 24 + jsonPadded, 0x004E4942u);
        memcpy(s_glb + 28 + jsonPadded, s_buffer, sizeof s_buffer);
    }
    s_glbSize = total;
}

static maniImportResult OpenGlb(maniBuilder* builder, maniGltf** gltf)
{
    return maniOpenGltf(builder, s_glb, s_glbSize, gltf);
}

static void TestGlb(void)
{
    Document("", kViews, "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}",
             "");
    Glb(true);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(OpenGlb(builder, &gltf).result == mani_success, "a GLB");
    float out[6];
    CHECK(maniGltfReadFloats(gltf, 0, out).result == mani_success && out[3] == -4, "BIN data");
    maniCloseGltf(gltf);
    // Bytes past the GLB's length are not its.
    s_glbSize += 7;
    CHECK(OpenGlb(builder, &gltf).result == mani_success, "trailing bytes ignored");
    maniCloseGltf(gltf);
    Glb(true);
    size_t jsonEnd = 20 + maniLoadU32(s_glb + 12);
    // An unknown chunk after BIN is skipped.
    maniStoreU32(s_glb + s_glbSize, 4);
    maniStoreU32(s_glb + s_glbSize + 4, 0x12345678u);
    s_glbSize += 12;
    maniStoreU32(s_glb + 8, (uint32_t)s_glbSize);
    CHECK(OpenGlb(builder, &gltf).result == mani_success, "an unknown chunk");
    maniCloseGltf(gltf);
    struct
    {
        uint64_t at;
        uint32_t value;
        uint64_t offset;
        const char* what;
    } kBad[] = {
        {4, 1, 4, "GLB version 1"},
        {8, 4096, 8, "a length past the bytes"},
        {8, 18, 8, "a length below a chunk"},
        {12, 5, 12, "a JSON chunk not 4-aligned"},
        {16, 0x004E4942u, 16, "BIN first"},
        {jsonEnd + 4, 0x4E4F534Au, jsonEnd + 4, "JSON twice"},
        {jsonEnd, 4096, jsonEnd, "a chunk past the length"},
    };
    for (size_t i = 0; i < sizeof kBad / sizeof kBad[0]; ++i)
    {
        Glb(true);
        maniStoreU32(s_glb + kBad[i].at, kBad[i].value);
        maniImportResult r = OpenGlb(builder, &gltf);
        CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonContainer &&
                  r.offset == kBad[i].offset,
              kBad[i].what);
    }
    // BIN third, after an unknown chunk.
    Glb(false);
    maniStoreU32(s_glb + s_glbSize, 4);
    maniStoreU32(s_glb + s_glbSize + 4, 0x12345678u);
    maniStoreU32(s_glb + s_glbSize + 12, 4);
    maniStoreU32(s_glb + s_glbSize + 16, 0x004E4942u);
    uint64_t third = s_glbSize + 12;
    s_glbSize += 24;
    maniStoreU32(s_glb + 8, (uint32_t)s_glbSize);
    maniImportResult r = OpenGlb(builder, &gltf);
    CHECK(r.result == mani_errorFormat && r.offset == third + 4, "BIN third");
    // A first buffer without a URI needs the BIN chunk.
    Glb(false);
    r = OpenGlb(builder, &gltf);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBuffer, "no BIN");
    // A BIN chunk shorter than the first buffer.
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":68}]}");
    Glb(true);
    r = OpenGlb(builder, &gltf);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonBuffer, "a short BIN");
    // Only the first buffer is BIN's: a second without a URI is the host's.
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":64},"
             "{\"byteLength\":8}]}");
    Glb(true);
    CHECK(OpenGlb(builder, &gltf).result == mani_success, "opens");
    CHECK(maniGltfSupplyBuffer(gltf, 1, s_buffer, 8).result == mani_success,
          "the second buffer supplied");
    maniCloseGltf(gltf);
    // A total length not a multiple of 4, its bytes there.
    Document("", kViews, "", "");
    Glb(true);
    s_glb[s_glbSize] = 0;
    s_glbSize += 1;
    maniStoreU32(s_glb + 8, (uint32_t)s_glbSize);
    r = OpenGlb(builder, &gltf);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonContainer && r.offset == 8,
          "a length not 4-aligned");
    // JSON offsets count from the GLB's start.
    snprintf(s_json, sizeof s_json, "{\"asset\":{\"version\":\"3.0\"}}");
    Glb(false);
    r = OpenGlb(builder, &gltf);
    CHECK(r.reason == maniImportReasonVersion && r.offset == 20 + 21, "an offset in the GLB");
    maniDestroyBuilder(builder);
}

static void TestPrefixes(void)
{
    Document("", kViews, "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"}",
             "");
    Glb(true);
    maniBuilder* builder = NewBuilder();
    size_t full = s_glbSize;
    for (size_t n = 0; n < full; ++n)
    {
        maniGltf* gltf = NULL;
        maniImportResult r = maniOpenGltf(builder, s_glb, n, &gltf);
        CHECK(r.result == mani_errorFormat && gltf == NULL, "a GLB's proper prefix");
    }
    maniDestroyBuilder(builder);
}

static int WriteSeed(const char* path)
{
    Document("", kViews,
             "{\"bufferView\":0,\"componentType\":5126,\"count\":2,\"type\":\"VEC3\"},"
             "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"SCALAR\",\"sparse\":"
             "{\"count\":1,\"indices\":{\"bufferView\":0,\"byteOffset\":44,\"componentType\":5121},"
             "\"values\":{\"bufferView\":0,\"byteOffset\":48}}}",
             "");
    Glb(true);
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(s_glb, 1, s_glbSize, out);
    return fclose(out) == 0 && written == s_glbSize ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestMinimal();
    TestVersionsAndExtensions();
    TestAccessors();
    TestStrideAndSparseOrder();
    TestLayoutRefusals();
    TestBufferRefusals();
    TestExternalBuffer();
    TestGlb();
    TestPrefixes();
    return s_failures == 0 ? 0 : 1;
}
