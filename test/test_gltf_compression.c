// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF's meshopt compression in the importer: both
// extensions' objects on buffer views and buffers and every rule of
// theirs refused at its member, streams decoded at open and when an
// external buffer is supplied, fallback and placeholder buffers never
// read, allocation failures; and three files gltfpack compressed
// (test/data/gltf), their decoded views the bits meshoptimizer's own
// decoder gives (tools/meshopt_files.py), one's equal to
// its uncompressed fallback, their skeletons, skins and clips built. Run
// as test_gltf_compression --seed <file> it writes a document whose view
// claims two gigabytes from a stream of 259 bytes, a seed of fuzz_gltf.

// MSVC's CRT deprecates fopen, which the fixtures use: CMake defines
// _CRT_SECURE_NO_WARNINGS for the fixture tests.

#include "gltf_document.h"
#include "gltf_layout.h"
#include "gltf_meshopt.h"
#include "gltf_meshopt_fixtures.h"
#include "test_budget.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MANI_TEST_DATA
#error "MANI_TEST_DATA names test/data"
#endif

static char s_json[16384];
static char s_digits[8192];
static uint8_t s_stream[4096];
static float s_floats[4096];

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

// The version 0 attribute stream: 37 elements of 8 bytes.
static const char kStream[] =
    "\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"ATTRIBUTES\"";
static const char kFallback[] =
    "{\"byteLength\":296,\"extensions\":{\"EXT_meshopt_compression\":{\"fallback\":true}}}";

// A document: buffer 0 the stream's bytes (`uri`: a data: URI of them
// when empty), buffer 1 `second`; view 0 of `length` bytes on buffer 1
// with the stream object `members` under extension `name`, then `views`;
// an accessor of unsigned short VEC4 over view 0.
typedef struct Doc
{
    const char* name;
    const char* members;
    const char* second;
    const char* views;
    uint32_t length;
    const uint8_t* stream;
    size_t size;
    const char* uri;
} Doc;

static void Document(Doc d)
{
    Base64(d.stream, d.size, s_digits);
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"extensionsRequired\":[\"%s\"],"
             "\"buffers\":[{\"byteLength\":%zu,\"uri\":\"%s%s\"},%s],"
             "\"bufferViews\":[{\"buffer\":1,\"byteLength\":%u,\"extensions\":{\"%s\":{%s}}}%s],"
             "\"accessors\":[{\"bufferView\":0,\"componentType\":5123,\"type\":\"VEC4\","
             "\"count\":%u}]}",
             d.name, d.size, d.uri[0] != '\0' ? "" : "data:application/octet-stream;base64,",
             d.uri[0] != '\0' ? d.uri : s_digits, d.second, d.length, d.name, d.members, d.views,
             d.length / 8);
}

// The version 0 stream under EXT in a view of 296 bytes.
static void Plain(const char* members, const char* second, const char* views)
{
    Document((Doc){"EXT_meshopt_compression", members, second, views, 296, kAttributesV0,
                   sizeof kAttributesV0, ""});
}

// The version 1 stream (257 elements of 12 bytes) under KHR with a
// filter.
static void Twelve(const char* filter)
{
    char members[160];
    snprintf(members, sizeof members,
             "\"buffer\":0,\"byteLength\":1031,\"byteStride\":12,\"count\":257,"
             "\"mode\":\"ATTRIBUTES\",\"filter\":\"%s\"",
             filter);
    Document((Doc){"KHR_meshopt_compression", members, "{\"byteLength\":3084}", "", 3084,
                   kAttributesV1, sizeof kAttributesV1, ""});
}

// A view of two gigabytes claimed by the 259-byte stream.
static void HugeView(void)
{
    Document((Doc){"EXT_meshopt_compression",
                   "\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":250000000,"
                   "\"mode\":\"ATTRIBUTES\"",
                   "{\"byteLength\":2000000000}", "", 2000000000u, kAttributesV0,
                   sizeof kAttributesV0, ""});
}

static maniImportResult Open(maniBuilder* builder, maniGltf** gltf)
{
    return maniOpenGltf(builder, s_json, strlen(s_json), gltf);
}

// A refusal with its reason, at the first byte of `at` in the JSON (or
// in the buffer, for Data, at `offset`), for the item.
static void ExpectRefusal(maniImportReason reason, const char* at, uint64_t offset, uint32_t item,
                          const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniImportResult r = Open(builder, &gltf);
    const char* found = at != NULL ? strstr(s_json, at) : NULL;
    uint64_t want = found != NULL ? (uint64_t)(found - s_json) : offset;
    bool right = r.result == mani_errorFormat && r.reason == reason && r.item == item &&
                 r.offset == want && gltf == NULL && (at == NULL || found != NULL);
    CHECK(right, what);
    if (!right)
    {
        printf("  result %d reason %u offset %llu (want %llu) item %u\n", (int)r.result, r.reason,
               (unsigned long long)r.offset, (unsigned long long)want, r.item);
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// View 0 decoded to the expected bytes, read through its accessor.
static bool Decoded(const maniGltf* gltf, const uint8_t* expected, size_t size)
{
    const maniGltfView* v = &gltf->views[0];
    return v->decoded != NULL && v->length == size && memcmp(v->decoded, expected, size) == 0;
}

static bool ReadsAsShorts(const maniGltf* gltf)
{
    if (maniGltfReadFloats(gltf, 0, s_floats).result != mani_success)
    {
        return false;
    }
    for (uint32_t i = 0; i < 37 * 4; ++i)
    {
        uint32_t value = (uint32_t)kAttributesV0Out[2 * i] | (uint32_t)kAttributesV0Out[2 * i + 1]
                                                                 << 8;
        if (s_floats[i] != (float)value)
        {
            return false;
        }
    }
    return true;
}

static void TestDecodedAtOpen(void)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    Plain(kStream, kFallback, "");
    CHECK(Open(builder, &gltf).result == mani_success, "opens");
    CHECK(gltf != NULL && Decoded(gltf, kAttributesV0Out, sizeof kAttributesV0Out),
          "decoded at open");
    CHECK(gltf != NULL && ReadsAsShorts(gltf), "the accessor reads the decoded bytes");
    CHECK(gltf != NULL && maniGltf_GetBufferUri(gltf, 1, NULL) == NULL,
          "a fallback is not external");
    CHECK(gltf != NULL && maniGltf_SupplyBuffer(gltf, 1, s_stream, 296).result == mani_errorInvalid,
          "nor supplied");
    maniCloseGltf(gltf);
    // A fallback with a URI is never read either.
    Plain(kStream,
          "{\"byteLength\":296,\"uri\":\"fallback.bin\",\"extensions\":{"
          "\"EXT_meshopt_compression\":{\"fallback\":true}}}",
          "");
    CHECK(Open(builder, &gltf).result == mani_success &&
              maniGltf_GetBufferUri(gltf, 1, NULL) == NULL,
          "a fallback's URI is left alone");
    maniCloseGltf(gltf);
    // A buffer without a URI that only compressed views use is a
    // placeholder; one a plain view uses stays the host's.
    Plain(kStream, "{\"byteLength\":296}", "");
    CHECK(Open(builder, &gltf).result == mani_success &&
              maniGltf_GetBufferUri(gltf, 1, NULL) == NULL &&
              maniGltf_SupplyBuffer(gltf, 1, s_stream, 296).result == mani_errorInvalid,
          "a placeholder");
    maniCloseGltf(gltf);
    Plain(kStream, "{\"byteLength\":296}", ",{\"buffer\":1,\"byteLength\":8}");
    size_t length = 1;
    CHECK(Open(builder, &gltf).result == mani_success &&
              maniGltf_GetBufferUri(gltf, 1, &length) != NULL && length == 0,
          "a buffer a plain view uses is external");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

// Every fixture stream through the importer, under both extensions.
static void TestModes(void)
{
    static const char* const kModes[3] = {"ATTRIBUTES", "TRIANGLES", "INDICES"};
    maniBuilder* builder = NewBuilder();
    for (size_t f = 0; f < sizeof kMeshoptFixtures / sizeof kMeshoptFixtures[0]; ++f)
    {
        const MeshoptFixture* x = &kMeshoptFixtures[f];
        uint32_t length = x->count * x->stride;
        for (int khr = 0; khr < 2; ++khr)
        {
            const char* name = khr ? "KHR_meshopt_compression" : "EXT_meshopt_compression";
            char members[256];
            snprintf(members, sizeof members,
                     "\"buffer\":0,\"byteLength\":%zu,\"byteStride\":%u,\"count\":%u,"
                     "\"mode\":\"%s\"",
                     x->size, x->stride, x->count, kModes[x->mode]);
            char second[64];
            snprintf(second, sizeof second, "{\"byteLength\":%u}", length);
            Document((Doc){name, members, second, "", length, x->stream, x->size, ""});
            maniGltf* gltf = NULL;
            maniImportResult r = Open(builder, &gltf);
            if (x->v1 && !khr)
            {
                CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData &&
                          r.offset == 0 && r.item == 0,
                      "version 1 is KHR's");
            }
            else
            {
                CHECK(r.result == mani_success && Decoded(gltf, x->out, length), "a mode decoded");
            }
            maniCloseGltf(gltf);
        }
    }
    maniDestroyBuilder(builder);
}

static void TestRules(void)
{
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":6,\"count\":37,\"mode\":\"ATTRIBUTES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "6,\"count", 0, 0, "an attribute stride");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"TRIANGLES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "8,\"count", 0, 0, "a triangle stride");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":2,\"count\":148,\"mode\":\"TRIANGLES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "148", 0, 0, "triangles of a count not of threes");
    // The strides again, on views their counts fill.
    Document((Doc){"EXT_meshopt_compression",
                   "\"buffer\":0,\"byteLength\":259,\"byteStride\":6,\"count\":48,"
                   "\"mode\":\"ATTRIBUTES\"",
                   "{\"byteLength\":288}", "", 288, kAttributesV0, sizeof kAttributesV0, ""});
    ExpectRefusal(maniImportReasonLayout, "6,\"count", 0, 0, "an attribute stride, the view full");
    Document((Doc){"EXT_meshopt_compression",
                   "\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":36,"
                   "\"mode\":\"TRIANGLES\"",
                   "{\"byteLength\":288}", "", 288, kAttributesV0, sizeof kAttributesV0, ""});
    ExpectRefusal(maniImportReasonLayout, "8,\"count", 0, 0, "a triangle stride, the view full");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":2,\"count\":148,\"mode\":\"INDICES\","
          "\"filter\":\"OCTAHEDRAL\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "OCTAHEDRAL\"", 0, 0, "a filter on indices");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"ATTRIBUTES\","
          "\"filter\":\"COLOR\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonProperty, "COLOR\"", 0, 0, "COLOR is KHR's");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"ATTRIBUTES\","
          "\"filter\":\"EXPONENTIAL\"",
          kFallback, "");
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(Open(builder, &gltf).result == mani_success, "an exponential filter on 8 bytes");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":4,\"count\":74,\"mode\":\"ATTRIBUTES\","
          "\"filter\":\"QUATERNION\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "QUATERNION\"", 0, 0, "a quaternion of 4 bytes");
    // On 12 bytes an element: octahedral and color are for 4 or 8, a
    // quaternion for 8; exponential is for any.
    static const char* const kNarrow[3] = {"OCTAHEDRAL", "QUATERNION", "COLOR"};
    for (int k = 0; k < 3; ++k)
    {
        Twelve(kNarrow[k]);
        char at[16];
        snprintf(at, sizeof at, "%s\"", kNarrow[k]);
        ExpectRefusal(maniImportReasonLayout, at, 0, 0, "a filter on 12 bytes");
    }
    Twelve("EXPONENTIAL");
    builder = NewBuilder();
    CHECK(Open(builder, &gltf).result == mani_success, "an exponential filter on 12 bytes");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":36,\"mode\":\"ATTRIBUTES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "36", 0, 0, "a count short of the view");
    Plain("\"buffer\":0,\"byteOffset\":4,\"byteLength\":259,\"byteStride\":8,\"count\":37,"
          "\"mode\":\"ATTRIBUTES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "259,\"byteStride", 0, 0, "a stream past its buffer");
    Plain("\"buffer\":1,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"ATTRIBUTES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "1,\"byteLength\":259", 0, 0, "a stream in a fallback");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"VERTICES\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonProperty, "VERTICES\"", 0, 0, "an unknown mode");
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"mode\":\"ATTRIBUTES\"", kFallback,
          "");
    ExpectRefusal(maniImportReasonProperty, "{\"buffer\":0,\"byteLength\":259", 0, 0, "no count");
}

// The fallback marks, and a view longer than its stream can decode.
static void TestMarks(void)
{
    Plain(kStream, kFallback, ",{\"buffer\":1,\"byteLength\":8}");
    ExpectRefusal(maniImportReasonLayout, "1,\"byteLength\":8}", 0, 1,
                  "a plain view on a fallback");
    Plain(kStream,
          "{\"byteLength\":296,\"extensions\":{\"KHR_meshopt_compression\":{\"fallback\":true}}}",
          "");
    ExpectRefusal(maniImportReasonLayout, "1,\"byteLength\":296", 0, 0,
                  "an EXT view on a KHR fallback");
    Plain(kStream,
          "{\"byteLength\":296,\"extensions\":{\"EXT_meshopt_compression\":{\"fallback\":true},"
          "\"KHR_meshopt_compression\":{\"fallback\":true}}}",
          "");
    ExpectRefusal(maniImportReasonProperty, "{\"fallback\":true}}}", 0, 1,
                  "both marks on a buffer");
    // The view's length beyond what a stream of 259 bytes can decode.
    char second[64];
    snprintf(second, sizeof second, "{\"byteLength\":%u}", 1024u * 259u + 8u);
    Document((Doc){"EXT_meshopt_compression",
                   "\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":33153,"
                   "\"mode\":\"ATTRIBUTES\"",
                   second, "", 1024u * 259u + 8u, kAttributesV0, sizeof kAttributesV0, ""});
    ExpectRefusal(maniImportReasonLayout, "259,\"byteStride\":8,\"count\":33153", 0, 0,
                  "more than a stream can hold");
    HugeView();
    ExpectRefusal(maniImportReasonLayout, "259,\"byteStride\":8,\"count\":250000000", 0, 0,
                  "two gigabytes from 259 bytes");
}

// The edges of the rules: what they allow is opened.
static void TestEdges(void)
{
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":37,\"mode\":\"ATTRIBUTES\","
          "\"filter\":\"OCTAHEDRAL\"",
          kFallback, "");
    CHECK(Open(builder, &gltf).result == mani_success, "an octahedral filter on 8 bytes");
    maniCloseGltf(gltf);
    Plain("\"buffer\":0,\"byteLength\":259,\"byteStride\":2,\"count\":148,\"mode\":\"INDICES\","
          "\"filter\":\"EXPONENTIAL\"",
          kFallback, "");
    ExpectRefusal(maniImportReasonLayout, "EXPONENTIAL\"", 0, 0,
                  "an exponential filter on indices");
    // Exactly 1,024 times the stream passes the rules; the stream itself
    // then holds too few elements.
    char second[64];
    snprintf(second, sizeof second, "{\"byteLength\":%u}", 1024u * 259u);
    Document((Doc){"EXT_meshopt_compression",
                   "\"buffer\":0,\"byteLength\":259,\"byteStride\":8,\"count\":33152,"
                   "\"mode\":\"ATTRIBUTES\"",
                   second, "", 1024u * 259u, kAttributesV0, sizeof kAttributesV0, ""});
    ExpectRefusal(maniImportReasonData, NULL, 0, 0, "1,024 times the stream is allowed");
    // A buffer without a URI that holds a stream stays the host's, even
    // when only its compressed view uses it.
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":296}],"
             "\"bufferViews\":[{\"buffer\":0,\"byteLength\":296,\"extensions\":{"
             "\"EXT_meshopt_compression\":{%s}}}]}",
             kStream);
    size_t length = 1;
    CHECK(Open(builder, &gltf).result == mani_success &&
              maniGltf_GetBufferUri(gltf, 0, &length) != NULL && length == 0,
          "a stream's buffer is the host's");
    memset(s_stream, 0, 296);
    memcpy(s_stream, kAttributesV0, sizeof kAttributesV0);
    CHECK(gltf != NULL && maniGltf_SupplyBuffer(gltf, 0, s_stream, 296).result == mani_success &&
              Decoded(gltf, kAttributesV0Out, sizeof kAttributesV0Out),
          "and decoded when supplied");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static void TestBothOnAView(void)
{
    snprintf(s_json, sizeof s_json,
             "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":259,\"uri\":\"a\"},"
             "{\"byteLength\":296}],\"bufferViews\":[{\"buffer\":1,\"byteLength\":296,"
             "\"extensions\":{\"EXT_meshopt_compression\":{%s},\"KHR_meshopt_compression\":{%s}}}"
             "]}",
             kStream, kStream);
    ExpectRefusal(maniImportReasonProperty,
                  "{\"buffer\":0,\"byteLength\":259,\"byteStride\":8,"
                  "\"count\":37,\"mode\":\"ATTRIBUTES\"}}}",
                  0, 0, "both extensions on a view");
}

static void TestBadStream(void)
{
    memcpy(s_stream, kAttributesV0, sizeof kAttributesV0);
    s_stream[0] = 0xa2;
    Document((Doc){"EXT_meshopt_compression", kStream, kFallback, "", 296, s_stream,
                   sizeof kAttributesV0, ""});
    ExpectRefusal(maniImportReasonData, NULL, 0, 0, "a stream's header");
    Document((Doc){"EXT_meshopt_compression",
                   "\"buffer\":0,\"byteOffset\":3,\"byteLength\":256,\"byteStride\":8,"
                   "\"count\":37,\"mode\":\"ATTRIBUTES\"",
                   kFallback, "", 296, kAttributesV0, sizeof kAttributesV0, ""});
    ExpectRefusal(maniImportReasonData, NULL, 3, 0, "the offset in the buffer");
}

// An external stream: pending until supplied, a bad one refused and the
// buffer pending again, a good one decoded, supplied again.
static void TestSupplied(void)
{
    Document((Doc){"EXT_meshopt_compression", kStream, kFallback, "", 296, kAttributesV0,
                   sizeof kAttributesV0, "stream.bin"});
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(Open(builder, &gltf).result == mani_success, "opens");
    if (gltf == NULL)
    {
        maniDestroyBuilder(builder);
        return;
    }
    size_t length = 0;
    const char* uri = maniGltf_GetBufferUri(gltf, 0, &length);
    CHECK(uri != NULL && length == 10 && memcmp(uri, "stream.bin", 10) == 0, "the stream's URI");
    CHECK(maniGltfReadFloats(gltf, 0, s_floats).reason == maniImportReasonPending, "pending");
    memcpy(s_stream, kAttributesV0, sizeof kAttributesV0);
    s_stream[0] = 0xa2;
    maniImportResult r = maniGltf_SupplyBuffer(gltf, 0, s_stream, sizeof kAttributesV0);
    CHECK(r.result == mani_errorFormat && r.reason == maniImportReasonData && r.item == 0,
          "a bad stream refused");
    CHECK(maniGltfReadFloats(gltf, 0, s_floats).reason == maniImportReasonPending,
          "and the buffer pending again");
    CHECK(maniGltf_SupplyBuffer(gltf, 0, kAttributesV0, sizeof kAttributesV0).result ==
                  mani_success &&
              ReadsAsShorts(gltf),
          "a good one decoded");
    CHECK(maniGltf_SupplyBuffer(gltf, 0, kAttributesV0, sizeof kAttributesV0).result ==
                  mani_success &&
              Decoded(gltf, kAttributesV0Out, sizeof kAttributesV0Out),
          "and supplied again");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
}

static TestOutcome OpenWork(maniBuilder* builder, void* context)
{
    (void)context;
    maniGltf* gltf = NULL;
    TestOutcome outcome = TestOutcomeOf(Open(builder, &gltf).result);
    if (outcome == testDone && !Decoded(gltf, kAttributesV0Out, sizeof kAttributesV0Out))
    {
        outcome = testWrong;
    }
    maniCloseGltf(gltf);
    return outcome;
}

static void TestAllocationFailures(void)
{
    Plain(kStream, kFallback, "");
    CHECK(TestSweep(OpenWork, NULL, 4096), "allocation failures handled, nothing kept");
}

// A file of test/data/gltf, or NULL; the caller frees it.
static uint8_t* ReadFile(const char* name, size_t* sizeOut)
{
    char path[512];
    snprintf(path, sizeof path, "%s/gltf/%s", MANI_TEST_DATA, name);
    FILE* f = fopen(path, "rb");
    uint8_t* bytes = NULL;
    long size = -1;
    if (f != NULL && fseek(f, 0, SEEK_END) == 0)
    {
        size = ftell(f);
    }
    if (size > 0 && fseek(f, 0, SEEK_SET) == 0)
    {
        bytes = malloc((size_t)size);
    }
    if (bytes != NULL && fread(bytes, 1, (size_t)size, f) != (size_t)size)
    {
        free(bytes);
        bytes = NULL;
    }
    if (f != NULL)
    {
        fclose(f);
    }
    *sizeOut = bytes != NULL ? (size_t)size : 0;
    CHECK(bytes != NULL, name);
    return bytes;
}

// FNV-1a 64 of every decoded view in view order.
static uint64_t DecodedHash(const maniGltf* gltf)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (uint32_t i = 0; i < gltf->viewCount; ++i)
    {
        const maniGltfView* v = &gltf->views[i];
        for (uint64_t k = 0; v->decoded != NULL && k < v->length; ++k)
        {
            h = (h ^ v->decoded[k]) * 0x100000001b3ull;
        }
    }
    return h;
}

typedef struct PackedFile
{
    const char* name;
    // meshoptimizer's decoding of the file (meshopt_files.py).
    uint64_t hash;
    uint32_t compressed;
} PackedFile;

static const PackedFile kPackedFiles[] = {
    {"InterpolationTest-meshopt.glb", 0x10dfa13619e463f9ull, 8},
    {"InterpolationTest-khr.glb", 0x0793822888a5d368ull, 8},
    {"SimpleSkin-khr.glb", 0x59b7c367ca0b1382ull, 7},
};

static void TestPackedFiles(void)
{
    for (size_t f = 0; f < sizeof kPackedFiles / sizeof kPackedFiles[0]; ++f)
    {
        size_t size = 0;
        uint8_t* bytes = ReadFile(kPackedFiles[f].name, &size);
        maniBuilder* builder = NewBuilder();
        maniGltf* gltf = NULL;
        CHECK(bytes != NULL && maniOpenGltf(builder, bytes, size, &gltf).result == mani_success,
              kPackedFiles[f].name);
        uint32_t compressed = 0;
        for (uint32_t i = 0; gltf != NULL && i < gltf->viewCount; ++i)
        {
            compressed += gltf->views[i].decoded != NULL ? 1u : 0u;
        }
        CHECK(compressed == kPackedFiles[f].compressed, "every compressed view decoded");
        CHECK(gltf != NULL && DecodedHash(gltf) == kPackedFiles[f].hash,
              "the bits meshoptimizer decodes");
        if (gltf != NULL && DecodedHash(gltf) != kPackedFiles[f].hash)
        {
            printf("  %s: 0x%016llx\n", kPackedFiles[f].name,
                   (unsigned long long)DecodedHash(gltf));
        }
        maniCloseGltf(gltf);
        maniDestroyBuilder(builder);
        free(bytes);
    }
}

// A triangle view's indices equal the fallback's up to each triangle's
// rotation (the encoder may rotate a triangle, keeping its winding).
static bool SameTriangles(const maniGltfView* v, const uint8_t* fallback)
{
    uint32_t s = v->elementStride;
    for (uint32_t t = 0; t < v->count; t += 3)
    {
        uint32_t x[3];
        uint32_t y[3];
        for (uint32_t k = 0; k < 3; ++k)
        {
            const uint8_t* p = v->decoded + (size_t)(t + k) * s;
            const uint8_t* q = fallback + (size_t)(t + k) * s;
            x[k] = s == 2 ? (uint32_t)(p[0] | p[1] << 8)
                          : (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
                                (uint32_t)p[3] << 24;
            y[k] = s == 2 ? (uint32_t)(q[0] | q[1] << 8)
                          : (uint32_t)q[0] | (uint32_t)q[1] << 8 | (uint32_t)q[2] << 16 |
                                (uint32_t)q[3] << 24;
        }
        bool same = false;
        for (uint32_t r = 0; r < 3; ++r)
        {
            same = same || (x[0] == y[r] && x[1] == y[(r + 1) % 3] && x[2] == y[(r + 2) % 3]);
        }
        if (!same)
        {
            return false;
        }
    }
    return true;
}

// gltfpack's -cf file holds its views uncompressed in a fallback file
// beside it, which the importer never reads: equal to the decoding.
static void TestFallbackEqual(void)
{
    size_t size = 0;
    size_t fallbackSize = 0;
    uint8_t* bytes = ReadFile("InterpolationTest-meshopt.glb", &size);
    uint8_t* fallback = ReadFile("InterpolationTest-meshopt.fallback.bin", &fallbackSize);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(bytes != NULL && maniOpenGltf(builder, bytes, size, &gltf).result == mani_success,
          "opens");
    bool equal = gltf != NULL && fallback != NULL;
    for (uint32_t i = 0; equal && i < gltf->viewCount; ++i)
    {
        const maniGltfView* v = &gltf->views[i];
        bool inside = v->offset + v->length <= fallbackSize;
        equal =
            v->decoded == NULL ||
            (inside && v->mode == maniMeshoptTriangles && SameTriangles(v, fallback + v->offset)) ||
            (inside && memcmp(v->decoded, fallback + v->offset, (size_t)v->length) == 0);
    }
    CHECK(equal, "the decoded views equal the fallback");
    CHECK(gltf != NULL && maniGltf_GetBufferUri(gltf, 1, NULL) == NULL,
          "the fallback is not asked for");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    free(fallback);
    free(bytes);
}

// A skeleton's and a clip's files, one per document compared.
typedef struct Built
{
    _Alignas(16) uint8_t skeletonFile[16384];
    _Alignas(16) uint8_t clipFile[65536];
    maniSkeleton skeleton;
    maniClip clip;
} Built;

static Built s_built[2];
static _Alignas(32) uint8_t s_poseMemory[16384];

// The animation's clip (rate 30) over the node's skeleton; false when
// anything is refused.
static bool BuildClip(maniBuilder* builder, const maniGltf* gltf, uint32_t node, uint32_t animation,
                      Built* b)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    size_t size = 0;
    if (maniBuildGltfNodeSkeleton(builder, gltf, node, &options).result != mani_success ||
        maniBuilder_CopyBuilt(builder, b->skeletonFile, sizeof b->skeletonFile, &size) !=
            mani_success ||
        maniOpenSkeleton(b->skeletonFile, size, &b->skeleton).result != mani_success)
    {
        return false;
    }
    maniImportClipOptions clipOptions = {.sampleRate = 30.0f, .loop = maniLoopClamp};
    return maniBuildGltfClip(builder, gltf, animation, &b->skeleton, &options, &clipOptions, NULL)
                   .result == mani_success &&
           maniBuilder_CopyBuilt(builder, b->clipFile, sizeof b->clipFile, &size) == mani_success &&
           maniOpenClip(b->clipFile, size, &b->clip).result == mani_success;
}

// The node of this name, or UINT32_MAX.
static uint32_t FindNode(const maniGltf* gltf, const char* name, uint32_t length)
{
    for (uint32_t n = 0; n < maniGltf_GetNodeCount(gltf); ++n)
    {
        size_t l = 0;
        const char* s = maniGltf_GetNodeBoneName(gltf, n, &l);
        if (l == length && memcmp(s, name, length) == 0)
        {
            return n;
        }
    }
    return UINT32_MAX;
}

// The largest difference of the animated bone's local transform between
// the two clips over the first's samples (rotations up to sign).
static double Difference(const maniClip* a, const maniClip* b, const maniSkeleton* sa,
                         const maniSkeleton* sb, uint32_t boneA, uint32_t boneB)
{
    maniPose* pa = NULL;
    maniPose* pb = NULL;
    if (maniInitPose(sa, s_poseMemory, sizeof s_poseMemory / 2, &pa) != mani_success ||
        maniInitPose(sb, s_poseMemory + sizeof s_poseMemory / 2, sizeof s_poseMemory / 2, &pb) !=
            mani_success)
    {
        return HUGE_VAL;
    }
    double worst = 0;
    for (uint32_t k = 0; k < maniClip_GetSampleCount(a); ++k)
    {
        float t = fminf((float)k / 30.0f, maniClip_GetDuration(a));
        if (maniClip_Sample(a, sa, t, pa) != mani_success ||
            maniClip_Sample(b, sb, t, pb) != mani_success)
        {
            return HUGE_VAL;
        }
        maniTransform x = maniPose_GetLocal(pa, boneA);
        maniTransform y = maniPose_GetLocal(pb, boneB);
        float u[10];
        float w[10];
        memcpy(u, &x.translation, 12);
        memcpy(u + 3, &x.rotation, 16);
        memcpy(u + 7, &x.scale, 12);
        memcpy(w, &y.translation, 12);
        memcpy(w + 3, &y.rotation, 16);
        memcpy(w + 7, &y.scale, 12);
        double dot = 0;
        for (int c = 3; c < 7; ++c)
        {
            dot += (double)u[c] * (double)w[c];
        }
        for (int c = 0; c < 10; ++c)
        {
            double other = c >= 3 && c < 7 && dot < 0 ? -(double)w[c] : (double)w[c];
            worst = fmax(worst, fabs((double)u[c] - other));
        }
    }
    return worst;
}

// The KHR file (filters, version 1 streams, quantized) against the
// lossless EXT one: each animation's bone within the quantization.
static void TestQuantizedClips(void)
{
    size_t sizeA = 0;
    size_t sizeB = 0;
    uint8_t* bytesA = ReadFile("InterpolationTest-meshopt.glb", &sizeA);
    uint8_t* bytesB = ReadFile("InterpolationTest-khr.glb", &sizeB);
    maniBuilder* builder = NewBuilder();
    maniGltf* a = NULL;
    maniGltf* b = NULL;
    bool open = bytesA != NULL && bytesB != NULL &&
                maniOpenGltf(builder, bytesA, sizeA, &a).result == mani_success &&
                maniOpenGltf(builder, bytesB, sizeB, &b).result == mani_success &&
                maniGltf_GetAnimationCount(a) == 9 && maniGltf_GetAnimationCount(b) == 9;
    CHECK(open, "both open, nine animations");
    // The animated node of each animation, by its first track: the
    // node a cube's name gives in both.
    static const char* const kCubes[9] = {"Cube",     "Cube.001", "Cube.002",
                                          "Cube.003", "Cube.004", "Cube.005",
                                          "Cube.006", "Cube.008", "Cube.009"};
    double worst = 0;
    for (uint32_t i = 0; open && i < 9; ++i)
    {
        uint32_t length = (uint32_t)strlen(kCubes[i]);
        uint32_t na = FindNode(a, kCubes[i], length);
        uint32_t nb = FindNode(b, kCubes[i], length);
        bool built = na != UINT32_MAX && nb != UINT32_MAX &&
                     BuildClip(builder, a, na, i, &s_built[0]) &&
                     BuildClip(builder, b, nb, i, &s_built[1]);
        CHECK(built, "both clips build");
        const Built* x = &s_built[0];
        const Built* y = &s_built[1];
        uint32_t boneA =
            built ? maniSkeleton_FindBone(&x->skeleton, kCubes[i], length) : MANI_NO_BONE;
        uint32_t boneB =
            built ? maniSkeleton_FindBone(&y->skeleton, kCubes[i], length) : MANI_NO_BONE;
        worst = boneA != MANI_NO_BONE && boneB != MANI_NO_BONE
                    ? fmax(worst,
                           Difference(&x->clip, &y->clip, &x->skeleton, &y->skeleton, boneA, boneB))
                    : HUGE_VAL;
    }
    CHECK(worst < 1e-3, "within the quantization");
    if (!(worst < 1e-3))
    {
        printf("  the quantized file's largest difference: %g\n", worst);
    }
    maniCloseGltf(b);
    maniCloseGltf(a);
    maniDestroyBuilder(builder);
    free(bytesB);
    free(bytesA);
}

// The KHR skin: skeleton, skin (inverse binds of 64 bytes) and clip.
static void TestPackedSkin(void)
{
    size_t size = 0;
    uint8_t* bytes = ReadFile("SimpleSkin-khr.glb", &size);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniGltfOptions options = maniDefaultGltfOptions();
    size_t built = 0;
    maniSkeleton skeleton;
    Built* b = &s_built[0];
    bool open = bytes != NULL && maniOpenGltf(builder, bytes, size, &gltf).result == mani_success &&
                maniBuildGltfSkeleton(builder, gltf, 0, &options).result == mani_success &&
                maniBuilder_CopyBuilt(builder, b->skeletonFile, sizeof b->skeletonFile, &built) ==
                    mani_success &&
                maniOpenSkeleton(b->skeletonFile, built, &skeleton).result == mani_success;
    CHECK(open, "the skin's skeleton");
    CHECK(open && maniBuildGltfSkin(builder, gltf, 0, &skeleton, &options).result == mani_success,
          "the skin");
    maniImportClipOptions clipOptions = {.sampleRate = 30.0f, .loop = maniLoopClamp};
    CHECK(open &&
              maniBuildGltfClip(builder, gltf, 0, &skeleton, &options, &clipOptions, NULL).result ==
                  mani_success,
          "the clip");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    free(bytes);
}

static int WriteSeed(const char* path)
{
    HugeView();
    FILE* file = fopen(path, "wb");
    bool ok = file != NULL && fwrite(s_json, 1, strlen(s_json), file) == strlen(s_json);
    ok = file != NULL && fclose(file) == 0 && ok;
    return ok ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc >= 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestDecodedAtOpen();
    TestModes();
    TestRules();
    TestMarks();
    TestEdges();
    TestBothOnAView();
    TestBadStream();
    TestSupplied();
    TestAllocationFailures();
    TestPackedFiles();
    TestFallbackEqual();
    TestQuantizedClips();
    TestPackedSkin();
    return s_failures == 0 ? 0 : 1;
}
