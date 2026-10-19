// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// glTF's meshopt compression (mani-0008): streams meshoptimizer's
// encoders made (gltf_meshopt_fixtures.h) decoded to the data they
// encode, attributes of versions 0 and 1 over every control and channel
// mode and two blocks, triangles and index sequences at 2 and 4 bytes;
// version 1 refused where only version 0 is allowed (EXT); each stream
// refused with another header, a byte short or a byte over; the
// specification's stricter rules (channel mode bytes, the triangle code
// table, FIFO entries never written, LEB128 lengths); the filters within
// one unit of meshoptimizer's output (exponential exactly), by the
// specification's formulas on hand cases, and their bits pinned. Run as
// test_gltf_meshopt --seed <file>... it writes fuzz_meshopt's seeds, a
// stream a file in fixture order behind its 5-byte prefix.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "gltf_meshopt.h"
#include "gltf_meshopt_fixtures.h"
#include "test_harness.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t s_out[8192];
static uint8_t s_stream[8192];

static bool Decode(const MeshoptFixture* f, const uint8_t* stream, size_t size, bool v1)
{
    return maniMeshoptDecode(s_out, f->count, f->stride, (maniMeshoptMode)f->mode, v1, stream,
                             size);
}

static void TestStreams(void)
{
    for (size_t k = 0; k < sizeof kMeshoptFixtures / sizeof kMeshoptFixtures[0]; ++k)
    {
        const MeshoptFixture* f = &kMeshoptFixtures[k];
        size_t bytes = (size_t)f->count * f->stride;
        CHECK(bytes <= sizeof s_out && f->size + 1 <= sizeof s_stream, "room");
        memset(s_out, 0xCD, sizeof s_out);
        CHECK(Decode(f, f->stream, f->size, true) && memcmp(s_out, f->out, bytes) == 0,
              "a stream decoded to its data");
        CHECK(!f->v1 || !Decode(f, f->stream, f->size, false), "version 1 only under KHR");
        memcpy(s_stream, f->stream, f->size);
        s_stream[0] ^= 2u;
        CHECK(!Decode(f, s_stream, f->size, true), "another header refused");
        s_stream[0] ^= 2u;
        CHECK(!Decode(f, s_stream, f->size - 1, true), "a byte short refused");
        // A byte over: the data runs into what is now padding or table.
        memmove(s_stream + 2, s_stream + 1, f->size - 1);
        CHECK(!Decode(f, s_stream, f->size + 1, true), "a byte over refused");
    }
}

// The fixture of version 1 (its channel bytes are its last 3: byte
// deltas, 2-byte deltas, XOR).
static void TestChannels(void)
{
    const MeshoptFixture* f = &kMeshoptFixtures[1];
    CHECK(f->v1 == 1 && f->stride == 12, "the version 1 fixture");
    size_t at = f->size - 3;
    const uint8_t kBad[4][2] = {{0, 0x10}, {0, 0x03}, {0, 0x04}, {1, 0x21}};
    for (int k = 0; k < 4; ++k)
    {
        memcpy(s_stream, f->stream, f->size);
        s_stream[at + kBad[k][0]] = kBad[k][1];
        CHECK(!Decode(f, s_stream, f->size, true), "a channel byte the specification forbids");
    }
}

static void TestTriangleRules(void)
{
    // One triangle by the code table's first entry (0x00: three new
    // indices).
    uint8_t s[1 + 1 + 16] = {0xE1, 0xF0};
    uint8_t out[12];
    CHECK(maniMeshoptDecode(out, 3, 4, maniMeshoptTriangles, false, s, sizeof s) &&
              maniLoadU32(out) == 0 && maniLoadU32(out + 4) == 1 && maniLoadU32(out + 8) == 2,
          "a triangle of new indices");
    s[1 + 1 + 14] = 1;
    CHECK(!maniMeshoptDecode(out, 3, 4, maniMeshoptTriangles, false, s, sizeof s),
          "the table's last two bytes not 0");
    s[1 + 1 + 14] = 0;
    s[1 + 1 + 3] = 0xF1;
    CHECK(!maniMeshoptDecode(out, 3, 4, maniMeshoptTriangles, false, s, sizeof s),
          "a nibble of 0xf in the table");
    // An edge FIFO entry never written.
    uint8_t edge[1 + 1 + 16] = {0xE1, 0x00};
    CHECK(!maniMeshoptDecode(out, 3, 4, maniMeshoptTriangles, false, edge, sizeof edge),
          "the edge FIFO read before a write");
    // A vertex FIFO entry never written: 0xfe, then Z = 1.
    uint8_t vertex[1 + 1 + 1 + 16] = {0xE1, 0xFE, 0x10};
    CHECK(!maniMeshoptDecode(out, 3, 4, maniMeshoptTriangles, false, vertex, sizeof vertex),
          "the vertex FIFO read before a write");
    // Two triangles: new indices (0, 1, 2), then 0x00, the edge pushed
    // last, (a, c) = (0, 2), with the next index.
    uint8_t two[1 + 2 + 16] = {0xE1, 0xF0, 0x00};
    uint8_t six[24];
    CHECK(maniMeshoptDecode(six, 6, 4, maniMeshoptTriangles, false, two, sizeof two) &&
              maniLoadU32(six + 12) == 0 && maniLoadU32(six + 16) == 2 &&
              maniLoadU32(six + 20) == 3,
          "an edge from the FIFO and the next index");
}

static void TestIndexRules(void)
{
    uint8_t out[4];
    // A 5-byte LEB128 value ending high, then one ending low (0).
    const uint8_t high[1 + 5 + 4] = {0xD1, 0x80, 0x80, 0x80, 0x80, 0x80};
    const uint8_t low[1 + 5 + 4] = {0xD1, 0x80, 0x80, 0x80, 0x80, 0x00};
    CHECK(!maniMeshoptDecode(out, 1, 4, maniMeshoptIndices, false, high, sizeof high),
          "a LEB128 value of more than 5 bytes");
    CHECK(maniMeshoptDecode(out, 1, 4, maniMeshoptIndices, false, low, sizeof low) &&
              maniLoadU32(out) == 0,
          "a LEB128 value of 5 bytes");
    // 2: baseline 0, zigzag 1, a delta of -1 with 32-bit wraparound.
    const uint8_t back[1 + 1 + 4] = {0xD1, 0x02};
    CHECK(maniMeshoptDecode(out, 1, 4, maniMeshoptIndices, false, back, sizeof back) &&
              maniLoadU32(out) == 0xFFFFFFFFu,
          "a delta below 0 wraps");
    CHECK(maniMeshoptDecode(out, 1, 2, maniMeshoptIndices, false, back, sizeof back) &&
              maniLoadU16(out) == 0xFFFFu,
          "2-byte indices wrap");
    // Two baselines: 1 (baseline 1, +0), 8 (baseline 0, +2), 5 (baseline
    // 1, +1).
    const uint8_t two[1 + 3 + 4] = {0xD1, 0x01, 0x08, 0x05};
    uint8_t three[12];
    CHECK(maniMeshoptDecode(three, 3, 4, maniMeshoptIndices, false, two, sizeof two) &&
              maniLoadU32(three) == 0 && maniLoadU32(three + 4) == 2 && maniLoadU32(three + 8) == 1,
          "two baselines");
}

// Signed or unsigned components of 1 or 2 bytes within one unit.
static bool Near(const uint8_t* a, const uint8_t* b, uint32_t count, uint32_t bytes, bool sign)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        int32_t x = bytes == 1 ? (sign ? (int8_t)a[i] : a[i])
                               : (sign ? (int16_t)maniLoadU16(a + 2 * i) : maniLoadU16(a + 2 * i));
        int32_t y = bytes == 1 ? (sign ? (int8_t)b[i] : b[i])
                               : (sign ? (int16_t)maniLoadU16(b + 2 * i) : maniLoadU16(b + 2 * i));
        if (abs(x - y) > 1)
        {
            return false;
        }
    }
    return true;
}

static uint64_t Hash(uint64_t h, const uint8_t* p, size_t n)
{
    for (size_t i = 0; i < n; ++i)
    {
        h = (h ^ p[i]) * 0x100000001B3ull;
    }
    return h;
}

// The filters' outputs over the fixtures, hashed (FNV-1a), as this
// implementation gives them on every platform.
#define PINNED_FILTERS 0x723a04e3679878a4ull

static void TestFilters(void)
{
    uint64_t h = 0xCBF29CE484222325ull;
    for (size_t k = 0; k < sizeof kMeshoptFilterFixtures / sizeof kMeshoptFilterFixtures[0]; ++k)
    {
        const MeshoptFilterFixture* f = &kMeshoptFilterFixtures[k];
        size_t bytes = (size_t)f->count * f->stride;
        memcpy(s_out, f->in, bytes);
        maniMeshoptApplyFilter(s_out, f->count, f->stride, (maniMeshoptFilter)f->filter);
        bool same =
            f->filter == maniMeshoptExponential
                ? memcmp(s_out, f->out, bytes) == 0
                : Near(s_out, f->out, f->count * 4, f->stride / 4, f->filter != maniMeshoptColor);
        CHECK(same, "a filter within one unit of meshoptimizer's");
        h = Hash(h, s_out, bytes);
    }
    if (h != PINNED_FILTERS)
    {
        printf("  filters hash 0x%016llxull\n", (unsigned long long)h);
    }
    CHECK(h == PINNED_FILTERS, "the filters' bits pinned");
}

static void TestFilterCases(void)
{
    // Exponential: e = -3, m = 5: 0.625.
    uint8_t e[4];
    maniStoreU32(e, 0xFD000005u);
    maniMeshoptApplyFilter(e, 1, 4, maniMeshoptExponential);
    CHECK(maniLoadU32(e) == 0x3F200000u, "2^-3 * 5");
    // Octahedral: x = 1 and 1.0 at 8 bits; the fourth passed through.
    uint8_t o[4] = {127, 0, 127, 99};
    maniMeshoptApplyFilter(o, 1, 4, maniMeshoptOctahedral);
    CHECK(o[0] == 127 && o[1] == 0 && o[2] == 0 && o[3] == 99, "an octahedral +x");
    // Octahedral, negative hemisphere: x = -0.75, y = 0.75 at 1.0 = 127
    // gives z = -0.5, folded to (-1.25, 1.25, -0.5) before normalizing.
    uint8_t n[4] = {(uint8_t)-95, 95, 127, 0};
    maniMeshoptApplyFilter(n, 1, 4, maniMeshoptOctahedral);
    CHECK((int8_t)n[0] < 0 && (int8_t)n[1] > 0 && (int8_t)n[2] < 0 && (int8_t)n[0] == -(int8_t)n[1],
          "the negative hemisphere folded");
    // Quaternion: the identity with w the largest (index 3) at 16 bits.
    uint8_t q[8] = {0};
    maniStoreU16(q + 6, 0x7FFF);
    maniMeshoptApplyFilter(q, 1, 8, maniMeshoptQuaternion);
    CHECK(maniLoadU16(q) == 0 && maniLoadU16(q + 2) == 0 && maniLoadU16(q + 4) == 0 &&
              maniLoadU16(q + 6) == 32767,
          "the identity");
    // Color: Y 200, no chroma, alpha of 7 bits all set at K = 8.
    uint8_t c[4] = {200, 0, 0, 0xFF};
    maniMeshoptApplyFilter(c, 1, 4, maniMeshoptColor);
    CHECK(c[0] == 200 && c[1] == 200 && c[2] == 200 && c[3] == 255, "a grey");
}

// The fuzzer's prefix for a fixture: mode, stride, count, version 1.
static int WriteSeeds(int count, char** paths)
{
    size_t fixtures = sizeof kMeshoptFixtures / sizeof kMeshoptFixtures[0];
    for (int k = 0; k < count && (size_t)k < fixtures; ++k)
    {
        const MeshoptFixture* f = &kMeshoptFixtures[k];
        uint8_t prefix[5] = {(uint8_t)f->mode,
                             (uint8_t)(f->mode == 0 ? f->stride / 4u - 1u : f->stride / 4u),
                             (uint8_t)f->count, (uint8_t)(f->count >> 8), (uint8_t)f->v1};
        FILE* file = fopen(paths[k], "wb");
        bool ok = file != NULL && fwrite(prefix, 1, sizeof prefix, file) == sizeof prefix &&
                  fwrite(f->stream, 1, f->size, file) == f->size;
        ok = file != NULL && fclose(file) == 0 && ok;
        if (!ok)
        {
            return 1;
        }
    }
    return 0;
}

int main(int argc, char** argv)
{
    if (argc >= 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeeds(argc - 2, argv + 2);
    }
    TestStreams();
    TestChannels();
    TestTriangleRules();
    TestIndexRules();
    TestFilters();
    TestFilterCases();
    return s_failures == 0 ? 0 : 1;
}
