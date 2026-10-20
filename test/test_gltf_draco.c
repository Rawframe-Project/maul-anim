// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Draco decoder (mani-0021): the streams of test/data/draco, which
// Google's encoder wrote, sequential and Edgebreaker (standard and
// valence, both traversals, every mesh prediction), decoded to the
// FNV-1a 64 of Google's decoding of each; the header's and Edgebreaker
// decoder headers' rules and the point bound; every prefix and every
// single byte changed decoded without fault; an attribute count bounded
// by the bytes; allocation failures handled with nothing kept.

#include "arena.h"
#include "gltf_draco.h"
#include "test_budget.h"
#include "test_harness.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MANI_TEST_DATA
#error "MANI_TEST_DATA names test/data"
#endif

typedef struct Stream
{
    const char* name;
    // Google's decoding (research's draco_fixtures).
    uint64_t hash;
    uint32_t points;
} Stream;

static const Stream kStreams[] = {
    {"tube-seq-s0-q", 0x219a70d146566af7ull, 91},
    {"tube-seq-s10-raw", 0xce868800d48e8e75ull, 91},
    {"holes-seq-s5-q", 0xb1b17352a5ef68e1ull, 96},
    {"small-seq-s8-raw", 0xc333ef5b10604b89ull, 15},
    {"small-seq-s0-q", 0xdc3fab2093e67667ull, 15},
    {"large-seq-s2-q", 0xd1a7235e94961974ull, 650},
    // Constrained multi-parallelogram, geometric normals, portable texture
    // coordinates over their own table, prediction degree.
    {"tube-eb-s0-q", 0x5a27bcb514878437ull, 91},
    {"small-eb-s5-q", 0xe5ab3c15cad1b9f7ull, 15},
    // Holes, a second component, a non-manifold edge.
    {"holes-eb-s2-q", 0x1f7c27d3062e6385ull, 98},
    // Valence.
    {"large-eb-s0-q", 0x54d49231c7fe4420ull, 650},
    {"largeholes-eb-s2-q", 0x875585b2196c9b99ull, 657},
    {"small-eb-s10-q", 0x41ce2b514cc1cc07ull, 15},
    // Multi-parallelogram, which Google's decoder reads but its encoder no
    // longer writes.
    {"small-eb-s5-q-multi", 0xc81cc9914d042038ull, 15},
};

static uint8_t s_bytes[1u << 16];
static uint8_t s_copy[1u << 16];

static size_t ReadStream(const char* name)
{
    char path[512];
    snprintf(path, sizeof path, "%s/draco/%s.drc", MANI_TEST_DATA, name);
    FILE* f = fopen(path, "rb");
    size_t size = f != NULL ? fread(s_bytes, 1, sizeof s_bytes, f) : 0;
    if (f != NULL)
    {
        fclose(f);
    }
    CHECK(size > 0, name);
    return size;
}

static uint64_t Fnv(uint64_t h, const void* data, size_t size)
{
    const uint8_t* p = data;
    for (size_t i = 0; i < size; ++i)
    {
        h = (h ^ p[i]) * 0x100000001b3ull;
    }
    return h;
}

// The dump research hashes: counts, faces, then each attribute's
// description and every point's value bytes.
static uint64_t Dump(const maniDracoMesh* m)
{
    uint64_t h = 0xcbf29ce484222325ull;
    uint32_t counts[2] = {m->faceCount, m->pointCount};
    h = Fnv(h, counts, sizeof counts);
    h = Fnv(h, m->faces, (size_t)m->faceCount * 12u);
    for (uint32_t i = 0; i < m->attributeCount; ++i)
    {
        const maniDracoAttribute* a = &m->attributes[i];
        uint32_t head[5] = {a->type, a->dataType, a->components, a->normalized ? 1u : 0u,
                            a->uniqueId};
        h = Fnv(h, head, sizeof head);
        size_t stride = (size_t)a->components * maniDracoTypeSize(a->dataType);
        for (uint32_t p = 0; p < m->pointCount; ++p)
        {
            uint32_t v = a->pointToValue != NULL ? a->pointToValue[p] : p;
            h = Fnv(h, a->values + (size_t)v * stride, stride);
        }
    }
    return h;
}

static maniDracoStatus Decode(const uint8_t* bytes, size_t size, uint32_t most, uint64_t* hash)
{
    maniAllocator allocator = {0};
    maniArena arena = maniArenaMake(&allocator);
    maniDracoMesh m;
    maniDracoStatus s = maniDracoDecode(&arena, bytes, size, most, &m);
    if (s == maniDracoDone && hash != NULL)
    {
        *hash = Dump(&m);
    }
    maniArenaRelease(&arena);
    return s;
}

static void TestStreams(void)
{
    for (size_t i = 0; i < sizeof kStreams / sizeof kStreams[0]; ++i)
    {
        size_t size = ReadStream(kStreams[i].name);
        uint64_t hash = 0;
        CHECK(Decode(s_bytes, size, kStreams[i].points, &hash) == maniDracoDone &&
                  hash == kStreams[i].hash,
              "Google's decoding");
        if (hash != kStreams[i].hash)
        {
            printf("  %s: 0x%016llx\n", kStreams[i].name, (unsigned long long)hash);
        }
        CHECK(Decode(s_bytes, size, kStreams[i].points - 1u, NULL) == maniDracoInvalid,
              "more points than allowed");
    }
}

static void TestHeader(void)
{
    size_t size = ReadStream("small-seq-s0-q");
    static const struct
    {
        size_t at;
        uint8_t value;
        const char* what;
    } kChanges[] = {
        {0, 'd', "the magic"},     {5, 1, "major version 1"}, {6, 1, "minor version 1"},
        {6, 3, "minor version 3"}, {7, 0, "a point cloud"},   {8, 2, "an unknown method"},
    };
    for (size_t i = 0; i < sizeof kChanges / sizeof kChanges[0]; ++i)
    {
        memcpy(s_copy, s_bytes, size);
        s_copy[kChanges[i].at] = kChanges[i].value;
        CHECK(Decode(s_copy, size, 15, NULL) == maniDracoInvalid, kChanges[i].what);
    }
    CHECK(Decode(s_bytes, 10, 15, NULL) == maniDracoInvalid, "a short header");
    CHECK(Decode(s_bytes, 0, 15, NULL) == maniDracoInvalid, "no bytes");
}

// Edgebreaker's decoder headers, at tube-eb-s0-q's byte 89: per decoder
// its data id, kind and traversal (positions' data, vertex, prediction
// degree; then attribute data 0 to 3, the second a corner attribute).
static void TestDecoderHeaders(void)
{
    size_t size = ReadStream("tube-eb-s0-q");
    static const struct
    {
        size_t at;
        uint8_t value;
        maniDracoStatus status;
        const char* what;
    } kChanges[] = {
        {89, 0x80, maniDracoDone, "any negative id is the positions' data"},
        {96, 7, maniDracoDone, "any kind but 0 a corner attribute"},
        {92, 0xFF, maniDracoInvalid, "two decoders of the positions' data"},
        {92, 1, maniDracoInvalid, "an attribute data read twice"},
        {92, 4, maniDracoInvalid, "an attribute data past the count"},
        {91, 2, maniDracoInvalid, "an unknown traversal"},
        {97, 1, maniDracoInvalid, "a corner attribute by prediction degree"},
        {90, 1, maniDracoInvalid, "a corner attribute of the positions' data"},
    };
    for (size_t i = 0; i < sizeof kChanges / sizeof kChanges[0]; ++i)
    {
        uint64_t hash = 0;
        memcpy(s_copy, s_bytes, size);
        s_copy[kChanges[i].at] = kChanges[i].value;
        maniDracoStatus st = Decode(s_copy, size, 91, &hash);
        CHECK(st == kChanges[i].status && (st != maniDracoDone || hash == 0x5a27bcb514878437ull),
              kChanges[i].what);
    }
}

// Every prefix and every byte changed (to its complement and to zero),
// for a byte step, decoded without fault under the sanitizers; prefixes
// never give a mesh shorter than the stream's.
static void Damage(const char* name, uint32_t points, size_t step)
{
    size_t size = ReadStream(name);
    bool prefixes = true;
    for (size_t n = 0; n < size; n += step)
    {
        prefixes = prefixes && Decode(s_bytes, n, points, NULL) != maniDracoDone;
    }
    CHECK(prefixes, "no prefix decodes");
    for (size_t at = 0; at < size; at += step)
    {
        memcpy(s_copy, s_bytes, size);
        s_copy[at] = (uint8_t)~s_copy[at];
        (void)Decode(s_copy, size, points, NULL);
        s_copy[at] = 0;
        (void)Decode(s_copy, size, points, NULL);
    }
}

static void TestDamage(void)
{
    Damage("small-seq-s0-q", 15, 1);
    Damage("small-eb-s5-q", 15, 1);
    Damage("tube-eb-s0-q", 91, 1);
    Damage("large-eb-s0-q", 650, 5);
}

// The largest block a decode asks for.
static void* LargestAlloc(size_t size, size_t alignment, void* context)
{
    size_t* largest = context;
    (void)alignment;
    *largest = size > *largest ? size : *largest;
    return malloc(size);
}

static void LargestFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    free(memory);
}

// A decoder's attribute count past what the bytes left can describe (six
// bytes each) is refused before anything is allocated for it: 4,000
// attributes over 1,000 bytes, a count Google's bound (five per byte)
// allows, take no block their descriptions would need.
static void TestAttributeCount(void)
{
    static const uint8_t kHead[] = {'D', 'R', 'A', 'C', 'O', 2, 2, 1, 0, 0, 0,
                                    // No faces, one point, plain indices,
                                    // one decoder of 4,000 attributes.
                                    0, 1, 1, 1, 0xA0, 0x1F};
    memset(s_copy, 0, 1000u + sizeof kHead);
    memcpy(s_copy, kHead, sizeof kHead);
    size_t largest = 0;
    maniAllocator allocator = {LargestAlloc, LargestFree, &largest};
    maniArena arena = maniArenaMake(&allocator);
    maniDracoMesh m;
    CHECK(maniDracoDecode(&arena, s_copy, 1000u + sizeof kHead, 1, &m) == maniDracoInvalid,
          "more attributes than the bytes hold");
    maniArenaRelease(&arena);
    CHECK(largest < 4000u * sizeof(maniDracoAttribute), "nothing taken for them");
}

static void Sweep(const Stream* stream)
{
    size_t size = ReadStream(stream->name);
    bool clean = true;
    bool done = false;
    for (int32_t calls = 0; !done && clean && calls < 4096; ++calls)
    {
        TestBudget budget = {calls, 0};
        maniAllocator allocator = {TestBudgetAlloc, TestBudgetFree, &budget};
        maniArena arena = maniArenaMake(&allocator);
        maniDracoMesh m;
        maniDracoStatus s = maniDracoDecode(&arena, s_bytes, size, stream->points, &m);
        done = s == maniDracoDone && Dump(&m) == stream->hash;
        clean = s != maniDracoInvalid;
        maniArenaRelease(&arena);
        clean = clean && budget.outstanding == 0;
    }
    CHECK(done && clean, "allocation failures handled, nothing kept");
}

static void TestAllocationFailures(void)
{
    Sweep(&kStreams[0]);
    Sweep(&kStreams[6]);
}

int main(void)
{
    TestStreams();
    TestHeader();
    TestDecoderHeaders();
    TestDamage();
    TestAttributeCount();
    TestAllocationFailures();
    return s_failures == 0 ? 0 : 1;
}
