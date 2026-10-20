// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The Draco decoder (mani-0021): the streams of test/data/draco, which
// Google's encoder wrote, decoded to the FNV-1a 64 of Google's decoding
// of each; the header's refusals and the point bound; every prefix and
// every single byte changed decoded without fault; allocation failures
// handled with nothing kept.

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
    {"tube-seq-s0-q", 0x219a70d146566af7ull, 91},  {"tube-seq-s10-raw", 0xce868800d48e8e75ull, 91},
    {"holes-seq-s5-q", 0xb1b17352a5ef68e1ull, 96}, {"small-seq-s8-raw", 0xc333ef5b10604b89ull, 15},
    {"small-seq-s0-q", 0xdc3fab2093e67667ull, 15}, {"large-seq-s2-q", 0xd1a7235e94961974ull, 650},
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

// Every prefix and every byte changed (to its complement and to zero)
// decoded without fault under the sanitizers; prefixes never give a
// mesh shorter than the stream's.
static void TestDamage(void)
{
    size_t size = ReadStream("small-seq-s0-q");
    bool prefixes = true;
    for (size_t n = 0; n < size; ++n)
    {
        prefixes = prefixes && Decode(s_bytes, n, 15, NULL) != maniDracoDone;
    }
    CHECK(prefixes, "no prefix decodes");
    for (size_t at = 0; at < size; ++at)
    {
        memcpy(s_copy, s_bytes, size);
        s_copy[at] = (uint8_t)~s_copy[at];
        (void)Decode(s_copy, size, 15, NULL);
        s_copy[at] = 0;
        (void)Decode(s_copy, size, 15, NULL);
    }
}

static void TestAllocationFailures(void)
{
    size_t size = ReadStream("tube-seq-s0-q");
    bool clean = true;
    bool done = false;
    for (int32_t calls = 0; !done && clean && calls < 4096; ++calls)
    {
        TestBudget budget = {calls, 0};
        maniAllocator allocator = {TestBudgetAlloc, TestBudgetFree, &budget};
        maniArena arena = maniArenaMake(&allocator);
        maniDracoMesh m;
        maniDracoStatus s = maniDracoDecode(&arena, s_bytes, size, 91, &m);
        done = s == maniDracoDone && Dump(&m) == kStreams[0].hash;
        clean = s != maniDracoInvalid;
        maniArenaRelease(&arena);
        clean = clean && budget.outstanding == 0;
    }
    CHECK(done && clean, "allocation failures handled, nothing kept");
}

int main(void)
{
    TestStreams();
    TestHeader();
    TestDamage();
    TestAllocationFailures();
    return s_failures == 0 ? 0 : 1;
}
