// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The file container: round trips, a pinned file, and every refusal at
// the offset it names.

#include "bytes.h"
#include "container.h"
#include "test_harness.h"

#include "maul-anim/base.h"

#include <string.h>

#define PINNED_FILE_HASH 0x8c14ee6de964f4f6ull

enum
{
    kCapacity = 512,
    kKind = maniKindSkeleton,
};

static _Alignas(16) uint8_t s_file[kCapacity + 16];
static uint64_t s_size;

static const uint8_t kFirst[5] = {1, 2, 3, 4, 5};
static const uint8_t kSecond[20] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

// Two sections, tags 3 and 7, the first unaligned in size.
static void WriteSample(void)
{
    maniSectionSource sections[2] = {{3, kFirst, sizeof kFirst}, {7, kSecond, sizeof kSecond}};
    CHECK(maniContainerSize(sections, 2, &s_size), "sample size");
    CHECK(s_size <= kCapacity, "sample fits");
    maniWriteContainer(s_file, kKind, 1, 0x1122334455667788ull, sections, 2);
}

// Restores the payload hash after a test changes the table.
static void Rehash(void)
{
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file + MANI_CONTAINER_HEADER_SIZE,
                               (int32_t)(s_size - MANI_CONTAINER_HEADER_SIZE));
    maniStoreU64(s_file + 32, hash);
}

static maniFileResult Open(void)
{
    maniContainer container;
    return maniOpenContainer(s_file, (size_t)s_size, kKind, 1, &container);
}

static void ExpectFormat(uint64_t offset, const char* what)
{
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat, what);
    CHECK(r.offset == offset, what);
}

static void TestRoundTrip(void)
{
    WriteSample();
    // Header 64, table 48, so the first section at 112, the second at 128.
    CHECK(s_size == 148, "laid out size");
    maniContainer container;
    maniFileResult r = maniOpenContainer(s_file, (size_t)s_size, kKind, 1, &container);
    CHECK(r.result == mani_success, "opens");
    CHECK(container.identity == 0x1122334455667788ull, "identity");
    CHECK(container.formatVersion == 1 && container.kind == kKind, "kind and version");
    CHECK(container.sectionCount == 2 && container.size == s_size, "count and size");
    CHECK(maniLoadU16(s_file + 16) == MANI_VERSION_MAJOR &&
              maniLoadU16(s_file + 18) == MANI_VERSION_MINOR &&
              maniLoadU16(s_file + 20) == MANI_VERSION_PATCH,
          "writer's version");

    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t offset = 0;
    CHECK(maniContainerSection(&container, 3, &data, &size, &offset), "first section");
    CHECK(offset == 112 && size == 5 && memcmp(data, kFirst, 5) == 0, "first bytes");
    CHECK(maniContainerSection(&container, 7, &data, &size, &offset), "second section");
    CHECK(offset == 128 && size == 20 && memcmp(data, kSecond, 20) == 0, "second bytes");
    CHECK(!maniContainerSection(&container, 5, &data, &size, &offset), "no such section");

    // A newer reader opens an older file.
    CHECK(maniOpenContainer(s_file, (size_t)s_size, kKind, 9, &container).result == mani_success,
          "newer reader");
    // Trailing bytes past the file's size are the caller's, not the file's.
    CHECK(maniOpenContainer(s_file, (size_t)s_size + 7, kKind, 1, &container).result ==
              mani_success,
          "trailing bytes");
}

static void TestEmptyContainer(void)
{
    CHECK(maniContainerSize(NULL, 0, &s_size) && s_size == 64, "empty size");
    maniWriteContainer(s_file, kKind, 1, 0, NULL, 0);
    maniContainer container;
    CHECK(maniOpenContainer(s_file, 64, kKind, 1, &container).result == mani_success, "empty");
    CHECK(container.sectionCount == 0, "no sections");
}

static void TestBadArguments(void)
{
    WriteSample();
    maniContainer container;
    CHECK(maniOpenContainer(NULL, 64, kKind, 1, &container).result == mani_errorInvalid, "null");
    CHECK(maniOpenContainer(s_file, (size_t)s_size, kKind, 1, NULL).result == mani_errorInvalid,
          "null out");
    CHECK(maniOpenContainer(s_file + 8, (size_t)s_size, kKind, 1, &container).result ==
              mani_errorInvalid,
          "misaligned");
}

static void TestHeaderRefusals(void)
{
    WriteSample();
    maniContainer container;
    maniFileResult r = maniOpenContainer(s_file, 63, kKind, 1, &container);
    CHECK(r.result == mani_errorFormat && r.offset == 63, "short of a header");

    s_file[1] = 'B';
    ExpectFormat(0, "magic");
    WriteSample();
    r = maniOpenContainer(s_file, (size_t)s_size, maniKindClip, 1, &container);
    CHECK(r.result == mani_errorFormat && r.offset == 4, "kind");

    maniStoreU16(s_file + 6, 0);
    ExpectFormat(6, "version zero");
    maniStoreU16(s_file + 6, 2);
    r = Open();
    CHECK(r.result == mani_errorVersion && r.offset == 6, "newer version");
    WriteSample();

    maniStoreU32(s_file + 8, 128);
    ExpectFormat(8, "header size");
    WriteSample();
    maniStoreU32(s_file + 12, MANI_CONTAINER_MAX_SECTIONS + 1);
    ExpectFormat(12, "section limit");
    WriteSample();
    maniStoreU16(s_file + 22, 1);
    ExpectFormat(22, "version padding");
    WriteSample();
    maniStoreU64(s_file + 24, 63);
    ExpectFormat(24, "file size below a header");
    maniStoreU64(s_file + 24, s_size + 1);
    ExpectFormat(24, "file size past the bytes");
    WriteSample();
    for (uint32_t i = 48; i < 64; ++i)
    {
        s_file[i] = 1;
        ExpectFormat(i, "reserved header bytes");
        s_file[i] = 0;
    }
}

static void TestTableRefusals(void)
{
    WriteSample();
    // A table running past the file.
    maniStoreU32(s_file + 12, 5);
    ExpectFormat(12, "table past the file");
    WriteSample();

    maniStoreU32(s_file + 88, 3);
    Rehash();
    ExpectFormat(88, "tags not increasing");
    WriteSample();
    maniStoreU32(s_file + 68, 1);
    Rehash();
    ExpectFormat(68, "entry padding");
    WriteSample();
    maniStoreU64(s_file + 72, 120);
    Rehash();
    ExpectFormat(72, "misaligned section");
    WriteSample();
    maniStoreU64(s_file + 72, 96);
    Rehash();
    ExpectFormat(72, "section over the table");
    WriteSample();
    maniStoreU64(s_file + 80, 17);
    Rehash();
    ExpectFormat(96, "overlapping sections");
    WriteSample();
    maniStoreU64(s_file + 104, 21);
    Rehash();
    ExpectFormat(104, "section past the end");
    WriteSample();
    maniStoreU64(s_file + 96, 160);
    Rehash();
    ExpectFormat(96, "offset past the end");
}

// Every byte the hash or a check covers refuses when changed; the
// writer's version and the identity are free fields.
static void TestEveryByteFlip(void)
{
    WriteSample();
    for (uint64_t i = 0; i < s_size; ++i)
    {
        bool free = (i >= 16 && i < 22) || (i >= 40 && i < 48);
        s_file[i] ^= 0x40;
        bool opened = Open().result == mani_success;
        s_file[i] ^= 0x40;
        CHECK(opened == free, "byte flip");
    }
}

static void TestSizeLimits(void)
{
    uint64_t size = 0;
    maniSectionSource wraps[2] = {{1, kFirst, 300}, {2, kFirst, UINT64_MAX - 50}};
    CHECK(!maniContainerSize(wraps, 2, &size), "sum overflows");
    // A last section is not padded, so one this large still has a size;
    // as the first of two it leaves no room to align the next.
    maniSectionSource last[1] = {{1, kFirst, UINT64_MAX - 200}};
    CHECK(maniContainerSize(last, 1, &size) && size == UINT64_MAX - 200 + 96, "a large last");
    maniSectionSource unaligned[2] = {{1, kFirst, UINT64_MAX - 120}, {2, kFirst, 0}};
    CHECK(!maniContainerSize(unaligned, 2, &size), "no room to align");
    maniSectionSource many[MANI_CONTAINER_MAX_SECTIONS + 1] = {{0}};
    CHECK(maniContainerSize(many, MANI_CONTAINER_MAX_SECTIONS, &size), "the limit");
    CHECK(!maniContainerSize(many, MANI_CONTAINER_MAX_SECTIONS + 1, &size), "over the limit");
}

// The sample's bytes, apart from the writer's version, on every platform.
static void TestPinnedBytes(void)
{
    WriteSample();
    memset(s_file + 16, 0, 6);
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_FILE_HASH, "pinned file");
}

int main(void)
{
    TestRoundTrip();
    TestEmptyContainer();
    TestBadArguments();
    TestHeaderRefusals();
    TestTableRefusals();
    TestEveryByteFlip();
    TestSizeLimits();
    TestPinnedBytes();
    return s_failures == 0 ? 0 : 1;
}
