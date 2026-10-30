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

enum
{
    kCapacity = 2048,
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
                               (int64_t)(s_size - MANI_CONTAINER_HEADER_SIZE));
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
    // Header 48, table 48, so the first section at 96, the second at 112.
    CHECK(s_size == 132, "laid out size");
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
    CHECK(offset == 96 && size == 5 && memcmp(data, kFirst, 5) == 0, "first bytes");
    CHECK(maniContainerSection(&container, 7, &data, &size, &offset), "second section");
    CHECK(offset == 112 && size == 20 && memcmp(data, kSecond, 20) == 0, "second bytes");
    CHECK(!maniContainerSection(&container, 5, &data, &size, &offset), "no such section");

    // A reader of a later version refuses an earlier file.
    r = maniOpenContainer(s_file, (size_t)s_size, kKind, 2, &container);
    CHECK(r.result == mani_errorVersion && r.offset == 6, "a later reader");
    // Trailing bytes past the file's size are the caller's, not the file's.
    CHECK(maniOpenContainer(s_file, (size_t)s_size + 7, kKind, 1, &container).result ==
              mani_success,
          "trailing bytes");
}

static void TestEmptyContainer(void)
{
    CHECK(maniContainerSize(NULL, 0, &s_size) && s_size == 48, "empty size");
    maniWriteContainer(s_file, kKind, 1, 0, NULL, 0);
    maniContainer container;
    CHECK(maniOpenContainer(s_file, 48, kKind, 1, &container).result == mani_success, "empty");
    CHECK(container.sectionCount == 0, "no sections");
}

static void TestBadArguments(void)
{
    WriteSample();
    maniContainer container;
    CHECK(maniOpenContainer(NULL, 48, kKind, 1, &container).result == mani_errorInvalid, "null");
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
    maniFileResult r = maniOpenContainer(s_file, 47, kKind, 1, &container);
    CHECK(r.result == mani_errorFormat && r.offset == 47, "short of a header");

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
    maniStoreU64(s_file + 24, 47);
    ExpectFormat(24, "file size below a header");
    maniStoreU64(s_file + 24, s_size + 1);
    ExpectFormat(24, "file size past the bytes");
}

// One section over the limit, in a file whose table is otherwise right.
static void TestSectionLimit(void)
{
    uint32_t count = MANI_CONTAINER_MAX_SECTIONS + 1;
    uint64_t tableEnd = MANI_CONTAINER_HEADER_SIZE + (uint64_t)count * MANI_CONTAINER_ENTRY_SIZE;
    uint64_t sectionsAt = (tableEnd + 15) / 16 * 16;
    WriteSample();
    memset(s_file, 0, sizeof s_file);
    maniWriteContainer(s_file, kKind, 1, 0, NULL, 0);
    maniStoreU32(s_file + 12, count);
    maniStoreU64(s_file + 24, sectionsAt);
    for (uint32_t i = 0; i < count; ++i)
    {
        uint8_t* entry =
            s_file + MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        maniStoreU32(entry, i + 1);
        maniStoreU64(entry + 8, sectionsAt);
    }
    s_size = sectionsAt;
    Rehash();
    ExpectFormat(12, "sections over the limit");
}

static void TestTableRefusals(void)
{
    WriteSample();
    // A table running past the file.
    maniStoreU32(s_file + 12, 5);
    ExpectFormat(12, "table past the file");
    WriteSample();

    maniStoreU32(s_file + 72, 3);
    Rehash();
    ExpectFormat(72, "tags not increasing");
    WriteSample();
    maniStoreU32(s_file + 52, 1);
    Rehash();
    ExpectFormat(52, "entry padding");
    WriteSample();
    maniStoreU64(s_file + 56, 104);
    Rehash();
    ExpectFormat(56, "misaligned section");
    WriteSample();
    maniStoreU64(s_file + 56, 80);
    Rehash();
    ExpectFormat(56, "section over the table");
    WriteSample();
    maniStoreU64(s_file + 64, 17);
    Rehash();
    ExpectFormat(80, "overlapping sections");
    WriteSample();
    maniStoreU64(s_file + 88, 21);
    Rehash();
    ExpectFormat(88, "section past the end");
    WriteSample();
    maniStoreU64(s_file + 80, 144);
    Rehash();
    ExpectFormat(80, "offset past the end");
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
    CHECK(maniContainerSize(last, 1, &size) && size == UINT64_MAX - 200 + 80, "a large last");
    maniSectionSource unaligned[2] = {{1, kFirst, UINT64_MAX - 120}, {2, kFirst, 0}};
    CHECK(!maniContainerSize(unaligned, 2, &size), "no room to align");
    maniSectionSource many[MANI_CONTAINER_MAX_SECTIONS + 1] = {{0}};
    CHECK(maniContainerSize(many, MANI_CONTAINER_MAX_SECTIONS, &size), "the limit");
    CHECK(!maniContainerSize(many, MANI_CONTAINER_MAX_SECTIONS + 1, &size), "over the limit");
}

int main(void)
{
    TestRoundTrip();
    TestEmptyContainer();
    TestBadArguments();
    TestHeaderRefusals();
    TestTableRefusals();
    TestSectionLimit();
    TestEveryByteFlip();
    TestSizeLimits();
    return s_failures == 0 ? 0 : 1;
}
