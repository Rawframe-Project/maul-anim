// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tag a refusal at an offset of a file must name, read from the
// file's own header and table: the tag of the section holding the
// offset, 0 in the header, the table and the padding between sections.
// The openers' suites check it with every format refusal they make, and
// check that a file of another format version is refused, that sections
// placed or padded other than as the writer does are refused, and that a
// tag past the kind's last is refused as a later writer's.

#ifndef MAUL_ANIM_TEST_FILE_TAG_H
#define MAUL_ANIM_TEST_FILE_TAG_H

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static inline uint64_t TestFileWord(const uint8_t* p, int bytes)
{
    uint64_t v = 0;
    for (int i = 0; i < bytes; ++i)
    {
        v |= (uint64_t)p[i] << (8 * i);
    }
    return v;
}

static inline uint32_t TestTagAt(const uint8_t* file, size_t size, uint64_t offset)
{
    if (size < 16)
    {
        return 0;
    }
    uint64_t header = TestFileWord(file + 8, 4);
    uint64_t count = TestFileWord(file + 12, 4);
    for (uint64_t i = 0; i < count && header + 24 * (i + 1) <= size; ++i)
    {
        const uint8_t* entry = file + header + 24 * i;
        uint64_t at = TestFileWord(entry + 8, 8);
        if (offset >= at && offset - at < TestFileWord(entry + 16, 8))
        {
            return (uint32_t)TestFileWord(entry, 4);
        }
    }
    return 0;
}

// An opener of a whole file, its view or state discarded.
typedef maniFileResult (*TestOpener)(const void* bytes, size_t size);

// Whether open refuses a copy of the file stamped with the format
// version before its own and with the one after, at the version (offset
// 6, tag 0).
static inline bool TestOtherVersionsRefused(const uint8_t* file, size_t size, TestOpener open)
{
    // Aligned by hand, as aligned_alloc is not everywhere.
    uint8_t* block = (uint8_t*)malloc(size + 16);
    uint8_t* copy = block != NULL ? block + (16 - (uintptr_t)block % 16) % 16 : NULL;
    if (copy == NULL || size < 8)
    {
        free(block);
        return false;
    }
    memcpy(copy, file, size);
    uint64_t own = TestFileWord(file + 6, 2);
    bool refused = own > 1;
    for (int k = 0; k < 2; ++k)
    {
        uint64_t other = k == 0 ? own - 1 : own + 1;
        copy[6] = (uint8_t)other;
        copy[7] = (uint8_t)(other >> 8);
        maniFileResult r = open(copy, size);
        refused = refused && r.result == mani_errorVersion && r.offset == 6 && r.tag == 0;
    }
    free(block);
    return refused;
}

// A block of size bytes at a 16-byte aligned address, aligned by hand
// as aligned_alloc is not everywhere; *blockOut is what to free.
static inline uint8_t* TestAlignedCopy(size_t size, uint8_t** blockOut)
{
    uint8_t* block = (uint8_t*)malloc(size + 16);
    *blockOut = block;
    return block != NULL ? block + (16 - (uintptr_t)block % 16) % 16 : NULL;
}

static inline void TestStoreWord(uint8_t* p, uint64_t v, int bytes)
{
    for (int i = 0; i < bytes; ++i)
    {
        p[i] = (uint8_t)(v >> (8 * i));
    }
}

static inline uint64_t TestAlignUp(uint64_t v)
{
    return (v + 15) / 16 * 16;
}

// The payload hash restamped after a change.
static inline void TestRehashFile(uint8_t* file, size_t size)
{
    TestStoreWord(file + 32, maniHash64(MANI_HASH_INIT, file + 48, (int64_t)size - 48), 8);
}

// Where the section at index i of a table of total entries goes, laid
// out as the writer lays it out after end (the previous section's end,
// or the table's), the last one shift bytes past its place.
static inline uint64_t TestPlace(uint64_t end, uint64_t i, uint64_t total, uint64_t shift)
{
    return TestAlignUp(end) + (i + 1 == total ? shift : 0);
}

// The file laid out again into out, as the writer lays out its sections:
// with one more section, extraSize bytes of extra tagged extraTag (none
// for 0, past every tag the file has), the last section shift bytes past
// its place, and trailing zero bytes after it, every other byte zero.
// Returns the new size; out holds size + 24 + extraSize + shift +
// trailing + 32.
static inline size_t TestRelayout(const uint8_t* file, uint32_t extraTag, const uint8_t* extra,
                                  uint64_t extraSize, uint64_t shift, uint64_t trailing,
                                  uint8_t* out)
{
    uint64_t count = TestFileWord(file + 12, 4);
    uint64_t total = count + (extraTag != 0 ? 1 : 0);
    uint64_t end = 48 + 24 * total;
    for (uint64_t i = 0; i < total; ++i)
    {
        uint64_t sectionSize = i == count ? extraSize : TestFileWord(file + 48 + 24 * i + 16, 8);
        end = TestPlace(end, i, total, shift) + sectionSize;
    }
    size_t size = (size_t)(end + trailing);
    memset(out, 0, size);
    memcpy(out, file, 48);
    TestStoreWord(out + 12, total, 4);
    end = 48 + 24 * total;
    for (uint64_t i = 0; i < total; ++i)
    {
        const uint8_t* entry = file + 48 + 24 * i;
        bool last = i == count;
        uint64_t sectionSize = last ? extraSize : TestFileWord(entry + 16, 8);
        uint64_t offset = TestPlace(end, i, total, shift);
        uint8_t* to = out + 48 + 24 * i;
        TestStoreWord(to, last ? extraTag : TestFileWord(entry, 4), 4);
        TestStoreWord(to + 8, offset, 8);
        TestStoreWord(to + 16, sectionSize, 8);
        memcpy(out + offset, last ? extra : file + TestFileWord(entry + 8, 8), (size_t)sectionSize);
        end = offset + sectionSize;
    }
    TestStoreWord(out + 24, size, 8);
    TestRehashFile(out, size);
    return size;
}

// Whether the file has no section of the tag: an absent field, which
// holds its default.
static inline bool TestSectionAbsent(const uint8_t* file, uint32_t tag)
{
    uint64_t count = TestFileWord(file + 12, 4);
    for (uint64_t i = 0; i < count; ++i)
    {
        if (TestFileWord(file + 48 + 24 * i, 4) == tag)
        {
            return false;
        }
    }
    return true;
}

// The first byte between two sections, or between the table and the
// first section; 0 when the file has none.
static inline uint64_t TestFirstGap(const uint8_t* file)
{
    uint64_t count = TestFileWord(file + 12, 4);
    uint64_t next = 48 + 24 * count;
    for (uint64_t i = 0; i < count; ++i)
    {
        const uint8_t* entry = file + 48 + 24 * i;
        if (TestFileWord(entry + 8, 8) > next)
        {
            return next;
        }
        next = TestFileWord(entry + 8, 8) + TestFileWord(entry + 16, 8);
    }
    return 0;
}

static inline bool TestRefusedAt(maniFileResult r, maniResult result, uint64_t offset)
{
    return r.result == result && r.offset == offset && r.tag == 0;
}

// Whether open takes the file laid out again as the writer lays it out
// (the same bytes), and refuses it with its last section 16 bytes past
// its place (at that entry's offset), with 16 zero bytes after its last
// section (at the file size, 24), with a nonzero byte between two
// sections (at that byte) and with its last section empty, the file
// ending where it starts (at that entry's size): an empty section is
// left out, never written empty.
static inline bool TestPlacementRefused(const uint8_t* file, size_t size, TestOpener open)
{
    uint64_t count = TestFileWord(file + 12, 4);
    uint8_t* block = NULL;
    uint8_t* copy = TestAlignedCopy(size + 24 + 32 + 16, &block);
    if (copy == NULL || size < 48 || count == 0)
    {
        free(block);
        return false;
    }
    size_t same = TestRelayout(file, 0, NULL, 0, 0, 0, copy);
    bool refused =
        same == size && memcmp(copy, file, size) == 0 && open(copy, same).result == mani_success;
    size_t shifted = TestRelayout(file, 0, NULL, 0, 16, 0, copy);
    refused =
        refused && TestRefusedAt(open(copy, shifted), mani_errorFormat, 48 + 24 * (count - 1) + 8);
    size_t longer = TestRelayout(file, 0, NULL, 0, 0, 16, copy);
    refused = refused && TestRefusedAt(open(copy, longer), mani_errorFormat, 24);
    uint64_t gap = TestFirstGap(file);
    if (gap != 0)
    {
        memcpy(copy, file, size);
        copy[gap] = 1;
        TestRehashFile(copy, size);
        refused = refused && TestRefusedAt(open(copy, size), mani_errorFormat, gap);
    }
    memcpy(copy, file, size);
    uint64_t last = 48 + 24 * (count - 1);
    uint64_t emptyAt = TestFileWord(file + last + 8, 8);
    TestStoreWord(copy + last + 16, 0, 8);
    TestStoreWord(copy + 24, emptyAt, 8);
    TestRehashFile(copy, (size_t)emptyAt);
    refused = refused && TestRefusedAt(open(copy, size), mani_errorFormat, last + 16);
    free(block);
    return refused;
}

// Whether open refuses the file with one more section tagged past the
// kind's last tag with mani_errorVersion at that table entry, as only a
// later writer knows the tag, and with mani_errorFormat at the hash when
// the hash is also wrong, since damage reads as damage.
static inline bool TestLaterTagRefused(const uint8_t* file, size_t size, TestOpener open,
                                       uint32_t lastTag)
{
    uint64_t count = TestFileWord(file + 12, 4);
    uint8_t* block = NULL;
    uint8_t* copy = TestAlignedCopy(size + 24 + 32 + 16, &block);
    if (copy == NULL || size < 48)
    {
        free(block);
        return false;
    }
    static const uint8_t kLater[16] = {0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A,
                                       0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    size_t later = TestRelayout(file, lastTag + 1, kLater, sizeof kLater, 0, 0, copy);
    bool refused = TestRefusedAt(open(copy, later), mani_errorVersion, 48 + 24 * count);
    copy[32] ^= 1;
    refused = refused && TestRefusedAt(open(copy, later), mani_errorFormat, 32);
    free(block);
    return refused;
}

#endif // MAUL_ANIM_TEST_FILE_TAG_H
