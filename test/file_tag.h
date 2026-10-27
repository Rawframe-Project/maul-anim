// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tag a refusal at an offset of a file must name, read from the
// file's own header and table: the tag of the section holding the
// offset, 0 in the header, the table and the padding between sections.
// The openers' suites check it with every format refusal they make, and
// check that a file of another format version is refused.

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
    uint8_t* copy = (uint8_t*)aligned_alloc(16, (size + 15) / 16 * 16 + 16);
    if (copy == NULL || size < 8)
    {
        free(copy);
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
    free(copy);
    return refused;
}

#endif // MAUL_ANIM_TEST_FILE_TAG_H
