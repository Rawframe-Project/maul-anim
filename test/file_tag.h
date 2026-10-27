// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The tag a refusal at an offset of a file must name, read from the
// file's own header and table: the tag of the section holding the
// offset, 0 in the header, the table and the padding between sections.
// The openers' suites check it with every format refusal they make.

#ifndef MAUL_ANIM_TEST_FILE_TAG_H
#define MAUL_ANIM_TEST_FILE_TAG_H

#include <stddef.h>
#include <stdint.h>

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

#endif // MAUL_ANIM_TEST_FILE_TAG_H
