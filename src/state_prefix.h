// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The prefix every saved state starts with, the family's (maul2d's and
// maul3d's): the format's magic, the writer's MANI_ABI_VERSION, the
// format's own version and the header's size, each a little-endian u32.
// A state is read only by a release of the ABI version that wrote it.
#ifndef MAUL_ANIM_SRC_STATE_PREFIX_H
#define MAUL_ANIM_SRC_STATE_PREFIX_H
#include "bytes.h"

#include "maul-anim/base.h"

#include <stddef.h>
#include <stdint.h>

#define MANI_STATE_PREFIX_SIZE 16u

static inline void maniStoreStatePrefix(uint8_t* p, uint32_t magic, uint32_t formatVersion,
                                        uint32_t headerSize)
{
    maniStoreU32(p, magic);
    maniStoreU32(p + 4, MANI_ABI_VERSION);
    maniStoreU32(p + 8, formatVersion);
    maniStoreU32(p + 12, headerSize);
}

// The prefix checked in the family's order: bytes too short for it or
// another magic are not this format, another ABI or format version was
// written by another release, another header size or bytes too short
// for the header are corrupt.
static inline maniFileResult maniCheckStatePrefix(const uint8_t* b, size_t size, uint32_t magic,
                                                  uint32_t formatVersion, uint32_t headerSize)
{
    if (size < MANI_STATE_PREFIX_SIZE || maniLoadU32(b) != magic)
    {
        return (maniFileResult){mani_errorFormat, 0, 0};
    }
    if (maniLoadU32(b + 4) != MANI_ABI_VERSION)
    {
        return (maniFileResult){mani_errorVersion, 0, 4};
    }
    if (maniLoadU32(b + 8) != formatVersion)
    {
        return (maniFileResult){mani_errorVersion, 0, 8};
    }
    if (maniLoadU32(b + 12) != headerSize || size < headerSize)
    {
        return (maniFileResult){mani_errorFormat, 0, 12};
    }
    return (maniFileResult){mani_success, 0, 0};
}

#endif // MAUL_ANIM_SRC_STATE_PREFIX_H
