// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Little-endian reads and writes of file fields at any alignment. The
// library's files are little-endian, and so is every platform it builds
// for; a big-endian build is refused here rather than misreading files.

#ifndef MAUL_ANIM_SRC_BYTES_H
#define MAUL_ANIM_SRC_BYTES_H

#include <stdint.h>
#include <string.h>

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "maul-anim: files are little-endian and this target is not"
#endif

static inline uint16_t maniLoadU16(const uint8_t* p)
{
    uint16_t value = 0;
    memcpy(&value, p, sizeof value);
    return value;
}

static inline uint32_t maniLoadU32(const uint8_t* p)
{
    uint32_t value = 0;
    memcpy(&value, p, sizeof value);
    return value;
}

static inline uint64_t maniLoadU64(const uint8_t* p)
{
    uint64_t value = 0;
    memcpy(&value, p, sizeof value);
    return value;
}

static inline void maniStoreU16(uint8_t* p, uint16_t value)
{
    memcpy(p, &value, sizeof value);
}

static inline void maniStoreU32(uint8_t* p, uint32_t value)
{
    memcpy(p, &value, sizeof value);
}

static inline void maniStoreU64(uint8_t* p, uint64_t value)
{
    memcpy(p, &value, sizeof value);
}

#endif // MAUL_ANIM_SRC_BYTES_H
