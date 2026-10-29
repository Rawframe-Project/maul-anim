// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// UTF-8 as RFC 3629 defines it: no overlong forms, no surrogates, nothing
// past U+10FFFF.

#ifndef MAUL_ANIM_SRC_TEXT_H
#define MAUL_ANIM_SRC_TEXT_H

#include "maul-anim/base.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The length of the well-formed sequence the bytes start with, 1 to 4,
// or 0 when they start with none (or are empty).
size_t maniUtf8Sequence(const uint8_t* bytes, size_t length);

// True when the bytes are well-formed UTF-8.
bool maniValidUtf8(const uint8_t* bytes, size_t length);

// True when a name's length passes MANI_MAX_NAME_BYTES.
static inline bool maniNamePastLimit(size_t length)
{
    return length > MANI_MAX_NAME_BYTES;
}

#endif // MAUL_ANIM_SRC_TEXT_H
