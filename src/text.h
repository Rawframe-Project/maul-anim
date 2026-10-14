// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// UTF-8 as RFC 3629 defines it: no overlong forms, no surrogates, nothing
// past U+10FFFF.

#ifndef MAUL_ANIM_SRC_TEXT_H
#define MAUL_ANIM_SRC_TEXT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// True when the bytes are well-formed UTF-8.
bool maniValidUtf8(const uint8_t* bytes, size_t length);

#endif // MAUL_ANIM_SRC_TEXT_H
