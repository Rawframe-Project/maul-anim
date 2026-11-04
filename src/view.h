// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What every file view shares: its cookie and the check that a section
// lies within the file it names. A view is state the library writes and
// the caller holds, so every call checks it (maul-conventions.md): its
// cookie, then each of its sections against the file's size it recorded.
// A view changed or mixed from two then reads only within the one file it
// names, or is refused.

#ifndef MAUL_ANIM_SRC_VIEW_H
#define MAUL_ANIM_SRC_VIEW_H

#include <stdbool.h>
#include <stdint.h>

// The cookie of a view of a container kind: the top bit set, so that a
// zeroed or hand-written view never matches, and distinct per kind.
#define MANI_VIEW_COOKIE(kind) (0x8D410000u | (uint32_t)(kind))

// Whether bytes at offset lie within a file of size.
static inline bool maniViewFits(uint64_t offset, uint64_t bytes, uint64_t size)
{
    return offset <= size && bytes <= size - offset;
}

#endif // MAUL_ANIM_SRC_VIEW_H
