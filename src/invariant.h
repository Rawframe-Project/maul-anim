// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// MANI_ASSERT states an internal invariant: a state that cannot occur
// unless the library itself is wrong. Input from callers (arguments,
// files, text, saved states) is never asserted; it is validated and
// refused with a status. The check traps in every build without NDEBUG
// (Debug, the sanitizer and fuzz builds), so a broken invariant stops the
// run where it happened instead of surfacing later as a wrong pose.
// Release builds compile it out without evaluating the condition, since
// sampling, blending and the graph's passes run per bone and per frame.

#ifndef MAUL_ANIM_SRC_INVARIANT_H
#define MAUL_ANIM_SRC_INVARIANT_H

#if defined(NDEBUG)
#define MANI_ASSERT(condition) ((void)sizeof((condition) ? 1 : 0))
#else
#define MANI_ASSERT(condition) ((condition) ? (void)0 : __builtin_trap())
#endif

// MANI_COLD marks a function that runs once for a file or a saved state
// (opening, checking, restoring) rather than every frame: GCC and Clang
// compile it for size and treat calls to it as unlikely. A hint to the
// optimizer only, which changes no result (the build allows no fast math
// and no contraction); empty on MSVC.
#if defined(__GNUC__) || defined(__clang__)
#define MANI_COLD __attribute__((cold))
#else
#define MANI_COLD
#endif

#endif // MAUL_ANIM_SRC_INVARIANT_H
