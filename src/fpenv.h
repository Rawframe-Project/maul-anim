// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The floating-point environment every computation of the library runs
// in (mani-0001): round to nearest, denormals kept, no traps. A host may
// run its threads with flush-to-zero, denormals-are-zero or another
// rounding mode; each public call that computes floats enters this
// environment and leaves the host's as it found it.
//
// Compilers treat floating-point arithmetic as free of side effects (the
// build uses -fno-trapping-math), so they may move it across the writes
// to the control register. A public call therefore does no arithmetic of
// its own between entering and leaving: it calls a worker marked
// MANI_NOINLINE, whose body cannot move out of the call.

#ifndef MAUL_ANIM_SRC_FPENV_H
#define MAUL_ANIM_SRC_FPENV_H

#include <stdint.h>

#if defined(_MSC_VER) && !defined(__clang__)
#define MANI_NOINLINE __declspec(noinline)
#else
#define MANI_NOINLINE __attribute__((noinline))
#endif

// The host's control register, saved on entry.
typedef struct maniFloatEnv
{
    uint64_t saved;
} maniFloatEnv;

// Saves the thread's floating-point control register into env and sets
// round to nearest, denormals kept and every exception masked, writing
// the register only when that changes it.
void maniFloatEnvEnter(maniFloatEnv* env);

// Restores the control register maniFloatEnvEnter saved, writing it only
// when it differs.
void maniFloatEnvLeave(const maniFloatEnv* env);

#endif // MAUL_ANIM_SRC_FPENV_H
