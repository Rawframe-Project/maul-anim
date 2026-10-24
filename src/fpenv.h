// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The floating-point environment every computation of the library runs
// in: round to nearest, denormals kept, no traps. A host may
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

// Sets round to nearest, denormals kept and every exception masked,
// writing the thread's floating-point control register only when that
// changes it, and returns the register as the host had it: everything
// maniFloatEnvLeave needs to restore it. The value is returned rather
// than stored through a pointer so a public call keeps it in a register:
// a local whose address is taken costs every wrapper a stack protector's
// canary.
uint64_t maniFloatEnvEnter(void);

// Restores the control register maniFloatEnvEnter returned, writing it
// only when it differs.
void maniFloatEnvLeave(uint64_t saved);

// The thread's control register as it is: MXCSR on x86, FPCR on AArch64,
// 0 on WebAssembly. For tests and diagnostics.
uint64_t maniFloatControlRead(void);

// Sets the thread's control register to value; nothing on WebAssembly. For
// tests, which play a host with another setting.
void maniFloatControlWrite(uint64_t value);

#endif // MAUL_ANIM_SRC_FPENV_H
