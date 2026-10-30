// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The host's code an owner object runs (its allocator, its task hooks),
// behind the library's own functions, which mark the object while the
// host's code runs: a function of the object called back in
// from there is refused with mani_errorBusy and one misuse counted.

#ifndef MAUL_ANIM_SRC_HOST_CODE_H
#define MAUL_ANIM_SRC_HOST_CODE_H

#include "maul-anim/base.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct maniHostCode
{
    // The host's allocator and task hooks, as the def gave them.
    maniAllocator allocator;
    maniTaskHooks tasks;
    // Set while the host's code runs; read by a call back in, which a
    // task hook may make from a worker.
    _Atomic bool running;
    // Misuses counted on the object: atomic, as a refusal may come from
    // a worker.
    _Atomic uint32_t misuses;
} maniHostCode;

// Keeps the host's allocator and task hooks (either may be NULL for
// none).
void maniHostCodeInit(maniHostCode* host, const maniAllocator* allocator,
                      const maniTaskHooks* tasks);

// The allocator the object's code uses: the host's, marked while it
// runs. It lives as long as host does.
maniAllocator maniHostAllocator(maniHostCode* host);

// The task hooks the object's code uses: the host's, marked while
// enqueueTask and finishTask run; without the host's, every item on the
// calling thread.
maniTaskHooks maniHostTasks(maniHostCode* host);

// Whether a function of the object is called back in from the host's
// code: then one misuse is counted and the function refuses. NULL is
// not.
bool maniHostCalledBack(maniHostCode* host);

// Marks the object for good as its destroy begins, before the first
// host call: a function of the object called back in from then on, the
// last free among it, is refused like a call from the host's code.
void maniHostRetire(maniHostCode* host);

// The same for an allocator maniHostAllocator gave (a document's copy of
// its builder's); false for any other.
bool maniHostAllocatorCalledBack(const maniAllocator* allocator);

static inline void maniHostCountMisuse(maniHostCode* host)
{
    atomic_fetch_add_explicit(&host->misuses, 1u, memory_order_relaxed);
}

static inline uint32_t maniHostMisuses(const maniHostCode* host)
{
    return atomic_load_explicit(&host->misuses, memory_order_relaxed);
}

#endif // MAUL_ANIM_SRC_HOST_CODE_H
