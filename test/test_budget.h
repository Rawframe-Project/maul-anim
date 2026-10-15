// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Allocation-failure sweeps: an allocator that fails its n-th call and
// counts what is outstanding, and a sweep that runs a piece of work with
// every allocation failing in turn until it succeeds, checking that each
// run fails cleanly and gives back everything it took at its size.

#ifndef MAUL_ANIM_TEST_BUDGET_H
#define MAUL_ANIM_TEST_BUDGET_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct TestBudget
{
    int32_t callsLeft;
    int64_t outstanding;
} TestBudget;

static inline void* TestBudgetAlloc(size_t size, size_t alignment, void* context)
{
    TestBudget* budget = context;
    (void)alignment;
    if (budget->callsLeft-- <= 0)
    {
        return NULL;
    }
    budget->outstanding += (int64_t)size;
    return malloc(size);
}

static inline void TestBudgetFree(void* memory, size_t size, size_t alignment, void* context)
{
    TestBudget* budget = context;
    (void)alignment;
    budget->outstanding -= (int64_t)size;
    free(memory);
}

// What a piece of work under a sweep returns.
typedef enum TestOutcome
{
    // It ran to the end.
    testDone,
    // It stopped on an allocation failure, as it should.
    testCapacity,
    // It stopped otherwise.
    testWrong,
} TestOutcome;

// A step's result as an outcome: done, Capacity, or wrong.
static inline TestOutcome TestOutcomeOf(maniResult result)
{
    return result == mani_success         ? testDone
           : result == mani_errorCapacity ? testCapacity
                                          : testWrong;
}

typedef TestOutcome (*TestWork)(maniBuilder* builder, void* context);

// Runs the work on a builder whose allocator fails its n-th call, for n
// from 0 until the work is done (at most limit runs); whether it got
// done, every other run stopping on Capacity, and nothing was kept.
static inline bool TestSweep(TestWork work, void* context, int32_t limit)
{
    bool clean = true;
    bool done = false;
    for (int32_t calls = 0; !done && clean && calls < limit; ++calls)
    {
        TestBudget budget = {calls, 0};
        maniBuilderDef def = maniDefaultBuilderDef();
        def.allocator = (maniAllocator){TestBudgetAlloc, TestBudgetFree, &budget};
        maniBuilder* builder = NULL;
        maniResult created = maniCreateBuilder(&def, &builder);
        if (created == mani_success)
        {
            TestOutcome outcome = work(builder, context);
            done = outcome == testDone;
            clean = outcome != testWrong;
            maniDestroyBuilder(builder);
        }
        else
        {
            clean = created == mani_errorCapacity;
        }
        clean = clean && budget.outstanding == 0;
    }
    return done && clean;
}

#endif // MAUL_ANIM_TEST_BUDGET_H
