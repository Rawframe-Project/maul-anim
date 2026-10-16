// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Allocation-failure sweeps over FBX imports (test_budget.h): opening a
// text, building the skeleton of its first model tree, a clip of one of
// its stacks, and a skin, each step's result mapped to an outcome.

#ifndef MAUL_ANIM_TEST_FBX_SWEEP_H
#define MAUL_ANIM_TEST_FBX_SWEEP_H

#include "test_budget.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>
#include <string.h>

typedef struct FbxSweep
{
    const char* text;
    // The stack to build a clip of, or UINT32_MAX for none.
    uint32_t stack;
    // The skin to build, or UINT32_MAX for none.
    uint32_t skin;
} FbxSweep;

static _Alignas(16) uint8_t s_sweepSkeleton[16384];

static inline TestOutcome FbxSweepSkeleton(maniBuilder* builder, const maniFbx* fbx,
                                           maniSkeleton* out)
{
    maniFbxOptions options = maniDefaultFbxOptions();
    size_t size = 0;
    maniImportResult r = maniFbxBuildSkeleton(builder, fbx, 0, &options);
    if (r.result != mani_success)
    {
        return TestOutcomeOf(r.result);
    }
    maniResult copied = maniCopyBuilt(builder, s_sweepSkeleton, sizeof s_sweepSkeleton, &size);
    if (copied != mani_success)
    {
        return testWrong;
    }
    return maniOpenSkeleton(s_sweepSkeleton, size, out).result == mani_success ? testDone
                                                                               : testWrong;
}

// The work: open, skeleton, then the clip and the skin asked for.
static inline TestOutcome FbxSweepWork(maniBuilder* builder, void* context)
{
    const FbxSweep* sweep = context;
    maniFbx* fbx = NULL;
    maniImportResult r = maniOpenFbx(builder, sweep->text, strlen(sweep->text), &fbx);
    TestOutcome outcome = TestOutcomeOf(r.result);
    maniSkeleton skeleton;
    maniFbxOptions options = maniDefaultFbxOptions();
    outcome = outcome == testDone ? FbxSweepSkeleton(builder, fbx, &skeleton) : outcome;
    if (outcome == testDone && sweep->stack != UINT32_MAX)
    {
        r = maniFbxBuildClip(builder, fbx, sweep->stack, &skeleton, &options,
                             &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                             NULL);
        outcome = TestOutcomeOf(r.result);
    }
    if (outcome == testDone && sweep->skin != UINT32_MAX)
    {
        r = maniFbxBuildSkin(builder, fbx, sweep->skin, &skeleton, &options);
        outcome = TestOutcomeOf(r.result);
    }
    maniCloseFbx(fbx);
    return outcome;
}

#endif // MAUL_ANIM_TEST_FBX_SWEEP_H
