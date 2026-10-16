// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Allocation-failure sweeps over glTF imports (test_budget.h): opening a
// document whose buffers are embedded, building its first skeleton, a
// clip of one of its animations and a skin, and reading its springs,
// each step's result mapped to an outcome.

#ifndef MAUL_ANIM_TEST_GLTF_SWEEP_H
#define MAUL_ANIM_TEST_GLTF_SWEEP_H

#include "test_budget.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct GltfSweep
{
    const char* json;
    // The animation to build a clip of, or UINT32_MAX for none.
    uint32_t animation;
    // The skin to build, or UINT32_MAX for none.
    uint32_t skin;
    // Whether to read the springs.
    bool springs;
} GltfSweep;

static _Alignas(16) uint8_t s_gltfSweepSkeleton[16384];

static inline TestOutcome GltfSweepSkeleton(maniBuilder* builder, const maniGltf* gltf,
                                            maniSkeleton* out)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    size_t size = 0;
    maniImportResult r = maniGltfBuildSkeleton(builder, gltf, 0, &options);
    if (r.result != mani_success)
    {
        return TestOutcomeOf(r.result);
    }
    if (maniCopyBuilt(builder, s_gltfSweepSkeleton, sizeof s_gltfSweepSkeleton, &size) !=
        mani_success)
    {
        return testWrong;
    }
    return maniOpenSkeleton(s_gltfSweepSkeleton, size, out).result == mani_success ? testDone
                                                                                   : testWrong;
}

// The work: open, skeleton, then the clip, skin and springs asked for.
static inline TestOutcome GltfSweepWork(maniBuilder* builder, void* context)
{
    const GltfSweep* sweep = context;
    maniGltf* gltf = NULL;
    maniImportResult r = maniOpenGltf(builder, sweep->json, strlen(sweep->json), &gltf);
    TestOutcome outcome = TestOutcomeOf(r.result);
    maniSkeleton skeleton;
    maniGltfOptions options = maniDefaultGltfOptions();
    outcome = outcome == testDone ? GltfSweepSkeleton(builder, gltf, &skeleton) : outcome;
    if (outcome == testDone && sweep->animation != UINT32_MAX)
    {
        r = maniGltfBuildClip(builder, gltf, sweep->animation, &skeleton, &options,
                              &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp},
                              NULL);
        outcome = TestOutcomeOf(r.result);
    }
    if (outcome == testDone && sweep->skin != UINT32_MAX)
    {
        r = maniGltfBuildSkin(builder, gltf, sweep->skin, &skeleton, &options);
        outcome = TestOutcomeOf(r.result);
    }
    if (outcome == testDone && sweep->springs)
    {
        maniSpringSetSource set;
        r = maniGltfReadSprings(gltf, &skeleton, &options, &set);
        outcome = TestOutcomeOf(r.result);
    }
    maniCloseGltf(gltf);
    return outcome;
}

#endif // MAUL_ANIM_TEST_GLTF_SWEEP_H
