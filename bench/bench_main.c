// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Timings of the calls a host makes every frame, on the fixed scenes of
// scenes.h: sampling full and compressed clips (random times across the
// clips, and forward through a cursor), blending two poses, the model
// pass, a graph's update and evaluation, a motion database search, a
// spring step and apply, the IK solvers, the skin's palette and its
// linear and dual quaternion jobs over the whole mesh, and the morph
// targets over it. Prints the build's backend, then each case's best
// of five runs in nanoseconds a call. Given case names, runs only those;
// --list prints the names; --count with a case and a number of calls
// runs only those calls, for an instruction counter (the scenes' build
// is the same for any number, so two counts' difference over the
// calls' difference is a call's).

#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "scenes.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum
{
    RUNS = 5
};

#define STEP (1.0f / 60.0f)

static BenchScenes s_scenes;

static void Stop(const char* what)
{
    fprintf(stderr, "bench: %s failed\n", what);
    exit(1);
}

static void Must(maniResult result, const char* what)
{
    if (result != mani_success)
    {
        Stop(what);
    }
}

// Call i's clip and time: the clips in turn, times spread over the
// duration by the golden ratio.
static float RandomTime(uint32_t i)
{
    double t = fmod((double)i * 0.6180339887498949, 1.0);
    return (float)t * BENCH_DURATION;
}

static void SampleFull(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniSampleClip(&s->full[i % BENCH_CLIPS], &s->skeleton, RandomTime(i), s->pose),
             "a full sample");
    }
}

static void SampleCompressed(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniSampleClip(&s->compressed[i % BENCH_CLIPS], &s->skeleton, RandomTime(i), s->pose),
             "a compressed sample");
    }
}

// Forward through a compressed clip at 60 frames a second, around its
// loop.
static void SampleCursor(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        float t = (float)(i % 600u) * STEP;
        Must(maniSampleClipCursor(s->cursor, &s->compressed[2], &s->skeleton, t, s->pose),
             "a cursor sample");
    }
}

static void BlendTwo(BenchScenes* s, uint32_t n)
{
    const maniBlendLayer layers[2] = {{s->layers[0], 0.5f, NULL}, {s->layers[1], 0.5f, NULL}};
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniBlendPoses(&s->skeleton, layers, 2, MANI_BLEND_THRESHOLD, s->pose), "a blend");
    }
}

static void Model(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniComputeModel(s->pose, &s->skeleton), "a model pass");
    }
}

static void GraphUpdate(BenchScenes* s, uint32_t n)
{
    maniGraphEvent events[8];
    uint32_t count = 0;
    maniRootMove move;
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniUpdateGraph(s->instance, STEP, events, 8, &count, &move), "a graph update");
    }
}

static void GraphEvaluate(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniEvaluateGraph(s->instance, s->scratch, s->scratchSize, s->pose),
             "a graph evaluation");
    }
}

static void MatchSearch(BenchScenes* s, uint32_t n)
{
    maniMatchResult result;
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniSearchMotionDatabase(&s->database, &s->queries[i % BENCH_QUERIES], &result),
             "a search");
    }
}

// From the first pose each time: the hair and the tail stepped a frame
// on and applied.
static void Springs(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        BenchRestore(s);
        Must(maniStepSprings(s->springs, s->pose, NULL, NULL, STEP), "a spring step");
        Must(maniApplySprings(s->springs, s->pose), "a spring apply");
    }
}

// Each solve from the first pose.
static void TwoBone(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        BenchRestore(s);
        Must(maniSolveTwoBone(s->pose, &s->skeleton, &s->leg, NULL), "a two-bone solve");
    }
}

static void Aim(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        BenchRestore(s);
        Must(maniSolveAim(s->pose, &s->skeleton, &s->look, NULL), "an aim");
    }
}

static void Ccd(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        BenchRestore(s);
        Must(maniSolveCcd(s->pose, &s->skeleton, &s->tail, NULL), "a CCD solve");
    }
}

static void Fabrik(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        BenchRestore(s);
        Must(maniSolveFabrik(s->pose, &s->skeleton, &s->tail, NULL), "a FABRIK solve");
    }
}

static void Palette(BenchScenes* s, uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(
            maniComputePalette(s->layers[0], &s->skeleton, &s->skin, s->palette, BENCH_BONES, NULL),
            "a palette");
    }
}

// The whole mesh's positions and normals.
static maniSkinJob SkinJob(const BenchScenes* s, maniSkinStream* streams)
{
    streams[0] = (maniSkinStream){s->positions, s->movedPositions, 12, 12, maniStreamPoint, 0};
    streams[1] = (maniSkinStream){s->normals, s->movedNormals, 12, 12, maniStreamNormal, 0};
    return (maniSkinJob){
        &s->influences, s->palette, s->dualQuaternions, BENCH_BONES, 0, BENCH_VERTICES, 2, streams};
}

static void SkinLinear(BenchScenes* s, uint32_t n)
{
    maniSkinStream streams[2];
    maniSkinJob job = SkinJob(s, streams);
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniSkinLinear(&job), "a linear skin");
    }
}

static void SkinDual(BenchScenes* s, uint32_t n)
{
    maniSkinStream streams[2];
    maniSkinJob job = SkinJob(s, streams);
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniSkinDualQuaternion(&job), "a dual quaternion skin");
    }
}

// The targets' weights from the channels', then the whole mesh's
// positions and normals.
static void Morph(BenchScenes* s, uint32_t n)
{
    const maniMorphStream streams[2] = {
        {s->positions, s->movedPositions, 12, 12, maniMorphPosition, 0, 3, 0},
        {s->normals, s->movedNormals, 12, 12, maniMorphNormal, 0, 3, 0}};
    const maniMorphJob job = {&s->morphs, s->targetWeights, 0, BENCH_VERTICES, 2, 0, streams};
    for (uint32_t i = 0; i < n; ++i)
    {
        Must(maniMorphTargetWeights(&s->morphs, s->channelWeights, s->targetWeights),
             "the target weights");
        Must(maniApplyMorphTargets(&job), "a morph");
    }
}

typedef struct Case
{
    const char* name;
    void (*run)(BenchScenes* s, uint32_t n);
    // Calls a run: a few milliseconds' worth.
    uint32_t calls;
} Case;

static const Case kCases[] = {
    {"sample.full.random", SampleFull, 4000},
    {"sample.compressed.random", SampleCompressed, 2000},
    {"sample.compressed.cursor", SampleCursor, 4000},
    {"blend.two", BlendTwo, 8000},
    {"model", Model, 8000},
    {"graph.update", GraphUpdate, 8000},
    {"graph.evaluate", GraphEvaluate, 2000},
    {"match.search", MatchSearch, 400},
    {"spring.step", Springs, 2000},
    {"ik.twobone", TwoBone, 8000},
    {"ik.aim", Aim, 8000},
    {"ik.ccd", Ccd, 2000},
    {"ik.fabrik", Fabrik, 2000},
    {"skin.palette", Palette, 8000},
    {"skin.linear", SkinLinear, 40},
    {"skin.dualquat", SkinDual, 40},
    {"morph.apply", Morph, 40},
};

#define CASE_COUNT (sizeof kCases / sizeof kCases[0])

static const Case* Find(const char* name)
{
    for (size_t k = 0; k < CASE_COUNT; ++k)
    {
        if (strcmp(kCases[k].name, name) == 0)
        {
            return &kCases[k];
        }
    }
    fprintf(stderr, "bench: no case %s (--list names them)\n", name);
    exit(2);
}

static double Seconds(void)
{
    struct timespec now;
    timespec_get(&now, TIME_UTC);
    return (double)now.tv_sec + (double)now.tv_nsec * 1e-9;
}

// The best of five runs, after one to warm up, in nanoseconds a call.
static double Time(const Case* c)
{
    c->run(&s_scenes, c->calls);
    double best = HUGE_VAL;
    for (int r = 0; r < RUNS; ++r)
    {
        double start = Seconds();
        c->run(&s_scenes, c->calls);
        best = fmin(best, Seconds() - start);
    }
    return best * 1e9 / (double)c->calls;
}

int main(int argc, char** argv)
{
    const char* backend = NULL;
    if (maniCheckCpu(&backend) != 1)
    {
        fprintf(stderr, "bench: this CPU cannot run the %s backend\n", backend);
        return 1;
    }
    if (argc == 2 && strcmp(argv[1], "--list") == 0)
    {
        for (size_t k = 0; k < CASE_COUNT; ++k)
        {
            printf("%s\n", kCases[k].name);
        }
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "--count") == 0)
    {
        char* end = NULL;
        unsigned long n = argc == 4 ? strtoul(argv[3], &end, 10) : 0;
        if (argc != 4 || end == argv[3] || *end != '\0' || n == 0 || n > UINT32_MAX)
        {
            fprintf(stderr, "usage: %s --count <case> <calls>\n", argv[0]);
            return 2;
        }
        const Case* c = Find(argv[2]);
        if (!BenchBuild(&s_scenes))
        {
            return 1;
        }
        c->run(&s_scenes, (uint32_t)n);
        return 0;
    }
    for (int a = 1; a < argc; ++a)
    {
        (void)Find(argv[a]);
    }
    if (!BenchBuild(&s_scenes))
    {
        return 1;
    }
    printf("# maul-anim %d.%d.%d, backend %s; nanoseconds a call, the best of %d runs\n",
           MANI_VERSION_MAJOR, MANI_VERSION_MINOR, MANI_VERSION_PATCH, maniGetSimdBackend(), RUNS);
    printf("# %u bones, %u clips of %.0f s, %u vertices, %u morph targets\n", BENCH_BONES,
           BENCH_CLIPS, (double)BENCH_DURATION, BENCH_VERTICES, BENCH_TARGETS);
    for (size_t k = 0; k < CASE_COUNT; ++k)
    {
        bool chosen = argc == 1;
        for (int a = 1; a < argc && !chosen; ++a)
        {
            chosen = strcmp(argv[a], kCases[k].name) == 0;
        }
        if (chosen)
        {
            printf("%-28s %12.1f\n", kCases[k].name, Time(&kCases[k]));
        }
    }
    return 0;
}
