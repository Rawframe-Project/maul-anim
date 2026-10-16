// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The graph suites' fixture (mani-0023): a three-bone skeleton, nine
// clips (walks and runs, a wave of differences, clips with left and
// right markers, turns in opposite hemispheres), and graph sources built,
// opened, bound to every clip and instanced in static memory. Included by
// one suite each.

#ifndef MAUL_ANIM_TEST_GRAPH_FIXTURE_H
#define MAUL_ANIM_TEST_GRAPH_FIXTURE_H

#include "graph_instance.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/blend.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define CLIPS 9

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_clipFiles[CLIPS][8192];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(8) uint8_t s_bindingMemory[256];
static _Alignas(8) uint8_t s_instanceMemory[8192];
static maniSkeleton s_skeleton;
static maniClip s_clips[CLIPS];

// Hips, a spine under them, an arm under the spine.
static const maniBoneSource kBones[3] = {
    {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"spine", 5, 0, {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"arm", 3, 1, {{0.3f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
};

static inline maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// Clips: 0 a walk with its hips' travel as root motion and steps (left
// at 0, right at 0.5) as events, 1 a run, 2 a wave of differences; with
// left and right markers, 3 a step of a second (left at 0, right at
// 0.5), 4 one of two seconds starting between its feet (right at 0.5,
// left at 1.5, an event at 1), 5 two steps in two seconds, 6 a second
// and a half with two lefts (0, 0.5) before a right (1); 7 and 8 the
// spine turned half about z and nearly back, in opposite hemispheres.
static inline void OpenClip(maniBuilder* builder, int i, maniClipSource source)
{
    size_t size = 0;
    source.skeleton = &s_skeleton;
    source.boneTrackCount = 1;
    source.loop = maniLoopRepeat;
    source.sampleRate = 10.0f;
    CHECK(maniBuildClip(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_clipFiles[i], sizeof s_clipFiles[i], &size) ==
                  mani_success &&
              maniOpenClip(s_clipFiles[i], size, &s_clips[i]).result == mani_success,
          "a clip");
}

static inline void Open(void)
{
    static const float kTimes[2] = {0.0f, 1.0f};
    static const float kLongTimes[2] = {0.0f, 2.0f};
    static const float kTripleTimes[2] = {0.0f, 1.5f};
    static const float kWalk[6] = {0, 1, 0, 1, 1, 0};
    static const float kRun[6] = {0, 1, 0, 3, 1, 0};
    static const float kWave[8] = {0, 0, 0, 1, 0, 0, 0.38268343f, 0.92387953f};
    static const float kTurnA[8] = {0, 0, 1, 0, 0, 0, 1, 0};
    static const float kTurnB[8] = {0, 0, -0.9f, 0.43588989f, 0, 0, -0.9f, 0.43588989f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    static const maniRootMotionSource kHips = {0, maniRootAxisX, 0, 0, 0, 0, 0, 0};
    static const maniEventSource kSteps[2] = {{0.0f, 7, "left", 4, 0}, {0.5f, 8, "right", 5, 0}};
    static const maniEventSource kShort[2] = {{0.0f, 0, "left", 4, 0}, {0.5f, 0, "right", 5, 0}};
    static const maniEventSource kLong[2] = {{0.5f, 0, "right", 5, 0}, {1.5f, 0, "left", 4, 0}};
    static const maniEventSource kDouble[4] = {{0.0f, 0, "left", 4, 0},
                                               {0.5f, 0, "right", 5, 0},
                                               {1.0f, 0, "left", 4, 0},
                                               {1.5f, 0, "right", 5, 0}};
    static const maniEventSource kTriple[3] = {
        {0.0f, 0, "left", 4, 0}, {0.5f, 0, "left", 4, 0}, {1.0f, 0, "right", 5, 0}};
    static const maniEventSource kMiddle = {1.0f, 3, "middle", 6, 0};
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource skeleton = {kBones, 3, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &skeleton).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "skeleton");
    const maniBoneTrackSource walk = {
        0, 0, {kTimes, kWalk, 2, maniInterpolationLinear}, kNone, kNone};
    const maniBoneTrackSource run = {
        0, 0, {kTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone};
    const maniBoneTrackSource longRun = {
        0, 0, {kLongTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone};
    const maniBoneTrackSource tripleRun = {
        0, 0, {kTripleTimes, kRun, 2, maniInterpolationLinear}, kNone, kNone};
    const maniBoneTrackSource wave = {
        2, 0, kNone, {kTimes, kWave, 2, maniInterpolationLinear}, kNone};
    const maniBoneTrackSource turnA = {
        1, 0, kNone, {kTimes, kTurnA, 2, maniInterpolationLinear}, kNone};
    const maniBoneTrackSource turnB = {
        1, 0, kNone, {kTimes, kTurnB, 2, maniInterpolationLinear}, kNone};
    OpenClip(builder, 0,
             (maniClipSource){
                 .boneTracks = &walk, .events = kSteps, .eventCount = 2, .rootMotion = &kHips});
    OpenClip(builder, 1, (maniClipSource){.boneTracks = &run});
    OpenClip(builder, 2, (maniClipSource){.boneTracks = &wave, .additive = maniAdditiveRest});
    OpenClip(builder, 3,
             (maniClipSource){.boneTracks = &walk, .markers = kShort, .markerCount = 2});
    OpenClip(builder, 4,
             (maniClipSource){.boneTracks = &longRun,
                              .markers = kLong,
                              .markerCount = 2,
                              .events = &kMiddle,
                              .eventCount = 1});
    OpenClip(builder, 5,
             (maniClipSource){.boneTracks = &longRun, .markers = kDouble, .markerCount = 4});
    OpenClip(builder, 6,
             (maniClipSource){.boneTracks = &tripleRun, .markers = kTriple, .markerCount = 3});
    OpenClip(builder, 7, (maniClipSource){.boneTracks = &turnA});
    OpenClip(builder, 8, (maniClipSource){.boneTracks = &turnB});
    maniDestroyBuilder(builder);
}

static inline maniGraphValue Float(float f)
{
    maniGraphValue v = {{f, 0.0f, 0.0f, 0.0f}};
    return v;
}

static inline maniGraphValue Int(int32_t i)
{
    maniGraphValue v = {{0}};
    v.i = i;
    return v;
}

static inline maniGraphParameterSource Parameter(const char* name, maniGraphParameterType type,
                                                 maniGraphValue value)
{
    return (maniGraphParameterSource){name, (uint32_t)strlen(name), type, value, Int(0), Int(0), 0,
                                      0};
}

#define LITERAL(x) ((maniGraphScalar){MANI_NO_PARAMETER, (x)})

static const maniGraphClipSource kSlots[CLIPS] = {
    {&s_clips[0], "walk", 4, 0},   {&s_clips[1], "run", 3, 0},   {&s_clips[2], "wave", 4, 0},
    {&s_clips[3], "short", 5, 0},  {&s_clips[4], "long", 4, 0},  {&s_clips[5], "double", 6, 0},
    {&s_clips[6], "triple", 6, 0}, {&s_clips[7], "turnA", 5, 0}, {&s_clips[8], "turnB", 5, 0}};

static inline maniGraphNodeSource ClipNode(uint32_t clip, float speed, maniGraphLoop loop)
{
    return (maniGraphNodeSource){maniGraphNodeClip, 0, NULL,
                                 .clip = {clip, MANI_NO_SYNC_GROUP, LITERAL(speed), loop, 0}};
}

// The source built, opened, bound to every clip and instanced.
static inline maniGraphInstance* Make(maniGraphSource source, maniGraph* g)
{
    source.skeleton = &s_skeleton;
    source.clips = kSlots;
    source.clipCount = CLIPS;
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    const maniClip* clips[CLIPS];
    for (uint32_t c = 0; c < CLIPS; ++c)
    {
        clips[c] = &s_clips[c];
    }
    maniGraphBinding* binding = NULL;
    maniGraphInstance* instance = NULL;
    CHECK(maniBuildGraph(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &size) == mani_success &&
              maniOpenGraph(s_file, size, g).result == mani_success &&
              maniBindGraph(g, &s_skeleton, clips, CLIPS, s_bindingMemory, sizeof s_bindingMemory,
                            &binding, NULL) == mani_success &&
              maniInitGraphInstance(binding, s_instanceMemory, sizeof s_instanceMemory,
                                    &instance) == mani_success,
          "a graph");
    maniDestroyBuilder(builder);
    return instance;
}

static inline const maniGraphNodeState* State(const maniGraphInstance* instance, const maniGraph* g,
                                              uint32_t p)
{
    return &instance->cells[g->parameterCount + p].node;
}

static inline bool Near(double a, double b)
{
    return fabs(a - b) <= 1e-6;
}

static inline bool NearF(float a, double b)
{
    return Near((double)a, b);
}

static inline void Update(maniGraphInstance* instance, float delta)
{
    CHECK(maniUpdateGraph(instance, delta, NULL, 0, NULL, NULL) == mani_success, "updates");
}

static inline void Set(maniGraphInstance* instance, uint32_t parameter, maniGraphParameterType type,
                       maniGraphValue value)
{
    CHECK(maniSetGraphParameter(instance, parameter, type, value) == mani_success, "set");
}

#endif // MAUL_ANIM_TEST_GRAPH_FIXTURE_H
