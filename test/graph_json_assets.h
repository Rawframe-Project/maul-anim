// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The assets a graph's JSON form is built against in the graph JSON fuzz
// target and test_graph_nested: a skeleton of three bones,
// three clips named walk, run and wave (the wave additive), a motion
// database named moves of the walk and run, a learned model
// named steps of it and a spring set named hair from the
// spine to the arm, with s_builder and a file buffer.

#ifndef MAUL_ANIM_TEST_GRAPH_JSON_ASSETS_H
#define MAUL_ANIM_TEST_GRAPH_JSON_ASSETS_H

#include "fuzz_model.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_clipFiles[3][8192];
static _Alignas(16) uint8_t s_file[1u << 20];
static _Alignas(16) uint8_t s_databaseFile[8192];
static _Alignas(16) uint8_t s_modelFile[8192];
static _Alignas(16) uint8_t s_setFile[1024];
static maniSpringSet s_set;
static maniMotionDatabase s_database;
static maniLearnedModel s_model;
static maniSkeleton s_skeleton;
static maniClip s_clips[3];
static maniBuilder* s_builder;

static const maniBoneSource kBones[3] = {
    {"hips", 4, -1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"spine", 5, 0, {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
    {"arm", 3, 1, {{0.3f, 0.2f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
};

static const maniGraphClipSource kHost[3] = {{&s_clips[0], "walk", 4, 0, NULL},
                                             {&s_clips[1], "run", 3, 0, NULL},
                                             {&s_clips[2], "wave", 4, 0, NULL}};
static const maniGraphDatabaseSource kHostDatabase[1] = {{&s_database, "moves", 5, 0}};
static const maniGraphModelSource kHostModel[1] = {{&s_model, "steps", 5, 0}};
static const maniGraphSpringSetSource kHostSet[1] = {{&s_set, "hair", 4, 0}};

// A spring from the spine to the arm.
static inline bool MakeSet(void)
{
    maniSpringJointSource joints[2];
    for (uint32_t j = 0; j < 2; ++j)
    {
        joints[j] = (maniSpringJointSource){.bone = j + 1,
                                            .stiffness = 0.5f,
                                            .gravityPower = 1.0f,
                                            .gravityDirection = {0.0f, -1.0f, 0.0f},
                                            .drag = 0.3f,
                                            .limitRotation = {0.0f, 0.0f, 0.0f, 1.0f}};
    }
    maniSpringSource spring = {"arm", 3, 2, joints, NULL, 0, MANI_NO_BONE};
    maniSpringSetSource source = {&s_skeleton, NULL, NULL, &spring, 0, 0, 1, 0.0f, 0.0f, 0};
    size_t size = 0;
    return maniBuildSpringSet(s_builder, &source).result == mani_success &&
           maniCopyBuilt(s_builder, s_setFile, sizeof s_setFile, &size) == mani_success &&
           maniOpenSpringSet(s_setFile, size, &s_set).result == mani_success;
}

// The walk and the run, the hips' position now their channel.
static inline bool MakeDatabase(void)
{
    static const float kNow[1] = {0.0f};
    const maniClip* clips[2] = {&s_clips[0], &s_clips[1]};
    const maniMatchChannelSource channel = {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f};
    const maniMatchEntrySource entries[2] = {{0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u},
                                             {1, 0, 0.0f, 1.0f, 0.0f, 2u}};
    maniMotionDatabaseSource source = {&s_skeleton, clips, &channel, entries,        2,
                                       1,           2,     10.0f,    maniMatchAxisY, maniMatchAxisZ,
                                       NULL,        0,     0};
    size_t size = 0;
    return maniBuildMotionDatabase(s_builder, &source).result == mani_success &&
           maniCopyBuilt(s_builder, s_databaseFile, sizeof s_databaseFile, &size) == mani_success &&
           maniOpenMotionDatabase(s_databaseFile, size, &s_database).result == mani_success;
}

// The skeleton and clips, as test_graph_json makes them, the database,
// the model and the spring set; aborts on a failure.
static inline void SetupGraphAssets(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    size_t size = 0;
    maniSkeletonSource skeleton = {kBones, 3, 0};
    if (maniCreateBuilder(&def, &s_builder) != mani_success ||
        maniBuildSkeleton(s_builder, &skeleton).result != mani_success ||
        maniCopyBuilt(s_builder, s_skeletonFile, sizeof s_skeletonFile, &size) != mani_success ||
        maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result != mani_success)
    {
        abort();
    }
    static const float kTimes[2] = {0.0f, 1.0f};
    static const maniChannelSource kNone = {NULL, NULL, 0, maniInterpolationLinear};
    for (int i = 0; i < 3; ++i)
    {
        float values[6] = {0, 1, 0, (float)(i + 1), 1, 0};
        maniBoneTrackSource track = {
            0, 0, {kTimes, values, 2, maniInterpolationLinear}, kNone, kNone};
        maniClipSource source = {.skeleton = &s_skeleton,
                                 .boneTracks = &track,
                                 .boneTrackCount = 1,
                                 .loop = maniLoopRepeat,
                                 .sampleRate = 10.0f,
                                 .additive = i == 2 ? maniAdditiveRest : maniAdditiveNone};
        if (maniBuildClip(s_builder, &source).result != mani_success ||
            maniCopyBuilt(s_builder, s_clipFiles[i], sizeof s_clipFiles[i], &size) !=
                mani_success ||
            maniOpenClip(s_clipFiles[i], size, &s_clips[i]).result != mani_success)
        {
            abort();
        }
    }
    if (!MakeDatabase() || !MakeSet() ||
        !MakeFuzzModel(s_builder, &s_skeleton, &s_database, s_modelFile, sizeof s_modelFile,
                       &s_model))
    {
        abort();
    }
}

#endif // MAUL_ANIM_TEST_GRAPH_JSON_ASSETS_H
