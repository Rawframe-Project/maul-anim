// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Reading a graph's JSON form (mani-0023) against a fixed skeleton,
// three clips named walk, run and wave, a motion database named moves
// of the walk and run (mani-0024) and a learned model named steps of it
// (mani-0025): any text is refused or built, and a graph built opens.
// Run as fuzz_graph_json <corpus dir>; the seed comes from
// test_graph_json --seed <file>.

#include "fuzz_model.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_clipFiles[3][8192];
static _Alignas(16) uint8_t s_file[1u << 20];
static _Alignas(16) uint8_t s_databaseFile[8192];
static _Alignas(16) uint8_t s_modelFile[8192];
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

static const maniGraphClipSource kHost[3] = {
    {&s_clips[0], "walk", 4, 0}, {&s_clips[1], "run", 3, 0}, {&s_clips[2], "wave", 4, 0}};
static const maniGraphDatabaseSource kHostDatabase[1] = {{&s_database, "moves", 5, 0}};
static const maniGraphModelSource kHostModel[1] = {{&s_model, "steps", 5, 0}};

// The walk and the run, the hips' position now their channel.
static bool MakeDatabase(void)
{
    static const float kNow[1] = {0.0f};
    const maniClip* clips[2] = {&s_clips[0], &s_clips[1]};
    const maniMatchChannelSource channel = {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f};
    const maniMatchEntrySource entries[2] = {{0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u},
                                             {1, 0, 0.0f, 1.0f, 0.0f, 2u}};
    maniMotionDatabaseSource source = {&s_skeleton, clips, &channel,       entries,       2, 1,
                                       2,           10.0f, maniMatchAxisY, maniMatchAxisZ};
    size_t size = 0;
    return maniBuildMotionDatabase(s_builder, &source).result == mani_success &&
           maniCopyBuilt(s_builder, s_databaseFile, sizeof s_databaseFile, &size) == mani_success &&
           maniOpenMotionDatabase(s_databaseFile, size, &s_database).result == mani_success;
}

// The skeleton and clips, as test_graph_json makes them.
static void Setup(void)
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
    if (!MakeDatabase() || !MakeFuzzModel(s_builder, &s_skeleton, &s_database, s_modelFile,
                                          sizeof s_modelFile, &s_model))
    {
        abort();
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (s_builder == NULL)
    {
        Setup();
    }
    maniBuildResult r = maniBuildGraphJson(
        s_builder, (const char*)data, size,
        &(maniGraphNamedAssets){&s_skeleton, kHost, kHostDatabase, kHostModel, 3, 1, 1, 0});
    if (r.result != mani_success)
    {
        bool text = r.reason == maniBuildReasonJson || r.reason == maniBuildReasonForm;
        if (r.result == mani_errorInvalid && text && r.item > size)
        {
            abort();
        }
        return 0;
    }
    size_t fileSize = 0;
    maniGraph graph;
    if (maniCopyBuilt(s_builder, s_file, sizeof s_file, &fileSize) == mani_success &&
        maniOpenGraph(s_file, fileSize, &graph).result != mani_success)
    {
        abort();
    }
    return 0;
}
