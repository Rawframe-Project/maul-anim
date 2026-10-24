// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The trainer the checkpoint fuzz target restores into, and
// test_learned_train --seed writes its seed from: a bone moving along x
// over a looping clip of a second, a database of its position now at 10
// frames a second, a model of 2 latent values and hidden layers of 4,
// batches of 4, windows of 2 and 3 iterations a stage, one Projector of
// every frame. Included by one file each.

#ifndef MAUL_ANIM_TEST_CHECKPOINT_HOST_H
#define MAUL_ANIM_TEST_CHECKPOINT_HOST_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/learnedtrain.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct CheckpointHost
{
    _Alignas(16) uint8_t skeletonFile[1024];
    _Alignas(16) uint8_t clipFile[4096];
    _Alignas(16) uint8_t databaseFile[8192];
    maniSkeleton skeleton;
    maniClip clip;
    maniMotionDatabase database;
    const maniClip* clips[1];
} CheckpointHost;

static inline bool HostCopied(maniBuilder* builder, uint8_t* out, size_t capacity, size_t* size)
{
    return maniCopyBuilt(builder, out, capacity, size) == mani_success;
}

static inline bool HostAssets(CheckpointHost* h, maniBuilder* builder)
{
    static const maniBoneSource kBone = {
        "root", 4, -1, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}};
    static const float kTimes[3] = {0.0f, 0.5f, 1.0f};
    static const float kValues[9] = {0, 0, 0, 1, 0, 0.5f, 0, 0, 0};
    static const float kNow[1] = {0.0f};
    maniSkeletonSource skeleton = {&kBone, 1, 0};
    const maniChannelSource none = {NULL, NULL, 0, maniInterpolationLinear};
    const maniBoneTrackSource track = {
        0, 0, {kTimes, kValues, 3, maniInterpolationLinear}, none, none};
    maniClipSource clip = {.skeleton = &h->skeleton, .boneTracks = &track, .boneTrackCount = 1};
    const maniMatchChannelSource channel = {maniMatchBonePosition, 0, 0, 1, kNow, NULL, 0, 1.0f};
    const maniMatchEntrySource entry = {0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u};
    maniMotionDatabaseSource database = {
        &h->skeleton, h->clips,       &channel,       &entry, 1, 1, 1,
        10.0f,        maniMatchAxisY, maniMatchAxisZ, NULL};
    size_t size = 0;
    h->clips[0] = &h->clip;
    return maniBuildSkeleton(builder, &skeleton).result == mani_success &&
           HostCopied(builder, h->skeletonFile, sizeof h->skeletonFile, &size) &&
           maniOpenSkeleton(h->skeletonFile, size, &h->skeleton).result == mani_success &&
           maniBuildClip(builder, &clip).result == mani_success &&
           HostCopied(builder, h->clipFile, sizeof h->clipFile, &size) &&
           maniOpenClip(h->clipFile, size, &h->clip).result == mani_success &&
           maniBuildMotionDatabase(builder, &database).result == mani_success &&
           HostCopied(builder, h->databaseFile, sizeof h->databaseFile, &size) &&
           maniOpenMotionDatabase(h->databaseFile, size, &h->database).result == mani_success;
}

// The host's trainer, or NULL.
static inline maniLearnedTrainer* MakeCheckpointHost(CheckpointHost* h)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        return NULL;
    }
    bool made = HostAssets(h, builder);
    maniDestroyBuilder(builder);
    static const maniLearnedTagFilter kEvery = {0, 0};
    maniLearnedTrainerSource source = {.skeleton = &h->skeleton,
                                       .database = &h->database,
                                       .clips = h->clips,
                                       .projectors = &kEvery,
                                       .clipCount = 1,
                                       .projectorCount = 1,
                                       .options = {2, 4, 1, 1, 1, 1, 4, 2, 3, 7, 0.001f, 0.1f}};
    maniLearnedTrainerDef trainerDef = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* trainer = NULL;
    return made && maniCreateLearnedTrainer(&trainerDef, &source, &trainer).result == mani_success
               ? trainer
               : NULL;
}

#endif // MAUL_ANIM_TEST_CHECKPOINT_HOST_H
