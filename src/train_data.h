// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's training data (mani-0025), frame by frame of its
// motion database: the normalized features; the Decompressor's targets
// in its output layout (each bone's local translation and first two
// rotation columns, the scaled bones' scales, the root's velocity and
// angular velocity in its own frame, the float tracks, the events'
// states); and the Compressor's inputs, group by group (all bones'
// local translations, then their columns, local velocities, local
// angular velocities, character-space translations, columns, velocities
// and column velocities, the scaled bones' scales, the root's velocity
// and angular velocity, the float tracks and events). Velocities are
// central differences a database frame either side (nearer at a clamped
// entry's ends), as the database's own; character space is a bone's
// model matrix relative to bone 0, turned by bone 0's rotation undone.

#ifndef MAUL_ANIM_SRC_TRAIN_DATA_H
#define MAUL_ANIM_SRC_TRAIN_DATA_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/match.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct maniTrainDataSource
{
    const maniSkeleton* skeleton;
    const maniMotionDatabase* database;
    // The database's clips, in its order.
    const maniClip* const* clips;
    const maniLearnedNameSource* floatTracks;
    const maniLearnedNameSource* events;
    uint32_t floatTrackCount;
    uint32_t eventCount;
    // An event's state is 1 on the frames less than this after it.
    float eventSeconds;
    uint32_t reserved;
} maniTrainDataSource;

// The Compressor's input groups, each normalized by one deviation.
enum
{
    maniTrainGroupPosition = 0,
    maniTrainGroupColumns,
    maniTrainGroupVelocity,
    maniTrainGroupSpin,
    maniTrainGroupCharacterPosition,
    maniTrainGroupCharacterColumns,
    maniTrainGroupCharacterVelocity,
    maniTrainGroupCharacterColumnVelocity,
    maniTrainGroupScale,
    maniTrainGroupRootVelocity,
    maniTrainGroupRootSpin,
    maniTrainGroupExtra,
    maniTrainGroupCount
};

typedef struct maniTrainData
{
    uint32_t frameCount;
    uint32_t featureCount;
    uint32_t boneCount;
    uint32_t scaledBoneCount;
    uint32_t floatTrackCount;
    uint32_t eventCount;
    uint32_t outputCount;
    uint32_t inputCount;
    uint32_t pairCount;
    // Seconds between a database's frames.
    float frameSeconds;
    // Where each input group starts; groupStarts[maniTrainGroupCount] is
    // the input count.
    uint32_t groupStarts[maniTrainGroupCount + 1];
    float* features;
    float* outputs;
    float* inputs;
    uint32_t* scaledBones;
    // Three a bone: frame 0's scales, kept for the bones not scaled.
    float* scales;
    // The first frame of each pair of consecutive frames of one entry.
    uint32_t* pairs;
} maniTrainData;

// The data of a source whose clips and names are the database's (checked
// by the caller). False when memory runs out or sampling fails, with
// nothing held.
bool maniTrainDataBuild(const maniAllocator* allocator, const maniTrainDataSource* source,
                        maniTrainData* dataOut);

// Releases what maniTrainDataBuild allocated.
void maniTrainDataRelease(const maniAllocator* allocator, maniTrainData* data);

#endif // MAUL_ANIM_SRC_TRAIN_DATA_H
