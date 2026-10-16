// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A motion database frame's features, raw (not yet normalized), sampled
// from its entry's clip in the root's space at the frame (mani-0024).

#ifndef MAUL_ANIM_SRC_MATCH_SAMPLE_H
#define MAUL_ANIM_SRC_MATCH_SAMPLE_H

#include "match_move.h"

#include "maul-anim/clip.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stdint.h>

// What sampling a frame needs: the source, a pose of its skeleton (its
// model kept for the clip and time it last sampled), room for the most
// float tracks a clip has, and per channel per entry the float track's
// index in that entry's clip (channel-major, entries along; UINT32_MAX
// for other kinds).
typedef struct maniMatchSampler
{
    const maniMotionDatabaseSource* source;
    maniPose* pose;
    float* floats;
    uint32_t floatCapacity;
    float sampledTime;
    const maniClip* sampledClip;
    const uint32_t* tracks;
} maniMatchSampler;

// Writes the raw features of entry e's frame at clip time t, channel by
// channel, each time in order. False when sampling a clip fails.
bool maniMatchSampleFrame(maniMatchSampler* sampler, uint32_t e, float t, float* out);

#endif // MAUL_ANIM_SRC_MATCH_SAMPLE_H
