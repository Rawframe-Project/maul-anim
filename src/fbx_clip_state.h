// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The state of an FBX clip build (mani-0016), shared by the bones'
// sampling (fbx_clip.c) and the float tracks (fbx_tracks.c).

#ifndef MAUL_ANIM_SRC_FBX_CLIP_STATE_H
#define MAUL_ANIM_SRC_FBX_CLIP_STATE_H

#include "arena.h"
#include "basis.h"
#include "fbx_anim.h"
#include "fbx_document.h"
#include "fbx_sample.h"
#include "fbx_world.h"

#include "maul-anim/build.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

typedef struct maniFbxClipState
{
    const maniFbx* fbx;
    const maniSkeleton* skeleton;
    const maniFbxOptions* options;
    maniBasis basis;
    maniArena arena;
    maniFbxAnimation animation;
    maniFbxSampler sampler;
    uint32_t boneCount;
    // Per bone: its model (MANI_FBX_NONE for none) and its rule.
    uint32_t* modelOf;
    uint8_t* rule;
    // Per model: needed for the worlds.
    uint8_t* needed;
    maniFbxWorld* worlds;
    // The rate as the builder takes it, and as the source states it
    // (NTSC's 30000 / 1001 exactly), for the ticks of samples.
    float rate;
    double exactRate;
    float duration;
    uint32_t sampleCount;
    // The builder's sample times, and the poses: sampleCount by boneCount.
    float* times;
    maniTransform* poses;
    maniBoneTrackSource* tracks;
    uint32_t trackCount;
    maniFloatTrackSource* floats;
    uint32_t floatCount;
    // Per float track, its model, for refusals.
    uint32_t* floatModel;
    // The weight bindings of the blend shape channels kept.
    uint32_t blendBound;
} maniFbxClipState;

// Sample k's time in the stack, in seconds: k frames of the exact rate
// in whole ticks from its start, the last at its stop.
double maniFbxClipStackTime(const maniFbxClipState* c, uint32_t k);

#endif // MAUL_ANIM_SRC_FBX_CLIP_STATE_H
