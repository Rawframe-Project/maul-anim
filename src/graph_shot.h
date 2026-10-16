// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// One-shots (mani-0023): a one-shot's data words read and checked on
// open, its block in an instance (phase, fade clock, fade-start weight,
// weight, its leading clip's clip seconds left) reset and checked when a
// saved state restores it.

#ifndef MAUL_ANIM_SRC_GRAPH_SHOT_H
#define MAUL_ANIM_SRC_GRAPH_SHOT_H

#include "maul-anim/graph.h"

#include <stdbool.h>
#include <stdint.h>

// The one-shot whose data words begin at data in a graph.
typedef struct maniGraphShot
{
    const maniGraph* graph;
    uint32_t data;
    uint32_t reserved;
} maniGraphShot;

// Its data words: fire, fade-out and abort triggers, curve, fade-in and
// fade-out seconds, mask, block, clip count; then the clips.
enum
{
    maniGraphShotFire = 0,
    maniGraphShotFadeOutTrigger = 1,
    maniGraphShotAbort = 2,
    maniGraphShotCurve = 3,
    maniGraphShotFadeIn = 4,
    maniGraphShotFadeOut = 5,
    maniGraphShotMask = 6,
    maniGraphShotBlock = 7,
    maniGraphShotClips = 8,
};

uint32_t maniGraphShotWord(const maniGraphShot* s, uint32_t at);
float maniGraphShotFloat(const maniGraphShot* s, uint32_t at);
// The i-th clip node beneath the shot, in node order.
uint32_t maniGraphShotClip(const maniGraphShot* s, uint32_t i);

// A block's words, and its phases.
enum
{
    maniGraphShotPhase = 0,
    maniGraphShotElapsed = 1,
    maniGraphShotStart = 2,
    maniGraphShotWeight = 3,
    maniGraphShotRemaining = 4,
};

enum
{
    maniGraphShotIdle = 0,
    maniGraphShotIn = 1,
    maniGraphShotPlaying = 2,
    maniGraphShotOut = 3,
};

// Idle at weight 0, nothing left to play.
void maniGraphResetShot(uint32_t* block);

// The first wrong word of a saved block (little-endian words at bytes),
// or UINT32_MAX.
uint32_t maniGraphSavedShotValid(const uint8_t* bytes);

// The data words the one-shot at data word data takes, or UINT32_MAX when
// they do not fit in available words.
uint32_t maniGraphShotDataWords(const maniGraph* g, uint32_t data, uint32_t available);

// The first wrong data word of one-shot node p whose block starts at
// block, or UINT32_MAX; its words counted already and the nodes before it
// checked.
uint32_t maniGraphShotValid(const maniGraph* g, uint32_t p, uint32_t data, uint32_t block);

#endif // MAUL_ANIM_SRC_GRAPH_SHOT_H
