// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A playhead moved through a motion database entry's span (mani-0024),
// for the database's sampling and a motion matching node's play: its
// clip time and the root's move there; the axes and the ground plane the
// database's trajectories lie on.

#ifndef MAUL_ANIM_SRC_MATCH_MOVE_H
#define MAUL_ANIM_SRC_MATCH_MOVE_H

#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/rootmotion.h"

#include <stdbool.h>
#include <stdint.h>

// An entry's span of its clip: its ends, seconds, and whether it loops.
typedef struct maniMatchSpan
{
    float start;
    float end;
    bool loop;
} maniMatchSpan;

// A time placed in a span: the clip time, the times a looping span's end
// (forward) or start (backward) was passed, and the root's move there.
typedef struct maniMatchPlace
{
    float time;
    uint32_t reserved;
    double wraps;
    maniRootMove move;
} maniMatchPlace;

// The place a step (either sign, seconds) from clip time t reaches in a
// span: a clamped span holds it within its ends; a looping one wraps it,
// the root's move composed across (whole spans by doubling).
maniMatchPlace maniMatchLocate(const maniClip* clip, maniMatchSpan span, float t, double step);

// An axis as a unit vector of its sign.
maniVec3 maniMatchAxisVector(maniMatchAxis axis);

// A vector on the ground plane of an up axis: its two components besides
// that axis, in order.
void maniMatchGround(maniVec3 v, maniMatchAxis up, float* out);

#endif // MAUL_ANIM_SRC_MATCH_MOVE_H
