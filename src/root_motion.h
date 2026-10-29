// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion's runtime parts: the source's check and the
// section's check at open.

#ifndef MAUL_ANIM_SRC_ROOT_MOTION_H
#define MAUL_ANIM_SRC_ROOT_MOTION_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/rootmotion.h"

#include <stdbool.h>
#include <stdint.h>

// The source's fields in range and a channel chosen.
bool maniRootMotionSourceValid(const maniRootMotionSource* source);

// Checks section 16 (size 0 for none) against the clip's header and
// tracks and sets clip->rootMotion; entry is where its size is in the
// table.
maniFileResult maniCheckClipRootMotion(const uint8_t* data, uint64_t size, uint64_t at,
                                       uint64_t entry, maniClip* clip);

// A weighted sum of moves as maniBlendRootMotion takes it: translations
// in binary64, rotations turned into the first weighted one's hemisphere.
typedef struct maniRootMotionSum
{
    double translation[3];
    double rotation[4];
    double total;
    maniQuat lead;
    bool led;
} maniRootMotionSum;

// Adds a finite move at a weight above 0.
void maniRootMotionSumAdd(maniRootMotionSum* sum, const maniRootMove* move, double weight);

// The blend: the move of no motion when nothing was added.
maniRootMove maniRootMotionSumResult(const maniRootMotionSum* sum);

// maniClip_GetRootMotion for a caller already in the library's
// floating-point environment (fpenv.h), without entering it again.
maniResult maniClipRootMotionEntered(const maniClip* clip, float time, float step,
                                     maniRootMove* moveOut);

#endif // MAUL_ANIM_SRC_ROOT_MOTION_H
