// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion's runtime parts (mani-0022): the source's check and the
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

#endif // MAUL_ANIM_SRC_ROOT_MOTION_H
