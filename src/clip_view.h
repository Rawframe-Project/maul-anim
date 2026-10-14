// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// What clip.c and clip_tracks.c share: the checks of a clip's float and
// discrete tracks, events, markers and names at open.

#ifndef MAUL_ANIM_SRC_CLIP_VIEW_H
#define MAUL_ANIM_SRC_CLIP_VIEW_H

#include "clip_file.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"

#include <stdint.h>

// Checks sections 4 to 10 against the counts the header gave (in clip)
// and fills clip's pointers to them.
maniFileResult maniCheckClipNamed(const uint8_t* const data[MANI_CLIP_SECTIONS],
                                  const uint64_t sizes[MANI_CLIP_SECTIONS],
                                  const uint64_t at[MANI_CLIP_SECTIONS], maniClip* clip);

#endif // MAUL_ANIM_SRC_CLIP_VIEW_H
