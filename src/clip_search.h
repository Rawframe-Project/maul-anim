// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Folding and the bit rate search by the shell metric (mani-0006).

#ifndef MAUL_ANIM_SRC_CLIP_SEARCH_H
#define MAUL_ANIM_SRC_CLIP_SEARCH_H

#include "clip_plan.h"

#include "maul-anim/base.h"

// Bone channels folded to default or constant by the shell metric, each
// bone's channels in turn.
maniResult maniSearchFold(maniCompressPlan* p);

// Each animated bone channel's rate in each segment by the search.
maniResult maniSearchRates(maniCompressPlan* p);

#endif // MAUL_ANIM_SRC_CLIP_SEARCH_H
