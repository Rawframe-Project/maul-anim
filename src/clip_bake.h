// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Baking a clip into another (mani-0032, mani-0033): each sample of a
// clip carried by a kind's pose carry into linear keys at the same times
// (an additive clip's differences added to the rest, carried, and taken
// from the rest carried), channels at the rest left out, float tracks
// sampled, discrete tracks, events and markers copied (renamed by the
// kind), the root motion's moves carried by the kind, then built as any
// clip with its moves given.

#ifndef MAUL_ANIM_SRC_CLIP_BAKE_H
#define MAUL_ANIM_SRC_CLIP_BAKE_H

#include "builder.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

// What a bake takes from its kind. The callbacks get context.
typedef struct maniClipBakeKind
{
    const void* context;
    // The clip, of from; the clip made is of to.
    const maniClip* clip;
    const maniSkeleton* from;
    const maniSkeleton* to;
    // The root motion's source for the clip made (its bone one of to's),
    // when the clip has root motion.
    maniRootMotionSource declared;
    // The scratch the pose carry takes, in bytes.
    size_t scratchSize;
    // Makes out, a pose of to, from in, a pose of from.
    maniResult (*carry)(const void* context, const maniPose* in, maniPose* out, void* scratch,
                        size_t scratchSize);
    // A stored move carried.
    maniRootMove (*move)(const void* context, maniRootMove move);
    // A name's partner (length in, its length out); NULL keeps names.
    const char* (*rename)(const void* context, const char* name, uint32_t* length);
} maniClipBakeKind;

// Bakes the clip and keeps it in the builder; maniBuildReasonBasis when
// a carry fails (an additive clip's difference not finite),
// mani_errorCapacity when memory runs out, else what the clip build
// returns.
maniBuildResult maniBakeClip(maniBuilder* builder, const maniClipBakeKind* kind);

// The axis (0 to 2) a vector of length above 0 lies along, within a
// ten-thousandth; 3 for none.
uint32_t maniBakeAxisOf(maniVec3 v);

// The unit vector along axis 0 to 2.
maniVec3 maniBakeUnit(uint32_t axis);

#endif // MAUL_ANIM_SRC_CLIP_BAKE_H
