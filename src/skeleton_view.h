// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The skeleton view checked: maniSkeletonValid for every call that takes
// a skeleton, and its sections' bytes for the modules that read them. An
// index read from the bytes is bounded where it is used, since the bytes
// may have changed since the open.

#ifndef MAUL_ANIM_SRC_SKELETON_VIEW_H
#define MAUL_ANIM_SRC_SKELETON_VIEW_H

#include "container.h"
#include "skeleton_file.h"
#include "view.h"

#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniSkeleton.sections.
enum
{
    maniSkeletonParents,
    maniSkeletonSubtreeEnds,
    maniSkeletonRests,
    maniSkeletonNameHashes,
    maniSkeletonNameOrder,
    maniSkeletonNameSpans,
    maniSkeletonNameBytes,
};

// Whether a view is one an open wrote: its cookie, a bone count in range
// and every section within the file it names.
bool maniSkeletonValid(const maniSkeleton* s);

// A section's bytes; the view checked.
static inline const uint8_t* maniSkeletonSection(const maniSkeleton* s, uint32_t section)
{
    return s->base + s->sections[section];
}

// For a call that checked the view on entry, its getters without the
// check again: a bone's parent, MANI_NO_BONE for a root, a bone past the
// count or a parent in the bytes not a lower index; its rest, the
// identity past the count.
uint32_t maniSkeletonParentOf(const maniSkeleton* s, uint32_t bone);
maniTransform maniSkeletonRestOf(const maniSkeleton* s, uint32_t bone);

#endif // MAUL_ANIM_SRC_SKELETON_VIEW_H
