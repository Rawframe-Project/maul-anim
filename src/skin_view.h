// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The skin view checked: maniSkinValid for every call that takes a skin,
// and its sections for the modules that read them. A joint's bone read
// from the bytes is bounded where it is used, since the bytes may have
// changed since the open.

#ifndef MAUL_ANIM_SRC_SKIN_VIEW_H
#define MAUL_ANIM_SRC_SKIN_VIEW_H

#include "bytes.h"
#include "view.h"

#include "maul-anim/skin.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniSkin.sections.
enum
{
    maniSkinJoints,
    maniSkinInverseBinds,
};

// Whether a view is one an open wrote: its cookie, its counts in range
// and both sections within the file it names.
MANI_NODISCARD bool maniSkinValid(const maniSkin* s);

// A section's bytes; the view checked.
static inline const uint8_t* maniSkinSection(const maniSkin* s, uint32_t section)
{
    return s->base + s->sections[section];
}

// A joint's bone, the view checked: MANI_NO_BONE for a bone in bytes
// changed since the open that is past the skeleton's count.
static inline uint32_t maniSkinJointBone(const maniSkin* s, uint32_t joint)
{
    uint32_t bone = maniLoadU16(maniSkinSection(s, maniSkinJoints) + 2 * (size_t)joint);
    return bone < s->skeletonBoneCount ? bone : MANI_NO_BONE;
}

#endif // MAUL_ANIM_SRC_SKIN_VIEW_H
