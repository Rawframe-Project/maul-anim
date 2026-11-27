// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The mirror map view checked: maniMirrorMapValid for every call that
// takes a map, and its sections for the modules that read them. A
// partner read from the bytes is bounded where it is used, since the
// bytes may have changed since the open.

#ifndef MAUL_ANIM_SRC_MIRROR_VIEW_H
#define MAUL_ANIM_SRC_MIRROR_VIEW_H

#include "bytes.h"
#include "mirror_file.h"
#include "view.h"

#include "maul-anim/mirror.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniMirrorMap.sections.
enum
{
    maniMirrorHeader,
    maniMirrorBones,
    maniMirrorNames,
    maniMirrorStrings,
};

// Whether a view is one an open wrote: its cookie, its counts and axis
// in range and every section within the file it names.
MANI_NODISCARD bool maniMirrorMapValid(const maniMirrorMap* m);

// A section's bytes; the view checked.
static inline const uint8_t* maniMirrorSection(const maniMirrorMap* m, uint32_t section)
{
    return m->base + m->sections[section];
}

// A bone's record; the view checked and the bone within its count.
static inline const uint8_t* maniMirrorBone(const maniMirrorMap* m, uint32_t bone)
{
    return maniMirrorSection(m, maniMirrorBones) + (size_t)bone * MANI_MIRROR_BONE_SIZE;
}

// A bone's partner; the view checked and the bone within its count. A
// partner in bytes changed since the open past the count is the bone
// itself.
static inline uint32_t maniMirrorPartner(const maniMirrorMap* m, uint32_t bone)
{
    uint32_t partner = maniLoadU32(maniMirrorBone(m, bone));
    return partner < m->boneCount ? partner : bone;
}

#endif // MAUL_ANIM_SRC_MIRROR_VIEW_H
