// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The retargeting map view checked: maniRetargetMapValid for every call
// that takes a map, and its sections for the modules that read them. A
// source bone read from a plan is bounded where it is used, since the
// bytes may have changed since the open.

#ifndef MAUL_ANIM_SRC_RETARGET_VIEW_H
#define MAUL_ANIM_SRC_RETARGET_VIEW_H

#include "retarget_file.h"
#include "view.h"

#include "maul-anim/retarget.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniRetargetMap.sections.
enum
{
    maniRetargetSources,
    maniRetargetTargets,
    maniRetargetPlans,
    maniRetargetPelvisSection,
};

// The pelvis of a map without one: MANI_NO_BONE both, the axis +y, the
// scales and the ratio 1, every other number 0.
extern const uint8_t maniRetargetNoPelvis[MANI_RETARGET_PELVIS_SIZE];

// Whether a view is one an open wrote: its cookie, its counts in range
// and every section within the file it names.
MANI_NODISCARD bool maniRetargetMapValid(const maniRetargetMap* m);

// A section's bytes; the view checked.
static inline const uint8_t* maniRetargetSection(const maniRetargetMap* m, uint32_t section)
{
    return m->base + m->sections[section];
}

// The pelvis record, or maniRetargetNoPelvis; the view checked.
static inline const uint8_t* maniRetargetPelvisOf(const maniRetargetMap* m)
{
    return m->pelvis != 0 ? maniRetargetSection(m, maniRetargetPelvisSection)
                          : maniRetargetNoPelvis;
}

// A source bone's record; the view checked and the bone within the
// source's count.
static inline const uint8_t* maniRetargetSourceBone(const maniRetargetMap* m, uint32_t b)
{
    return maniRetargetSection(m, maniRetargetSources) + (size_t)b * MANI_RETARGET_SOURCE_SIZE;
}

// A target bone's record; the view checked and the bone within the
// target's count.
static inline const uint8_t* maniRetargetTargetBone(const maniRetargetMap* m, uint32_t b)
{
    return maniRetargetSection(m, maniRetargetTargets) + (size_t)b * MANI_RETARGET_TARGET_SIZE;
}

// A target bone's plan; the view checked and the bone within the
// target's count.
static inline const uint8_t* maniRetargetPlan(const maniRetargetMap* m, uint32_t b)
{
    return maniRetargetSection(m, maniRetargetPlans) + (size_t)b * MANI_RETARGET_PLAN_SIZE;
}

// A source bone read from a plan; the view checked. One past the
// source's bones, in bytes changed since the open, is the source's root.
static inline uint32_t maniRetargetSourceOf(const maniRetargetMap* m, uint32_t a)
{
    return a < m->sourceBoneCount ? a : 0;
}

#endif // MAUL_ANIM_SRC_RETARGET_VIEW_H
