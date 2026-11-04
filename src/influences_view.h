// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The influences view checked: maniInfluencesValid for every call that
// takes influences, and its sections for the modules that read them. A
// joint read from the bytes is bounded where it is used, since the bytes
// may have changed since the open.

#ifndef MAUL_ANIM_SRC_INFLUENCES_VIEW_H
#define MAUL_ANIM_SRC_INFLUENCES_VIEW_H

#include "bytes.h"
#include "view.h"

#include "maul-anim/skinning.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniInfluences.offsets and sizes.
enum
{
    maniInfluencesJoints,
    maniInfluencesWeights,
    maniInfluencesDualWeights,
    maniInfluencesCenters,
};

// Whether a view is one an open wrote: its cookie, its counts in range
// and every section within the file it names, the joints and weights
// holding every vertex's slots, a present optional section every
// vertex's value.
bool maniInfluencesValid(const maniInfluences* v);

// A section's bytes, and whether the file has it; the view checked.
static inline const uint8_t* maniInfluencesSection(const maniInfluences* v, uint32_t section)
{
    return v->base + v->offsets[section];
}

static inline bool maniInfluencesHas(const maniInfluences* v, uint32_t section)
{
    return v->sizes[section] != 0;
}

// Slot k's joint, below the skin's joint count: joint 0 for one in bytes
// changed since the open that is past it, so a palette read stays in the
// palette.
static inline uint32_t maniInfluencesJoint(const maniInfluences* v, uint64_t k)
{
    uint32_t joint = maniLoadU16(maniInfluencesSection(v, maniInfluencesJoints) + 2 * (size_t)k);
    return joint < v->skinJointCount ? joint : 0;
}

#endif // MAUL_ANIM_SRC_INFLUENCES_VIEW_H
