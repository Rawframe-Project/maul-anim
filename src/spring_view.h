// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The spring set view checked: maniSpringSetValid for every call that
// takes a set, and its records for the modules that read them. A
// spring's joint and link ranges and a link's collider, read from the
// bytes, are bounded where they are used, since the bytes may have
// changed since the open.

#ifndef MAUL_ANIM_SRC_SPRING_VIEW_H
#define MAUL_ANIM_SRC_SPRING_VIEW_H

#include "bytes.h"
#include "spring_file.h"
#include "view.h"

#include "maul-anim/spring.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniSpringSet.sections.
enum
{
    maniSpringColliders,
    maniSpringSprings,
    maniSpringJointRecords,
    maniSpringLinks,
    maniSpringNames,
};

// Whether a view is one an open wrote: its cookie, its counts and rates
// in range and every section within the file it names.
MANI_NODISCARD bool maniSpringSetValid(const maniSpringSet* set);

// A section's bytes; the view checked.
static inline const uint8_t* maniSpringSection(const maniSpringSet* set, uint32_t section)
{
    return set->base + set->sections[section];
}

// A collider's record; the view checked and the collider within its
// count.
static inline const uint8_t* maniSpringCollider(const maniSpringSet* set, uint32_t collider)
{
    return maniSpringSection(set, maniSpringColliders) +
           (size_t)collider * MANI_SPRING_COLLIDER_SIZE;
}

// A spring's record; the view checked and the spring within its count.
static inline const uint8_t* maniSpringRecord(const maniSpringSet* set, uint32_t spring)
{
    return maniSpringSection(set, maniSpringSprings) + (size_t)spring * MANI_SPRING_SPRING_SIZE;
}

// A joint's record; the view checked and the joint within its count.
static inline const uint8_t* maniSpringJoint(const maniSpringSet* set, uint32_t joint)
{
    return maniSpringSection(set, maniSpringJointRecords) + (size_t)joint * MANI_SPRING_JOINT_SIZE;
}

// A range of count records from first, cut at total: the bytes may have
// changed since the open.
static inline void maniSpringRange(uint32_t first, uint32_t count, uint32_t total,
                                   uint32_t* firstOut, uint32_t* endOut)
{
    first = first < total ? first : total;
    *firstOut = first;
    *endOut = first + (count < total - first ? count : total - first);
}

// A spring record's joints, [first, end), within the set's joints.
static inline void maniSpringJointsOf(const maniSpringSet* set, const uint8_t* spring,
                                      uint32_t* firstOut, uint32_t* endOut)
{
    maniSpringRange(maniLoadU32(spring), maniLoadU32(spring + 4), set->jointCount, firstOut,
                    endOut);
}

// A spring record's links, [first, end), within the set's links.
static inline void maniSpringLinksOf(const maniSpringSet* set, const uint8_t* spring,
                                     uint32_t* firstOut, uint32_t* endOut)
{
    maniSpringRange(maniLoadU32(spring + 8), maniLoadU32(spring + 12), set->linkCount, firstOut,
                    endOut);
}

// The collider of link k, within the links; UINT32_MAX for one past the
// colliders, which the caller skips.
static inline uint32_t maniSpringLinkCollider(const maniSpringSet* set, uint32_t k)
{
    uint32_t c = maniLoadU32(maniSpringSection(set, maniSpringLinks) + 4 * (size_t)k);
    return c < set->colliderCount ? c : UINT32_MAX;
}

#endif // MAUL_ANIM_SRC_SPRING_VIEW_H
