// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The motion database view checked: maniMotionDatabaseValid for every
// call that takes a database, and its records for the modules that read
// them. A frame's place, a position's frame and entry and a node's second
// half, read from the bytes, are bounded where they are used, since the
// bytes may have changed since the open.

#ifndef MAUL_ANIM_SRC_MATCH_VIEW_H
#define MAUL_ANIM_SRC_MATCH_VIEW_H

#include "bytes.h"
#include "match_file.h"
#include "view.h"

#include "maul-anim/match.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniMotionDatabase.sections.
enum
{
    maniMatchChannels,
    maniMatchTimes,
    maniMatchClips,
    maniMatchEntries,
    maniMatchNormalization,
    maniMatchFeatures,
    maniMatchBoxes,
    maniMatchNames,
    maniMatchNodes,
    maniMatchPositions,
    maniMatchPlaces,
};

// Whether a view is one an open wrote: its cookie, its counts, rate and
// leaf size in range and every section within the file it names.
MANI_NODISCARD bool maniMotionDatabaseValid(const maniMotionDatabase* database);

// A section's bytes; the view checked.
static inline const uint8_t* maniMatchSection(const maniMotionDatabase* d, uint32_t section)
{
    return d->base + d->sections[section];
}

// An entry's record; the view checked and the entry within its count.
static inline const uint8_t* maniMatchEntryRecord(const maniMotionDatabase* d, uint32_t entry)
{
    return maniMatchSection(d, maniMatchEntries) + (size_t)entry * MANI_MATCH_ENTRY_SIZE;
}

// A node's record; the view checked and the node within its count.
static inline const uint8_t* maniMatchNodeRecord(const maniMotionDatabase* d, uint32_t node)
{
    return maniMatchSection(d, maniMatchNodes) + (size_t)node * MANI_MATCH_NODE_SIZE;
}

// A frame's position in the tree's order, within the frames: the last
// for a place past them. The view checked and the frame within its count.
static inline uint32_t maniMatchPlaceOf(const maniMotionDatabase* d, uint32_t frame)
{
    uint32_t p = maniLoadU32(maniMatchSection(d, maniMatchPlaces) + 4 * (size_t)frame);
    return p < d->frameCount ? p : d->frameCount - 1;
}

// A position's record: its frame and entry, which the caller bounds.
static inline const uint8_t* maniMatchPositionRecord(const maniMotionDatabase* d, uint32_t p)
{
    return maniMatchSection(d, maniMatchPositions) + (size_t)p * MANI_MATCH_POSITION_SIZE;
}

// A position's entry, within the entries; UINT32_MAX for one past them,
// which the caller skips.
static inline uint32_t maniMatchPositionEntry(const maniMotionDatabase* d, uint32_t p)
{
    uint32_t e = maniLoadU32(maniMatchPositionRecord(d, p) + 4);
    return e < d->entryCount ? e : UINT32_MAX;
}

// A frame's features, normalized; the frame within its count.
static inline const uint8_t* maniMatchFrameFeatures(const maniMotionDatabase* d, uint32_t frame)
{
    return maniMatchSection(d, maniMatchFeatures) +
           4 * (size_t)maniMatchPlaceOf(d, frame) * d->featureCount;
}

#endif // MAUL_ANIM_SRC_MATCH_VIEW_H
