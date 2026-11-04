// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The morph targets view checked: maniMorphTargetsValid for every call
// that takes morph targets, and its sections for the modules that read
// them. What a call reads from the bytes (a channel's targets, a target's
// deltas and indices, a sparse vertex, a name's span) is bounded where it
// is used, since the bytes may have changed since the open.

#ifndef MAUL_ANIM_SRC_MORPH_VIEW_H
#define MAUL_ANIM_SRC_MORPH_VIEW_H

#include "bytes.h"
#include "morph_file.h"
#include "view.h"

#include "maul-anim/morph.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Whether a view is one an open wrote: its cookie, its counts in range
// and every section within the file it names, the fixed ones holding
// their counts' entries.
bool maniMorphTargetsValid(const maniMorphTargets* m);

// A section by its tag, maniMorphTagAttributes to maniMorphTagNames: its
// bytes and size. The view checked.
static inline const uint8_t* maniMorphSection(const maniMorphTargets* m, uint32_t tag)
{
    return m->base + m->offsets[tag - maniMorphTagAttributes];
}

static inline uint64_t maniMorphSectionSize(const maniMorphTargets* m, uint32_t tag)
{
    return m->sizes[tag - maniMorphTagAttributes];
}

// An attribute's semantic and set, or -1 and set 0 past the table. The
// view checked.
static inline maniMorphSemantic maniMorphAttribute(const maniMorphTargets* m, uint32_t attribute,
                                                   uint32_t* setOut)
{
    if (attribute >= m->attributeCount)
    {
        if (setOut != NULL)
        {
            *setOut = 0;
        }
        return -1;
    }
    const uint8_t* e =
        maniMorphSection(m, maniMorphTagAttributes) + (size_t)attribute * MANI_MORPH_ATTRIBUTE_SIZE;
    if (setOut != NULL)
    {
        *setOut = maniLoadU32(e + 4);
    }
    return (maniMorphSemantic)maniLoadU32(e);
}

// A target's entry, the view checked and the target below its count.
static inline const uint8_t* maniMorphTargetEntry(const maniMorphTargets* m, uint32_t target)
{
    return maniMorphSection(m, maniMorphTagTargets) + (size_t)target * MANI_MORPH_TARGET_SIZE;
}

#endif // MAUL_ANIM_SRC_MORPH_VIEW_H
