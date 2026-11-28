// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The ragdoll profile view checked: maniRagdollValid for every call that
// takes a profile, and its bodies for the modules that read them. A
// body's parent read from the bytes is bounded where it is used, since
// the bytes may have changed since the open.

#ifndef MAUL_ANIM_SRC_RAGDOLL_VIEW_H
#define MAUL_ANIM_SRC_RAGDOLL_VIEW_H

#include "bytes.h"
#include "ragdoll_file.h"
#include "view.h"

#include "maul-anim/ragdoll.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The sections, by their index in maniRagdoll.sections.
enum
{
    maniRagdollBodies,
    maniRagdollNames,
};

// Whether a view is one an open wrote: its cookie, its counts in range
// and both sections within the file it names.
MANI_NODISCARD bool maniRagdollValid(const maniRagdoll* r);

// A body's record; the view checked and the body within its count.
static inline const uint8_t* maniRagdollRecord(const maniRagdoll* r, uint32_t body)
{
    return r->base + r->sections[maniRagdollBodies] + (size_t)body * MANI_RAGDOLL_BODY_SIZE;
}

// A body's parent; the view checked and the body within its count. The
// root's is MANI_NO_BONE; another body's parent in bytes changed since the
// open that is not before it is the root body.
static inline uint32_t maniRagdollParentOf(const maniRagdoll* r, uint32_t body)
{
    uint32_t parent = maniLoadU32(maniRagdollRecord(r, body) + maniRagdollBodyParent);
    return body == 0 ? MANI_NO_BONE : parent < body ? parent : 0;
}

#endif // MAUL_ANIM_SRC_RAGDOLL_VIEW_H
