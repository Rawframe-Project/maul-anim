// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion (mani-0022): the move a clip's declared bone makes, taken
// out of the bone at build and kept beside the clip, so the clip plays
// in place and the host moves the character by the moves it reads.

#ifndef MAUL_ANIM_ROOTMOTION_H
#define MAUL_ANIM_ROOTMOTION_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // The axes of a root motion's translation, any of them.
    enum
    {
        maniRootAxisX = 1,
        maniRootAxisY = 2,
        maniRootAxisZ = 4,
    };

    // How much of the bone's rotation a root motion takes.
    typedef uint32_t maniRootRotation;
    enum
    {
        maniRootRotationNone = 0,
        // The twist about one axis (swing-twist): the turn about an up
        // axis, whatever the bone's pitch and roll.
        maniRootRotationTwist = 1,
        maniRootRotationWhole = 2,
    };

    // What a root motion channel is measured from.
    typedef uint32_t maniRootReference;
    enum
    {
        // The bone's rest.
        maniRootReferenceRest = 0,
        // The bone at the clip's first sample.
        maniRootReferenceFirst = 1,
        // The parent's origin and axes.
        maniRootReferenceOrigin = 2,
    };

    // Where a clip's root motion comes from: a bone's translation along
    // some axes and some of its rotation, in its parent's frame (the
    // model space for a root bone), each from a reference.
    typedef struct maniRootMotionSource
    {
        // The bone; the clip must have a track for it.
        uint32_t bone;
        // maniRootAxisX, Y and Z, any of them.
        uint32_t translationAxes;
        maniRootRotation rotation;
        // The twist's axis: 0 x, 1 y, 2 z (its sign changes nothing).
        uint32_t twistAxis;
        maniRootReference translationReference;
        maniRootReference rotationReference;
        // Spread the end's difference from the start over the clip, so
        // the motion ends where it began (against an idle's drift).
        uint32_t loop;
        uint32_t reserved;
    } maniRootMotionSource;

    /// Reads the source a clip's root motion was taken from.
    ///
    /// @param clip       An opened clip.
    /// @param sourceOut  Receives the source; unchanged on false.
    /// @return True when the clip has root motion.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API bool maniClipRootMotionSource(const maniClip* clip, maniRootMotionSource* sourceOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_ROOTMOTION_H
