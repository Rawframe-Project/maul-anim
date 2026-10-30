// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Root motion: the move a clip's declared bone makes, taken
// out of the bone at build and kept beside the clip, so the clip plays
// in place and the host moves the character by the moves it reads.

#ifndef MAUL_ANIM_ROOTMOTION_H
#define MAUL_ANIM_ROOTMOTION_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

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
    } maniRootMotionSource;

    // A move: where the character goes and how it turns, in its own frame
    // where the move begins (unit rotation).
    typedef struct maniRootMove
    {
        maniVec3 translation;
        maniQuat rotation;
    } maniRootMove;

    /// Returns the move of no motion.
    ///
    /// @return Translation 0, rotation identity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniRootMove maniMakeRootMoveIdentity(void);

    /// Gives the move a clip's root motion makes from a playhead time over
    /// a signed step, in the character's frame where the move begins: for
    /// a clamped clip between the two times clamped to the clip; for a
    /// repeating clip across every wrap between the unwrapped times, the
    /// whole period's move composed by squaring (at most 2^32 periods),
    /// forward or backward. A clip without root motion moves nothing.
    ///
    /// @param clip     An opened clip, of either form.
    /// @param time     The playhead's time, seconds (a repeating clip's
    ///                 taken within the period).
    /// @param step     The step, seconds, either sign.
    /// @param moveOut  Receives the move.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a time or step not finite (nothing written).
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniClip_GetRootMotion(const maniClip* clip, float time,
                                                              float step, maniRootMove* moveOut);

    /// Composes two moves: first, then then from where first ends.
    ///
    /// @param first  The first move.
    /// @param then   The move after it, in the frame first ends in.
    /// @return The whole move, in first's frame.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniRootMove maniComposeRootMotion(maniRootMove first, maniRootMove then);

    /// Blends moves as poses are blended: the weights normalized, the
    /// translations' weighted sum, the rotations' normalized weighted sum
    /// with each turned into the hemisphere of the first weighted one.
    /// Weights of 0 count for nothing; all 0 give the move of no motion.
    ///
    /// @param moves    count moves.
    /// @param weights  count weights, finite and not negative.
    /// @param count    The number of moves.
    /// @param moveOut  Receives the blend.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument
    /// with a count, a weight negative or not finite, or a move not
    /// finite (nothing written).
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniBlendRootMotion(const maniRootMove* moves,
                                                           const float* weights, uint32_t count,
                                                           maniRootMove* moveOut);

    /// Puts a clip's root motion back onto its bone, in a pose sampled
    /// from the clip at the same time, for a host that keeps the motion
    /// in the pose: the bone's local becomes the motion at the time
    /// composed with it (translation m_t + m_r t, rotation m_r r, scale
    /// kept), and the model is marked stale. The time is clamped to
    /// [0, duration] (NaN is 0), as sampling clamps it. A clip without root
    /// motion leaves the pose unchanged.
    ///
    /// @param clip      An opened clip.
    /// @param skeleton  The clip's skeleton.
    /// @param time      The time in seconds.
    /// @param pose      A pose of the skeleton, not a difference pose.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton that is not the clip's or the pose's, or a difference
    /// pose (nothing written).
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniClip_RestoreRootMotion(const maniClip* clip,
                                                                  const maniSkeleton* skeleton,
                                                                  float time, maniPose* pose);

    /// Reads the source a clip's root motion was taken from.
    ///
    /// @param clip       An opened clip.
    /// @param sourceOut  Receives the source; zeroed on false.
    /// @return True when the clip has root motion.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API bool maniClip_GetRootMotionSource(const maniClip* clip,
                                               maniRootMotionSource* sourceOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_ROOTMOTION_H
