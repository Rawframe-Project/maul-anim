// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Full-body inverse kinematics: a part of a skeleton, every
// bone on a path from a top bone to one of several effectors, bent so
// the effectors reach their goals, within per-bone stiffness and limits.
// One description, two solvers over it: position-based rigid bodies
// with XPBD compliance (fast, stiffness meaning the same at any
// iteration count as the solve converges) and selectively damped least
// squares (Buss and Kim; every effector in one least-squares step, no
// damping constant). Both start from the input pose on every call and
// keep nothing between calls; the caller lends the scratch memory. As
// the other solvers (ik.h), they read the pose's locals, compose the
// model frames of the bones they need, write the locals of the bones
// they move and leave the model stale. Goals are in the model space of
// the pose; angles and axes fixed to a bone are in its parent's frame
// unless stated.

#ifndef MAUL_ANIM_BODYIK_H
#define MAUL_ANIM_BODYIK_H

#include "maul-anim/base.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The most bones a full-body solve moves: the top and every bone on a
// path from it to an effector.
#define MANI_MAX_BODY_IK_BONES 128u
// The most effectors of a full-body solve.
#define MANI_MAX_BODY_IK_EFFECTORS 32u

    // How the top bone of a full-body solve moves.
    typedef int32_t maniBodyTopMode;
    enum
    {
        // Held at its input translation and rotation.
        maniBodyTopPinned = 0,
        // Turned and moved as any other bone (its settings apply).
        maniBodyTopFree = 1,
        // Held, after first moving by the strength-weighted mean of the
        // misses of the effectors whose goals lie past their reach.
        maniBodyTopPrePull = 2,
    };

    // An effector: a bone of the solved part and its goals.
    typedef struct maniBodyEffector
    {
        uint32_t bone;
        // Where the bone's origin should be, in model space.
        maniVec3 position;
        // The bone's wanted model rotation.
        maniQuat rotation;
        // In [0, 1]: how far the position goal lies from the bone's input
        // origin toward position, 0 for the input's.
        float positionWeight;
        // In [0, 1]: how far the rotation goal lies from the bone's input
        // rotation toward rotation, 0 for none.
        float rotationWeight;
        // In [0, 1]: how hard the effector pulls; 0 pulls nothing, 1 as
        // hard as the joints hold.
        float strength;
        // In [0, 1]: the share of the shortest arc toward the goal by
        // which the effector's chain (the bone up to the nearest branch
        // or the top) turns before the solve.
        float pullChain;
    } maniBodyEffector;

    // Settings of one bone of the solved part; a bone without settings
    // is free, unlimited and has no preferred angles.
    typedef struct maniBodyBone
    {
        uint32_t bone;
        // Its limit, on its local rotation relative to its rest's, as a
        // chain's (ik.h): the axis is the direction of its child on the
        // solved part at rest, none for a bone with several or none.
        maniJointLimit limit;
        // How the bone bends as its effectors' goals come closer than in
        // the input: a turn (a rotation vector, radians) in its parent's
        // frame taken before the solve, times the compression.
        maniVec3 preferred;
        // In [0, 1]: 0 turns freely, 1 locks the bone's rotation.
        float rotationStiffness;
        // In [0, 1], with stretch: 0 moves freely from its place in the
        // parent, 1 holds it there.
        float positionStiffness;
    } maniBodyBone;

    // A full-body solve.
    typedef struct maniBodySolve
    {
        // The top bone: the solved part is every bone on a path from it to
        // an effector, at most MANI_MAX_BODY_IK_BONES.
        uint32_t top;
        maniBodyTopMode topMode;
        // effectorCount effectors, 1 to MANI_MAX_BODY_IK_EFFECTORS, each
        // the top or below it, no bone twice.
        const maniBodyEffector* effectors;
        // boneCount settings (NULL for 0), each for a bone of the solved
        // part, no bone twice.
        const maniBodyBone* bones;
        uint32_t effectorCount;
        uint32_t boneCount;
        // From 1 to MANI_MAX_IK_ITERATIONS.
        uint32_t iterations;
        // Whether bones may leave their places in their parents (by
        // their position stiffness); false keeps every length.
        bool stretch;
        // The solve stops once every effector misses by no more than
        // this; 0 for every iteration. An effector's miss is its distance
        // from its goal and, wanting a rotation, its rotation's angle from
        // the goal's times its reach (the summed offsets from the top to
        // it, no less than the part's mean offset), whichever is larger.
        float tolerance;
        // In [0, 1]: the corrections' share, 1 for all of them.
        float weight;
    } maniBodySolve;

    /// Returns the bytes of scratch memory a full-body solve needs, by
    /// either solver (the larger of the two).
    ///
    /// @param skeleton  The skeleton, or NULL.
    /// @param solve     The solve, or NULL.
    /// @return The size; 0 for NULL or a solve maniPose_SolveBodyXpbd refuses.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniSkeleton_GetBodyIkScratchSize(const maniSkeleton* skeleton,
                                                      const maniBodySolve* solve);

    /// Bends the solved part of a pose toward its effectors' goals as
    /// rigid bodies (Müller et al.'s positional and angular corrections
    /// with XPBD compliance): each bone a body whose resistance to
    /// turning comes from its rotation stiffness, each joint holding a
    /// child's origin at its place in the parent (softly by position
    /// stiffness with stretch), each effector pulling its bone's origin
    /// and rotation by its strength, then limits; one Gauss-Seidel pass
    /// an iteration in a fixed order (effectors by index, joints from the
    /// tips up, stiffness, limits). Preferred angles, pull chains and
    /// pre-pull act first. The bones' rotations come from the bodies;
    /// their translations stay the input's unless stretching. Stops early
    /// once every effector lies within the tolerance. Strength and
    /// stiffness set the pose the passes converge to (XPBD's compliance
    /// means the same whatever the count only as the solve converges);
    /// short of convergence they have acted less, so the result changes
    /// with the iteration count.
    ///
    /// @param pose         A pose of the skeleton (not a difference); the
    ///                     solved part's locals are written.
    /// @param skeleton     Its skeleton.
    /// @param solve        The solve.
    /// @param scratch      Scratch memory, 16-byte aligned.
    /// @param scratchSize  Its size: at least maniSkeleton_GetBodyIkScratchSize.
    /// @param reachedOut   Receives whether every effector misses by no
    ///                     more than the tolerance at full weight; may be
    ///                     NULL.
    /// @return `mani_success`; `mani_errorLimit` for more than
    /// MANI_MAX_BODY_IK_EFFECTORS effectors or MANI_MAX_BODY_IK_BONES
    /// settings, more than MANI_MAX_IK_ITERATIONS iterations or a solved
    /// part past MANI_MAX_BODY_IK_BONES; `mani_errorCapacity` for less
    /// scratch than maniSkeleton_GetBodyIkScratchSize; `mani_errorInvalid` for a NULL
    /// pose, skeleton, solve or scratch, a pose of another skeleton or a
    /// difference, a top or bone out of range, an effector not the top
    /// or below it, a bone twice among the effectors or the settings, a
    /// setting's bone off the solved part, no effectors or no
    /// iterations, a goal, rotation or preferred angle not finite, a
    /// rotation of length 0, a limit of an unknown kind or out of range,
    /// a weight, strength, pull chain or stiffness outside [0, 1], a
    /// tolerance below 0 or not finite, a top mode unknown, misaligned
    /// scratch.
    /// @par Thread safety
    /// Safe from any thread; the pose and the scratch are used by one
    /// thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_SolveBodyXpbd(maniPose* pose,
                                                              const maniSkeleton* skeleton,
                                                              const maniBodySolve* solve,
                                                              void* scratch, size_t scratchSize,
                                                              bool* reachedOut);

    /// Bends the solved part of a pose toward its effectors' goals by
    /// selectively damped least squares (Buss and Kim): each iteration
    /// takes the Jacobian of every effector's position (and, with a
    /// rotation weight, its rotation, as a length: the angle times the
    /// effector's reach from the top) by three model axes per bone (and
    /// three translations for a free top), its rows scaled by strength
    /// and its columns by one minus rotation stiffness (a locked bone
    /// and a held top have none); decomposes it (one-sided Jacobi, at
    /// most 30 sweeps); and steps each singular direction by the most
    /// its effectors' movement allows, each bone's turn at most pi/4 an
    /// iteration; then limits. Each miss counts at most half the mean
    /// bone length. Preferred angles, pull chains and pre-pull act
    /// first. Stops early once every effector lies within the
    /// tolerance. Every length is kept: stretch is refused.
    ///
    /// The decomposition's work in an iteration grows with the square of
    /// the rows (six an effector, those of an effector without a rotation
    /// weight free) times the columns, so an iteration costs far more
    /// than one of maniPose_SolveBodyXpbd's: about 1.8 million instructions
    /// for a 22-bone part with five effectors, 54 million for 128 bones
    /// and 32 effectors without rotations (XPBD: 57 thousand and half a
    /// million). The iteration count is the caller's budget: suited to
    /// small parts, a few iterations, or work off the frame.
    ///
    /// @param pose         A pose of the skeleton (not a difference); the
    ///                     solved part's locals are written.
    /// @param skeleton     Its skeleton.
    /// @param solve        The solve; its stretch must be false.
    /// @param scratch      Scratch memory, 16-byte aligned.
    /// @param scratchSize  Its size: at least maniSkeleton_GetBodyIkScratchSize.
    /// @param reachedOut   Receives whether every effector misses by no
    ///                     more than the tolerance at full weight; may be
    ///                     NULL.
    /// @return As maniPose_SolveBodyXpbd, and `mani_errorInvalid` for stretch.
    /// @par Thread safety
    /// Safe from any thread; the pose and the scratch are used by one
    /// thread at a time.
    MANI_NODISCARD MANI_API maniResult maniPose_SolveBodySdls(maniPose* pose,
                                                              const maniSkeleton* skeleton,
                                                              const maniBodySolve* solve,
                                                              void* scratch, size_t scratchSize,
                                                              bool* reachedOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_BODYIK_H
