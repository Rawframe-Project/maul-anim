// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Aim and look-at (mani-0029), after ozz-animation's IKAimJob: the
// aiming bone's forward line through its offset point moved so the
// point aims at the goal (the forward replaced by the vector from the
// bone to where that line meets the sphere of the goal's distance), the
// shortest arc of it onto the goal, then a turn about the aim line
// taking the up axis's plane to the pole's (blended near the line as the
// two-bone solve's), then the twist. A chain turns root first, each bone
// its share of the aim still needed from it down, recomputed after the
// bones above it turned; the last turns what is left, so it ends aimed.
// A cone bends the goal onto its edge first.

#include "ik_frame.h"
#include "scalar.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// The direction the aiming bone's forward takes from its origin so that
// its line through the offset point passes through the goal t (from the
// origin); false when the offset lies further from the forward line than
// the goal from the origin.
static bool Aimed(maniVec3 forward, maniVec3 offset, maniVec3 t, maniVec3* aimedOut)
{
    float ao = maniVec3Dot(forward, offset);
    float ac2 = maniVec3Dot(offset, offset) - ao * ao;
    float r2 = maniVec3Dot(t, t);
    if (!(r2 > 0.0f) || ac2 > r2)
    {
        return false;
    }
    *aimedOut = maniVec3Add(offset, maniVec3Scale(forward, sqrtf(r2 - ac2) - ao));
    return true;
}

// The model-space turn of the aiming bone, about its origin, that aims
// it at the goal; the identity when the goal cannot be aimed at.
static maniQuat Turn(const maniAimSolve* s, const maniAffine* aiming, maniVec3 goal, bool* aimed)
{
    maniQuat q = maniIkRotation(aiming);
    maniVec3 t = maniVec3Sub(goal, maniIkOrigin(aiming));
    maniVec3 from;
    *aimed = Aimed(maniQuatRotate(q, s->forward), maniQuatRotate(q, s->offset), t, &from);
    if (!*aimed)
    {
        return maniQuatIdentity();
    }
    maniQuat turn = maniQuatBetween(from, t);
    maniVec3 u = maniVec3Scale(t, 1.0f / maniVec3Length(t));
    maniVec3 up = maniQuatRotate(turn, maniQuatRotate(q, s->up));
    turn = maniQuatMul(maniIkPlane(u, maniVec3Cross(u, up), s->pole), turn);
    return s->twist != 0.0f ? maniQuatMul(maniQuatFromAxisAngle(u, s->twist), turn) : turn;
}

// The goal bent onto the cone's edge about the aiming bone's input
// forward line, when further from it than the cone allows.
static maniVec3 Coned(const maniAimSolve* s, const maniAffine* aiming, bool* inside)
{
    *inside = true;
    maniQuat q = maniIkRotation(aiming);
    maniVec3 o = maniIkOrigin(aiming);
    maniVec3 t = maniVec3Sub(s->goal, o);
    maniVec3 from;
    if (s->cone == 0.0f ||
        !Aimed(maniQuatRotate(q, s->forward), maniQuatRotate(q, s->offset), t, &from))
    {
        return s->goal;
    }
    maniVec3 axis = maniVec3Cross(from, t);
    float sine = maniVec3Length(axis);
    float angle = maniAtan2(sine, maniVec3Dot(from, t));
    if (angle <= s->cone || !(sine > 0.0f))
    {
        return s->goal;
    }
    *inside = false;
    maniQuat edge = maniQuatFromAxisAngle(maniVec3Scale(axis, 1.0f / sine), s->cone);
    maniVec3 d = maniVec3Normalize(maniQuatRotate(edge, from), from);
    return maniVec3Add(o, maniVec3Scale(d, maniVec3Length(t)));
}

static bool Valid(const maniPose* pose, const maniSkeleton* skeleton, const maniAimSolve* s)
{
    if (!maniIkPoseUsable(pose, skeleton) || s->count == 0 || s->count > MANI_AIM_MAX_BONES)
    {
        return false;
    }
    float total = 0.0f;
    for (uint32_t k = 0; k < s->count; ++k)
    {
        if (s->bones[k] >= skeleton->boneCount || !(s->shares[k] >= 0.0f) ||
            !isfinite(s->shares[k]) ||
            (k > 0 && !maniIkAbove(skeleton, s->bones[k - 1], s->bones[k])))
        {
            return false;
        }
        total += s->shares[k];
    }
    float length = maniVec3Length(s->forward);
    return total > 0.0f && isfinite(total) && fabsf(length - 1.0f) <= 1e-3f &&
           maniIkFiniteVec(s->goal) && maniIkFiniteVec(s->pole) && maniIkFiniteVec(s->offset) &&
           maniIkFiniteVec(s->up) && isfinite(s->twist) && s->cone >= 0.0f &&
           s->cone <= MANI_AIM_MAX_CONE && s->weight >= 0.0f && s->weight <= 1.0f;
}

maniResult maniSolveAim(maniPose* pose, const maniSkeleton* skeleton, const maniAimSolve* solve,
                        bool* reachedOut)
{
    if (pose == NULL || skeleton == NULL || solve == NULL || !Valid(pose, skeleton, solve))
    {
        return mani_errorInvalid;
    }
    uint32_t last = solve->bones[solve->count - 1];
    maniAffine parent = maniIkParentModel(pose, skeleton, solve->bones[0]);
    maniAffine local = maniIkLocal(pose, solve->bones[0]);
    maniAffine frame = maniAffineMul(&parent, &local);
    maniAffine below = maniIkBelow(pose, skeleton, solve->bones[0], last);
    maniAffine aiming = maniAffineMul(&frame, &below);
    bool inside = true;
    maniVec3 goal = Coned(solve, &aiming, &inside);
    float remaining = 0.0f;
    for (uint32_t k = 0; k < solve->count; ++k)
    {
        remaining += solve->shares[k];
    }
    maniTransform before[MANI_AIM_MAX_BONES];
    maniQuat corrections[MANI_AIM_MAX_BONES];
    bool aimed = false;
    for (uint32_t k = 0; k < solve->count; ++k)
    {
        uint32_t bone = solve->bones[k];
        parent = maniIkParentModel(pose, skeleton, bone);
        local = maniIkLocal(pose, bone);
        frame = maniAffineMul(&parent, &local);
        below = maniIkBelow(pose, skeleton, bone, last);
        aiming = maniAffineMul(&frame, &below);
        maniQuat turn = Turn(solve, &aiming, goal, &aimed);
        float share = k + 1 == solve->count ? 1.0f
                      : remaining > 0.0f    ? solve->shares[k] / remaining
                                            : 0.0f;
        remaining -= solve->shares[k];
        turn = maniIkWeighted(turn, 1.0f);
        turn = share >= 1.0f ? turn : maniQuatSlerp(maniQuatIdentity(), turn, share);
        before[k] = maniPoseLocal(pose, bone);
        corrections[k] = maniIkInParent(turn, &parent);
        maniIkCorrect(pose, bone, corrections[k]);
    }
    if (solve->weight < 1.0f)
    {
        // Each correction weighted on the input pose.
        for (uint32_t k = 0; k < solve->count; ++k)
        {
            maniPoseSetLocal(pose, solve->bones[k], before[k]);
            maniIkCorrect(pose, solve->bones[k], maniIkWeighted(corrections[k], solve->weight));
        }
    }
    if (reachedOut != NULL)
    {
        *reachedOut = aimed && inside && solve->weight >= 1.0f;
    }
    return mani_success;
}
