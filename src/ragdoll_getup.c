// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Getting up from a ragdoll: face up or down from the root
// body's facing along the ground's normal; the heading from its up axis
// on the ground's plane and its facing, weighed by how upright the body
// stands; the placement on the plane under the root body, the
// character's model axes turned onto the normal and the heading.

#include "fpenv.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/ragdoll.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static bool Finite3(maniVec3 v)
{
    return fabsf(v.x) < HUGE_VALF && fabsf(v.y) < HUGE_VALF && fabsf(v.z) < HUGE_VALF;
}

// Of length 1 within a thousandth (its square within 0.002 of 1).
static bool Unit(maniVec3 v)
{
    float l = maniVec3Dot(v, v);
    return l >= 0.998f && l <= 1.002f;
}

static bool GroundValid(const maniRagdollGround* g)
{
    return Finite3(g->point) && Finite3(g->normal) && Finite3(g->forward) && Finite3(g->up) &&
           Unit(g->normal) && Unit(g->forward) && Unit(g->up) &&
           fabsf(maniVec3Dot(g->forward, g->up)) <= 1e-3f;
}

static bool BodyValid(const maniRagdollBodyState* body)
{
    maniQuat r = body->rotation;
    float l = r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w;
    return Finite3(body->position) && fabsf(r.w) < HUGE_VALF &&
           Finite3((maniVec3){r.x, r.y, r.z}) && l >= 0.998f && l <= 1.002f;
}

// v on the plane of unit normal n.
static maniVec3 OnPlane(maniVec3 v, maniVec3 n)
{
    return maniVec3Sub(v, maniVec3Scale(n, maniVec3Dot(v, n)));
}

// The turn about unit n taking a onto b, both on n's plane and of
// length 1: half way between them, or half a turn about n when they are
// opposite.
static maniQuat TurnAbout(maniVec3 n, maniVec3 a, maniVec3 b)
{
    float w = 1.0f + maniVec3Dot(a, b);
    if (w < 1e-6f)
    {
        return (maniQuat){n.x, n.y, n.z, 0.0f};
    }
    maniVec3 c = maniVec3Cross(a, b);
    return maniQuatNormalize((maniQuat){c.x, c.y, c.z, w});
}

static MANI_NOINLINE maniResult RagdollGetUpPlacementWork(const maniRagdoll* ragdoll,
                                                          const maniRagdollBodyState* bodies,
                                                          uint32_t count,
                                                          const maniRagdollGround* ground,
                                                          maniRagdollGetUp* out)
{
    if (ragdoll == NULL || bodies == NULL || ground == NULL || out == NULL)
    {
        return mani_errorInvalid;
    }
    if (count < ragdoll->bodyCount)
    {
        return mani_errorCapacity;
    }
    for (uint32_t i = 0; i < ragdoll->bodyCount; ++i)
    {
        if (!BodyValid(&bodies[i]))
        {
            return mani_errorInvalid;
        }
    }
    if (!GroundValid(ground))
    {
        return mani_errorInvalid;
    }
    const maniRagdollBodyState* root = &bodies[0];
    maniVec3 n = maniVec3Normalize(ground->normal, ground->normal);
    maniQuat r = maniQuatNormalize(root->rotation);
    maniVec3 chest = maniQuatRotate(r, ragdoll->facing);
    bool faceUp = maniVec3Dot(chest, n) >= 0.0f;
    maniVec3 up = maniQuatRotate(r, ragdoll->up);
    maniVec3 head = OnPlane(up, n);
    head = faceUp ? maniVec3Scale(head, -1.0f) : head;
    // The chest's say as much as the body stands: none lying, all upright.
    chest = maniVec3Scale(OnPlane(chest, n), fabsf(maniVec3Dot(up, n)));
    maniVec3 heading = maniVec3Add(head, chest);
    if (maniVec3Dot(heading, heading) < 1e-6f)
    {
        // The two cancel, equal and opposite: the head's way, as ALS's.
        heading = maniVec3Dot(head, head) >= 1e-6f ? head : chest;
    }
    heading = maniVec3Normalize(heading, heading);
    // The model up onto the normal, then about the normal the model
    // forward (now on the plane) onto the heading.
    maniQuat lift = maniQuatBetween(maniVec3Normalize(ground->up, ground->up), n);
    maniVec3 forward = OnPlane(maniQuatRotate(lift, ground->forward), n);
    forward = maniVec3Normalize(forward, forward);
    maniQuat turn = maniQuatMul(TurnAbout(n, forward, heading), lift);
    maniVec3 p = root->position;
    maniVec3 at = maniVec3Sub(p, maniVec3Scale(n, maniVec3Dot(maniVec3Sub(p, ground->point), n)));
    out->placement = (maniTransform){at, maniQuatNormalize(turn), {1.0f, 1.0f, 1.0f}};
    out->faceUp = faceUp;
    return mani_success;
}

maniResult maniRagdollGetUpPlacement(const maniRagdoll* ragdoll, const maniRagdollBodyState* bodies,
                                     uint32_t count, const maniRagdollGround* ground,
                                     maniRagdollGetUp* out)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = RagdollGetUpPlacementWork(ragdoll, bodies, count, ground, out);
    maniFloatEnvLeave(env);
    return result;
}
