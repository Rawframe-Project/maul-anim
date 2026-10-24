// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body solve as rigid bodies, Müller et al.'s
// positional and angular corrections with XPBD compliance: each part
// bone a body (a unit mass and an inverse inertia from its length; a
// held top neither moves nor turns); each effector pulls its body's
// origin and rotation with compliance (1 - strength) / strength; each
// joint, from the tips up, holds a child's origin at its place on the
// parent (with stretch, compliance 1 - the child's position stiffness);
// each stiff bone's rotation is held to its parent's with compliance
// (1 - stiffness) / stiffness; then limits; then each locked bone takes
// its parent's turn exactly, so it stays rigid with its parent through
// every pass (its hard stiffness alone splits a correction with the
// bone above, which then turns within the pass). One Gauss-Seidel pass
// an iteration in that order, every multiplier from 0 each call. Turns
// are taken whole by rotation vectors (Müller's linearized update is
// their first order). Without stretch the joints are rigid, so after
// each pass every origin is placed from the turns again: what a
// correction moved without turning is dropped, and lengths are exact.

#include "body_ik.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// The solver's floats in the part's room: per node an inverse mass, an
// inverse inertia, the joint's and the stiffness' multipliers and a
// placed origin; per effector the position's and the rotation's
// multipliers.
typedef struct Bodies
{
    float* inverseMass;
    float* inverseInertia;
    float* joint;
    float* stiff;
    float* pull;
    maniVec3* placed;
} Bodies;

size_t maniBodyXpbdRoom(uint32_t count, uint32_t effectors)
{
    return 7u * (size_t)count + 2u * (size_t)effectors;
}

static Bodies Lay(const maniBodyPart* part)
{
    float* f = part->room;
    uint32_t n = part->count;
    void* placed = f + 4u * n + 2u * part->effectors;
    Bodies b = {f, f + n, f + 2u * n, f + 3u * n, f + 4u * n, placed};
    for (uint32_t k = 0; k < 2u * n; ++k)
    {
        b.joint[k] = 0.0f;
    }
    for (uint32_t e = 0; e < 2u * part->effectors; ++e)
    {
        b.pull[e] = 0.0f;
    }
    return b;
}

// Unit masses; inverse inertias of 2 over the squared length to the
// farthest child (the bone's own offset for a leaf), that square no less
// than a hundredth of the mean offset's square; a held top fixed. Of 0.5,
// 1, 2, 3 (a rod's about its end) and 4 over the square, 2 converged in
// the fewest passes on the tests' figure.
static void Masses(const maniBodyPart* part, const maniBodySolve* solve, const Bodies* b)
{
    float sum = 0.0f;
    uint32_t lengths = 0;
    for (uint32_t k = 0; k < part->count; ++k)
    {
        float l = maniVec3Length(part->nodes[k].offset);
        sum += l;
        lengths += l > 0.0f;
        b->inverseInertia[k] = 0.0f;
    }
    float mean = lengths > 0 ? sum / (float)lengths : 1.0f;
    for (uint32_t k = part->count; k-- > 1;)
    {
        float square = maniVec3Dot(part->nodes[k].offset, part->nodes[k].offset);
        float* parent = &b->inverseInertia[part->nodes[k].parent];
        *parent = square > *parent ? square : *parent;
        b->inverseInertia[k] = part->nodes[k].children == 0 && square > b->inverseInertia[k]
                                   ? square
                                   : b->inverseInertia[k];
    }
    for (uint32_t k = 0; k < part->count; ++k)
    {
        float square = b->inverseInertia[k];
        square = square > 0.01f * mean * mean ? square : 0.01f * mean * mean;
        bool held = k == 0 && solve->topMode != maniBodyTopFree;
        b->inverseMass[k] = held ? 0.0f : 1.0f;
        b->inverseInertia[k] = held ? 0.0f : 2.0f / square;
    }
}

static float Compliance(float strength)
{
    return (1.0f - strength) / strength;
}

// Node k's turn turned by rotation vector v.
static void Spin(maniBodyNode* node, maniVec3 v)
{
    node->turn = maniQuatNormalize(maniQuatMul(maniQuatFromRotationVector(v), node->turn));
}

// The effector's body's origin pulled toward its goal, then its
// rotation turned toward its goal rotation.
static void Effector(maniBodyPart* part, const maniBodySolve* solve, const Bodies* b, uint32_t e)
{
    const maniBodyEffector* effector = &solve->effectors[e];
    const maniBodyGoal* goal = &part->goals[e];
    maniBodyNode* node = &part->nodes[goal->node];
    float alpha = Compliance(effector->strength);
    float m = b->inverseMass[goal->node];
    maniVec3 d = maniVec3Sub(node->origin, goal->position);
    float c = maniVec3Length(d);
    if (c > 0.0f && m > 0.0f)
    {
        float dl = (-c - alpha * b->pull[2u * e]) / (m + alpha);
        b->pull[2u * e] += dl;
        node->origin = maniVec3Add(node->origin, maniVec3Scale(d, m * dl / c));
    }
    float inertia = b->inverseInertia[goal->node];
    if (!(effector->rotationWeight > 0.0f) || !(inertia > 0.0f))
    {
        return;
    }
    maniQuat miss = maniQuatMul(goal->rotation, maniQuatConjugate(maniBodyRotation(node)));
    miss = miss.w < 0.0f ? (maniQuat){-miss.x, -miss.y, -miss.z, -miss.w} : miss;
    maniVec3 v = maniQuatToRotationVector(miss);
    c = maniVec3Length(v);
    if (c > 0.0f)
    {
        float dl = (-c - alpha * b->pull[2u * e + 1u]) / (inertia + alpha);
        b->pull[2u * e + 1u] += dl;
        Spin(node, maniVec3Scale(v, -dl * inertia / c));
    }
}

// Each child's origin pulled to its place on its parent: the child
// moves, the parent moves and turns about its origin.
static void Joints(maniBodyPart* part, const maniBodySolve* solve, const Bodies* b)
{
    for (uint32_t k = part->count; k-- > 1;)
    {
        maniBodyNode* child = &part->nodes[k];
        uint32_t p = child->parent;
        maniBodyNode* parent = &part->nodes[p];
        const maniBodyBone* s = maniBodySetting(solve, child);
        float alpha = solve->stretch ? 1.0f - (s != NULL ? s->positionStiffness : 0.0f) : 0.0f;
        maniVec3 r = maniQuatRotate(parent->turn, child->offset);
        maniVec3 d = maniVec3Sub(child->origin, maniVec3Add(parent->origin, r));
        float c = maniVec3Length(d);
        if (!(c > 0.0f))
        {
            continue;
        }
        maniVec3 n = maniVec3Scale(d, 1.0f / c);
        maniVec3 rn = maniVec3Cross(r, n);
        float w =
            b->inverseMass[k] + b->inverseMass[p] + b->inverseInertia[p] * maniVec3Dot(rn, rn);
        if (!(w + alpha > 0.0f))
        {
            continue;
        }
        float dl = (-c - alpha * b->joint[k]) / (w + alpha);
        b->joint[k] += dl;
        maniVec3 impulse = maniVec3Scale(n, dl);
        child->origin = maniVec3Add(child->origin, maniVec3Scale(impulse, b->inverseMass[k]));
        parent->origin = maniVec3Sub(parent->origin, maniVec3Scale(impulse, b->inverseMass[p]));
        Spin(parent, maniVec3Scale(maniVec3Cross(r, impulse), -b->inverseInertia[p]));
    }
}

// Each stiff bone's turn pulled toward its parent's, both turning.
static void Stiffness(maniBodyPart* part, const maniBodySolve* solve, const Bodies* b)
{
    for (uint32_t k = 1; k < part->count; ++k)
    {
        maniBodyNode* child = &part->nodes[k];
        const maniBodyBone* s = maniBodySetting(solve, child);
        uint32_t p = child->parent;
        float w = b->inverseInertia[k] + b->inverseInertia[p];
        if (s == NULL || !(s->rotationStiffness > 0.0f) || !(w > 0.0f))
        {
            continue;
        }
        maniBodyNode* parent = &part->nodes[p];
        maniQuat miss = maniQuatMul(parent->turn, maniQuatConjugate(child->turn));
        miss = miss.w < 0.0f ? (maniQuat){-miss.x, -miss.y, -miss.z, -miss.w} : miss;
        maniVec3 v = maniQuatToRotationVector(miss);
        float c = maniVec3Length(v);
        if (!(c > 0.0f))
        {
            continue;
        }
        float alpha = Compliance(s->rotationStiffness);
        float dl = (-c - alpha * b->stiff[k]) / (w + alpha);
        b->stiff[k] += dl;
        Spin(child, maniVec3Scale(v, -dl * b->inverseInertia[k] / c));
        Spin(parent, maniVec3Scale(v, dl * b->inverseInertia[p] / c));
    }
}

// The largest miss of an effector from where the turns place its bone
// (its body's origin with stretch).
static float Miss(maniBodyPart* part, const maniBodySolve* solve, const Bodies* b)
{
    if (solve->stretch)
    {
        return maniBodyMiss(part);
    }
    b->placed[0] = part->nodes[0].origin;
    for (uint32_t k = 1; k < part->count; ++k)
    {
        const maniBodyNode* node = &part->nodes[k];
        b->placed[k] = maniVec3Add(b->placed[node->parent],
                                   maniQuatRotate(part->nodes[node->parent].turn, node->offset));
    }
    float worst = 0.0f;
    for (uint32_t e = 0; e < part->effectors; ++e)
    {
        float miss = maniBodyGoalMiss(part, e, b->placed[part->goals[e].node]);
        worst = miss > worst ? miss : worst;
    }
    return worst;
}

// Locked bones take their parents' turns exactly, from the top down.
static void Lock(maniBodyPart* part, const maniBodySolve* solve)
{
    for (uint32_t k = 1; k < part->count; ++k)
    {
        const maniBodyBone* s = maniBodySetting(solve, &part->nodes[k]);
        if (s != NULL && s->rotationStiffness >= 1.0f)
        {
            part->nodes[k].turn = part->nodes[part->nodes[k].parent].turn;
        }
    }
}

maniResult maniSolveBodyXpbd(maniPose* pose, const maniSkeleton* skeleton,
                             const maniBodySolve* solve, void* scratch, size_t scratchSize,
                             bool* reachedOut)
{
    uint32_t count = maniBodyPartCount(pose, skeleton, solve, scratch, scratchSize);
    if (count == 0)
    {
        return mani_errorInvalid;
    }
    maniBodyPart part;
    maniBodyBegin(&part, pose, skeleton, solve, scratch, count);
    Bodies b = Lay(&part);
    Masses(&part, solve, &b);
    maniBodyPrepare(&part, solve);
    for (uint32_t i = 0; i < solve->iterations && Miss(&part, solve, &b) > solve->tolerance; ++i)
    {
        for (uint32_t e = 0; e < part.effectors; ++e)
        {
            if (solve->effectors[e].strength > 0.0f)
            {
                Effector(&part, solve, &b, e);
            }
        }
        Joints(&part, solve, &b);
        Stiffness(&part, solve, &b);
        for (uint32_t k = 0; k < count; ++k)
        {
            maniBodyLimit(&part, skeleton, solve, k);
        }
        Lock(&part, solve);
        if (!solve->stretch)
        {
            maniBodyPlace(&part);
        }
    }
    bool reached = maniBodyFinish(&part, pose, solve);
    if (reachedOut != NULL)
    {
        *reachedOut = reached;
    }
    return mani_success;
}
