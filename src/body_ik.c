// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The full-body description and solved part: the part is
// found by walking each effector up to the top, its bones kept sorted
// by index (parents first, each subtree contiguous) in at most
// MANI_MAX_BODY_IK_BONES slots; the whole description is checked before
// anything is written.

#include "body_ik.h"

#include "ik_frame.h"
#include "ik_limit.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/bodyik.h"
#include "maul-anim/ik.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static bool Unit(float v)
{
    return v >= 0.0f && v <= 1.0f;
}

static bool FiniteVec(maniVec3 v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// The slot of bone in the sorted list, or count when absent.
static uint32_t Find(const uint32_t* list, uint32_t count, uint32_t bone)
{
    uint32_t low = 0;
    uint32_t high = count;
    while (low < high)
    {
        uint32_t mid = low + (high - low) / 2;
        if (list[mid] < bone)
        {
            low = mid + 1;
        }
        else
        {
            high = mid;
        }
    }
    return low < count && list[low] == bone ? low : count;
}

// Adds bone to the sorted list if absent; false when the list is full.
static bool Add(uint32_t* list, uint32_t* count, uint32_t bone)
{
    if (Find(list, *count, bone) < *count)
    {
        return true;
    }
    if (*count == MANI_MAX_BODY_IK_BONES)
    {
        return false;
    }
    uint32_t k = *count;
    for (; k > 0 && list[k - 1] > bone; --k)
    {
        list[k] = list[k - 1];
    }
    list[k] = bone;
    ++*count;
    return true;
}

// The part of a solve whose top is in range: every bone from each
// effector up to the top; 0 when an effector is not the top or below it
// or the part passes the bound (then past is set).
static uint32_t Collect(const maniSkeleton* skeleton, const maniBodySolve* solve, uint32_t* list,
                        bool* past)
{
    uint32_t count = 0;
    uint32_t n = maniSkeleton_GetBoneCount(skeleton);
    for (uint32_t e = 0; e < solve->effectorCount; ++e)
    {
        uint32_t b = solve->effectors[e].bone;
        if (b >= n || (b != solve->top && !maniIkAbove(skeleton, solve->top, b)))
        {
            return 0;
        }
        for (; b != solve->top; b = maniSkeleton_GetParent(skeleton, b))
        {
            if (!Add(list, &count, b))
            {
                *past = true;
                return 0;
            }
        }
        if (!Add(list, &count, b))
        {
            *past = true;
            return 0;
        }
    }
    return count;
}

// The rest direction of a part bone's one child on the part, in its
// frame; zero for none or several.
static maniVec3 Axis(const maniSkeleton* skeleton, const uint32_t* list, uint32_t count,
                     uint32_t slot)
{
    maniVec3 zero = maniVec3Make(0.0f, 0.0f, 0.0f);
    uint32_t child = MANI_BODY_NONE;
    for (uint32_t k = slot + 1; k < count; ++k)
    {
        if (maniSkeleton_GetParent(skeleton, list[k]) == list[slot])
        {
            if (child != MANI_BODY_NONE)
            {
                return zero;
            }
            child = list[k];
        }
    }
    return child == MANI_BODY_NONE
               ? zero
               : maniVec3Normalize(maniSkeleton_GetRest(skeleton, child).translation, zero);
}

static bool EffectorValid(const maniBodyEffector* e)
{
    maniQuat q = e->rotation;
    float length = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return FiniteVec(e->position) && length > 0.0f && isfinite(length) && Unit(e->positionWeight) &&
           Unit(e->rotationWeight) && Unit(e->strength) && Unit(e->pullChain);
}

static bool SettingValid(const maniSkeleton* skeleton, const uint32_t* list, uint32_t count,
                         const maniBodyBone* s)
{
    uint32_t slot = Find(list, count, s->bone);
    if (slot == count || !maniIkLimitValid(&s->limit) || !FiniteVec(s->preferred) ||
        !Unit(s->rotationStiffness) || !Unit(s->positionStiffness))
    {
        return false;
    }
    // A hinge along the bone's own axis is no hinge.
    maniVec3 axis = Axis(skeleton, list, count, slot);
    return s->limit.kind != maniJointLimitHinge || maniVec3Dot(axis, axis) == 0.0f ||
           maniVec3Length(maniVec3Cross(s->limit.axis, axis)) >= 1e-3f;
}

// Whether the counts within their bounds are in range, and the numbers
// and modes.
static bool HeaderValid(const maniBodySolve* solve)
{
    return solve->effectors != NULL && solve->effectorCount >= 1 &&
           (solve->bones != NULL || solve->boneCount == 0) && solve->iterations >= 1 &&
           solve->tolerance >= 0.0f && isfinite(solve->tolerance) && Unit(solve->weight) &&
           solve->topMode <= maniBodyTopPrePull;
}

uint32_t maniBodyPartShape(const maniSkeleton* skeleton, const maniBodySolve* solve, bool* past)
{
    *past = false;
    if (skeleton == NULL || solve == NULL || solve->top >= maniSkeleton_GetBoneCount(skeleton))
    {
        return 0;
    }
    *past = solve->effectorCount > MANI_MAX_BODY_IK_EFFECTORS ||
            solve->boneCount > MANI_MAX_BODY_IK_BONES || solve->iterations > MANI_MAX_IK_ITERATIONS;
    if (*past || !HeaderValid(solve))
    {
        return 0;
    }
    uint32_t list[MANI_MAX_BODY_IK_BONES] = {0};
    uint32_t count = Collect(skeleton, solve, list, past);
    for (uint32_t e = 0; count > 0 && e < solve->effectorCount; ++e)
    {
        for (uint32_t f = 0; f < e; ++f)
        {
            count = solve->effectors[f].bone == solve->effectors[e].bone ? 0 : count;
        }
        count = EffectorValid(&solve->effectors[e]) ? count : 0;
    }
    for (uint32_t s = 0; count > 0 && s < solve->boneCount; ++s)
    {
        for (uint32_t t = 0; t < s; ++t)
        {
            count = solve->bones[t].bone == solve->bones[s].bone ? 0 : count;
        }
        count = SettingValid(skeleton, list, count, &solve->bones[s]) ? count : 0;
    }
    return count;
}

static size_t Rounded(size_t bytes)
{
    return (bytes + 15u) & ~(size_t)15u;
}

// The scratch bytes for a part of count nodes: the nodes, the goals and
// the largest room a solver takes.
static size_t ScratchBytes(uint32_t count, uint32_t effectors)
{
    size_t xpbd = maniBodyXpbdRoom(count, effectors);
    size_t sdls = maniBodySdlsRoom(count, effectors);
    return Rounded(count * sizeof(maniBodyNode)) + Rounded(effectors * sizeof(maniBodyGoal)) +
           Rounded((xpbd > sdls ? xpbd : sdls) * sizeof(float));
}

size_t maniSkeleton_GetBodyIkScratchSize(const maniSkeleton* skeleton, const maniBodySolve* solve)
{
    bool past = false;
    uint32_t count = maniBodyPartShape(skeleton, solve, &past);
    return count == 0 ? 0 : ScratchBytes(count, solve->effectorCount);
}

maniResult maniBodyPartCheck(const maniPose* pose, const maniSkeleton* skeleton,
                             const maniBodySolve* solve, const void* scratch, size_t scratchSize,
                             uint32_t* countOut)
{
    *countOut = 0;
    if (pose == NULL || skeleton == NULL || solve == NULL || scratch == NULL ||
        ((uintptr_t)scratch & 15u) != 0 || !maniIkPoseUsable(pose, skeleton))
    {
        return mani_errorInvalid;
    }
    bool past = false;
    uint32_t count = maniBodyPartShape(skeleton, solve, &past);
    if (count == 0)
    {
        return past ? mani_errorLimit : mani_errorInvalid;
    }
    if (scratchSize < ScratchBytes(count, solve->effectorCount))
    {
        return mani_errorCapacity;
    }
    *countOut = count;
    return mani_success;
}

const maniBodyBone* maniBodySetting(const maniBodySolve* solve, const maniBodyNode* node)
{
    return node->setting != MANI_BODY_NONE ? &solve->bones[node->setting] : NULL;
}

maniQuat maniBodyRotation(const maniBodyNode* node)
{
    return maniQuatNormalize(maniQuatMul(node->turn, maniIkRotation(&node->frame)));
}

void maniBodyPlace(maniBodyPart* part)
{
    for (uint32_t k = 1; k < part->count; ++k)
    {
        maniBodyNode* node = &part->nodes[k];
        const maniBodyNode* parent = &part->nodes[node->parent];
        node->origin = maniVec3Add(parent->origin, maniQuatRotate(parent->turn, node->offset));
    }
}

float maniBodyGoalMiss(const maniBodyPart* part, uint32_t e, maniVec3 origin)
{
    const maniBodyGoal* goal = &part->goals[e];
    float miss = maniVec3Length(maniVec3Sub(origin, goal->position));
    if (goal->turns == 0)
    {
        return miss;
    }
    maniQuat q =
        maniQuatMul(goal->rotation, maniQuatConjugate(maniBodyRotation(&part->nodes[goal->node])));
    q = q.w < 0.0f ? (maniQuat){-q.x, -q.y, -q.z, -q.w} : q;
    float turned = maniVec3Length(maniQuatToRotationVector(q)) * goal->reach;
    return turned > miss ? turned : miss;
}

float maniBodyMiss(const maniBodyPart* part)
{
    float worst = 0.0f;
    for (uint32_t e = 0; e < part->effectors; ++e)
    {
        float miss = maniBodyGoalMiss(part, e, part->nodes[part->goals[e].node].origin);
        worst = miss > worst ? miss : worst;
    }
    return worst;
}

// Reads node k of the part's input; its parent's node read before it.
static void ReadNode(maniBodyPart* part, const maniPose* pose, const maniSkeleton* skeleton,
                     const maniBodySolve* solve, const uint32_t* list, uint32_t k)
{
    maniBodyNode* node = &part->nodes[k];
    node->bone = list[k];
    node->parent = k == 0 ? MANI_BODY_NONE
                          : Find(list, part->count, maniSkeleton_GetParent(skeleton, list[k]));
    node->parentFrame =
        k == 0 ? maniIkParentModel(pose, skeleton, list[k]) : part->nodes[node->parent].frame;
    maniIkFrame local = maniIkLocal(pose, list[k]);
    node->frame = maniIkMul(&node->parentFrame, &local);
    node->origin = maniIkOrigin(&node->frame);
    node->turn = maniQuatIdentity();
    node->local = maniQuatNormalize(maniPose_GetLocal(pose, list[k]).rotation);
    node->offset = k == 0 ? maniVec3Make(0.0f, 0.0f, 0.0f)
                          : maniVec3Sub(node->origin, part->nodes[node->parent].origin);
    node->axis = Axis(skeleton, list, part->count, k);
    node->setting = MANI_BODY_NONE;
    for (uint32_t s = 0; s < solve->boneCount; ++s)
    {
        node->setting = solve->bones[s].bone == list[k] ? s : node->setting;
    }
    node->below = 0;
    node->children = 0;
    node->reserved = 0;
}

void maniBodyBegin(maniBodyPart* part, const maniPose* pose, const maniSkeleton* skeleton,
                   const maniBodySolve* solve, void* scratch, uint32_t count)
{
    uint8_t* bytes = scratch;
    size_t nodes = Rounded(count * sizeof(maniBodyNode));
    size_t goals = Rounded(solve->effectorCount * sizeof(maniBodyGoal));
    // The scratch is 16-byte aligned and each block a multiple of 16.
    void* at[3] = {bytes, bytes + nodes, bytes + nodes + goals};
    part->nodes = at[0];
    part->goals = at[1];
    part->room = at[2];
    part->count = count;
    part->effectors = solve->effectorCount;
    uint32_t list[MANI_MAX_BODY_IK_BONES] = {0};
    bool past = false;
    (void)Collect(skeleton, solve, list, &past);
    for (uint32_t k = 0; k < count; ++k)
    {
        ReadNode(part, pose, skeleton, solve, list, k);
    }
    float sum = 0.0f;
    uint32_t lengths = 0;
    for (uint32_t k = count; k-- > 1;)
    {
        maniBodyNode* parent = &part->nodes[part->nodes[k].parent];
        parent->below += 1 + part->nodes[k].below;
        ++parent->children;
        float l = maniVec3Length(part->nodes[k].offset);
        sum += l;
        lengths += l > 0.0f;
    }
    part->mean = lengths > 0 ? sum / (float)lengths : 1.0f;
    part->reserved = 0.0f;
    for (uint32_t e = 0; e < solve->effectorCount; ++e)
    {
        const maniBodyEffector* effector = &solve->effectors[e];
        maniBodyGoal* goal = &part->goals[e];
        goal->node = Find(list, count, effector->bone);
        const maniBodyNode* node = &part->nodes[goal->node];
        maniVec3 from = node->origin;
        maniVec3 to = effector->position;
        float w = effector->positionWeight;
        goal->position = maniVec3Add(from, maniVec3Scale(maniVec3Sub(to, from), w));
        goal->rotation =
            maniQuatSlerp(maniIkRotation(&node->frame), maniQuatNormalize(effector->rotation),
                          effector->rotationWeight);
        goal->turns = effector->rotationWeight > 0.0f;
        goal->reach = 0.0f;
        for (uint32_t k = goal->node; k != 0; k = part->nodes[k].parent)
        {
            goal->reach += maniVec3Length(part->nodes[k].offset);
        }
        goal->reach = goal->reach > part->mean ? goal->reach : part->mean;
        goal->reserved = 0.0f;
    }
}

void maniBodyTurn(maniBodyPart* part, uint32_t k, maniQuat turn)
{
    uint32_t end = k + 1 + part->nodes[k].below;
    for (uint32_t j = k; j < end; ++j)
    {
        part->nodes[j].turn = maniQuatNormalize(maniQuatMul(turn, part->nodes[j].turn));
    }
    maniBodyPlace(part);
}

maniQuat maniBodyLocal(const maniBodyPart* part, uint32_t k)
{
    const maniBodyNode* node = &part->nodes[k];
    maniQuat parentTurn = k == 0 ? maniQuatIdentity() : part->nodes[node->parent].turn;
    maniQuat relative = maniQuatMul(maniQuatConjugate(parentTurn), node->turn);
    return maniQuatNormalize(
        maniQuatMul(maniIkInParent(relative, &node->parentFrame), node->local));
}

// How far node k's effectors' goals have come closer to its origin than
// their bones' input origins, the most among them, in [0, 1].
static float Compression(const maniBodyPart* part, uint32_t k)
{
    const maniBodyNode* node = &part->nodes[k];
    float most = 0.0f;
    for (uint32_t e = 0; e < part->effectors; ++e)
    {
        const maniBodyGoal* goal = &part->goals[e];
        if (goal->node <= k || goal->node > k + node->below)
        {
            continue;
        }
        float input = maniVec3Length(maniVec3Sub(part->nodes[goal->node].origin, node->origin));
        float wanted = maniVec3Length(maniVec3Sub(goal->position, node->origin));
        float c = input > 0.0f ? 1.0f - wanted / input : 0.0f;
        c = c < 0.0f ? 0.0f : (c > 1.0f ? 1.0f : c);
        most = c > most ? c : most;
    }
    return most;
}

// Each node with preferred angles turned by them times its compression,
// read from the input before any turns, from the top down.
static void Preferred(maniBodyPart* part, const maniBodySolve* solve)
{
    float compression[MANI_MAX_BODY_IK_BONES];
    for (uint32_t k = 0; k < part->count; ++k)
    {
        compression[k] = Compression(part, k);
    }
    for (uint32_t k = 1; k < part->count; ++k)
    {
        const maniBodyBone* s = maniBodySetting(solve, &part->nodes[k]);
        if (s == NULL || compression[k] == 0.0f || s->rotationStiffness >= 1.0f)
        {
            continue;
        }
        maniQuat local = maniQuatFromRotationVector(maniVec3Scale(s->preferred, compression[k]));
        maniQuat parentTurn = part->nodes[part->nodes[k].parent].turn;
        maniQuat model = maniIkFromParent(local, &part->nodes[k].parentFrame);
        maniBodyTurn(part, k,
                     maniQuatMul(maniQuatMul(parentTurn, model), maniQuatConjugate(parentTurn)));
    }
}

// Whether node k turns: not locked, not a held top.
static bool Turns(const maniBodyPart* part, const maniBodySolve* solve, uint32_t k)
{
    const maniBodyBone* s = maniBodySetting(solve, &part->nodes[k]);
    return (k != 0 || solve->topMode == maniBodyTopFree) &&
           (s == NULL || s->rotationStiffness < 1.0f);
}

// Effector e's chain, from its node up to the nearest branch or the
// top, turned about its first node by the shortest arc from the
// effector toward its goal, times the pull-chain weight.
static void Pull(maniBodyPart* part, const maniBodySolve* solve, uint32_t e)
{
    const maniBodyGoal* goal = &part->goals[e];
    uint32_t k = goal->node;
    while (k != 0 && part->nodes[part->nodes[k].parent].children == 1 &&
           (part->nodes[k].parent != 0 || solve->topMode == maniBodyTopFree))
    {
        k = part->nodes[k].parent;
    }
    maniVec3 pivot = part->nodes[k].origin;
    maniVec3 now = maniVec3Sub(part->nodes[goal->node].origin, pivot);
    maniVec3 wanted = maniVec3Sub(goal->position, pivot);
    if (!Turns(part, solve, k) || !(maniVec3Dot(now, now) > 0.0f) ||
        !(maniVec3Dot(wanted, wanted) > 0.0f))
    {
        return;
    }
    maniQuat arc = maniIkWeighted(maniQuatBetween(now, wanted), solve->effectors[e].pullChain);
    maniBodyTurn(part, k, arc);
}

// The top moved so each stretched effector's goal (past the summed
// offsets from the top) would lie at its reach: the strength-weighted
// mean of the excesses along the top-to-goal lines.
static void PrePull(maniBodyPart* part, const maniBodySolve* solve)
{
    maniVec3 sum = maniVec3Make(0.0f, 0.0f, 0.0f);
    float strengths = 0.0f;
    maniBodyNode* top = &part->nodes[0];
    for (uint32_t e = 0; e < part->effectors; ++e)
    {
        float reach = 0.0f;
        for (uint32_t k = part->goals[e].node; k != 0; k = part->nodes[k].parent)
        {
            reach += maniVec3Length(part->nodes[k].offset);
        }
        maniVec3 line = maniVec3Sub(part->goals[e].position, top->origin);
        float distance = maniVec3Length(line);
        float strength = solve->effectors[e].strength;
        if (distance > reach && strength > 0.0f)
        {
            sum = maniVec3Add(sum, maniVec3Scale(line, strength * (distance - reach) / distance));
            strengths += strength;
        }
    }
    if (strengths > 0.0f)
    {
        top->origin = maniVec3Add(top->origin, maniVec3Scale(sum, 1.0f / strengths));
        maniBodyPlace(part);
    }
}

void maniBodyPrepare(maniBodyPart* part, const maniBodySolve* solve)
{
    Preferred(part, solve);
    for (uint32_t e = 0; e < part->effectors; ++e)
    {
        if (solve->effectors[e].pullChain > 0.0f)
        {
            Pull(part, solve, e);
        }
    }
    if (solve->topMode == maniBodyTopPrePull)
    {
        PrePull(part, solve);
    }
}

void maniBodyLimit(maniBodyPart* part, const maniSkeleton* skeleton, const maniBodySolve* solve,
                   uint32_t k)
{
    maniBodyNode* node = &part->nodes[k];
    const maniBodyBone* s = maniBodySetting(solve, node);
    if (s == NULL || s->limit.kind == maniJointLimitNone)
    {
        return;
    }
    maniQuat rest = maniSkeleton_GetRest(skeleton, node->bone).rotation;
    maniQuat r = maniQuatNormalize(maniQuatMul(maniQuatConjugate(rest), maniBodyLocal(part, k)));
    maniQuat limited = maniIkLimit(&s->limit, node->axis, r);
    if (limited.x == r.x && limited.y == r.y && limited.z == r.z && limited.w == r.w)
    {
        return;
    }
    // The limited local as a turn: its correction of the input local,
    // in model space, after the parent's turn.
    maniQuat local = maniQuatNormalize(maniQuatMul(rest, limited));
    maniQuat correction = maniQuatMul(local, maniQuatConjugate(node->local));
    maniQuat parentTurn = k == 0 ? maniQuatIdentity() : part->nodes[node->parent].turn;
    node->turn = maniQuatNormalize(
        maniQuatMul(parentTurn, maniIkFromParent(correction, &node->parentFrame)));
}

// A model-space displacement in node k's parent's local frame as it is
// now (its input frame after its turn).
static maniVec3 InParentVector(const maniBodyPart* part, uint32_t k, maniVec3 d)
{
    const maniBodyNode* node = &part->nodes[k];
    if (k != 0)
    {
        d = maniQuatRotate(maniQuatConjugate(part->nodes[node->parent].turn), d);
    }
    if (!node->parentFrame.scaled)
    {
        return maniQuatRotate(maniQuatConjugate(node->parentFrame.rotation), d);
    }
    maniAffine inverse;
    return maniAffineInverse(&node->parentFrame.affine, &inverse)
               ? maniAffineTransformVector(&inverse, d)
               : maniVec3Make(0.0f, 0.0f, 0.0f);
}

bool maniBodyFinish(maniBodyPart* part, maniPose* pose, const maniBodySolve* solve)
{
    bool reached = maniBodyMiss(part) <= solve->tolerance && solve->weight >= 1.0f;
    for (uint32_t k = 0; k < part->count; ++k)
    {
        maniBodyNode* node = &part->nodes[k];
        maniQuat parentTurn = k == 0 ? maniQuatIdentity() : part->nodes[node->parent].turn;
        maniQuat relative = maniQuatMul(maniQuatConjugate(parentTurn), node->turn);
        // An unturned or locked bone keeps its bits.
        const maniBodyBone* s = maniBodySetting(solve, node);
        bool locked = s != NULL && s->rotationStiffness >= 1.0f;
        if (!locked && (relative.x != 0.0f || relative.y != 0.0f || relative.z != 0.0f))
        {
            maniQuat correction =
                maniIkWeighted(maniIkInParent(relative, &node->parentFrame), solve->weight);
            maniIkCorrect(pose, node->bone, correction);
        }
        // Where the parent's turn alone puts the node; the top's input.
        maniVec3 placed = k == 0 ? maniIkOrigin(&node->frame)
                                 : maniVec3Add(part->nodes[node->parent].origin,
                                               maniQuatRotate(parentTurn, node->offset));
        maniVec3 moved = maniVec3Sub(node->origin, placed);
        if ((k == 0 || solve->stretch) && maniVec3Dot(moved, moved) > 0.0f)
        {
            maniTransform t = maniPose_GetLocal(pose, node->bone);
            maniVec3 d = maniVec3Scale(InParentVector(part, k, moved), solve->weight);
            t.translation = maniVec3Add(t.translation, d);
            maniPose_SetLocal(pose, node->bone, t);
        }
    }
    return reached;
}
