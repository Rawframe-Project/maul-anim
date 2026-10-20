// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A node's words name its mask, its speed (a scalar) and its reset
// trigger; a saved block keeps a time finite and not negative and each
// bone's rotation all zero (holding nothing) or unit (kept normalized). A step turns each bone from
// the rotation the block holds toward its input's by at most its budget, about the relative turn's
// axis: the slerp to budget over angle in exact arithmetic, built from
// the axis and the budget's half-angle.

#include "graph_turn.h"

#include "bytes.h"
#include "graph_file.h"
#include "pose_memory.h"
#include "scalar.h"
#include "vecmath.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static bool ParameterOf(const maniGraph* g, uint32_t parameter, uint32_t type)
{
    return parameter == MANI_NO_PARAMETER ||
           (parameter < g->parameterCount &&
            maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE) == type);
}

uint32_t maniGraphTurnBlockWords(uint32_t bones)
{
    return maniGraphTurnRotations + 4 * bones;
}

uint32_t maniGraphTurnValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    uint32_t mask = Word(g, data + maniGraphTurnMask);
    if (mask != MANI_GRAPH_NO_MASK && mask >= g->maskCount)
    {
        return data + maniGraphTurnMask;
    }
    uint32_t speed = Word(g, data + maniGraphTurnSpeed);
    uint32_t bits = Word(g, data + maniGraphTurnSpeed + 1);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    bool literal = speed == MANI_NO_PARAMETER && isfinite(value) && value >= 0.0f;
    if (!literal && !(speed != MANI_NO_PARAMETER && bits == 0 &&
                      ParameterOf(g, speed, maniGraphParameterFloat)))
    {
        return data + maniGraphTurnSpeed;
    }
    if (!ParameterOf(g, Word(g, data + maniGraphTurnReset), maniGraphParameterTrigger))
    {
        return data + maniGraphTurnReset;
    }
    return Word(g, data + maniGraphTurnBlock) == block ? UINT32_MAX : data + maniGraphTurnBlock;
}

static double Pending(const uint32_t* block)
{
    uint64_t bits =
        (uint64_t)block[maniGraphTurnPending] | (uint64_t)block[maniGraphTurnPending + 1] << 32;
    double v = 0.0;
    memcpy(&v, &bits, sizeof v);
    return v;
}

static void SetPending(uint32_t* block, double v)
{
    uint64_t bits = 0;
    memcpy(&bits, &v, sizeof bits);
    block[maniGraphTurnPending] = (uint32_t)bits;
    block[maniGraphTurnPending + 1] = (uint32_t)(bits >> 32);
}

static float FloatOf(uint32_t bits)
{
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

uint32_t maniGraphSavedTurnValid(const uint8_t* bytes, uint32_t bones)
{
    double pending = 0.0;
    uint64_t bits = maniLoadU64(bytes + 4 * maniGraphTurnPending);
    memcpy(&pending, &bits, sizeof pending);
    if (!(pending >= 0.0 && isfinite(pending)))
    {
        return maniGraphTurnPending;
    }
    for (uint32_t b = 0; b < bones; ++b)
    {
        uint32_t at = maniGraphTurnRotations + 4 * b;
        float q[4];
        float length = 0.0f;
        for (uint32_t k = 0; k < 4; ++k)
        {
            q[k] = FloatOf(maniLoadU32(bytes + 4 * (size_t)(at + k)));
            if (!isfinite(q[k]))
            {
                return at + k;
            }
            length += q[k] * q[k];
        }
        if (length != 0.0f && !(length >= 0.998f && length <= 1.002f))
        {
            return at;
        }
    }
    return UINT32_MAX;
}

void maniGraphTurnAdd(uint32_t* block, double seconds)
{
    SetPending(block, Pending(block) + seconds);
}

void maniGraphResetTurn(uint32_t* block, uint32_t bones)
{
    SetPending(block, 0.0);
    memset(block + maniGraphTurnRotations, 0, 16 * (size_t)bones);
}

static maniQuat Held(const uint32_t* block, uint32_t bone)
{
    const uint32_t* r = block + maniGraphTurnRotations + 4 * (size_t)bone;
    return (maniQuat){FloatOf(r[0]), FloatOf(r[1]), FloatOf(r[2]), FloatOf(r[3])};
}

static void Hold(uint32_t* block, uint32_t bone, maniQuat q)
{
    float v[4] = {q.x, q.y, q.z, q.w};
    memcpy(block + maniGraphTurnRotations + 4 * (size_t)bone, v, sizeof v);
}

// From p toward q by at most budget radians: q itself when within it,
// p itself for no budget, else p turned by budget about the axis of
// p^-1 q on the shorter arc (normalized).
static maniQuat Turn(maniQuat p, maniQuat q, float budget, bool* turnedOut)
{
    *turnedOut = false;
    maniQuat d = maniQuatMul(maniQuatConjugate(p), q);
    if (d.w < 0.0f)
    {
        d = (maniQuat){-d.x, -d.y, -d.z, -d.w};
    }
    float v = sqrtf((d.x * d.x + d.y * d.y) + d.z * d.z);
    float angle = 2.0f * maniAtan2(v, d.w);
    if (angle <= budget)
    {
        return q;
    }
    *turnedOut = true;
    if (!(budget > 0.0f))
    {
        return p;
    }
    float s = 0.0f;
    float c = 1.0f;
    maniSinCos(0.5f * budget, &s, &c);
    float k = s / v;
    return maniQuatNormalize(maniQuatMul(p, (maniQuat){d.x * k, d.y * k, d.z * k, c}));
}

void maniGraphTurnPose(uint32_t* block, const float* weights, float speed, const uint32_t* list,
                       uint32_t count, maniPose* pose)
{
    uint32_t bones = list != NULL ? count : maniPoseBoneCount(pose);
    float seconds = (float)Pending(block);
    SetPending(block, 0.0);
    // The rotations read and written in place (components 3 to 6 of a
    // bone's slot), the model marked stale once.
    float* locals = maniPoseLocals(pose);
    for (uint32_t i = 0; i < bones; ++i)
    {
        uint32_t b = list != NULL ? list[i] : i;
        float* slot = maniPoseSlot(locals, b) + 3 * MANI_POSE_GROUP_BONES;
        maniQuat q = {slot[0], slot[MANI_POSE_GROUP_BONES], slot[2 * MANI_POSE_GROUP_BONES],
                      slot[3 * MANI_POSE_GROUP_BONES]};
        maniQuat p = Held(block, b);
        bool held = p.x != 0.0f || p.y != 0.0f || p.z != 0.0f || p.w != 0.0f;
        float weight = weights != NULL ? weights[b] : 1.0f;
        bool turned = false;
        if (held && weight > 0.0f)
        {
            // The weight is how much the limit holds: the budget grows as
            // it falls, without bound toward 0, where the bone is free.
            q = Turn(p, q, speed * seconds / weight, &turned);
            const float v[4] = {q.x, q.y, q.z, q.w};
            maniPoseSlotPut(slot, 0, v, 4);
        }
        // A turned rotation is unit and kept as given, so a held bone
        // gives the same bits again; an input is kept normalized (the
        // identity for one of no length).
        Hold(block, b, turned ? q : maniQuatNormalize(q));
    }
}
