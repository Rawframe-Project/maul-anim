// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A saved block's words are finite but the pending request, which is
// +inf when there is none; seconds are not negative; flags name only the
// four states; inertialization's directions and axes are unit or zero,
// as a capture leaves them (checked to a loose bound: the capture
// normalizes in binary32); dead blending's decay rates are not negative.

#include "graph_inertia.h"

#include "bytes.h"
#include "graph_file.h"
#include "pose_memory.h"

#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static float FloatOf(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

float maniGraphInertiaGet(const uint32_t* block, uint32_t at)
{
    return FloatOf(block[at]);
}

void maniGraphInertiaSet(uint32_t* block, uint32_t at, float value)
{
    memcpy(&block[at], &value, sizeof value);
}

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

maniGraphInertia maniGraphInertiaOf(const maniGraph* g, uint32_t data)
{
    return (maniGraphInertia){g->skeletonBoneCount, Word(g, data + maniGraphInertiaMethod),
                              FloatOf(Word(g, data + maniGraphInertiaHalfLifeScale)),
                              FloatOf(Word(g, data + maniGraphInertiaHalfLifeMin)),
                              FloatOf(Word(g, data + maniGraphInertiaHalfLifeMax))};
}

maniGraphTrs maniGraphTrsRead(const float* slot)
{
    float v[MANI_POSE_COMPONENTS];
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        v[c] = slot[c * MANI_POSE_GROUP_BONES];
    }
    return (maniGraphTrs){{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}, {v[7], v[8], v[9]}};
}

void maniGraphTrsWrite(float* slot, maniGraphTrs x)
{
    const float v[MANI_POSE_COMPONENTS] = {x.t.x, x.t.y, x.t.z, x.r.x, x.r.y,
                                           x.r.z, x.r.w, x.s.x, x.s.y, x.s.z};
    maniPoseSlotPut(slot, 0, v, MANI_POSE_COMPONENTS);
}

maniGraphTrs maniGraphTrsKept(const uint32_t* block, uint32_t at)
{
    float v[MANI_POSE_COMPONENTS];
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        v[c] = maniGraphInertiaGet(block, at + c);
    }
    return (maniGraphTrs){{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}, {v[7], v[8], v[9]}};
}

void maniGraphTrsKeep(uint32_t* block, uint32_t at, maniGraphTrs x)
{
    const float v[MANI_POSE_COMPONENTS] = {x.t.x, x.t.y, x.t.z, x.r.x, x.r.y,
                                           x.r.z, x.r.w, x.s.x, x.s.y, x.s.z};
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        maniGraphInertiaSet(block, at + c, v[c]);
    }
}

// Bone b's component c in a history at word at.
static uint32_t Lane(uint32_t at, uint32_t b, uint32_t c)
{
    return at + b / MANI_POSE_GROUP_BONES * (MANI_POSE_GROUP_BONES * MANI_POSE_COMPONENTS) +
           c * MANI_POSE_GROUP_BONES + b % MANI_POSE_GROUP_BONES;
}

maniGraphTrs maniGraphTrsHeld(const uint32_t* block, uint32_t at, uint32_t b)
{
    float v[MANI_POSE_COMPONENTS];
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        v[c] = maniGraphInertiaGet(block, Lane(at, b, c));
    }
    return (maniGraphTrs){{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}, {v[7], v[8], v[9]}};
}

void maniGraphTrsHold(uint32_t* block, uint32_t at, uint32_t b, maniGraphTrs x)
{
    const float v[MANI_POSE_COMPONENTS] = {x.t.x, x.t.y, x.t.z, x.r.x, x.r.y,
                                           x.r.z, x.r.w, x.s.x, x.s.y, x.s.z};
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        maniGraphInertiaSet(block, Lane(at, b, c), v[c]);
    }
}

void maniGraphResetInertia(uint32_t* block, const maniGraphInertia* node)
{
    memset(block, 0, 4 * (size_t)MANI_GRAPH_INERTIA_BLOCK(node->bones, node->method));
    maniGraphInertiaSet(block, maniGraphInertiaPending, HUGE_VALF);
}

static bool Seconds(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static uint32_t HeaderValid(const uint8_t* bytes)
{
    float pending = FloatOf(maniLoadU32(bytes + 4 * maniGraphInertiaPending));
    if (maniLoadU32(bytes) > 15u)
    {
        return maniGraphInertiaFlags;
    }
    const uint32_t clocks[4] = {maniGraphInertiaStep, maniGraphInertiaElapsed,
                                maniGraphInertiaDuration, maniGraphInertiaLastStep};
    for (uint32_t k = 0; k < 4; ++k)
    {
        if (!Seconds(FloatOf(maniLoadU32(bytes + 4 * clocks[k]))))
        {
            return clocks[k];
        }
    }
    if (!(pending >= 0.0f))
    {
        return maniGraphInertiaPending;
    }
    return maniLoadU32(bytes + 4 * 9) == 0 ? UINT32_MAX : 9u;
}

// A channel's capture: a direction of length 0 or about 1, the rest
// finite, its duration not negative.
static bool CaptureValid(const uint8_t* bytes)
{
    float v[6];
    for (uint32_t k = 0; k < 6; ++k)
    {
        v[k] = FloatOf(maniLoadU32(bytes + 4 * k));
        if (!isfinite(v[k]))
        {
            return false;
        }
    }
    float length = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    return (length == 0.0f || (length > 0.99f && length < 1.01f)) && v[5] >= 0.0f;
}

// Dead blending's capture: finite, its decay rates (the last nine) not
// negative.
static uint32_t DeadValid(const uint8_t* bytes)
{
    for (uint32_t k = 0; k < 28; ++k)
    {
        float v = FloatOf(maniLoadU32(bytes + 4 * k));
        if (!isfinite(v) || (k >= 19 && v < 0.0f))
        {
            return k;
        }
    }
    return UINT32_MAX;
}

uint32_t maniGraphSavedInertiaValid(const uint8_t* bytes, const maniGraphInertia* node)
{
    uint32_t wrong = HeaderValid(bytes);
    uint32_t history = 2u * MANI_GRAPH_INERTIA_HISTORY(node->bones);
    for (uint32_t k = MANI_GRAPH_INERTIA_HEADER;
         k < MANI_GRAPH_INERTIA_HEADER + history && wrong == UINT32_MAX; ++k)
    {
        wrong = isfinite(FloatOf(maniLoadU32(bytes + 4 * (size_t)k))) ? UINT32_MAX : k;
    }
    for (uint32_t b = 0; b < node->bones && wrong == UINT32_MAX; ++b)
    {
        uint32_t capture = maniGraphInertiaCapture(node, b);
        if (node->method != maniGraphInertialization)
        {
            uint32_t k = DeadValid(bytes + 4 * (size_t)capture);
            wrong = k == UINT32_MAX ? k : capture + k;
        }
        for (uint32_t c = 0;
             c < 3 && node->method == maniGraphInertialization && wrong == UINT32_MAX; ++c)
        {
            wrong =
                CaptureValid(bytes + 4 * (size_t)(capture + 6 * c)) ? UINT32_MAX : capture + 6 * c;
        }
    }
    return wrong;
}

static uint32_t ParameterType(const maniGraph* g, uint32_t parameter)
{
    return parameter < g->parameterCount
               ? maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE)
               : UINT32_MAX;
}

uint32_t maniGraphHalfLivesValid(const maniGraphInertia* node)
{
    const float v[3] = {node->halfLifeScale, node->halfLifeMin, node->halfLifeMax};
    bool dead = node->method != maniGraphInertialization;
    const bool valid[3] = {dead ? Seconds(v[0]) : v[0] == 0.0f,
                           dead ? v[1] > 0.0f && v[1] < HUGE_VALF : v[1] == 0.0f,
                           dead ? v[2] >= v[1] && v[2] < HUGE_VALF : v[2] == 0.0f};
    for (uint32_t k = 0; k < 3; ++k)
    {
        if (!valid[k])
        {
            return k;
        }
    }
    return UINT32_MAX;
}

// Its trigger none or a trigger; its method known; its duration a float
// parameter or a literal finite and not negative; its half-lives as its
// method wants them; its block where the count puts it.
uint32_t maniGraphInertiaValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    uint32_t trigger = Word(g, data);
    uint32_t method = Word(g, data + maniGraphInertiaMethod);
    uint32_t parameter = Word(g, data + maniGraphInertiaDurationScalar);
    float value = FloatOf(Word(g, data + maniGraphInertiaDurationScalar + 1));
    if (trigger != MANI_NO_PARAMETER && ParameterType(g, trigger) != maniGraphParameterTrigger)
    {
        return data;
    }
    if (method > maniGraphDeadBlending)
    {
        return data + maniGraphInertiaMethod;
    }
    bool literal = parameter == MANI_NO_PARAMETER && Seconds(value);
    if (!(literal || (ParameterType(g, parameter) == maniGraphParameterFloat &&
                      Word(g, data + maniGraphInertiaDurationScalar + 1) == 0)))
    {
        return data + maniGraphInertiaDurationScalar;
    }
    maniGraphInertia node = maniGraphInertiaOf(g, data);
    uint32_t wrong = maniGraphHalfLivesValid(&node);
    if (wrong != UINT32_MAX)
    {
        return data + maniGraphInertiaHalfLifeScale + wrong;
    }
    return Word(g, data + maniGraphInertiaBlock) == block ? UINT32_MAX
                                                          : data + maniGraphInertiaBlock;
}
