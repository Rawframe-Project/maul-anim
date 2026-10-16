// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A saved block's words are finite but the pending request, which is
// +inf when there is none; seconds are not negative; flags name only the
// three states; directions and axes are unit or zero, as a capture leaves
// them (checked to a loose bound: the capture normalizes in binary32).

#include "graph_inertia.h"

#include "bytes.h"
#include "graph_file.h"

#include "maul-anim/graph.h"

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

void maniGraphResetInertia(uint32_t* block, uint32_t bones)
{
    memset(block, 0, 4 * (size_t)MANI_GRAPH_INERTIA_BLOCK(bones));
    maniGraphInertiaSet(block, maniGraphInertiaPending, HUGE_VALF);
}

static bool Seconds(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static uint32_t HeaderValid(const uint8_t* bytes)
{
    float pending = FloatOf(maniLoadU32(bytes + 4 * maniGraphInertiaPending));
    if (maniLoadU32(bytes) > 7u)
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

uint32_t maniGraphSavedInertiaValid(const uint8_t* bytes, uint32_t bones)
{
    uint32_t wrong = HeaderValid(bytes);
    for (uint32_t b = 0; b < bones && wrong == UINT32_MAX; ++b)
    {
        uint32_t at = maniGraphInertiaBone(b);
        for (uint32_t k = 0; k < MANI_GRAPH_INERTIA_CAPTURE && wrong == UINT32_MAX; ++k)
        {
            wrong =
                isfinite(FloatOf(maniLoadU32(bytes + 4 * (size_t)(at + k)))) ? UINT32_MAX : at + k;
        }
        for (uint32_t c = 0; c < 3 && wrong == UINT32_MAX; ++c)
        {
            uint32_t capture = at + MANI_GRAPH_INERTIA_CAPTURE + 6 * c;
            wrong = CaptureValid(bytes + 4 * (size_t)capture) ? UINT32_MAX : capture;
        }
    }
    return wrong;
}

static uint32_t Word(const maniGraph* g, uint32_t at)
{
    return maniLoadU32(g->data + 4 * (size_t)at);
}

static uint32_t ParameterType(const maniGraph* g, uint32_t parameter)
{
    return parameter < g->parameterCount
               ? maniLoadU32(g->parameters + (size_t)parameter * MANI_GRAPH_PARAMETER_SIZE)
               : UINT32_MAX;
}

// Its trigger none or a trigger; its duration a float parameter or a
// literal finite and not negative; its block where the count puts it.
uint32_t maniGraphInertiaValid(const maniGraph* g, uint32_t data, uint32_t block)
{
    uint32_t trigger = Word(g, data);
    uint32_t parameter = Word(g, data + 1);
    if (trigger != MANI_NO_PARAMETER && ParameterType(g, trigger) != maniGraphParameterTrigger)
    {
        return data;
    }
    bool literal = parameter == MANI_NO_PARAMETER && Seconds(FloatOf(Word(g, data + 2)));
    if (!(literal ||
          (ParameterType(g, parameter) == maniGraphParameterFloat && Word(g, data + 2) == 0)))
    {
        return data + 1;
    }
    return Word(g, data + 3) == block ? UINT32_MAX : data + 3;
}
