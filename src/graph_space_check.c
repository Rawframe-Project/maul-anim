// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_space_check.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graph.h"
#include "maul-anim/graphbuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static bool ScalarValid(const maniGraphSource* source, maniGraphScalar s)
{
    return s.parameter == MANI_NO_PARAMETER
               ? isfinite(s.value)
               : s.parameter < source->parameterCount &&
                     source->parameters[s.parameter].type == maniGraphParameterFloat;
}

// A vec2 parameter, or two numbers (a 1D space's y unused).
static maniBuildResult CheckPosition(const maniGraphSource* source, const maniGraphPosition* p,
                                     bool line, uint32_t node)
{
    if (p->vector != MANI_NO_PARAMETER)
    {
        bool ok = p->vector < source->parameterCount &&
                  source->parameters[p->vector].type == maniGraphParameterVec2;
        return ok ? kOk : Refuse(maniBuildReasonScalar, node, 0);
    }
    if (!ScalarValid(source, p->x))
    {
        return Refuse(maniBuildReasonScalar, node, 0);
    }
    return line || ScalarValid(source, p->y) ? kOk : Refuse(maniBuildReasonScalar, node, 1);
}

uint64_t maniGraphSpaceKey(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x80000000u) != 0 ? (uint64_t)~bits : (uint64_t)(bits | 0x80000000u);
}

static maniBuildResult CheckLine(const maniGraphBlendSpace1DNodeSource* s, uint32_t count,
                                 uint32_t node)
{
    float period = s->period;
    if (!isfinite(period) || period < 0.0f)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    float low = s->points[0];
    float high = s->points[0];
    for (uint32_t k = 0; k < count; ++k)
    {
        float v = s->points[k];
        bool repeated = false;
        for (uint32_t j = 0; j < k && !repeated; ++j)
        {
            repeated = s->points[j] == v;
        }
        if (!isfinite(v) || repeated)
        {
            return Refuse(maniBuildReasonPoint, node, k);
        }
        low = v < low ? v : low;
        high = v > high ? v : high;
    }
    for (uint32_t k = 0; k < count && period > 0.0f; ++k)
    {
        if (s->points[k] == high && !(high - low < period))
        {
            return Refuse(maniBuildReasonPoint, node, k);
        }
    }
    return kOk;
}

static maniBuildResult CheckBands(const maniGraphBandsNodeSource* s, uint32_t count, uint32_t node)
{
    if (s->polar > 1u)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        const float* p = s->points + 2 * k;
        bool repeated = false;
        for (uint32_t j = 0; j < k && !repeated; ++j)
        {
            repeated = s->points[2 * j] == p[0] && s->points[2 * j + 1] == p[1];
        }
        if (!isfinite(p[0]) || !isfinite(p[1]) || repeated)
        {
            return Refuse(maniBuildReasonPoint, node, k);
        }
    }
    return kOk;
}

maniBuildResult maniCheckGraphSpace(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node)
{
    bool line = n->kind == maniGraphNodeBlendSpace1D;
    const maniGraphPosition* position = line ? &n->blendSpace1D.position : &n->bands.position;
    const float* points = line ? n->blendSpace1D.points : n->bands.points;
    if (points == NULL)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    maniBuildResult r = CheckPosition(source, position, line, node);
    if (r.result != mani_success)
    {
        return r;
    }
    return line ? CheckLine(&n->blendSpace1D, n->inputCount, node)
                : CheckBands(&n->bands, n->inputCount, node);
}
