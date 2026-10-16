// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "graph_space_check.h"

#include "exact.h"
#include "graph_file.h"

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

// Points in a plane: finite, no two equal.
static maniBuildResult CheckPairs(const float* points, uint32_t count, uint32_t node)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        const float* p = points + 2 * k;
        bool repeated = false;
        for (uint32_t j = 0; j < k && !repeated; ++j)
        {
            repeated = points[2 * j] == p[0] && points[2 * j + 1] == p[1];
        }
        if (!isfinite(p[0]) || !isfinite(p[1]) || repeated)
        {
            return Refuse(maniBuildReasonPoint, node, k);
        }
    }
    return kOk;
}

static maniBuildResult CheckBands(const maniGraphBandsNodeSource* s, uint32_t count, uint32_t node)
{
    return s->polar > 1u ? Refuse(maniBuildReasonNode, node, 0)
                         : CheckPairs(s->points, count, node);
}

// Whether every corner of b lies on or right of one of a's edges.
static bool SeparatedBy(const float* points, const uint32_t* a, const uint32_t* b)
{
    for (uint32_t i = 0; i < 3; ++i)
    {
        const float* u = points + 2 * (size_t)a[i];
        const float* v = points + 2 * (size_t)a[(i + 1) % 3];
        bool right = true;
        for (uint32_t k = 0; k < 3 && right; ++k)
        {
            right = maniExactOrient(u, v, points + 2 * (size_t)b[k]) <= 0;
        }
        if (right)
        {
            return true;
        }
    }
    return false;
}

// Whether the interiors of two anticlockwise triangles meet: apart in x
// or y, or apart across an edge of either (the separating axis test).
static bool Overlap(const float* points, const uint32_t* a, const uint32_t* b)
{
    for (uint32_t axis = 0; axis < 2; ++axis)
    {
        float low[2] = {HUGE_VALF, HUGE_VALF};
        float high[2] = {-HUGE_VALF, -HUGE_VALF};
        for (uint32_t k = 0; k < 3; ++k)
        {
            float va = points[2 * (size_t)a[k] + axis];
            float vb = points[2 * (size_t)b[k] + axis];
            low[0] = va < low[0] ? va : low[0];
            high[0] = va > high[0] ? va : high[0];
            low[1] = vb < low[1] ? vb : low[1];
            high[1] = vb > high[1] ? vb : high[1];
        }
        if (high[0] <= low[1] || high[1] <= low[0])
        {
            return false;
        }
    }
    return !SeparatedBy(points, a, b) && !SeparatedBy(points, b, a);
}

// Authored triangles: corners among the points, anticlockwise, no two
// overlapping, every point a corner.
static maniBuildResult CheckAuthored(const maniGraphTrianglesNodeSource* s, uint32_t count,
                                     uint32_t node)
{
    if (s->triangles == NULL || s->triangleCount > MANI_GRAPH_MAX_TRIANGLES(count))
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    bool used[MANI_GRAPH_MAX_SPACE_POINTS] = {false};
    for (uint32_t t = 0; t < s->triangleCount; ++t)
    {
        const uint32_t* v = s->triangles + 3 * (size_t)t;
        bool ok = v[0] < count && v[1] < count && v[2] < count &&
                  maniExactOrient(s->points + 2 * (size_t)v[0], s->points + 2 * (size_t)v[1],
                                  s->points + 2 * (size_t)v[2]) > 0;
        for (uint32_t j = 0; j < t && ok; ++j)
        {
            ok = !Overlap(s->points, s->triangles + 3 * (size_t)j, v);
        }
        if (!ok)
        {
            return Refuse(maniBuildReasonSpaceTriangle, node, t);
        }
        used[v[0]] = used[v[1]] = used[v[2]] = true;
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!used[k])
        {
            return Refuse(maniBuildReasonPoint, node, k);
        }
    }
    return kOk;
}

// Triangles: the points, then the authored triangles, or for a Delaunay
// triangulation points not all on one line.
static maniBuildResult CheckTriangles(const maniGraphTrianglesNodeSource* s, uint32_t count,
                                      uint32_t node)
{
    maniBuildResult r = CheckPairs(s->points, count, node);
    if (r.result != mani_success || s->triangleCount > 0)
    {
        return r.result != mani_success ? r : CheckAuthored(s, count, node);
    }
    for (uint32_t k = 2; k < count; ++k)
    {
        if (maniExactOrient(s->points, s->points + 2, s->points + 2 * (size_t)k) != 0)
        {
            return kOk;
        }
    }
    return Refuse(maniBuildReasonNode, node, 0);
}

maniBuildResult maniCheckGraphSpace(const maniGraphSource* source, const maniGraphNodeSource* n,
                                    uint32_t node)
{
    bool line = n->kind == maniGraphNodeBlendSpace1D;
    bool bands = n->kind == maniGraphNodeBlendSpaceBands;
    const maniGraphPosition* position = line    ? &n->blendSpace1D.position
                                        : bands ? &n->bands.position
                                                : &n->triangulated.position;
    const float* points =
        line ? n->blendSpace1D.points : (bands ? n->bands.points : n->triangulated.points);
    if (points == NULL)
    {
        return Refuse(maniBuildReasonNode, node, 0);
    }
    maniBuildResult r = CheckPosition(source, position, line, node);
    if (r.result != mani_success)
    {
        return r;
    }
    if (line)
    {
        return CheckLine(&n->blendSpace1D, n->inputCount, node);
    }
    return bands ? CheckBands(&n->bands, n->inputCount, node)
                 : CheckTriangles(&n->triangulated, n->inputCount, node);
}
