// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A 1D space blends the two points either side of the position (clamped
// to the ends, or wrapped across the seam); a triangulated space the
// corners of the triangle holding it, or the ends of the nearest edge; a
// banded space weighs each point by Johansen's gradient bands (2009,
// section 6.3), cartesian or polar, normalized. Binary32 in a fixed
// order.

#include "graph_space.h"

#include "bytes.h"
#include "graph_file.h"
#include "graph_instance.h"
#include "scalar.h"

#include "maul-anim/graph.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct Space
{
    const maniGraph* graph;
    const maniGraphCell* cells;
    uint32_t data;
    uint32_t count;
} Space;

static uint32_t Word(const Space* s, uint32_t at)
{
    return maniLoadU32(s->graph->data + 4 * (size_t)(s->data + at));
}

static float Float(const Space* s, uint32_t at)
{
    uint32_t bits = Word(s, at);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static float Scalar(const Space* s, uint32_t at)
{
    uint32_t parameter = Word(s, at);
    return parameter != MANI_NO_PARAMETER ? s->cells[parameter].value.f[0] : Float(s, at + 1);
}

// The position: a vec2 parameter's, or the two scalars'.
static void Position(const Space* s, float* x, float* y)
{
    uint32_t vector = Word(s, 0);
    if (vector != MANI_NO_PARAMETER)
    {
        *x = s->cells[vector].value.f[0];
        *y = s->cells[vector].value.f[1];
        return;
    }
    *x = Scalar(s, 1);
    *y = Scalar(s, 3);
}

// The points' order: the input of the r-th smallest point.
static uint32_t Ordered(const Space* s, uint32_t r)
{
    return Word(s, MANI_GRAPH_SPACE_WORDS + s->count + r);
}

static float Point(const Space* s, uint32_t k)
{
    return Float(s, MANI_GRAPH_SPACE_WORDS + k);
}

// Between inputs a and b (points pa < pb), at x within them.
static void Between(float* weights, uint32_t a, uint32_t b, float pa, float pb, float x)
{
    // pa <= x <= pb, and rounding keeps the order, so t is in [0, 1].
    float t = (x - pa) / (pb - pa);
    weights[a] = 1.0f - t;
    weights[b] = t;
}

static void Line(const Space* s, float x, float* weights)
{
    uint32_t n = s->count;
    uint32_t first = Ordered(s, 0);
    uint32_t last = Ordered(s, n - 1);
    float low = Point(s, first);
    float high = Point(s, last);
    float period = Float(s, MANI_GRAPH_POSITION_WORDS);
    if (n == 1)
    {
        weights[0] = 1.0f;
        return;
    }
    if (period > 0.0f)
    {
        float w = fmodf(x - low, period);
        x = low + (w < 0.0f ? w + period : w);
        if (x > high)
        {
            // Across the seam, from the last point to the first a period on.
            Between(weights, last, first, high, low + period, x);
            return;
        }
    }
    if (x <= low || x >= high)
    {
        weights[x <= low ? first : last] = 1.0f;
        return;
    }
    // The last point at or below x, by halves.
    uint32_t lo = 0;
    uint32_t hi = n - 1;
    while (hi - lo > 1)
    {
        uint32_t mid = lo + (hi - lo) / 2;
        if (Point(s, Ordered(s, mid)) <= x)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    uint32_t a = Ordered(s, lo);
    uint32_t b = Ordered(s, hi);
    Between(weights, a, b, Point(s, a), Point(s, b), x);
}

// The signed turn from direction a to direction b, in [-pi, pi]; 0 when
// either is the origin's.
static float Turn(float a, float b, bool origin)
{
    float d = origin ? 0.0f : b - a;
    return d > 3.14159265f ? d - 6.28318531f : (d < -3.14159265f ? d + 6.28318531f : d);
}

// A banded space's points, and per point (polar) its magnitude and
// direction, whether it is the origin (whose direction no turn uses),
// and the position's.
typedef struct Points
{
    float p[2 * MANI_MAX_GRAPH_SPACE_POINTS];
    float magnitude[MANI_MAX_GRAPH_SPACE_POINTS];
    float direction[MANI_MAX_GRAPH_SPACE_POINTS];
    bool origin[MANI_MAX_GRAPH_SPACE_POINTS];
    float at[2];
    float atMagnitude;
    float atDirection;
    bool atOrigin;
    bool polar;
} Points;

// The band vectors from point i to point j and to the position, polar
// (magnitudes over the pair's mean, twice the turn) or cartesian.
static void Bands(const Points* q, uint32_t i, uint32_t j, float* vij, float* vip)
{
    const float* pi = q->p + 2 * i;
    const float* pj = q->p + 2 * j;
    if (!q->polar)
    {
        vij[0] = pj[0] - pi[0];
        vij[1] = pj[1] - pi[1];
        vip[0] = q->at[0] - pi[0];
        vip[1] = q->at[1] - pi[1];
        return;
    }
    float mean = (q->magnitude[i] + q->magnitude[j]) * 0.5f;
    vij[0] = (q->magnitude[j] - q->magnitude[i]) / mean;
    vij[1] = 2.0f * Turn(q->direction[i], q->direction[j], q->origin[i] || q->origin[j]);
    vip[0] = (q->atMagnitude - q->magnitude[i]) / mean;
    vip[1] = 2.0f * Turn(q->direction[i], q->atDirection, q->origin[i] || q->atOrigin);
}

// The nearest point to the position, the first of equals.
static uint32_t Nearest(const float* p, uint32_t n, const float* at)
{
    uint32_t best = 0;
    float bestDistance = 0.0f;
    for (uint32_t k = 0; k < n; ++k)
    {
        float dx = p[2 * k] - at[0];
        float dy = p[2 * k + 1] - at[1];
        float d = dx * dx + dy * dy;
        if (k == 0 || d < bestDistance)
        {
            best = k;
            bestDistance = d;
        }
    }
    return best;
}

// The weights of triangle v's corners at the position: barycentric, in
// v[0]'s frame.
static void Corners(const Space* s, const uint32_t* v, const float* at, float* w)
{
    float ax = Point(s, 2 * v[0]);
    float ay = Point(s, 2 * v[0] + 1);
    float ux = Point(s, 2 * v[1]) - ax;
    float uy = Point(s, 2 * v[1] + 1) - ay;
    float vx = Point(s, 2 * v[2]) - ax;
    float vy = Point(s, 2 * v[2] + 1) - ay;
    float px = at[0] - ax;
    float py = at[1] - ay;
    float area = ux * vy - uy * vx;
    w[1] = (px * vy - py * vx) / area;
    w[2] = (ux * py - uy * px) / area;
    w[0] = 1.0f - w[1] - w[2];
}

// The first triangle holding the position weighs its corners. Outside
// every one (or where rounding puts it outside both triangles of an
// edge), the nearest point on any triangle's edge weighs that edge's
// ends: Godot's rule, continuous with the inside.
static void Triangulated(const Space* s, const float* at, float* weights)
{
    uint32_t count = Word(s, MANI_GRAPH_POSITION_WORDS);
    uint32_t corners = MANI_GRAPH_SPACE_WORDS + 2 * s->count;
    for (uint32_t t = 0; t < count; ++t)
    {
        const uint32_t v[3] = {Word(s, corners + 3 * t), Word(s, corners + 3 * t + 1),
                               Word(s, corners + 3 * t + 2)};
        float w[3];
        Corners(s, v, at, w);
        if (w[0] >= 0.0f && w[1] >= 0.0f && w[2] >= 0.0f)
        {
            weights[v[0]] = w[0];
            weights[v[1]] = w[1];
            weights[v[2]] = w[2];
            return;
        }
    }
    uint32_t ends[2] = {0, 0};
    float along = 0.0f;
    float best = HUGE_VALF;
    for (uint32_t e = 0; e < 3 * count; ++e)
    {
        uint32_t a = Word(s, corners + e);
        uint32_t b = Word(s, corners + e - e % 3 + (e + 1) % 3);
        float ax = Point(s, 2 * a);
        float ay = Point(s, 2 * a + 1);
        float ex = Point(s, 2 * b) - ax;
        float ey = Point(s, 2 * b + 1) - ay;
        // A NaN (an edge too short to measure) clamps to the edge's start.
        float u = ((at[0] - ax) * ex + (at[1] - ay) * ey) / (ex * ex + ey * ey);
        u = u > 0.0f ? (u < 1.0f ? u : 1.0f) : 0.0f;
        float dx = ax + u * ex - at[0];
        float dy = ay + u * ey - at[1];
        float d = dx * dx + dy * dy;
        if (e == 0 || d < best)
        {
            ends[0] = a;
            ends[1] = b;
            along = u;
            best = d;
        }
    }
    weights[ends[0]] = 1.0f - along;
    weights[ends[1]] = along;
}

static void Banded(const Space* s, const float* at, float* weights)
{
    uint32_t n = s->count;
    Points q;
    q.polar = Word(s, MANI_GRAPH_POSITION_WORDS) != 0;
    q.at[0] = at[0];
    q.at[1] = at[1];
    q.atMagnitude = sqrtf(at[0] * at[0] + at[1] * at[1]);
    q.atDirection = maniAtan2(at[1], at[0]);
    q.atOrigin = at[0] == 0.0f && at[1] == 0.0f;
    for (uint32_t k = 0; k < n; ++k)
    {
        float* pk = q.p + 2 * k;
        pk[0] = Point(s, 2 * k);
        pk[1] = Point(s, 2 * k + 1);
        q.magnitude[k] = sqrtf(pk[0] * pk[0] + pk[1] * pk[1]);
        q.direction[k] = maniAtan2(pk[1], pk[0]);
        q.origin[k] = pk[0] == 0.0f && pk[1] == 0.0f;
    }
    float sum = 0.0f;
    for (uint32_t i = 0; i < n; ++i)
    {
        float h = 1.0f;
        for (uint32_t j = 0; j < n; ++j)
        {
            float vij[2];
            float vip[2];
            Bands(&q, i, j, vij, vip);
            float d = vij[0] * vij[0] + vij[1] * vij[1];
            float band = d > 0.0f ? 1.0f - (vip[0] * vij[0] + vip[1] * vij[1]) / d : 1.0f;
            // A point's band to itself is 1 (d is 0): no lower than h.
            h = band < h ? band : h;
        }
        weights[i] = h > 0.0f ? h : 0.0f;
        sum += weights[i];
    }
    if (!(sum > 0.0f))
    {
        // No band reaches the position (polar spaces with gaps past half
        // a turn): the nearest point.
        for (uint32_t i = 0; i < n; ++i)
        {
            weights[i] = 0.0f;
        }
        weights[Nearest(q.p, n, at)] = 1.0f;
        return;
    }
    float inverse = 1.0f / sum;
    for (uint32_t i = 0; i < n; ++i)
    {
        weights[i] *= inverse;
    }
}

void maniGraphSpaceWeights(const maniGraph* graph, const maniGraphCell* cells, uint32_t kind,
                           uint32_t data, uint32_t count, float* weights)
{
    Space s = {graph, cells, data, count};
    for (uint32_t k = 0; k < count; ++k)
    {
        weights[k] = 0.0f;
    }
    float at[2];
    Position(&s, &at[0], &at[1]);
    if (kind == maniGraphNodeBlendSpace1D)
    {
        Line(&s, at[0], weights);
        return;
    }
    if (kind == maniGraphNodeBlendSpaceTriangles)
    {
        Triangulated(&s, at, weights);
        return;
    }
    Banded(&s, at, weights);
}
