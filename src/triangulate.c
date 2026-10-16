// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The points swept in coordinate order (x, then y) into a fan, each
// joined to the hull edges it sees strictly; then Lawson's flips of every
// edge whose far point lies strictly inside the circle across it, so
// cocircular ties stay as the sweep made them. Every test is exact.

#include "triangulate.h"

#include "exact.h"
#include "sort.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NONE MANI_TRIANGULATE_MAX

typedef struct Sweep
{
    const float* points;
    maniTriangulation* m;
} Sweep;

static const float* P(const Sweep* s, uint32_t v)
{
    return s->points + 2 * (size_t)v;
}

// The slot of t opposite its edge from a to b, or 3.
static uint32_t SlotOf(const maniTriangle* t, uint32_t a, uint32_t b)
{
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (t->v[(i + 1) % 3] == a && t->v[(i + 2) % 3] == b)
        {
            return i;
        }
    }
    return 3;
}

static uint32_t Add(maniTriangulation* m, uint32_t a, uint32_t b, uint32_t c)
{
    m->t[m->count] = (maniTriangle){{a, b, c}, {NONE, NONE, NONE}};
    return m->count++;
}

// Joins t and u across the edge they share, if both are triangles.
static void Join(maniTriangulation* m, uint32_t t, uint32_t u)
{
    if (t == NONE || u == NONE)
    {
        return;
    }
    for (uint32_t i = 0; i < 3; ++i)
    {
        uint32_t a = m->t[t].v[(i + 1) % 3];
        uint32_t b = m->t[t].v[(i + 2) % 3];
        uint32_t j = SlotOf(&m->t[u], b, a);
        if (j < 3)
        {
            m->t[t].n[i] = u;
            m->t[u].n[j] = t;
        }
    }
}

// The first fan: the k collinear points before the first off their
// line, each pair with that point; the hull around them.
static void Start(Sweep* s, uint32_t k)
{
    maniTriangulation* m = s->m;
    const uint32_t* order = m->order;
    uint32_t apex = order[k];
    bool left = maniExactOrient(P(s, order[0]), P(s, order[1]), P(s, apex)) > 0;
    uint32_t previous = NONE;
    for (uint32_t i = 0; i + 1 < k; ++i)
    {
        uint32_t t =
            left ? Add(m, order[i], order[i + 1], apex) : Add(m, order[i + 1], order[i], apex);
        Join(m, t, previous);
        previous = t;
    }
    // Anticlockwise: the chain then the apex, or the apex then the chain
    // back.
    m->hullCount = k + 1;
    m->hull[0] = order[0];
    for (uint32_t i = 1; i < k; ++i)
    {
        m->hull[left ? i : k + 1 - i] = order[i];
    }
    m->hull[left ? k : 1] = apex;
    for (uint32_t i = 0; i < m->hullCount; ++i)
    {
        uint32_t a = m->hull[i];
        uint32_t b = m->hull[(i + 1) % m->hullCount];
        m->hullTriangle[i] = NONE;
        for (uint32_t t = 0; t < m->count; ++t)
        {
            m->hullTriangle[i] = SlotOf(&m->t[t], a, b) < 3 ? t : m->hullTriangle[i];
        }
    }
}

static bool Sees(const Sweep* s, uint32_t i, uint32_t p)
{
    const maniTriangulation* m = s->m;
    uint32_t a = m->hull[i % m->hullCount];
    uint32_t b = m->hull[(i + 1) % m->hullCount];
    return maniExactOrient(P(s, a), P(s, b), P(s, p)) < 0;
}

// Joins p, outside the hull, to the hull edges it sees (a run of them),
// and puts it in the hull in their place.
static void Insert(Sweep* s, uint32_t p)
{
    maniTriangulation* m = s->m;
    uint32_t h = m->hullCount;
    uint32_t first = 0;
    while (!Sees(s, first, p))
    {
        ++first;
    }
    while (Sees(s, first + h - 1, p))
    {
        first = (first + h - 1) % h;
    }
    uint32_t seen = 0;
    uint32_t previous = NONE;
    uint32_t firstNew = NONE;
    while (Sees(s, first + seen, p))
    {
        uint32_t i = (first + seen) % h;
        uint32_t t = Add(m, m->hull[(i + 1) % h], m->hull[i], p);
        Join(m, t, m->hullTriangle[i]);
        Join(m, t, previous);
        firstNew = seen == 0 ? t : firstNew;
        previous = t;
        ++seen;
    }
    // The new hull: from the run's end round to its start, then p.
    uint32_t hull[MANI_TRIANGULATE_MAX_POINTS];
    uint32_t edges[MANI_TRIANGULATE_MAX_POINTS];
    uint32_t kept = h - seen + 1;
    for (uint32_t k = 0; k < kept; ++k)
    {
        uint32_t i = (first + seen + k) % h;
        hull[k] = m->hull[i];
        edges[k] = m->hullTriangle[i];
    }
    edges[kept - 1] = firstNew;
    hull[kept] = p;
    edges[kept] = previous;
    memcpy(m->hull, hull, (kept + 1) * sizeof hull[0]);
    memcpy(m->hullTriangle, edges, (kept + 1) * sizeof edges[0]);
    m->hullCount = kept + 1;
}

static void Push(maniTriangulation* m, uint32_t* tail, uint32_t t, uint32_t i)
{
    uint32_t e = 3 * t + i;
    if (m->t[t].n[i] != NONE && m->queued[e] == 0)
    {
        m->queued[e] = 1;
        m->queue[*tail % (3 * MANI_TRIANGULATE_MAX)] = e;
        *tail += 1;
    }
}

// Points the neighbour x's slot that held old (across its edge from a to
// b) at now.
static void Repoint(maniTriangulation* m, uint32_t x, uint32_t a, uint32_t b, uint32_t now)
{
    if (x != NONE)
    {
        m->t[x].n[SlotOf(&m->t[x], a, b)] = now;
    }
}

// Flips the edge of t opposite slot i when the far point lies strictly
// inside the circle through t's corners.
static void Flip(Sweep* s, uint32_t t, uint32_t i, uint32_t* tail)
{
    maniTriangulation* m = s->m;
    uint32_t o = m->t[t].n[i];
    uint32_t a = m->t[t].v[i];
    uint32_t b = m->t[t].v[(i + 1) % 3];
    uint32_t c = m->t[t].v[(i + 2) % 3];
    uint32_t j = SlotOf(&m->t[o], c, b);
    uint32_t d = m->t[o].v[j];
    if (maniExactInCircle(P(s, a), P(s, b), P(s, c), P(s, d)) <= 0)
    {
        return;
    }
    uint32_t ab = m->t[t].n[(i + 2) % 3];
    uint32_t ca = m->t[t].n[(i + 1) % 3];
    uint32_t bd = m->t[o].n[(j + 1) % 3];
    uint32_t dc = m->t[o].n[(j + 2) % 3];
    m->t[t] = (maniTriangle){{a, b, d}, {bd, o, ab}};
    m->t[o] = (maniTriangle){{a, d, c}, {dc, ca, t}};
    Repoint(m, bd, d, b, t);
    Repoint(m, ca, a, c, o);
    Push(m, tail, t, 0);
    Push(m, tail, t, 2);
    Push(m, tail, o, 0);
    Push(m, tail, o, 1);
}

// Flips until every edge is locally Delaunay: a queue of edges, each in
// it at most once.
static void Flips(Sweep* s)
{
    maniTriangulation* m = s->m;
    uint32_t head = 0;
    uint32_t tail = 0;
    memset(m->queued, 0, sizeof m->queued);
    for (uint32_t t = 0; t < m->count; ++t)
    {
        for (uint32_t i = 0; i < 3; ++i)
        {
            Push(m, &tail, t, i);
        }
    }
    while (head != tail)
    {
        uint32_t e = m->queue[head % (3 * MANI_TRIANGULATE_MAX)];
        head += 1;
        m->queued[e] = 0;
        if (m->t[e / 3].n[e % 3] != NONE)
        {
            Flip(s, e / 3, e % 3, &tail);
        }
    }
}

uint32_t maniTriangulate(const float* points, uint32_t n, maniTriangulation* m)
{
    Sweep s = {points, m};
    m->count = 0;
    for (uint32_t k = 0; k < n; ++k)
    {
        m->keys[k] = (maniFloatKey(points[2 * k]) << 32) + maniFloatKey(points[2 * k + 1]);
    }
    maniSortByKey(m->keys, n, m->order, m->scratch);
    uint32_t k = 2;
    while (k < n &&
           maniExactOrient(P(&s, m->order[0]), P(&s, m->order[1]), P(&s, m->order[k])) == 0)
    {
        ++k;
    }
    if (k == n)
    {
        return 0;
    }
    Start(&s, k);
    for (uint32_t i = k + 1; i < n; ++i)
    {
        Insert(&s, m->order[i]);
    }
    Flips(&s);
    return m->count;
}
