// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Exact predicates and Delaunay triangulations: orientation
// and in-circle signs against long double on small integers, the same at
// scales from 2^-140 to 2^100, subnormals and the widest finite values;
// triangulations of random sets (grids full of ties, collinear runs,
// circles, wide exponents) and a grid checked by brute force: 2n - 2 - h
// triangles, every one anticlockwise, no point strictly inside any
// circumcircle, neighbours each other's, every point a corner; points on
// one line refused.

#include "exact.h"
#include "test_harness.h"
#include "triangulate.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t s_seed = 12345u;

static uint32_t Next(uint32_t range)
{
    s_seed = s_seed * 1664525u + 1013904223u;
    return (s_seed >> 8) % range;
}

static int Sign(long double v)
{
    return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

static int OrientWide(const float* a, const float* b, const float* c)
{
    long double ux = (long double)b[0] - (long double)a[0];
    long double uy = (long double)b[1] - (long double)a[1];
    long double vx = (long double)c[0] - (long double)a[0];
    long double vy = (long double)c[1] - (long double)a[1];
    return Sign(ux * vy - uy * vx);
}

static int InCircleWide(const float* p, const float* d)
{
    long double m[3][3];
    for (int i = 0; i < 3; ++i)
    {
        long double x = (long double)p[2 * i] - (long double)d[0];
        long double y = (long double)p[2 * i + 1] - (long double)d[1];
        m[i][0] = x;
        m[i][1] = y;
        m[i][2] = x * x + y * y;
    }
    return Sign(m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]));
}

static void TestPredicates(void)
{
    static const float kA[2] = {0, 0};
    static const float kB[2] = {1, 0};
    static const float kC[2] = {0, 1};
    static const float kD[2] = {1, 1};
    static const float kE[2] = {0.5f, 0.5f};
    static const float kF[2] = {2, 2};
    CHECK(maniExactOrient(kA, kB, kC) == 1 && maniExactOrient(kA, kC, kB) == -1 &&
              maniExactOrient(kA, kD, kF) == 0,
          "orientations");
    CHECK(maniExactInCircle(kA, kB, kC, kD) == 0 && maniExactInCircle(kA, kB, kC, kE) == 1 &&
              maniExactInCircle(kA, kB, kC, kF) == -1,
          "on, inside and outside a circle");
    float s0[2] = {ldexpf(1, -149), 0};
    float s1[2] = {ldexpf(2, -149), 0};
    float s2[2] = {ldexpf(3, -149), 0};
    float up[2] = {ldexpf(1, -149), ldexpf(1, -149)};
    CHECK(maniExactOrient(s0, s1, s2) == 0 && maniExactOrient(s0, s1, up) == 1, "subnormals");
    // Each point a normal and a subnormal coordinate, b twice a.
    float origin[2] = {0, 0};
    float a[2] = {ldexpf(1, -126), ldexpf(1, -127)};
    float b[2] = {ldexpf(1, -125), ldexpf(1, -126)};
    CHECK(maniExactOrient(origin, a, b) == 0, "normals and subnormals on one line");
    float w0[2] = {ldexpf(1, -149), 0};
    float w1[2] = {3.4028235e38f, -3.4028235e38f};
    float w2[2] = {-3.4028235e38f, ldexpf(1, -149)};
    float w3[2] = {3.4028235e38f, 3.4028235e38f};
    CHECK(maniExactOrient(w0, w1, w2) == -1 && maniExactInCircle(w0, w1, w3, w2) != 0,
          "the widest values");
    bool same = true;
    for (int k = 0; k < 5000 && same; ++k)
    {
        float p[8];
        for (int i = 0; i < 8; ++i)
        {
            p[i] = (float)((int)Next(64) - 32) * 0.25f;
        }
        same = maniExactOrient(p, p + 2, p + 4) == OrientWide(p, p + 2, p + 4) &&
               maniExactInCircle(p, p + 2, p + 4, p + 6) == InCircleWide(p, p + 6);
        float lo[8];
        float hi[8];
        for (int i = 0; i < 8; ++i)
        {
            lo[i] = ldexpf(p[i], -140);
            hi[i] = ldexpf(p[i], 100);
        }
        int o = maniExactOrient(p, p + 2, p + 4);
        int c = maniExactInCircle(p, p + 2, p + 4, p + 6);
        same = same && maniExactOrient(lo, lo + 2, lo + 4) == o &&
               maniExactOrient(hi, hi + 2, hi + 4) == o &&
               maniExactInCircle(lo, lo + 2, lo + 4, lo + 6) == c &&
               maniExactInCircle(hi, hi + 2, hi + 4, hi + 6) == c;
    }
    CHECK(same, "random signs, at every scale");
}

static maniTriangulation s_mesh;

// The points on the hull's boundary: those with a line through them and
// another point with every point on its left or on it.
static uint32_t HullPoints(const float* p, uint32_t n)
{
    uint32_t h = 0;
    for (uint32_t a = 0; a < n; ++a)
    {
        bool on = false;
        for (uint32_t b = 0; b < n && !on; ++b)
        {
            bool all = b != a;
            for (uint32_t r = 0; r < n && all; ++r)
            {
                all = maniExactOrient(p + 2 * a, p + 2 * b, p + 2 * r) >= 0;
            }
            on = all;
        }
        h += on ? 1u : 0u;
    }
    return h;
}

// A triangulation of n points, h of them on the hull, checked.
static bool Delaunay(const float* p, uint32_t n, uint32_t h)
{
    uint32_t count = maniTriangulate(p, n, &s_mesh);
    bool line = true;
    for (uint32_t r = 2; r < n; ++r)
    {
        line = line && maniExactOrient(p, p + 2, p + 2 * r) == 0;
    }
    if (line)
    {
        return count == 0;
    }
    bool ok = count == 2 * n - 2 - h;
    bool used[256] = {false};
    for (uint32_t t = 0; t < count && ok; ++t)
    {
        const uint32_t* v = s_mesh.t[t].v;
        ok = maniExactOrient(p + 2 * v[0], p + 2 * v[1], p + 2 * v[2]) > 0;
        used[v[0]] = used[v[1]] = used[v[2]] = true;
        for (uint32_t r = 0; r < n && ok; ++r)
        {
            ok = maniExactInCircle(p + 2 * v[0], p + 2 * v[1], p + 2 * v[2], p + 2 * r) <= 0;
        }
        for (uint32_t i = 0; i < 3 && ok; ++i)
        {
            uint32_t o = s_mesh.t[t].n[i];
            bool joined = o == MANI_TRIANGULATE_MAX;
            for (uint32_t j = 0; j < 3 && !joined; ++j)
            {
                joined = o < count && s_mesh.t[o].v[(j + 1) % 3] == v[(i + 2) % 3] &&
                         s_mesh.t[o].v[(j + 2) % 3] == v[(i + 1) % 3] && s_mesh.t[o].n[j] == t;
            }
            ok = joined;
        }
    }
    for (uint32_t r = 0; r < n && ok; ++r)
    {
        ok = used[r];
    }
    return ok;
}

// A random point of a kind: a 5 by 5 grid, uniform, near a line, on two
// circles, or with wide exponents.
static void RandomPoint(uint32_t kind, float* x, float* y)
{
    if (kind == 0)
    {
        *x = (float)Next(5);
        *y = (float)Next(5);
    }
    else if (kind == 1)
    {
        *x = (float)Next(1u << 20) / (float)(1u << 19) - 1.0f;
        *y = (float)Next(1u << 20) / (float)(1u << 19) - 1.0f;
    }
    else if (kind == 2)
    {
        *x = (float)Next(9);
        *y = Next(3) == 0 ? (float)Next(9) : *x * 0.5f;
    }
    else if (kind == 3)
    {
        static const float kCircle[16][2] = {{1, 0},  {0.6f, 0.8f},   {0, 1},  {-0.6f, 0.8f},
                                             {-1, 0}, {-0.6f, -0.8f}, {0, -1}, {0.6f, -0.8f},
                                             {2, 0},  {1.2f, 1.6f},   {0, 2},  {-1.2f, 1.6f},
                                             {-2, 0}, {-1.2f, -1.6f}, {0, -2}, {1.2f, -1.6f}};
        uint32_t k = Next(16);
        *x = kCircle[k][0];
        *y = kCircle[k][1];
    }
    else
    {
        *x = ldexpf((float)((int)Next(7) - 3), (int)Next(200) - 140);
        *y = ldexpf((float)((int)Next(7) - 3), (int)Next(200) - 140);
    }
}

// The random triangulations' rounds, ten of each point kind; the dense
// test (`test_exact dense`) takes 250.
static uint32_t s_rounds = 50;

static void TestTriangulations(void)
{
    static float p[512];
    bool ok = true;
    for (uint32_t round = 0; round < s_rounds && ok; ++round)
    {
        uint32_t want = 3 + Next(22);
        uint32_t kind = round % 5;
        uint32_t n = 0;
        for (uint32_t draw = 0; draw < 200 && n < want; ++draw)
        {
            float x = 0;
            float y = 0;
            RandomPoint(kind, &x, &y);
            bool repeated = false;
            for (uint32_t j = 0; j < n && !repeated; ++j)
            {
                repeated = p[2 * j] == x && p[2 * j + 1] == y;
            }
            if (!repeated)
            {
                p[2 * n] = x;
                p[2 * n + 1] = y;
                ++n;
            }
        }
        ok = n < 3 || Delaunay(p, n, HullPoints(p, n));
        if (!ok)
        {
            printf("  round %u (kind %u, %u points)\n", round, kind, n);
        }
    }
    CHECK(ok, "random triangulations Delaunay");
    for (uint32_t i = 0; i < 64; ++i)
    {
        p[2 * i] = (float)(i % 8);
        p[2 * i + 1] = (float)(i / 8);
    }
    CHECK(Delaunay(p, 64, 28), "an 8 by 8 grid");
    for (uint32_t i = 0; i < 5; ++i)
    {
        p[2 * i] = (float)i;
        p[2 * i + 1] = (float)i * 2.0f;
    }
    CHECK(maniTriangulate(p, 5, &s_mesh) == 0, "points on one line");
}

int main(int argc, char** argv)
{
    if (argc == 2 && strcmp(argv[1], "dense") == 0)
    {
        s_rounds = 250;
    }
    TestPredicates();
    TestTriangulations();
    return s_failures == 0 ? 0 : 1;
}
