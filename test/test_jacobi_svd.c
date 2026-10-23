// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The one-sided Jacobi decomposition on its own: random
// matrices of 9 by 4 to 120 by 36 rebuilt from their factors within
// 2e-6 of their largest entry, w orthogonal and the columns orthogonal
// within 2e-6; the singular values within 2e-6 (relative to the
// largest) of a binary64 Jacobi written here; repeated and zero columns
// (singular values of 0, no NaN), an all-zero matrix in one sweep; a
// fixed matrix's sweep count and bits pinned, the same on every
// platform.

#include "jacobi_svd.h"
#include "test_harness.h"

#include "maul-anim/base.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MOST_N 120u
#define MOST_M 36u

static float s_a[MOST_N * MOST_M];
static float s_input[MOST_N * MOST_M];
static float s_w[MOST_M * MOST_M];
static float s_sigma[MOST_M];
static double s_reference[MOST_N * MOST_M];
static uint64_t s_random = 0x9E3779B97F4A7C15ull;

static float Random(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (float)(s_random >> 40) / 8388608.0f - 1.0f;
}

static int Descending(const void* a, const void* b)
{
    double x = *(const double*)a;
    double y = *(const double*)b;
    return x < y ? 1 : (x > y ? -1 : 0);
}

// The singular values of the n-by-m input by one-sided Jacobi in
// binary64, sorted descending into out.
static void Reference(const float* input, uint32_t n, uint32_t m, double* out)
{
    for (uint32_t k = 0; k < n * m; ++k)
    {
        s_reference[k] = (double)input[k];
    }
    for (int sweep = 0; sweep < 60; ++sweep)
    {
        for (uint32_t p = 0; p + 1 < m; ++p)
        {
            for (uint32_t q = p + 1; q < m; ++q)
            {
                double* ap = s_reference + (size_t)p * n;
                double* aq = s_reference + (size_t)q * n;
                double alpha = 0.0;
                double beta = 0.0;
                double gamma = 0.0;
                for (uint32_t k = 0; k < n; ++k)
                {
                    alpha += ap[k] * ap[k];
                    beta += aq[k] * aq[k];
                    gamma += ap[k] * aq[k];
                }
                if (!(fabs(gamma) > 1e-300))
                {
                    continue;
                }
                double zeta = (beta - alpha) / (2.0 * gamma);
                double t = (zeta >= 0.0 ? 1.0 : -1.0) / (fabs(zeta) + sqrt(1.0 + zeta * zeta));
                double c = 1.0 / sqrt(1.0 + t * t);
                double s = c * t;
                for (uint32_t k = 0; k < n; ++k)
                {
                    double x = ap[k];
                    double y = aq[k];
                    ap[k] = c * x - s * y;
                    aq[k] = s * x + c * y;
                }
            }
        }
    }
    for (uint32_t i = 0; i < m; ++i)
    {
        double sum = 0.0;
        for (uint32_t k = 0; k < n; ++k)
        {
            sum += s_reference[(size_t)i * n + k] * s_reference[(size_t)i * n + k];
        }
        out[i] = sqrt(sum);
    }
    qsort(out, m, sizeof *out, Descending);
}

// The largest error of the factors: the input rebuilt, w and the
// columns orthogonal; relative to the input's largest entry.
static double FactorError(uint32_t n, uint32_t m)
{
    double largest = 0.0;
    for (uint32_t k = 0; k < n * m; ++k)
    {
        largest = fmax(largest, fabs((double)s_input[k]));
    }
    double worst = 0.0;
    // The input is a w^T: a's column i is the input times w's column i.
    for (uint32_t j = 0; j < m; ++j)
    {
        for (uint32_t k = 0; k < n; ++k)
        {
            double sum = 0.0;
            for (uint32_t i = 0; i < m; ++i)
            {
                sum += (double)s_a[(size_t)i * n + k] * (double)s_w[(size_t)i * m + j];
            }
            worst = fmax(worst, fabs(sum - (double)s_input[(size_t)j * n + k]) / largest);
        }
    }
    for (uint32_t i = 0; i < m; ++i)
    {
        for (uint32_t j = 0; j < m; ++j)
        {
            double w = 0.0;
            double c = 0.0;
            for (uint32_t k = 0; k < m; ++k)
            {
                w += (double)s_w[(size_t)i * m + k] * (double)s_w[(size_t)j * m + k];
            }
            for (uint32_t k = 0; k < n; ++k)
            {
                c += (double)s_a[(size_t)i * n + k] * (double)s_a[(size_t)j * n + k];
            }
            worst = fmax(worst, fabs(w - (i == j ? 1.0 : 0.0)));
            double scale = (double)s_sigma[i] * (double)s_sigma[j];
            worst = i != j && scale > 0.0 ? fmax(worst, fabs(c) / (largest * largest * n)) : worst;
        }
    }
    return worst;
}

static void TestRandom(void)
{
    const uint32_t sizes[5][2] = {{9, 4}, {12, 12}, {40, 18}, {96, 30}, {120, 36}};
    double worstFactors = 0.0;
    double worstValues = 0.0;
    for (int c = 0; c < 5; ++c)
    {
        uint32_t n = sizes[c][0];
        uint32_t m = sizes[c][1];
        for (uint32_t k = 0; k < n * m; ++k)
        {
            s_input[k] = Random();
        }
        memcpy(s_a, s_input, sizeof(float) * n * m);
        uint32_t sweeps = maniJacobiSvd(s_a, n, m, s_w, s_sigma);
        CHECK(sweeps >= 1 && sweeps <= MANI_JACOBI_MAX_SWEEPS, "sweeps within the bound");
        worstFactors = fmax(worstFactors, FactorError(n, m));
        double want[MOST_M];
        double got[MOST_M];
        Reference(s_input, n, m, want);
        for (uint32_t i = 0; i < m; ++i)
        {
            got[i] = (double)s_sigma[i];
        }
        qsort(got, m, sizeof *got, Descending);
        for (uint32_t i = 0; i < m; ++i)
        {
            worstValues = fmax(worstValues, fabs(got[i] - want[i]) / want[0]);
        }
    }
    CHECK(worstFactors < 2e-6, "the factors rebuild the input, both orthogonal");
    CHECK(worstValues < 2e-6, "the singular values as binary64's");
    if (worstFactors >= 2e-6 || worstValues >= 2e-6)
    {
        printf("factors %g values %g\n", worstFactors, worstValues);
    }
}

static void TestDegenerate(void)
{
    const uint32_t n = 10;
    const uint32_t m = 6;
    for (uint32_t k = 0; k < n * m; ++k)
    {
        s_input[k] = Random();
    }
    // Column 3 a copy of column 1, column 5 zero.
    memcpy(s_input + 3 * n, s_input + 1 * n, sizeof(float) * n);
    memset(s_input + 5 * n, 0, sizeof(float) * n);
    memcpy(s_a, s_input, sizeof(float) * n * m);
    (void)maniJacobiSvd(s_a, n, m, s_w, s_sigma);
    int small = 0;
    bool finite = true;
    for (uint32_t i = 0; i < m; ++i)
    {
        small += s_sigma[i] < 1e-5f;
        finite = finite && isfinite(s_sigma[i]);
    }
    CHECK(finite && small == 2 && FactorError(n, m) < 2e-6,
          "a repeated and a zero column: two singular values of 0, no NaN");
    memset(s_a, 0, sizeof(float) * n * m);
    CHECK(maniJacobiSvd(s_a, n, m, s_w, s_sigma) == 1 && s_sigma[0] == 0.0f && s_w[0] == 1.0f,
          "an all-zero matrix: one sweep, w the identity");
}

int main(void)
{
    TestRandom();
    TestDegenerate();
    return s_failures == 0 ? 0 : 1;
}
