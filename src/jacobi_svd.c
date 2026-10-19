// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The singular value decomposition by one-sided Jacobi (Hestenes;
// mani-0031), for the full-body SDLS step: the columns of an n-by-m
// matrix A (stored column by column) are rotated in pairs until every
// pair is orthogonal; then A W = B with W orthogonal, the columns' lengths
// are A's singular values, B's columns over them A's left singular
// vectors and W's columns its right ones. Pairs whose cosine is below
// TOL = sqrt(n) eps are left alone and the sweeps stop when one rotates
// nothing, at 30 at most, as LAPACK's sgesvj: the rotation from
// Rutishauser's tangent. Dot products accumulate in binary64, in a
// fixed order; every operation is an addition, a multiplication, a
// division or a square root, each correctly rounded, so the stop falls
// on the same sweep on every platform. Each column's squared length is
// kept and taken again only when a rotation changes the column, so it
// is always what a fresh dot product would give; a pair with a zero
// column (length 0, so its product 0 too) is skipped without one.

#include "jacobi_svd.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static double Dot(const float* a, const float* b, uint32_t n)
{
    double sum = 0.0;
    for (uint32_t k = 0; k < n; ++k)
    {
        sum += (double)a[k] * (double)b[k];
    }
    return sum;
}

// Columns p and q of a (n long) and of w (m long) rotated by (c, s).
static void Rotate(float* a, uint32_t n, float* w, uint32_t m, uint32_t p, uint32_t q, double c,
                   double s)
{
    float* ap = a + (size_t)p * n;
    float* aq = a + (size_t)q * n;
    for (uint32_t k = 0; k < n; ++k)
    {
        double x = (double)ap[k];
        double y = (double)aq[k];
        ap[k] = (float)(c * x - s * y);
        aq[k] = (float)(s * x + c * y);
    }
    float* wp = w + (size_t)p * m;
    float* wq = w + (size_t)q * m;
    for (uint32_t k = 0; k < m; ++k)
    {
        double x = (double)wp[k];
        double y = (double)wq[k];
        wp[k] = (float)(c * x - s * y);
        wq[k] = (float)(s * x + c * y);
    }
}

uint32_t maniJacobiSvd(float* a, uint32_t n, uint32_t m, float* w, float* sigma)
{
    for (uint32_t i = 0; i < m * m; ++i)
    {
        w[i] = 0.0f;
    }
    for (uint32_t i = 0; i < m; ++i)
    {
        w[(size_t)i * m + i] = 1.0f;
    }
    double norms[MANI_JACOBI_MAX_COLUMNS];
    for (uint32_t i = 0; i < m; ++i)
    {
        norms[i] = Dot(a + (size_t)i * n, a + (size_t)i * n, n);
    }
    double tol = sqrt((double)n) * (double)MANI_JACOBI_EPSILON;
    uint32_t sweeps = 0;
    bool rotated = true;
    while (rotated && sweeps < MANI_JACOBI_MAX_SWEEPS)
    {
        rotated = false;
        ++sweeps;
        for (uint32_t p = 0; p + 1 < m; ++p)
        {
            for (uint32_t q = p + 1; q < m; ++q)
            {
                double alpha = norms[p];
                double beta = norms[q];
                if (alpha == 0.0 || beta == 0.0)
                {
                    continue;
                }
                const float* ap = a + (size_t)p * n;
                const float* aq = a + (size_t)q * n;
                double gamma = Dot(ap, aq, n);
                if (!(fabs(gamma) > tol * sqrt(alpha) * sqrt(beta)))
                {
                    continue;
                }
                double zeta = (beta - alpha) / (2.0 * gamma);
                double t = (zeta >= 0.0 ? 1.0 : -1.0) / (fabs(zeta) + sqrt(1.0 + zeta * zeta));
                double c = 1.0 / sqrt(1.0 + t * t);
                Rotate(a, n, w, m, p, q, c, c * t);
                norms[p] = Dot(ap, ap, n);
                norms[q] = Dot(aq, aq, n);
                rotated = true;
            }
        }
    }
    for (uint32_t i = 0; i < m; ++i)
    {
        sigma[i] = (float)sqrt(norms[i]);
    }
    return sweeps;
}
