// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The importers' basis change (mani-0009, mani-0014): a handedness flip
// and an axis swap with a unit factor, on transforms, vectors,
// quaternions and matrices, checked against B G B^-1 computed directly.

#include "basis.h"
#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/import.h"

#include <math.h>
#include <string.h>

static bool Near(const float* a, const float* b, int n, float tolerance)
{
    for (int i = 0; i < n; ++i)
    {
        if (!(fabsf(a[i] - b[i]) <= tolerance))
        {
            return false;
        }
    }
    return true;
}

// B as a matrix (output row i is input axis p[i] times s[i]), scaled by
// the unit for translations: B G B^-1 composed directly.
static maniAffine Conjugate(const maniBasis* b, const maniAffine* g)
{
    float m[3][3] = {{0}};
    for (int i = 0; i < 3; ++i)
    {
        m[i][b->p[i]] = b->s[i];
    }
    maniAffine out;
    for (int c = 0; c < 3; ++c)
    {
        for (int r = 0; r < 3; ++r)
        {
            // (B G B^T)[r][c] = sum B[r][k] G[k][l] B[c][l].
            float v = 0;
            for (int k = 0; k < 3; ++k)
            {
                for (int l = 0; l < 3; ++l)
                {
                    v += m[r][k] * g->m[3 * l + k] * m[c][l];
                }
            }
            out.m[3 * c + r] = v;
        }
        float t = 0;
        for (int k = 0; k < 3; ++k)
        {
            t += m[c][k] * g->m[9 + k];
        }
        out.m[9 + c] = b->unit * t;
    }
    return out;
}

static void TestAxes(void)
{
    maniAxis good[3] = {maniAxisX, maniAxisNegativeZ, maniAxisY};
    maniAxis twice[3] = {maniAxisX, maniAxisNegativeX, maniAxisY};
    maniAxis past[3] = {maniAxisX, maniAxisY, 6};
    CHECK(maniBasisAxesValid(good) && !maniBasisAxesValid(twice) && !maniBasisAxesValid(past),
          "valid axes");
    maniBasis b = maniBasisMake(good, 1.0f);
    CHECK(b.det == 1.0f, "Y up to Z up keeps handedness");
    maniAxis mirror[3] = {maniAxisNegativeX, maniAxisY, maniAxisZ};
    b = maniBasisMake(mirror, 1.0f);
    CHECK(b.det == -1.0f, "a mirror flips it");
    maniAxis odd[3] = {maniAxisY, maniAxisX, maniAxisZ};
    b = maniBasisMake(odd, 1.0f);
    CHECK(b.det == -1.0f, "so does a swap");
}

static void TestUnit(void)
{
    maniAxis axes[3] = {maniAxisZ, maniAxisNegativeX, maniAxisY};
    maniBasis b = maniBasisMake(axes, 0.01f);
    maniTransform t = {{1, 2, 3}, {0.1f, 0.2f, 0.3f, 0.927361849549570f}, {4, 5, 6}};
    maniAffine g = maniAffineFromTrs(t.translation, t.rotation, t.scale);
    maniAffine expected = Conjugate(&b, &g);
    maniAffine got = maniBasisAffine(&b, g);
    CHECK(Near(got.m, expected.m, 12, 1e-6f), "a matrix: B G B^-1, translation in the unit");
    maniTransform u = maniBasisTransform(&b, t);
    maniAffine fromTrs = maniAffineFromTrs(u.translation, u.rotation, u.scale);
    CHECK(Near(fromTrs.m, expected.m, 12, 1e-5f), "a transform the same");
    float v[3] = {1, 2, 3};
    maniBasisVector(&b, v, true);
    float translated[3] = {0.03f, -0.01f, 0.02f};
    CHECK(Near(v, translated, 3, 1e-9f), "a translation permuted, signed and scaled");
    float s[3] = {4, 5, 6};
    maniBasisVector(&b, s, false);
    float scaled[3] = {6, 4, 5};
    CHECK(Near(s, scaled, 3, 0), "a scale only permuted");
    float q[4] = {0.1f, 0.2f, 0.3f, 0.9f};
    maniBasisQuat(&b, q);
    float rotated[4] = {b.det * 0.3f, b.det * -0.1f, b.det * 0.2f, 0.9f};
    CHECK(Near(q, rotated, 4, 0), "a quaternion's vector part times det B");
    maniAffine identity = maniIdentityAffine();
    float one[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
    CHECK(Near(identity.m, one, 12, 0), "the identity");
}

int main(void)
{
    TestAxes();
    TestUnit();
    return s_failures == 0 ? 0 : 1;
}
