// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Splitting matrices into translation, rotation and scale: recomposition
// gives the matrix back, mirrors included; without shear the rotation and
// scale are the ones composed; with shear the rotation is the polar
// factor (its transpose times the matrix is symmetric); degenerate
// matrices still split into finite values; results pinned. The rigid
// split: without shear, mirrors included, maniDecompose's
// rotation within rounding; with shear, a small one in each pair of
// columns alone among them, its bits; the identity for a matrix not
// finite.

#include "decompose.h"
#include "test_harness.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"

#include <math.h>
#include <string.h>

#define PINNED_DECOMPOSE_HASH 0xdec3ebe3703143dcull

static uint64_t s_random = 0x2545F4914F6CDD1Dull;

static uint32_t Next(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (uint32_t)(s_random >> 32);
}

static float Uniform(float low, float high)
{
    return low + (high - low) * (float)(Next() >> 8) * 0x1p-24f;
}

static maniQuat RandomRotation(void)
{
    maniQuat q = {Uniform(-1, 1), Uniform(-1, 1), Uniform(-1, 1), Uniform(-1, 1)};
    return maniQuatNormalize(q);
}

static maniAffine Compose(const maniTransform* t)
{
    return maniAffineFromTrs(t->translation, t->rotation, t->scale);
}

// The largest difference relative to the matrix's largest component.
static double Difference(const maniAffine* a, const maniAffine* b)
{
    double magnitude = 1e-30;
    double worst = 0.0;
    for (int k = 0; k < 9; ++k)
    {
        magnitude = fmax(magnitude, fabs((double)a->m[k]));
    }
    for (int k = 0; k < 12; ++k)
    {
        double scaleOf = k < 9 ? magnitude : fmax(1.0, fabs((double)a->m[k]));
        worst = fmax(worst, fabs((double)a->m[k] - (double)b->m[k]) / scaleOf);
    }
    return worst;
}

static double QuaternionLengthError(maniQuat q)
{
    double x = (double)q.x;
    double y = (double)q.y;
    double z = (double)q.z;
    double w = (double)q.w;
    return fabs(sqrt(x * x + y * y + z * z + w * w) - 1.0);
}

static void TestRecomposesWithoutShear(void)
{
    double worst = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        maniTransform t = {{Uniform(-5, 5), Uniform(-5, 5), Uniform(-5, 5)},
                           RandomRotation(),
                           {Uniform(0.2f, 3), Uniform(0.2f, 3), Uniform(0.2f, 3)}};
        // A mirror on a random axis for a third of them.
        uint32_t mirror = Next() % 9;
        if (mirror < 3)
        {
            float* s = mirror == 0 ? &t.scale.x : (mirror == 1 ? &t.scale.y : &t.scale.z);
            *s = -*s;
        }
        maniAffine m = Compose(&t);
        maniTransform d;
        CHECK(maniDecomposeAffine(&m, &d) == mani_success, "splits");
        CHECK(d.rotation.w >= 0.0f, "w not negative");
        CHECK(QuaternionLengthError(d.rotation) < 1e-6, "unit rotation");
        maniAffine back = Compose(&d);
        worst = fmax(worst, Difference(&m, &back));
        if (mirror >= 3)
        {
            // No mirror: the composed rotation (either sign) and scale.
            double dot = (double)t.rotation.x * (double)d.rotation.x +
                         (double)t.rotation.y * (double)d.rotation.y +
                         (double)t.rotation.z * (double)d.rotation.z +
                         (double)t.rotation.w * (double)d.rotation.w;
            CHECK(fabs(fabs(dot) - 1.0) < 1e-6, "the composed rotation");
            CHECK(fabs((double)d.scale.x - (double)t.scale.x) < 1e-5 * (double)t.scale.x &&
                      fabs((double)d.scale.y - (double)t.scale.y) < 1e-5 * (double)t.scale.y &&
                      fabs((double)d.scale.z - (double)t.scale.z) < 1e-5 * (double)t.scale.z,
                  "the composed scale");
        }
        else
        {
            CHECK(d.scale.x < 0.0f && d.scale.y > 0.0f && d.scale.z > 0.0f, "a mirror on x");
        }
    }
    CHECK(worst < 4e-6, "recomposes within float rounding");
}

// With shear, R^T A is the polar decomposition's symmetric factor.
static void TestShearGivesThePolarRotation(void)
{
    for (int i = 0; i < 500; ++i)
    {
        maniTransform parent = {
            {0, 0, 0}, RandomRotation(), {Uniform(0.3f, 3), Uniform(0.3f, 3), Uniform(0.3f, 3)}};
        maniTransform child = {{Uniform(-1, 1), 0, 0}, RandomRotation(), {1, 1, 1}};
        maniAffine p = Compose(&parent);
        maniAffine c = Compose(&child);
        maniAffine m = maniAffineMul(&p, &c);
        maniTransform d;
        CHECK(maniDecomposeAffine(&m, &d) == mani_success, "splits");
        maniAffine r = maniAffineFromTrs((maniVec3){0, 0, 0}, d.rotation, (maniVec3){1, 1, 1});
        // S = R^T A, column-major: S[c][k] = dot(R column k, A column c).
        double s[3][3];
        double magnitude = 0.0;
        for (int col = 0; col < 3; ++col)
        {
            for (int k = 0; k < 3; ++k)
            {
                s[col][k] = 0.0;
                for (int j = 0; j < 3; ++j)
                {
                    s[col][k] += (double)r.m[3 * k + j] * (double)m.m[3 * col + j];
                }
                magnitude = fmax(magnitude, fabs(s[col][k]));
            }
        }
        double asymmetry =
            fmax(fabs(s[0][1] - s[1][0]), fmax(fabs(s[0][2] - s[2][0]), fabs(s[1][2] - s[2][1])));
        CHECK(asymmetry < 1e-5 * magnitude, "the symmetric factor");
        CHECK(s[0][0] > 0.0 && s[1][1] > 0.0 && s[2][2] > 0.0, "positive along its axes");
    }
}

static bool AllFinite(const maniTransform* t)
{
    const float v[10] = {t->translation.x, t->translation.y, t->translation.z, t->rotation.x,
                         t->rotation.y,    t->rotation.z,    t->rotation.w,    t->scale.x,
                         t->scale.y,       t->scale.z};
    for (int k = 0; k < 10; ++k)
    {
        if (!isfinite(v[k]))
        {
            return false;
        }
    }
    return true;
}

static void TestDegenerateMatrices(void)
{
    const maniAffine cases[] = {
        // All zero.
        {{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3}},
        // A zero x axis, then y, then z.
        {{0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0}},
        {{2, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0}},
        {{2, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0}},
        // Two zero axes.
        {{0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0}},
        {{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
        // Parallel axes.
        {{1, 0, 0, 2, 0, 0, 0, 0, 1, 0, 0, 0}},
        {{1, 1, 0, 2, 2, 0, 3, 3, 0, 0, 0, 0}},
        // A tiny but regular matrix is not degenerate.
        {{1e-12f, 0, 0, 0, 1e-12f, 0, 0, 0, 1e-12f, 0, 0, 0}},
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; ++i)
    {
        maniTransform d;
        CHECK(maniDecomposeAffine(&cases[i], &d) == mani_success, "splits");
        CHECK(AllFinite(&d), "finite");
        CHECK(QuaternionLengthError(d.rotation) < 1e-6, "unit rotation");
    }
    maniTransform d;
    CHECK(maniDecomposeAffine(&cases[1], &d) == mani_success && d.scale.x == 0.0f &&
              d.scale.y == 2.0f && d.scale.z == 3.0f,
          "column lengths");
    CHECK(maniDecomposeAffine(&cases[8], &d) == mani_success && fabsf(d.scale.x - 1e-12f) < 1e-18f,
          "tiny scale kept");
}

// Half turns, where Shepperd's method must not divide by 1 + trace.
static void TestHalfTurns(void)
{
    const maniQuat turns[3] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    for (int i = 0; i < 3; ++i)
    {
        maniAffine m = maniAffineFromTrs((maniVec3){0, 0, 0}, turns[i], (maniVec3){1, 2, 3});
        maniTransform d;
        CHECK(maniDecomposeAffine(&m, &d) == mani_success && AllFinite(&d), "a half turn splits");
        double dot = (double)turns[i].x * (double)d.rotation.x +
                     (double)turns[i].y * (double)d.rotation.y +
                     (double)turns[i].z * (double)d.rotation.z;
        CHECK(fabs(fabs(dot) - 1.0) < 1e-6, "the half turn back");
    }
}

// A tiny axis counts as zero: here x, parallel to y, is rebuilt from y
// and z.
static void TestTinyAxisIsZero(void)
{
    maniAffine m = {{0, 1e-35f, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    maniTransform d;
    CHECK(maniDecomposeAffine(&m, &d) == mani_success, "splits");
    CHECK(d.rotation.w == 1.0f && d.rotation.x == 0.0f && d.rotation.y == 0.0f &&
              d.rotation.z == 0.0f,
          "the identity rotation");
}

static void TestRefusals(void)
{
    maniAffine m = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    maniTransform d;
    CHECK(maniDecomposeAffine(NULL, &d) == mani_errorInvalid, "null matrix");
    CHECK(maniDecomposeAffine(&m, NULL) == mani_errorInvalid, "null out");
    m.m[10] = INFINITY;
    CHECK(maniDecomposeAffine(&m, &d) == mani_errorInvalid, "infinite translation");
    m.m[10] = 0;
    m.m[4] = NAN;
    CHECK(maniDecomposeAffine(&m, &d) == mani_errorInvalid, "NaN");
}

static void TestPinned(void)
{
    s_random = 0x9E3779B97F4A7C15ull;
    uint64_t hash = MANI_HASH_INIT;
    for (int i = 0; i < 500; ++i)
    {
        maniTransform parent = {
            {0, 0, 0}, RandomRotation(), {Uniform(0.3f, 3), Uniform(-3, -0.3f), Uniform(0.3f, 3)}};
        maniTransform child = {{Uniform(-1, 1), Uniform(-1, 1), 0}, RandomRotation(), {1, 1, 1}};
        maniAffine p = Compose(&parent);
        maniAffine c = Compose(&child);
        maniAffine m = maniAffineMul(&p, &c);
        maniTransform d;
        CHECK(maniDecomposeAffine(&m, &d) == mani_success, "splits");
        hash = maniHash64(hash, &d, (int32_t)sizeof d);
    }
    CHECK(hash == PINNED_DECOMPOSE_HASH, "pinned");
    if (hash != PINNED_DECOMPOSE_HASH)
    {
        printf("decompose hash 0x%016llx\n", (unsigned long long)hash);
    }
}

static bool SameTurn(maniQuat a, maniQuat b, double tolerance)
{
    double dot = (double)a.x * (double)b.x + (double)a.y * (double)b.y + (double)a.z * (double)b.z +
                 (double)a.w * (double)b.w;
    return fabs(fabs(dot) - 1.0) <= tolerance;
}

static void TestRigid(void)
{
    bool near = true;
    for (int i = 0; i < 500; ++i)
    {
        float mirror = (i % 5) == 0 ? -1.0f : 1.0f;
        maniTransform t = {{Uniform(-5, 5), Uniform(-5, 5), Uniform(-5, 5)},
                           RandomRotation(),
                           {mirror * Uniform(0.3f, 3), Uniform(0.3f, 3), Uniform(0.3f, 3)}};
        maniAffine m = Compose(&t);
        maniTransform d = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
        maniVec3 translation;
        maniQuat rotation;
        maniDecomposeRigid(&m, &translation, &rotation);
        near = near && maniDecompose(&m, &d) && SameTurn(rotation, d.rotation, 1e-6) &&
               memcmp(&translation, &d.translation, sizeof translation) == 0;
    }
    CHECK(near, "without shear, mirrors included: maniDecompose's rotation within rounding");
    bool same = true;
    for (int i = 0; i < 200; ++i)
    {
        maniTransform parent = {
            {0, 0, 0}, RandomRotation(), {Uniform(0.3f, 3), Uniform(0.3f, 3), Uniform(0.3f, 3)}};
        maniTransform child = {{Uniform(-1, 1), 0, 0}, RandomRotation(), {1, 1, 1}};
        maniAffine p = Compose(&parent);
        maniAffine c = Compose(&child);
        maniAffine m = maniAffineMul(&p, &c);
        maniTransform d = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
        maniVec3 translation;
        maniQuat rotation;
        maniDecomposeRigid(&m, &translation, &rotation);
        same =
            same && maniDecompose(&m, &d) && memcmp(&rotation, &d.rotation, sizeof rotation) == 0;
    }
    CHECK(same, "with shear: maniDecompose's rotation, its bits");
    // A small shear in one pair of columns only, each pair: still the
    // iteration's bits (the fast path takes orthogonal columns alone).
    bool pairs = true;
    for (int pair = 0; pair < 3; ++pair)
    {
        for (int step = 1; step <= 9; ++step)
        {
            maniAffine m = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
            int from = pair == 2 ? 1 : 0;
            int to = pair == 0 ? 1 : 2;
            m.m[3 * to + from] = 0.01f * (float)step;
            maniTransform d = {{0, 0, 0}, {0, 0, 0, 1}, {1, 1, 1}};
            maniVec3 translation;
            maniQuat rotation;
            maniDecomposeRigid(&m, &translation, &rotation);
            pairs = pairs && maniDecompose(&m, &d) &&
                    memcmp(&rotation, &d.rotation, sizeof rotation) == 0;
        }
    }
    CHECK(pairs, "a small shear in each pair of columns: maniDecompose's bits");
    maniAffine bad = {{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    bad.m[4] = NAN;
    maniVec3 translation;
    maniQuat rotation;
    maniDecomposeRigid(&bad, &translation, &rotation);
    CHECK(rotation.x == 0.0f && rotation.y == 0.0f && rotation.z == 0.0f && rotation.w == 1.0f,
          "not finite: the identity");
}

int main(void)
{
    TestRecomposesWithoutShear();
    TestShearGivesThePolarRotation();
    TestDegenerateMatrices();
    TestHalfTurns();
    TestTinyAxisIsZero();
    TestRefusals();
    TestPinned();
    TestRigid();
    return s_failures == 0 ? 0 : 1;
}
