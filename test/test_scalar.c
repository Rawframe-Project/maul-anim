// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library's own sine, cosine, arc cosine and arc tangent against the
// C library's binary64 functions, rounded to binary32: within one unit in
// the last place over dense sweeps, exact where the value is exact, and
// defined for every input.

#include "scalar.h"
#include "test_harness.h"

#include "maul-anim/base.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// The hash of HashAll on the reference build; every platform must match.
#define PINNED_HASH       0xc78d5a708acb5a58ull
#define PINNED_DENSE_HASH 0xe3101e4aa62210f9ull

// The distance in units in the last place between two floats of the same
// sign; equal values, signed zeros included, are 0 apart.
static int64_t UlpDistance(float a, float b)
{
    if (a == b)
    {
        return 0;
    }
    int32_t ia = 0;
    int32_t ib = 0;
    memcpy(&ia, &a, sizeof ia);
    memcpy(&ib, &b, sizeof ib);
    // Map the sign-magnitude bits onto a line ordered like the floats.
    int64_t la = ia < 0 ? (int64_t)INT32_MIN - ia : ia;
    int64_t lb = ib < 0 ? (int64_t)INT32_MIN - ib : ib;
    return la > lb ? la - lb : lb - la;
}

static bool SameBits(float a, float b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

static void TestSinCosWithinOneUlpOverEightTurns(void)
{
    int64_t worstSin = 0;
    int64_t worstCos = 0;
    for (int32_t i = -400000; i <= 400000; ++i)
    {
        float x = (float)i * 0.000125f * 0.628318548f * 8.0f;
        float s = 0.0f;
        float c = 0.0f;
        maniSinCos(x, &s, &c);
        int64_t ds = UlpDistance(s, (float)sin((double)x));
        int64_t dc = UlpDistance(c, (float)cos((double)x));
        worstSin = ds > worstSin ? ds : worstSin;
        worstCos = dc > worstCos ? dc : worstCos;
        CHECK(SameBits(s, maniSin(x)) && SameBits(c, maniCos(x)), "the three entry points agree");
    }
    CHECK(worstSin <= 1, "sine within one ulp");
    CHECK(worstCos <= 1, "cosine within one ulp");
}

static void TestSinCosWithinOneUlpForLargeAngles(void)
{
    int64_t worst = 0;
    for (int32_t i = 0; i < 200000; ++i)
    {
        // Spread over [2^10, 2^20] by stepping the exponent and mantissa.
        float x = ldexpf(1.0f + (float)(i % 1000) / 1000.0f, 10 + i / 20000);
        float s = 0.0f;
        float c = 0.0f;
        maniSinCos(x, &s, &c);
        int64_t ds = UlpDistance(s, (float)sin((double)x));
        int64_t dc = UlpDistance(c, (float)cos((double)x));
        worst = ds > worst ? ds : worst;
        worst = dc > worst ? dc : worst;
    }
    CHECK(worst <= 1, "within one ulp up to 2^20");
}

static void TestSinCosExactValues(void)
{
    CHECK(SameBits(maniSin(0.0f), 0.0f), "sin 0 is +0");
    CHECK(SameBits(maniSin(-0.0f), -0.0f), "sin -0 is -0");
    CHECK(SameBits(maniCos(0.0f), 1.0f), "cos 0 is 1");
    CHECK(SameBits(maniCos(-0.0f), 1.0f), "cos -0 is 1");
}

static void TestSinCosStayFiniteAndBounded(void)
{
    const float inputs[] = {1e10f, -3e20f, FLT_MAX, -FLT_MAX, 1e-40f, 33554432.0f};
    for (size_t i = 0; i < sizeof inputs / sizeof inputs[0]; ++i)
    {
        float s = 0.0f;
        float c = 0.0f;
        maniSinCos(inputs[i], &s, &c);
        CHECK(s >= -1.0f && s <= 1.0f && c >= -1.0f && c <= 1.0f, "bounded for any finite input");
    }
    CHECK(SameBits(maniSin(1e-40f), 1e-40f), "a subnormal's sine is itself");
}

// A quiet NaN with a payload, and one with its sign set: platforms differ
// in what arithmetic does with them, so every NaN result is canonical.
static float NanWithBits(uint32_t bits)
{
    float value = 0.0f;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static void TestNonFiniteGivesTheCanonicalNan(void)
{
    const float inputs[] = {INFINITY, -INFINITY, NAN, NanWithBits(0x7fc01234u),
                            NanWithBits(0xffc00000u)};
    for (size_t i = 0; i < sizeof inputs / sizeof inputs[0]; ++i)
    {
        float s = 0.0f;
        float c = 0.0f;
        maniSinCos(inputs[i], &s, &c);
        CHECK(SameBits(s, NAN) && SameBits(c, NAN), "sine and cosine");
        if (isnan(inputs[i]))
        {
            CHECK(SameBits(maniAcos(inputs[i]), NAN), "arc cosine");
            CHECK(SameBits(maniAtan2(inputs[i], 1.0f), NAN), "arc tangent of NaN y");
            CHECK(SameBits(maniAtan2(-2.0f, inputs[i]), NAN), "arc tangent of NaN x");
        }
    }
}

static void TestAcosWithinOneUlp(void)
{
    int64_t worst = 0;
    for (int32_t i = -1000000; i <= 1000000; ++i)
    {
        float x = (float)i / 1000000.0f;
        worst = UlpDistance(maniAcos(x), (float)acos((double)x)) > worst
                    ? UlpDistance(maniAcos(x), (float)acos((double)x))
                    : worst;
    }
    // Near the ends the derivative grows without bound: every float there.
    for (float x = 0.999f; x <= 1.0f; x = nextafterf(x, 2.0f))
    {
        int64_t d = UlpDistance(maniAcos(x), (float)acos((double)x));
        int64_t e = UlpDistance(maniAcos(-x), (float)acos((double)-x));
        worst = d > worst ? d : worst;
        worst = e > worst ? e : worst;
    }
    CHECK(worst <= 1, "arc cosine within one ulp");
}

static void TestAcosExactValuesAndClamping(void)
{
    CHECK(SameBits(maniAcos(1.0f), 0.0f), "acos 1 is 0");
    CHECK(SameBits(maniAcos(-1.0f), (float)acos(-1.0)), "acos -1 is pi");
    CHECK(SameBits(maniAcos(0.0f), (float)acos(0.0)), "acos 0 is half pi");
    CHECK(SameBits(maniAcos(1.0000001f), 0.0f), "above 1 clamps to 1");
    CHECK(SameBits(maniAcos(-5.0f), (float)acos(-1.0)), "below -1 clamps to -1");
}

static void TestAtan2WithinOneUlpOnCircles(void)
{
    int64_t worst = 0;
    const float radii[] = {1e-30f, 1e-3f, 1.0f, 7.5f, 1e5f, 1e30f};
    for (size_t r = 0; r < sizeof radii / sizeof radii[0]; ++r)
    {
        for (int32_t i = 0; i < 100000; ++i)
        {
            double angle = -3.1415926 + 6.2831852 * (double)i / 100000.0;
            float y = (float)((double)radii[r] * sin(angle));
            float x = (float)((double)radii[r] * cos(angle));
            int64_t d = UlpDistance(maniAtan2(y, x), (float)atan2((double)y, (double)x));
            worst = d > worst ? d : worst;
        }
    }
    CHECK(worst <= 1, "arc tangent within one ulp");
}

static void TestAtan2Quadrants(void)
{
    const float values[] = {0.0f, -0.0f, 1e-38f, -2.5f, 3.0f, INFINITY, -INFINITY};
    const size_t count = sizeof values / sizeof values[0];
    for (size_t i = 0; i < count; ++i)
    {
        for (size_t j = 0; j < count; ++j)
        {
            float y = values[i];
            float x = values[j];
            CHECK(SameBits(maniAtan2(y, x), (float)atan2((double)y, (double)x)),
                  "zeros, signs and infinities as C defines them");
        }
    }
}

// Results are hashed with the family hash in blocks, as every platform
// must produce the same bits.
typedef struct Hasher
{
    uint64_t hash;
    int32_t count;
    float block[1024];
} Hasher;

static void HashFloat(Hasher* h, float value)
{
    h->block[h->count++] = value;
    if (h->count == 1024)
    {
        h->hash = maniHash64(h->hash, h->block, (int32_t)sizeof h->block);
        h->count = 0;
    }
}

static uint64_t HashEnd(Hasher* h)
{
    return maniHash64(h->hash, h->block, h->count * (int32_t)sizeof(float));
}

static uint64_t HashAll(void)
{
    Hasher h = {MANI_HASH_INIT, 0, {0}};
    for (int32_t i = -50000; i <= 50000; ++i)
    {
        float x = (float)i * 0.0137f;
        float s = 0.0f;
        float c = 0.0f;
        maniSinCos(x, &s, &c);
        HashFloat(&h, s);
        HashFloat(&h, c);
        HashFloat(&h, maniAcos((float)i / 50000.0f));
        HashFloat(&h, maniAtan2(x, (float)(i % 97) - 48.0f));
    }
    return HashEnd(&h);
}

// Every one of 2^24 angles over eight turns and 2^24 tangents over
// [0, 1]: dense enough that the last term of each series changes some
// results.
static uint64_t HashDense(void)
{
    Hasher h = {MANI_HASH_INIT, 0, {0}};
    for (int32_t i = -(1 << 23); i <= (1 << 23); ++i)
    {
        float x = (float)((double)i / 8388608.0 * 25.132741228718345);
        float s = 0.0f;
        float c = 0.0f;
        maniSinCos(x, &s, &c);
        HashFloat(&h, s);
        HashFloat(&h, c);
    }
    for (int32_t i = 0; i <= (1 << 24); ++i)
    {
        HashFloat(&h, maniAtan2((float)i, 16777216.0f));
    }
    return HashEnd(&h);
}

static void TestSameBitsEverywhere(void)
{
    uint64_t hash = HashAll();
    if (hash != PINNED_HASH)
    {
        printf("hash %016llx\n", (unsigned long long)hash);
    }
    CHECK(hash == PINNED_HASH, "the pinned results");
    uint64_t dense = HashDense();
    if (dense != PINNED_DENSE_HASH)
    {
        printf("dense hash %016llx\n", (unsigned long long)dense);
    }
    CHECK(dense == PINNED_DENSE_HASH, "the pinned dense results");
}

int main(void)
{
    TestSinCosWithinOneUlpOverEightTurns();
    TestSinCosWithinOneUlpForLargeAngles();
    TestSinCosExactValues();
    TestSinCosStayFiniteAndBounded();
    TestNonFiniteGivesTheCanonicalNan();
    TestAcosWithinOneUlp();
    TestAcosExactValuesAndClamping();
    TestAtan2WithinOneUlpOnCircles();
    TestAtan2Quadrants();
    TestSameBitsEverywhere();
    return s_failures == 0 ? 0 : 1;
}
