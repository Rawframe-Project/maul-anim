// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The blend's eight-lane steps against the scalar lane loops they
// replaced: the same groups through both, every output's
// bits compared (NaN for NaN). It runs on every backend a build chooses: AVX2,
// NEON and the scalar fallback.

#include "blend_lanes.h"
#include "pose_memory.h"
#include "test_harness.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define LANES      MANI_POSE_GROUP_BONES
#define COMPONENTS MANI_POSE_COMPONENTS
#define FLOATS     (LANES * COMPONENTS)

// The scalar loops, as blend.c had them before the kernels.
static void AccumulateScalar(float* o, const float* in, const float* w, bool first)
{
    if (first)
    {
        for (uint32_t c = 0; c < COMPONENTS; ++c)
        {
            for (uint32_t k = 0; k < LANES; ++k)
            {
                o[c * LANES + k] = 0.0f + in[c * LANES + k] * w[k];
            }
        }
        return;
    }
    float rw[LANES];
    for (uint32_t k = 0; k < LANES; ++k)
    {
        float dot = ((o[3 * LANES + k] * in[3 * LANES + k] + o[4 * LANES + k] * in[4 * LANES + k]) +
                     o[5 * LANES + k] * in[5 * LANES + k]) +
                    o[6 * LANES + k] * in[6 * LANES + k];
        rw[k] = dot < 0.0f ? -w[k] : w[k];
    }
    for (uint32_t c = 0; c < COMPONENTS; ++c)
    {
        const float* weights = c >= 3 && c < 7 ? rw : w;
        for (uint32_t k = 0; k < LANES; ++k)
        {
            o[c * LANES + k] += in[c * LANES + k] * weights[k];
        }
    }
}

static void NormalizeScalar(float* o)
{
    float root[LANES];
    for (uint32_t k = 0; k < LANES; ++k)
    {
        float x = o[3 * LANES + k];
        float y = o[4 * LANES + k];
        float z = o[5 * LANES + k];
        float w = o[6 * LANES + k];
        float squared = ((x * x + y * y) + z * z) + w * w;
        root[k] = squared > 0.0f && squared <= FLT_MAX ? squared : -1.0f;
    }
    for (uint32_t k = 0; k < LANES; ++k)
    {
        root[k] = root[k] > 0.0f ? sqrtf(root[k]) : -1.0f;
    }
    float inverse[LANES];
    for (uint32_t k = 0; k < LANES; ++k)
    {
        inverse[k] = 1.0f / (root[k] > 0.0f ? root[k] : 1.0f);
    }
    for (uint32_t c = 3; c < 7; ++c)
    {
        float fallback = c == 6 ? 1.0f : 0.0f;
        for (uint32_t k = 0; k < LANES; ++k)
        {
            o[c * LANES + k] = root[k] > 0.0f ? o[c * LANES + k] * inverse[k] : fallback;
        }
    }
}

static void SharesScalar(const float* total, float threshold, float* inverse, float* share)
{
    for (uint32_t k = 0; k < LANES; ++k)
    {
        bool under = total[k] < threshold;
        inverse[k] = 1.0f / (under ? threshold : total[k]);
        share[k] = under && total[k] > 0.0f ? (threshold - total[k]) * inverse[k] : 0.0f;
    }
}

static uint64_t s_state = 0x9E3779B97F4A7C15ull;

static uint32_t Next(void)
{
    s_state ^= s_state << 13;
    s_state ^= s_state >> 7;
    s_state ^= s_state << 17;
    return (uint32_t)(s_state >> 32);
}

// A float of every kind a pose may hold or a fault may leave: ordinary
// values of both signs over many scales, zeros of both signs,
// denormals, infinities, NaN and values whose squares overflow.
static float Value(void)
{
    static const float specials[] = {0.0f,  -0.0f, 1e-40f,  -1e-40f, INFINITY, -INFINITY,
                                     NAN,   1e20f, -1e20f,  3e19f,   FLT_MAX,  1.0f,
                                     -1.0f, 0.5f,  FLT_MIN, -FLT_MIN};
    uint32_t r = Next();
    if (r % 8 == 0)
    {
        return specials[(r >> 8) % (sizeof specials / sizeof specials[0])];
    }
    float unit = (float)(Next() >> 8) * (1.0f / 16777216.0f) * 2.0f - 1.0f;
    return ldexpf(unit, (int)((r >> 8) % 40) - 20);
}

// A weight as the blend makes them: shares of a total in [0, 1], at
// times 0, and a few signs and specials no blend makes, for the kernel.
static float Weight(void)
{
    uint32_t r = Next();
    if (r % 16 == 0)
    {
        return Value();
    }
    return r % 5 == 0 ? 0.0f : (float)(Next() >> 8) * (1.0f / 16777216.0f);
}

static void Fill(float* group, float (*make)(void), uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        group[i] = make();
    }
}

// Every value's bits the same, NaN for NaN: which of two NaN operands a
// result carries is the order of an operation's operands, which a
// compiler may swap in a commutative one, so no kernel can promise it.
static int SameBits(const float* a, const float* b, uint32_t count)
{
    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t x;
        uint32_t y;
        memcpy(&x, &a[i], sizeof x);
        memcpy(&y, &b[i], sizeof y);
        if (x != y && !(isnan(a[i]) && isnan(b[i])))
        {
            return 0;
        }
    }
    return 1;
}

static void TestAccumulate(void)
{
    int same = 1;
    for (uint32_t round = 0; round < 20000; ++round)
    {
        float out[FLOATS];
        float in[FLOATS];
        float w[LANES];
        Fill(out, Value, FLOATS);
        Fill(in, Value, FLOATS);
        Fill(w, Weight, LANES);
        bool first = round % 3 == 0;
        float expect[FLOATS];
        memcpy(expect, out, sizeof out);
        AccumulateScalar(expect, in, w, first);
        maniBlendLanesAccumulate(out, in, w, first);
        same &= SameBits(out, expect, FLOATS);
    }
    CHECK(same, "accumulating gives the scalar loop's bits");
}

static void TestNormalize(void)
{
    int same = 1;
    for (uint32_t round = 0; round < 20000; ++round)
    {
        float group[FLOATS];
        Fill(group, Value, FLOATS);
        float expect[FLOATS];
        memcpy(expect, group, sizeof group);
        NormalizeScalar(expect);
        maniBlendLanesNormalize(group);
        same &= SameBits(group, expect, FLOATS);
    }
    CHECK(same, "normalizing gives the scalar loop's bits");
}

// Each lane's own case once: a zero sum, an overflowing one, NaN, a
// denormal sum and ordinary ones, side by side.
static void TestNormalizeCases(void)
{
    float group[FLOATS] = {0};
    const float x[LANES] = {0.0f, 3e19f, NAN, 1e-25f, 0.6f, -0.0f, INFINITY, 2.0f};
    const float w[LANES] = {0.0f, 3e19f, 1.0f, 0.0f, 0.8f, 0.0f, 0.0f, -2.0f};
    for (uint32_t k = 0; k < LANES; ++k)
    {
        group[3 * LANES + k] = x[k];
        group[6 * LANES + k] = w[k];
    }
    float expect[FLOATS];
    memcpy(expect, group, sizeof group);
    NormalizeScalar(expect);
    maniBlendLanesNormalize(group);
    CHECK(SameBits(group, expect, FLOATS), "each special sum as the scalar loop gives it");
    CHECK(group[6 * LANES + 0] == 1.0f && group[3 * LANES + 0] == 0.0f,
          "a zero sum is the identity");
    CHECK(group[6 * LANES + 1] == 1.0f, "an overflowing sum is the identity");
}

// The weights' shares against the scalar loop, over totals of every
// kind and thresholds the blend takes.
static void TestShares(void)
{
    int same = 1;
    for (uint32_t round = 0; round < 20000; ++round)
    {
        float total[LANES];
        Fill(total, round % 2 == 0 ? Weight : Value, LANES);
        float threshold = (float)(Next() >> 8) * (1.0f / 16777216.0f) + 1e-6f;
        float inverse[LANES];
        float share[LANES];
        float inverseExpect[LANES];
        float shareExpect[LANES];
        SharesScalar(total, threshold, inverseExpect, shareExpect);
        maniBlendLanesShares(total, threshold, inverse, share);
        same &= SameBits(inverse, inverseExpect, LANES) && SameBits(share, shareExpect, LANES);
    }
    CHECK(same, "the shares give the scalar loop's bits");
}

int main(void)
{
    TestAccumulate();
    TestNormalize();
    TestNormalizeCases();
    TestShares();
    return s_failures == 0 ? 0 : 1;
}
