// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library's binary32 elementary functions against the C library's
// binary64 ones, whose own error is a billionth of a float ulp, so the
// bounds hold on every platform alike. Each sweep holds a bound a little
// above the worst error measured on it, under the 1 ulp the functions
// promise.

#include "elementary_float.h"
#include "test_harness.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// SplitMix64: a fixed sequence, so every run checks the same inputs.
static uint64_t Next(uint64_t* state)
{
    uint64_t z = (*state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

// A uniform float in [lo, hi).
static float Uniform(uint64_t* state, float lo, float hi)
{
    return lo + (hi - lo) * ((float)(Next(state) >> 40) * 0x1.0p-24f);
}

// The error of a result in units in the last place of the float nearest
// the reference.
static double UlpError(float value, double reference)
{
    float nearest = (float)reference;
    float magnitude = fabsf(nearest);
    float ulp = nextafterf(magnitude, HUGE_VALF) - magnitude;
    return fabs((double)value - reference) / (double)ulp;
}

static void TestExpSpecialValues(void)
{
    CHECK(maniExpF(0.0f) == 1.0f, "e^0 is 1");
    CHECK(maniExpF(HUGE_VALF) == HUGE_VALF && maniExpF(-HUGE_VALF) == 0.0f, "infinities");
    CHECK(isnan(maniExpF(NAN)), "NaN");
    CHECK(maniExpF(0x1.62e42ep+6f) < HUGE_VALF, "the largest finite argument");
    CHECK(maniExpF(0x1.62e430p+6f) == HUGE_VALF, "the next overflows");
    CHECK(maniExpF(-0x1.9fe368p+6f) == 0x1.0p-149f, "the smallest subnormal");
    CHECK(maniExpF(-0x1.9fe36ap+6f) == 0.0f, "the next rounds to 0");
}

static void TestExpWithinOneUlp(void)
{
    uint64_t state = 1;
    double worst = 0.0;
    for (int i = 0; i < 400000; ++i)
    {
        float x = (i % 4 == 0) ? Uniform(&state, -87.0f, 88.7f) : Uniform(&state, -20.0f, 20.0f);
        double error = UlpError(maniExpF(x), exp((double)x));
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.75, "e^x within 0.75 ulp");
}

static void TestExp2IntegersExact(void)
{
    int wrong = 0;
    for (int k = -149; k <= 127; ++k)
    {
        wrong += maniExp2F((float)k) != ldexpf(1.0f, k);
    }
    CHECK(wrong == 0, "2^k is exact for an integer k");
    CHECK(maniExp2F(128.0f) == HUGE_VALF && maniExp2F(-151.0f) == 0.0f, "past the ends");
    CHECK(isnan(maniExp2F(NAN)), "NaN");
}

static void TestExp2WithinOneUlp(void)
{
    uint64_t state = 3;
    double worst = 0.0;
    for (int i = 0; i < 400000; ++i)
    {
        float x = (i % 4 == 0) ? Uniform(&state, -125.0f, 127.0f) : Uniform(&state, -12.0f, 12.0f);
        double error = UlpError(maniExp2F(x), exp2((double)x));
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.7, "2^x within 0.7 ulp");
}

static void TestLogSpecialValues(void)
{
    CHECK(maniLogF(1.0f) == 0.0f, "ln 1 is 0");
    CHECK(maniLogF(0.0f) == -HUGE_VALF && maniLogF(-0.0f) == -HUGE_VALF, "ln 0 is -inf");
    CHECK(isnan(maniLogF(-1.0f)) && isnan(maniLogF(NAN)), "NaN below 0 and for NaN");
    CHECK(maniLogF(HUGE_VALF) == HUGE_VALF, "ln inf is inf");
    CHECK(UlpError(maniLogF(0x1.0p-149f), log(0x1.0p-149)) <= 1.0, "ln of the smallest subnormal");
    CHECK(UlpError(maniLogF(FLT_MAX), log((double)FLT_MAX)) <= 1.0, "ln FLT_MAX");
}

static void TestLogWithinOneUlp(void)
{
    uint64_t state = 4;
    double worst = 0.0;
    for (int i = 0; i < 400000; ++i)
    {
        float x = ldexpf(Uniform(&state, 1.0f, 2.0f), (int)(Next(&state) % 277) - 149);
        double error = UlpError(maniLogF(x), log((double)x));
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.55, "ln x within 0.55 ulp");
}

static void TestLogNearOne(void)
{
    uint64_t state = 5;
    double worst = 0.0;
    for (int i = 0; i < 200000; ++i)
    {
        float x = 1.0f + ldexpf(Uniform(&state, -1.0f, 1.0f), -(int)(Next(&state) % 23));
        if (x == 1.0f)
        {
            continue;
        }
        double error = UlpError(maniLogF(x), log((double)x));
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.55, "ln x within 0.55 ulp near 1");
}

// The larger of the sine's and the cosine's errors at x.
static double SinCosError(float x)
{
    float sine = 0.0f;
    float cosine = 0.0f;
    maniSinCosF(x, &sine, &cosine);
    double sineError = UlpError(sine, sin((double)x));
    double cosineError = UlpError(cosine, cos((double)x));
    return sineError > cosineError ? sineError : cosineError;
}

static void TestSinCosSpecialValues(void)
{
    float sine = 1.0f;
    float cosine = 0.0f;
    maniSinCosF(0.0f, &sine, &cosine);
    CHECK(sine == 0.0f && !signbit(sine) && cosine == 1.0f, "exact at 0");
    maniSinCosF(-0.0f, &sine, &cosine);
    CHECK(sine == 0.0f && signbit(sine) && cosine == 1.0f, "the sign of -0 is kept");
    maniSinCosF(0x1.0p-14f, &sine, &cosine);
    CHECK(sine == 0x1.0p-14f && cosine == 1.0f, "a tiny angle");
    maniSinCosF(HUGE_VALF, &sine, &cosine);
    CHECK(isnan(sine) && isnan(cosine), "infinity gives NaN");
    maniSinCosF(NAN, &sine, &cosine);
    CHECK(isnan(sine) && isnan(cosine), "NaN gives NaN");
}

static void TestSinCosWithinOneUlp(void)
{
    static const float ranges[] = {1.0f, 10.0f, 2000.0f, 1.0e5f};
    for (size_t r = 0; r < sizeof ranges / sizeof ranges[0]; ++r)
    {
        uint64_t state = 10 + r;
        double worst = 0.0;
        for (int i = 0; i < 200000; ++i)
        {
            double error = SinCosError(Uniform(&state, -ranges[r], ranges[r]));
            worst = error > worst ? error : worst;
        }
        CHECK(worst <= 0.8, "sine and cosine within 0.8 ulp up to 1e5");
    }
}

static void TestSinCosNearQuadrantBoundaries(void)
{
    // The floats nearest k pi / 2 and their neighbors, where the reduced
    // argument is small and its leading bits cancel.
    double worst = 0.0;
    for (int k = 1; k < 65536; ++k)
    {
        float x = (float)((double)k * 0x1.921fb54442d18p+0);
        for (int step = 0; step < 3; ++step)
        {
            double error = SinCosError(x);
            worst = error > worst ? error : worst;
            x = nextafterf(x, HUGE_VALF);
        }
    }
    CHECK(worst <= 0.55, "sine and cosine within 0.55 ulp next to k pi / 2");
}

// The quiet NaN with its sign clear, by its bits.
static bool IsCanonicalNan(float x)
{
    uint32_t bits = 0;
    memcpy(&bits, &x, sizeof bits);
    return bits == 0x7FC00000u;
}

static void TestSinCosNonFinite(void)
{
    static const float inputs[] = {HUGE_VALF, -HUGE_VALF, NAN, -NAN};
    bool canonical = true;
    for (size_t i = 0; i < sizeof inputs / sizeof inputs[0]; ++i)
    {
        float sine = 0.0f;
        float cosine = 0.0f;
        maniSinCosF(inputs[i], &sine, &cosine);
        canonical = canonical && IsCanonicalNan(sine) && IsCanonicalNan(cosine);
    }
    CHECK(canonical, "a non-finite x gives the same quiet NaN on every target");
}

static void TestSinCosFarOut(void)
{
    // Every binade from the far reduction's start to the largest float,
    // both signs, and the points Maul Anim found unbounded before.
    uint64_t state = 16;
    double worst = 0.0;
    bool odd = true;
    for (int e = 16; e <= 127; ++e)
    {
        for (int i = 0; i < 400; ++i)
        {
            float x = ldexpf(Uniform(&state, 1.0f, 2.0f), e);
            if (!isfinite(x))
            {
                continue;
            }
            double error = SinCosError(x);
            worst = error > worst ? error : worst;
            float sine = 0.0f;
            float cosine = 0.0f;
            float negativeSine = 0.0f;
            float negativeCosine = 0.0f;
            maniSinCosF(x, &sine, &cosine);
            maniSinCosF(-x, &negativeSine, &negativeCosine);
            odd = odd && negativeSine == -sine && negativeCosine == cosine;
        }
    }
    static const float points[] = {1e6f,    0x1.0p+25f, 1e10f,          3e20f,
                                   FLT_MAX, 0x1.8p+16f, 0x1.7ffffep+16f};
    for (size_t i = 0; i < sizeof points / sizeof points[0]; ++i)
    {
        double error = SinCosError(points[i]);
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.85, "sine and cosine within 0.85 ulp out to the largest float");
    CHECK(odd, "sine odd and cosine even out there");
}

static void TestSinCosOnTheUnitCircle(void)
{
    uint64_t state = 15;
    double worst = 0.0;
    for (int i = 0; i < 200000; ++i)
    {
        float sine = 0.0f;
        float cosine = 0.0f;
        maniSinCosF(Uniform(&state, -2000.0f, 2000.0f), &sine, &cosine);
        double unit = fabs((double)sine * (double)sine + (double)cosine * (double)cosine - 1.0);
        worst = unit > worst ? unit : worst;
    }
    CHECK(worst <= 2.0 * (double)FLT_EPSILON, "on the unit circle");
}

static void TestAtan2SpecialValues(void)
{
    const float pi = 0x1.921fb6p+1f;
    const float half = 0x1.921fb6p+0f;
    CHECK(maniAtan2F(0.0f, 1.0f) == 0.0f && !signbit(maniAtan2F(0.0f, 1.0f)), "+0 at +x");
    CHECK(maniAtan2F(-0.0f, 1.0f) == 0.0f && signbit(maniAtan2F(-0.0f, 1.0f)), "-0 at +x");
    CHECK(maniAtan2F(0.0f, -1.0f) == pi && maniAtan2F(-0.0f, -1.0f) == -pi, "+-pi at -x");
    CHECK(maniAtan2F(0.0f, -0.0f) == pi && maniAtan2F(-0.0f, -0.0f) == -pi, "zero over -0");
    CHECK(maniAtan2F(1.0f, 0.0f) == half && maniAtan2F(-1.0f, -0.0f) == -half, "the y axis");
    CHECK(maniAtan2F(HUGE_VALF, HUGE_VALF) == 0.5f * half, "pi / 4 at two infinities");
    CHECK(UlpError(maniAtan2F(HUGE_VALF, -HUGE_VALF), atan2(HUGE_VAL, -HUGE_VAL)) <= 1.0,
          "3 pi / 4 at two infinities");
    CHECK(maniAtan2F(5.0f, HUGE_VALF) == 0.0f && maniAtan2F(-5.0f, -HUGE_VALF) == -pi,
          "an infinite x");
    CHECK(isnan(maniAtan2F(NAN, 1.0f)) && isnan(maniAtan2F(1.0f, NAN)), "NaN");
    CHECK(maniAtan2F(FLT_MAX, FLT_MAX) == 0.5f * half, "pi / 4 at the largest floats");
    CHECK(UlpError(maniAtan2F(0x1.0p-149f, 3.0f), atan2(0x1.0p-149, 3.0)) <= 1.0,
          "a subnormal angle");
}

static void TestAtan2WithinOneUlp(void)
{
    uint64_t state = 30;
    double worst = 0.0;
    for (int i = 0; i < 400000; ++i)
    {
        float y = ldexpf(Uniform(&state, -1.0f, 1.0f), (int)(Next(&state) % 80) - 40);
        float x = ldexpf(Uniform(&state, -1.0f, 1.0f), (int)(Next(&state) % 80) - 40);
        double error = UlpError(maniAtan2F(y, x), atan2((double)y, (double)x));
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.505, "atan2 within 0.505 ulp, all but correctly rounded");
}

static void TestCbrtSpecialValues(void)
{
    CHECK(maniCbrtF(0.0f) == 0.0f && signbit(maniCbrtF(-0.0f)), "signed zeros");
    CHECK(maniCbrtF(HUGE_VALF) == HUGE_VALF && maniCbrtF(-HUGE_VALF) == -HUGE_VALF, "infinities");
    CHECK(isnan(maniCbrtF(NAN)), "NaN");
    int wrong = 0;
    for (int k = -255; k <= 255; ++k)
    {
        float cube = (float)(k * k * k);
        wrong += maniCbrtF(cube) != (float)k;
    }
    CHECK(wrong == 0, "the cube root of an integer's cube is the integer");
    CHECK(UlpError(maniCbrtF(0x1.0p-149f), cbrt(0x1.0p-149)) <= 0.55, "the smallest subnormal");
}

static void TestCbrtWithinHalfAnUlp(void)
{
    uint64_t state = 40;
    double worst = 0.0;
    for (int i = 0; i < 400000; ++i)
    {
        float x = ldexpf(Uniform(&state, -2.0f, 2.0f), (int)(Next(&state) % 277) - 149);
        double error = UlpError(maniCbrtF(x), cbrt((double)x));
        worst = error > worst ? error : worst;
    }
    CHECK(worst <= 0.505, "cube root within 0.505 ulp, all but correctly rounded");
}

static void TestPowSpecialValues(void)
{
    CHECK(maniPowF(0.0f, 0.0f) == 1.0f && maniPowF(NAN, 0.0f) == 1.0f, "x^0 is 1");
    CHECK(maniPowF(1.0f, NAN) == 1.0f && maniPowF(1.0f, HUGE_VALF) == 1.0f, "1^y is 1");
    CHECK(isnan(maniPowF(-2.0f, 2.0f)), "a negative base is NaN");
    CHECK(isnan(maniPowF(NAN, 2.0f)) && isnan(maniPowF(2.0f, NAN)), "NaN");
    CHECK(maniPowF(0.0f, 3.0f) == 0.0f && maniPowF(0.0f, -3.0f) == HUGE_VALF, "powers of 0");
    CHECK(maniPowF(HUGE_VALF, 0.5f) == HUGE_VALF && maniPowF(HUGE_VALF, -0.5f) == 0.0f,
          "powers of infinity");
    CHECK(maniPowF(2.0f, HUGE_VALF) == HUGE_VALF && maniPowF(0.5f, HUGE_VALF) == 0.0f,
          "to +infinity");
    CHECK(maniPowF(2.0f, -HUGE_VALF) == 0.0f && maniPowF(0.5f, -HUGE_VALF) == HUGE_VALF,
          "to -infinity");
    CHECK(maniPowF(10.0f, 40.0f) == HUGE_VALF && maniPowF(10.0f, -50.0f) == 0.0f, "past the range");
    int wrong = 0;
    for (int k = -149; k <= 127; ++k)
    {
        wrong += maniPowF(2.0f, (float)k) != ldexpf(1.0f, k);
    }
    float ten = 1.0f;
    for (int k = 0; k <= 10; ++k)
    {
        wrong += maniPowF(10.0f, (float)k) != ten;
        ten *= 10.0f;
    }
    CHECK(wrong == 0, "2^k and 10^k up to 10^10 are exact");
}

static void TestPowWithinOneUlp(void)
{
    // |y ln x| up to 8 and 32 within 1 ulp, and over the whole range,
    // where the logarithm's remaining error shows most.
    static const float limits[] = {8.0f, 32.0f, 88.0f};
    static const double bounds[] = {0.7, 0.7, 1.25};
    for (int l = 0; l < 3; ++l)
    {
        uint64_t state = 41;
        double worst = 0.0;
        for (int i = 0; i < 200000; ++i)
        {
            float x = maniExpF(Uniform(&state, -20.0f, 20.0f));
            float logX = fabsf(maniLogF(x));
            float y = Uniform(&state, -limits[l], limits[l]) / (logX > 1e-3f ? logX : 1.0f);
            double error = UlpError(maniPowF(x, y), pow((double)x, (double)y));
            worst = error > worst ? error : worst;
        }
        CHECK(worst <= bounds[l], "x^y within its bound");
    }
}

static void TestPowOnTheSrgbRanges(void)
{
    double worst = 0.0;
    for (int i = 0; i <= 1000; ++i)
    {
        float toLinear = 0.0899f + 0.9101f * (float)i / 1000.0f;
        float fromLinear = 0.0031308f + 0.9968692f * (float)i / 1000.0f;
        double a = UlpError(maniPowF(toLinear, 2.4f), pow((double)toLinear, (double)2.4f));
        double b = UlpError(maniPowF(fromLinear, 1.0f / 2.4f),
                            pow((double)fromLinear, (double)(1.0f / 2.4f)));
        worst = a > worst ? a : worst;
        worst = b > worst ? b : worst;
    }
    CHECK(worst <= 0.7, "the sRGB powers within 0.7 ulp");
}

int main(void)
{
    TestExpSpecialValues();
    TestExpWithinOneUlp();
    TestExp2IntegersExact();
    TestExp2WithinOneUlp();
    TestLogSpecialValues();
    TestLogWithinOneUlp();
    TestLogNearOne();
    TestSinCosSpecialValues();
    TestSinCosWithinOneUlp();
    TestSinCosNearQuadrantBoundaries();
    TestSinCosNonFinite();
    TestSinCosFarOut();
    TestSinCosOnTheUnitCircle();
    TestAtan2SpecialValues();
    TestAtan2WithinOneUlp();
    TestCbrtSpecialValues();
    TestCbrtWithinHalfAnUlp();
    TestPowSpecialValues();
    TestPowWithinOneUlp();
    TestPowOnTheSrgbRanges();
    return s_failures == 0 ? 0 : 1;
}
