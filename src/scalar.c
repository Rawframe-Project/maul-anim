// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Sine and cosine reduce the angle by multiples of pi/2 with pi/2 split in
// two parts (Cody and Waite), then sum their Taylor series to the 13th and
// 14th powers; the arc tangent halves its argument twice with
// atan(t) = 2 atan(t / (1 + sqrt(1 + t^2))) and sums the series to the
// 17th power. These are the shortest sums that round to the same binary32
// as the C library's binary64 functions at every one of 2^24 points over
// eight turns and over [0, 1]; one term fewer changes a few of them. Every step is an exactly
// rounded binary64 operation, so the result is the same everywhere; a NaN result is always the
// canonical quiet NaN, since NaN payloads differ between platforms.

#include "scalar.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

// pi/2 as a 33-bit head, so that k times it is exact for |k| < 2^20, and
// the binary64 tail of the remainder.
static const double s_halfPiHead = 1.57079632673412561417e+00;
static const double s_halfPiTail = 6.07710050650619224932e-11;
static const double s_halfPi = 1.57079632679489655800e+00;
static const double s_pi = 3.14159265358979311600e+00;
static const double s_twoOverPi = 6.36619772367581382433e-01;
static const double s_twoPi = 6.28318530717958623200e+00;

// Taylor coefficients 1/n! with alternating signs.
static const double s_sinCoefficients[] = {
    -1.0 / 6.0, 1.0 / 120.0, -1.0 / 5040.0, 1.0 / 362880.0, -1.0 / 39916800.0, 1.0 / 6227020800.0,
};
static const double s_cosCoefficients[] = {
    -1.0 / 2.0,       1.0 / 24.0,        -1.0 / 720.0,         1.0 / 40320.0,
    -1.0 / 3628800.0, 1.0 / 479001600.0, -1.0 / 87178291200.0,
};

static double SinSeries(double r)
{
    // The sum below would turn -0 into +0.
    if (r == 0.0)
    {
        return r;
    }
    double r2 = r * r;
    double p = s_sinCoefficients[5];
    for (int32_t i = 4; i >= 0; --i)
    {
        p = s_sinCoefficients[i] + r2 * p;
    }
    return r + r * r2 * p;
}

static double CosSeries(double r)
{
    double r2 = r * r;
    double p = s_cosCoefficients[6];
    for (int32_t i = 5; i >= 0; --i)
    {
        p = s_cosCoefficients[i] + r2 * p;
    }
    return 1.0 + r2 * p;
}

void maniSinCos(float angle, float* sinOut, float* cosOut)
{
    double x = (double)angle;
    if (!(fabs(x) <= 3.5e38))
    {
        *sinOut = NAN;
        *cosOut = NAN;
        return;
    }
    // Beyond 2^30 no reduction keeps the promised accuracy; bring the angle
    // into range first so the quotient below fits and the result stays
    // bounded.
    if (fabs(x) > 1073741824.0)
    {
        x = fmod(x, s_twoPi);
    }
    double q = x * s_twoOverPi;
    int64_t k = (int64_t)(q >= 0.0 ? q + 0.5 : q - 0.5);
    double kd = (double)k;
    double r = (x - kd * s_halfPiHead) - kd * s_halfPiTail;
    double s = SinSeries(r);
    double c = CosSeries(r);
    switch ((uint32_t)k & 3u)
    {
    case 0:
        *sinOut = (float)s;
        *cosOut = (float)c;
        break;
    case 1:
        *sinOut = (float)c;
        *cosOut = (float)-s;
        break;
    case 2:
        *sinOut = (float)-s;
        *cosOut = (float)-c;
        break;
    default:
        *sinOut = (float)-c;
        *cosOut = (float)s;
        break;
    }
}

float maniSin(float angle)
{
    float s = 0.0f;
    float c = 0.0f;
    maniSinCos(angle, &s, &c);
    return s;
}

float maniCos(float angle)
{
    float s = 0.0f;
    float c = 0.0f;
    maniSinCos(angle, &s, &c);
    return c;
}

// The arc tangent of t in [0, 1].
static double AtanUnit(double t)
{
    double u = t / (1.0 + sqrt(1.0 + t * t));
    u = u / (1.0 + sqrt(1.0 + u * u));
    double u2 = u * u;
    double p = 1.0 / 17.0;
    for (int32_t n = 15; n >= 1; n -= 2)
    {
        p = 1.0 / (double)n - u2 * p;
    }
    return 4.0 * u * p;
}

// The angle of (x, y) for finite or infinite x and y, not NaN.
double maniAtan2Double(double y, double x)
{
    double ay = fabs(y);
    double ax = fabs(x);
    double base = 0.0;
    if (isinf(ay) && isinf(ax))
    {
        base = s_pi / 4.0;
    }
    else if (ay <= ax)
    {
        base = ax == 0.0 ? 0.0 : AtanUnit(ay / ax);
    }
    else
    {
        base = s_halfPi - AtanUnit(ax / ay);
    }
    if (signbit(x))
    {
        base = s_pi - base;
    }
    return signbit(y) ? -base : base;
}

float maniAtan2(float y, float x)
{
    if (isnan(y) || isnan(x))
    {
        return NAN;
    }
    return (float)maniAtan2Double((double)y, (double)x);
}

float maniAcos(float value)
{
    if (isnan(value))
    {
        return NAN;
    }
    double v = (double)value;
    v = v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v);
    return (float)maniAtan2Double(sqrt((1.0 - v) * (1.0 + v)), v);
}

void maniSinCosQuarterDouble(double angle, double* sinOut, double* cosOut)
{
    if (angle <= s_halfPi / 2.0)
    {
        *sinOut = SinSeries(angle);
        *cosOut = CosSeries(angle);
        return;
    }
    // sin x = cos(pi / 2 - x), the difference taken in two parts.
    double r = (s_halfPiHead - angle) + s_halfPiTail;
    *sinOut = CosSeries(r);
    *cosOut = SinSeries(r);
}

// ln 2 as a head with zeros in its low 32 bits, so that n times it is
// exact for |n| < 2^20, and the tail.
static const double s_ln2Head = 6.93147180369123816490e-01;
static const double s_ln2Tail = 1.90821492927058770002e-10;

// The natural logarithm of a positive finite x: x = m 2^k with m in
// [sqrt(1/2), sqrt(2)], and ln m = 2 atanh(s), s = (m - 1) / (m + 1),
// |s| < 0.172, by its series to the 25th power.
static double LogPositive(double x)
{
    int32_t k = 0;
    if (x < 2.2250738585072014e-308)
    {
        // Subnormal: made normal by 2^54.
        x *= 18014398509481984.0;
        k = -54;
    }
    uint64_t bits = 0;
    memcpy(&bits, &x, sizeof bits);
    k += (int32_t)((bits >> 52) & 0x7ffu) - 1023;
    bits = (bits & 0x000fffffffffffffu) | 0x3ff0000000000000u;
    double m = 1.0;
    memcpy(&m, &bits, sizeof m);
    if (m > 1.4142135623730951)
    {
        m *= 0.5;
        ++k;
    }
    double s = (m - 1.0) / (m + 1.0);
    double s2 = s * s;
    double p = 1.0 / 25.0;
    for (int32_t n = 23; n >= 1; n -= 2)
    {
        p = 1.0 / (double)n + s2 * p;
    }
    return ((double)k * s_ln2Head + 2.0 * s * p) + (double)k * s_ln2Tail;
}

// 2^n for n in [-1022, 1023].
static double PowerOfTwo(int32_t n)
{
    uint64_t bits = (uint64_t)(n + 1023) << 52;
    double d = 0.0;
    memcpy(&d, &bits, sizeof d);
    return d;
}

// e^y for y in [-746, 710]: y = n ln 2 + r, |r| < ln 2 (n truncated),
// e^r by its Taylor series to the 20th power, scaled by 2^n in two
// halves (each within the normal range).
static double Exp(double y)
{
    int32_t n = (int32_t)(y * 1.4426950408889634);
    double r = (y - (double)n * s_ln2Head) - (double)n * s_ln2Tail;
    double p = 1.0;
    for (int32_t k = 20; k >= 1; --k)
    {
        p = 1.0 + p * r / (double)k;
    }
    int32_t half = n / 2;
    return p * PowerOfTwo(half) * PowerOfTwo(n - half);
}

double maniPowDouble(double x, double e)
{
    double y = e * LogPositive(x);
    if (y > 710.0)
    {
        return (double)INFINITY;
    }
    return y < -746.0 ? 0.0 : Exp(y);
}
