// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The library's own sine, cosine, arc cosine and arc tangent. Results
// must be the same bits on every platform, so nothing here
// calls a libm function other than the exactly rounded ones. Each works
// in binary64 and rounds once, which keeps it within one unit in the
// last place of the true value.

#ifndef MAUL_ANIM_SRC_SCALAR_H
#define MAUL_ANIM_SRC_SCALAR_H

// The sine and cosine of an angle in radians, within one unit in the last
// place for |angle| up to 2^20; for any finite angle both lie in [-1, 1].
// An infinite or NaN angle gives the canonical quiet NaN for both.
void maniSinCos(float angle, float* sinOut, float* cosOut);

// The sine of an angle in radians, as maniSinCos.
float maniSin(float angle);

// The cosine of an angle in radians, as maniSinCos.
float maniCos(float angle);

// The arc cosine in [0, pi], within one unit in the last place. A value
// outside [-1, 1] is taken as the nearer end; NaN gives the canonical
// quiet NaN.
float maniAcos(float value);

// The angle of the point (x, y) in [-pi, pi], within one unit in the last
// place, with C's results for zeros, signs and infinities; NaN gives the
// canonical quiet NaN.
float maniAtan2(float y, float x);

// Binary64 entry points for the offline side, which evaluates sources in
// binary64 and rounds once. The angle of (x, y) in [-pi, pi], about one
// unit in binary64's last place short of the true value at worst.
double maniAtan2Double(double y, double x);

// The sine and cosine of an angle in [0, pi / 2], in binary64.
void maniSinCosQuarterDouble(double angle, double* sinOut, double* cosOut);

// x to the power e for x positive and finite and e e^y within binary64's
// range, through the library's own logarithm and exponential: within
// about 1e-15 of the true value, relatively, times 1 + |e ln x|.
double maniPowDouble(double x, double e);

#endif // MAUL_ANIM_SRC_SCALAR_H
