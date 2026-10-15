// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Binary64 quaternion operations for the offline side.

#include "quat64.h"

#include "scalar.h"

#include <math.h>

void maniSlerpDouble(const double a[4], const double b[4], double u, double out[4])
{
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    double sign = dot < 0.0 ? -1.0 : 1.0;
    double difference = 0.0;
    double sum = 0.0;
    for (int k = 0; k < 4; ++k)
    {
        double d = a[k] - sign * b[k];
        double s = a[k] + sign * b[k];
        difference += d * d;
        sum += s * s;
    }
    // The angle between them, from half-angle lengths: well conditioned
    // at every angle, unlike the arc cosine of the dot product.
    double angle = 2.0 * maniAtan2Double(sqrt(difference), sqrt(sum));
    double wa = 1.0 - u;
    double wb = u;
    if (angle > 1e-9)
    {
        double sinAngle = 0.0;
        double unused = 0.0;
        double sinA = 0.0;
        double sinB = 0.0;
        maniSinCosQuarterDouble(angle, &sinAngle, &unused);
        maniSinCosQuarterDouble((1.0 - u) * angle, &sinA, &unused);
        maniSinCosQuarterDouble(u * angle, &sinB, &unused);
        wa = sinA / sinAngle;
        wb = sinB / sinAngle;
    }
    for (int k = 0; k < 4; ++k)
    {
        out[k] = wa * a[k] + sign * wb * b[k];
    }
}
