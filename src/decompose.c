// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Everything here runs in binary64 and rounds to binary32 once. Matrices
// are 3x3, column-major: m[3 * c + r] is row r of column c.

#include "decompose.h"

#include "vecmath.h"

#include <math.h>
#include <string.h>

enum
{
    kMaxSteps = 40,
};

static double Max(double a, double b)
{
    return a > b ? a : b;
}

static double Determinant(const double m[9])
{
    return m[0] * (m[4] * m[8] - m[7] * m[5]) - m[3] * (m[1] * m[8] - m[7] * m[2]) +
           m[6] * (m[1] * m[5] - m[4] * m[2]);
}

// The inverse's transpose: the cofactor matrix over the determinant.
static void InverseTranspose(const double m[9], double det, double out[9])
{
    out[0] = (m[4] * m[8] - m[7] * m[5]) / det;
    out[1] = (m[6] * m[5] - m[3] * m[8]) / det;
    out[2] = (m[3] * m[7] - m[6] * m[4]) / det;
    out[3] = (m[7] * m[2] - m[1] * m[8]) / det;
    out[4] = (m[0] * m[8] - m[6] * m[2]) / det;
    out[5] = (m[6] * m[1] - m[0] * m[7]) / det;
    out[6] = (m[1] * m[5] - m[4] * m[2]) / det;
    out[7] = (m[3] * m[2] - m[0] * m[5]) / det;
    out[8] = (m[0] * m[4] - m[3] * m[1]) / det;
}

static double Frobenius(const double m[9])
{
    double sum = 0.0;
    for (int i = 0; i < 9; ++i)
    {
        sum += m[i] * m[i];
    }
    return sqrt(sum);
}

// Higham's scaled Newton iteration toward the orthogonal polar factor of
// a matrix of positive determinant.
static void PolarRotation(const double a[9], double q[9])
{
    memcpy(q, a, 9 * sizeof(double));
    for (int step = 0; step < kMaxSteps; ++step)
    {
        double det = Determinant(q);
        double inverseT[9];
        InverseTranspose(q, det, inverseT);
        double gamma = sqrt(Frobenius(inverseT) / Frobenius(q));
        double change = 0.0;
        for (int i = 0; i < 9; ++i)
        {
            double next = 0.5 * (gamma * q[i] + inverseT[i] / gamma);
            change += (next - q[i]) * (next - q[i]);
            q[i] = next;
        }
        if (sqrt(change) <= 0x1p-45 * Frobenius(q))
        {
            break;
        }
    }
}

static double Length(const double v[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void Cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void Normalize(double v[3])
{
    double length = Length(v);
    for (int k = 0; k < 3; ++k)
    {
        v[k] /= length;
    }
}

// A rotation from the columns that are not zero (zero[c] marks those
// that are), by cross products in the order x, y, z; a column the
// others cannot fix takes a perpendicular of the first that is not zero.
static void DegenerateRotation(const double a[9], const bool zero[3], double q[9])
{
    static const double axes[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double* c[3] = {q, q + 3, q + 6};
    int first = zero[0] ? (zero[1] ? (zero[2] ? -1 : 2) : 1) : 0;
    if (first < 0)
    {
        memcpy(q, axes, sizeof axes);
        return;
    }
    memcpy(c[first], a + 3 * first, 3 * sizeof(double));
    Normalize(c[first]);
    // The second axis: the next column that is not zero, or one made
    // from the last; c[third] = first x helper, c[second] = third x first.
    int second = (first + 1) % 3;
    int third = (first + 2) % 3;
    double helper[3] = {0.0, 0.0, 0.0};
    if (!zero[second])
    {
        memcpy(helper, a + 3 * second, sizeof helper);
    }
    else if (!zero[third])
    {
        double t[3];
        memcpy(t, a + 3 * third, sizeof t);
        Cross(t, c[first], helper);
    }
    Cross(c[first], helper, c[third]);
    // No helper, or one parallel to the first axis: the world axis least
    // aligned with the first.
    if (Length(c[third]) <= 1e-12 * Length(helper) || Length(helper) == 0.0)
    {
        int least = 0;
        for (int k = 1; k < 3; ++k)
        {
            least = fabs(c[first][k]) < fabs(c[first][least]) ? k : least;
        }
        Cross(c[first], axes + 3 * least, c[third]);
    }
    Normalize(c[third]);
    Cross(c[third], c[first], c[second]);
}

// Shepperd's method: the largest of the four candidates is divided by.
static maniQuat ToQuaternion(const double q[9])
{
    double m00 = q[0];
    double m11 = q[4];
    double m22 = q[8];
    double trace = m00 + m11 + m22;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
    if (trace >= m00 && trace >= m11 && trace >= m22)
    {
        double s = 2.0 * sqrt(1.0 + trace);
        w = 0.25 * s;
        x = (q[5] - q[7]) / s;
        y = (q[6] - q[2]) / s;
        z = (q[1] - q[3]) / s;
    }
    else if (m00 >= m11 && m00 >= m22)
    {
        double s = 2.0 * sqrt(1.0 + m00 - m11 - m22);
        x = 0.25 * s;
        w = (q[5] - q[7]) / s;
        y = (q[3] + q[1]) / s;
        z = (q[6] + q[2]) / s;
    }
    else if (m11 >= m22)
    {
        double s = 2.0 * sqrt(1.0 + m11 - m00 - m22);
        y = 0.25 * s;
        w = (q[6] - q[2]) / s;
        x = (q[3] + q[1]) / s;
        z = (q[7] + q[5]) / s;
    }
    else
    {
        double s = 2.0 * sqrt(1.0 + m22 - m00 - m11);
        z = 0.25 * s;
        w = (q[1] - q[3]) / s;
        x = (q[6] + q[2]) / s;
        y = (q[7] + q[5]) / s;
    }
    double length = sqrt(x * x + y * y + z * z + w * w);
    double sign = w < 0.0 ? -1.0 : 1.0;
    return (maniQuat){(float)(sign * x / length), (float)(sign * y / length),
                      (float)(sign * z / length), (float)(sign * w / length)};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

bool maniDecompose(const maniAffine* matrix, maniTransform* out)
{
    for (int i = 0; i < 12; ++i)
    {
        if (!Finite(matrix->m[i]))
        {
            return false;
        }
    }
    double a[9];
    for (int i = 0; i < 9; ++i)
    {
        a[i] = (double)matrix->m[i];
    }
    double lengths[3] = {Length(a), Length(a + 3), Length(a + 6)};
    double largest = Max(lengths[0], Max(lengths[1], lengths[2]));
    double det = Determinant(a);
    double q[9];
    double scale[3];
    if (largest == 0.0 || fabs(det) < 1e-30 * largest * largest * largest)
    {
        bool zero[3];
        for (int c = 0; c < 3; ++c)
        {
            zero[c] = lengths[c] <= 1e-30 * largest || lengths[c] == 0.0;
        }
        DegenerateRotation(a, zero, q);
        memcpy(scale, lengths, sizeof scale);
    }
    else
    {
        double flip = det < 0.0 ? -1.0 : 1.0;
        for (int k = 0; k < 3; ++k)
        {
            a[k] *= flip;
        }
        PolarRotation(a, q);
        for (int c = 0; c < 3; ++c)
        {
            scale[c] =
                q[3 * c] * a[3 * c] + q[3 * c + 1] * a[3 * c + 1] + q[3 * c + 2] * a[3 * c + 2];
        }
        scale[0] *= flip;
    }
    out->translation = (maniVec3){matrix->m[9], matrix->m[10], matrix->m[11]};
    out->rotation = ToQuaternion(q);
    out->scale = (maniVec3){(float)scale[0], (float)scale[1], (float)scale[2]};
    return true;
}

static double Dot3(const double* a, const double* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void maniDecomposeRigid(const maniAffine* matrix, maniVec3* translationOut, maniQuat* rotationOut)
{
    double a[9];
    bool finite = true;
    for (int i = 0; i < 12; ++i)
    {
        finite = finite && Finite(matrix->m[i]);
    }
    for (int i = 0; i < 9; ++i)
    {
        a[i] = (double)matrix->m[i];
    }
    double lengths[3] = {Length(a), Length(a + 3), Length(a + 6)};
    bool orthogonal = finite && lengths[0] > 0.0 && lengths[1] > 0.0 && lengths[2] > 0.0 &&
                      fabs(Dot3(a, a + 3)) <= 1e-6 * lengths[0] * lengths[1] &&
                      fabs(Dot3(a, a + 6)) <= 1e-6 * lengths[0] * lengths[2] &&
                      fabs(Dot3(a + 3, a + 6)) <= 1e-6 * lengths[1] * lengths[2];
    *translationOut = (maniVec3){matrix->m[9], matrix->m[10], matrix->m[11]};
    if (!orthogonal)
    {
        maniTransform t = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
        (void)maniDecompose(matrix, &t);
        *translationOut = t.translation;
        *rotationOut = t.rotation;
        return;
    }
    double q[9];
    for (int c = 0; c < 3; ++c)
    {
        double inverse = 1.0 / lengths[c];
        for (int k = 0; k < 3; ++k)
        {
            q[3 * c + k] = a[3 * c + k] * inverse;
        }
    }
    if (Determinant(q) < 0.0)
    {
        // A mirror: x's column negated, as maniDecompose gives it.
        for (int k = 0; k < 3; ++k)
        {
            q[k] = -q[k];
        }
    }
    *rotationOut = ToQuaternion(q);
}

static float ColumnLength(const maniAffine* a, uint32_t c)
{
    const float* v = a->m + 3 * c;
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool maniDecomposeExact(const maniAffine* matrix, maniTransform* out)
{
    if (!maniDecompose(matrix, out))
    {
        return false;
    }
    maniAffine back = maniAffineFromTrs(out->translation, out->rotation, out->scale);
    float largest = ColumnLength(matrix, 0);
    largest = ColumnLength(matrix, 1) > largest ? ColumnLength(matrix, 1) : largest;
    largest = ColumnLength(matrix, 2) > largest ? ColumnLength(matrix, 2) : largest;
    for (uint32_t i = 0; i < 9; ++i)
    {
        if (!(fabsf(back.m[i] - matrix->m[i]) <= 1e-4f * largest))
        {
            return false;
        }
    }
    return true;
}
