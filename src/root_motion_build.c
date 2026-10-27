// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Taking root motion out of a bone at build, in binary64: at
// each sample the bone's local transform gives the motion M (its
// translation from a reference along the chosen axes, its rotation from a
// reference whole or as the twist about an axis), the loop option spreads
// the end's difference from the start over the clip, and the bone keeps
// M^-1 times its transform, so M times the stored bone is the source.

#include "root_motion_build.h"

#include "allocator.h"
#include "bytes.h"
#include "clip_file.h"
#include "quat64.h"

#include "maul-anim/base.h"
#include "maul-anim/rootmotion.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOVE 7u

static void Normalize(double q[4])
{
    double length = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(length > 0.0))
    {
        q[0] = q[1] = q[2] = 0.0;
        q[3] = 1.0;
        return;
    }
    for (int k = 0; k < 4; ++k)
    {
        q[k] /= length;
    }
}

// a times b.
static void Multiply(const double a[4], const double b[4], double out[4])
{
    double m[4] = {
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    };
    memcpy(out, m, sizeof m);
}

static void Conjugate(const double q[4], double out[4])
{
    out[0] = -q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = q[3];
}

// v turned by the unit q.
static void Rotate(const double q[4], const double v[3], double out[3])
{
    // v + 2 w (u x v) + 2 u x (u x v), u the vector part.
    double c[3] = {q[1] * v[2] - q[2] * v[1], q[2] * v[0] - q[0] * v[2], q[0] * v[1] - q[1] * v[0]};
    double cc[3] = {q[1] * c[2] - q[2] * c[1], q[2] * c[0] - q[0] * c[2],
                    q[0] * c[1] - q[1] * c[0]};
    for (int k = 0; k < 3; ++k)
    {
        out[k] = v[k] + 2.0 * (q[3] * c[k] + cc[k]);
    }
}

// The references: translation and rotation, by the source's choice
// (locals the first sample's, rest the bone's rest).
static void References(const maniRootMotionSource* s, const double* first, const double* rest,
                       double t[3], double r[4])
{
    static const double kOrigin[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    const double* tFrom = s->translationReference == maniRootReferenceRest    ? rest
                          : s->translationReference == maniRootReferenceFirst ? first
                                                                              : kOrigin;
    const double* rFrom = s->rotationReference == maniRootReferenceRest    ? rest
                          : s->rotationReference == maniRootReferenceFirst ? first
                                                                           : kOrigin;
    memcpy(t, tFrom, 3 * sizeof(double));
    memcpy(r, rFrom + 3, 4 * sizeof(double));
}

// The motion of one local transform: m_t the chosen axes of t - t_ref,
// m_r r r_ref^-1 whole, its twist about the axis, or none.
static void Motion(const maniRootMotionSource* s, const double t[3], const double r[4],
                   const double tRef[3], const double rRef[4], double out[MOVE])
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        out[k] = ((s->translationAxes >> k) & 1u) != 0 ? t[k] - tRef[k] : 0.0;
    }
    double back[4];
    double q[4];
    Conjugate(rRef, back);
    Multiply(r, back, q);
    Normalize(q);
    double* m = out + 3;
    if (s->rotation == maniRootRotationNone)
    {
        m[0] = m[1] = m[2] = 0.0;
        m[3] = 1.0;
        return;
    }
    if (s->rotation == maniRootRotationTwist)
    {
        // The vector part projected onto the axis; at a half turn of
        // swing (both 0) no twist is defined, and none is taken.
        double twist[4] = {0.0, 0.0, 0.0, q[3]};
        twist[s->twistAxis] = q[s->twistAxis];
        memcpy(q, twist, sizeof twist);
        Normalize(q);
    }
    memcpy(m, q, sizeof q);
}

// The loop option: the end's difference from the start taken off in
// proportion to time, translation by subtraction, rotation by the slerp
// of the correction from the identity.
static void Loop(maniRootMotionWork* w, const double* times)
{
    uint32_t n = w->sampleCount;
    double duration = n > 0 ? times[n - 1] : 0.0;
    if (n < 2 || !(duration > 0.0))
    {
        return;
    }
    const double* first = w->motion;
    const double* last = w->motion + MOVE * (size_t)(n - 1);
    double dt[3] = {last[0] - first[0], last[1] - first[1], last[2] - first[2]};
    double back[4];
    double correction[4];
    Conjugate(last + 3, back);
    Multiply(first + 3, back, correction);
    Normalize(correction);
    static const double kIdentity[4] = {0.0, 0.0, 0.0, 1.0};
    for (uint32_t k = 0; k < n; ++k)
    {
        double* m = w->motion + MOVE * (size_t)k;
        double u = times[k] / duration;
        for (int i = 0; i < 3; ++i)
        {
            m[i] -= u * dt[i];
        }
        double c[4];
        maniSlerpDouble(kIdentity, correction, u, c);
        Multiply(c, m + 3, m + 3);
        Normalize(m + 3);
    }
}

maniResult maniRootMotionTake(const maniRootMotionSource* source, const double* locals,
                              const double* rest, const double* times, uint32_t n,
                              const maniAllocator* allocator, maniRootMotionWork* out)
{
    *out = (maniRootMotionWork){*source, NULL, n};
    out->motion = maniAllocate(allocator, (size_t)n * MOVE, sizeof(double), 8);
    if (out->motion == NULL)
    {
        return mani_errorCapacity;
    }
    double tRef[3];
    double rRef[4];
    References(source, locals, rest, tRef, rRef);
    for (uint32_t k = 0; k < n; ++k)
    {
        const double* local = locals + MOVE * (size_t)k;
        double* m = out->motion + MOVE * (size_t)k;
        // Signs are left as they come: moves nlerp into one hemisphere,
        // the loop's correction slerps by the shorter arc, and the clip
        // builder keeps the stored bone continuous.
        Motion(source, local, local + 3, tRef, rRef, m);
    }
    if (out->source.loop != 0)
    {
        Loop(out, times);
    }
    return mani_success;
}

void maniRootMotionBake(const maniRootMotionWork* work, uint32_t k, double t[3], double r[4])
{
    const double* m = work->motion + MOVE * (size_t)k;
    double back[4];
    Conjugate(m + 3, back);
    double d[3] = {t[0] - m[0], t[1] - m[1], t[2] - m[2]};
    Rotate(back, d, t);
    Multiply(back, r, r);
    Normalize(r);
}

static void StoreFloat(uint8_t* p, double value)
{
    float f = (float)value;
    memcpy(p, &f, sizeof f);
}

static bool InReach(const double t[3])
{
    return fabs(t[0]) <= MANI_ROOT_MOTION_REACH && fabs(t[1]) <= MANI_ROOT_MOTION_REACH &&
           fabs(t[2]) <= MANI_ROOT_MOTION_REACH;
}

bool maniRootMotionWrite(const maniRootMotionWork* work, uint8_t* out)
{
    const maniRootMotionSource* s = &work->source;
    const uint32_t fields[8] = {s->bone,
                                s->translationAxes,
                                s->rotation,
                                s->twistAxis,
                                s->translationReference,
                                s->rotationReference,
                                s->loop,
                                0};
    for (uint32_t i = 0; i < 8; ++i)
    {
        maniStoreU32(out + 4 * i, fields[i]);
    }
    // The whole period's move, in the frame where it begins: m_r0^-1
    // (m_tn - m_t0) and m_r0^-1 m_rn.
    uint32_t n = work->sampleCount;
    const double* first = work->motion;
    const double* last = work->motion + MOVE * (size_t)(n - 1);
    double back[4];
    Conjugate(first + 3, back);
    double period[MOVE];
    const double d[3] = {last[0] - first[0], last[1] - first[1], last[2] - first[2]};
    Rotate(back, d, period);
    Multiply(back, last + 3, period + 3);
    Normalize(period + 3);
    bool reach = InReach(period);
    uint8_t* p = out + MANI_ROOT_MOTION_HEADER_SIZE;
    for (uint32_t i = 0; i < MOVE; ++i)
    {
        StoreFloat(p + 4 * i, period[i]);
    }
    maniStoreU32(p + 4 * MOVE, 0);
    p += MANI_ROOT_MOTION_PERIOD_SIZE;
    for (size_t i = 0; i < (size_t)n * MOVE; ++i)
    {
        StoreFloat(p + 4 * i, work->motion[i]);
        reach = reach && (i % MOVE != 0 || InReach(work->motion + i));
    }
    return reach;
}

void maniRootMotionRelease(maniRootMotionWork* work, const maniAllocator* allocator)
{
    maniRelease(allocator, work->motion, (size_t)work->sampleCount * MOVE, sizeof(double), 8);
    work->motion = NULL;
}
