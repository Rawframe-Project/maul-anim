// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Google's wrap and canonicalized normal octahedron decoding transforms
// (d429de8, mani-0021), in its 32-bit wrapping arithmetic.

#include "gltf_draco_transform.h"

#include "gltf_draco_read.h"

#include <stdbool.h>
#include <stdint.h>

static int32_t Signed(uint32_t u)
{
    return u <= INT32_MAX ? (int32_t)u : -(int32_t)(~u) - 1;
}

static int32_t Clamp(int32_t v, int32_t lo, int32_t hi)
{
    return v > hi ? hi : (v < lo ? lo : v);
}

// The wrap transform: the prediction clamped into [min, max], the
// correction added and wrapped back into it.
static void Wrap(const maniDracoTransform* v, const uint32_t* pred, const uint32_t* corr,
                 uint32_t* out)
{
    int64_t span = (int64_t)v->wrapMax - v->wrapMin + 1;
    for (uint32_t c = 0; c < v->components; ++c)
    {
        int64_t x = (int64_t)Clamp(Signed(pred[c]), v->wrapMin, v->wrapMax) + Signed(corr[c]);
        x = x > v->wrapMax ? x - span : (x < v->wrapMin ? x + span : x);
        out[c] = (uint32_t)(uint64_t)x;
    }
}

static uint32_t Abs32(int32_t v)
{
    return v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
}

// Octahedral coordinates centered on the origin folded across the
// diamond's edge, in 32-bit wrapping arithmetic (Google's).
static void InvertDiamond(int32_t center, int32_t* s, int32_t* t)
{
    int32_t ss = *s >= 0 && *t >= 0 ? 1 : (*s <= 0 && *t <= 0 ? -1 : (*s > 0 ? 1 : -1));
    int32_t st = *s >= 0 && *t >= 0 ? 1 : (*s <= 0 && *t <= 0 ? -1 : (*t > 0 ? 1 : -1));
    uint32_t cs = (uint32_t)(ss * center);
    uint32_t ct = (uint32_t)(st * center);
    uint32_t us = (uint32_t)*s + (uint32_t)*s - cs;
    uint32_t ut = (uint32_t)*t + (uint32_t)*t - ct;
    if (ss * st >= 0)
    {
        uint32_t temp = us;
        us = 0u - ut;
        ut = 0u - temp;
    }
    else
    {
        uint32_t temp = us;
        us = ut;
        ut = temp;
    }
    *s = Signed(us + cs) / 2;
    *t = Signed(ut + ct) / 2;
}

static void Rotate(int32_t count, int32_t* s, int32_t* t)
{
    int32_t a = *s;
    int32_t b = *t;
    *s = count == 1 ? b
                    : (count == 2 ? Signed(0u - (uint32_t)a)
                                  : (count == 3 ? Signed(0u - (uint32_t)b) : a));
    *t = count == 1 ? Signed(0u - (uint32_t)a)
                    : (count == 2 ? Signed(0u - (uint32_t)b) : (count == 3 ? a : b));
}

static int32_t Rotation(int32_t s, int32_t t)
{
    if (s == 0)
    {
        return t == 0 ? 0 : (t > 0 ? 3 : 1);
    }
    if (s > 0)
    {
        return t >= 0 ? 2 : 1;
    }
    return t <= 0 ? 0 : 3;
}

static int32_t ModMax(int32_t x, int32_t center, int32_t max)
{
    return x > center ? Signed((uint32_t)x - (uint32_t)max)
                      : (x < -center ? Signed((uint32_t)x + (uint32_t)max) : x);
}

// The canonicalized octahedral transform on a normal's two coordinates.
static void Octahedron(const maniDracoTransform* v, const uint32_t* pred, const uint32_t* corr,
                       uint32_t* out)
{
    int32_t max = v->octahedronMax;
    int32_t center = (max - 1) / 2;
    int32_t s = Signed(pred[0] - (uint32_t)center);
    int32_t t = Signed(pred[1] - (uint32_t)center);
    bool diamond = Abs32(s) + Abs32(t) <= (uint32_t)center;
    if (!diamond)
    {
        InvertDiamond(center, &s, &t);
    }
    bool bottomLeft = (s == 0 && t == 0) || (s < 0 && t <= 0);
    int32_t rotation = Rotation(s, t);
    if (!bottomLeft)
    {
        Rotate(rotation, &s, &t);
    }
    int32_t os = ModMax(Signed((uint32_t)s + corr[0]), center, max);
    int32_t ot = ModMax(Signed((uint32_t)t + corr[1]), center, max);
    if (!bottomLeft)
    {
        Rotate((4 - rotation) % 4, &os, &ot);
    }
    if (!diamond)
    {
        InvertDiamond(center, &os, &ot);
    }
    out[0] = (uint32_t)os + (uint32_t)center;
    out[1] = (uint32_t)ot + (uint32_t)center;
}

void maniDracoTransformApply(const void* transform, const uint32_t* pred, const uint32_t* corr,
                             uint32_t* out)
{
    const maniDracoTransform* v = transform;
    if (v->type == maniDracoWrap)
    {
        Wrap(v, pred, corr, out);
    }
    else
    {
        Octahedron(v, pred, corr, out);
    }
}

bool maniDracoTransformRead(maniDracoReader* r, maniDracoTransform* v)
{
    uint32_t a = 0;
    uint32_t b = 0;
    if (!maniDracoU32(r, &a) || !maniDracoU32(r, &b))
    {
        return false;
    }
    if (v->type == maniDracoWrap)
    {
        v->wrapMin = Signed(a);
        v->wrapMax = Signed(b);
        return v->wrapMin <= v->wrapMax && (int64_t)v->wrapMax - v->wrapMin < INT32_MAX;
    }
    int32_t max = Signed(a);
    uint32_t bits = 0;
    for (uint32_t x = (uint32_t)max; x != 0; x >>= 1)
    {
        ++bits;
    }
    v->octahedronMax = (int32_t)((1u << (bits < 31 ? bits : 0)) - 1u);
    return max > 0 && max % 2 != 0 && bits >= 2 && bits <= 30;
}
