// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Holden's dead blending in the closed form of his Unreal node. Per bone
// and axis a speed: translation's from the last two outputs, rotation's
// the scaled axis-angle of the turn between them on the shorter arc,
// scale's in log space (so the extrapolation stays positive; an axis not
// positive in either takes none). Per axis a half-life clamp(scale d /
// v, min, max), d the distance to the input at capture in the same terms
// and v kept at least 1e-8 in magnitude by its sign: an axis moving
// slowly toward the input extrapolates long, one moving fast or away
// decays soon. With y = ln 2 / half-life the extrapolation moves x by
// v (1 - e^(-y t)) / y (v t for y 0, which only a saved state holds);
// it is blended into the input by smoothstep(t / T), lerp and slerp.

#include "graph_dead_blend.h"

#include "elementary_float.h"
#include "graph_file.h"
#include "graph_inertia.h"
#include "pose_memory.h"
#include "scalar.h"
#include "vecmath.h"

#include <stdbool.h>
#include <stdint.h>

#define LN2 0.693147180559945309f

// A capture's words after the bone's history: the last output, its
// speeds, their decay rates.
enum
{
    Origin = 0,
    Speed = 10,
    Decay = 19,
};

// A scale axis's logarithm, or none (0, false) when not positive.
static bool LogScale(float s, float* out)
{
    *out = s > 0.0f ? maniLogF(s) : 0.0f;
    return s > 0.0f;
}

static float Get(const maniVec3* v, uint32_t k)
{
    return k == 0 ? v->x : (k == 1 ? v->y : v->z);
}

// Holden's half-life heuristic, as a decay rate.
static float Rate(const maniGraphInertia* node, float distance, float speed)
{
    float v = speed >= 0.0f ? (speed > 1e-8f ? speed : 1e-8f) : (speed < -1e-8f ? speed : -1e-8f);
    float h = node->halfLifeScale * distance / v;
    h = h > node->halfLifeMin ? h : node->halfLifeMin;
    h = h < node->halfLifeMax ? h : node->halfLifeMax;
    return LN2 / h;
}

// One channel's speeds and decay rates, from the distances to the input.
static void Channel(uint32_t* block, uint32_t at, const maniGraphInertia* node, maniVec3 speed,
                    maniVec3 distance)
{
    for (uint32_t k = 0; k < 3; ++k)
    {
        float v = Get(&speed, k);
        maniGraphInertiaSet(block, at + Speed + k, v);
        maniGraphInertiaSet(block, at + Decay + k, Rate(node, Get(&distance, k), v));
    }
}

// The scale's speeds and distances in log space.
static void Scales(maniVec3 last, maniVec3 before, maniVec3 input, float rate, maniVec3* speed,
                   maniVec3* distance)
{
    const float x[3] = {last.x, last.y, last.z};
    const float p[3] = {before.x, before.y, before.z};
    const float in[3] = {input.x, input.y, input.z};
    float v[3];
    float d[3];
    for (uint32_t k = 0; k < 3; ++k)
    {
        float lx = 0.0f;
        float lp = 0.0f;
        float li = 0.0f;
        bool positive = LogScale(x[k], &lx);
        bool earlier = LogScale(p[k], &lp);
        bool target = LogScale(in[k], &li);
        v[k] = positive && earlier ? (lx - lp) * rate : 0.0f;
        d[k] = positive && target ? li - lx : 0.0f;
    }
    *speed = (maniVec3){v[0], v[1], v[2]};
    *distance = (maniVec3){d[0], d[1], d[2]};
}

void maniGraphDeadCapture(uint32_t* block, const maniGraphInertia* node, float* locals, bool moving,
                          float dt)
{
    // 1 / dt, or 0 for no speed.
    float rate = moving && dt > 0.0f ? 1.0f / dt : 0.0f;
    uint32_t lastAt = maniGraphInertiaHistory(block, node->bones, true);
    uint32_t beforeAt = maniGraphInertiaHistory(block, node->bones, false);
    for (uint32_t b = 0; b < node->bones; ++b)
    {
        maniGraphTrs in = maniGraphTrsRead(maniPoseSlot(locals, b));
        maniGraphTrs x = maniGraphTrsHeld(block, lastAt, b);
        // With no speed the rate zeroes every difference from it.
        maniGraphTrs p = maniGraphTrsHeld(block, beforeAt, b);
        uint32_t capture = maniGraphInertiaCapture(node, b);
        maniGraphTrsKeep(block, capture + Origin, x);
        Channel(block, capture, node, maniVec3Scale(maniVec3Sub(x.t, p.t), rate),
                maniVec3Sub(in.t, x.t));
        maniVec3 turn =
            maniVec3Scale(maniQuatToRotationVector(maniQuatMul(x.r, maniQuatConjugate(p.r))), rate);
        Channel(block, capture + 3, node, turn,
                maniQuatToRotationVector(maniQuatMul(in.r, maniQuatConjugate(x.r))));
        maniVec3 speed;
        maniVec3 distance;
        Scales(x.s, p.s, in.s, rate, &speed, &distance);
        Channel(block, capture + 6, node, speed, distance);
    }
}

// How far a speed decaying at rate y carries in t seconds (none for no
// speed, without the exponential: most bones hold their translation and
// scale).
static float Carry(float v, float y, float t)
{
    if (v == 0.0f)
    {
        return 0.0f;
    }
    return y > 0.0f ? v * (1.0f - maniExpF(-y * t)) / y : v * t;
}

// A scale axis grown by e to the carried logarithm.
static float Grow(float s, float carried)
{
    return carried != 0.0f ? s * maniExpF(carried) : s;
}

static maniVec3 Carried(const uint32_t* block, uint32_t at, float t)
{
    float c[3];
    for (uint32_t k = 0; k < 3; ++k)
    {
        c[k] = Carry(maniGraphInertiaGet(block, at + Speed + k),
                     maniGraphInertiaGet(block, at + Decay + k), t);
    }
    return (maniVec3){c[0], c[1], c[2]};
}

static maniVec3 Lerp(maniVec3 a, maniVec3 b, float w)
{
    return maniVec3Add(a, maniVec3Scale(maniVec3Sub(b, a), w));
}

void maniGraphDeadApply(const uint32_t* block, const maniGraphInertia* node, float* locals, float t,
                        float duration)
{
    float u = t / duration;
    float w = u * u * (3.0f - 2.0f * u);
    for (uint32_t b = 0; b < node->bones; ++b)
    {
        uint32_t capture = maniGraphInertiaCapture(node, b);
        float* slot = maniPoseSlot(locals, b);
        maniGraphTrs in = maniGraphTrsRead(slot);
        maniGraphTrs x = maniGraphTrsKept(block, capture + Origin);
        maniVec3 s = Carried(block, capture + 6, t);
        x.t = maniVec3Add(x.t, Carried(block, capture, t));
        x.r = maniQuatMul(maniQuatFromRotationVector(Carried(block, capture + 3, t)), x.r);
        x.s = (maniVec3){Grow(x.s.x, s.x), Grow(x.s.y, s.y), Grow(x.s.z, s.z)};
        maniGraphTrsWrite(slot, (maniGraphTrs){Lerp(x.t, in.t, w), maniQuatSlerp(x.r, in.r, w),
                                               Lerp(x.s, in.s, w)});
    }
}
