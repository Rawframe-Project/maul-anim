// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Bollo's inertialization (SIGGRAPH 2017 supplemental, GDC 2018). Per
// bone and channel the offset from the input to the last output: a
// vector by its direction and length, a rotation by its axis and angle on
// the shorter arc; its speed from the output before, projected on that
// direction (a rotation's by its twist about that axis), clamped to at
// most 0; its duration cut to -5 x0 / v0 when that is sooner, so the
// curve never passes 0. The quintic is evaluated in the fraction u = t /
// t1 of its duration, which leaves t1 no division to underflow:
// x = x0 + u (w + u (k / 2 + u (-R / 2 + u (Q / 2 - u P / 2)))), w = v0 t1,
// k = max(-8 w - 20 x0, 0) (the initial acceleration of zero jerk at t1,
// times t1^2), P = k + 6 w + 12 x0, Q = 3 k + 16 w + 30 x0, R = 3 k +
// 12 w + 20 x0.

#include "graph_inertia_apply.h"

#include "graph_file.h"
#include "graph_inertia.h"
#include "pose_memory.h"
#include "scalar.h"
#include "vecmath.h"

#include "maul-anim/pose.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

float maniGraphQuintic(float x0, float v0, float t1, float t)
{
    if (!(t < t1) || x0 == 0.0f)
    {
        return 0.0f;
    }
    float u = t / t1;
    float w = v0 * t1;
    float k = -8.0f * w - 20.0f * x0;
    k = k > 0.0f ? k : 0.0f;
    float p = k + 6.0f * w + 12.0f * x0;
    float q = 3.0f * k + 16.0f * w + 30.0f * x0;
    float r = 3.0f * k + 12.0f * w + 20.0f * x0;
    return x0 + u * (w + u * (0.5f * k + u * (-0.5f * r + u * (0.5f * q - u * 0.5f * p))));
}

// Ten components of a bone: translation, rotation, scale.
typedef struct Trs
{
    maniVec3 t;
    maniQuat r;
    maniVec3 s;
} Trs;

static Trs Read(const float* slot)
{
    float v[MANI_POSE_COMPONENTS];
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        v[c] = slot[c * MANI_POSE_GROUP_BONES];
    }
    return (Trs){{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}, {v[7], v[8], v[9]}};
}

static void Write(float* slot, Trs x)
{
    const float v[MANI_POSE_COMPONENTS] = {x.t.x, x.t.y, x.t.z, x.r.x, x.r.y,
                                           x.r.z, x.r.w, x.s.x, x.s.y, x.s.z};
    maniPoseSlotPut(slot, 0, v, MANI_POSE_COMPONENTS);
}

static Trs Kept(const uint32_t* block, uint32_t at)
{
    float v[MANI_POSE_COMPONENTS];
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        v[c] = maniGraphInertiaGet(block, at + c);
    }
    return (Trs){{v[0], v[1], v[2]}, {v[3], v[4], v[5], v[6]}, {v[7], v[8], v[9]}};
}

static void Keep(uint32_t* block, uint32_t at, Trs x)
{
    const float v[MANI_POSE_COMPONENTS] = {x.t.x, x.t.y, x.t.z, x.r.x, x.r.y,
                                           x.r.z, x.r.w, x.s.x, x.s.y, x.s.z};
    for (uint32_t c = 0; c < MANI_POSE_COMPONENTS; ++c)
    {
        maniGraphInertiaSet(block, at + c, v[c]);
    }
}

// A channel's capture: direction, magnitude, speed, duration.
typedef struct Channel
{
    maniVec3 direction;
    float x0;
    float v0;
    float t1;
} Channel;

static void Store(uint32_t* block, uint32_t at, Channel c)
{
    const float v[6] = {c.direction.x, c.direction.y, c.direction.z, c.x0, c.v0, c.t1};
    for (uint32_t k = 0; k < 6; ++k)
    {
        maniGraphInertiaSet(block, at + k, v[k]);
    }
}

static Channel Load(const uint32_t* block, uint32_t at)
{
    float v[6];
    for (uint32_t k = 0; k < 6; ++k)
    {
        v[k] = maniGraphInertiaGet(block, at + k);
    }
    return (Channel){{v[0], v[1], v[2]}, v[3], v[4], v[5]};
}

// The speed toward the offset clamped to at most 0 and the duration cut
// so the curve does not pass 0. dt 0: no speed.
static Channel Settle(maniVec3 direction, float x0, float before, float dt, float duration)
{
    float v0 = dt > 0.0f ? (x0 - before) / dt : 0.0f;
    v0 = v0 < 0.0f ? v0 : 0.0f;
    float cut = v0 < 0.0f ? -5.0f * x0 / v0 : duration;
    return (Channel){direction, x0, v0, cut < duration ? cut : duration};
}

// A vector channel: the offset last - input, the one before projected
// on its direction.
static Channel Vector(maniVec3 last, maniVec3 before, maniVec3 input, bool moving, float dt,
                      float duration)
{
    maniVec3 d = maniVec3Sub(last, input);
    float x0 = maniVec3Length(d);
    if (!(x0 > 0.0f))
    {
        return (Channel){{0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f};
    }
    maniVec3 direction = maniVec3Scale(d, 1.0f / x0);
    float projected = maniVec3Dot(maniVec3Sub(before, input), direction);
    return Settle(direction, x0, moving ? projected : x0, dt, duration);
}

static maniQuat Shorter(maniQuat q)
{
    return q.w < 0.0f ? (maniQuat){-q.x, -q.y, -q.z, -q.w} : q;
}

// A rotation channel: the turn last * input^-1 by axis and angle, the
// one before by its twist about that axis.
static Channel Rotation(maniQuat last, maniQuat before, maniQuat input, bool moving, float dt,
                        float duration)
{
    maniQuat inverse = maniQuatConjugate(input);
    maniQuat q = Shorter(maniQuatMul(last, inverse));
    maniVec3 xyz = {q.x, q.y, q.z};
    float s = maniVec3Length(xyz);
    if (!(s > 0.0f))
    {
        return (Channel){{0.0f, 0.0f, 0.0f}, 0.0f, 0.0f, 0.0f};
    }
    maniVec3 axis = maniVec3Scale(xyz, 1.0f / s);
    float x0 = 2.0f * maniAtan2(s, q.w);
    maniQuat p = Shorter(maniQuatMul(before, inverse));
    float twist = 2.0f * maniAtan2(maniVec3Dot(axis, (maniVec3){p.x, p.y, p.z}), p.w);
    return Settle(axis, x0, moving ? twist : x0, dt, duration);
}

static void Capture(uint32_t* block, uint32_t bones, float* locals, bool moving, float dt,
                    float duration)
{
    for (uint32_t b = 0; b < bones; ++b)
    {
        uint32_t at = maniGraphInertiaBone(b);
        Trs in = Read(maniPoseSlot(locals, b));
        Trs last = Kept(block, at + MANI_GRAPH_INERTIA_LAST);
        Trs before = moving ? Kept(block, at + MANI_GRAPH_INERTIA_PREV) : last;
        uint32_t capture = at + MANI_GRAPH_INERTIA_CAPTURE;
        Store(block, capture, Vector(last.t, before.t, in.t, moving, dt, duration));
        Store(block, capture + 6, Rotation(last.r, before.r, in.r, moving, dt, duration));
        Store(block, capture + 12, Vector(last.s, before.s, in.s, moving, dt, duration));
    }
}

static maniVec3 Ease(maniVec3 input, Channel c, float t)
{
    return maniVec3Add(input, maniVec3Scale(c.direction, maniGraphQuintic(c.x0, c.v0, c.t1, t)));
}

static void Apply(const uint32_t* block, uint32_t bones, float* locals, float t)
{
    for (uint32_t b = 0; b < bones; ++b)
    {
        uint32_t capture = maniGraphInertiaBone(b) + MANI_GRAPH_INERTIA_CAPTURE;
        float* slot = maniPoseSlot(locals, b);
        Trs x = Read(slot);
        Channel r = Load(block, capture + 6);
        float angle = maniGraphQuintic(r.x0, r.v0, r.t1, t);
        x.t = Ease(x.t, Load(block, capture), t);
        x.r = angle != 0.0f ? maniQuatMul(maniQuatFromAxisAngle(r.direction, angle), x.r) : x.r;
        x.s = Ease(x.s, Load(block, capture + 12), t);
        Write(slot, x);
    }
}

// The output kept: replacing the last within one update, else the last
// becoming the one before.
static void Remember(uint32_t* block, uint32_t bones, float* locals)
{
    uint32_t flags = block[maniGraphInertiaFlags];
    uint32_t serial = block[maniGraphInertiaSerial];
    bool again =
        (flags & maniGraphInertiaLastKept) != 0 && block[maniGraphInertiaLastSerial] == serial;
    for (uint32_t b = 0; b < bones && !again; ++b)
    {
        uint32_t at = maniGraphInertiaBone(b);
        Keep(block, at + MANI_GRAPH_INERTIA_PREV, Kept(block, at + MANI_GRAPH_INERTIA_LAST));
    }
    if (!again && (flags & maniGraphInertiaLastKept) != 0)
    {
        flags |= maniGraphInertiaPrevKept;
        block[maniGraphInertiaPrevSerial] = block[maniGraphInertiaLastSerial];
    }
    for (uint32_t b = 0; b < bones; ++b)
    {
        Trs x = Read(maniPoseSlot(locals, b));
        Keep(block, maniGraphInertiaBone(b) + MANI_GRAPH_INERTIA_LAST, x);
    }
    block[maniGraphInertiaFlags] = flags | maniGraphInertiaLastKept;
    block[maniGraphInertiaLastSerial] = serial;
    block[maniGraphInertiaLastStep] = block[maniGraphInertiaStep];
}

void maniGraphInertiaEvaluate(uint32_t* block, uint32_t bones, const maniPose* input,
                              maniPose* output)
{
    maniPoseCopy(output, input);
    float* locals = maniPoseLocals(output);
    uint32_t flags = block[maniGraphInertiaFlags];
    float pending = maniGraphInertiaGet(block, maniGraphInertiaPending);
    if (pending < HUGE_VALF && (flags & maniGraphInertiaLastKept) != 0)
    {
        bool moving = (flags & maniGraphInertiaPrevKept) != 0 &&
                      block[maniGraphInertiaPrevSerial] + 1u == block[maniGraphInertiaLastSerial];
        float dt = maniGraphInertiaGet(block, maniGraphInertiaLastStep);
        Capture(block, bones, locals, moving, dt, pending);
        maniGraphInertiaSet(block, maniGraphInertiaElapsed,
                            maniGraphInertiaGet(block, maniGraphInertiaStep));
        maniGraphInertiaSet(block, maniGraphInertiaDuration, pending);
        flags |= maniGraphInertiaActive;
    }
    maniGraphInertiaSet(block, maniGraphInertiaPending, HUGE_VALF);
    float elapsed = maniGraphInertiaGet(block, maniGraphInertiaElapsed);
    flags = elapsed < maniGraphInertiaGet(block, maniGraphInertiaDuration)
                ? flags
                : flags & ~(uint32_t)maniGraphInertiaActive;
    block[maniGraphInertiaFlags] = flags;
    if ((flags & maniGraphInertiaActive) != 0)
    {
        Apply(block, bones, locals, elapsed);
    }
    Remember(block, bones, locals);
}
