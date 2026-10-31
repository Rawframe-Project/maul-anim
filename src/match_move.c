// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clamped span holds a time within its ends. A looping span wraps it:
// the root's move runs to the span's end, then whole spans (composed by
// doubling, at most 2^32 of them, as maniClip_GetRootMotion bounds its
// periods), then from the start to what is left.

#include "match_move.h"

#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/rootmotion.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static maniRootMove Move(const maniClip* clip, float from, float step)
{
    maniRootMove move = maniMakeRootMoveIdentity();
    return maniClip_GetRootMotion(clip, from, step, &move) == mani_success
               ? move
               : maniMakeRootMoveIdentity();
}

// k whole spans of move w, by doubling.
static maniRootMove Repeat(maniRootMove w, uint64_t k)
{
    maniRootMove r = maniMakeRootMoveIdentity();
    while (k > 0)
    {
        r = (k & 1u) != 0 ? maniComposeRootMotion(r, w) : r;
        w = maniComposeRootMotion(w, w);
        k >>= 1;
    }
    return r;
}

// Whole spans counted, at most 2^32 (a root motion's bound, as
// maniClip_GetRootMotion's).
static uint64_t Count(double whole)
{
    return whole < 4294967296.0 ? (uint64_t)whole : 4294967296u;
}

// What is left past whole spans, kept within the span against rounding
// when the step dwarfs it.
static double Within(double last, double span)
{
    return last < 0.0 ? 0.0 : (last < span ? last : 0.0);
}

// Forward from t by step (>= 0) in a looping span.
static maniMatchPlace Ahead(const maniClip* clip, maniMatchSpan e, float t, double step)
{
    double span = (double)e.end - (double)e.start;
    double toEnd = (double)e.end - (double)t;
    if (step < toEnd)
    {
        return (maniMatchPlace){(float)((double)t + step), 0, 0.0, Move(clip, t, (float)step)};
    }
    double rest = step - toEnd;
    double whole = floor(rest / span);
    double last = Within(rest - whole * span, span);
    maniRootMove move = Move(clip, t, (float)toEnd);
    move = maniComposeRootMotion(move, Repeat(Move(clip, e.start, (float)span), Count(whole)));
    move = maniComposeRootMotion(move, Move(clip, e.start, (float)last));
    return (maniMatchPlace){(float)((double)e.start + last), 0, whole + 1.0, move};
}

// Backward from t by step (<= 0) in a looping span.
static maniMatchPlace Behind(const maniClip* clip, maniMatchSpan e, float t, double step)
{
    double span = (double)e.end - (double)e.start;
    double toStart = (double)e.start - (double)t;
    if (step >= toStart)
    {
        return (maniMatchPlace){(float)((double)t + step), 0, 0.0, Move(clip, t, (float)step)};
    }
    double rest = toStart - step;
    double whole = floor(rest / span);
    double last = Within(rest - whole * span, span);
    maniRootMove move = Move(clip, t, (float)toStart);
    move = maniComposeRootMotion(move, Repeat(Move(clip, e.end, (float)-span), Count(whole)));
    move = maniComposeRootMotion(move, Move(clip, e.end, (float)-last));
    return (maniMatchPlace){(float)((double)e.end - last), 0, whole + 1.0, move};
}

maniMatchPlace maniMatchLocate(const maniClip* clip, maniMatchSpan span, float t, double step)
{
    if (span.loop)
    {
        return step >= 0.0 ? Ahead(clip, span, t, step) : Behind(clip, span, t, step);
    }
    double x = (double)t + step;
    float at = (float)(x < (double)span.start ? (double)span.start
                                              : (x > (double)span.end ? (double)span.end : x));
    return (maniMatchPlace){at, 0, 0.0, Move(clip, t, at - t)};
}

maniVec3 maniMatchAxisVector(maniMatchAxis axis)
{
    float sign = axis >= maniMatchAxisNegativeX ? -1.0f : 1.0f;
    uint32_t a = axis % 3;
    return (maniVec3){a == 0 ? sign : 0.0f, a == 1 ? sign : 0.0f, a == 2 ? sign : 0.0f};
}

void maniMatchGround(maniVec3 v, maniMatchAxis up, float* out)
{
    const float c[3] = {v.x, v.y, v.z};
    uint32_t a = up % 3;
    out[0] = c[a == 0 ? 1 : 0];
    out[1] = c[a == 2 ? 1 : 2];
}
