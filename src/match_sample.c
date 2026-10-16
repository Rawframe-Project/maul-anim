// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A time x from the frame at t (x = t + a channel's time, unwrapped) is
// placed in its entry: a clamped entry holds it within its ends; a
// looping one wraps it into its span. The root's move from t to x is the
// clip's root motion along the way, through the entry's end and back to
// its start as often as a looping x passes it (whole spans composed by
// doubling). A bone's position at x is its model translation carried by
// that move; its velocity the difference of its positions a frame either
// side over their time apart (clamped times nearer); its facing its axis
// turned by its model rotation and the move. The trajectory is the move
// itself on the ground plane: the two axes besides the up axis, in
// order; a facing is the forward axis turned by the move's rotation,
// projected the same way (not renormalized, as Holden's).

#include "match_sample.h"

#include "match_file.h"
#include "vecmath.h"

#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// A time placed in an entry: the clip time, and the root's move from the
// frame's time to it.
typedef struct Place
{
    float time;
    maniRootMove move;
} Place;

static maniRootMove Move(const maniClip* clip, float from, float step)
{
    maniRootMove move = maniRootMoveIdentity();
    return maniClipRootMotion(clip, from, step, &move) == mani_success ? move
                                                                       : maniRootMoveIdentity();
}

// k whole spans of move w, by doubling.
static maniRootMove Repeat(maniRootMove w, uint64_t k)
{
    maniRootMove r = maniRootMoveIdentity();
    while (k > 0)
    {
        r = (k & 1u) != 0 ? maniComposeRootMotion(r, w) : r;
        w = maniComposeRootMotion(w, w);
        k >>= 1;
    }
    return r;
}

// Forward from t by step (>= 0) in a looping entry.
static Place Ahead(const maniClip* clip, const maniMatchEntrySource* e, float t, double step)
{
    double span = (double)e->end - (double)e->start;
    double toEnd = (double)e->end - (double)t;
    if (step < toEnd)
    {
        return (Place){(float)((double)t + step), Move(clip, t, (float)step)};
    }
    double rest = step - toEnd;
    double whole = floor(rest / span);
    double last = rest - whole * span;
    maniRootMove move = Move(clip, t, (float)toEnd);
    move = maniComposeRootMotion(move, Repeat(Move(clip, e->start, (float)span), (uint64_t)whole));
    move = maniComposeRootMotion(move, Move(clip, e->start, (float)last));
    return (Place){(float)((double)e->start + last), move};
}

// Backward from t by step (<= 0) in a looping entry.
static Place Behind(const maniClip* clip, const maniMatchEntrySource* e, float t, double step)
{
    double span = (double)e->end - (double)e->start;
    double toStart = (double)e->start - (double)t;
    if (step >= toStart)
    {
        return (Place){(float)((double)t + step), Move(clip, t, (float)step)};
    }
    double rest = toStart - step;
    double whole = floor(rest / span);
    double last = rest - whole * span;
    maniRootMove move = Move(clip, t, (float)toStart);
    move = maniComposeRootMotion(move, Repeat(Move(clip, e->end, (float)-span), (uint64_t)whole));
    move = maniComposeRootMotion(move, Move(clip, e->end, (float)-last));
    return (Place){(float)((double)e->end - last), move};
}

static Place Locate(const maniClip* clip, const maniMatchEntrySource* e, float t, double step)
{
    if ((e->flags & MANI_MATCH_LOOP) != 0)
    {
        return step >= 0.0 ? Ahead(clip, e, t, step) : Behind(clip, e, t, step);
    }
    double x = (double)t + step;
    float at = (float)(x < (double)e->start ? (double)e->start
                                            : (x > (double)e->end ? (double)e->end : x));
    return (Place){at, Move(clip, t, at - t)};
}

static maniVec3 Axis(maniMatchAxis axis)
{
    float sign = axis >= maniMatchAxisNegativeX ? -1.0f : 1.0f;
    uint32_t a = axis % 3;
    return (maniVec3){a == 0 ? sign : 0.0f, a == 1 ? sign : 0.0f, a == 2 ? sign : 0.0f};
}

// A vector on the ground plane: its two components besides the up axis.
static void Ground(maniVec3 v, maniMatchAxis up, float* out)
{
    const float c[3] = {v.x, v.y, v.z};
    uint32_t a = up % 3;
    out[0] = c[a == 0 ? 1 : 0];
    out[1] = c[a == 2 ? 1 : 2];
}

static bool Model(maniMatchSampler* s, const maniClip* clip, float time, uint32_t bone,
                  maniTransform* out)
{
    const maniSkeleton* skeleton = s->source->skeleton;
    uint32_t a = 0;
    uint32_t b = 0;
    memcpy(&a, &s->sampledTime, sizeof a);
    memcpy(&b, &time, sizeof b);
    bool sampled = s->sampledClip == clip && a == b;
    if (!sampled && (maniSampleClip(clip, skeleton, time, s->pose) != mani_success ||
                     maniComputeModel(s->pose, skeleton) != mani_success))
    {
        s->sampledClip = NULL;
        return false;
    }
    s->sampledClip = clip;
    s->sampledTime = time;
    *out = maniPoseModelTransform(s->pose, bone);
    return true;
}

static bool Position(maniMatchSampler* s, const maniClip* clip, Place p, uint32_t bone,
                     maniVec3* out)
{
    maniTransform m;
    if (!Model(s, clip, p.time, bone, &m))
    {
        return false;
    }
    *out = maniVec3Add(maniQuatRotate(p.move.rotation, m.translation), p.move.translation);
    return true;
}

static bool Velocity(maniMatchSampler* s, const maniClip* clip, const maniMatchEntrySource* e,
                     float t, double step, uint32_t bone, maniVec3* out)
{
    double h = 1.0 / (double)s->source->sampleRate;
    Place before = Locate(clip, e, t, step - h);
    Place after = Locate(clip, e, t, step + h);
    maniVec3 a;
    maniVec3 b;
    if (!Position(s, clip, before, bone, &a) || !Position(s, clip, after, bone, &b))
    {
        return false;
    }
    // A looping entry's times are a frame apart either way; a clamped
    // one's as far as its ends let them be.
    bool loop = (e->flags & MANI_MATCH_LOOP) != 0;
    double apart = loop ? 2.0 * h : (double)after.time - (double)before.time;
    *out = apart > 0.0 ? maniVec3Scale(maniVec3Sub(b, a), (float)(1.0 / apart))
                       : (maniVec3){0.0f, 0.0f, 0.0f};
    return true;
}

static bool Facing(maniMatchSampler* s, const maniClip* clip, Place p,
                   const maniMatchChannelSource* c, maniVec3* out)
{
    maniTransform m;
    if (!Model(s, clip, p.time, c->bone, &m))
    {
        return false;
    }
    *out = maniQuatRotate(p.move.rotation, maniQuatRotate(m.rotation, Axis(c->axis)));
    return true;
}

static bool Float(maniMatchSampler* s, const maniClip* clip, float time, uint32_t track, float* out)
{
    uint32_t count = 0;
    if (maniSampleClipFloats(clip, time, s->floats, s->floatCapacity, &count) != mani_success)
    {
        return false;
    }
    *out = s->floats[track];
    return true;
}

// One channel at one time: its features into out.
static bool Channel(maniMatchSampler* s, uint32_t e, uint32_t c, float t, float time, float* out)
{
    const maniMotionDatabaseSource* src = s->source;
    const maniMatchEntrySource* entry = &src->entries[e];
    const maniMatchChannelSource* ch = &src->channels[c];
    const maniClip* clip = src->clips[entry->clip];
    Place p = Locate(clip, entry, t, (double)time);
    maniVec3 v = {0.0f, 0.0f, 0.0f};
    bool ok = true;
    switch (ch->kind)
    {
    case maniMatchBonePosition:
        ok = Position(s, clip, p, ch->bone, &v);
        break;
    case maniMatchBoneVelocity:
        ok = Velocity(s, clip, entry, t, (double)time, ch->bone, &v);
        break;
    case maniMatchBoneFacing:
        ok = Facing(s, clip, p, ch, &v);
        break;
    case maniMatchTrajectoryPosition:
        Ground(p.move.translation, src->up, out);
        return true;
    case maniMatchTrajectoryFacing:
        Ground(maniQuatRotate(p.move.rotation, Axis(src->forward)), src->up, out);
        return true;
    default:
        return Float(s, clip, p.time, s->tracks[(size_t)c * src->entryCount + e], out);
    }
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
    return ok;
}

bool maniMatchSampleFrame(maniMatchSampler* sampler, uint32_t e, float t, float* out)
{
    const maniMotionDatabaseSource* src = sampler->source;
    bool ok = true;
    for (uint32_t c = 0; c < src->channelCount && ok; ++c)
    {
        const maniMatchChannelSource* ch = &src->channels[c];
        uint32_t width = maniMatchChannelWidth(ch->kind);
        for (uint32_t k = 0; k < ch->timeCount && ok; ++k)
        {
            ok = Channel(sampler, e, c, t, ch->times[k], out);
            out += width;
        }
    }
    return ok;
}
