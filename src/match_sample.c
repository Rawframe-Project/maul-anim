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
// projected the same way (not renormalized, as Holden's). A mirrored
// entry's poses are mirrored and its moves reflected through the
// source's mirror map, as a mirrored clip's would be.

#include "match_sample.h"

#include "match_file.h"
#include "vecmath.h"

#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static bool Mirrored(const maniMatchEntrySource* e)
{
    return (e->flags & maniMatchEntryMirrored) != 0;
}

static maniMatchPlace Locate(const maniMatchSampler* s, const maniClip* clip,
                             const maniMatchEntrySource* e, float t, double step)
{
    maniMatchSpan span = {e->start, e->end, (e->flags & maniMatchEntryLoop) != 0};
    maniMatchPlace p = maniMatchLocate(clip, span, t, step);
    p.move = Mirrored(e) ? maniMirrorMap_MirrorRootMotion(s->source->mirrorMap, p.move) : p.move;
    return p;
}

static bool Model(maniMatchSampler* s, const maniClip* clip, const maniMatchEntrySource* e,
                  float time, uint32_t bone, maniTransform* out)
{
    const maniSkeleton* skeleton = s->source->skeleton;
    uint32_t a = 0;
    uint32_t b = 0;
    memcpy(&a, &s->sampledTime, sizeof a);
    memcpy(&b, &time, sizeof b);
    bool mirrored = Mirrored(e);
    bool sampled = s->sampledClip == clip && a == b && s->sampledMirrored == mirrored;
    if (!sampled && (maniClip_Sample(clip, skeleton, time, s->pose) != mani_success ||
                     (mirrored && maniMirrorMap_MirrorPose(s->source->mirrorMap, skeleton, s->pose,
                                                           s->pose) != mani_success) ||
                     maniPose_ComputeModel(s->pose, skeleton) != mani_success))
    {
        s->sampledClip = NULL;
        return false;
    }
    s->sampledClip = clip;
    s->sampledTime = time;
    s->sampledMirrored = mirrored;
    *out = maniPose_GetModelTransform(s->pose, bone);
    return true;
}

static bool Position(maniMatchSampler* s, const maniClip* clip, const maniMatchEntrySource* e,
                     maniMatchPlace p, uint32_t bone, maniVec3* out)
{
    maniTransform m;
    if (!Model(s, clip, e, p.time, bone, &m))
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
    maniMatchPlace before = Locate(s, clip, e, t, step - h);
    maniMatchPlace after = Locate(s, clip, e, t, step + h);
    maniVec3 a;
    maniVec3 b;
    if (!Position(s, clip, e, before, bone, &a) || !Position(s, clip, e, after, bone, &b))
    {
        return false;
    }
    // A looping entry's times are a frame apart either way; a clamped
    // one's as far as its ends let them be.
    bool loop = (e->flags & maniMatchEntryLoop) != 0;
    double apart = loop ? 2.0 * h : (double)after.time - (double)before.time;
    *out = apart > 0.0 ? maniVec3Scale(maniVec3Sub(b, a), (float)(1.0 / apart))
                       : (maniVec3){0.0f, 0.0f, 0.0f};
    return true;
}

static bool Facing(maniMatchSampler* s, const maniClip* clip, const maniMatchEntrySource* e,
                   maniMatchPlace p, const maniMatchChannelSource* c, maniVec3* out)
{
    maniTransform m;
    if (!Model(s, clip, e, p.time, c->bone, &m))
    {
        return false;
    }
    *out =
        maniQuatRotate(p.move.rotation, maniQuatRotate(m.rotation, maniMatchAxisVector(c->axis)));
    return true;
}

static bool Float(maniMatchSampler* s, const maniClip* clip, float time, uint32_t track, float* out)
{
    uint32_t count = 0;
    if (maniClip_SampleFloats(clip, time, s->floats, s->floatCapacity, &count) != mani_success)
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
    maniMatchPlace p = Locate(s, clip, entry, t, (double)time);
    maniVec3 v = {0.0f, 0.0f, 0.0f};
    bool ok = true;
    switch (ch->kind)
    {
    case maniMatchBonePosition:
        ok = Position(s, clip, entry, p, ch->bone, &v);
        break;
    case maniMatchBoneVelocity:
        ok = Velocity(s, clip, entry, t, (double)time, ch->bone, &v);
        break;
    case maniMatchBoneFacing:
        ok = Facing(s, clip, entry, p, ch, &v);
        break;
    case maniMatchTrajectoryPosition:
        maniMatchGround(p.move.translation, src->up, out);
        return true;
    case maniMatchTrajectoryFacing:
        maniMatchGround(maniQuatRotate(p.move.rotation, maniMatchAxisVector(src->forward)), src->up,
                        out);
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
