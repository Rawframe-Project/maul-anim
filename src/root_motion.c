// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clip's root motion: its section checked at open, its
// source read back, and its moves: between two playhead times, across
// wraps by the whole period's move composed by squaring, composed and
// blended, in binary32.

#include "root_motion.h"

#include "bytes.h"
#include "clip_file.h"
#include "clip_view.h"
#include "fpenv.h"
#include "invariant.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// Stored float k after the header: finite, and a translation component
// within MANI_ROOT_MOTION_REACH (seven for the period, then seven per
// sample).
static bool StoredValid(const uint8_t* p, uint64_t k)
{
    uint32_t bits = maniLoadU32(p);
    if ((bits & 0x7F800000u) == 0x7F800000u)
    {
        return false;
    }
    uint64_t component = k % MANI_ROOT_MOTION_MOVE_FLOATS;
    return component >= 3 || fabs((double)LoadF32(p)) <= MANI_ROOT_MOTION_REACH;
}

// The source's fields in range, with a channel chosen.
static bool SourceValid(const maniRootMotionSource* s)
{
    return s->translationAxes <= 7u && (uint32_t)s->rotation <= maniRootRotationWhole &&
           s->twistAxis <= 2u && (uint32_t)s->translationReference <= maniRootReferenceOrigin &&
           (uint32_t)s->rotationReference <= maniRootReferenceOrigin && s->loop <= 1u &&
           (s->translationAxes != 0u || s->rotation != maniRootRotationNone);
}

bool maniRootMotionSourceValid(const maniRootMotionSource* source)
{
    return SourceValid(source);
}

static maniRootMotionSource LoadSource(const uint8_t* h)
{
    return (maniRootMotionSource){maniLoadU32(h),      maniLoadU32(h + 4),  maniLoadU32(h + 8),
                                  maniLoadU32(h + 12), maniLoadU32(h + 16), maniLoadU32(h + 20),
                                  maniLoadU32(h + 24)};
}

// Whether the clip's tracks hold the bone's translation and rotation.
static bool BoneTracked(const maniClip* clip, uint32_t bone)
{
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        const uint8_t* t = clip->tracks + (size_t)i * MANI_CLIP_TRACK_SIZE;
        uint32_t mask = maniChannelTranslation | maniChannelRotation;
        if (maniLoadU16(t) == bone)
        {
            return (t[2] & mask) == mask;
        }
    }
    return false;
}

MANI_COLD maniFileResult maniCheckClipRootMotion(const uint8_t* data, uint64_t size, uint64_t at,
                                                 uint64_t entry, maniClip* clip)
{
    clip->rootMotion = NULL;
    if (size == 0)
    {
        return (maniFileResult){mani_success, 0, 0};
    }
    if (size != maniRootMotionSize(clip->sampleCount))
    {
        return Fail(entry);
    }
    maniRootMotionSource s = LoadSource(data);
    if (!SourceValid(&s) || clip->additive != maniAdditiveNone)
    {
        return Fail(at);
    }
    if (s.bone >= clip->skeletonBoneCount || !BoneTracked(clip, s.bone))
    {
        return Fail(at);
    }
    uint64_t floats = (size - MANI_ROOT_MOTION_HEADER_SIZE) / 4;
    for (uint64_t k = 0; k < floats; ++k)
    {
        if (!StoredValid(data + MANI_ROOT_MOTION_HEADER_SIZE + 4 * k, k))
        {
            return Fail(at + MANI_ROOT_MOTION_HEADER_SIZE + 4 * k);
        }
    }
    clip->rootMotion = data;
    return (maniFileResult){mani_success, 0, 0};
}

bool maniClip_GetRootMotionSource(const maniClip* clip, maniRootMotionSource* sourceOut)
{
    if (sourceOut != NULL)
    {
        memset(sourceOut, 0, sizeof *sourceOut);
    }
    if (clip == NULL || clip->rootMotion == NULL || sourceOut == NULL)
    {
        return false;
    }
    *sourceOut = LoadSource(clip->rootMotion);
    return true;
}

maniRootMove maniMakeRootMoveIdentity(void)
{
    return (maniRootMove){{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
}

// first, then then: inlined where this file composes moves.
static maniRootMove Compose(maniRootMove first, maniRootMove then)
{
    return (maniRootMove){
        maniVec3Add(first.translation, maniQuatRotate(first.rotation, then.translation)),
        maniQuatNormalize(maniQuatMul(first.rotation, then.rotation))};
}

static MANI_NOINLINE maniRootMove ComposeRootMotionWork(maniRootMove first, maniRootMove then)
{
    return Compose(first, then);
}

maniRootMove maniComposeRootMotion(maniRootMove first, maniRootMove then)
{
    uint64_t env = maniFloatEnvEnter();
    maniRootMove result = ComposeRootMotionWork(first, then);
    maniFloatEnvLeave(env);
    return result;
}

static maniRootMove Inverse(maniRootMove m)
{
    maniQuat back = maniQuatConjugate(m.rotation);
    return (maniRootMove){maniVec3Scale(maniQuatRotate(back, m.translation), -1.0f), back};
}

// A stored move: the period's (k UINT32_MAX) or sample k's.
static maniRootMove Stored(const uint8_t* section, uint32_t k)
{
    const uint8_t* p = section + MANI_ROOT_MOTION_HEADER_SIZE +
                       (k == UINT32_MAX ? 0
                                        : MANI_ROOT_MOTION_PERIOD_SIZE +
                                              4 * (size_t)MANI_ROOT_MOTION_MOVE_FLOATS * k);
    float v[MANI_ROOT_MOTION_MOVE_FLOATS];
    for (uint32_t i = 0; i < MANI_ROOT_MOTION_MOVE_FLOATS; ++i)
    {
        v[i] = LoadF32(p + 4 * i);
    }
    return (maniRootMove){{v[0], v[1], v[2]},
                          maniQuatNormalize((maniQuat){v[3], v[4], v[5], v[6]})};
}

// Where the motion has the character at a time of the clip: between the
// two samples around it, translation lerped and rotation nlerped.
static maniRootMove At(const maniClip* clip, float time)
{
    maniClipPair pair = maniClipLocate(clip, time);
    maniRootMove a = Stored(clip->rootMotion, pair.first);
    maniRootMove b = Stored(clip->rootMotion, pair.second);
    maniVec3 t = maniVec3Add(a.translation,
                             maniVec3Scale(maniVec3Sub(b.translation, a.translation), pair.alpha));
    return (maniRootMove){t, maniQuatNlerp(a.rotation, b.rotation, pair.alpha)};
}

// The move from a to b, in a's frame.
static maniRootMove Between(maniRootMove a, maniRootMove b)
{
    return Compose(Inverse(a), b);
}

// A move made count times over, by squaring.
static maniRootMove Repeated(maniRootMove move, uint64_t count)
{
    maniRootMove made = maniMakeRootMoveIdentity();
    while (count != 0)
    {
        if ((count & 1u) != 0)
        {
            made = Compose(made, move);
        }
        move = Compose(move, move);
        count >>= 1u;
    }
    return made;
}

// The wraps between two whole period indices, bounded at 2^32.
static uint64_t Wraps(double from, double to)
{
    double d = fabs(to - from);
    return d < 4294967296.0 ? (uint64_t)d : 4294967296u;
}

// A repeating clip's move between unwrapped times u0 and u1.
static maniRootMove Repeating(const maniClip* clip, double u0, double u1)
{
    double d = (double)clip->duration;
    double w0 = floor(u0 / d);
    double w1 = floor(u1 / d);
    float f0 = (float)(u0 - w0 * d);
    float f1 = (float)(u1 - w1 * d);
    if (w0 == w1)
    {
        return Between(At(clip, f0), At(clip, f1));
    }
    maniRootMove start = Stored(clip->rootMotion, 0);
    maniRootMove end = Stored(clip->rootMotion, clip->sampleCount - 1);
    maniRootMove period = Stored(clip->rootMotion, UINT32_MAX);
    bool forward = w1 > w0;
    maniRootMove out = Between(At(clip, f0), forward ? end : start);
    out = Compose(out, Repeated(forward ? period : Inverse(period), Wraps(w0, w1) - 1));
    return Compose(out, Between(forward ? start : end, At(clip, f1)));
}

MANI_NOINLINE maniResult maniClipRootMotionEntered(const maniClip* clip, float time, float step,
                                                   maniRootMove* moveOut)
{
    if (clip == NULL || moveOut == NULL || !isfinite(time) || !isfinite(step))
    {
        return mani_errorInvalid;
    }
    if (clip->rootMotion == NULL)
    {
        *moveOut = maniMakeRootMoveIdentity();
        return mani_success;
    }
    double u0 = (double)time;
    double u1 = u0 + (double)step;
    if (clip->loop != maniLoopRepeat || !(clip->duration > 0.0f))
    {
        double d = (double)clip->duration;
        float a = (float)(u0 < 0.0 ? 0.0 : (u0 > d ? d : u0));
        float b = (float)(u1 < 0.0 ? 0.0 : (u1 > d ? d : u1));
        *moveOut = Between(At(clip, a), At(clip, b));
        return mani_success;
    }
    *moveOut = Repeating(clip, u0, u1);
    return mani_success;
}

maniResult maniClip_GetRootMotion(const maniClip* clip, float time, float step,
                                  maniRootMove* moveOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = maniClipRootMotionEntered(clip, time, step, moveOut);
    maniFloatEnvLeave(env);
    return result;
}

static bool MoveFinite(const maniRootMove* m)
{
    return isfinite(m->translation.x) && isfinite(m->translation.y) && isfinite(m->translation.z) &&
           isfinite(m->rotation.x) && isfinite(m->rotation.y) && isfinite(m->rotation.z) &&
           isfinite(m->rotation.w);
}

void maniRootMotionSumAdd(maniRootMotionSum* sum, const maniRootMove* move, double weight)
{
    if (!sum->led)
    {
        sum->lead = move->rotation;
        sum->led = true;
    }
    double sw = maniQuatDot(sum->lead, move->rotation) < 0.0f ? -weight : weight;
    sum->total += weight;
    sum->translation[0] += weight * (double)move->translation.x;
    sum->translation[1] += weight * (double)move->translation.y;
    sum->translation[2] += weight * (double)move->translation.z;
    sum->rotation[0] += sw * (double)move->rotation.x;
    sum->rotation[1] += sw * (double)move->rotation.y;
    sum->rotation[2] += sw * (double)move->rotation.z;
    sum->rotation[3] += sw * (double)move->rotation.w;
}

maniRootMove maniRootMotionSumResult(const maniRootMotionSum* sum)
{
    double total = sum->total;
    if (!(total > 0.0))
    {
        return maniMakeRootMoveIdentity();
    }
    const double* t = sum->translation;
    const double* q = sum->rotation;
    return (maniRootMove){
        {(float)(t[0] / total), (float)(t[1] / total), (float)(t[2] / total)},
        maniQuatNormalize((maniQuat){(float)q[0], (float)q[1], (float)q[2], (float)q[3]})};
}

static MANI_NOINLINE maniResult BlendRootMotionWork(const maniRootMove* moves, const float* weights,
                                                    uint32_t count, maniRootMove* moveOut)
{
    if (moveOut == NULL || (count > 0 && (moves == NULL || weights == NULL)))
    {
        return mani_errorInvalid;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!isfinite(weights[i]) || weights[i] < 0.0f || !MoveFinite(&moves[i]))
        {
            return mani_errorInvalid;
        }
    }
    maniRootMotionSum sum = {0};
    for (uint32_t i = 0; i < count; ++i)
    {
        if (weights[i] != 0.0f)
        {
            maniRootMotionSumAdd(&sum, &moves[i], (double)weights[i]);
        }
    }
    *moveOut = maniRootMotionSumResult(&sum);
    return mani_success;
}

maniResult maniBlendRootMotion(const maniRootMove* moves, const float* weights, uint32_t count,
                               maniRootMove* moveOut)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = BlendRootMotionWork(moves, weights, count, moveOut);
    maniFloatEnvLeave(env);
    return result;
}

static MANI_NOINLINE maniResult RestoreRootMotionWork(const maniClip* clip,
                                                      const maniSkeleton* skeleton, float time,
                                                      maniPose* pose)
{
    if (!maniClipFits(clip, skeleton, pose) || maniPose_IsDifference(pose))
    {
        return mani_errorInvalid;
    }
    if (clip->rootMotion == NULL)
    {
        return mani_success;
    }
    uint32_t bone = maniLoadU32(clip->rootMotion);
    maniRootMove m = At(clip, time);
    maniTransform local = maniPose_GetLocal(pose, bone);
    local.translation = maniVec3Add(m.translation, maniQuatRotate(m.rotation, local.translation));
    local.rotation = maniQuatNormalize(maniQuatMul(m.rotation, local.rotation));
    maniPose_SetLocal(pose, bone, local);
    return mani_success;
}

maniResult maniClip_RestoreRootMotion(const maniClip* clip, const maniSkeleton* skeleton,
                                      float time, maniPose* pose)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = RestoreRootMotionWork(clip, skeleton, time, pose);
    maniFloatEnvLeave(env);
    return result;
}
