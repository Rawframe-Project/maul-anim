// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a full-precision clip file: every channel checked,
// then evaluated at each sample time as glTF's Appendix C defines its
// interpolation, in binary64 with the library's own trigonometry, and
// rounded once. Rotations are normalized and each sample put in the
// hemisphere of the one before, so playback's normalized lerp between
// neighbours takes the short way.

#include "clip_build.h"

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "clip_file.h"
#include "container.h"
#include "fpenv.h"
#include "invariant.h"
#include "quat64.h"
#include "root_motion.h"
#include "root_motion_build.h"

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdckdint.h>
#include <string.h>

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return (maniBuildResult){mani_errorInvalid, reason, item, other};
}

static maniBuildResult Outcome(maniResult result)
{
    return (maniBuildResult){result, maniBuildReasonNone, 0, 0};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static uint32_t KeyFloats(const maniChannelSource* channel, uint32_t dimension)
{
    return channel->interpolation == maniInterpolationCubicSpline ? 3 * dimension : dimension;
}

// A channel's keys: a known interpolation, times from 0 strictly
// increasing, finite values, no zero rotation, two keys for a spline.
maniBuildResult maniCheckChannel(const maniChannelSource* c, uint32_t dimension, uint32_t track,
                                 maniBuildReason list)
{
    if (c->keyCount == 0)
    {
        return Outcome(mani_success);
    }
    if (c->interpolation > maniInterpolationCubicSpline)
    {
        return Refuse(list | maniBuildReasonKind, track, 0);
    }
    if (c->times == NULL || c->values == NULL)
    {
        return Refuse(list | maniBuildReasonCount, track, 0);
    }
    if (c->interpolation == maniInterpolationCubicSpline && c->keyCount < 2)
    {
        return Refuse(list | maniBuildReasonKeyCount, track, 0);
    }
    for (uint32_t k = 0; k < c->keyCount; ++k)
    {
        if (!Finite(c->times[k]) || c->times[k] < 0.0f || (k > 0 && c->times[k] <= c->times[k - 1]))
        {
            return Refuse(list | maniBuildReasonKeyTime, track, 0);
        }
    }
    uint32_t floats = KeyFloats(c, dimension);
    for (uint64_t i = 0; i < (uint64_t)c->keyCount * floats; ++i)
    {
        if (!Finite(c->values[i]))
        {
            return Refuse(list | maniBuildReasonNotFinite, track, 0);
        }
    }
    if (dimension == 4)
    {
        uint32_t at = c->interpolation == maniInterpolationCubicSpline ? 4 : 0;
        for (uint32_t k = 0; k < c->keyCount; ++k)
        {
            const float* q = c->values + (size_t)k * floats + at;
            if (q[0] == 0.0f && q[1] == 0.0f && q[2] == 0.0f && q[3] == 0.0f)
            {
                return Refuse(list | maniBuildReasonRotation, track, 0);
            }
        }
    }
    return Outcome(mani_success);
}

// The channels a track stores: its keyed ones; the root motion bone's
// translation and rotation always, its motion taken out of them.
static uint32_t Mask(const maniClipSource* source, const maniBoneTrackSource* t)
{
    bool root = source->rootMotion != NULL && t->bone == source->rootMotion->bone;
    return (t->translation.keyCount > 0 || root ? maniChannelTranslation : 0u) |
           (t->rotation.keyCount > 0 || root ? maniChannelRotation : 0u) |
           (t->scale.keyCount > 0 ? maniChannelScale : 0u);
}

static float Max(float a, float b)
{
    return a > b ? a : b;
}

static float LastKeyTime(const maniChannelSource* c)
{
    return c->keyCount > 0 ? c->times[c->keyCount - 1] : 0.0f;
}

// The source's tracks: each checked, bones in range and once each (seen
// holds a track's index + 1 per bone); the latest key time.
static maniBuildResult CheckTracks(const maniClipSource* source, uint32_t* seen, float* durationOut)
{
    float last = 0.0f;
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    for (uint32_t i = 0; i < source->boneTrackCount; ++i)
    {
        const maniBoneTrackSource* t = &source->boneTracks[i];
        if (t->bone >= bones)
        {
            return Refuse(maniBuildReasonTrack, i, 0);
        }
        if (seen[t->bone] != 0)
        {
            return Refuse(maniBuildReasonTrack, i, seen[t->bone] - 1);
        }
        seen[t->bone] = i + 1;
        maniBuildResult r = maniCheckChannel(&t->translation, 3, i, 0);
        if (r.result == mani_success)
        {
            r = maniCheckChannel(&t->rotation, 4, i, 0);
        }
        if (r.result == mani_success)
        {
            r = maniCheckChannel(&t->scale, 3, i, 0);
        }
        if (r.result != mani_success)
        {
            return r;
        }
        last = Max(last, Max(LastKeyTime(&t->translation),
                             Max(LastKeyTime(&t->rotation), LastKeyTime(&t->scale))));
    }
    *durationOut = last;
    return Outcome(mani_success);
}

static void NormalizeDouble(double q[4])
{
    double length = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int k = 0; k < 4; ++k)
    {
        q[k] /= length;
    }
}

static void LoadKey(const maniChannelSource* c, uint32_t key, uint32_t part, uint32_t dimension,
                    double out[4])
{
    MANI_ASSERT(key < c->keyCount &&
                part < (c->interpolation == maniInterpolationCubicSpline ? 3u : 1u));
    const float* v = c->values + (size_t)key * KeyFloats(c, dimension) + (size_t)part * dimension;
    for (uint32_t k = 0; k < dimension; ++k)
    {
        out[k] = (double)v[k];
    }
}

// The last key at or before a time strictly inside the keys.
static uint32_t KeyBefore(const maniChannelSource* c, double time)
{
    MANI_ASSERT(c->keyCount >= 2 && (double)c->times[0] < time &&
                time < (double)c->times[c->keyCount - 1]);
    uint32_t low = 0;
    uint32_t high = c->keyCount - 1;
    while (high - low > 1)
    {
        uint32_t middle = low + (high - low) / 2;
        if ((double)c->times[middle] <= time)
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    MANI_ASSERT(low + 1 < c->keyCount && (double)c->times[low] <= time &&
                time < (double)c->times[low + 1]);
    return low;
}

// Between keys low and low + 1, as Appendix C defines each mode.
static void Interpolate(const maniChannelSource* c, uint32_t dimension, uint32_t low, double time,
                        double out[4])
{
    uint32_t valuePart = c->interpolation == maniInterpolationCubicSpline ? 1 : 0;
    double t0 = (double)c->times[low];
    double span = (double)c->times[low + 1] - t0;
    MANI_ASSERT(span > 0.0);
    double u = (time - t0) / span;
    double a[4] = {0, 0, 0, 0};
    double b[4] = {0, 0, 0, 0};
    LoadKey(c, low, valuePart, dimension, a);
    LoadKey(c, low + 1, valuePart, dimension, b);
    if (c->interpolation == maniInterpolationStep || time == t0)
    {
        memcpy(out, a, sizeof a);
    }
    else if (c->interpolation == maniInterpolationLinear && dimension == 4)
    {
        NormalizeDouble(a);
        NormalizeDouble(b);
        maniSlerpDouble(a, b, u, out);
    }
    else if (c->interpolation == maniInterpolationLinear)
    {
        for (uint32_t k = 0; k < dimension; ++k)
        {
            out[k] = (1.0 - u) * a[k] + u * b[k];
        }
    }
    else
    {
        double outTangent[4] = {0, 0, 0, 0};
        double inTangent[4] = {0, 0, 0, 0};
        LoadKey(c, low, 2, dimension, outTangent);
        LoadKey(c, low + 1, 0, dimension, inTangent);
        double u2 = u * u;
        double u3 = u2 * u;
        for (uint32_t k = 0; k < dimension; ++k)
        {
            out[k] = (2 * u3 - 3 * u2 + 1) * a[k] + span * (u3 - 2 * u2 + u) * outTangent[k] +
                     (-2 * u3 + 3 * u2) * b[k] + span * (u3 - u2) * inTangent[k];
        }
    }
}

// A channel's value at a time; times outside the keys take the nearer
// end. Rotations come out normalized.
void maniEvaluateChannel(const maniChannelSource* c, uint32_t dimension, double time, double out[4])
{
    uint32_t valuePart = c->interpolation == maniInterpolationCubicSpline ? 1 : 0;
    uint32_t n = c->keyCount;
    MANI_ASSERT(n > 0 && c->times != NULL && c->values != NULL);
    if (time <= (double)c->times[0] || n == 1)
    {
        LoadKey(c, 0, valuePart, dimension, out);
    }
    else if (time >= (double)c->times[n - 1])
    {
        LoadKey(c, n - 1, valuePart, dimension, out);
    }
    else
    {
        Interpolate(c, dimension, KeyBefore(c, time), time, out);
    }
    if (dimension == 4)
    {
        NormalizeDouble(out);
    }
}

// An additive clip's basis for a track's channel (0 translation, 1
// rotation, 2 scale): the bone's rest, or the channel at time 0.
static void Basis(const maniClipSource* source, const maniBoneTrackSource* t, uint32_t channel,
                  double out[4])
{
    if (source->additive == maniAdditiveFirstFrame)
    {
        const maniChannelSource* c =
            channel == 0 ? &t->translation : (channel == 1 ? &t->rotation : &t->scale);
        maniEvaluateChannel(c, channel == 1 ? 4 : 3, 0.0, out);
        return;
    }
    maniTransform rest = maniSkeletonRest(source->skeleton, t->bone);
    const float r[3][4] = {
        {rest.translation.x, rest.translation.y, rest.translation.z, 0.0f},
        {rest.rotation.x, rest.rotation.y, rest.rotation.z, rest.rotation.w},
        {rest.scale.x, rest.scale.y, rest.scale.z, 0.0f},
    };
    for (int k = 0; k < 4; ++k)
    {
        out[k] = (double)r[channel][k];
    }
}

// A channel's value as its difference from the basis, in binary64:
// t - t_b, r_b^-1 r (normalized), s / s_b.
static void Difference(const maniClipSource* source, const maniBoneTrackSource* t, uint32_t channel,
                       double v[4])
{
    if (source->additive == maniAdditiveNone)
    {
        return;
    }
    double b[4];
    Basis(source, t, channel, b);
    if (channel == 1)
    {
        // The conjugate of b times v.
        double d[4] = {
            b[3] * v[0] - b[0] * v[3] - b[1] * v[2] + b[2] * v[1],
            b[3] * v[1] + b[0] * v[2] - b[1] * v[3] - b[2] * v[0],
            b[3] * v[2] - b[0] * v[1] + b[1] * v[0] - b[2] * v[3],
            b[3] * v[3] + b[0] * v[0] + b[1] * v[1] + b[2] * v[2],
        };
        NormalizeDouble(d);
        memcpy(v, d, sizeof d);
        return;
    }
    for (int k = 0; k < 3; ++k)
    {
        // CheckBasis refused a scale basis with a component 0.
        MANI_ASSERT(channel == 0 || b[k] != 0.0);
        v[k] = channel == 0 ? v[k] - b[k] : v[k] / b[k];
    }
}

// A rotation into out, in the hemisphere of the track's previous one.
static float* PutRotation(const double v[4], float* p, bool first, float* out)
{
    float q[4] = {(float)v[0], (float)v[1], (float)v[2], (float)v[3]};
    float dot = q[0] * p[0] + q[1] * p[1] + q[2] * p[2] + q[3] * p[3];
    float sign = !first && dot < 0.0f ? -1.0f : 1.0f;
    for (int k = 0; k < 4; ++k)
    {
        p[k] = sign * q[k];
        *out++ = p[k];
    }
    return out;
}

// The root motion of a clip being built: the motion, and the bone's local
// translation and rotation at each sample (seven binary64).
typedef struct RootBone
{
    maniRootMotionWork work;
    double* locals;
} RootBone;

// A bone's local translation and rotation at a time: its keys, or its
// rest where a channel has none.
static void Local(const maniClipSource* source, const maniBoneTrackSource* track, double time,
                  double out[7])
{
    maniTransform rest = maniSkeletonRest(source->skeleton, track->bone);
    double v[4];
    if (track->translation.keyCount > 0)
    {
        maniEvaluateChannel(&track->translation, 3, time, v);
    }
    else
    {
        v[0] = (double)rest.translation.x;
        v[1] = (double)rest.translation.y;
        v[2] = (double)rest.translation.z;
    }
    memcpy(out, v, 3 * sizeof(double));
    if (track->rotation.keyCount > 0)
    {
        maniEvaluateChannel(&track->rotation, 4, time, out + 3);
    }
    else
    {
        out[3] = (double)rest.rotation.x;
        out[4] = (double)rest.rotation.y;
        out[5] = (double)rest.rotation.z;
        out[6] = (double)rest.rotation.w;
    }
}

// The root motion bone's translation and rotation at sample k, the
// motion taken out.
static float* PutRootBone(const RootBone* root, uint32_t k, float* p, float* out)
{
    double v[3];
    double r[4];
    MANI_ASSERT(k < root->work.sampleCount);
    memcpy(v, root->locals + 7 * (size_t)k, sizeof v);
    memcpy(r, root->locals + 7 * (size_t)k + 3, sizeof r);
    maniRootMotionBake(&root->work, k, v, r);
    for (int i = 0; i < 3; ++i)
    {
        *out++ = (float)v[i];
    }
    return PutRotation(r, p, k == 0, out);
}

// One sample, k of the grid: every track's animated channels (an
// additive clip's as differences), rotations kept in the hemisphere of
// the track's previous rotation (previous holds four floats a track).
static void WriteSample(const maniClipSource* source, const uint32_t* order, uint32_t tracks,
                        const RootBone* root, uint32_t k, double time, float* previous, float* out)
{
    bool first = k == 0;
    for (uint32_t i = 0; i < tracks; ++i)
    {
        const maniBoneTrackSource* t = &source->boneTracks[order[i]];
        double v[4];
        if (root != NULL && t->bone == root->work.source.bone)
        {
            out = PutRootBone(root, k, previous + 4 * (size_t)i, out);
        }
        else
        {
            if (t->translation.keyCount > 0)
            {
                maniEvaluateChannel(&t->translation, 3, time, v);
                Difference(source, t, 0, v);
                for (int c = 0; c < 3; ++c)
                {
                    *out++ = (float)v[c];
                }
            }
            if (t->rotation.keyCount > 0)
            {
                maniEvaluateChannel(&t->rotation, 4, time, v);
                Difference(source, t, 1, v);
                out = PutRotation(v, previous + 4 * (size_t)i, first, out);
            }
        }
        if (t->scale.keyCount > 0)
        {
            maniEvaluateChannel(&t->scale, 3, time, v);
            Difference(source, t, 2, v);
            for (int c = 0; c < 3; ++c)
            {
                *out++ = (float)v[c];
            }
        }
    }
}

double maniClipSampleTime(uint32_t k, uint32_t n, float rate, float duration)
{
    return k + 1 == n && k > 0 ? (double)duration : (double)k / (double)rate;
}

typedef struct Plan
{
    // NULL without root motion.
    const RootBone* rootMotion;
    const uint8_t* rootMotionBytes;
    uint32_t* order;
    uint32_t tracks;
    uint32_t floatsPerSample;
    uint32_t sampleCount;
    float rate;
    float duration;
} Plan;

// The sections past the samples: the named ones, the compressed ones
// empty, the root motion.
static void LaySections(const Plan* plan, const maniClipNamedLayout* named,
                        const uint8_t** sections, uint64_t* sizes)
{
    static const uint8_t kNone = 0;
    for (uint32_t i = 0; i < 7; ++i)
    {
        sections[3 + i] = named->data[i];
        sizes[3 + i] = named->sizes[i];
    }
    for (uint32_t i = MANI_CLIP_SECTIONS; i < MANI_CLIP_ALL_SECTIONS; ++i)
    {
        sections[i] = &kNone;
        sizes[i] = 0;
    }
    if (plan->rootMotionBytes != NULL)
    {
        sections[maniClipTagRootMotion - 1] = plan->rootMotionBytes;
        sizes[maniClipTagRootMotion - 1] = maniRootMotionSize(plan->sampleCount);
    }
}

static maniBuildResult Write(maniBuilder* builder, const maniClipSource* source, const Plan* plan,
                             const maniClipNamedLayout* named, float* previous)
{
    uint8_t header[MANI_CLIP_HEADER_SIZE] = {0};
    maniStoreU64(header, maniSkeletonIdentity(source->skeleton));
    maniStoreU32(header + 8, maniSkeletonBoneCount(source->skeleton));
    maniStoreU32(header + 12, plan->sampleCount);
    memcpy(header + 16, &plan->rate, 4);
    memcpy(header + 20, &plan->duration, 4);
    maniStoreU32(header + 24, source->loop);
    maniStoreU32(header + 28, plan->tracks);
    maniStoreU32(header + 32, plan->floatsPerSample);
    for (uint32_t l = 0; l < 4; ++l)
    {
        maniStoreU32(header + 36 + 4 * l, named->counts[l]);
    }
    maniStoreU32(header + 60, source->additive);
    size_t trackBytes = (size_t)plan->tracks * MANI_CLIP_TRACK_SIZE;
    size_t sampleFloats = 0;
    if (ckd_mul(&sampleFloats, (size_t)plan->sampleCount, (size_t)plan->floatsPerSample))
    {
        return Refuse(maniBuildReasonTiming, 0, 0);
    }
    uint8_t* tracks = maniAllocate(&builder->allocator, trackBytes + 1, 1, 8);
    float* samples = maniAllocate(&builder->allocator, sampleFloats + 1, sizeof(float), 8);
    uint8_t* file = NULL;
    uint64_t fileSize = 0;
    if (tracks != NULL && samples != NULL)
    {
        for (uint32_t i = 0; i < plan->tracks; ++i)
        {
            const maniBoneTrackSource* t = &source->boneTracks[plan->order[i]];
            MANI_ASSERT(t->bone <= UINT16_MAX && Mask(source, t) != 0);
            maniStoreU16(tracks + 4 * (size_t)i, (uint16_t)t->bone);
            tracks[4 * (size_t)i + 2] = (uint8_t)Mask(source, t);
            tracks[4 * (size_t)i + 3] = 0;
        }
        for (uint32_t k = 0; k < plan->sampleCount; ++k)
        {
            double time = maniClipSampleTime(k, plan->sampleCount, plan->rate, plan->duration);
            WriteSample(source, plan->order, plan->tracks, plan->rootMotion, k, time, previous,
                        samples + (size_t)k * plan->floatsPerSample);
        }
        const uint8_t* sections[MANI_CLIP_ALL_SECTIONS] = {header, tracks, (const uint8_t*)samples};
        uint64_t sizes[MANI_CLIP_ALL_SECTIONS] = {sizeof header, trackBytes,
                                                  sampleFloats * sizeof(float)};
        LaySections(plan, named, sections, sizes);
        // Empty sections are left out of the table.
        maniSectionSource table[MANI_CLIP_ALL_SECTIONS];
        uint32_t count = 0;
        for (uint32_t i = 0; i < MANI_CLIP_ALL_SECTIONS; ++i)
        {
            table[count] = (maniSectionSource){i + 1, sections[i], sizes[i]};
            count += sizes[i] > 0 ? 1 : 0;
        }
        if (maniContainerSize(table, count, &fileSize) && fileSize <= SIZE_MAX)
        {
            file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
        }
        if (file != NULL)
        {
            maniWriteContainer(file, maniKindClip, MANI_CLIP_FORMAT_VERSION,
                               maniClipComputeIdentity(sections, sizes, MANI_CLIP_ALL_SECTIONS),
                               table, count);
        }
    }
    maniRelease(&builder->allocator, tracks, trackBytes + 1, 1, 8);
    maniRelease(&builder->allocator, samples, sampleFloats + 1, sizeof(float), 8);
    if (file == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    maniBuilderKeep(builder, maniKindClip, file, (size_t)fileSize, NULL, 0);
    return Outcome(mani_success);
}

// The root bone's locals and the motion taken from them, at each sample;
// Capacity when the allocator fails.
static maniResult TakeRoot(maniBuilder* builder, const maniClipSource* source,
                           const maniBoneTrackSource* track, const Plan* plan, RootBone* root)
{
    uint32_t n = plan->sampleCount;
    double* times = maniAllocate(&builder->allocator, n, sizeof(double), 8);
    root->locals = maniAllocate(&builder->allocator, (size_t)n * 7, sizeof(double), 8);
    maniResult r = times != NULL && root->locals != NULL ? mani_success : mani_errorCapacity;
    for (uint32_t k = 0; r == mani_success && k < n; ++k)
    {
        times[k] = maniClipSampleTime(k, n, plan->rate, plan->duration);
        Local(source, track, times[k], root->locals + 7 * (size_t)k);
    }
    maniTransform rest = maniSkeletonRest(source->skeleton, track->bone);
    const double restLocal[7] = {(double)rest.translation.x, (double)rest.translation.y,
                                 (double)rest.translation.z, (double)rest.rotation.x,
                                 (double)rest.rotation.y,    (double)rest.rotation.z,
                                 (double)rest.rotation.w};
    r = r == mani_success ? maniRootMotionTake(source->rootMotion, root->locals, restLocal, times,
                                               n, &builder->allocator, &root->work)
                          : r;
    maniRelease(&builder->allocator, times, n, sizeof(double), 8);
    return r;
}

// Writes the file with root motion given as it is, the bone's channels
// kept as they come.
static maniBuildResult WriteGiven(maniBuilder* builder, const maniClipSource* source, Plan* plan,
                                  const maniClipNamedLayout* named, float* previous,
                                  const maniRootMotionWork* given)
{
    uint64_t size = maniRootMotionSize(plan->sampleCount);
    uint8_t* bytes = maniAllocate(&builder->allocator, (size_t)size, 1, 8);
    if (bytes == NULL)
    {
        return Outcome(mani_errorCapacity);
    }
    plan->rootMotionBytes = bytes;
    maniBuildResult r = maniRootMotionWrite(given, bytes)
                            ? Write(builder, source, plan, named, previous)
                            : Refuse(maniBuildReasonRootMotion, given->source.bone, 0);
    plan->rootMotionBytes = NULL;
    maniRelease(&builder->allocator, bytes, (size_t)size, 1, 8);
    return r;
}

// Takes the root motion (if any) out, or writes the given one, then
// writes the file; seen holds each bone's track index + 1.
static maniBuildResult WriteWithRootMotion(maniBuilder* builder, const maniClipSource* source,
                                           Plan* plan, const maniClipNamedLayout* named,
                                           float* previous, const uint32_t* seen,
                                           const maniRootMotionWork* given)
{
    if (given != NULL)
    {
        return WriteGiven(builder, source, plan, named, previous, given);
    }
    if (source->rootMotion == NULL)
    {
        return Write(builder, source, plan, named, previous);
    }
    // CheckRootMotion refused a bone without a track.
    MANI_ASSERT(seen[source->rootMotion->bone] != 0);
    const maniBoneTrackSource* track = &source->boneTracks[seen[source->rootMotion->bone] - 1];
    RootBone root = {0};
    uint64_t size = maniRootMotionSize(plan->sampleCount);
    uint8_t* bytes = maniAllocate(&builder->allocator, (size_t)size, 1, 8);
    maniBuildResult r = Outcome(mani_errorCapacity);
    if (bytes != NULL && TakeRoot(builder, source, track, plan, &root) == mani_success)
    {
        plan->rootMotion = &root;
        plan->rootMotionBytes = bytes;
        r = maniRootMotionWrite(&root.work, bytes)
                ? Write(builder, source, plan, named, previous)
                : Refuse(maniBuildReasonRootMotion, source->rootMotion->bone, 0);
    }
    plan->rootMotion = NULL;
    plan->rootMotionBytes = NULL;
    maniRootMotionRelease(&root.work, &builder->allocator);
    maniRelease(&builder->allocator, root.locals, (size_t)plan->sampleCount * 7, sizeof(double), 8);
    maniRelease(&builder->allocator, bytes, (size_t)size, 1, 8);
    return r;
}

static maniBuildResult CheckTiming(const maniClipSource* source, float* rateOut)
{
    if (source->loop > maniLoopRepeat || source->additive > maniAdditiveFirstFrame)
    {
        return Refuse(maniBuildReasonKind, 0, 0);
    }
    float rate = source->sampleRate == 0.0f ? 30.0f : source->sampleRate;
    if (!Finite(rate) || !(rate > 0.0f) || !Finite(source->duration) || source->duration < 0.0f)
    {
        return Refuse(maniBuildReasonTiming, 0, 0);
    }
    if (source->boneTrackCount > 0 && source->boneTracks == NULL)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    *rateOut = rate;
    return Outcome(mani_success);
}

// An additive clip's tracks with scale keys: no basis scale component 0.
static maniBuildResult CheckBasis(const maniClipSource* source)
{
    for (uint32_t i = 0; source->additive != maniAdditiveNone && i < source->boneTrackCount; ++i)
    {
        const maniBoneTrackSource* t = &source->boneTracks[i];
        double b[4];
        if (t->scale.keyCount == 0)
        {
            continue;
        }
        Basis(source, t, 2, b);
        if (b[0] == 0.0 || b[1] == 0.0 || b[2] == 0.0)
        {
            return Refuse(maniBuildReasonBasis, i, 0);
        }
    }
    return Outcome(mani_success);
}

// A root motion source: its fields, a bone with a track (seen holds each
// bone's track index + 1), a clip of poses.
static maniBuildResult CheckRootMotion(const maniClipSource* source, const uint32_t* seen)
{
    const maniRootMotionSource* s = source->rootMotion;
    if (s == NULL)
    {
        return Outcome(mani_success);
    }
    bool valid = maniRootMotionSourceValid(s) && source->additive == maniAdditiveNone &&
                 s->bone < maniSkeletonBoneCount(source->skeleton) && seen[s->bone] != 0;
    return valid ? Outcome(mani_success) : Refuse(maniBuildReasonRootMotion, s->bone, 0);
}

maniBuildResult maniCheckClipSource(const maniClipSource* source, uint32_t* seen,
                                    maniClipTiming* timingOut)
{
    maniClipTiming timing = {0};
    maniBuildResult r = CheckTiming(source, &timing.rate);
    if (r.result == mani_success)
    {
        r = CheckTracks(source, seen, &timing.duration);
    }
    if (r.result == mani_success)
    {
        r = CheckBasis(source);
    }
    if (r.result == mani_success)
    {
        r = CheckRootMotion(source, seen);
    }
    float namedLast = 0.0f;
    if (r.result == mani_success)
    {
        r = maniCheckClipNamedSource(source, &namedLast);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (source->duration == 0.0f)
    {
        timing.duration = timing.duration > namedLast ? timing.duration : namedLast;
    }
    else
    {
        timing.duration = source->duration;
    }
    if (!maniClipSampleCountFor(timing.duration, timing.rate, &timing.sampleCount))
    {
        return (maniBuildResult){mani_errorLimit, maniBuildReasonTiming, 0, 0};
    }
    *timingOut = timing;
    return maniCheckClipNamedDuration(source, timing.duration);
}

// Given root motion: a source without its own, a clip of poses, a valid
// source whose bone has keyed translation and rotation, a move a sample.
static maniBuildResult CheckGiven(const maniClipSource* source, const uint32_t* seen,
                                  uint32_t sampleCount, const maniRootMotionWork* given)
{
    if (given == NULL)
    {
        return Outcome(mani_success);
    }
    const maniRootMotionSource* s = &given->source;
    bool valid = source->rootMotion == NULL && source->additive == maniAdditiveNone &&
                 maniRootMotionSourceValid(s) &&
                 s->bone < maniSkeletonBoneCount(source->skeleton) && seen[s->bone] != 0 &&
                 given->sampleCount == sampleCount;
    const maniBoneTrackSource* t = valid ? &source->boneTracks[seen[s->bone] - 1] : NULL;
    return t != NULL && t->translation.keyCount > 0 && t->rotation.keyCount > 0
               ? Outcome(mani_success)
               : Refuse(maniBuildReasonRootMotion, s->bone, 0);
}

static MANI_NOINLINE maniBuildResult BuildClipWork(maniBuilder* builder,
                                                   const maniClipSource* source)
{
    if (maniBuilderCalledBack(builder))
    {
        return maniBuildCalledBack();
    }
    return maniBuildClipMoved(builder, source, NULL);
}

maniBuildResult maniBuildClip(maniBuilder* builder, const maniClipSource* source)
{
    uint64_t env = maniFloatEnvEnter();
    maniBuildResult result = BuildClipWork(builder, source);
    maniFloatEnvLeave(env);
    return result;
}

maniBuildResult maniBuildClipMoved(maniBuilder* builder, const maniClipSource* source,
                                   const maniRootMotionWork* given)
{
    if (builder == NULL)
    {
        return Outcome(mani_errorInvalid);
    }
    if (source == NULL || source->skeleton == NULL)
    {
        maniBuilderMisuse(builder);
        return Outcome(mani_errorInvalid);
    }
    Plan plan = {0};
    maniBuildResult r;
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    size_t words = (size_t)bones + (size_t)source->boneTrackCount;
    uint32_t* scratch = maniAllocate(&builder->allocator, words, sizeof(uint32_t), 8);
    float* previous =
        maniAllocate(&builder->allocator, 4 * (size_t)source->boneTrackCount + 1, sizeof(float), 8);
    if (scratch == NULL || previous == NULL)
    {
        r = Outcome(mani_errorCapacity);
    }
    else
    {
        memset(scratch, 0, words * sizeof(uint32_t));
        memset(previous, 0, (4 * (size_t)source->boneTrackCount + 1) * sizeof(float));
        maniClipTiming timing = {0};
        r = maniCheckClipSource(source, scratch, &timing);
        plan.duration = timing.duration;
        plan.rate = timing.rate;
        plan.sampleCount = timing.sampleCount;
        r = r.result == mani_success ? CheckGiven(source, scratch, plan.sampleCount, given) : r;
    }
    maniClipNamedLayout named = {0};
    if (r.result == mani_success)
    {
        r = maniLayClipNamed(builder, source, plan.duration, plan.rate, plan.sampleCount, &named);
    }
    if (r.result == mani_success)
    {
        // Tracks with a channel, in bone order.
        plan.order = scratch + bones;
        for (uint32_t bone = 0; bone < bones; ++bone)
        {
            uint32_t index = scratch[bone];
            MANI_ASSERT(index <= source->boneTrackCount);
            if (index != 0 && Mask(source, &source->boneTracks[index - 1]) != 0)
            {
                plan.order[plan.tracks++] = index - 1;
                plan.floatsPerSample +=
                    maniChannelFloats(Mask(source, &source->boneTracks[index - 1]));
            }
        }
        MANI_ASSERT(plan.tracks <= source->boneTrackCount && plan.sampleCount > 0);
        r = WriteWithRootMotion(builder, source, &plan, &named, previous, scratch, given);
        maniReleaseClipNamed(builder, &named);
    }
    maniRelease(&builder->allocator, scratch, words, sizeof(uint32_t), 8);
    maniRelease(&builder->allocator, previous, 4 * (size_t)source->boneTrackCount + 1,
                sizeof(float), 8);
    return r;
}

bool maniImportClipOptionsValid(const maniImportClipOptions* options, const maniSkeleton* skeleton)
{
    const maniRootMotionSource* root = options->rootMotion;
    return isfinite(options->sampleRate) && options->sampleRate >= 0.0f &&
           options->loop <= maniLoopRepeat && options->additive <= maniAdditiveFirstFrame &&
           (root == NULL ||
            (maniRootMotionSourceValid(root) && root->bone < maniSkeletonBoneCount(skeleton)));
}
