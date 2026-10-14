// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a full-precision clip file (mani-0006): every channel checked,
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
#include "scalar.h"

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/skeleton.h"

#include <math.h>
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

static uint32_t Mask(const maniBoneTrackSource* t)
{
    return (t->translation.keyCount > 0 ? maniChannelTranslation : 0u) |
           (t->rotation.keyCount > 0 ? maniChannelRotation : 0u) |
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

// glTF's slerp of two unit quaternions, the short way, in binary64.
static void Slerp(const double a[4], const double b[4], double u, double out[4])
{
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    double sign = dot < 0.0 ? -1.0 : 1.0;
    double difference = 0.0;
    double sum = 0.0;
    for (int k = 0; k < 4; ++k)
    {
        double d = a[k] - sign * b[k];
        double s = a[k] + sign * b[k];
        difference += d * d;
        sum += s * s;
    }
    // The angle between them, from half-angle lengths: well conditioned
    // at every angle, unlike the arc cosine of the dot product.
    double angle = 2.0 * maniAtan2Double(sqrt(difference), sqrt(sum));
    double wa = 1.0 - u;
    double wb = u;
    if (angle > 1e-9)
    {
        double sinAngle = 0.0;
        double unused = 0.0;
        double sinA = 0.0;
        double sinB = 0.0;
        maniSinCosQuarterDouble(angle, &sinAngle, &unused);
        maniSinCosQuarterDouble((1.0 - u) * angle, &sinA, &unused);
        maniSinCosQuarterDouble(u * angle, &sinB, &unused);
        wa = sinA / sinAngle;
        wb = sinB / sinAngle;
    }
    for (int k = 0; k < 4; ++k)
    {
        out[k] = wa * a[k] + sign * wb * b[k];
    }
}

static void LoadKey(const maniChannelSource* c, uint32_t key, uint32_t part, uint32_t dimension,
                    double out[4])
{
    const float* v = c->values + (size_t)key * KeyFloats(c, dimension) + (size_t)part * dimension;
    for (uint32_t k = 0; k < dimension; ++k)
    {
        out[k] = (double)v[k];
    }
}

// The last key at or before a time strictly inside the keys.
static uint32_t KeyBefore(const maniChannelSource* c, double time)
{
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
    return low;
}

// Between keys low and low + 1, as Appendix C defines each mode.
static void Interpolate(const maniChannelSource* c, uint32_t dimension, uint32_t low, double time,
                        double out[4])
{
    uint32_t valuePart = c->interpolation == maniInterpolationCubicSpline ? 1 : 0;
    double t0 = (double)c->times[low];
    double span = (double)c->times[low + 1] - t0;
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
        Slerp(a, b, u, out);
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

// One sample: every track's animated channels, rotations kept in the
// hemisphere of the track's previous rotation (previous holds four floats
// a track).
static void WriteSample(const maniClipSource* source, const uint32_t* order, uint32_t tracks,
                        double time, float* previous, bool first, float* out)
{
    for (uint32_t i = 0; i < tracks; ++i)
    {
        const maniBoneTrackSource* t = &source->boneTracks[order[i]];
        double v[4];
        if (t->translation.keyCount > 0)
        {
            maniEvaluateChannel(&t->translation, 3, time, v);
            for (int k = 0; k < 3; ++k)
            {
                *out++ = (float)v[k];
            }
        }
        if (t->rotation.keyCount > 0)
        {
            maniEvaluateChannel(&t->rotation, 4, time, v);
            float q[4] = {(float)v[0], (float)v[1], (float)v[2], (float)v[3]};
            float* p = previous + 4 * (size_t)i;
            float dot = q[0] * p[0] + q[1] * p[1] + q[2] * p[2] + q[3] * p[3];
            float sign = !first && dot < 0.0f ? -1.0f : 1.0f;
            for (int k = 0; k < 4; ++k)
            {
                p[k] = sign * q[k];
                *out++ = p[k];
            }
        }
        if (t->scale.keyCount > 0)
        {
            maniEvaluateChannel(&t->scale, 3, time, v);
            for (int k = 0; k < 3; ++k)
            {
                *out++ = (float)v[k];
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
    uint32_t* order;
    uint32_t tracks;
    uint32_t floatsPerSample;
    uint32_t sampleCount;
    float rate;
    float duration;
} Plan;

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
            maniStoreU16(tracks + 4 * (size_t)i, (uint16_t)t->bone);
            tracks[4 * (size_t)i + 2] = (uint8_t)Mask(t);
            tracks[4 * (size_t)i + 3] = 0;
        }
        for (uint32_t k = 0; k < plan->sampleCount; ++k)
        {
            double time = maniClipSampleTime(k, plan->sampleCount, plan->rate, plan->duration);
            WriteSample(source, plan->order, plan->tracks, time, previous, k == 0,
                        samples + (size_t)k * plan->floatsPerSample);
        }
        const uint8_t* sections[MANI_CLIP_SECTIONS] = {header, tracks, (const uint8_t*)samples};
        uint64_t sizes[MANI_CLIP_SECTIONS] = {sizeof header, trackBytes,
                                              sampleFloats * sizeof(float)};
        for (uint32_t i = 0; i < 7; ++i)
        {
            sections[3 + i] = named->data[i];
            sizes[3 + i] = named->sizes[i];
        }
        maniSectionSource table[MANI_CLIP_SECTIONS];
        for (uint32_t i = 0; i < MANI_CLIP_SECTIONS; ++i)
        {
            table[i] = (maniSectionSource){i + 1, sections[i], sizes[i]};
        }
        if (maniContainerSize(table, MANI_CLIP_SECTIONS, &fileSize) && fileSize <= SIZE_MAX)
        {
            file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
        }
        if (file != NULL)
        {
            maniWriteContainer(file, maniKindClip, MANI_CLIP_FORMAT_VERSION,
                               maniClipComputeIdentity(sections, sizes), table, MANI_CLIP_SECTIONS);
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

static maniBuildResult CheckTiming(const maniClipSource* source, float* rateOut)
{
    if (source->loop > maniLoopRepeat)
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

maniBuildResult maniBuildClip(maniBuilder* builder, const maniClipSource* source)
{
    if (builder == NULL)
    {
        return Outcome(mani_errorInvalid);
    }
    if (source == NULL || source->skeleton == NULL)
    {
        ++builder->misuses;
        return Outcome(mani_errorInvalid);
    }
    Plan plan = {0};
    maniBuildResult r = CheckTiming(source, &plan.rate);
    if (r.result != mani_success)
    {
        return r;
    }
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
        r = CheckTracks(source, scratch, &plan.duration);
    }
    float namedLast = 0.0f;
    if (r.result == mani_success)
    {
        r = maniCheckClipNamedSource(source, &namedLast);
    }
    if (source->duration == 0.0f)
    {
        plan.duration = plan.duration > namedLast ? plan.duration : namedLast;
    }
    else
    {
        plan.duration = source->duration;
    }
    if (r.result == mani_success &&
        !maniClipSampleCountFor(plan.duration, plan.rate, &plan.sampleCount))
    {
        r = Refuse(maniBuildReasonTiming, 0, 0);
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
            if (index != 0 && Mask(&source->boneTracks[index - 1]) != 0)
            {
                plan.order[plan.tracks++] = index - 1;
                plan.floatsPerSample += maniChannelFloats(Mask(&source->boneTracks[index - 1]));
            }
        }
        r = Write(builder, source, &plan, &named, previous);
        maniReleaseClipNamed(builder, &named);
    }
    maniRelease(&builder->allocator, scratch, words, sizeof(uint32_t), 8);
    maniRelease(&builder->allocator, previous, 4 * (size_t)source->boneTrackCount + 1,
                sizeof(float), 8);
    return r;
}
