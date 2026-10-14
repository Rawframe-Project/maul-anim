// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// How far a clip plays from its source (mani-0006): the clip sampled
// through its own functions at every sample time and halfway between,
// against the source evaluated in binary64, and how late each step key
// plays once moved onto the grid.

#include "allocator.h"
#include "builder.h"
#include "clip_build.h"
#include "scalar.h"

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <string.h>

// What the measure holds while it runs.
typedef struct Measure
{
    const maniClipSource* source;
    const maniClip* clip;
    maniClipTiming timing;
    maniPose* pose;
    // The clip's float track for each source float track.
    uint32_t* floatMap;
    float* floats;
    uint32_t floatCount;
    maniClipReport report;
} Measure;

static void Note(maniClipError* e, double error, float time, uint32_t item)
{
    if (error > e->error)
    {
        *e = (maniClipError){error, time, item};
    }
}

static double Distance3(const float* played, const double* source)
{
    double x = (double)played[0] - source[0];
    double y = (double)played[1] - source[1];
    double z = (double)played[2] - source[2];
    return sqrt(x * x + y * y + z * z);
}

// The angle between two rotations: twice the angle between the unit
// quaternions on one side, as 2 atan2(|a - b|, |a + b|), exact near 0.
static double Angle(const float* played, const double* source)
{
    double dot = 0.0;
    for (int k = 0; k < 4; ++k)
    {
        dot += (double)played[k] * source[k];
    }
    double side = dot < 0.0 ? -1.0 : 1.0;
    double minus = 0.0;
    double plus = 0.0;
    for (int k = 0; k < 4; ++k)
    {
        double a = (double)played[k];
        double b = side * source[k];
        minus += (a - b) * (a - b);
        plus += (a + b) * (a + b);
    }
    return 2.0 * maniAtan2Double(sqrt(minus), sqrt(plus));
}

static bool Has(const maniChannelSource* c)
{
    return c->keyCount > 0;
}

// One bone track's channels at one time; step channels left out halfway
// between samples.
static void MeasureBone(Measure* m, const maniBoneTrackSource* t, double sourceTime, float time,
                        bool midpoint)
{
    maniClipReport* r = &m->report;
    maniTransform played = maniPoseLocal(m->pose, t->bone);
    const maniChannelSource* channels[3] = {&t->translation, &t->rotation, &t->scale};
    maniClipError* errors[3] = {midpoint ? &r->midTranslation : &r->sampleTranslation,
                                midpoint ? &r->midRotation : &r->sampleRotation,
                                midpoint ? &r->midScale : &r->sampleScale};
    const float* values[3] = {&played.translation.x, &played.rotation.x, &played.scale.x};
    for (uint32_t c = 0; c < 3; ++c)
    {
        if (!Has(channels[c]) || (midpoint && channels[c]->interpolation == maniInterpolationStep))
        {
            continue;
        }
        double v[4];
        maniEvaluateChannel(channels[c], c == 1 ? 4 : 3, sourceTime, v);
        double error = c == 1 ? Angle(values[c], v) : Distance3(values[c], v);
        Note(errors[c], error, time, t->bone);
    }
}

// One time: every bone channel and float track against the source.
static maniResult MeasureAt(Measure* m, double sourceTime, float time, bool midpoint)
{
    if (maniSampleClip(m->clip, m->source->skeleton, time, m->pose) != mani_success ||
        maniSampleClipFloats(m->clip, time, m->floats, m->floatCount, NULL) != mani_success)
    {
        return mani_errorInvalid;
    }
    for (uint32_t i = 0; i < m->source->boneTrackCount; ++i)
    {
        MeasureBone(m, &m->source->boneTracks[i], sourceTime, time, midpoint);
    }
    for (uint32_t i = 0; i < m->source->floatTrackCount; ++i)
    {
        const maniChannelSource* keys = &m->source->floatTracks[i].keys;
        if (midpoint && keys->interpolation == maniInterpolationStep)
        {
            continue;
        }
        double v[4];
        maniEvaluateChannel(keys, 1, sourceTime, v);
        double error = fabs((double)m->floats[m->floatMap[i]] - v[0]);
        Note(midpoint ? &m->report.midFloat : &m->report.sampleFloat, error, time, i);
    }
    return mani_success;
}

// How late each key of a step channel plays: from its time to the first
// sample at or after it. Keys past the duration play at the end.
static void StepShifts(const maniClipTiming* g, const maniChannelSource* c, uint32_t item,
                       maniClipError* e)
{
    if (!Has(c) || c->interpolation != maniInterpolationStep)
    {
        return;
    }
    for (uint32_t k = 0; k < c->keyCount; ++k)
    {
        double t = (double)c->times[k];
        if (t > (double)g->duration)
        {
            continue;
        }
        // A first guess, at or below the sample: the loops settle it.
        double estimate = t * (double)g->rate;
        uint32_t j = estimate < (double)g->sampleCount ? (uint32_t)estimate : g->sampleCount - 1;
        while (j > 0 && maniClipSampleTime(j - 1, g->sampleCount, g->rate, g->duration) >= t)
        {
            --j;
        }
        while (j + 1 < g->sampleCount &&
               maniClipSampleTime(j, g->sampleCount, g->rate, g->duration) < t)
        {
            ++j;
        }
        double shift = maniClipSampleTime(j, g->sampleCount, g->rate, g->duration) - t;
        Note(e, shift, c->times[k], item);
    }
}

static maniResult Run(Measure* m)
{
    const maniClipTiming* g = &m->timing;
    for (uint32_t k = 0; k < g->sampleCount; ++k)
    {
        double t = maniClipSampleTime(k, g->sampleCount, g->rate, g->duration);
        if (MeasureAt(m, t, (float)t, false) != mani_success)
        {
            return mani_errorInvalid;
        }
        if (k + 1 < g->sampleCount)
        {
            double next = maniClipSampleTime(k + 1, g->sampleCount, g->rate, g->duration);
            float mid = (float)(0.5 * (t + next));
            if (MeasureAt(m, (double)mid, mid, true) != mani_success)
            {
                return mani_errorInvalid;
            }
        }
    }
    for (uint32_t i = 0; i < m->source->boneTrackCount; ++i)
    {
        const maniBoneTrackSource* t = &m->source->boneTracks[i];
        StepShifts(g, &t->translation, t->bone, &m->report.boneStepShift);
        StepShifts(g, &t->rotation, t->bone, &m->report.boneStepShift);
        StepShifts(g, &t->scale, t->bone, &m->report.boneStepShift);
    }
    for (uint32_t i = 0; i < m->source->floatTrackCount; ++i)
    {
        StepShifts(g, &m->source->floatTracks[i].keys, i, &m->report.floatStepShift);
    }
    return mani_success;
}

// The clip must be the source's: its skeleton, grid and float tracks.
static bool Matches(Measure* m)
{
    const maniClipTiming* g = &m->timing;
    if (maniClipSkeletonIdentity(m->clip) != maniSkeletonIdentity(m->source->skeleton) ||
        maniClipSampleCount(m->clip) != g->sampleCount || maniClipSampleRate(m->clip) != g->rate ||
        maniClipDuration(m->clip) != g->duration)
    {
        return false;
    }
    for (uint32_t i = 0; i < m->source->floatTrackCount; ++i)
    {
        const maniFloatTrackSource* f = &m->source->floatTracks[i];
        m->floatMap[i] = maniClipFindFloatTrack(m->clip, f->name, f->nameLength);
        if (m->floatMap[i] == MANI_NO_TRACK)
        {
            return false;
        }
    }
    return true;
}

maniResult maniMeasureClip(maniBuilder* builder, const maniClipSource* source, const maniClip* clip,
                           maniClipReport* reportOut)
{
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if (source == NULL || source->skeleton == NULL || clip == NULL || reportOut == NULL)
    {
        ++builder->misuses;
        return mani_errorInvalid;
    }
    const maniAllocator* a = &builder->allocator;
    uint32_t bones = maniSkeletonBoneCount(source->skeleton);
    size_t poseBytes = maniPoseSize(source->skeleton) + MANI_POSE_ALIGNMENT;
    size_t floatCount = maniClipFloatTrackCount(clip);
    size_t words = (size_t)bones + source->floatTrackCount + 1;
    uint32_t* seen = maniAllocate(a, words, sizeof(uint32_t), 8);
    float* floats = maniAllocate(a, floatCount + 1, sizeof(float), 8);
    uint8_t* poseBlock = maniAllocate(a, poseBytes, 1, 8);
    maniResult result = mani_errorCapacity;
    if (seen != NULL && floats != NULL && poseBlock != NULL)
    {
        memset(seen, 0, words * sizeof(uint32_t));
        Measure m = {.source = source, .clip = clip, .floats = floats};
        m.floatCount = (uint32_t)floatCount;
        m.floatMap = seen + bones;
        uint8_t* aligned =
            poseBlock + (MANI_POSE_ALIGNMENT - (uintptr_t)poseBlock % MANI_POSE_ALIGNMENT) %
                            MANI_POSE_ALIGNMENT;
        result = mani_errorInvalid;
        if (maniCheckClipSource(source, seen, &m.timing).result == mani_success && Matches(&m) &&
            maniInitPose(source->skeleton, aligned, poseBytes - MANI_POSE_ALIGNMENT, &m.pose) ==
                mani_success)
        {
            result = Run(&m);
        }
        if (result == mani_success)
        {
            *reportOut = m.report;
        }
    }
    maniRelease(a, seen, words, sizeof(uint32_t), 8);
    maniRelease(a, floats, floatCount + 1, sizeof(float), 8);
    maniRelease(a, poseBlock, poseBytes, 1, 8);
    return result;
}
