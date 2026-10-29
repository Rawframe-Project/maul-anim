// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The compressor's sub-tracks: their values, clip and segment
// ranges, what a rate plays (by the codec's own arithmetic, so it is what
// the sampler decodes), and the float tracks' folding and rates.

#include "clip_plan.h"

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "vecmath.h"

#include <math.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

uint32_t maniPlanWidth(const maniSubTrack* t)
{
    return t->channel == maniSubFloat ? 1u : 3u;
}

float maniPlanPrecision(const maniCompressPlan* p, uint32_t bone)
{
    const maniCompressOptions* o = p->options;
    return o->bonePrecisions != NULL ? o->bonePrecisions[bone] : o->precision;
}

float maniPlanShell(const maniCompressPlan* p, uint32_t bone)
{
    const maniCompressOptions* o = p->options;
    return o->boneShellDistances != NULL ? o->boneShellDistances[bone] : o->shellDistance;
}

void maniPlanValue(const maniCompressPlan* p, const maniSubTrack* t, uint32_t sample, float out[3])
{
    const maniClip* c = p->clip;
    if (t->channel == maniSubFloat)
    {
        out[0] = LoadF32(c->floatSamples + 4 * ((size_t)sample * c->floatTrackCount + t->offset));
        return;
    }
    const uint8_t* v = c->samples + 4 * ((size_t)sample * c->floatsPerSample + t->offset);
    if (t->channel != maniSubRotation)
    {
        for (int k = 0; k < 3; ++k)
        {
            out[k] = LoadF32(v + 4 * k);
        }
        return;
    }
    float sign = LoadF32(v + 4 * t->dropped) < 0.0f ? -1.0f : 1.0f;
    for (uint32_t i = 0, k = 0; i < 4; ++i)
    {
        if (i != t->dropped)
        {
            out[k++] = sign * LoadF32(v + 4 * i);
        }
    }
}

void maniPlanChooseDropped(const maniCompressPlan* p, maniSubTrack* t)
{
    const maniClip* c = p->clip;
    float smallest[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    for (uint32_t k = 0; k < c->sampleCount; ++k)
    {
        const uint8_t* v = c->samples + 4 * ((size_t)k * c->floatsPerSample + t->offset);
        for (uint32_t i = 0; i < 4; ++i)
        {
            float m = fabsf(LoadF32(v + 4 * i));
            smallest[i] = m < smallest[i] ? m : smallest[i];
        }
    }
    t->dropped = 3;
    for (uint32_t i = 0; i < 3; ++i)
    {
        t->dropped = smallest[i] > smallest[t->dropped] ? i : t->dropped;
    }
}

uint32_t maniPlanSegmentCount(uint32_t samples)
{
    return samples <= MANI_CLIP_SEGMENT_MAX
               ? 1u
               : (samples + MANI_CLIP_SEGMENT_IDEAL - 1) / MANI_CLIP_SEGMENT_IDEAL;
}

// The last segment's shortfall is spread over the first ones, one sample
// each.
void maniPlanSplit(uint32_t samples, uint32_t segments, uint32_t* firsts)
{
    uint32_t base = samples / segments;
    uint32_t extra = samples % segments;
    uint32_t at = 0;
    for (uint32_t s = 0; s < segments; ++s)
    {
        firsts[s] = at;
        at += base + (s < extra ? 1u : 0u);
    }
    firsts[segments] = samples;
}

void maniPlanRanges(const maniCompressPlan* p, maniSubTrack* t)
{
    uint32_t width = maniPlanWidth(t);
    float first[3] = {0.0f, 0.0f, 0.0f};
    maniPlanValue(p, t, 0, first);
    float low[3] = {first[0], first[1], first[2]};
    float high[3] = {first[0], first[1], first[2]};
    for (uint32_t k = 1; k < p->clip->sampleCount; ++k)
    {
        float v[3] = {0.0f, 0.0f, 0.0f};
        maniPlanValue(p, t, k, v);
        for (uint32_t c = 0; c < width; ++c)
        {
            low[c] = v[c] < low[c] ? v[c] : low[c];
            high[c] = v[c] > high[c] ? v[c] : high[c];
        }
    }
    for (uint32_t c = 0; c < width; ++c)
    {
        t->constant[c] = first[c];
        t->minimum[c] = low[c];
        t->extent[c] = high[c] - low[c];
    }
}

// A value's clip-normalized form.
static float Normalized(const maniSubTrack* t, uint32_t c, float v)
{
    return t->extent[c] > 0.0f ? (v - t->minimum[c]) / t->extent[c] : 0.0f;
}

uint32_t maniPlanQuantize(const maniSubTrack* t, uint32_t c, float v, uint32_t rate,
                          const uint8_t* range)
{
    if (rate == MANI_CODEC_RAW_RATE)
    {
        uint32_t bits = 0;
        memcpy(&bits, &v, sizeof bits);
        return bits;
    }
    float n = Normalized(t, c, v);
    if (range != NULL)
    {
        float low = maniCodecRangeByte(range[c]);
        float size = maniCodecRangeByte(range[maniPlanWidth(t) + c]);
        n = size > 0.0f ? (n - low) / size : 0.0f;
    }
    return maniCodecPack(n, rate);
}

float maniPlanPlayed(const maniSubTrack* t, uint32_t c, float v, uint32_t rate,
                     const uint8_t* range)
{
    uint32_t width = maniPlanWidth(t);
    if (rate == MANI_CODEC_RAW_RATE)
    {
        return v;
    }
    if (rate == 0)
    {
        return maniCodecValue(maniCodecUnpack16(maniLoadU16(range + 2 * c)), t->minimum[c],
                              t->extent[c]);
    }
    float q = maniCodecUnpack(maniPlanQuantize(t, c, v, rate, range), rate);
    if (range != NULL)
    {
        q = maniCodecSegmentValue(q, range[c], range[width + c]);
    }
    return maniCodecValue(q, t->minimum[c], t->extent[c]);
}

void maniPlanSegmentRange(const maniCompressPlan* p, const maniSubTrack* t, uint32_t s,
                          uint8_t range[6], uint8_t constant[6])
{
    uint32_t width = maniPlanWidth(t);
    for (uint32_t c = 0; c < width; ++c)
    {
        float low = 1.0f;
        float high = 0.0f;
        for (uint32_t k = p->firsts[s]; k < p->firsts[s + 1]; ++k)
        {
            float v[3] = {0.0f, 0.0f, 0.0f};
            maniPlanValue(p, t, k, v);
            float n = Normalized(t, c, v[c]);
            n = n > 0.0f ? (n < 1.0f ? n : 1.0f) : 0.0f;
            low = n < low ? n : low;
            high = n > high ? n : high;
        }
        maniCodecSegmentRange(low, high, &range[c], &range[width + c]);
        maniStoreU16(constant + 2 * c, maniCodecPack16(low + (high - low) * 0.5f));
    }
}

void maniPlanSetChannel(const maniCompressPlan* p, const maniSubTrack* t, uint32_t sample,
                        uint32_t rate, const uint8_t* range, maniTransform* local)
{
    if (t->form == maniClipFormDefault)
    {
        // An additive clip's default is the identity difference.
        maniTransform rest =
            p->clip->additive != 0
                ? (maniTransform){{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}
                : maniSkeleton_GetRest(p->skeleton, t->bone);
        local->translation =
            t->channel == maniSubTranslation ? rest.translation : local->translation;
        local->rotation = t->channel == maniSubRotation ? rest.rotation : local->rotation;
        local->scale = t->channel == maniSubScale ? rest.scale : local->scale;
        return;
    }
    float o[3] = {t->constant[0], t->constant[1], t->constant[2]};
    if (t->form != maniClipFormConstant)
    {
        float v[3] = {0.0f, 0.0f, 0.0f};
        maniPlanValue(p, t, sample, v);
        for (uint32_t c = 0; c < 3; ++c)
        {
            o[c] = maniPlanPlayed(t, c, v[c], rate, range);
        }
    }
    if (t->channel == maniSubRotation)
    {
        // As the sampler: w rebuilt, then a normalized lerp of the sample
        // with itself.
        float q[4];
        maniCodecRotation(o, t->dropped, q);
        maniCodecNormalize(q);
        local->rotation = (maniQuat){q[0], q[1], q[2], q[3]};
    }
    else if (t->channel == maniSubTranslation)
    {
        local->translation = (maniVec3){o[0], o[1], o[2]};
    }
    else
    {
        local->scale = (maniVec3){o[0], o[1], o[2]};
    }
}

void maniPlanFoldFloat(const maniCompressPlan* p, maniSubTrack* t)
{
    bool constant = true;
    for (uint32_t k = 1; constant && k < p->clip->sampleCount; ++k)
    {
        float v[3] = {0.0f, 0.0f, 0.0f};
        maniPlanValue(p, t, k, v);
        constant = fabsf(v[0] - t->constant[0]) <= t->tolerance;
    }
    t->form = constant ? maniClipFormConstant : maniClipFormAnimated;
}

// Whether every sample of a segment plays within the tolerance at a rate.
static bool Fits(const maniCompressPlan* p, const maniSubTrack* t, uint32_t s, uint32_t rate,
                 const uint8_t* range)
{
    for (uint32_t k = p->firsts[s]; k < p->firsts[s + 1]; ++k)
    {
        float v[3] = {0.0f, 0.0f, 0.0f};
        maniPlanValue(p, t, k, v);
        if (!(fabsf(maniPlanPlayed(t, 0, v[0], rate, range) - v[0]) <= t->tolerance))
        {
            return false;
        }
    }
    return true;
}

// Each animated float track's rate and range bytes in each segment: rate
// 0 where the segment allows it, else the fewest bits that fit, else raw.
void maniPlanFloatRates(maniCompressPlan* p)
{
    bool several = p->segmentCount > 1;
    uint32_t n = p->animatedChannels + p->animatedFloats;
    for (uint32_t s = 0; s < p->segmentCount; ++s)
    {
        for (uint32_t i = 0; i < p->subCount; ++i)
        {
            const maniSubTrack* t = &p->subs[i];
            if (t->channel != maniSubFloat || t->form != maniClipFormAnimated)
            {
                continue;
            }
            size_t at = (size_t)s * n + t->animated;
            uint8_t* range = p->segmentRanges + 6 * at;
            uint8_t constant[6] = {0};
            maniPlanSegmentRange(p, t, s, range, constant);
            uint32_t rate = several && Fits(p, t, s, 0, constant) ? 0 : 1;
            while (rate != 0 && rate < MANI_CODEC_RAW_RATE &&
                   !Fits(p, t, s, rate, several ? range : NULL))
            {
                ++rate;
            }
            memcpy(range, rate == 0 ? constant : range, 6);
            p->rates[at] = (uint8_t)rate;
        }
    }
}
