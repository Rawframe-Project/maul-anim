// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A compressed clip (mani-0019): its sections checked once at open, and
// its samples decoded. Sub-tracks run in track order, each bone track's
// channels translation, rotation, scale, then the float tracks; each is
// absent, default (the rest), constant or animated. An animated value
// at a sample is its bits at its segment's rate, through the segment
// range (with several segments) and the clip range.

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_lanes.h"
#include "clip_view.h"
#include "container.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

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
    uint32_t bits = maniLoadU32(p);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static uint32_t Form(uint32_t byte, uint32_t channel)
{
    return (byte >> (2 * channel)) & 3u;
}

// The component a bone track's stored rotation leaves out.
static uint32_t Dropped(uint32_t byte)
{
    return 3u - (byte >> 6);
}

// The forms: a channel present in the track's mask has a form, an absent
// one none; a float track is constant or animated. Counts of constant
// and animated sub-tracks, channels and floats apart.
typedef struct Counts
{
    uint64_t constantChannels;
    uint64_t constantFloats;
    uint64_t animatedChannels;
    uint64_t animatedRotations;
    uint64_t animatedFloats;
} Counts;

// A bone track's forms byte against its channel mask, counted: false when
// a present channel has no form, an absent one has one, or a dropped
// component is named without a rotation stored.
static bool TrackForms(uint32_t byte, uint32_t mask, Counts* counts)
{
    uint32_t rotation = Form(byte, 1);
    bool right = byte < 64 || rotation == maniClipFormConstant || rotation == maniClipFormAnimated;
    for (uint32_t c = 0; right && c < 3; ++c)
    {
        uint32_t f = Form(byte, c);
        right = ((mask >> c) & 1u) != 0 ? f != maniClipFormAbsent : f == maniClipFormAbsent;
        counts->constantChannels += f == maniClipFormConstant ? 1 : 0;
        counts->animatedChannels += f == maniClipFormAnimated ? 1 : 0;
        counts->animatedRotations += f == maniClipFormAnimated && c == 1 ? 1 : 0;
    }
    return right;
}

static maniFileResult CheckForms(const uint8_t* forms, uint64_t at, const maniClip* clip,
                                 Counts* counts)
{
    *counts = (Counts){0, 0, 0, 0, 0};
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        if (!TrackForms(forms[i], clip->tracks[(size_t)i * MANI_CLIP_TRACK_SIZE + 2], counts))
        {
            return Fail(at + i);
        }
    }
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        uint32_t f = forms[clip->trackCount + i];
        if (f != maniClipFormConstant && f != maniClipFormAnimated)
        {
            return Fail(at + clip->trackCount + i);
        }
        counts->constantFloats += f == maniClipFormConstant ? 1 : 0;
        counts->animatedFloats += f == maniClipFormAnimated ? 1 : 0;
    }
    return (maniFileResult){mani_success, 0, 0};
}

// Where the constants and clip ranges are, walking the sub-tracks.
typedef struct ValueWalk
{
    const uint8_t* constants;
    const uint8_t* ranges;
    uint64_t constantAt;
    uint64_t rangeAt;
} ValueWalk;

// A sub-track's stored values: constants finite, a rotation's within
// [-1, 1]; a clip range's minimum and extent finite, the extent not
// negative, their sum finite, a rotation's minimum at least -1 and
// extent at most 2.
static maniFileResult CheckValues(ValueWalk* w, uint32_t form, uint32_t width, bool rotation)
{
    for (uint32_t k = 0; form == maniClipFormConstant && k < width; ++k)
    {
        float v = LoadF32(w->constants + 4 * k);
        if (!Finite(v) || (rotation && !(v >= -1.0f && v <= 1.0f)))
        {
            return Fail(w->constantAt + 4 * k);
        }
    }
    for (uint32_t k = 0; form == maniClipFormAnimated && k < width; ++k)
    {
        float low = LoadF32(w->ranges + 4 * k);
        float extent = LoadF32(w->ranges + 4 * (width + k));
        if (!Finite(low) || (rotation && !(low >= -1.0f)))
        {
            return Fail(w->rangeAt + 4 * k);
        }
        if (!Finite(extent) || !(extent >= 0.0f) || !Finite(low + extent) ||
            (rotation && !(extent <= 2.0f)))
        {
            return Fail(w->rangeAt + 4 * (width + k));
        }
    }
    uint32_t constant = form == maniClipFormConstant ? 4 * width : 0;
    uint32_t range = form == maniClipFormAnimated ? 8 * width : 0;
    w->constants += constant;
    w->constantAt += constant;
    w->ranges += range;
    w->rangeAt += range;
    return (maniFileResult){mani_success, 0, 0};
}

static maniFileResult CheckAllValues(const uint8_t* const* data, const uint64_t* at,
                                     const maniClip* clip)
{
    ValueWalk w = {data[11], data[12], at[11], at[12]};
    maniFileResult r = {mani_success, 0, 0};
    for (uint32_t i = 0; r.result == mani_success && i < clip->trackCount; ++i)
    {
        for (uint32_t c = 0; r.result == mani_success && c < 3; ++c)
        {
            r = CheckValues(&w, Form(clip->forms[i], c), 3, c == 1);
        }
    }
    for (uint32_t i = 0; r.result == mani_success && i < clip->floatTrackCount; ++i)
    {
        r = CheckValues(&w, clip->forms[clip->trackCount + i], 1, false);
    }
    return r;
}

// A segment's rates (each below MANI_CODEC_RATES, 0 only with several
// segments) and its bits per sample, bone channels and floats apart.
static bool CheckRates(const uint8_t* rates, const maniClip* clip, uint64_t* channelBits,
                       uint64_t* floatBits)
{
    *channelBits = 0;
    *floatBits = 0;
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    for (uint64_t k = 0; k < n; ++k)
    {
        uint32_t rate = rates[k];
        if (rate >= MANI_CODEC_RATES || (rate == 0 && clip->segmentCount == 1))
        {
            return false;
        }
        bool channel = k < clip->animatedChannels;
        *channelBits += channel ? 3 * maniCodecBits(rate) : 0;
        *floatBits += channel ? 0 : maniCodecBits(rate);
    }
    return true;
}

// The bytes before a segment's samples: its rates and, with several
// segments, its ranges.
static uint64_t SegmentHead(const maniClip* clip)
{
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    uint64_t ranges = 6 * (uint64_t)clip->animatedChannels + 2 * (uint64_t)clip->animatedFloats;
    return n + (clip->segmentCount > 1 ? ranges : 0);
}

// The segment table: in order, covering every sample, each segment's
// data where the one before it ends, its bits per sample its rates'.
static maniFileResult CheckSegments(const uint8_t* table, uint64_t at, const uint8_t* data,
                                    uint64_t dataSize, uint64_t dataEntry, const maniClip* clip)
{
    uint64_t next = 0;
    uint64_t offset = 0;
    for (uint32_t s = 0; s < clip->segmentCount; ++s)
    {
        const uint8_t* e = table + (size_t)s * MANI_CLIP_SEGMENT_SIZE;
        uint64_t entryAt = at + (uint64_t)s * MANI_CLIP_SEGMENT_SIZE;
        uint32_t count = maniLoadU32(e + 4);
        if (maniLoadU32(e) != next || count == 0 || next + count > clip->sampleCount)
        {
            return Fail(entryAt + (maniLoadU32(e) != next ? 0 : 4));
        }
        uint64_t head = SegmentHead(clip);
        uint64_t channelBits = 0;
        uint64_t floatBits = 0;
        if (maniLoadU32(e + 8) != offset || offset + head > dataSize ||
            !CheckRates(data + offset, clip, &channelBits, &floatBits))
        {
            return Fail(entryAt + 8);
        }
        if (maniLoadU32(e + 12) != channelBits || maniLoadU32(e + 16) != floatBits)
        {
            return Fail(entryAt + (maniLoadU32(e + 12) != channelBits ? 12 : 16));
        }
        offset += head + ((channelBits + floatBits) * count + 7) / 8;
        next += count;
    }
    if (next != clip->sampleCount)
    {
        return Fail(at);
    }
    // Eight spare bytes, so every read is one 64-bit load.
    return offset + 8 == dataSize ? (maniFileResult){mani_success, 0, 0} : Fail(dataEntry);
}

// A segment's raw values (rate 24), binary32 from the file: each finite.
// The offset of the first that is not, or 0.
static uint64_t RawNotFinite(const maniClip* clip, const uint8_t* e)
{
    const uint8_t* base = clip->segmentData + maniLoadU32(e + 8);
    const uint8_t* stream = base + SegmentHead(clip);
    uint64_t sampleBits = (uint64_t)maniLoadU32(e + 12) + maniLoadU32(e + 16);
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    uint64_t bit = 0;
    for (uint64_t j = 0; j < n; ++j)
    {
        uint32_t width = j < clip->animatedChannels ? 3 : 1;
        uint32_t bits = maniCodecBits(base[j]);
        for (uint32_t k = 0; base[j] == MANI_CODEC_RAW_RATE && k < maniLoadU32(e + 4); ++k)
        {
            for (uint32_t c = 0; c < width; ++c)
            {
                uint64_t at = k * sampleBits + bit + 32 * (uint64_t)c;
                uint32_t q = maniBitRead(stream, at, 32);
                float v;
                memcpy(&v, &q, sizeof v);
                if (!Finite(v))
                {
                    return (uint64_t)(stream - clip->segmentData) + at / 8;
                }
            }
        }
        bit += (uint64_t)width * bits;
    }
    return 0;
}

// A segment's ranges: minimum and extent bytes summing to at most 255,
// so a segment-normalized value stays in [0, 1]. The offset of the first
// extent that does not, or 0.
static uint64_t RangeTooWide(const maniClip* clip, const uint8_t* e)
{
    const uint8_t* rates = clip->segmentData + maniLoadU32(e + 8);
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    const uint8_t* range = rates + n;
    for (uint64_t j = 0; clip->segmentCount > 1 && j < n; ++j)
    {
        uint32_t width = j < clip->animatedChannels ? 3 : 1;
        for (uint32_t k = 0; rates[j] != 0 && k < width; ++k)
        {
            if (range[k] + range[width + k] > 255)
            {
                return (uint64_t)(range + width + k - clip->segmentData);
            }
        }
        range += 2 * (size_t)width;
    }
    return 0;
}

maniFileResult maniCheckClipCompressed(const uint8_t* const* data, const uint64_t* sizes,
                                       const uint64_t* at, const uint64_t* entry, maniClip* clip)
{
    if (sizes[10] != (uint64_t)clip->trackCount + clip->floatTrackCount)
    {
        return Fail(entry[10]);
    }
    Counts counts;
    maniFileResult r = CheckForms(data[10], at[10], clip, &counts);
    if (r.result != mani_success)
    {
        return r;
    }
    if (sizes[11] != 12 * counts.constantChannels + 4 * counts.constantFloats)
    {
        return Fail(entry[11]);
    }
    if (sizes[12] != 24 * counts.animatedChannels + 8 * counts.animatedFloats)
    {
        return Fail(entry[12]);
    }
    if (sizes[13] != (uint64_t)clip->segmentCount * MANI_CLIP_SEGMENT_SIZE)
    {
        return Fail(entry[13]);
    }
    clip->animatedChannels = (uint32_t)counts.animatedChannels;
    clip->animatedFloats = (uint32_t)counts.animatedFloats;
    clip->constantChannels = (uint32_t)counts.constantChannels;
    clip->animatedRotations = (uint32_t)counts.animatedRotations;
    clip->forms = data[10];
    r = CheckAllValues(data, at, clip);
    r = r.result == mani_success
            ? CheckSegments(data[13], at[13], data[14], sizes[14], entry[14], clip)
            : r;
    clip->constants = data[11];
    clip->ranges = data[12];
    clip->segments = data[13];
    clip->segmentData = data[14];
    for (uint32_t s = 0; r.result == mani_success && s < clip->segmentCount; ++s)
    {
        const uint8_t* e = data[13] + (size_t)s * MANI_CLIP_SEGMENT_SIZE;
        uint64_t wrong = RangeTooWide(clip, e);
        wrong = wrong != 0 ? wrong : RawNotFinite(clip, e);
        r = wrong != 0 ? Fail(at[14] + wrong) : r;
    }
    return r;
}

// Where a sample's animated sub-tracks are: its segment's rates and
// ranges, its bits, and how far along them the walk is.
typedef struct Walker
{
    const uint8_t* rates;
    const uint8_t* ranges;
    const uint8_t* stream;
    uint64_t bit;
    bool several;
} Walker;

// The segment holding a sample: segments are about the same length, so
// a division lands on it or next to it.
static const uint8_t* SegmentOf(const maniClip* clip, uint32_t sample)
{
    uint32_t n = clip->segmentCount;
    uint32_t per = clip->sampleCount / n;
    uint32_t s = sample / per < n ? sample / per : n - 1;
    while (s > 0 && maniLoadU32(clip->segments + (size_t)s * MANI_CLIP_SEGMENT_SIZE) > sample)
    {
        --s;
    }
    while (s + 1 < n &&
           maniLoadU32(clip->segments + (size_t)(s + 1) * MANI_CLIP_SEGMENT_SIZE) <= sample)
    {
        ++s;
    }
    return clip->segments + (size_t)s * MANI_CLIP_SEGMENT_SIZE;
}

// A walker at a sample's first bone channel, or (floats) its first float.
static Walker WalkerAt(const maniClip* clip, uint32_t sample, bool floats)
{
    const uint8_t* e = SegmentOf(clip, sample);
    const uint8_t* base = clip->segmentData + maniLoadU32(e + 8);
    uint64_t channelBits = maniLoadU32(e + 12);
    uint64_t sampleBits = channelBits + maniLoadU32(e + 16);
    uint64_t n = (uint64_t)clip->animatedChannels + clip->animatedFloats;
    Walker w = {base, base + n, base + SegmentHead(clip),
                (uint64_t)(sample - maniLoadU32(e)) * sampleBits, clip->segmentCount > 1};
    if (floats)
    {
        w.rates += clip->animatedChannels;
        w.ranges += w.several ? 6 * (size_t)clip->animatedChannels : 0;
        w.bit += channelBits;
    }
    return w;
}

// The next animated sub-track's `width` components at a walker, through
// its clip range (minimum, then extent, per component).
static void Decode(Walker* w, const uint8_t* clipRange, uint32_t width, float* out)
{
    uint32_t rate = *w->rates++;
    const uint8_t* segment = w->ranges;
    w->ranges += w->several ? 2 * (size_t)width : 0;
    uint32_t bits = maniCodecBits(rate);
    for (uint32_t k = 0; k < width; ++k)
    {
        uint32_t q = rate == 0 ? 0 : maniBitRead(w->stream, w->bit, bits);
        w->bit += bits;
        if (rate == MANI_CODEC_RAW_RATE)
        {
            memcpy(&out[k], &q, sizeof q);
            continue;
        }
        float n =
            rate == 0 ? maniCodecUnpack16(maniLoadU16(segment + 2 * k)) : maniCodecUnpack(q, bits);
        if (w->several && rate != 0)
        {
            n = maniCodecSegmentValue(n, segment[k], segment[width + k]);
        }
        out[k] = maniCodecValue(n, LoadF32(clipRange + 4 * k),
                                LoadF32(clipRange + 4 * (size_t)(width + k)));
    }
}

// The walk over the bone channels: constants, clip ranges, and the two
// samples' values, from their bits (walkers) or decoded before (caches,
// when not NULL: rotations rebuilt, four floats; vectors three), the
// same arithmetic after.
typedef struct Reading
{
    Walker a;
    Walker b;
    const float* cacheA;
    const float* cacheB;
    const uint8_t* constant;
    const uint8_t* range;
    float alpha;
} Reading;

// Moves a walker past a sub-track of `width` components unread.
static void Pass(Walker* w, uint32_t width)
{
    uint32_t rate = *w->rates++;
    w->ranges += w->several ? 2 * (size_t)width : 0;
    w->bit += (uint64_t)width * maniCodecBits(rate);
}

// Moves past a bone channel of a form unread.
static void Skip(Reading* r, uint32_t form, uint32_t channel)
{
    if (form == maniClipFormConstant)
    {
        r->constant += 12;
    }
    else if (form == maniClipFormAnimated)
    {
        uint32_t cached = channel == 1 ? 4 : 3;
        r->range += 24;
        r->cacheA += r->cacheA != NULL ? cached : 0;
        r->cacheB += r->cacheB != NULL ? cached : 0;
        if (r->cacheA == NULL)
        {
            Pass(&r->a, 3);
            Pass(&r->b, 3);
        }
    }
}

// The next animated channel's two values: from the caches, or decoded
// (and a rotation rebuilt with its dropped component).
static void Animated(Reading* r, uint32_t channel, uint32_t dropped, float* va, float* vb)
{
    uint32_t width = channel == 1 ? 4 : 3;
    if (r->cacheA != NULL && r->cacheB != NULL)
    {
        memcpy(va, r->cacheA, width * sizeof(float));
        memcpy(vb, r->cacheB, width * sizeof(float));
        r->cacheA += width;
        r->cacheB += width;
    }
    else
    {
        float ka[3];
        float kb[3];
        Decode(&r->a, r->range, 3, ka);
        Decode(&r->b, r->range, 3, kb);
        if (channel == 1)
        {
            maniCodecRotation(ka, dropped, va);
            maniCodecRotation(kb, dropped, vb);
        }
        else
        {
            memcpy(va, ka, sizeof ka);
            memcpy(vb, kb, sizeof kb);
        }
    }
    r->range += 24;
}

// Normalized lerp on the shorter arc: both rotations keep their dropped
// component positive, so the second is negated when they point apart.
static void Nlerp(const float a[4], const float b[4], float alpha, float q[4])
{
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    float s = dot < 0.0f ? -1.0f : 1.0f;
    for (int k = 0; k < 4; ++k)
    {
        q[k] = a[k] + (s * b[k] - a[k]) * alpha;
    }
    maniCodecNormalize(q);
}

// One stored channel (0 translation, 1 rotation, 2 scale) of a bone into
// its slot, interpolated.
static void Channel(Reading* r, uint32_t forms, uint32_t channel, float* slot)
{
    float va[4];
    float vb[4];
    if (Form(forms, channel) == maniClipFormConstant)
    {
        float kept[3];
        for (int k = 0; k < 3; ++k)
        {
            kept[k] = LoadF32(r->constant + 4 * k);
        }
        r->constant += 12;
        if (channel == 1)
        {
            maniCodecRotation(kept, Dropped(forms), va);
        }
        else
        {
            memcpy(va, kept, sizeof kept);
        }
        memcpy(vb, va, sizeof va);
    }
    else
    {
        Animated(r, channel, Dropped(forms), va, vb);
    }
    float out[4];
    if (channel == 1)
    {
        Nlerp(va, vb, r->alpha, out);
        maniPoseSlotPut(slot, 3, out, 4);
        return;
    }
    for (int k = 0; k < 3; ++k)
    {
        out[k] = va[k] + (vb[k] - va[k]) * r->alpha;
    }
    maniPoseSlotPut(slot, maniChannelFirst(channel), out, 3);
}

#if defined(MANI_CLIP_LANES)
// Up to eight stored rotations waiting for one pass on the lanes: their
// kept components at both samples, dropped components and slots.
typedef struct Rotations
{
    float a[3][MANI_POSE_GROUP_BONES];
    float b[3][MANI_POSE_GROUP_BONES];
    float dropped[MANI_POSE_GROUP_BONES];
    float* slots[MANI_POSE_GROUP_BONES];
    uint32_t count;
} Rotations;

// maniCodecRebuild on the lanes: sqrt(|((1 - a*a) - b*b) - c*c|), the
// absolute value as fabsf's bits.
static manif8 Rebuild(manif8 a, manif8 b, manif8 c)
{
    uint32_t bits = 0x7FFFFFFFu;
    float magnitude;
    memcpy(&magnitude, &bits, sizeof magnitude);
    manif8 squared = maniF8Sub(maniF8Set1(1.0f), maniF8Mul(a, a));
    squared = maniF8Sub(squared, maniF8Mul(b, b));
    squared = maniF8Sub(squared, maniF8Mul(c, c));
    return maniF8Sqrt(maniF8And(squared, maniF8Set1(magnitude)));
}

// maniCodecRotation on the lanes: the rebuilt component at its place.
static void Place(const float kept[3][MANI_POSE_GROUP_BONES], manif8 dropped, manif8 q[4])
{
    manif8 a = maniF8Load(kept[0]);
    manif8 b = maniF8Load(kept[1]);
    manif8 c = maniF8Load(kept[2]);
    manif8 r = Rebuild(a, b, c);
    manif8 d0 = maniF8EQ(dropped, maniF8Zero());
    manif8 d1 = maniF8EQ(dropped, maniF8Set1(1.0f));
    manif8 d2 = maniF8EQ(dropped, maniF8Set1(2.0f));
    manif8 d3 = maniF8EQ(dropped, maniF8Set1(3.0f));
    q[0] = maniF8Select(d0, r, a);
    q[1] = maniF8Select(d0, a, maniF8Select(d1, r, b));
    q[2] = maniF8Select(maniF8LT(dropped, maniF8Set1(2.0f)), b, maniF8Select(d2, r, c));
    q[3] = maniF8Select(d3, r, c);
}

// The waiting rotations rebuilt, interpolated on the shorter arc and
// normalized as Nlerp does, each into its slot.
static void Flush(Rotations* batch, float alpha)
{
    for (uint32_t i = batch->count; i < MANI_POSE_GROUP_BONES; ++i)
    {
        for (int k = 0; k < 3; ++k)
        {
            batch->a[k][i] = 0.0f;
            batch->b[k][i] = 0.0f;
        }
        batch->dropped[i] = 3.0f;
    }
    manif8 dropped = maniF8Load(batch->dropped);
    manif8 qa[4];
    manif8 qb[4];
    Place(batch->a, dropped, qa);
    Place(batch->b, dropped, qb);
    manif8 dot = maniF8Mul(qa[0], qb[0]);
    dot = maniF8Add(dot, maniF8Mul(qa[1], qb[1]));
    dot = maniF8Add(dot, maniF8Mul(qa[2], qb[2]));
    dot = maniF8Add(dot, maniF8Mul(qa[3], qb[3]));
    manif8 s = maniF8Select(maniF8LT(dot, maniF8Zero()), maniF8Set1(-1.0f), maniF8Set1(1.0f));
    manif8 t = maniF8Set1(alpha);
    float group[MANI_POSE_COMPONENTS * MANI_POSE_GROUP_BONES];
    for (int k = 0; k < 4; ++k)
    {
        manif8 v = maniF8Add(qa[k], maniF8Mul(maniF8Sub(maniF8Mul(s, qb[k]), qa[k]), t));
        maniF8Store(group + (3 + k) * MANI_POSE_GROUP_BONES, v);
    }
    maniBlendLanesNormalize(group);
    for (uint32_t i = 0; i < batch->count; ++i)
    {
        for (int k = 0; k < 4; ++k)
        {
            batch->slots[i][(3 + k) * MANI_POSE_GROUP_BONES] =
                group[(3 + k) * MANI_POSE_GROUP_BONES + i];
        }
    }
    batch->count = 0;
}

// A stored rotation's kept components at both samples into the batch,
// read as Channel reads them; the batch passed on the lanes when full.
static void Batch(Reading* r, Rotations* batch, uint32_t forms, float* slot)
{
    uint32_t i = batch->count;
    float ka[3];
    float kb[3];
    if (Form(forms, 1) == maniClipFormConstant)
    {
        for (int k = 0; k < 3; ++k)
        {
            ka[k] = LoadF32(r->constant + 4 * k);
        }
        r->constant += 12;
        memcpy(kb, ka, sizeof ka);
    }
    else
    {
        Decode(&r->a, r->range, 3, ka);
        Decode(&r->b, r->range, 3, kb);
        r->range += 24;
    }
    for (int k = 0; k < 3; ++k)
    {
        batch->a[k][i] = ka[k];
        batch->b[k][i] = kb[k];
    }
    batch->dropped[i] = (float)Dropped(forms);
    batch->slots[i] = slot;
    batch->count = i + 1;
    if (batch->count == MANI_POSE_GROUP_BONES)
    {
        Flush(batch, r->alpha);
    }
}
#endif

void maniSampleCompressedBones(const maniClip* clip, const maniSkeleton* skeleton,
                               const maniClipPair* pair, const maniBoneList* list, maniPose* pose)
{
    Reading r = {.cacheA = pair->cacheA,
                 .cacheB = pair->cacheB,
                 .constant = clip->constants,
                 .range = clip->ranges,
                 .alpha = pair->alpha};
    if (pair->cacheA == NULL)
    {
        r.a = WalkerAt(clip, pair->first, false);
        r.b = WalkerAt(clip, pair->second, false);
    }
    // Every bone (or listed one) at rest, then each stored channel over
    // its rest: the tracks are in bone order, each bone once (checked at
    // open), and an unlisted bone's channels are passed unread.
#if defined(MANI_CLIP_LANES)
    Rotations rotations;
    rotations.count = 0;
#endif
    float* locals = maniPoseLocals(pose);
    maniFillRests(locals, skeleton, clip->skeletonBoneCount, clip->additive != maniAdditiveNone,
                  list);
    uint32_t next = 0;
    for (uint32_t t = 0; t < clip->trackCount; ++t)
    {
        uint32_t bone = maniLoadU16(clip->tracks + (size_t)t * MANI_CLIP_TRACK_SIZE);
        uint32_t forms = clip->forms[t];
        bool wanted = maniBoneListed(list, bone, &next);
        float* slot = maniPoseSlot(locals, bone);
        for (uint32_t c = 0; c < 3; ++c)
        {
            uint32_t form = Form(forms, c);
            if (!wanted)
            {
                Skip(&r, form, c);
            }
#if defined(MANI_CLIP_LANES)
            else if (form >= maniClipFormConstant && c == 1 && r.cacheA == NULL)
            {
                Batch(&r, &rotations, forms, slot);
            }
#endif
            else if (form >= maniClipFormConstant)
            {
                Channel(&r, forms, c, slot);
            }
        }
    }
#if defined(MANI_CLIP_LANES)
    if (rotations.count > 0)
    {
        Flush(&rotations, r.alpha);
    }
#endif
    maniPoseMarkStale(pose);
}

#if defined(MANI_CLIP_LANES)
// A constant channel's value: a rotation rebuilt with its dropped
// component.
static void HeldConstant(const uint8_t* constant, uint32_t channel, uint32_t dropped, float* v)
{
    float kept[3];
    for (int k = 0; k < 3; ++k)
    {
        kept[k] = LoadF32(constant + 4 * k);
    }
    if (channel == 1)
    {
        maniCodecRotation(kept, dropped, v);
    }
    else
    {
        memcpy(v, kept, sizeof kept);
    }
}

// The lanes past the last bone: the identity, never interpolated.
static void HeldPadding(uint32_t bones, float* groups, float* masks)
{
    static const float kIdentity[MANI_POSE_COMPONENTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                                          0.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    for (uint32_t bone = bones; bone % MANI_POSE_GROUP_BONES != 0; ++bone)
    {
        maniPoseSlotPut(maniPoseSlot(groups, bone), 0, kIdentity, MANI_POSE_COMPONENTS);
        for (uint32_t c = 0; c < 3; ++c)
        {
            masks[(size_t)(bone / MANI_POSE_GROUP_BONES) * 3 * MANI_POSE_GROUP_BONES +
                  c * MANI_POSE_GROUP_BONES + bone % MANI_POSE_GROUP_BONES] = 0.0f;
        }
    }
}

// A held sample (maniDecodeCompressedSample's floats) in a pose's group
// layout for the forward cursor: each channel the clip stores at its
// value (a constant read, a rotation rebuilt), the others at rest as the
// walk writes them, padding lanes the identity; in masks (three rows of
// eight a group: translation, rotation, scale) all bits set where the
// walk interpolates the channel, clear where it writes a rest.
void maniExpandCompressedSample(const maniClip* clip, const maniSkeleton* skeleton,
                                const float* cache, float* groups, float* masks)
{
    uint32_t bones = clip->skeletonBoneCount;
    const uint8_t* constant = clip->constants;
    bool identity = clip->additive != maniAdditiveNone;
    uint32_t track = 0;
    for (uint32_t bone = 0; bone < bones; ++bone)
    {
        float* slot = maniPoseSlot(groups, bone);
        float* mask = masks + (size_t)(bone / MANI_POSE_GROUP_BONES) * 3 * MANI_POSE_GROUP_BONES +
                      bone % MANI_POSE_GROUP_BONES;
        bool tracked = track < clip->trackCount &&
                       maniLoadU16(clip->tracks + (size_t)track * MANI_CLIP_TRACK_SIZE) == bone;
        uint32_t forms = tracked ? clip->forms[track] : 0;
        track += tracked ? 1 : 0;
        for (uint32_t c = 0; c < 3; ++c)
        {
            uint32_t form = Form(forms, c);
            uint32_t width = c == 1 ? 4 : 3;
            float v[4];
            if (form == maniClipFormConstant)
            {
                HeldConstant(constant, c, Dropped(forms), v);
                constant += 12;
            }
            else if (form == maniClipFormAnimated)
            {
                memcpy(v, cache, width * sizeof(float));
                cache += width;
            }
            else
            {
                maniSlotRest(slot, skeleton, bone, c, identity);
                mask[c * MANI_POSE_GROUP_BONES] = 0.0f;
                continue;
            }
            maniPoseSlotPut(slot, maniChannelFirst(c), v, width);
            uint32_t all = UINT32_MAX;
            memcpy(&mask[c * MANI_POSE_GROUP_BONES], &all, sizeof all);
        }
    }
    HeldPadding(bones, groups, masks);
}
#endif

// The walk over the float tracks.
typedef struct FloatReading
{
    Walker a;
    Walker b;
    const float* cacheA;
    const float* cacheB;
    const uint8_t* constant;
    const uint8_t* range;
} FloatReading;

void maniSampleCompressedFloats(const maniClip* clip, const maniClipPair* pair, float* values)
{
    size_t channels = 4 * (size_t)clip->animatedRotations +
                      3 * (size_t)(clip->animatedChannels - clip->animatedRotations);
    FloatReading r = {.cacheA = pair->cacheA != NULL ? pair->cacheA + channels : NULL,
                      .cacheB = pair->cacheB != NULL ? pair->cacheB + channels : NULL,
                      // The bone channels' constants and ranges come first.
                      .constant = clip->constants + 12 * (size_t)clip->constantChannels,
                      .range = clip->ranges + 24 * (size_t)clip->animatedChannels};
    if (pair->cacheA == NULL)
    {
        r.a = WalkerAt(clip, pair->first, true);
        r.b = WalkerAt(clip, pair->second, true);
    }
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        float va = 0.0f;
        float vb = 0.0f;
        if (clip->forms[clip->trackCount + i] == maniClipFormConstant)
        {
            va = LoadF32(r.constant);
            vb = va;
            r.constant += 4;
        }
        else if (r.cacheA != NULL && r.cacheB != NULL)
        {
            va = *r.cacheA++;
            vb = *r.cacheB++;
        }
        else
        {
            Decode(&r.a, r.range, 1, &va);
            Decode(&r.b, r.range, 1, &vb);
            r.range += 8;
        }
        bool step =
            maniLoadU32(clip->floatTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 16) != 0;
        float alpha = pair->alpha;
        values[i] = step ? (alpha < 1.0f ? va : vb) : va + (vb - va) * alpha;
    }
}

uint32_t maniCompressedSampleFloats(const maniClip* clip)
{
    return 4 * clip->animatedRotations + 3 * (clip->animatedChannels - clip->animatedRotations) +
           clip->animatedFloats;
}

void maniDecodeCompressedSample(const maniClip* clip, uint32_t sample, float* out)
{
    Walker w = WalkerAt(clip, sample, false);
    const uint8_t* range = clip->ranges;
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        uint32_t forms = clip->forms[i];
        for (uint32_t c = 0; c < 3; ++c)
        {
            if (Form(forms, c) != maniClipFormAnimated)
            {
                continue;
            }
            float kept[3];
            Decode(&w, range, 3, kept);
            range += 24;
            if (c == 1)
            {
                maniCodecRotation(kept, Dropped(forms), out);
                out += 4;
            }
            else
            {
                memcpy(out, kept, sizeof kept);
                out += 3;
            }
        }
    }
    for (uint32_t j = 0; j < clip->animatedFloats; ++j)
    {
        Decode(&w, range, 1, out);
        out += 1;
        range += 8;
    }
}
