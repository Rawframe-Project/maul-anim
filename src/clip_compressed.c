// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A compressed clip (mani-0006): its sections checked once at open, and
// its samples decoded. Sub-tracks run in track order, each bone track's
// channels translation, rotation, scale, then the float tracks; each is
// absent, default (the rest), constant or animated. An animated value
// at a sample is its bits at its segment's rate, through the segment
// range (with several segments) and the clip range.

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
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

// The forms: a channel present in the track's mask has a form, an absent
// one none; a float track is constant or animated. Counts of constant
// and animated sub-tracks, channels and floats apart.
typedef struct Counts
{
    uint64_t constantChannels;
    uint64_t constantFloats;
    uint64_t animatedChannels;
    uint64_t animatedFloats;
} Counts;

static maniFileResult CheckForms(const uint8_t* forms, uint64_t at, const maniClip* clip,
                                 Counts* counts)
{
    *counts = (Counts){0, 0, 0, 0};
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        uint32_t mask = clip->tracks[(size_t)i * MANI_CLIP_TRACK_SIZE + 2];
        uint32_t byte = forms[i];
        bool right = byte < 64;
        for (uint32_t c = 0; right && c < 3; ++c)
        {
            uint32_t f = Form(byte, c);
            right = ((mask >> c) & 1u) != 0 ? f != maniClipFormAbsent : f == maniClipFormAbsent;
            counts->constantChannels += f == maniClipFormConstant ? 1 : 0;
            counts->animatedChannels += f == maniClipFormAnimated ? 1 : 0;
        }
        if (!right)
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
                                    uint64_t dataSize, const maniClip* clip)
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
    return offset + 8 == dataSize
               ? (maniFileResult){mani_success, 0, 0}
               : Fail(MANI_CONTAINER_HEADER_SIZE + 14 * (uint64_t)MANI_CONTAINER_ENTRY_SIZE + 16);
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

static uint64_t EntrySize(uint32_t section)
{
    return MANI_CONTAINER_HEADER_SIZE + (uint64_t)section * MANI_CONTAINER_ENTRY_SIZE + 16;
}

maniFileResult maniCheckClipCompressed(const uint8_t* const* data, const uint64_t* sizes,
                                       const uint64_t* at, maniClip* clip)
{
    if (sizes[10] != (uint64_t)clip->trackCount + clip->floatTrackCount)
    {
        return Fail(EntrySize(10));
    }
    Counts counts;
    maniFileResult r = CheckForms(data[10], at[10], clip, &counts);
    if (r.result != mani_success)
    {
        return r;
    }
    if (sizes[11] != 12 * counts.constantChannels + 4 * counts.constantFloats)
    {
        return Fail(EntrySize(11));
    }
    if (sizes[12] != 24 * counts.animatedChannels + 8 * counts.animatedFloats)
    {
        return Fail(EntrySize(12));
    }
    if (sizes[13] != (uint64_t)clip->segmentCount * MANI_CLIP_SEGMENT_SIZE)
    {
        return Fail(EntrySize(13));
    }
    clip->animatedChannels = (uint32_t)counts.animatedChannels;
    clip->animatedFloats = (uint32_t)counts.animatedFloats;
    clip->forms = data[10];
    r = CheckAllValues(data, at, clip);
    r = r.result == mani_success ? CheckSegments(data[13], at[13], data[14], sizes[14], clip) : r;
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

static maniQuat Rotation(const float xyz[3])
{
    return (maniQuat){xyz[0], xyz[1], xyz[2], maniCodecRebuildW(xyz[0], xyz[1], xyz[2])};
}

// Normalized lerp on the shorter arc: both rotations keep w positive, so
// the second is negated when they point apart.
static maniQuat Nlerp(maniQuat a, maniQuat b, float alpha)
{
    float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    float s = dot < 0.0f ? -1.0f : 1.0f;
    maniQuat q = {a.x + (s * b.x - a.x) * alpha, a.y + (s * b.y - a.y) * alpha,
                  a.z + (s * b.z - a.z) * alpha, a.w + (s * b.w - a.w) * alpha};
    return maniQuatNormalize(q);
}

static maniVec3 Lerp(const float a[3], const float b[3], float alpha)
{
    return (maniVec3){a[0] + (b[0] - a[0]) * alpha, a[1] + (b[1] - a[1]) * alpha,
                      a[2] + (b[2] - a[2]) * alpha};
}

// The walk over one bone channel's sub-track: its constant or its two
// animated values, the cursors moved past it.
typedef struct Reading
{
    Walker a;
    Walker b;
    const uint8_t* constant;
    const uint8_t* range;
} Reading;

// One channel (0 translation, 1 rotation, 2 scale) of a local; false
// when it is absent or the rest.
static bool Channel(Reading* r, uint32_t form, uint32_t channel, float alpha, maniTransform* local)
{
    float va[3];
    float vb[3];
    if (form == maniClipFormConstant)
    {
        for (int k = 0; k < 3; ++k)
        {
            va[k] = LoadF32(r->constant + 4 * k);
            vb[k] = va[k];
        }
        r->constant += 12;
    }
    else if (form == maniClipFormAnimated)
    {
        Decode(&r->a, r->range, 3, va);
        Decode(&r->b, r->range, 3, vb);
        r->range += 24;
    }
    else
    {
        return false;
    }
    if (channel == 1)
    {
        local->rotation = Nlerp(Rotation(va), Rotation(vb), alpha);
    }
    else if (channel == 0)
    {
        local->translation = Lerp(va, vb, alpha);
    }
    else
    {
        local->scale = Lerp(va, vb, alpha);
    }
    return true;
}

void maniSampleCompressedBones(const maniClip* clip, uint32_t first, uint32_t second, float alpha,
                               maniPose* pose)
{
    Reading r = {WalkerAt(clip, first, false), WalkerAt(clip, second, false), clip->constants,
                 clip->ranges};
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        uint32_t bone = maniLoadU16(clip->tracks + (size_t)i * MANI_CLIP_TRACK_SIZE);
        uint32_t forms = clip->forms[i];
        maniTransform local = maniPoseLocal(pose, bone);
        bool changed = false;
        for (uint32_t c = 0; c < 3; ++c)
        {
            changed = Channel(&r, Form(forms, c), c, alpha, &local) || changed;
        }
        if (changed)
        {
            maniPoseSetLocal(pose, bone, local);
        }
    }
}

void maniSampleCompressedFloats(const maniClip* clip, uint32_t first, uint32_t second, float alpha,
                                float* values)
{
    Walker a = WalkerAt(clip, first, true);
    Walker b = WalkerAt(clip, second, true);
    const uint8_t* range = clip->ranges + 24 * (size_t)clip->animatedChannels;
    // The bone channels' constants come first: skip them.
    const uint8_t* constant = clip->constants;
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        for (uint32_t c = 0; c < 3; ++c)
        {
            constant += Form(clip->forms[i], c) == maniClipFormConstant ? 12 : 0;
        }
    }
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        float va = 0.0f;
        float vb = 0.0f;
        if (clip->forms[clip->trackCount + i] == maniClipFormConstant)
        {
            va = LoadF32(constant);
            vb = va;
            constant += 4;
        }
        else
        {
            Decode(&a, range, 1, &va);
            Decode(&b, range, 1, &vb);
            range += 8;
        }
        bool step =
            maniLoadU32(clip->floatTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 16) != 0;
        values[i] = step ? (alpha < 1.0f ? va : vb) : va + (vb - va) * alpha;
    }
}
