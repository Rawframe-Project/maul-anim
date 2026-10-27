// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A compressed clip's sections checked once at open, against its header
// and tracks, so its samplers (clip_compressed.c) read without checks.
// Sub-tracks run in kind order: each kind of bone channel (rotation,
// translation, scale) over the tracks that store it, then the float
// tracks.

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_view.h"
#include "container.h"
#include "invariant.h"

#include "maul-anim/clip.h"

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

static bool Bit(const uint8_t* set, uint32_t t)
{
    return ((set[t / 8] >> (t % 8)) & 1u) != 0;
}

// Counts of constant and animated sub-tracks, bone channels by kind and
// float tracks.
typedef struct Counts
{
    uint32_t constant[maniClipKinds];
    uint32_t animated[maniClipKinds];
    uint32_t constantFloats;
    uint32_t animatedFloats;
} Counts;

// One kind's bitsets (at forms, at at) against the tracks' masks: a set
// bit names a track whose mask holds the channel (bits past the last
// track are clear), and no track is in both; the kind's counts.
static maniFileResult CheckKind(const uint8_t* forms, uint64_t at, const maniClip* clip,
                                uint32_t kind, Counts* counts)
{
    uint64_t bytes = maniClipBitsetBytes(clip->trackCount);
    const uint8_t* constant = forms + 2 * kind * bytes;
    const uint8_t* animated = constant + bytes;
    uint32_t channel = maniClipKindChannel(kind);
    for (uint32_t t = 0; t < 8 * bytes; ++t)
    {
        bool c = Bit(constant, t);
        bool a = Bit(animated, t);
        bool present = t < clip->trackCount &&
                       ((clip->tracks[(size_t)t * MANI_CLIP_TRACK_SIZE + 2] >> channel) & 1u) != 0;
        if (c && !present)
        {
            return Fail(at + 2 * kind * bytes + t / 8);
        }
        if (a && (!present || c))
        {
            return Fail(at + (2 * kind + 1) * bytes + t / 8);
        }
        counts->constant[kind] += c ? 1 : 0;
        counts->animated[kind] += a ? 1 : 0;
    }
    return (maniFileResult){mani_success, 0, 0};
}

// Each kind's bitsets; a dropped component only for a stored rotation; a
// float track constant or animated.
static maniFileResult CheckForms(const uint8_t* forms, uint64_t at, const maniClip* clip,
                                 Counts* counts)
{
    *counts = (Counts){{0, 0, 0}, {0, 0, 0}, 0, 0};
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        maniFileResult r = CheckKind(forms, at, clip, kind, counts);
        if (r.result != mani_success)
        {
            return r;
        }
    }
    uint64_t bytes = maniClipBitsetBytes(clip->trackCount);
    const uint8_t* dropped = forms + 2 * maniClipKinds * bytes;
    uint64_t droppedBytes = ((uint64_t)clip->trackCount + 3) / 4;
    for (uint32_t t = 0; t < 4 * droppedBytes; ++t)
    {
        bool stored = t < clip->trackCount && (Bit(forms, t) || Bit(forms + bytes, t));
        if (((dropped[t / 4] >> (2 * (t % 4))) & 3u) != 0 && !stored)
        {
            return Fail(at + 2 * maniClipKinds * bytes + t / 4);
        }
    }
    const uint8_t* floats = dropped + droppedBytes;
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        if (floats[i] != maniClipFormConstant && floats[i] != maniClipFormAnimated)
        {
            return Fail(at + 2 * maniClipKinds * bytes + droppedBytes + i);
        }
        counts->constantFloats += floats[i] == maniClipFormConstant ? 1 : 0;
        counts->animatedFloats += floats[i] == maniClipFormAnimated ? 1 : 0;
    }
    return (maniFileResult){mani_success, 0, 0};
}

// The constants: finite, a rotation's within [-1, 1].
static maniFileResult CheckConstants(const uint8_t* constants, uint64_t at, const Counts* counts)
{
    uint64_t k = 0;
    for (uint32_t kind = 0; kind <= maniClipKinds; ++kind)
    {
        uint64_t n =
            kind < maniClipKinds ? 3 * (uint64_t)counts->constant[kind] : counts->constantFloats;
        for (uint64_t end = k + n; k < end; ++k)
        {
            float v = LoadF32(constants + 4 * k);
            if (!Finite(v) || (kind == maniClipKindRotation && !(v >= -1.0f && v <= 1.0f)))
            {
                return Fail(at + 4 * k);
            }
        }
    }
    return (maniFileResult){mani_success, 0, 0};
}

// The clip ranges, kind by kind: minimum and extent finite, the extent
// not negative, their sum finite, a rotation's minimum at least -1 and
// extent at most 2.
static maniFileResult CheckRanges(const uint8_t* ranges, uint64_t at, const Counts* counts)
{
    uint64_t base = 0;
    for (uint32_t kind = 0; kind <= maniClipKinds; ++kind)
    {
        uint32_t n = kind < maniClipKinds ? counts->animated[kind] : counts->animatedFloats;
        uint32_t width = kind < maniClipKinds ? 3u : 1u;
        bool rotation = kind == maniClipKindRotation;
        for (uint32_t i = 0; i < n; ++i)
        {
            for (uint32_t c = 0; c < width; ++c)
            {
                uint64_t lowAt = base + maniClipRangeAt(n, width, i, c, 0);
                uint64_t extentAt = base + maniClipRangeAt(n, width, i, c, 1);
                float low = LoadF32(ranges + lowAt);
                float extent = LoadF32(ranges + extentAt);
                if (!Finite(low) || (rotation && !(low >= -1.0f)))
                {
                    return Fail(at + lowAt);
                }
                if (!Finite(extent) || !(extent >= 0.0f) || !Finite(low + extent) ||
                    (rotation && !(extent <= 2.0f)))
                {
                    return Fail(at + extentAt);
                }
            }
        }
        base += 8 * (uint64_t)width * n;
    }
    return (maniFileResult){mani_success, 0, 0};
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
        uint64_t head = maniCompressedSegmentHead(clip);
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
    const uint8_t* stream = base + maniCompressedSegmentHead(clip);
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

MANI_COLD maniFileResult maniCheckClipCompressed(const uint8_t* const* data, const uint64_t* sizes,
                                                 const uint64_t* at, const uint64_t* entry,
                                                 maniClip* clip)
{
    if (sizes[10] != maniClipFormsSize(clip->trackCount, clip->floatTrackCount))
    {
        return Fail(entry[10]);
    }
    Counts counts;
    maniFileResult r = CheckForms(data[10], at[10], clip, &counts);
    if (r.result != mani_success)
    {
        return r;
    }
    uint64_t constantChannels = 0;
    uint64_t animatedChannels = 0;
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        constantChannels += counts.constant[kind];
        animatedChannels += counts.animated[kind];
    }
    if (sizes[11] != 12 * constantChannels + 4 * (uint64_t)counts.constantFloats)
    {
        return Fail(entry[11]);
    }
    if (sizes[12] != 24 * animatedChannels + 8 * (uint64_t)counts.animatedFloats)
    {
        return Fail(entry[12]);
    }
    if (sizes[13] != (uint64_t)clip->segmentCount * MANI_CLIP_SEGMENT_SIZE)
    {
        return Fail(entry[13]);
    }
    // At most three channels a track, which the track count bounds.
    clip->animatedChannels = (uint32_t)animatedChannels;
    clip->animatedFloats = counts.animatedFloats;
    clip->constantChannels = (uint32_t)constantChannels;
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        clip->animatedKinds[kind] = counts.animated[kind];
        clip->constantKinds[kind] = counts.constant[kind];
    }
    clip->forms = data[10];
    r = CheckConstants(data[11], at[11], &counts);
    r = r.result == mani_success ? CheckRanges(data[12], at[12], &counts) : r;
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
