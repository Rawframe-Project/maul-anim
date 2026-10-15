// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Compression of a full-precision clip (mani-0006). Sub-tracks (each bone
// track's channels, then the float tracks) are folded to default or
// constant where every sample allows it, the rest range-reduced over
// the clip and, with several segments, over each segment, and stored per
// segment at a bit rate that keeps every sample within its tolerance.
// The rates chosen here are each sub-track's own fewest bits; the shell
// metric reports what the whole pose then does.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_shell.h"
#include "clip_view.h"
#include "container.h"

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <float.h>
#include <math.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stdckdint.h>
#include <stdint.h>
#include <string.h>

enum
{
    kTranslation = 0,
    kRotation = 1,
    kScale = 2,
    kFloat = 3,
};

typedef struct SubTrack
{
    // The bone track and channel, or the float track (channel kFloat).
    uint32_t track;
    uint32_t channel;
    // Where its value starts in a sample: floats into the bone samples,
    // or the float track's index.
    uint32_t offset;
    uint32_t form;
    // Its index among the animated sub-tracks.
    uint32_t animated;
    float tolerance;
    float constant[3];
    float minimum[3];
    float extent[3];
} SubTrack;

typedef struct Plan
{
    const maniClip* clip;
    const maniSkeleton* skeleton;
    const maniAllocator* allocator;
    SubTrack* subs;
    uint32_t subCount;
    uint32_t animatedChannels;
    uint32_t animatedFloats;
    uint32_t constantChannels;
    uint32_t constantFloats;
    uint32_t segmentCount;
    // First samples, segmentCount + 1 of them.
    uint32_t* firsts;
    // Per segment and animated sub-track: its rate and six range bytes.
    uint8_t* rates;
    uint8_t* segmentRanges;
} Plan;

static uint32_t Width(const SubTrack* t)
{
    return t->channel == kFloat ? 1u : 3u;
}

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static void StoreF32(uint8_t* p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(p, bits);
}

// A sub-track's value at a sample; a rotation as x, y, z with w made
// positive.
static void Value(const Plan* p, const SubTrack* t, uint32_t sample, float out[3])
{
    const maniClip* c = p->clip;
    if (t->channel == kFloat)
    {
        out[0] = LoadF32(c->floatSamples + 4 * ((size_t)sample * c->floatTrackCount + t->offset));
        return;
    }
    const uint8_t* v = c->samples + 4 * ((size_t)sample * c->floatsPerSample + t->offset);
    float sign = t->channel == kRotation && LoadF32(v + 12) < 0.0f ? -1.0f : 1.0f;
    for (int k = 0; k < 3; ++k)
    {
        out[k] = sign * LoadF32(v + 4 * k);
    }
}

// The rest value of a bone channel, as Value gives it.
static void Rest(const Plan* p, const SubTrack* t, float out[3])
{
    uint32_t bone = maniLoadU16(p->clip->tracks + (size_t)t->track * MANI_CLIP_TRACK_SIZE);
    maniTransform rest = maniSkeletonRest(p->skeleton, bone);
    const float* v = t->channel == kTranslation ? &rest.translation.x
                     : t->channel == kScale     ? &rest.scale.x
                                                : &rest.rotation.x;
    float sign = t->channel == kRotation && rest.rotation.w < 0.0f ? -1.0f : 1.0f;
    for (int k = 0; k < 3; ++k)
    {
        out[k] = sign * v[k];
    }
}

static bool Same(const float a[3], const float b[3], uint32_t width)
{
    for (uint32_t k = 0; k < width; ++k)
    {
        if (a[k] != b[k])
        {
            return false;
        }
    }
    return true;
}

// Default (every sample the rest), constant (every sample the first) or
// animated, with the clip range of an animated one.
static void Classify(const Plan* p, SubTrack* t)
{
    uint32_t width = Width(t);
    float rest[3] = {0.0f, 0.0f, 0.0f};
    if (t->channel != kFloat)
    {
        Rest(p, t, rest);
    }
    float first[3] = {0.0f, 0.0f, 0.0f};
    Value(p, t, 0, first);
    bool isRest = t->channel != kFloat && Same(first, rest, width);
    bool constant = true;
    float low[3] = {first[0], first[1], first[2]};
    float high[3] = {first[0], first[1], first[2]};
    for (uint32_t k = 1; k < p->clip->sampleCount; ++k)
    {
        float v[3] = {0.0f, 0.0f, 0.0f};
        Value(p, t, k, v);
        constant = constant && Same(v, first, width);
        for (uint32_t c = 0; c < width; ++c)
        {
            low[c] = v[c] < low[c] ? v[c] : low[c];
            high[c] = v[c] > high[c] ? v[c] : high[c];
        }
    }
    t->form =
        constant ? (isRest ? maniClipFormDefault : maniClipFormConstant) : maniClipFormAnimated;
    for (uint32_t c = 0; c < width; ++c)
    {
        t->constant[c] = first[c];
        t->minimum[c] = low[c];
        t->extent[c] = high[c] - low[c];
    }
}

// The segments: one up to MANI_CLIP_SEGMENT_MAX samples, else of
// MANI_CLIP_SEGMENT_IDEAL, the last one's shortfall spread over the
// first ones, one sample each.
static uint32_t SegmentCount(uint32_t samples)
{
    return samples <= MANI_CLIP_SEGMENT_MAX
               ? 1u
               : (samples + MANI_CLIP_SEGMENT_IDEAL - 1) / MANI_CLIP_SEGMENT_IDEAL;
}

static void Split(uint32_t samples, uint32_t segments, uint32_t* firsts)
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

// A value's clip-normalized form.
static float Normalized(const SubTrack* t, uint32_t c, float v)
{
    return t->extent[c] > 0.0f ? (v - t->minimum[c]) / t->extent[c] : 0.0f;
}

// The value a component plays at a rate: encoded and decoded as the
// sampler decodes it. range holds the segment's bytes (min, extent per
// component, or the 16-bit value at rate 0); NULL with one segment.
static float Played(const SubTrack* t, uint32_t c, float v, uint32_t rate, const uint8_t* range)
{
    uint32_t width = Width(t);
    if (rate == MANI_CODEC_RAW_RATE)
    {
        return v;
    }
    if (rate == 0)
    {
        return maniCodecValue(maniCodecUnpack16(maniLoadU16(range + 2 * c)), t->minimum[c],
                              t->extent[c]);
    }
    float n = Normalized(t, c, v);
    if (range != NULL)
    {
        float low = maniCodecRangeByte(range[c]);
        float size = maniCodecRangeByte(range[width + c]);
        n = size > 0.0f ? (n - low) / size : 0.0f;
    }
    float q = maniCodecUnpack(maniCodecPack(n, rate), rate);
    if (range != NULL)
    {
        q = maniCodecSegmentValue(q, range[c], range[width + c]);
    }
    return maniCodecValue(q, t->minimum[c], t->extent[c]);
}

// Whether every sample of a segment plays within the tolerance at a rate.
static bool Fits(const Plan* p, const SubTrack* t, uint32_t s, uint32_t rate, const uint8_t* range)
{
    for (uint32_t k = p->firsts[s]; k < p->firsts[s + 1]; ++k)
    {
        float v[3] = {0.0f, 0.0f, 0.0f};
        Value(p, t, k, v);
        for (uint32_t c = 0; c < Width(t); ++c)
        {
            if (!(fabsf(Played(t, c, v[c], rate, range) - v[c]) <= t->tolerance))
            {
                return false;
            }
        }
    }
    return true;
}

// A segment's range bytes for a sub-track, and the 16-bit constant it
// would hold at rate 0 (the middle of the segment's values).
static void SegmentRange(const Plan* p, const SubTrack* t, uint32_t s, uint8_t range[6],
                         uint8_t constant[6])
{
    for (uint32_t c = 0; c < Width(t); ++c)
    {
        float low = 1.0f;
        float high = 0.0f;
        for (uint32_t k = p->firsts[s]; k < p->firsts[s + 1]; ++k)
        {
            float v[3] = {0.0f, 0.0f, 0.0f};
            Value(p, t, k, v);
            float n = Normalized(t, c, v[c]);
            n = n > 0.0f ? (n < 1.0f ? n : 1.0f) : 0.0f;
            low = n < low ? n : low;
            high = n > high ? n : high;
        }
        maniCodecSegmentRange(low, high, &range[c], &range[Width(t) + c]);
        maniStoreU16(constant + 2 * c, maniCodecPack16(low + (high - low) * 0.5f));
    }
}

// Each animated sub-track's rate and range bytes in each segment: rate 0
// where the segment allows it, else the fewest bits that fit, else raw.
static void ChooseRates(Plan* p)
{
    bool several = p->segmentCount > 1;
    for (uint32_t s = 0; s < p->segmentCount; ++s)
    {
        for (uint32_t i = 0; i < p->subCount; ++i)
        {
            const SubTrack* t = &p->subs[i];
            if (t->form != maniClipFormAnimated)
            {
                continue;
            }
            size_t at = (size_t)s * (p->animatedChannels + p->animatedFloats) + t->animated;
            uint8_t* range = p->segmentRanges + 6 * at;
            uint8_t constant[6] = {0};
            SegmentRange(p, t, s, range, constant);
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

// The bone channels and float tracks as sub-tracks, with the plain
// tolerances: a channel's precision (a rotation's over its shell
// distance), a float track's own.
static void Gather(Plan* p, const maniCompressOptions* o)
{
    const maniClip* c = p->clip;
    uint32_t offset = 0;
    p->subCount = 0;
    for (uint32_t i = 0; i < c->trackCount; ++i)
    {
        const uint8_t* entry = c->tracks + (size_t)i * MANI_CLIP_TRACK_SIZE;
        uint32_t bone = maniLoadU16(entry);
        float precision = o->bonePrecisions != NULL ? o->bonePrecisions[bone] : o->precision;
        float shell =
            o->boneShellDistances != NULL ? o->boneShellDistances[bone] : o->shellDistance;
        for (uint32_t ch = 0; ch < 3; ++ch)
        {
            if (((entry[2] >> ch) & 1u) == 0)
            {
                continue;
            }
            p->subs[p->subCount++] = (SubTrack){
                .track = i,
                .channel = ch,
                .offset = offset,
                .tolerance = ch == kRotation ? precision / shell : precision,
            };
            offset += ch == kRotation ? 4u : 3u;
        }
    }
    for (uint32_t i = 0; i < c->floatTrackCount; ++i)
    {
        float precision = o->floatPrecisions != NULL ? o->floatPrecisions[i] : o->floatPrecision;
        p->subs[p->subCount++] =
            (SubTrack){.track = i, .channel = kFloat, .offset = i, .tolerance = precision};
    }
}

// Every sub-track classified, the animated ones numbered.
static void Fold(Plan* p)
{
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        SubTrack* t = &p->subs[i];
        Classify(p, t);
        bool channel = t->channel != kFloat;
        if (t->form == maniClipFormConstant)
        {
            p->constantChannels += channel ? 1 : 0;
            p->constantFloats += channel ? 0 : 1;
        }
        if (t->form == maniClipFormAnimated)
        {
            t->animated = channel ? p->animatedChannels++ : p->animatedFloats++;
        }
    }
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        p->subs[i].animated +=
            p->subs[i].channel == kFloat && p->subs[i].form == maniClipFormAnimated
                ? p->animatedChannels
                : 0;
    }
}

// The compressed sections (11 to 15), in one block.
typedef struct Sections
{
    uint8_t* block;
    size_t blockSize;
    uint64_t sizes[5];
    uint8_t* data[5];
} Sections;

// A segment's bits per sample: bone channels and floats.
static void SegmentBits(const Plan* p, uint32_t s, uint32_t* channelBits, uint32_t* floatBits)
{
    uint32_t n = p->animatedChannels + p->animatedFloats;
    *channelBits = 0;
    *floatBits = 0;
    for (uint32_t j = 0; j < n; ++j)
    {
        uint32_t bits = maniCodecBits(p->rates[(size_t)s * n + j]);
        *channelBits += j < p->animatedChannels ? 3 * bits : 0;
        *floatBits += j < p->animatedChannels ? 0 : bits;
    }
}

static uint64_t SegmentDataSize(const Plan* p)
{
    uint64_t n = (uint64_t)p->animatedChannels + p->animatedFloats;
    uint64_t ranges = 6 * (uint64_t)p->animatedChannels + 2 * (uint64_t)p->animatedFloats;
    uint64_t size = 8;
    for (uint32_t s = 0; s < p->segmentCount; ++s)
    {
        uint32_t channelBits = 0;
        uint32_t floatBits = 0;
        SegmentBits(p, s, &channelBits, &floatBits);
        uint64_t count = p->firsts[s + 1] - p->firsts[s];
        size += n + (p->segmentCount > 1 ? ranges : 0) +
                (((uint64_t)channelBits + floatBits) * count + 7) / 8;
    }
    return size;
}

// Forms, constants and clip ranges.
static void WriteFront(const Plan* p, Sections* out)
{
    uint8_t* constant = out->data[1];
    uint8_t* range = out->data[2];
    memset(out->data[0], 0, (size_t)out->sizes[0]);
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        const SubTrack* t = &p->subs[i];
        uint32_t width = Width(t);
        if (t->channel == kFloat)
        {
            out->data[0][p->clip->trackCount + t->track] = (uint8_t)t->form;
        }
        else
        {
            out->data[0][t->track] |= (uint8_t)(t->form << (2 * t->channel));
        }
        for (uint32_t c = 0; t->form == maniClipFormConstant && c < width; ++c)
        {
            StoreF32(constant, t->constant[c]);
            constant += 4;
        }
        for (uint32_t c = 0; t->form == maniClipFormAnimated && c < width; ++c)
        {
            StoreF32(range + 4 * c, t->minimum[c]);
            StoreF32(range + 4 * (width + c), t->extent[c]);
        }
        range += t->form == maniClipFormAnimated ? 8 * (size_t)width : 0;
    }
}

// One sample's bits in a segment.
static void WriteSample(const Plan* p, uint32_t s, uint32_t k, maniBitWriter* w)
{
    uint32_t n = p->animatedChannels + p->animatedFloats;
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        const SubTrack* t = &p->subs[i];
        if (t->form != maniClipFormAnimated)
        {
            continue;
        }
        size_t at = (size_t)s * n + t->animated;
        uint32_t rate = p->rates[at];
        const uint8_t* range = p->segmentCount > 1 ? p->segmentRanges + 6 * at : NULL;
        float v[3] = {0.0f, 0.0f, 0.0f};
        Value(p, t, k, v);
        for (uint32_t c = 0; rate != 0 && c < Width(t); ++c)
        {
            uint32_t q = 0;
            if (rate == MANI_CODEC_RAW_RATE)
            {
                memcpy(&q, &v[c], sizeof q);
            }
            else
            {
                float x = Normalized(t, c, v[c]);
                float low = range != NULL ? maniCodecRangeByte(range[c]) : 0.0f;
                float size = range != NULL ? maniCodecRangeByte(range[Width(t) + c]) : 1.0f;
                q = maniCodecPack(size > 0.0f ? (x - low) / size : 0.0f, rate);
            }
            maniBitWrite(w, q, maniCodecBits(rate));
        }
    }
}

// The segment table and data.
static void WriteSegments(const Plan* p, Sections* out)
{
    uint32_t n = p->animatedChannels + p->animatedFloats;
    uint8_t* data = out->data[4];
    memset(data, 0, (size_t)out->sizes[4]);
    uint64_t offset = 0;
    for (uint32_t s = 0; s < p->segmentCount; ++s)
    {
        uint32_t channelBits = 0;
        uint32_t floatBits = 0;
        SegmentBits(p, s, &channelBits, &floatBits);
        uint8_t* e = out->data[3] + (size_t)s * MANI_CLIP_SEGMENT_SIZE;
        maniStoreU32(e, p->firsts[s]);
        maniStoreU32(e + 4, p->firsts[s + 1] - p->firsts[s]);
        maniStoreU32(e + 8, (uint32_t)offset);
        maniStoreU32(e + 12, channelBits);
        maniStoreU32(e + 16, floatBits);
        uint8_t* head = data + offset;
        memcpy(head, p->rates + (size_t)s * n, n);
        head += n;
        for (uint32_t j = 0; p->segmentCount > 1 && j < n; ++j)
        {
            uint32_t width = j < p->animatedChannels ? 3 : 1;
            memcpy(head, p->segmentRanges + 6 * ((size_t)s * n + j), 2 * (size_t)width);
            head += 2 * (size_t)width;
        }
        maniBitWriter w = {head, 0};
        for (uint32_t k = p->firsts[s]; k < p->firsts[s + 1]; ++k)
        {
            WriteSample(p, s, k, &w);
        }
        offset = (uint64_t)(head - data) + (w.bit + 7) / 8;
    }
}

// The named sections (4 to 10) as the clip holds them, the float samples
// left out; empty sections point at a byte so no copy sees NULL.
static const uint8_t kEmpty = 0;

static void NamedSections(const maniClip* c, const uint8_t* data[7], uint64_t sizes[7])
{
    uint64_t keys = 0;
    if (c->discreteTrackCount > 0)
    {
        const uint8_t* last =
            c->discreteTracks + (size_t)(c->discreteTrackCount - 1) * MANI_CLIP_NAMED_TRACK_SIZE;
        keys = (uint64_t)maniLoadU32(last + 16) + maniLoadU32(last + 20);
    }
    const uint8_t* d[7] = {c->floatTracks, &kEmpty,    c->discreteTracks, c->discreteKeys,
                           c->events,      c->markers, c->names};
    uint64_t s[7] = {(uint64_t)c->floatTrackCount * MANI_CLIP_NAMED_TRACK_SIZE,
                     0,
                     (uint64_t)c->discreteTrackCount * MANI_CLIP_NAMED_TRACK_SIZE,
                     keys * MANI_CLIP_KEY_SIZE,
                     (uint64_t)c->eventCount * MANI_CLIP_EVENT_SIZE,
                     (uint64_t)c->markerCount * MANI_CLIP_EVENT_SIZE,
                     c->namesSize};
    for (uint32_t i = 0; i < 7; ++i)
    {
        data[i] = s[i] > 0 && d[i] != NULL ? d[i] : &kEmpty;
        sizes[i] = s[i];
    }
}

static void Header(const maniClip* c, uint32_t segments, uint8_t h[MANI_CLIP_HEADER_SIZE])
{
    memset(h, 0, MANI_CLIP_HEADER_SIZE);
    maniStoreU64(h, c->skeletonIdentity);
    maniStoreU32(h + 8, c->skeletonBoneCount);
    maniStoreU32(h + 12, c->sampleCount);
    StoreF32(h + 16, c->sampleRate);
    StoreF32(h + 20, c->duration);
    const uint32_t fields[8] = {c->loop,
                                c->trackCount,
                                c->floatsPerSample,
                                c->floatTrackCount,
                                c->discreteTrackCount,
                                c->eventCount,
                                c->markerCount,
                                segments};
    for (uint32_t i = 0; i < 8; ++i)
    {
        maniStoreU32(h + 24 + 4 * i, fields[i]);
    }
    maniStoreU32(h + 56, 1);
}

// The file from a planned clip, kept in the builder.
static maniResult Emit(maniBuilder* builder, const Plan* p)
{
    Sections out = {0};
    out.sizes[0] = (uint64_t)p->clip->trackCount + p->clip->floatTrackCount;
    out.sizes[1] = 12 * (uint64_t)p->constantChannels + 4 * (uint64_t)p->constantFloats;
    out.sizes[2] = 24 * (uint64_t)p->animatedChannels + 8 * (uint64_t)p->animatedFloats;
    out.sizes[3] = (uint64_t)p->segmentCount * MANI_CLIP_SEGMENT_SIZE;
    out.sizes[4] = SegmentDataSize(p);
    uint64_t total = 0;
    for (uint32_t i = 0; i < 5; ++i)
    {
        total += out.sizes[i];
    }
    if (total > SIZE_MAX)
    {
        return mani_errorCapacity;
    }
    out.blockSize = (size_t)total;
    out.block = maniAllocate(&builder->allocator, out.blockSize + 1, 1, 8);
    if (out.block == NULL)
    {
        return mani_errorCapacity;
    }
    uint8_t* at = out.block;
    for (uint32_t i = 0; i < 5; ++i)
    {
        out.data[i] = at;
        at += out.sizes[i];
    }
    WriteFront(p, &out);
    WriteSegments(p, &out);
    uint8_t header[MANI_CLIP_HEADER_SIZE];
    Header(p->clip, p->segmentCount, header);
    const uint8_t* sections[MANI_CLIP_COMPRESSED_SECTIONS] = {header, p->clip->tracks, &kEmpty};
    uint64_t sizes[MANI_CLIP_COMPRESSED_SECTIONS] = {
        MANI_CLIP_HEADER_SIZE, (uint64_t)p->clip->trackCount * MANI_CLIP_TRACK_SIZE, 0};
    NamedSections(p->clip, sections + 3, sizes + 3);
    for (uint32_t i = 0; i < 5; ++i)
    {
        sections[10 + i] = out.sizes[i] > 0 ? out.data[i] : &kEmpty;
        sizes[10 + i] = out.sizes[i];
    }
    maniSectionSource table[MANI_CLIP_COMPRESSED_SECTIONS];
    for (uint32_t i = 0; i < MANI_CLIP_COMPRESSED_SECTIONS; ++i)
    {
        table[i] = (maniSectionSource){i + 1, sections[i], sizes[i]};
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(table, MANI_CLIP_COMPRESSED_SECTIONS, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindClip, MANI_CLIP_FORMAT_VERSION,
                           maniClipComputeIdentity(sections, sizes, MANI_CLIP_COMPRESSED_SECTIONS),
                           table, MANI_CLIP_COMPRESSED_SECTIONS);
    }
    maniRelease(&builder->allocator, out.block, out.blockSize + 1, 1, 8);
    if (file == NULL)
    {
        return mani_errorCapacity;
    }
    maniBuilderKeep(builder, maniKindClip, file, (size_t)fileSize, NULL, 0);
    return mani_success;
}

// Scratch for measuring: two poses (in one block, aligned by hand, as
// allocators promise 8 bytes), each pose's matrices, two float rows.
typedef struct Measure
{
    uint8_t* poses;
    size_t posesSize;
    maniShellMatrix* models;
    size_t modelCount;
    float* floats;
    size_t floatCount;
    maniPose* full;
    maniPose* packed;
} Measure;

static void ReleaseMeasure(const maniAllocator* a, Measure* m)
{
    maniRelease(a, m->poses, m->posesSize, 1, 8);
    maniRelease(a, m->models, m->modelCount, sizeof(maniShellMatrix), alignof(maniShellMatrix));
    maniRelease(a, m->floats, m->floatCount, sizeof(float), alignof(float));
}

static maniResult MakeMeasure(const maniAllocator* a, const maniSkeleton* skeleton, uint32_t floats,
                              Measure* m)
{
    size_t pose = maniPoseSize(skeleton);
    m->posesSize = 2 * pose + MANI_POSE_ALIGNMENT;
    m->modelCount = 2 * (size_t)maniSkeletonBoneCount(skeleton);
    m->floatCount = 2 * ((size_t)floats + 1);
    m->poses = maniAllocate(a, m->posesSize, 1, 8);
    m->models = maniAllocate(a, m->modelCount, sizeof(maniShellMatrix), alignof(maniShellMatrix));
    m->floats = maniAllocate(a, m->floatCount, sizeof(float), alignof(float));
    if (m->poses == NULL || m->models == NULL || m->floats == NULL)
    {
        return mani_errorCapacity;
    }
    uint8_t* at = m->poses;
    at += (MANI_POSE_ALIGNMENT - (uintptr_t)at % MANI_POSE_ALIGNMENT) % MANI_POSE_ALIGNMENT;
    maniResult r = maniInitPose(skeleton, at, pose, &m->full);
    return r == mani_success ? maniInitPose(skeleton, at + pose, pose, &m->packed) : r;
}

// The largest shell and float errors of the compressed clip against the
// full one at every sample.
static void Report(const maniClip* full, const maniClip* packed, const maniSkeleton* skeleton,
                   const maniCompressOptions* o, Measure* m, maniCompressReport* report)
{
    uint32_t bones = maniSkeletonBoneCount(skeleton);
    maniShellMatrix* fullModels = m->models;
    maniShellMatrix* packedModels = m->models + bones;
    float* fullFloats = m->floats;
    float* packedFloats = m->floats + m->floatCount / 2;
    for (uint32_t k = 0; k < full->sampleCount; ++k)
    {
        maniSampleClipAt(full, skeleton, k, m->full, fullFloats);
        maniSampleClipAt(packed, skeleton, k, m->packed, packedFloats);
        maniShellModels(skeleton, m->full, fullModels);
        maniShellModels(skeleton, m->packed, packedModels);
        for (uint32_t b = 0; b < bones; ++b)
        {
            double shell = o->boneShellDistances != NULL ? (double)o->boneShellDistances[b]
                                                         : (double)o->shellDistance;
            double e = maniShellError(&fullModels[b], &packedModels[b], shell);
            if (e > report->shellError)
            {
                *report = (maniCompressReport){
                    report->bytes,     e, k, b, report->floatError, report->floatSample,
                    report->floatTrack};
            }
        }
        for (uint32_t i = 0; i < full->floatTrackCount; ++i)
        {
            double e = fabs((double)fullFloats[i] - (double)packedFloats[i]);
            if (e > report->floatError)
            {
                report->floatError = e;
                report->floatSample = k;
                report->floatTrack = i;
            }
        }
    }
}

static bool Positive(float value)
{
    return value > 0.0f && value <= FLT_MAX;
}

static bool OptionsValid(const maniCompressOptions* o, uint32_t bones, uint32_t floats)
{
    bool valid =
        Positive(o->precision) && Positive(o->shellDistance) && Positive(o->floatPrecision);
    for (uint32_t b = 0; valid && b < bones; ++b)
    {
        valid = (o->bonePrecisions == NULL || Positive(o->bonePrecisions[b])) &&
                (o->boneShellDistances == NULL || Positive(o->boneShellDistances[b]));
    }
    for (uint32_t i = 0; valid && o->floatPrecisions != NULL && i < floats; ++i)
    {
        valid = Positive(o->floatPrecisions[i]);
    }
    return valid;
}

maniCompressOptions maniDefaultCompressOptions(void)
{
    return (maniCompressOptions){
        .precision = 0.0001f,
        .shellDistance = 0.03f,
        .floatPrecision = 0.00001f,
    };
}

static void ReleasePlan(Plan* p, uint32_t subs)
{
    size_t cells = (size_t)p->segmentCount * subs;
    maniRelease(p->allocator, p->subs, (size_t)subs + 1, sizeof(SubTrack), alignof(SubTrack));
    maniRelease(p->allocator, p->firsts, (size_t)p->segmentCount + 1, sizeof(uint32_t),
                alignof(uint32_t));
    maniRelease(p->allocator, p->rates, 7 * cells + 1, 1, 1);
}

// The plan's arrays: sub-tracks, segment starts, and per segment and
// sub-track a rate and six range bytes.
static maniResult Prepare(Plan* p, uint32_t subs)
{
    p->segmentCount = SegmentCount(p->clip->sampleCount);
    size_t cells = (size_t)p->segmentCount * subs;
    // One more of each, so no count is 0.
    p->subs = maniAllocate(p->allocator, (size_t)subs + 1, sizeof(SubTrack), alignof(SubTrack));
    p->firsts = maniAllocate(p->allocator, (size_t)p->segmentCount + 1, sizeof(uint32_t),
                             alignof(uint32_t));
    p->rates = maniAllocate(p->allocator, 7 * cells + 1, 1, 1);
    if (p->subs == NULL || p->firsts == NULL || p->rates == NULL)
    {
        return mani_errorCapacity;
    }
    p->segmentRanges = p->rates + cells;
    Split(p->clip->sampleCount, p->segmentCount, p->firsts);
    return mani_success;
}

// The compressed clip opened from a copy of the builder's file (opening
// wants the container's alignment, more than allocators promise), and
// measured.
static maniResult Check(maniBuilder* builder, const maniClip* clip, const maniSkeleton* skeleton,
                        const maniCompressOptions* o, maniCompressReport* report)
{
    size_t copySize = builder->fileSize + MANI_CONTAINER_ALIGNMENT;
    uint8_t* copy = maniAllocate(&builder->allocator, copySize, 1, 8);
    if (copy == NULL)
    {
        return mani_errorCapacity;
    }
    uint8_t* at = copy + (MANI_CONTAINER_ALIGNMENT - (uintptr_t)copy % MANI_CONTAINER_ALIGNMENT) %
                             MANI_CONTAINER_ALIGNMENT;
    memcpy(at, builder->file, builder->fileSize);
    maniClip packed;
    maniResult r = maniOpenClip(at, builder->fileSize, &packed).result == mani_success
                       ? mani_success
                       : mani_errorInvalid;
    *report = (maniCompressReport){.bytes = builder->fileSize};
    Measure m = {0};
    r = r == mani_success ? MakeMeasure(&builder->allocator, skeleton, clip->floatTrackCount, &m)
                          : r;
    if (r == mani_success)
    {
        Report(clip, &packed, skeleton, o, &m, report);
    }
    ReleaseMeasure(&builder->allocator, &m);
    maniRelease(&builder->allocator, copy, copySize, 1, 8);
    return r;
}

maniResult maniCompressClip(maniBuilder* builder, const maniClip* clip,
                            const maniSkeleton* skeleton, const maniCompressOptions* options,
                            maniCompressReport* reportOut)
{
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if (clip == NULL || skeleton == NULL || options == NULL || clip->compressed ||
        clip->skeletonIdentity != maniSkeletonIdentity(skeleton) ||
        clip->skeletonBoneCount != maniSkeletonBoneCount(skeleton) ||
        !OptionsValid(options, clip->skeletonBoneCount, clip->floatTrackCount))
    {
        ++builder->misuses;
        return mani_errorInvalid;
    }
    uint32_t subs = 3 * clip->trackCount + clip->floatTrackCount;
    Plan p = {.clip = clip, .skeleton = skeleton, .allocator = &builder->allocator};
    maniResult r = Prepare(&p, subs);
    if (r == mani_success)
    {
        Gather(&p, options);
        Fold(&p);
        ChooseRates(&p);
        r = Emit(builder, &p);
    }
    ReleasePlan(&p, subs);
    maniCompressReport report = {0};
    r = r == mani_success ? Check(builder, clip, skeleton, options, &report) : r;
    if (r == mani_success && reportOut != NULL)
    {
        *reportOut = report;
    }
    return r;
}
