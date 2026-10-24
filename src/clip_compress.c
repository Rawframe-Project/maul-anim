// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Compression of a full-precision clip: the sub-tracks
// gathered, folded (float tracks within their precision, bone channels
// by the shell metric), their rates chosen (float tracks each on its
// own, bone channels by the search), the sections written, and the file
// opened and measured for the report.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_plan.h"
#include "clip_search.h"
#include "clip_shell.h"
#include "clip_view.h"
#include "container.h"
#include "fpenv.h"

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

static void StoreF32(uint8_t* p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    maniStoreU32(p, bits);
}

// The bone channels and float tracks as sub-tracks, with each bone's
// first.
static void Gather(maniCompressPlan* p)
{
    const maniClip* c = p->clip;
    const maniCompressOptions* o = p->options;
    uint32_t offset = 0;
    p->subCount = 0;
    for (uint32_t b = 0; b < c->skeletonBoneCount; ++b)
    {
        p->boneSubs[b] = UINT32_MAX;
    }
    for (uint32_t i = 0; i < c->trackCount; ++i)
    {
        const uint8_t* entry = c->tracks + (size_t)i * MANI_CLIP_TRACK_SIZE;
        uint32_t bone = maniLoadU16(entry);
        p->boneSubs[bone] = p->subCount;
        for (uint32_t ch = 0; ch < 3; ++ch)
        {
            if (((entry[2] >> ch) & 1u) == 0)
            {
                continue;
            }
            p->subs[p->subCount++] = (maniSubTrack){
                .track = i, .channel = ch, .bone = bone, .offset = offset, .dropped = 3};
            offset += ch == maniSubRotation ? 4u : 3u;
        }
    }
    for (uint32_t i = 0; i < c->floatTrackCount; ++i)
    {
        float precision = o->floatPrecisions != NULL ? o->floatPrecisions[i] : o->floatPrecision;
        p->subs[p->subCount++] = (maniSubTrack){
            .track = i, .channel = maniSubFloat, .offset = i, .tolerance = precision};
    }
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        if (p->subs[i].channel == maniSubRotation)
        {
            maniPlanChooseDropped(p, &p->subs[i]);
        }
        maniPlanRanges(p, &p->subs[i]);
    }
}

// The animated sub-tracks numbered, bone channels first, and the
// constants counted.
static void Number(maniCompressPlan* p)
{
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        maniSubTrack* t = &p->subs[i];
        bool channel = t->channel != maniSubFloat;
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
            p->subs[i].channel == maniSubFloat && p->subs[i].form == maniClipFormAnimated
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
static void SegmentBits(const maniCompressPlan* p, uint32_t s, uint32_t* channelBits,
                        uint32_t* floatBits)
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

static uint64_t SegmentDataSize(const maniCompressPlan* p)
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

// The sub-tracks' channels in the order the file stores them: each kind
// of bone channel (rotation, translation, scale), then the float tracks.
static const uint32_t kKindOrder[maniClipKinds + 1] = {maniSubRotation, maniSubTranslation,
                                                       maniSubScale, maniSubFloat};

// A kind's animated sub-tracks.
static uint32_t AnimatedOfKind(const maniCompressPlan* p, uint32_t channel)
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        n += p->subs[i].channel == channel && p->subs[i].form == maniClipFormAnimated ? 1 : 0;
    }
    return n;
}

// The forms: each kind's bitsets, the dropped components, the float
// tracks' forms.
static void WriteForms(const maniCompressPlan* p, uint8_t* forms)
{
    uint64_t bytes = maniClipBitsetBytes(p->clip->trackCount);
    uint8_t* dropped = forms + 2 * maniClipKinds * bytes;
    uint8_t* floats = dropped + ((uint64_t)p->clip->trackCount + 3) / 4;
    for (uint32_t i = 0; i < p->subCount; ++i)
    {
        const maniSubTrack* t = &p->subs[i];
        if (t->channel == maniSubFloat)
        {
            floats[t->track] = (uint8_t)t->form;
            continue;
        }
        bool stored = t->form == maniClipFormConstant || t->form == maniClipFormAnimated;
        if (!stored)
        {
            continue;
        }
        uint32_t kind = 0;
        while (kKindOrder[kind] != t->channel)
        {
            ++kind;
        }
        uint32_t animated = t->form == maniClipFormAnimated ? 1u : 0u;
        forms[(2 * kind + animated) * bytes + t->track / 8] |= (uint8_t)(1u << (t->track % 8));
        if (t->channel == maniSubRotation)
        {
            dropped[t->track / 4] |= (uint8_t)((3u - t->dropped) << (2 * (t->track % 4)));
        }
    }
}

// Forms, then the constants and the clip ranges in kind order.
static void WriteFront(const maniCompressPlan* p, Sections* out)
{
    uint8_t* constant = out->data[1];
    uint8_t* ranges = out->data[2];
    memset(out->data[0], 0, (size_t)out->sizes[0]);
    WriteForms(p, out->data[0]);
    for (uint32_t kind = 0; kind <= maniClipKinds; ++kind)
    {
        uint32_t channel = kKindOrder[kind];
        uint32_t n = AnimatedOfKind(p, channel);
        uint32_t width = channel == maniSubFloat ? 1u : 3u;
        uint32_t e = 0;
        for (uint32_t i = 0; i < p->subCount; ++i)
        {
            const maniSubTrack* t = &p->subs[i];
            if (t->channel != channel)
            {
                continue;
            }
            for (uint32_t c = 0; t->form == maniClipFormConstant && c < width; ++c)
            {
                StoreF32(constant, t->constant[c]);
                constant += 4;
            }
            for (uint32_t c = 0; t->form == maniClipFormAnimated && c < width; ++c)
            {
                StoreF32(ranges + maniClipRangeAt(n, width, e, c, 0), t->minimum[c]);
                StoreF32(ranges + maniClipRangeAt(n, width, e, c, 1), t->extent[c]);
            }
            e += t->form == maniClipFormAnimated ? 1 : 0;
        }
        ranges += 8 * (size_t)width * n;
    }
}

// One sample's bits in a segment, the sub-tracks in kind order.
static void WriteSample(const maniCompressPlan* p, uint32_t s, uint32_t k, maniBitWriter* w)
{
    uint32_t n = p->animatedChannels + p->animatedFloats;
    for (uint32_t kind = 0; kind <= maniClipKinds; ++kind)
    {
        for (uint32_t i = 0; i < p->subCount; ++i)
        {
            const maniSubTrack* t = &p->subs[i];
            if (t->channel != kKindOrder[kind] || t->form != maniClipFormAnimated)
            {
                continue;
            }
            size_t at = (size_t)s * n + t->animated;
            uint32_t rate = p->rates[at];
            const uint8_t* range = p->segmentCount > 1 ? p->segmentRanges + 6 * at : NULL;
            float v[3] = {0.0f, 0.0f, 0.0f};
            maniPlanValue(p, t, k, v);
            for (uint32_t c = 0; rate != 0 && c < maniPlanWidth(t); ++c)
            {
                maniBitWrite(w, maniPlanQuantize(t, c, v[c], rate, range), maniCodecBits(rate));
            }
        }
    }
}

// The segment table and data.
static void WriteSegments(const maniCompressPlan* p, Sections* out)
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
        uint8_t* ranges = head + n;
        for (uint32_t kind = 0; kind <= maniClipKinds; ++kind)
        {
            for (uint32_t i = 0; i < p->subCount; ++i)
            {
                const maniSubTrack* t = &p->subs[i];
                if (t->channel != kKindOrder[kind] || t->form != maniClipFormAnimated)
                {
                    continue;
                }
                size_t at = (size_t)s * n + t->animated;
                *head++ = p->rates[at];
                size_t width = maniPlanWidth(t);
                if (p->segmentCount > 1)
                {
                    memcpy(ranges, p->segmentRanges + 6 * at, 2 * width);
                    ranges += 2 * width;
                }
            }
        }
        head = ranges;
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
    maniStoreU32(h + 60, c->additive);
    maniStoreU32(h + 56, 1);
}

// The file from a planned clip, kept in the builder.
static maniResult Emit(maniBuilder* builder, const maniCompressPlan* p)
{
    Sections out = {0};
    out.sizes[0] = maniClipFormsSize(p->clip->trackCount, p->clip->floatTrackCount);
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
    const uint8_t* sections[MANI_CLIP_ALL_SECTIONS] = {header, p->clip->tracks, &kEmpty};
    uint64_t sizes[MANI_CLIP_ALL_SECTIONS] = {
        MANI_CLIP_HEADER_SIZE, (uint64_t)p->clip->trackCount * MANI_CLIP_TRACK_SIZE, 0};
    NamedSections(p->clip, sections + 3, sizes + 3);
    for (uint32_t i = 0; i < 5; ++i)
    {
        sections[10 + i] = out.sizes[i] > 0 ? out.data[i] : &kEmpty;
        sizes[10 + i] = out.sizes[i];
    }
    // The root motion, as the clip has it.
    sections[maniClipTagRootMotion - 1] =
        p->clip->rootMotion != NULL ? p->clip->rootMotion : &kEmpty;
    sizes[maniClipTagRootMotion - 1] =
        p->clip->rootMotion != NULL ? maniRootMotionSize(p->clip->sampleCount) : 0;
    // Empty sections are left out of the table.
    maniSectionSource table[MANI_CLIP_ALL_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_CLIP_ALL_SECTIONS; ++i)
    {
        table[count] = (maniSectionSource){i + 1, sections[i], sizes[i]};
        count += sizes[i] > 0 ? 1 : 0;
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(table, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindClip, MANI_CLIP_FORMAT_VERSION,
                           maniClipComputeIdentity(sections, sizes, MANI_CLIP_ALL_SECTIONS), table,
                           count);
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

static void ReleasePlan(maniCompressPlan* p, uint32_t subs)
{
    size_t cells = (size_t)p->segmentCount * subs;
    maniRelease(p->allocator, p->subs, (size_t)subs + 1, sizeof(maniSubTrack),
                alignof(maniSubTrack));
    maniRelease(p->allocator, p->firsts, (size_t)p->segmentCount + 1, sizeof(uint32_t),
                alignof(uint32_t));
    maniRelease(p->allocator, p->rates, 7 * cells + 1, 1, 1);
    maniRelease(p->allocator, p->boneSubs, p->clip->skeletonBoneCount, sizeof(uint32_t),
                alignof(uint32_t));
}

// The plan's arrays: sub-tracks, segment starts, and per segment and
// sub-track a rate and six range bytes.
static maniResult Prepare(maniCompressPlan* p, uint32_t subs)
{
    p->segmentCount = maniPlanSegmentCount(p->clip->sampleCount);
    size_t cells = (size_t)p->segmentCount * subs;
    // One more of each, so no count is 0.
    p->subs =
        maniAllocate(p->allocator, (size_t)subs + 1, sizeof(maniSubTrack), alignof(maniSubTrack));
    p->firsts = maniAllocate(p->allocator, (size_t)p->segmentCount + 1, sizeof(uint32_t),
                             alignof(uint32_t));
    p->rates = maniAllocate(p->allocator, 7 * cells + 1, 1, 1);
    p->boneSubs =
        maniAllocate(p->allocator, p->clip->skeletonBoneCount, sizeof(uint32_t), alignof(uint32_t));
    if (p->subs == NULL || p->firsts == NULL || p->rates == NULL || p->boneSubs == NULL)
    {
        return mani_errorCapacity;
    }
    p->segmentRanges = p->rates + cells;
    maniPlanSplit(p->clip->sampleCount, p->segmentCount, p->firsts);
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

static MANI_NOINLINE maniResult CompressClipWork(maniBuilder* builder, const maniClip* clip,
                                                 const maniSkeleton* skeleton,
                                                 const maniCompressOptions* options,
                                                 maniCompressReport* reportOut)
{
    if (maniBuilderCalledBack(builder))
    {
        return mani_errorState;
    }
    if (builder == NULL)
    {
        return mani_errorInvalid;
    }
    if (clip == NULL || skeleton == NULL || options == NULL)
    {
        maniBuilderMisuse(builder);
        return mani_errorInvalid;
    }
    // A clip already compressed or built for another skeleton is the
    // opened file's doing, not the program's, so it counts no misuse.
    if (clip->compressed || clip->skeletonIdentity != maniSkeletonIdentity(skeleton) ||
        clip->skeletonBoneCount != maniSkeletonBoneCount(skeleton))
    {
        return mani_errorInvalid;
    }
    if (!OptionsValid(options, clip->skeletonBoneCount, clip->floatTrackCount))
    {
        maniBuilderMisuse(builder);
        return mani_errorInvalid;
    }
    uint32_t subs = 3 * clip->trackCount + clip->floatTrackCount;
    maniCompressPlan p = {
        .clip = clip, .skeleton = skeleton, .options = options, .allocator = &builder->allocator};
    maniResult r = Prepare(&p, subs);
    if (r == mani_success)
    {
        Gather(&p);
        for (uint32_t i = 0; i < p.subCount; ++i)
        {
            if (p.subs[i].channel == maniSubFloat)
            {
                maniPlanFoldFloat(&p, &p.subs[i]);
            }
        }
        r = maniSearchFold(&p);
    }
    if (r == mani_success)
    {
        Number(&p);
        maniPlanFloatRates(&p);
        r = maniSearchRates(&p);
    }
    r = r == mani_success ? Emit(builder, &p) : r;
    ReleasePlan(&p, subs);
    maniCompressReport report = {0};
    r = r == mani_success ? Check(builder, clip, skeleton, options, &report) : r;
    if (r == mani_success && reportOut != NULL)
    {
        *reportOut = report;
    }
    return r;
}

maniResult maniCompressClip(maniBuilder* builder, const maniClip* clip,
                            const maniSkeleton* skeleton, const maniCompressOptions* options,
                            maniCompressReport* reportOut)
{
    maniFloatEnv env;
    maniFloatEnvEnter(&env);
    maniResult result = CompressClipWork(builder, clip, skeleton, options, reportOut);
    maniFloatEnvLeave(&env);
    return result;
}
