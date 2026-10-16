// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building a motion database (mani-0024): the source checked, every
// entry's frames sampled (match_sample), each channel normalized as
// Holden's reference does (the mean per feature, the scale the mean of
// the features' standard deviations over the channel's weight; both in
// binary64 over the frames in order, stored in binary32), the bounds of
// every run of 16 and 64 frames taken from the normalized features, and
// the file written.

#include "allocator.h"
#include "builder.h"
#include "bytes.h"
#include "container.h"
#include "match_file.h"
#include "match_sample.h"
#include "text.h"

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static maniBuildResult Result(maniResult result, maniBuildReason reason, uint32_t item,
                              uint32_t other)
{
    return (maniBuildResult){result, reason, item, other};
}

static const maniBuildResult kOk = {mani_success, maniBuildReasonNone, 0, 0};

static maniBuildResult Refuse(maniBuildReason reason, uint32_t item, uint32_t other)
{
    return Result(mani_errorInvalid, reason, item, other);
}

// The totals a valid source gives.
typedef struct Totals
{
    uint32_t frames;
    uint32_t features;
    uint32_t times;
    uint32_t names;
} Totals;

static bool ChannelValid(const maniMotionDatabaseSource* s, const maniMatchChannelSource* c)
{
    uint32_t bones = maniSkeletonBoneCount(s->skeleton);
    bool bone = c->kind <= maniMatchBoneFacing;
    bool track = c->kind == maniMatchFloatTrack;
    if (c->kind > maniMatchFloatTrack || (bone && c->bone >= bones) ||
        (c->kind == maniMatchBoneFacing && c->axis > maniMatchAxisNegativeZ) || c->timeCount < 1 ||
        c->timeCount > MANI_MATCH_MAX_TIMES || c->times == NULL ||
        !(c->weight > 0.0f && c->weight < HUGE_VALF))
    {
        return false;
    }
    for (uint32_t k = 0; k < c->timeCount; ++k)
    {
        if (!(fabsf(c->times[k]) <= MANI_MATCH_MAX_REACH))
        {
            return false;
        }
    }
    return !track || (c->track != NULL && c->trackLength > 0 &&
                      maniValidUtf8((const uint8_t*)c->track, c->trackLength));
}

static bool EntryValid(const maniMotionDatabaseSource* s, const maniMatchEntrySource* e)
{
    if (e->clip >= s->clipCount || e->flags > (MANI_MATCH_LOOP | MANI_MATCH_NO_JUMP))
    {
        return false;
    }
    bool loop = (e->flags & MANI_MATCH_LOOP) != 0;
    float duration = maniClipDuration(s->clips[e->clip]);
    return e->start >= 0.0f && e->end <= duration &&
           (loop ? e->end > e->start : e->end >= e->start) && fabsf(e->costBias) < HUGE_VALF;
}

static maniBuildResult CheckClips(const maniMotionDatabaseSource* s)
{
    uint64_t skeleton = maniSkeletonIdentity(s->skeleton);
    for (uint32_t i = 0; i < s->clipCount; ++i)
    {
        const maniClip* clip = s->clips[i];
        if (maniClipSkeletonIdentity(clip) != skeleton ||
            maniClipAdditiveBasis(clip) != maniAdditiveNone)
        {
            return Refuse(maniBuildReasonJoint, i, 0);
        }
    }
    return kOk;
}

static maniBuildResult CheckChannels(const maniMotionDatabaseSource* s, Totals* t)
{
    uint64_t features = 0;
    for (uint32_t c = 0; c < s->channelCount; ++c)
    {
        const maniMatchChannelSource* ch = &s->channels[c];
        if (!ChannelValid(s, ch))
        {
            return Refuse(maniBuildReasonChannel, c, 0);
        }
        features += (uint64_t)maniMatchChannelWidth(ch->kind) * ch->timeCount;
        t->times += ch->timeCount;
        t->names += ch->kind == maniMatchFloatTrack ? ch->trackLength : 0u;
        for (uint32_t e = 0; ch->kind == maniMatchFloatTrack && e < s->entryCount; ++e)
        {
            const maniClip* clip = s->clips[s->entries[e].clip];
            if (maniClipFindFloatTrack(clip, ch->track, ch->trackLength) == MANI_NO_TRACK)
            {
                return Refuse(maniBuildReasonMissingTrack, c, e);
            }
        }
    }
    if (features > MANI_MATCH_MAX_FEATURES)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    t->features = (uint32_t)features;
    return kOk;
}

static maniBuildResult CheckEntries(const maniMotionDatabaseSource* s, Totals* t)
{
    uint64_t frames = 0;
    for (uint32_t e = 0; e < s->entryCount; ++e)
    {
        const maniMatchEntrySource* entry = &s->entries[e];
        if (!EntryValid(s, entry))
        {
            return Refuse(maniBuildReasonEntry, e, 0);
        }
        frames += maniMatchEntryFrames(entry->start, entry->end, s->sampleRate,
                                       (entry->flags & MANI_MATCH_LOOP) != 0);
    }
    if (frames > MANI_MATCH_MAX_FRAMES)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    t->frames = (uint32_t)frames;
    return kOk;
}

static maniBuildResult Check(const maniMotionDatabaseSource* s, Totals* t)
{
    if (!(s->sampleRate > 0.0f && s->sampleRate <= MANI_MATCH_MAX_RATE))
    {
        return Refuse(maniBuildReasonTiming, 0, 0);
    }
    if (s->up > maniMatchAxisNegativeZ || s->forward > maniMatchAxisNegativeZ ||
        s->up % 3 == s->forward % 3)
    {
        return Refuse(maniBuildReasonOption, 0, 0);
    }
    if (s->clipCount < 1 || s->clipCount > MANI_MATCH_MAX_CLIPS || s->channelCount < 1 ||
        s->channelCount > MANI_MATCH_MAX_CHANNELS || s->entryCount < 1 ||
        s->entryCount > MANI_MATCH_MAX_ENTRIES)
    {
        return Refuse(maniBuildReasonCount, 0, 0);
    }
    maniBuildResult r = CheckClips(s);
    r = r.result == mani_success ? CheckEntries(s, t) : r;
    return r.result == mani_success ? CheckChannels(s, t) : r;
}

// What building holds: the raw then normalized features, the
// normalization, the bounds, and the sampler's pose, floats and tracks.
typedef struct Work
{
    const maniMotionDatabaseSource* source;
    const maniAllocator* allocator;
    Totals totals;
    float* features;
    float* normalization;
    float* bounds;
    void* poseMemory;
    size_t poseSize;
    float* floats;
    uint32_t floatCapacity;
    uint32_t reserved;
    uint32_t* tracks;
    size_t featureFloats;
    size_t boundFloats;
} Work;

static void Release(Work* w)
{
    const maniAllocator* a = w->allocator;
    maniRelease(a, w->features, w->featureFloats, sizeof(float), 8);
    maniRelease(a, w->normalization, 2 * (size_t)w->totals.features, sizeof(float), 8);
    maniRelease(a, w->bounds, w->boundFloats, sizeof(float), 8);
    maniRelease(a, w->poseMemory, w->poseSize + MANI_POSE_ALIGNMENT, 1, 8);
    maniRelease(a, w->floats, w->floatCapacity + 1, sizeof(float), 4);
    maniRelease(a, w->tracks, (size_t)w->source->channelCount * w->source->entryCount + 1,
                sizeof(uint32_t), 4);
}

static bool Allocate(Work* w)
{
    const maniMotionDatabaseSource* s = w->source;
    const maniAllocator* a = w->allocator;
    uint64_t runs = (uint64_t)maniMatchRuns(w->totals.frames, MANI_MATCH_SMALL_RUN) +
                    maniMatchRuns(w->totals.frames, MANI_MATCH_LARGE_RUN);
    uint64_t features = (uint64_t)w->totals.frames * w->totals.features;
    if (features > SIZE_MAX / sizeof(float) / 4)
    {
        return false;
    }
    w->featureFloats = (size_t)features;
    w->boundFloats = (size_t)(2 * runs * w->totals.features);
    for (uint32_t i = 0; i < s->clipCount; ++i)
    {
        uint32_t count = maniClipFloatTrackCount(s->clips[i]);
        w->floatCapacity = count > w->floatCapacity ? count : w->floatCapacity;
    }
    w->poseSize = maniPoseSize(s->skeleton);
    w->features = maniAllocate(a, w->featureFloats, sizeof(float), 8);
    w->normalization = maniAllocate(a, 2 * (size_t)w->totals.features, sizeof(float), 8);
    w->bounds = maniAllocate(a, w->boundFloats, sizeof(float), 8);
    // A pose wants 32 bytes' alignment, past what an allocator promises
    // (8): aligned by hand within the slack.
    w->poseMemory = maniAllocate(a, w->poseSize + MANI_POSE_ALIGNMENT, 1, 8);
    w->floats = maniAllocate(a, w->floatCapacity + 1, sizeof(float), 4);
    w->tracks = maniAllocate(a, (size_t)s->channelCount * s->entryCount + 1, sizeof(uint32_t), 4);
    return w->features != NULL && w->normalization != NULL && w->bounds != NULL &&
           w->poseMemory != NULL && w->floats != NULL && w->tracks != NULL;
}

// Every frame's raw features, entry by entry.
static bool Sample(Work* w)
{
    const maniMotionDatabaseSource* s = w->source;
    maniPose* pose = NULL;
    uint8_t* memory = w->poseMemory;
    memory += (MANI_POSE_ALIGNMENT - (uintptr_t)memory % MANI_POSE_ALIGNMENT) % MANI_POSE_ALIGNMENT;
    if (maniInitPose(s->skeleton, memory, w->poseSize, &pose) != mani_success)
    {
        return false;
    }
    for (uint32_t c = 0; c < s->channelCount; ++c)
    {
        const maniMatchChannelSource* ch = &s->channels[c];
        for (uint32_t e = 0; e < s->entryCount; ++e)
        {
            w->tracks[(size_t)c * s->entryCount + e] =
                ch->kind == maniMatchFloatTrack
                    ? maniClipFindFloatTrack(s->clips[s->entries[e].clip], ch->track,
                                             ch->trackLength)
                    : UINT32_MAX;
        }
    }
    maniMatchSampler sampler = {s, pose, w->floats, w->floatCapacity, 0.0f, NULL, w->tracks};
    float* out = w->features;
    bool ok = true;
    for (uint32_t e = 0; e < s->entryCount && ok; ++e)
    {
        const maniMatchEntrySource* entry = &s->entries[e];
        uint32_t frames = maniMatchEntryFrames(entry->start, entry->end, s->sampleRate,
                                               (entry->flags & MANI_MATCH_LOOP) != 0);
        for (uint32_t k = 0; k < frames && ok; ++k)
        {
            float t = maniMatchFrameTime(entry->start, k, s->sampleRate);
            ok = maniMatchSampleFrame(&sampler, e, t, out);
            out += w->totals.features;
        }
    }
    return ok;
}

// Channel c's features from first, width wide: offset and scale, then
// normalized in place; false when they never vary.
static bool Normalize(Work* w, uint32_t first, uint32_t width, float weight)
{
    uint32_t n = w->totals.features;
    uint32_t frames = w->totals.frames;
    double spread = 0.0;
    for (uint32_t j = first; j < first + width; ++j)
    {
        double mean = 0.0;
        for (uint32_t f = 0; f < frames; ++f)
        {
            mean += (double)w->features[(size_t)f * n + j];
        }
        mean /= (double)frames;
        double variance = 0.0;
        for (uint32_t f = 0; f < frames; ++f)
        {
            double d = (double)w->features[(size_t)f * n + j] - mean;
            variance += d * d;
        }
        spread += sqrt(variance / (double)frames);
        w->normalization[j] = (float)mean;
    }
    float scale = (float)(spread / (double)width / (double)weight);
    if (!(scale > 0.0f && scale < HUGE_VALF))
    {
        return false;
    }
    for (uint32_t j = first; j < first + width; ++j)
    {
        w->normalization[n + j] = scale;
        for (uint32_t f = 0; f < frames; ++f)
        {
            float* v = &w->features[(size_t)f * n + j];
            *v = (*v - w->normalization[j]) / scale;
        }
    }
    return true;
}

static void Bound(Work* w, float* low, uint32_t size)
{
    uint32_t n = w->totals.features;
    uint32_t runs = maniMatchRuns(w->totals.frames, size);
    float* high = low + (size_t)runs * n;
    for (uint32_t r = 0; r < runs; ++r)
    {
        uint32_t to = (r + 1) * size < w->totals.frames ? (r + 1) * size : w->totals.frames;
        for (uint32_t j = 0; j < n; ++j)
        {
            float a = HUGE_VALF;
            float b = -HUGE_VALF;
            for (uint32_t f = r * size; f < to; ++f)
            {
                float v = w->features[(size_t)f * n + j];
                a = v < a ? v : a;
                b = v > b ? v : b;
            }
            low[(size_t)r * n + j] = a;
            high[(size_t)r * n + j] = b;
        }
    }
}

static uint8_t* LayChannels(const Work* w, uint8_t* channels, uint8_t* times, uint8_t* names)
{
    const maniMotionDatabaseSource* s = w->source;
    uint32_t time = 0;
    uint32_t feature = 0;
    uint32_t name = 0;
    for (uint32_t c = 0; c < s->channelCount; ++c)
    {
        const maniMatchChannelSource* ch = &s->channels[c];
        uint8_t* e = channels + (size_t)c * MANI_MATCH_CHANNEL_SIZE;
        uint32_t width = maniMatchChannelWidth(ch->kind) * ch->timeCount;
        bool track = ch->kind == maniMatchFloatTrack;
        const uint32_t words[7] = {ch->kind,
                                   ch->kind <= maniMatchBoneFacing ? ch->bone : 0u,
                                   ch->kind == maniMatchBoneFacing ? ch->axis : 0u,
                                   ch->timeCount,
                                   time,
                                   feature,
                                   width};
        for (uint32_t k = 0; k < 7; ++k)
        {
            maniStoreU32(e + 4 * k, words[k]);
        }
        memcpy(e + 28, &ch->weight, 4);
        maniStoreU32(e + 32, track ? name : 0u);
        maniStoreU32(e + 36, track ? ch->trackLength : 0u);
        memcpy(times + 4 * (size_t)time, ch->times, 4 * (size_t)ch->timeCount);
        if (track)
        {
            memcpy(names + name, ch->track, ch->trackLength);
            name += ch->trackLength;
        }
        time += ch->timeCount;
        feature += width;
    }
    return names + name;
}

static void LayEntries(const Work* w, uint8_t* entries, uint8_t* clips, uint8_t* header)
{
    const maniMotionDatabaseSource* s = w->source;
    uint32_t frame = 0;
    for (uint32_t i = 0; i < s->entryCount; ++i)
    {
        const maniMatchEntrySource* e = &s->entries[i];
        uint8_t* at = entries + (size_t)i * MANI_MATCH_ENTRY_SIZE;
        uint32_t frames = maniMatchEntryFrames(e->start, e->end, s->sampleRate,
                                               (e->flags & MANI_MATCH_LOOP) != 0);
        const float spans[3] = {e->start, e->end, e->costBias};
        maniStoreU32(at, e->clip);
        maniStoreU32(at + 4, e->flags);
        memcpy(at + 8, spans, sizeof spans);
        maniStoreU32(at + 20, e->tags);
        maniStoreU32(at + 24, frame);
        maniStoreU32(at + 28, frames);
        frame += frames;
    }
    for (uint32_t i = 0; i < s->clipCount; ++i)
    {
        maniStoreU64(clips + 8 * (size_t)i, maniClipIdentity(s->clips[i]));
    }
    const Totals* t = &w->totals;
    const uint32_t counts[7] = {maniSkeletonBoneCount(s->skeleton),
                                t->frames,
                                t->features,
                                s->channelCount,
                                t->times,
                                s->entryCount,
                                s->clipCount};
    maniStoreU64(header, maniSkeletonIdentity(s->skeleton));
    for (uint32_t k = 0; k < 7; ++k)
    {
        maniStoreU32(header + 8 + 4 * k, counts[k]);
    }
    memcpy(header + 36, &s->sampleRate, 4);
    maniStoreU32(header + 40, s->up);
    maniStoreU32(header + 44, s->forward);
}

static maniBuildResult Write(maniBuilder* builder, const Work* w)
{
    const maniMotionDatabaseSource* s = w->source;
    const Totals* t = &w->totals;
    const uint64_t sizes[MANI_MATCH_SECTIONS] = {MANI_MATCH_HEADER_SIZE,
                                                 (uint64_t)s->channelCount *
                                                     MANI_MATCH_CHANNEL_SIZE,
                                                 4 * (uint64_t)t->times,
                                                 8 * (uint64_t)s->clipCount,
                                                 (uint64_t)s->entryCount * MANI_MATCH_ENTRY_SIZE,
                                                 0,
                                                 0,
                                                 0,
                                                 t->names};
    uint64_t own = 0;
    for (uint32_t i = 0; i < MANI_MATCH_SECTIONS; ++i)
    {
        own += sizes[i];
    }
    uint8_t* staging = maniAllocate(&builder->allocator, (size_t)own + 1, 1, 8);
    if (staging == NULL)
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    const uint8_t* data[MANI_MATCH_SECTIONS];
    uint64_t lengths[MANI_MATCH_SECTIONS];
    uint8_t* at = staging;
    for (uint32_t i = 0; i < MANI_MATCH_SECTIONS; ++i)
    {
        data[i] = at;
        lengths[i] = sizes[i];
        at += sizes[i];
    }
    (void)LayChannels(w, staging + sizes[0], staging + sizes[0] + sizes[1], (uint8_t*)data[8]);
    LayEntries(w, (uint8_t*)data[4], (uint8_t*)data[3], staging);
    data[5] = (const uint8_t*)w->normalization;
    lengths[5] = 8 * (uint64_t)t->features;
    data[6] = (const uint8_t*)w->features;
    lengths[6] = 4 * (uint64_t)w->featureFloats;
    data[7] = (const uint8_t*)w->bounds;
    lengths[7] = 4 * (uint64_t)w->boundFloats;
    maniSectionSource sections[MANI_MATCH_SECTIONS];
    uint32_t count = 0;
    for (uint32_t i = 0; i < MANI_MATCH_SECTIONS; ++i)
    {
        if (lengths[i] > 0)
        {
            sections[count++] = (maniSectionSource){i + 1, data[i], lengths[i]};
        }
    }
    uint64_t fileSize = 0;
    uint8_t* file = NULL;
    if (maniContainerSize(sections, count, &fileSize) && fileSize <= SIZE_MAX)
    {
        file = maniAllocate(&builder->allocator, (size_t)fileSize, 1, 8);
    }
    if (file != NULL)
    {
        maniWriteContainer(file, maniKindMotionDatabase, MANI_MATCH_FORMAT_VERSION,
                           maniMatchComputeIdentity(data, lengths), sections, count);
        maniBuilderKeep(builder, maniKindMotionDatabase, file, (size_t)fileSize, NULL, 0);
    }
    maniRelease(&builder->allocator, staging, (size_t)own + 1, 1, 8);
    return file != NULL ? kOk : Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
}

static maniBuildResult Build(maniBuilder* builder, Work* w)
{
    const maniMotionDatabaseSource* s = w->source;
    if (!Allocate(w) || !Sample(w))
    {
        return Result(mani_errorCapacity, maniBuildReasonNone, 0, 0);
    }
    uint32_t first = 0;
    for (uint32_t c = 0; c < s->channelCount; ++c)
    {
        const maniMatchChannelSource* ch = &s->channels[c];
        uint32_t width = maniMatchChannelWidth(ch->kind) * ch->timeCount;
        if (!Normalize(w, first, width, ch->weight))
        {
            return Refuse(maniBuildReasonSpread, c, 0);
        }
        first += width;
    }
    uint32_t small = maniMatchRuns(w->totals.frames, MANI_MATCH_SMALL_RUN);
    Bound(w, w->bounds, MANI_MATCH_SMALL_RUN);
    Bound(w, w->bounds + 2 * (size_t)small * w->totals.features, MANI_MATCH_LARGE_RUN);
    return Write(builder, w);
}

maniBuildResult maniBuildMotionDatabase(maniBuilder* builder,
                                        const maniMotionDatabaseSource* source)
{
    if (builder == NULL)
    {
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    bool lists = source != NULL && (source->clips != NULL || source->clipCount == 0) &&
                 (source->channels != NULL || source->channelCount == 0) &&
                 (source->entries != NULL || source->entryCount == 0);
    for (uint32_t i = 0; lists && i < source->clipCount; ++i)
    {
        lists = source->clips[i] != NULL;
    }
    if (!lists || source->skeleton == NULL)
    {
        ++builder->misuses;
        return Result(mani_errorInvalid, maniBuildReasonNone, 0, 0);
    }
    Totals totals = {0, 0, 0, 0};
    maniBuildResult r = Check(source, &totals);
    if (r.result != mani_success)
    {
        return r;
    }
    Work w = {.source = source, .allocator = &builder->allocator, .totals = totals};
    r = Build(builder, &w);
    Release(&w);
    return r;
}
