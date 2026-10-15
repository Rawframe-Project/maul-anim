// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a clip file and sampling it. A time maps to two samples and a
// blend factor by binary32 operations alone, so the same time samples the
// same bits everywhere and costs the same in any order.

#include "maul-anim/clip.h"

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_view.h"
#include "container.h"
#include "pose_memory.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/pose.h"

#include <math.h>
#include <stdckdint.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static bool Finite(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

static float LoadF32(const uint8_t* p)
{
    float value = 0.0f;
    memcpy(&value, p, sizeof value);
    return value;
}

bool maniClipSampleCountFor(float duration, float rate, uint32_t* countOut)
{
    if (duration == 0.0f)
    {
        *countOut = 1;
        return true;
    }
    // A duration that is a whole number of frames in decimal (1/3 s at
    // 30 per second) multiplies to a few ulps off that number: the
    // duration's rounding and the product's are each at most 2^-24 of it.
    // A product within 2^-22 of a whole number is taken as it, so no
    // near-duplicate last sample appears; the nearest whole number is
    // found exactly, without rounding product + 0.5.
    float product = duration * rate;
    float nearest = floorf(product);
    if (product - nearest >= 0.5f)
    {
        nearest += 1.0f;
    }
    float intervals = fabsf(product - nearest) <= nearest * 0x1p-22f ? nearest : ceilf(product);
    if (!(intervals < (float)MANI_MAX_SAMPLES))
    {
        return false;
    }
    *countOut = (uint32_t)intervals + 1;
    return true;
}

uint64_t maniClipComputeIdentity(const uint8_t* const* sections, const uint64_t* sizes,
                                 uint32_t count)
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < count; ++i)
    {
        hash = maniHashBytes(hash, sections[i], sizes[i]);
    }
    return hash;
}

// The header's fields, each checked; the offset of the first wrong one.
static maniFileResult CheckHeader(const uint8_t* h, uint64_t at, maniClip* clip)
{
    uint32_t bones = maniLoadU32(h + 8);
    float rate = LoadF32(h + 16);
    float duration = LoadF32(h + 20);
    if (bones == 0 || bones > MANI_MAX_BONES)
    {
        return Fail(at + 8);
    }
    if (!Finite(rate) || !(rate > 0.0f))
    {
        return Fail(at + 16);
    }
    if (!Finite(duration) || !(duration >= 0.0f))
    {
        return Fail(at + 20);
    }
    uint32_t expected = 0;
    if (!maniClipSampleCountFor(duration, rate, &expected) || maniLoadU32(h + 12) != expected)
    {
        return Fail(at + 12);
    }
    if (maniLoadU32(h + 24) > maniLoopRepeat)
    {
        return Fail(at + 24);
    }
    // A compressed clip has its segments; full precision none.
    uint32_t segments = maniLoadU32(h + 52);
    uint32_t form = maniLoadU32(h + 56);
    if (form > 1)
    {
        return Fail(at + 56);
    }
    if (form == 1 ? segments == 0 || segments > expected : segments != 0)
    {
        return Fail(at + 52);
    }
    if (maniLoadU32(h + 60) > maniAdditiveFirstFrame)
    {
        return Fail(at + 60);
    }
    *clip = (maniClip){
        .skeletonIdentity = maniLoadU64(h),
        .skeletonBoneCount = bones,
        .sampleCount = expected,
        .trackCount = maniLoadU32(h + 28),
        .floatsPerSample = maniLoadU32(h + 32),
        .floatTrackCount = maniLoadU32(h + 36),
        .discreteTrackCount = maniLoadU32(h + 40),
        .eventCount = maniLoadU32(h + 44),
        .markerCount = maniLoadU32(h + 48),
        .segmentCount = segments,
        .compressed = form,
        .sampleRate = rate,
        .duration = duration,
        .loop = maniLoadU32(h + 24),
        .additive = maniLoadU32(h + 60),
    };
    return (maniFileResult){mani_success, 0, 0};
}

// Tracks: bones in range and strictly increasing, masks not empty, the
// floats per sample their sum.
static maniFileResult CheckTracks(const uint8_t* t, uint64_t at, uint64_t headerAt,
                                  const maniClip* clip)
{
    uint32_t floats = 0;
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        const uint8_t* entry = t + (uint64_t)i * MANI_CLIP_TRACK_SIZE;
        uint32_t bone = maniLoadU16(entry);
        if (bone >= clip->skeletonBoneCount || (i > 0 && bone <= maniLoadU16(entry - 4)))
        {
            return Fail(at + (uint64_t)i * MANI_CLIP_TRACK_SIZE);
        }
        if (entry[2] == 0 || entry[2] > maniChannelAll)
        {
            return Fail(at + (uint64_t)i * MANI_CLIP_TRACK_SIZE + 2);
        }
        if (entry[3] != 0)
        {
            return Fail(at + (uint64_t)i * MANI_CLIP_TRACK_SIZE + 3);
        }
        floats += maniChannelFloats(entry[2]);
    }
    if (floats != clip->floatsPerSample)
    {
        return Fail(headerAt + 32);
    }
    return (maniFileResult){mani_success, 0, 0};
}

// Samples: finite, rotations unit within 2^-20 in squared length.
static maniFileResult CheckSamples(const uint8_t* s, uint64_t at, const uint8_t* tracks,
                                   const maniClip* clip)
{
    uint64_t offset = 0;
    for (uint32_t k = 0; k < clip->sampleCount; ++k)
    {
        for (uint32_t i = 0; i < clip->trackCount; ++i)
        {
            uint32_t mask = tracks[(uint64_t)i * MANI_CLIP_TRACK_SIZE + 2];
            uint32_t floats = maniChannelFloats(mask);
            for (uint32_t f = 0; f < floats; ++f)
            {
                if (!Finite(LoadF32(s + offset + 4 * (uint64_t)f)))
                {
                    return Fail(at + offset + 4 * (uint64_t)f);
                }
            }
            if (mask & maniChannelRotation)
            {
                uint64_t r = offset + ((mask & maniChannelTranslation) ? 12 : 0);
                double x = (double)LoadF32(s + r);
                double y = (double)LoadF32(s + r + 4);
                double z = (double)LoadF32(s + r + 8);
                double w = (double)LoadF32(s + r + 12);
                if (fabs(x * x + y * y + z * z + w * w - 1.0) > 0x1p-20)
                {
                    return Fail(at + r);
                }
            }
            offset += 4 * (uint64_t)floats;
        }
    }
    return (maniFileResult){mani_success, 0, 0};
}

// A clip file's sections by tag (index tag - 1), an absent one empty.
typedef struct Sections
{
    const uint8_t* data[MANI_CLIP_COMPRESSED_SECTIONS];
    uint64_t sizes[MANI_CLIP_COMPRESSED_SECTIONS];
    uint64_t at[MANI_CLIP_COMPRESSED_SECTIONS];
    // Where each one's size is in the table; the section count for an
    // absent one.
    uint64_t entry[MANI_CLIP_COMPRESSED_SECTIONS];
} Sections;

static const uint8_t kEmpty = 0;

// The table: tags 1 to 15 (the container keeps them increasing), none
// empty, since an empty section is left out.
static maniFileResult LoadSections(const maniContainer* c, Sections* s)
{
    for (uint32_t i = 0; i < MANI_CLIP_COMPRESSED_SECTIONS; ++i)
    {
        s->data[i] = &kEmpty;
        s->sizes[i] = 0;
        s->at[i] = 0;
        s->entry[i] = 12;
    }
    for (uint32_t i = 0; i < c->sectionCount; ++i)
    {
        uint64_t entryAt = MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE;
        const uint8_t* entry = c->bytes + entryAt;
        uint32_t tag = maniLoadU32(entry);
        if (tag == 0 || tag > MANI_CLIP_COMPRESSED_SECTIONS)
        {
            return Fail(entryAt);
        }
        uint64_t size = maniLoadU64(entry + 16);
        if (size == 0)
        {
            return Fail(entryAt + 16);
        }
        uint64_t offset = maniLoadU64(entry + 8);
        s->data[tag - 1] = c->bytes + offset;
        s->sizes[tag - 1] = size;
        s->at[tag - 1] = offset;
        s->entry[tag - 1] = entryAt + 16;
    }
    return (maniFileResult){mani_success, 0, 0};
}

// The header and the tracks (sections 1 and 2), and the samples: section
// 3 at full precision, none in a compressed clip, which alone has
// sections 11 to 15.
static maniFileResult CheckFront(const Sections* s, maniClip* clip)
{
    if (s->sizes[0] != MANI_CLIP_HEADER_SIZE)
    {
        return Fail(s->entry[0]);
    }
    maniFileResult r = CheckHeader(s->data[0], s->at[0], clip);
    if (r.result != mani_success)
    {
        return r;
    }
    for (uint32_t i = MANI_CLIP_SECTIONS; !clip->compressed && i < MANI_CLIP_COMPRESSED_SECTIONS;
         ++i)
    {
        if (s->sizes[i] != 0)
        {
            return Fail(s->entry[i]);
        }
    }
    if (s->sizes[1] != (uint64_t)clip->trackCount * MANI_CLIP_TRACK_SIZE)
    {
        return Fail(s->entry[1]);
    }
    r = CheckTracks(s->data[1], s->at[1], s->at[0], clip);
    if (r.result != mani_success)
    {
        return r;
    }
    clip->tracks = s->data[1];
    uint64_t sampleBytes = 0;
    if (ckd_mul(&sampleBytes, (uint64_t)clip->sampleCount, 4 * (uint64_t)clip->floatsPerSample) ||
        s->sizes[2] != (clip->compressed ? 0 : sampleBytes))
    {
        return Fail(s->entry[2]);
    }
    return clip->compressed ? r : CheckSamples(s->data[2], s->at[2], s->data[1], clip);
}

maniFileResult maniOpenClip(const void* bytes, size_t size, maniClip* clipOut)
{
    if (clipOut == NULL)
    {
        return (maniFileResult){mani_errorInvalid, 0, 0};
    }
    maniContainer c;
    maniFileResult r = maniOpenContainer(bytes, size, maniKindClip, MANI_CLIP_FORMAT_VERSION, &c);
    if (r.result != mani_success)
    {
        return r;
    }
    Sections s;
    maniClip clip;
    r = LoadSections(&c, &s);
    r = r.result == mani_success ? CheckFront(&s, &clip) : r;
    r = r.result == mani_success ? maniCheckClipNamed(s.data, s.sizes, s.at, s.entry, &clip) : r;
    if (r.result == mani_success && clip.compressed)
    {
        r = maniCheckClipCompressed(s.data, s.sizes, s.at, s.entry, &clip);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniClipComputeIdentity(s.data, s.sizes, MANI_CLIP_COMPRESSED_SECTIONS) != c.identity)
    {
        return Fail(40);
    }
    clip.samples = clip.compressed ? NULL : s.data[2];
    clip.identity = c.identity;
    *clipOut = clip;
    return (maniFileResult){mani_success, 0, 0};
}

uint64_t maniClipIdentity(const maniClip* clip)
{
    return clip->identity;
}

bool maniClipCompressed(const maniClip* clip)
{
    return clip->compressed != 0;
}

float maniClipDuration(const maniClip* clip)
{
    return clip->duration;
}

float maniClipSampleRate(const maniClip* clip)
{
    return clip->sampleRate;
}

uint32_t maniClipSampleCount(const maniClip* clip)
{
    return clip->sampleCount;
}

maniLoopPolicy maniClipLoopPolicy(const maniClip* clip)
{
    return clip->loop;
}

maniAdditiveBasis maniClipAdditiveBasis(const maniClip* clip)
{
    return clip->additive;
}

uint64_t maniClipSkeletonIdentity(const maniClip* clip)
{
    return clip->skeletonIdentity;
}

maniClipPair maniClipLocate(const maniClip* clip, float time)
{
    maniClipPair pair = {0, 0, 0.0f, NULL, NULL};
    uint32_t n = clip->sampleCount;
    if (n == 1)
    {
        return pair;
    }
    float t = time > 0.0f ? time : 0.0f;
    t = t < clip->duration ? t : clip->duration;
    float position = floorf(t * clip->sampleRate);
    uint32_t i = position < (float)(n - 2) ? (uint32_t)position : n - 2;
    float start = (float)i / clip->sampleRate;
    float end = i + 1 == n - 1 ? clip->duration : (float)(i + 1) / clip->sampleRate;
    float alpha = end > start ? (t - start) / (end - start) : 0.0f;
    pair.first = i;
    pair.second = i + 1;
    pair.alpha = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
    return pair;
}

bool maniBoneListed(const maniBoneList* list, uint32_t bone, uint32_t* next)
{
    if (list == NULL)
    {
        return true;
    }
    while (*next < list->count && list->bones[*next] < bone)
    {
        ++*next;
    }
    return *next < list->count && list->bones[*next] == bone;
}

// count components of two samples at alpha into a slot from first; a
// rotation normalized (the builder put consecutive ones in one
// hemisphere).
static void LerpInto(const uint8_t* a, const uint8_t* b, float alpha, float* slot, uint32_t first,
                     uint32_t count)
{
    float out[4];
    for (uint32_t k = 0; k < count; ++k)
    {
        float va = LoadF32(a + 4 * k);
        float vb = LoadF32(b + 4 * k);
        out[k] = va + (vb - va) * alpha;
    }
    if (count == 4)
    {
        maniCodecNormalize(out);
    }
    maniPoseSlotPut(slot, first, out, count);
}

// A bone's channels at a pair into its pose slot, the samples' pointers
// moved past its track's floats; the rest where the track has none.
static void FullChannels(uint32_t mask, const uint8_t** a, const uint8_t** b,
                         const maniClipPair* pair, float* slot, const maniSkeleton* skeleton,
                         uint32_t bone, bool identity)
{
    float alpha = pair->alpha;
    static const uint32_t kCounts[3] = {3, 4, 3};
    for (uint32_t c = 0; c < 3; ++c)
    {
        if (((mask >> c) & 1u) == 0)
        {
            maniSlotRest(slot, skeleton, bone, c, identity);
            continue;
        }
        LerpInto(*a, *b, alpha, slot, maniChannelFirst(c), kCounts[c]);
        *a += 4 * kCounts[c];
        *b += 4 * kCounts[c];
    }
}

// The full-precision samples of a pair into the pose: every bone (or the
// listed ones) written once, from its track or its rest.
static void SampleFull(const maniClip* clip, const maniSkeleton* skeleton, const maniClipPair* pair,
                       const maniBoneList* list, maniPose* pose)
{
    size_t stride = 4 * (size_t)clip->floatsPerSample;
    const uint8_t* a = clip->samples + stride * pair->first;
    const uint8_t* b = clip->samples + stride * pair->second;
    float* locals = maniPoseLocals(pose);
    uint32_t track = 0;
    uint32_t next = 0;
    for (uint32_t bone = 0; bone < clip->skeletonBoneCount; ++bone)
    {
        const uint8_t* entry = clip->tracks + (size_t)track * MANI_CLIP_TRACK_SIZE;
        bool tracked = track < clip->trackCount && maniLoadU16(entry) == bone;
        uint32_t mask = tracked ? entry[2] : 0;
        track += tracked ? 1 : 0;
        if (!maniBoneListed(list, bone, &next))
        {
            size_t skip = 4 * (size_t)maniChannelFloats(mask);
            a += skip;
            b += skip;
            continue;
        }
        FullChannels(mask, &a, &b, pair, maniPoseSlot(locals, bone), skeleton, bone,
                     clip->additive != maniAdditiveNone);
    }
    maniPoseMarkStale(pose);
}

static void SampleFullFloats(const maniClip* clip, const maniClipPair* pair, float* values)
{
    size_t stride = 4 * (size_t)clip->floatTrackCount;
    const uint8_t* a = clip->floatSamples + stride * pair->first;
    const uint8_t* b = clip->floatSamples + stride * pair->second;
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        float va = LoadF32(a + 4 * (size_t)i);
        float vb = LoadF32(b + 4 * (size_t)i);
        bool step =
            maniLoadU32(clip->floatTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 16) != 0;
        // A step track holds the sample at or before the time.
        values[i] = step ? (pair->alpha < 1.0f ? va : vb) : va + (vb - va) * pair->alpha;
    }
}

void maniSampleClipPair(const maniClip* clip, const maniSkeleton* skeleton,
                        const maniClipPair* pair, const maniBoneList* list, maniPose* pose)
{
    if (clip->compressed)
    {
        maniSampleCompressedBones(clip, skeleton, pair, list, pose);
    }
    else
    {
        SampleFull(clip, skeleton, pair, list, pose);
    }
    if (list == NULL)
    {
        maniPoseSetKind(pose, clip->additive != maniAdditiveNone);
    }
}

void maniSampleClipPairFloats(const maniClip* clip, const maniClipPair* pair, float* values)
{
    if (clip->compressed)
    {
        maniSampleCompressedFloats(clip, pair, values);
    }
    else
    {
        SampleFullFloats(clip, pair, values);
    }
}

void maniSampleClipAt(const maniClip* clip, const maniSkeleton* skeleton, uint32_t k,
                      maniPose* pose, float* values)
{
    maniClipPair pair = {k, k, 0.0f, NULL, NULL};
    maniSampleClipPair(clip, skeleton, &pair, NULL, pose);
    if (values != NULL)
    {
        maniSampleClipPairFloats(clip, &pair, values);
    }
}

bool maniClipFits(const maniClip* clip, const maniSkeleton* skeleton, const maniPose* pose)
{
    return clip != NULL && maniPoseMatches(pose, skeleton) &&
           clip->skeletonIdentity == skeleton->identity &&
           clip->skeletonBoneCount == skeleton->boneCount;
}

maniResult maniSampleClip(const maniClip* clip, const maniSkeleton* skeleton, float time,
                          maniPose* pose)
{
    if (!maniClipFits(clip, skeleton, pose))
    {
        return mani_errorInvalid;
    }
    maniClipPair pair = maniClipLocate(clip, time);
    maniSampleClipPair(clip, skeleton, &pair, NULL, pose);
    return mani_success;
}

maniResult maniSampleClipBones(const maniClip* clip, const maniSkeleton* skeleton, float time,
                               const uint32_t* bones, uint32_t count, maniPose* pose)
{
    // A subset keeps the other bones, so it writes into a pose of the
    // clip's kind only.
    if (!maniClipFits(clip, skeleton, pose) || (bones == NULL && count > 0) ||
        maniPoseIsDifference(pose) != (clip->additive != maniAdditiveNone))
    {
        return mani_errorInvalid;
    }
    for (uint32_t i = 0; i < count; ++i)
    {
        if (bones[i] >= skeleton->boneCount || (i > 0 && bones[i] <= bones[i - 1]))
        {
            return mani_errorInvalid;
        }
    }
    maniClipPair pair = maniClipLocate(clip, time);
    maniBoneList list = {bones, count};
    maniSampleClipPair(clip, skeleton, &pair, &list, pose);
    return mani_success;
}

maniResult maniSampleClipFloats(const maniClip* clip, float time, float* values, uint32_t capacity,
                                uint32_t* countOut)
{
    if (clip == NULL || (values == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    if (countOut != NULL)
    {
        *countOut = clip->floatTrackCount;
    }
    if (capacity < clip->floatTrackCount)
    {
        return mani_errorCapacity;
    }
    maniClipPair pair = maniClipLocate(clip, time);
    maniSampleClipPairFloats(clip, &pair, values);
    return mani_success;
}
