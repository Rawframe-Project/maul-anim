// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a clip file and sampling it. A time maps to two samples and a
// blend factor by binary32 operations alone, so the same time samples the
// same bits everywhere and costs the same in any order.

#include "maul-anim/clip.h"

#include "bytes.h"
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

uint64_t maniClipComputeIdentity(const uint8_t* const sections[MANI_CLIP_SECTIONS],
                                 const uint64_t sizes[MANI_CLIP_SECTIONS])
{
    uint64_t hash = MANI_HASH_INIT;
    for (uint32_t i = 0; i < MANI_CLIP_SECTIONS; ++i)
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
    for (uint32_t i = 52; i < MANI_CLIP_HEADER_SIZE; ++i)
    {
        if (h[i] != 0)
        {
            return Fail(at + i);
        }
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
        .sampleRate = rate,
        .duration = duration,
        .loop = maniLoadU32(h + 24),
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
    if (c.sectionCount != MANI_CLIP_SECTIONS)
    {
        return Fail(12);
    }
    const uint8_t* data[MANI_CLIP_SECTIONS];
    uint64_t sizes[MANI_CLIP_SECTIONS];
    uint64_t at[MANI_CLIP_SECTIONS];
    for (uint32_t i = 0; i < MANI_CLIP_SECTIONS; ++i)
    {
        if (!maniContainerSection(&c, i + 1, &data[i], &sizes[i], &at[i]))
        {
            return Fail(MANI_CONTAINER_HEADER_SIZE + (uint64_t)i * MANI_CONTAINER_ENTRY_SIZE);
        }
    }
    if (sizes[0] != MANI_CLIP_HEADER_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + 16);
    }
    maniClip clip;
    r = CheckHeader(data[0], at[0], &clip);
    if (r.result != mani_success)
    {
        return r;
    }
    if (sizes[1] != (uint64_t)clip.trackCount * MANI_CLIP_TRACK_SIZE)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + MANI_CONTAINER_ENTRY_SIZE + 16);
    }
    r = CheckTracks(data[1], at[1], at[0], &clip);
    if (r.result != mani_success)
    {
        return r;
    }
    uint64_t sampleBytes = 0;
    if (ckd_mul(&sampleBytes, (uint64_t)clip.sampleCount, 4 * (uint64_t)clip.floatsPerSample) ||
        sizes[2] != sampleBytes)
    {
        return Fail(MANI_CONTAINER_HEADER_SIZE + 2 * MANI_CONTAINER_ENTRY_SIZE + 16);
    }
    r = CheckSamples(data[2], at[2], data[1], &clip);
    if (r.result == mani_success)
    {
        r = maniCheckClipNamed(data, sizes, at, &clip);
    }
    if (r.result != mani_success)
    {
        return r;
    }
    if (maniClipComputeIdentity(data, sizes) != c.identity)
    {
        return Fail(40);
    }
    clip.tracks = data[1];
    clip.samples = data[2];
    *clipOut = clip;
    return (maniFileResult){mani_success, 0, 0};
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

uint64_t maniClipSkeletonIdentity(const maniClip* clip)
{
    return clip->skeletonIdentity;
}

// The two samples around a time and the factor between them.
static void Locate(const maniClip* clip, float time, uint32_t* firstOut, float* alphaOut)
{
    *firstOut = 0;
    *alphaOut = 0.0f;
    uint32_t n = clip->sampleCount;
    if (n == 1)
    {
        return;
    }
    float t = time > 0.0f ? time : 0.0f;
    t = t < clip->duration ? t : clip->duration;
    float position = floorf(t * clip->sampleRate);
    uint32_t i = position < (float)(n - 2) ? (uint32_t)position : n - 2;
    float start = (float)i / clip->sampleRate;
    float end = i + 1 == n - 1 ? clip->duration : (float)(i + 1) / clip->sampleRate;
    float alpha = end > start ? (t - start) / (end - start) : 0.0f;
    *firstOut = i;
    *alphaOut = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
}

static maniVec3 LerpVec3(const uint8_t* a, const uint8_t* b, float alpha)
{
    maniVec3 out;
    float* o = &out.x;
    for (int k = 0; k < 3; ++k)
    {
        float va = LoadF32(a + 4 * k);
        float vb = LoadF32(b + 4 * k);
        o[k] = va + (vb - va) * alpha;
    }
    return out;
}

static maniQuat NlerpQuat(const uint8_t* a, const uint8_t* b, float alpha)
{
    // The builder put consecutive samples in one hemisphere.
    maniQuat q;
    float* o = &q.x;
    for (int k = 0; k < 4; ++k)
    {
        float va = LoadF32(a + 4 * k);
        float vb = LoadF32(b + 4 * k);
        o[k] = va + (vb - va) * alpha;
    }
    return maniQuatNormalize(q);
}

maniResult maniSampleClip(const maniClip* clip, const maniSkeleton* skeleton, float time,
                          maniPose* pose)
{
    if (clip == NULL || !maniPoseMatches(pose, skeleton) ||
        clip->skeletonIdentity != skeleton->identity ||
        clip->skeletonBoneCount != skeleton->boneCount)
    {
        return mani_errorInvalid;
    }
    maniResult result = maniPoseSetRestLocals(pose, skeleton);
    if (result != mani_success)
    {
        return result;
    }
    uint32_t first = 0;
    float alpha = 0.0f;
    Locate(clip, time, &first, &alpha);
    uint32_t second = clip->sampleCount > 1 ? first + 1 : first;
    size_t stride = 4 * (size_t)clip->floatsPerSample;
    const uint8_t* a = clip->samples + stride * first;
    const uint8_t* b = clip->samples + stride * second;
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        const uint8_t* entry = clip->tracks + (size_t)i * MANI_CLIP_TRACK_SIZE;
        uint32_t bone = maniLoadU16(entry);
        uint32_t mask = entry[2];
        maniTransform local = maniPoseLocal(pose, bone);
        if (mask & maniChannelTranslation)
        {
            local.translation = LerpVec3(a, b, alpha);
            a += 12;
            b += 12;
        }
        if (mask & maniChannelRotation)
        {
            local.rotation = NlerpQuat(a, b, alpha);
            a += 16;
            b += 16;
        }
        if (mask & maniChannelScale)
        {
            local.scale = LerpVec3(a, b, alpha);
            a += 12;
            b += 12;
        }
        maniPoseSetLocal(pose, bone, local);
    }
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
    uint32_t first = 0;
    float alpha = 0.0f;
    Locate(clip, time, &first, &alpha);
    uint32_t second = clip->sampleCount > 1 ? first + 1 : first;
    size_t stride = 4 * (size_t)clip->floatTrackCount;
    const uint8_t* a = clip->floatSamples + stride * first;
    const uint8_t* b = clip->floatSamples + stride * second;
    for (uint32_t i = 0; i < clip->floatTrackCount; ++i)
    {
        float va = LoadF32(a + 4 * (size_t)i);
        float vb = LoadF32(b + 4 * (size_t)i);
        bool step =
            maniLoadU32(clip->floatTracks + (size_t)i * MANI_CLIP_NAMED_TRACK_SIZE + 16) != 0;
        // A step track holds the sample at or before the time.
        values[i] = step ? (alpha < 1.0f ? va : vb) : va + (vb - va) * alpha;
    }
    return mani_success;
}
