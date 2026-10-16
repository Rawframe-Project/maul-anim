// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A clip's root motion (mani-0022): its section checked at open, and its
// source read back.

#include "root_motion.h"

#include "bytes.h"
#include "clip_file.h"

#include "maul-anim/base.h"
#include "maul-anim/clip.h"
#include "maul-anim/rootmotion.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static maniFileResult Fail(uint64_t offset)
{
    return (maniFileResult){mani_errorFormat, 0, offset};
}

static bool Finite(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    return (bits & 0x7F800000u) != 0x7F800000u;
}

// The source's fields in range, with a channel chosen.
static bool SourceValid(const maniRootMotionSource* s)
{
    return s->translationAxes <= 7u && s->rotation <= maniRootRotationWhole && s->twistAxis <= 2u &&
           s->translationReference <= maniRootReferenceOrigin &&
           s->rotationReference <= maniRootReferenceOrigin && s->loop <= 1u && s->reserved == 0u &&
           (s->translationAxes != 0u || s->rotation != maniRootRotationNone);
}

bool maniRootMotionSourceValid(const maniRootMotionSource* source)
{
    return SourceValid(source);
}

static maniRootMotionSource LoadSource(const uint8_t* h)
{
    return (maniRootMotionSource){maniLoadU32(h),      maniLoadU32(h + 4),  maniLoadU32(h + 8),
                                  maniLoadU32(h + 12), maniLoadU32(h + 16), maniLoadU32(h + 20),
                                  maniLoadU32(h + 24), maniLoadU32(h + 28)};
}

// Whether the clip's tracks hold the bone's translation and rotation.
static bool BoneTracked(const maniClip* clip, uint32_t bone)
{
    for (uint32_t i = 0; i < clip->trackCount; ++i)
    {
        const uint8_t* t = clip->tracks + (size_t)i * MANI_CLIP_TRACK_SIZE;
        uint32_t mask = maniChannelTranslation | maniChannelRotation;
        if (maniLoadU16(t) == bone)
        {
            return (t[2] & mask) == mask;
        }
    }
    return false;
}

maniFileResult maniCheckClipRootMotion(const uint8_t* data, uint64_t size, uint64_t at,
                                       uint64_t entry, maniClip* clip)
{
    clip->rootMotion = NULL;
    if (size == 0)
    {
        return (maniFileResult){mani_success, 0, 0};
    }
    if (size != maniRootMotionSize(clip->sampleCount))
    {
        return Fail(entry);
    }
    maniRootMotionSource s = LoadSource(data);
    if (!SourceValid(&s) || clip->additive != maniAdditiveNone)
    {
        return Fail(at);
    }
    if (s.bone >= clip->skeletonBoneCount || !BoneTracked(clip, s.bone))
    {
        return Fail(at);
    }
    uint64_t floats = (size - MANI_ROOT_MOTION_HEADER_SIZE) / 4;
    for (uint64_t k = 0; k < floats; ++k)
    {
        if (!Finite(data + MANI_ROOT_MOTION_HEADER_SIZE + 4 * k))
        {
            return Fail(at + MANI_ROOT_MOTION_HEADER_SIZE + 4 * k);
        }
    }
    if (maniLoadU32(data + MANI_ROOT_MOTION_HEADER_SIZE + 28) != 0)
    {
        return Fail(at + MANI_ROOT_MOTION_HEADER_SIZE + 28);
    }
    clip->rootMotion = data;
    return (maniFileResult){mani_success, 0, 0};
}

bool maniClipRootMotionSource(const maniClip* clip, maniRootMotionSource* sourceOut)
{
    if (clip == NULL || clip->rootMotion == NULL || sourceOut == NULL)
    {
        return false;
    }
    *sourceOut = LoadSource(clip->rootMotion);
    return true;
}
