// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A compressed clip's held sample in a pose's group layout, for the
// forward cursor's lanes: the decoded floats (maniDecodeCompressedSample's)
// and the constants placed at their bones, the other channels at rest.

#include "bytes.h"
#include "clip_codec.h"
#include "clip_file.h"
#include "clip_lanes.h"
#include "clip_view.h"
#include "invariant.h"
#include "pose_memory.h"

#include "maul-anim/clip.h"
#include "maul-anim/pose.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(MANI_CLIP_LANES)
static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
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

static float* MaskOf(float* masks, uint32_t bone, uint32_t channel)
{
    return masks + (size_t)(bone / MANI_POSE_GROUP_BONES) * 3 * MANI_POSE_GROUP_BONES +
           channel * MANI_POSE_GROUP_BONES + bone % MANI_POSE_GROUP_BONES;
}

// A held sample (maniDecodeCompressedSample's floats) in a pose's group
// layout for the forward cursor: each channel the clip stores at its
// value (a constant read, a rotation rebuilt), the others at rest as the
// walk writes them, padding lanes the identity; in masks (three rows of
// eight a group: translation, rotation, scale) all bits set where the
// walk interpolates the channel, clear where it writes a rest.
// A stored channel's held value for the groups: the next decoded one
// (animated) or the next constant, a rotation rebuilt.
static void HeldValue(const uint8_t* dropped, uint32_t channel, uint32_t t, bool animated,
                      const float** cache, const uint8_t** constant, float v[4])
{
    uint32_t width = channel == 1 ? 4 : 3;
    if (animated)
    {
        memcpy(v, *cache, width * sizeof(float));
        *cache += width;
        return;
    }
    float kept[3];
    for (int k = 0; k < 3; ++k)
    {
        kept[k] = LoadF32(*constant + 4 * k);
    }
    *constant += 12;
    if (channel == 1)
    {
        maniCodecRotation(kept, 3u - maniCompressedDroppedField(dropped, t), v);
        return;
    }
    memcpy(v, kept, sizeof kept);
}

void maniExpandCompressedSample(const maniClip* clip, const maniSkeleton* skeleton,
                                const float* cache, float* groups, float* masks)
{
    uint32_t bones = clip->skeletonBoneCount;
    bool identity = clip->additive != maniAdditiveNone;
    for (uint32_t bone = 0; bone < bones; ++bone)
    {
        for (uint32_t c = 0; c < 3; ++c)
        {
            maniSlotRest(maniPoseSlot(groups, bone), skeleton, bone, c, identity);
            *MaskOf(masks, bone, c) = 0.0f;
        }
    }
    const uint8_t* constant = maniClipSection(clip, maniClipTagConstants);
    const uint8_t* tracks = maniClipSection(clip, maniClipTagTracks);
    const uint8_t* dropped = maniCompressedDropped(clip);
    uint32_t all = UINT32_MAX;
    for (uint32_t kind = 0; kind < maniClipKinds; ++kind)
    {
        uint32_t channel = maniClipKindChannel(kind);
        for (uint32_t animated = 0; animated < 2; ++animated)
        {
            maniTrackWalk walk = maniWalkTracks(clip, kind, animated);
            uint32_t n = animated != 0 ? clip->animatedKinds[kind] : clip->constantKinds[kind];
            for (uint32_t i = 0; i < n; ++i)
            {
                uint32_t t = maniNextTrack(&walk);
                uint32_t bone = maniClipTrackBoneIn(tracks, bones, t);
                float v[4];
                HeldValue(dropped, channel, t, animated != 0, &cache, &constant, v);
                maniPoseSlotPut(maniPoseSlot(groups, bone), maniChannelFirst(channel), v,
                                channel == 1 ? 4 : 3);
                memcpy(MaskOf(masks, bone, channel), &all, sizeof all);
            }
        }
    }
    MANI_ASSERT(constant ==
                maniClipSection(clip, maniClipTagConstants) + 12 * (size_t)clip->constantChannels);
    HeldPadding(bones, groups, masks);
}
#endif
