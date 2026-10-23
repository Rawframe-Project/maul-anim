// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a clip file, full precision or compressed, reading its named items and, for
// a clip of a seed's skeleton, sampling it; then the same bytes with the payload's hash made right,
// so that mutations reach the checks behind it. Run as fuzz_clip <corpus dir>; the seeds come from
// test_clip --seed <file> and test_clip_compress --seed <file>.

#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/pose.h"
#include "maul-anim/rootmotion.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

#define BONE(n, p, x, y)                                                                           \
    {                                                                                              \
        n, sizeof(n) - 1, p,                                                                       \
        {                                                                                          \
            {x, y, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}                             \
        }                                                                                          \
    }

// The seeds' skeletons, as their tests build them.
static const maniBoneSource kClipBones[4] = {BONE("root", -1, 0.5f, 0.0f), BONE("a", 0, 0.5f, 0.0f),
                                             BONE("b", 1, 0.5f, 0.0f), BONE("c", 0, 0.5f, 0.0f)};
static const maniBoneSource kCompressBones[6] = {
    BONE("root", -1, 0.0f, 0.0f), BONE("spine", 0, 0.0f, 0.3f), BONE("chest", 1, 0.0f, 0.3f),
    BONE("arm", 2, 0.0f, 0.3f),   BONE("hand", 3, 0.0f, 0.3f),  BONE("leg", 0, 0.0f, -0.3f)};

static _Alignas(16) uint8_t s_skeletonFiles[2][4096];
static maniSkeleton s_skeletons[2];
static int s_skeletonCount = -1;
static _Alignas(32) uint8_t s_poseMemory[8192];
static _Alignas(32) uint8_t s_cursorPoseMemory[8192];
static _Alignas(8) uint8_t s_cursorMemory[1 << 16];
static float s_values[64];
static float s_cursorValues[64];

static void BuildSkeletons(void)
{
    s_skeletonCount = 0;
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(&def, &builder) != mani_success)
    {
        return;
    }
    const maniSkeletonSource sources[2] = {{kClipBones, 4, 0}, {kCompressBones, 6, 0}};
    for (int i = 0; i < 2; ++i)
    {
        size_t size = 0;
        if (maniBuildSkeleton(builder, &sources[i]).result == mani_success &&
            maniCopyBuilt(builder, s_skeletonFiles[i], sizeof s_skeletonFiles[i], &size) ==
                mani_success &&
            maniOpenSkeleton(s_skeletonFiles[i], size, &s_skeletons[s_skeletonCount]).result ==
                mani_success)
        {
            ++s_skeletonCount;
        }
    }
    maniDestroyBuilder(builder);
}

// The same bits from random access and through a cursor, and for some
// bones alone.
static void Compare(const maniClip* clip, const maniSkeleton* skeleton, const maniPose* pose,
                    maniClipCursor* cursor, maniPose* other, float time)
{
    if (maniSampleClipCursor(cursor, clip, skeleton, time, other) == mani_success)
    {
        for (uint32_t b = 0; b < maniSkeletonBoneCount(skeleton); ++b)
        {
            maniTransform x = maniPoseLocal(pose, b);
            maniTransform y = maniPoseLocal(other, b);
            if (memcmp(&x, &y, sizeof x) != 0)
            {
                abort();
            }
        }
    }
    uint32_t bones[2] = {0, maniSkeletonBoneCount(skeleton) - 1};
    if (maniSampleClipBones(clip, skeleton, time, bones, bones[1] > 0 ? 2 : 1, other) ==
        mani_success)
    {
        for (uint32_t i = 0; i < 2; ++i)
        {
            maniTransform x = maniPoseLocal(pose, bones[i]);
            maniTransform y = maniPoseLocal(other, bones[i]);
            if (memcmp(&x, &y, sizeof x) != 0)
            {
                abort();
            }
        }
    }
    uint32_t n = 0;
    if (maniSampleClipFloats(clip, time, s_values, 64, &n) == mani_success &&
        maniSampleClipFloatsCursor(cursor, clip, time, s_cursorValues, 64, &n) == mani_success &&
        memcmp(s_values, s_cursorValues, n * sizeof(float)) != 0)
    {
        abort();
    }
}

// Samples a clip of a seed's skeleton at times inside and outside it.
static void Sample(const maniClip* clip)
{
    for (int i = 0; i < s_skeletonCount; ++i)
    {
        const maniSkeleton* skeleton = &s_skeletons[i];
        maniPose* pose = NULL;
        maniPose* other = NULL;
        maniClipCursor* cursor = NULL;
        if (maniClipSkeletonIdentity(clip) != maniSkeletonIdentity(skeleton) ||
            maniInitPose(skeleton, s_poseMemory, sizeof s_poseMemory, &pose) != mani_success ||
            maniInitPose(skeleton, s_cursorPoseMemory, sizeof s_cursorPoseMemory, &other) !=
                mani_success ||
            maniInitClipCursor(s_cursorMemory, sizeof s_cursorMemory, &cursor) != mani_success)
        {
            continue;
        }
        float d = maniClipDuration(clip);
        const float times[6] = {-1.0f, 0.0f, d * 0.37f, d * 0.5f, d, d + 1.0f};
        for (int t = 0; t < 6; ++t)
        {
            maniResult bones = maniSampleClip(clip, skeleton, times[t], pose);
            maniResult floats = maniSampleClipFloats(clip, times[t], s_values, 64, NULL);
            bool fits = maniClipFloatTrackCount(clip) <= 64;
            if ((clip->skeletonBoneCount == maniSkeletonBoneCount(skeleton)) !=
                    (bones == mani_success) ||
                (floats == mani_success) != fits)
            {
                abort();
            }
            if (bones == mani_success)
            {
                Compare(clip, skeleton, pose, cursor, other, times[t]);
            }
        }
    }
}

static bool MoveFinite(const maniRootMove* m)
{
    return isfinite(m->translation.x) && isfinite(m->translation.y) && isfinite(m->translation.z) &&
           isfinite(m->rotation.x) && isfinite(m->rotation.y) && isfinite(m->rotation.z) &&
           isfinite(m->rotation.w);
}

// An opened clip's moves are finite, across wraps either way, and blend.
static void ExerciseRootMotion(const maniClip* clip)
{
    static const float kSteps[5][2] = {
        {0.0f, 0.37f}, {0.5f, -3.1f}, {-2.0f, 1e9f}, {1e30f, -1e30f}, {0.25f, 0.0f}};
    maniRootMove moves[5];
    const float weights[5] = {1.0f, 0.5f, 0.0f, 2.0f, 1.0f};
    for (int i = 0; i < 5; ++i)
    {
        if (maniClipRootMotion(clip, kSteps[i][0], kSteps[i][1], &moves[i]) != mani_success ||
            !MoveFinite(&moves[i]))
        {
            abort();
        }
    }
    maniRootMove blend;
    if (maniBlendRootMotion(moves, weights, 5, &blend) != mani_success || !MoveFinite(&blend))
    {
        abort();
    }
}

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniClip clip;
    if (maniOpenClip(bytes, size, &clip).result == 0)
    {
        if (maniClipSampleCount(&clip) == 0 || !(maniClipDuration(&clip) >= 0.0f) ||
            !(maniClipSampleRate(&clip) > 0.0f))
        {
            abort();
        }
        // An opened clip's root motion source is in range.
        maniRootMotionSource root;
        if (maniClipRootMotionSource(&clip, &root) &&
            (root.translationAxes > 7 || root.rotation > maniRootRotationWhole ||
             (root.translationAxes == 0 && root.rotation == maniRootRotationNone)))
        {
            abort();
        }
        ExerciseRootMotion(&clip);
        for (uint32_t i = 0; i < maniClipFloatTrackCount(&clip); ++i)
        {
            uint32_t length = 0;
            const char* name = maniClipFloatTrackName(&clip, i, &length);
            if (maniClipFindFloatTrack(&clip, name, length) != i)
            {
                abort();
            }
        }
        for (uint32_t i = 0; i < maniClipDiscreteTrackCount(&clip); ++i)
        {
            uint32_t length = 0;
            const char* name = maniClipDiscreteTrackName(&clip, i, &length);
            if (maniClipFindDiscreteTrack(&clip, name, length) != i)
            {
                abort();
            }
            (void)maniClipDiscreteValue(&clip, i, maniClipDuration(&clip) * 0.5f);
        }
        uint32_t first = 0;
        uint32_t count = 0;
        maniClipEventRange(&clip, 0.0f, maniClipDuration(&clip), &first, &count);
        if (first + count > maniClipEventCount(&clip))
        {
            abort();
        }
        Sample(&clip);
    }
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    // The file is opened from a 16-byte aligned copy, as a host would;
    // aligned by hand, as aligned_alloc is not everywhere.
    if (s_skeletonCount < 0)
    {
        BuildSkeletons();
    }
    uint8_t* block = malloc(size + 16);
    if (block == NULL)
    {
        return 0;
    }
    uint8_t* copy = block + (16 - (uintptr_t)block % 16) % 16;
    if (size > 0)
    {
        memcpy(copy, data, size);
    }
    Exercise(copy, size);
    uint64_t fileSize = 0;
    if (size >= 64)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= 64 && fileSize <= size && fileSize - 64 < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + 64, (int32_t)(fileSize - 64));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
