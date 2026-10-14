// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Clips (mani-0006): a skeleton's motion as samples on one uniform grid,
// read in place from a clip file and bound to its skeleton by identity.
// Sampling costs the same at any time, in any order.

#ifndef MAUL_ANIM_CLIP_H
#define MAUL_ANIM_CLIP_H

#include "maul-anim/base.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// The most samples a clip holds: past 2^24, binary32 time cannot tell
// samples apart.
#define MANI_MAX_SAMPLES 16777216u

    // What a clip says happens past its end; a graph applies it, sampling
    // clamps.
    typedef uint32_t maniLoopPolicy;
    enum
    {
        maniLoopClamp = 0,
        maniLoopRepeat = 1,
    };

    // A clip over a file's bytes, which the caller keeps alive and
    // unchanged while the view is used. Its fields are the library's; read
    // a clip through the functions below. A view is immutable and
    // copyable.
    typedef struct maniClip
    {
        const uint8_t* tracks;
        const uint8_t* samples;
        uint64_t skeletonIdentity;
        uint32_t skeletonBoneCount;
        uint32_t sampleCount;
        uint32_t trackCount;
        uint32_t floatsPerSample;
        float sampleRate;
        float duration;
        maniLoopPolicy loop;
        uint32_t reserved;
    } maniClip;

    /// Opens a clip file in place: checks the container, the header, every
    /// track and every sample (finite, rotations unit) and the identity.
    ///
    /// @param bytes    The file, 16-byte aligned.
    /// @param size     Its size in bytes; bytes past the file are ignored.
    /// @param clipOut  Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for a newer format.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenClip(const void* bytes, size_t size,
                                                        maniClip* clipOut);

    /// Returns the clip's duration in seconds, at least 0.
    ///
    /// @param clip  An opened clip.
    /// @return The duration.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API float maniClipDuration(const maniClip* clip);

    /// Returns the number of samples per second.
    ///
    /// @param clip  An opened clip.
    /// @return The rate.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API float maniClipSampleRate(const maniClip* clip);

    /// Returns the number of samples, at least 1: one at each multiple of
    /// the sample period, the last at the duration.
    ///
    /// @param clip  An opened clip.
    /// @return The sample count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipSampleCount(const maniClip* clip);

    /// Returns the clip's loop policy.
    ///
    /// @param clip  An opened clip.
    /// @return maniLoopClamp or maniLoopRepeat.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniLoopPolicy maniClipLoopPolicy(const maniClip* clip);

    /// Returns the identity of the skeleton the clip is bound to.
    ///
    /// @param clip  An opened clip.
    /// @return The skeleton's identity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniClipSkeletonIdentity(const maniClip* clip);

    /// Samples the clip at a time into a pose: every bone's local
    /// transform, the clip's channels where it animates them and the
    /// skeleton's rest elsewhere; the model is marked stale. The time is
    /// clamped to [0, duration] (NaN is 0). Between two samples,
    /// translation and scale interpolate linearly and rotations by
    /// normalized linear interpolation.
    ///
    /// @param clip      An opened clip.
    /// @param skeleton  The clip's skeleton.
    /// @param time      The time in seconds.
    /// @param pose      A pose of the skeleton.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a skeleton that is not the clip's or the pose's.
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSampleClip(const maniClip* clip,
                                                      const maniSkeleton* skeleton, float time,
                                                      maniPose* pose);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_CLIP_H
