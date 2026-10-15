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
        const uint8_t* floatTracks;
        const uint8_t* floatSamples;
        const uint8_t* discreteTracks;
        const uint8_t* discreteKeys;
        const uint8_t* events;
        const uint8_t* markers;
        const uint8_t* names;
        // A compressed clip's sections (NULL at full precision).
        const uint8_t* forms;
        const uint8_t* constants;
        const uint8_t* ranges;
        const uint8_t* segments;
        const uint8_t* segmentData;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint64_t namesSize;
        uint32_t skeletonBoneCount;
        uint32_t sampleCount;
        uint32_t trackCount;
        uint32_t floatsPerSample;
        uint32_t floatTrackCount;
        uint32_t discreteTrackCount;
        uint32_t eventCount;
        uint32_t markerCount;
        uint32_t segmentCount;
        uint32_t compressed;
        uint32_t animatedChannels;
        uint32_t animatedFloats;
        uint32_t constantChannels;
        uint32_t animatedRotations;
        float sampleRate;
        float duration;
        maniLoopPolicy loop;
        uint32_t reserved;
    } maniClip;

// What a lookup by name finds for no track.
#define MANI_NO_TRACK 0xFFFFFFFFu

    // An event or a sync marker: its time, payload (0 for a marker) and
    // name, the name's bytes inside the file, UTF-8, not terminated.
    typedef struct maniClipEvent
    {
        float time;
        int32_t payload;
        uint64_t nameHash;
        const char* name;
        uint32_t nameLength;
        uint32_t reserved;
    } maniClipEvent;

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

    /// Returns whether a clip is compressed (mani-0019); a full-precision
    /// clip is the build's output and the reference.
    ///
    /// @param clip  An opened clip.
    /// @return True for a compressed clip.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API bool maniClipCompressed(const maniClip* clip);

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

    /// Returns the clip's identity: a hash of its sections, the same for
    /// the same bytes.
    ///
    /// @param clip  An opened clip.
    /// @return The identity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniClipIdentity(const maniClip* clip);

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

    /// Returns the number of float tracks (morph weights, curves).
    ///
    /// @param clip  An opened clip.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipFloatTrackCount(const maniClip* clip);

    /// Returns a float track's name, UTF-8 and not terminated.
    ///
    /// @param clip       An opened clip.
    /// @param track      The track, below the count.
    /// @param lengthOut  Receives the length in bytes, 0 for a track past
    ///                   the count. May be NULL.
    /// @return The name's bytes, or NULL for a track past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniClipFloatTrackName(const maniClip* clip, uint32_t track,
                                                uint32_t* lengthOut);

    /// Samples some bones of the clip at a time, as maniSampleClip samples
    /// them; every other bone's local is left as it was, and the clip's
    /// other channels are not decoded.
    ///
    /// @param clip      An opened clip.
    /// @param skeleton  The clip's skeleton.
    /// @param time      The time in seconds.
    /// @param bones     The bones, strictly increasing; may be NULL when
    ///                  count is 0.
    /// @param count     How many.
    /// @param pose      A pose of the skeleton.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// skeleton that is not the clip's or the pose's, bones out of range
    /// or not increasing, or a difference pose (the pose unchanged).
    /// @par Thread safety
    /// Safe from any thread; the pose is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSampleClipBones(const maniClip* clip,
                                                           const maniSkeleton* skeleton, float time,
                                                           const uint32_t* bones, uint32_t count,
                                                           maniPose* pose);

    // A cursor for playing clips forward: caller memory caching
    // the decoded samples a time falls between, so a time moving by
    // less than a sample decodes nothing and one moving by one sample
    // decodes one. Poses and values are the same bits as without it.
    typedef struct maniClipCursor maniClipCursor;

#define MANI_CLIP_CURSOR_ALIGNMENT 8u

    /// Returns the bytes a cursor needs to play a clip: a cursor made for
    /// a clip plays any clip that needs no more.
    ///
    /// @param clip  An opened clip.
    /// @return The size in bytes; 0 for a NULL clip.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API size_t maniClipCursorSize(const maniClip* clip);

    /// Makes a cursor in caller memory, bound to no clip yet.
    ///
    /// @param memory     At least maniClipCursorSize bytes,
    ///                   MANI_CLIP_CURSOR_ALIGNMENT-aligned; the cursor
    ///                   lives in it.
    /// @param size       The memory's size; what it holds beyond the
    ///                   smallest cursor sets the clips it can play.
    /// @param cursorOut  Receives the cursor.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL or misaligned
    /// argument; `mani_errorCapacity` when the memory is too small for
    /// any cursor.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniInitClipCursor(void* memory, size_t size,
                                                          maniClipCursor** cursorOut);

    /// Samples the clip at a time into a pose, as maniSampleClip does,
    /// through the cursor: a clip of another identity or at another
    /// address rebinds it, dropping what it held.
    ///
    /// @param cursor    A cursor.
    /// @param clip      An opened clip.
    /// @param skeleton  The clip's skeleton.
    /// @param time      The time in seconds.
    /// @param pose      A pose of the skeleton.
    /// @return `mani_success`; `mani_errorInvalid` as maniSampleClip, or
    /// for a NULL cursor; `mani_errorCapacity` when the cursor is too
    /// small for the clip (the cursor and pose unchanged).
    /// @par Thread safety
    /// Safe from any thread; the cursor and pose are used by one thread
    /// at a time.
    MANI_NODISCARD MANI_API maniResult maniSampleClipCursor(maniClipCursor* cursor,
                                                            const maniClip* clip,
                                                            const maniSkeleton* skeleton,
                                                            float time, maniPose* pose);

    /// Samples every float track at a time, as maniSampleClipFloats does,
    /// through the cursor.
    ///
    /// @param cursor    A cursor.
    /// @param clip      An opened clip.
    /// @param time      The time in seconds.
    /// @param values    Receives one value per track; may be NULL when
    ///                  capacity is 0.
    /// @param capacity  The array's length.
    /// @param countOut  Receives the track count. May be NULL.
    /// @return As maniSampleClipFloats; `mani_errorInvalid` also for a NULL
    /// cursor; `mani_errorCapacity` also when the cursor is too small.
    /// @par Thread safety
    /// Safe from any thread; the cursor is used by one thread at a time.
    MANI_NODISCARD MANI_API maniResult maniSampleClipFloatsCursor(maniClipCursor* cursor,
                                                                  const maniClip* clip, float time,
                                                                  float* values, uint32_t capacity,
                                                                  uint32_t* countOut);

    /// Finds a float track by name.
    ///
    /// @param clip    An opened clip.
    /// @param name    The name's UTF-8 bytes; may be NULL when length is 0.
    /// @param length  The name's length in bytes.
    /// @return The track, or MANI_NO_TRACK.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipFindFloatTrack(const maniClip* clip, const char* name, size_t length);

    /// Samples every float track at a time, clamped as maniSampleClip
    /// clamps it: a linear track interpolates its two samples, a step
    /// track holds the sample at or before the time.
    ///
    /// @param clip      An opened clip.
    /// @param time      The time in seconds.
    /// @param values    Receives one value per track, in track order; may
    ///                  be NULL when capacity is 0.
    /// @param capacity  The array's length.
    /// @param countOut  Receives the track count. May be NULL.
    /// @return `mani_success`; `mani_errorCapacity` when the array is
    /// shorter than the track count, with the count written and nothing
    /// else; `mani_errorInvalid` for a NULL clip or a NULL array with a
    /// positive capacity.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniSampleClipFloats(const maniClip* clip, float time,
                                                            float* values, uint32_t capacity,
                                                            uint32_t* countOut);

    /// Returns the number of discrete tracks (whole numbers held between
    /// keys).
    ///
    /// @param clip  An opened clip.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipDiscreteTrackCount(const maniClip* clip);

    /// Returns a discrete track's name, as maniClipFloatTrackName does.
    ///
    /// @param clip       An opened clip.
    /// @param track      The track.
    /// @param lengthOut  Receives the length in bytes. May be NULL.
    /// @return The name's bytes, or NULL for a track past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniClipDiscreteTrackName(const maniClip* clip, uint32_t track,
                                                   uint32_t* lengthOut);

    /// Finds a discrete track by name.
    ///
    /// @param clip    An opened clip.
    /// @param name    The name's UTF-8 bytes; may be NULL when length is 0.
    /// @param length  The name's length in bytes.
    /// @return The track, or MANI_NO_TRACK.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipFindDiscreteTrack(const maniClip* clip, const char* name,
                                                size_t length);

    /// Returns a discrete track's value at a time: its last key at or
    /// before the time, or its first key before that (NaN is 0).
    ///
    /// @param clip   An opened clip.
    /// @param track  The track.
    /// @param time   The time in seconds.
    /// @return The value, or 0 for a track past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API int32_t maniClipDiscreteValue(const maniClip* clip, uint32_t track, float time);

    /// Returns the number of events.
    ///
    /// @param clip  An opened clip.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipEventCount(const maniClip* clip);

    /// Returns an event; events are in increasing time.
    ///
    /// @param clip   An opened clip.
    /// @param index  The event.
    /// @return The event, or a zeroed one for an index past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniClipEvent maniClipEventAt(const maniClip* clip, uint32_t index);

    /// Finds the events a playhead passes going from one time to another:
    /// forward (from below to) those with from <= time < to, backward
    /// (from above to) those with to < time <= from; none when the two are
    /// equal or either is NaN. They are consecutive indices.
    ///
    /// @param clip      An opened clip.
    /// @param from      Where the playhead was.
    /// @param to        Where it is.
    /// @param firstOut  Receives the first event's index.
    /// @param countOut  Receives how many.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API void maniClipEventRange(const maniClip* clip, float from, float to, uint32_t* firstOut,
                                     uint32_t* countOut);

    /// Returns the number of sync markers.
    ///
    /// @param clip  An opened clip.
    /// @return The count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint32_t maniClipMarkerCount(const maniClip* clip);

    /// Returns a sync marker; markers are in increasing time.
    ///
    /// @param clip   An opened clip.
    /// @param index  The marker.
    /// @return The marker, or a zeroed one for an index past the count.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API maniClipEvent maniClipMarkerAt(const maniClip* clip, uint32_t index);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_CLIP_H
