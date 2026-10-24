// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Motion matching: a motion database built offline from a
// skeleton's clips, opened in place. Each frame of the database is a clip
// at a time; its features describe the pose and the root's path around
// that time in the root's space, normalized per channel, so a search
// compares a query's features with every frame's.

#ifndef MAUL_ANIM_MATCH_H
#define MAUL_ANIM_MATCH_H

#include "maul-anim/base.h"
#include "maul-anim/clip.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Named limits: channels of one database, times of one channel, features
// of one frame, entries, clips and frames of one database.
#define MANI_MATCH_MAX_CHANNELS 64u
#define MANI_MATCH_MAX_TIMES    16u
#define MANI_MATCH_MAX_FEATURES 1024u
#define MANI_MATCH_MAX_ENTRIES  65536u
#define MANI_MATCH_MAX_CLIPS    65536u
#define MANI_MATCH_MAX_FRAMES   16777216u
// The furthest a channel's time reaches from its frame, seconds.
#define MANI_MATCH_MAX_REACH 60.0f
// The highest sample rate, frames a second.
#define MANI_MATCH_MAX_RATE 1000.0f

    // What a channel describes at each of its times, in the root's space
    // at the frame: a bone's position (3 features), its velocity (3), its
    // axis (3); the root's position on the ground plane (2) and its
    // facing there (2); a float track's value (1).
    typedef uint32_t maniMatchChannelKind;
    enum
    {
        maniMatchBonePosition = 0,
        maniMatchBoneVelocity = 1,
        maniMatchBoneFacing = 2,
        maniMatchTrajectoryPosition = 3,
        maniMatchTrajectoryFacing = 4,
        maniMatchFloatTrack = 5,
    };

    // An axis and its sign.
    typedef uint32_t maniMatchAxis;
    enum
    {
        maniMatchAxisX = 0,
        maniMatchAxisY = 1,
        maniMatchAxisZ = 2,
        maniMatchAxisNegativeX = 3,
        maniMatchAxisNegativeY = 4,
        maniMatchAxisNegativeZ = 5,
    };

// No frame: a search with no continuing frame, or one that found none.
#define MANI_MATCH_NO_FRAME 0xFFFFFFFFu
// The most earlier picks a search excludes.
#define MANI_MATCH_MAX_HISTORY 8u

// An entry's flags: its times wrap from its end to its start (else they
// clamp to its ends); its frames play but are never jumped to; its clip
// plays mirrored through the database's mirror map.
#define MANI_MATCH_LOOP     1u
#define MANI_MATCH_NO_JUMP  2u
#define MANI_MATCH_MIRRORED 4u

    // A channel as the database holds it: its kind, bone (bone kinds),
    // axis (a bone's facing), times (the first timeCount), weight, and its
    // features' place in a frame (the times' features in order).
    typedef struct maniMatchChannel
    {
        maniMatchChannelKind kind;
        uint32_t bone;
        maniMatchAxis axis;
        uint32_t timeCount;
        float times[MANI_MATCH_MAX_TIMES];
        float weight;
        uint32_t firstFeature;
        uint32_t featureCount;
        uint32_t reserved;
    } maniMatchChannel;

    // An entry: a clip's span the database holds, from start to end
    // seconds, its flags, a cost added to its frames, its tags, and its
    // frames' place (start + k / rate for its k-th frame).
    typedef struct maniMatchEntry
    {
        uint32_t clip;
        uint32_t flags;
        float start;
        float end;
        float costBias;
        uint32_t tags;
        uint32_t firstFrame;
        uint32_t frameCount;
    } maniMatchEntry;

    // An opened database: a view over the caller's bytes, which must
    // outlive it.
    typedef struct maniMotionDatabase
    {
        const uint8_t* channels;
        const uint8_t* times;
        const uint8_t* clips;
        const uint8_t* entries;
        const uint8_t* normalization;
        const uint8_t* features;
        const uint8_t* boxes;
        const uint8_t* names;
        const uint8_t* nodes;
        const uint8_t* positions;
        const uint8_t* places;
        uint64_t identity;
        uint64_t skeletonIdentity;
        uint64_t namesSize;
        uint64_t mirrorMapIdentity;
        uint32_t skeletonBoneCount;
        uint32_t frameCount;
        uint32_t featureCount;
        uint32_t channelCount;
        uint32_t timeCount;
        uint32_t entryCount;
        uint32_t clipCount;
        float sampleRate;
        maniMatchAxis up;
        maniMatchAxis forward;
        uint32_t leafFrames;
        uint32_t nodeCount;
    } maniMotionDatabase;

    /// Opens a motion database file in place: checks the container, every
    /// record, count and float, that each entry's frames follow from its
    /// span and the rate, that the search tree places every frame once
    /// with its entry, and that every node's box, lowest frame and least
    /// bias are exactly its frames'.
    ///
    /// @param bytes       The file, 16-byte aligned.
    /// @param size        Its size in bytes; bytes past the file are
    ///                    ignored.
    /// @param databaseOut Receives the view on success.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// misaligned bytes; `mani_errorFormat` with the offset of the first
    /// byte found wrong; `mani_errorVersion` for any other format version.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniFileResult maniOpenMotionDatabase(const void* bytes, size_t size,
                                                                  maniMotionDatabase* databaseOut);

    /// Returns a database's identity: the hash of its sections.
    ///
    /// @param database  An opened database, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMotionDatabaseIdentity(const maniMotionDatabase* database);

    /// Returns the identity of the mirror map a database's mirrored
    /// entries play through.
    ///
    /// @param database  An opened database, or NULL.
    /// @return The map's identity; 0 for a database without mirrored
    /// entries, or NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMotionDatabaseMirrorMapIdentity(const maniMotionDatabase* database);

    /// Returns the identity of the skeleton a database was built for.
    ///
    /// @param database  An opened database, or NULL.
    /// @return The identity; 0 for NULL.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMotionDatabaseSkeletonIdentity(const maniMotionDatabase* database);

    /// Returns the identity of a database's clip.
    ///
    /// @param database  An opened database.
    /// @param clip      The clip's index.
    /// @return The identity; 0 for NULL or an index past the clips.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API uint64_t maniMotionDatabaseClipIdentity(const maniMotionDatabase* database,
                                                     uint32_t clip);

    /// Reads a database's channel.
    ///
    /// @param database    An opened database.
    /// @param channel     The channel's index.
    /// @param channelOut  Receives the channel.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// an index past the channels.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMotionDatabaseChannel(const maniMotionDatabase* database,
                                                                 uint32_t channel,
                                                                 maniMatchChannel* channelOut);

    /// Returns a float track channel's name.
    ///
    /// @param database   An opened database.
    /// @param channel    The channel's index.
    /// @param lengthOut  Receives the name's length in bytes, or NULL.
    /// @return The name, UTF-8, not terminated; NULL for a NULL database,
    /// an index past the channels or a channel of another kind.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_API const char* maniMotionDatabaseChannelName(const maniMotionDatabase* database,
                                                       uint32_t channel, uint32_t* lengthOut);

    /// Reads a database's entry.
    ///
    /// @param database  An opened database.
    /// @param entry     The entry's index.
    /// @param entryOut  Receives the entry.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// an index past the entries.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMotionDatabaseEntry(const maniMotionDatabase* database,
                                                               uint32_t entry,
                                                               maniMatchEntry* entryOut);

    /// Finds a frame's entry and its clip time.
    ///
    /// @param database  An opened database.
    /// @param frame     The frame.
    /// @param entryOut  Receives the entry's index.
    /// @param timeOut   Receives the clip time, seconds.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a frame past the frames.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMotionDatabaseFrame(const maniMotionDatabase* database,
                                                               uint32_t frame, uint32_t* entryOut,
                                                               float* timeOut);

    /// Reads a frame's features, normalized: (value - offset) / scale.
    ///
    /// @param database     An opened database.
    /// @param frame        The frame.
    /// @param featuresOut  Receives the database's feature count of
    ///                     values.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument or
    /// a frame past the frames.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMotionDatabaseFeatures(
        const maniMotionDatabase* database, uint32_t frame, float* featuresOut);

    /// Reads a database's normalization: per feature, the offset taken
    /// from a value and the scale it is divided by.
    ///
    /// @param database  An opened database.
    /// @param offsetsOut Receives the feature count of offsets, or NULL.
    /// @param scalesOut  Receives the feature count of scales, or NULL.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL database.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniMotionDatabaseNormalization(
        const maniMotionDatabase* database, float* offsetsOut, float* scalesOut);

    // A search: the query's features (raw, the database's feature count),
    // the continuing frame (MANI_MATCH_NO_FRAME for none), the tags an
    // entry must have all of and none of, and up to
    // MANI_MATCH_MAX_HISTORY earlier picks; the cost added to the
    // continuing frame and to any other, and three times in seconds (not
    // negative): frames of the continuing frame's entry nearer it than
    // jumpThreshold, frames of a clamped entry nearer its end than
    // endExclusion, and frames of an earlier pick's entry nearer it than
    // historyTime are not candidates (looping entries measured around
    // their span the shorter way).
    typedef struct maniMatchQuery
    {
        const float* features;
        uint32_t current;
        uint32_t requiredTags;
        uint32_t excludedTags;
        uint32_t historyCount;
        uint32_t history[MANI_MATCH_MAX_HISTORY];
        float continuingBias;
        float transitionCost;
        float jumpThreshold;
        float endExclusion;
        float historyTime;
        uint32_t reserved;
    } maniMatchQuery;

    // A search's result: the frame (MANI_MATCH_NO_FRAME when no frame is
    // a candidate) and its cost (+inf then).
    typedef struct maniMatchResult
    {
        uint32_t frame;
        float cost;
    } maniMatchResult;

    /// Finds the frame of least cost, exactly (a k-d tree of the frames
    /// whose nodes' bounding boxes prune without changing the result).
    /// A frame's cost is its entry's cost bias plus the
    /// transition cost (the continuing frame: the continuing bias), plus
    /// the squared differences of the normalized query and its features
    /// in feature order, in binary32. The continuing frame is a candidate
    /// when its entry's tags pass; another frame when its entry's tags
    /// pass, its entry is not MANI_MATCH_NO_JUMP, and no exclusion holds.
    /// Ties go to the continuing frame, then to the lowest frame.
    ///
    /// @param database  An opened database.
    /// @param query     The search.
    /// @param resultOut Receives the result.
    /// @return `mani_success`; `mani_errorInvalid` for a NULL argument, a
    /// feature or cost not finite, a negative time, a continuing frame or
    /// pick past the frames, or more than MANI_MATCH_MAX_HISTORY picks.
    /// @par Thread safety
    /// Safe from any thread.
    MANI_NODISCARD MANI_API maniResult maniSearchMotionDatabase(const maniMotionDatabase* database,
                                                                const maniMatchQuery* query,
                                                                maniMatchResult* resultOut);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MATCH_H
