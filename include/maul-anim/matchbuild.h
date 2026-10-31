// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building motion databases: channels sampled from a
// skeleton's clips at a rate, normalized per channel as Holden's
// reference does, with a k-d tree of the frames for the search.

#ifndef MAUL_ANIM_MATCHBUILD_H
#define MAUL_ANIM_MATCHBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/match.h"
#include "maul-anim/mirror.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // A channel: its kind; its bone (bone kinds); its axis (a bone's
    // facing: the bone's axis, in the root's space); for a float track its
    // name, which every clip an entry names must have; its times, seconds
    // from the frame (ahead positive, behind negative, at most
    // MANI_MAX_MATCH_REACH either way); and its weight, above 0. Each
    // channel is normalized on its own: the mean per feature taken off,
    // divided by the mean of its features' deviations over the weight. A
    // channel whose features never vary is refused.
    typedef struct maniMatchChannelSource
    {
        maniMatchChannelKind kind;
        uint32_t bone;
        maniMatchAxis axis;
        uint32_t timeCount;
        const float* times;
        const char* track;
        size_t trackLength;
        float weight;
    } maniMatchChannelSource;

    // An entry: a clip's span from start to end seconds (0 <= start <= end
    // <= the clip's duration; start < end for a looping one), its flags
    // (maniMatchEntryLoop, maniMatchEntryNoJump, maniMatchEntryMirrored), a cost added to its
    // frames (finite) and its tags (any 32 bits, for a search's filter). Its frames are at start +
    // k / sampleRate (in binary64, rounded to binary32), every k whose time is at most end (before
    // it when the entry loops): counted in binary64, then one less or one more when the binary32
    // times of the last frame or the next say otherwise.
    typedef struct maniMatchEntrySource
    {
        uint32_t clip;
        maniMatchEntryFlags flags;
        float start;
        float end;
        float costBias;
        uint32_t tags;
    } maniMatchEntrySource;

    // A database: the skeleton and clips (each the skeleton's, not
    // additive), the channels, the entries, the sample rate (above 0, at
    // most MANI_MAX_MATCH_RATE), the up axis whose plane is the ground and
    // the forward axis a facing projects (on different axes), and the
    // mirror map of the skeleton its mirrored entries play through (NULL
    // when none is mirrored), and the most frames a leaf of the search's
    // tree holds: 8, 16 or 32, or 0 for 16. Smaller leaves search in
    // fewer instructions and take more bytes.
    typedef struct maniMotionDatabaseSource
    {
        const maniSkeleton* skeleton;
        const maniClip* const* clips;
        const maniMatchChannelSource* channels;
        const maniMatchEntrySource* entries;
        uint32_t clipCount;
        uint32_t channelCount;
        uint32_t entryCount;
        float sampleRate;
        maniMatchAxis up;
        maniMatchAxis forward;
        const maniMirrorMap* mirrorMap;
        uint32_t leafFrames;
    } maniMotionDatabaseSource;

    /// Builds a motion database: samples every entry's frames, each
    /// channel at each of its times (clamped to the entry's ends, or
    /// wrapped when it loops), in the root's space at the frame (the
    /// clip's root motion carrying the root between times; a bone's
    /// velocity the difference of its positions a frame either side over
    /// their time apart), normalizes each channel, and orders the frames
    /// in a k-d tree (each node's frames split at the median of the
    /// feature they spread widest over) with each node's bounding box. A
    /// mirrored entry's frames are sampled from its
    /// clip's poses mirrored, its root moves reflected and each float
    /// track channel from the track the map pairs its name with, as a
    /// mirrored clip's would be. Keeps the file in place of any built
    /// before.
    ///
    /// @param builder  The builder.
    /// @param source   The database.
    /// @return `mani_success`; `mani_errorLimit` with the reason and item
    /// for a source past a bound (no misuse): `maniBuildReasonTiming` (a
    /// rate past MANI_MAX_MATCH_RATE), `maniBuildReasonCount` (more than
    /// MANI_MAX_MATCH_CLIPS clips, MANI_MAX_MATCH_CHANNELS channels or
    /// MANI_MAX_MATCH_ENTRIES entries, more than MANI_MAX_MATCH_FRAMES
    /// frames or MANI_MAX_MATCH_FEATURES features),
    /// `maniBuildReasonChannel` (more than MANI_MAX_MATCH_TIMES times, a
    /// time past MANI_MAX_MATCH_REACH or a track name past
    /// MANI_MAX_NAME_BYTES; item the channel);
    /// `mani_errorInvalid` for a NULL argument (a misuse), or with the
    /// reason and item for a refused source: `maniBuildReasonTiming` (a
    /// rate not above 0), `maniBuildReasonOption` (the axes or the leaf
    /// size), `maniBuildReasonCount` (no channels, clips or entries, or
    /// the channels' track names past 4 GiB in all),
    /// `maniBuildReasonJoint` (a clip
    /// of another skeleton or additive; item the clip),
    /// `maniBuildReasonChannel` (item the channel), `maniBuildReasonEntry`
    /// (item the entry), `maniBuildReasonMissingTrack` (a float track an
    /// entry's clip lacks, for a mirrored entry the track the mirror map
    /// pairs the channel's with; item the channel, other the entry),
    /// `maniBuildReasonSpread` (a channel that never varies; item the
    /// channel), `maniBuildReasonMirrorDatabase` (a mirror map of another
    /// skeleton, item 0; a mirrored entry without one, item the entry);
    /// `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult
    maniBuildMotionDatabase(maniBuilder* builder, const maniMotionDatabaseSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MATCHBUILD_H
