// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building mirror maps: a skeleton's bones paired by explicit
// pairs and by find/replace rules on their names, a reference pose, a
// plane, and the names of float tracks, events and markers paired the
// same way, laid out as each bone's partner and corrections so that
// mirroring at runtime only composes rotations. Offline.

#ifndef MAUL_ANIM_MIRRORBUILD_H
#define MAUL_ANIM_MIRRORBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/mirror.h"
#include "maul-anim/skeleton.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // Where a rule's find must lie in a name.
    typedef uint32_t maniMirrorPlace;
    enum
    {
        maniMirrorPrefix = 0,
        maniMirrorSuffix = 1,
        maniMirrorAnywhere = 2,
    };

    // A find/replace rule: a name holding find at its place (anywhere:
    // its first occurrence) has its partner's name with find replaced.
    // Rules run one way: give both directions.
    typedef struct maniMirrorRuleSource
    {
        const char* find;
        size_t findLength;
        maniMirrorPlace place;
        const char* replace;
        size_t replaceLength;
    } maniMirrorRuleSource;

    // Two bones that mirror each other (equal for a bone on itself).
    typedef struct maniMirrorPairSource
    {
        uint32_t first;
        uint32_t second;
    } maniMirrorPairSource;

    // Two names (float tracks, events, markers) that mirror each other.
    typedef struct maniMirrorNamePairSource
    {
        const char* first;
        size_t firstLength;
        size_t secondLength;
        const char* second;
    } maniMirrorNamePairSource;

    // A name the rules pair, if one matches.
    typedef struct maniMirrorNameSource
    {
        const char* name;
        size_t nameLength;
    } maniMirrorNameSource;

    // A mirror map: the skeleton; its reference pose as locals in bone
    // order (NULL for its rest); the plane's normal axis; bone pairs
    // first, then the rules for the bones they leave (the first rule
    // that matches and names a bone; none leaves a bone to itself); name
    // pairs, then names the rules pair.
    typedef struct maniMirrorMapSource
    {
        const maniSkeleton* skeleton;
        const maniTransform* reference;
        const maniMirrorPairSource* pairs;
        const maniMirrorRuleSource* rules;
        const maniMirrorNamePairSource* namePairs;
        const maniMirrorNameSource* names;
        uint32_t pairCount;
        uint32_t ruleCount;
        uint32_t namePairCount;
        uint32_t nameCount;
        maniMirrorAxis axis;
    } maniMirrorMapSource;

    /// Builds a mirror map file (kind 13) from a source and keeps it in
    /// the builder. A rule's partner must pair back by the rules (or be
    /// left to the rules by the pairs). The map records the first bone
    /// whose parent's partner is not its partner's parent, without
    /// refusing it.
    ///
    /// @param builder  The builder.
    /// @param source   The map.
    /// @return `mani_success`; `mani_errorInvalid` with
    /// `maniBuildReasonNone` for a NULL builder, source or skeleton, or a
    /// count without its list;
    /// `maniBuildReasonOption` for an unknown axis;
    /// `maniBuildReasonMirrorReference` (item the bone);
    /// `maniBuildReasonMirrorRule` (item the rule);
    /// `maniBuildReasonMirrorPair` (item the bone, other its partner);
    /// `maniBuildReasonMirrorName` (item the name, other the earlier);
    /// `maniBuildReasonNameHash` for two names of one hash (item the later
    /// name, other the earlier);
    /// `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildMirrorMap(maniBuilder* builder,
                                                               const maniMirrorMapSource* source);

    /// Builds a mirror map as maniBuildMirrorMap does and, when it builds,
    /// keeps its JSON text form in the builder in place of the file:
    /// format "maul-anim mirror map", version 1; the axis ("x", "y",
    /// "z"); the reference pose as the bones whose local is not the rest
    /// bit for bit, by name, with translation, rotation and scale; bone
    /// pairs by name; rules (find, place of "prefix", "suffix" or
    /// "anywhere", replace); name pairs and names. Numbers as the
    /// shortest text that reads back to the same float.
    ///
    /// @param builder  The builder.
    /// @param source   The map.
    /// @return As maniBuildMirrorMap.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult
    maniWriteMirrorMapJson(maniBuilder* builder, const maniMirrorMapSource* source);

    /// Builds a mirror map file from its JSON text form, as
    /// maniWriteMirrorMapJson writes it: members an object does not take
    /// are refused; the reference, pairs, rules, name pairs and names
    /// may be left out (empty), as may a reference entry's translation,
    /// rotation or scale (its bone's rest). A bone the reference lists
    /// twice is refused. The source read is then built as
    /// maniBuildMirrorMap builds it.
    ///
    /// @param builder   The builder.
    /// @param text      The UTF-8 text (a leading byte order mark
    ///                  skipped); may be NULL when length is 0.
    /// @param length    The text's length in bytes.
    /// @param skeleton  The skeleton.
    /// @return As maniBuildMirrorMap, items by place in the text's lists;
    /// or `mani_errorInvalid` with maniBuildReasonNone for a NULL builder
    /// or skeleton or a NULL text with a length, maniBuildReasonJson or
    /// maniBuildReasonForm (other a maniGraphForm), item the byte offset.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildMirrorMapJson(maniBuilder* builder,
                                                                   const char* text, size_t length,
                                                                   const maniSkeleton* skeleton);

    // A clip to mirror.
    typedef struct maniMirrorClipSource
    {
        const maniMirrorMap* map;
        const maniSkeleton* skeleton;
        const maniClip* clip;
    } maniMirrorClipSource;

    /// Builds a full-precision clip of a clip mirrored (bake it once,
    /// play it as any clip), and keeps it in the builder. The clip keeps
    /// the clip's sample rate, duration, loop policy and additive basis.
    /// At each sample the pose is mirrored by maniMirrorPose; a channel
    /// that is the rest at every sample is left out. An additive clip's
    /// differences are each added to the rest, mirrored, and taken as
    /// differences from the rest mirrored. Float tracks keep their values
    /// at the samples and their interpolation; discrete tracks, events
    /// and markers are copied; every name is renamed by maniMirrorName.
    /// Root motion moves to the root motion bone's partner: each stored
    /// move (in the bone's parent's frame) reflected as
    /// maniMirrorRootMotion reflects it and turned into the partner's
    /// parent's frame mirrored, its source's axes following that turn
    /// (all axes, and the whole rotation, where it takes an axis off the
    /// axes).
    ///
    /// @param builder  The builder.
    /// @param source   The clip to mirror.
    /// @return `mani_success`; `mani_errorInvalid` with
    /// `maniBuildReasonNone` for a NULL builder, source or field;
    /// `maniBuildReasonMirrorClip` (other 0 the skeleton,
    /// 1 the clip: a skeleton not the map's, a clip not of the map's
    /// skeleton); `maniBuildReasonBasis` for an additive clip whose
    /// difference from its basis mirrored is not finite;
    /// `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult
    maniBuildMirroredClip(maniBuilder* builder, const maniMirrorClipSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MIRRORBUILD_H
