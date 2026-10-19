// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Building mirror maps (mani-0033): a skeleton's bones paired by explicit
// pairs and by find/replace rules on their names, a reference pose, a
// plane, and the names of float tracks, events and markers paired the
// same way, laid out as each bone's partner and corrections so that
// mirroring at runtime only composes rotations. Offline.

#ifndef MAUL_ANIM_MIRRORBUILD_H
#define MAUL_ANIM_MIRRORBUILD_H

#include "maul-anim/base.h"
#include "maul-anim/build.h"
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
        uint32_t findLength;
        maniMirrorPlace place;
        const char* replace;
        uint32_t replaceLength;
        uint32_t reserved;
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
        uint32_t firstLength;
        uint32_t secondLength;
        const char* second;
    } maniMirrorNamePairSource;

    // A name the rules pair, if one matches.
    typedef struct maniMirrorNameSource
    {
        const char* name;
        uint32_t nameLength;
        uint32_t reserved;
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
        uint32_t reserved;
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
    /// `maniBuildReasonNone` for a NULL builder, source or skeleton, a
    /// count without its list, or reserved not 0;
    /// `maniBuildReasonOption` for an unknown axis;
    /// `maniBuildReasonMirrorReference` (item the bone);
    /// `maniBuildReasonMirrorRule` (item the rule);
    /// `maniBuildReasonMirrorPair` (item the bone, other its partner);
    /// `maniBuildReasonMirrorName` (item the name, other the earlier);
    /// `mani_errorCapacity` when memory runs out.
    /// @par Thread safety
    /// Safe from any thread; the builder is used by one thread at a time.
    MANI_NODISCARD MANI_API maniBuildResult maniBuildMirrorMap(maniBuilder* builder,
                                                               const maniMirrorMapSource* source);

#ifdef __cplusplus
}
#endif

#endif // MAUL_ANIM_MIRRORBUILD_H
