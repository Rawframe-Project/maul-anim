// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The JSON text form of retargeting maps (mani-0034): a map with a
// turned reference, pairs, chains of every mode, the pelvis and names
// written as the pinned text and read back to the same file; the rest
// given in full written as no reference; members left out; every
// refusal of the text at its offset, then the build's by place; the
// allocator failing at each allocation. Run as test_retarget_json --seed
// <file> it writes the pinned text, the seed of the fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "map_json_assets.h"
#include "map_json_harness.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFiles[2][4096];
static _Alignas(16) uint8_t s_built[16384];
static _Alignas(16) uint8_t s_read[16384];
static char s_text[16384];
static maniSkeleton s_source;
static maniSkeleton s_target;

static const maniRetargetPairSource kHeadPair = {9, 7, 0.5f, maniRetargetTranslationAbsolute, true};
static const maniRetargetChainSource kChains[2] = {
    {2, 4, 2, 5, maniRetargetRotationInterpolated, 1.0f, maniRetargetTranslationOwn, false},
    {5, 6, 6, 8, maniRetargetRotationOneToOneReversed, 0.25f, maniRetargetTranslationScaled, true}};
static const maniRetargetPelvisSource kPelvis = {
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 1.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 1.0f, 0.75f};

static maniTransform s_sourceReference[8];
static maniTransform s_targetReference[10];

// The source's reference: the arm turned a quarter about z, the hand
// stretched and its z a negative zero; the target's: its rest given in
// full.
static void References(void)
{
    for (uint32_t b = 0; b < 8; ++b)
    {
        s_sourceReference[b] = maniSkeletonRest(&s_source, b);
    }
    s_sourceReference[5].rotation = (maniQuat){0.0f, 0.0f, 0.70710678f, 0.70710678f};
    s_sourceReference[6].scale = (maniVec3){1.0f, 1.5f, 1.0f};
    s_sourceReference[6].translation.z = -0.0f;
    for (uint32_t b = 0; b < 10; ++b)
    {
        s_targetReference[b] = maniSkeletonRest(&s_target, b);
    }
}

static maniRetargetMapSource Map(void)
{
    return (maniRetargetMapSource){.source = &s_source,
                                   .target = &s_target,
                                   .sourceReference = s_sourceReference,
                                   .targetReference = s_targetReference,
                                   .pairs = &kHeadPair,
                                   .chains = kChains,
                                   .pelvis = &kPelvis,
                                   .pairCount = 1,
                                   .chainCount = 2,
                                   .matchNames = true};
}

static const char kExpected[] =
    "{\n"
    "  \"format\": \"maul-anim retarget map\",\n"
    "  \"version\": 1,\n"
    "  \"sourceReference\": [\n"
    "    {\"bone\": \"armL\", \"translation\": [0.2, 0.1, 0], \"rotation\": [0, 0, 0.70710677, "
    "0.70710677], \"scale\": [1, 1, 1]},\n"
    "    {\"bone\": \"handL\", \"translation\": [0.3, 0, -0], \"rotation\": [0, 0, 0, 1], "
    "\"scale\": [1, 1.5, 1]}\n"
    "  ],\n"
    "  \"targetReference\": [],\n"
    "  \"pairs\": [\n"
    "    {\"target\": \"head\", \"source\": \"head\", \"rotationWeight\": 0.5, "
    "\"translation\": \"absolute\", \"scale\": true}\n"
    "  ],\n"
    "  \"chains\": [\n"
    "    {\"sourceStart\": \"s1\", \"sourceEnd\": \"chest\", \"targetStart\": \"sp1\", "
    "\"targetEnd\": \"chest\", \"rotation\": \"interpolated\", \"rotationWeight\": 1, "
    "\"translation\": \"own\", \"scale\": false},\n"
    "    {\"sourceStart\": \"armL\", \"sourceEnd\": \"handL\", \"targetStart\": \"upperarm\", "
    "\"targetEnd\": \"hand\", \"rotation\": \"oneToOneReversed\", \"rotationWeight\": 0.25, "
    "\"translation\": \"scaled\", \"scale\": true}\n"
    "  ],\n"
    "  \"pelvis\": {\"source\": \"hips\", \"target\": \"pelvis\", \"up\": [0, 1, 0], "
    "\"horizontal\": 1, \"vertical\": 0.5, \"absolute\": [0, 1, 0], \"toSource\": [0.25, 0, "
    "0], \"translationWeight\": 1, \"rotationWeight\": 0.75},\n"
    "  \"matchNames\": true\n"
    "}\n";

// A source written; the text's size, 0 when refused.
static size_t Write(const maniRetargetMapSource* source, maniBuildResult* resultOut)
{
    maniBuilder* builder = NewBuilder(NULL);
    *resultOut = maniWriteRetargetMapJson(builder, source);
    size_t size = 0;
    if (resultOut->result == mani_success)
    {
        size = CopyText(builder, s_text, sizeof s_text);
    }
    maniDestroyBuilder(builder);
    return size;
}

// A source built into s_built; its size.
static size_t Built(const maniRetargetMapSource* source)
{
    maniBuilder* builder = NewBuilder(NULL);
    size_t size = 0;
    CHECK(maniBuildRetargetMap(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &size) == mani_success,
          "built");
    maniDestroyBuilder(builder);
    return size;
}

// A text read into s_read; its result, the size in *sizeOut.
static maniBuildResult ReadInto(const char* text, size_t* sizeOut)
{
    maniBuilder* builder = NewBuilder(NULL);
    maniBuildResult r = maniBuildRetargetMapJson(builder, text, strlen(text), &s_source, &s_target);
    *sizeOut = 0;
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_read, sizeof s_read, sizeOut) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return r;
}

static maniBuildResult Read(const char* text)
{
    size_t size = 0;
    return ReadInto(text, &size);
}

// Whether a text reads back to the bytes a source builds.
static bool ReadsAs(const char* text, const maniRetargetMapSource* source)
{
    size_t want = Built(source);
    size_t size = 0;
    maniBuildResult r = ReadInto(text, &size);
    return r.result == mani_success && size == want && memcmp(s_read, s_built, size) == 0;
}

static void TestWrite(void)
{
    maniRetargetMapSource source = Map();
    maniBuildResult r;
    size_t size = Write(&source, &r);
    bool same = size == sizeof kExpected - 1 && memcmp(s_text, kExpected, size) == 0;
    CHECK(r.result == mani_success && same, "the pinned text");
    if (!same)
    {
        printf("%.*s", (int)size, s_text);
    }
    CHECK(ReadsAs(kExpected, &source), "read back bit for bit");
}

// No reference, pair, chain or pelvis: each list on one line, the
// pelvis null; the rest given in full written as none.
static const char kMinimal[] = "{\n"
                               "  \"format\": \"maul-anim retarget map\",\n"
                               "  \"version\": 1,\n"
                               "  \"sourceReference\": [],\n"
                               "  \"targetReference\": [],\n"
                               "  \"pairs\": [],\n"
                               "  \"chains\": [],\n"
                               "  \"pelvis\": null,\n"
                               "  \"matchNames\": true\n"
                               "}\n";

static void TestMinimal(void)
{
    maniRetargetMapSource source = {.source = &s_source, .target = &s_target, .matchNames = true};
    maniBuildResult r;
    size_t size = Write(&source, &r);
    CHECK(r.result == mani_success && size == sizeof kMinimal - 1 &&
              memcmp(s_text, kMinimal, size) == 0,
          "empty lists on one line");
    source.targetReference = s_targetReference;
    size = Write(&source, &r);
    CHECK(r.result == mani_success && size == sizeof kMinimal - 1 &&
              memcmp(s_text, kMinimal, size) == 0,
          "the rest written as no reference");
    CHECK(ReadsAs(kMinimal, &source), "and read back");
}

// Members left out take their off values: the lists, the pelvis,
// matchNames, a reference entry's members, a pair's or chain's
// translation and scale, the pelvis's blends.
static void TestDefaults(void)
{
    static const char kShort[] =
        "\xEF\xBB\xBF{\"format\": \"maul-anim retarget map\", \"version\": 1, "
        "\"sourceReference\": [{\"bone\": \"armL\", \"rotation\": [0, 0, 0.70710678, "
        "0.70710678]}, {\"bone\": \"handL\"}], \"pairs\": [{\"target\": \"head\", \"source\": "
        "\"head\", \"rotationWeight\": 1}], \"chains\": [{\"sourceStart\": \"s1\", "
        "\"sourceEnd\": \"chest\", \"targetStart\": \"sp1\", \"targetEnd\": \"chest\", "
        "\"rotation\": \"oneToOne\", \"rotationWeight\": 1}], \"pelvis\": {\"source\": \"hips\", "
        "\"target\": \"pelvis\", \"up\": [0, 1, 0], \"horizontal\": 1, \"vertical\": 1, "
        "\"translationWeight\": 1, \"rotationWeight\": 1}}";
    static const maniRetargetPairSource kPair = {9, 7, 1.0f, maniRetargetTranslationOwn, false};
    static const maniRetargetChainSource kChain = {
        2, 4, 2, 5, maniRetargetRotationOneToOne, 1.0f, maniRetargetTranslationOwn, false};
    static const maniRetargetPelvisSource kPlain = {
        1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 1.0f, 1.0f};
    maniTransform reference[8];
    for (uint32_t b = 0; b < 8; ++b)
    {
        reference[b] = maniSkeletonRest(&s_source, b);
    }
    reference[5].rotation = (maniQuat){0.0f, 0.0f, 0.70710678f, 0.70710678f};
    maniRetargetMapSource source = {.source = &s_source,
                                    .target = &s_target,
                                    .sourceReference = reference,
                                    .pairs = &kPair,
                                    .chains = &kChain,
                                    .pelvis = &kPlain,
                                    .pairCount = 1,
                                    .chainCount = 1};
    CHECK(ReadsAs(kShort, &source), "members left out");
    CHECK(ReadsAs("{\"format\": \"maul-anim retarget map\", \"version\": 1, \"pelvis\": null}",
                  &(maniRetargetMapSource){.source = &s_source, .target = &s_target}),
          "every list left out, the pelvis null");
}

#define TOP        "{\"format\": \"maul-anim retarget map\", \"version\": 1, "
#define PAIR(rest) "\"pairs\": [{\"target\": \"head\", \"source\": \"head\", " rest "}]}"

static const Refusal kRefusals[] = {
    {TOP "\"pairs\": [x]}", maniBuildReasonJson, 1, "x]", 1},
    {TOP "\"bogus\": 1}", maniBuildReasonForm, maniGraphFormMember, "\"bogus", 1},
    {TOP "\"pairs\": [], \"pairs\": []}", maniBuildReasonForm, maniGraphFormRepeat, "\"pairs", 2},
    {"{\"format\": \"maul-anim retarget map\"}", maniBuildReasonForm, maniGraphFormMissing, "{", 1},
    {"{\"format\": \"maul-anim graph\", \"version\": 1}", maniBuildReasonForm, maniGraphFormVersion,
     "\"maul-anim graph", 1},
    {"{\"format\": \"maul-anim retarget map\", \"version\": 2}", maniBuildReasonForm,
     maniGraphFormVersion, "2}", 1},
    {TOP "\"sourceReference\": {}}", maniBuildReasonForm, maniGraphFormValue, "{}", 1},
    {TOP "\"sourceReference\": [{\"bone\": \"armL\"}, {\"bone\": \"armL\"}]}", maniBuildReasonForm,
     maniGraphFormRepeat, "\"armL", 2},
    {TOP "\"sourceReference\": [{\"bone\": \"upperarm\"}]}", maniBuildReasonForm, maniGraphFormName,
     "\"upperarm", 1},
    {TOP "\"targetReference\": [{\"bone\": \"armL\"}]}", maniBuildReasonForm, maniGraphFormName,
     "\"armL", 1},
    {TOP "\"targetReference\": [{\"bone\": \"hand\", \"rotation\": [0, 0, 1]}]}",
     maniBuildReasonForm, maniGraphFormValue, "[0, 0, 1]", 1},
    {TOP "\"targetReference\": [{\"translation\": [0, 0, 0]}]}", maniBuildReasonForm,
     maniGraphFormMissing, "{\"translation", 1},
    {TOP PAIR("\"rotationWeight\": 1, \"translation\": \"moved\""), maniBuildReasonForm,
     maniGraphFormValue, "\"moved", 1},
    {TOP PAIR("\"rotationWeight\": true"), maniBuildReasonForm, maniGraphFormValue, "true", 1},
    {TOP PAIR("\"translation\": \"own\""), maniBuildReasonForm, maniGraphFormMissing, "{\"target",
     1},
    {TOP PAIR("\"rotationWeight\": 1, \"scale\": 1"), maniBuildReasonForm, maniGraphFormValue, "1}",
     1},
    {TOP "\"pairs\": [{\"target\": \"hips\", \"source\": \"head\", \"rotationWeight\": 1}]}",
     maniBuildReasonForm, maniGraphFormName, "\"hips", 1},
    {TOP "\"chains\": [{\"sourceStart\": \"s1\", \"sourceEnd\": \"chest\", \"targetStart\": "
         "\"sp1\", \"targetEnd\": \"chest\", \"rotation\": \"some\", \"rotationWeight\": 1}]}",
     maniBuildReasonForm, maniGraphFormValue, "\"some", 1},
    {TOP "\"chains\": [{\"sourceStart\": \"s1\", \"sourceEnd\": \"chest\", \"targetStart\": "
         "\"sp1\", \"targetEnd\": \"chest\", \"rotation\": \"none\"}]}",
     maniBuildReasonForm, maniGraphFormMissing, "{\"sourceStart", 1},
    {TOP "\"pelvis\": {\"source\": \"hips\", \"target\": \"pelvis\", \"up\": [0, 1], "
         "\"horizontal\": 1, \"vertical\": 1, \"translationWeight\": 1, \"rotationWeight\": 1}}",
     maniBuildReasonForm, maniGraphFormValue, "[0, 1]", 1},
    {TOP "\"pelvis\": {\"source\": \"hips\", \"target\": \"pelvis\", \"up\": [0, 1, 0], "
         "\"horizontal\": 1, \"translationWeight\": 1, \"rotationWeight\": 1}}",
     maniBuildReasonForm, maniGraphFormMissing, "{\"source", 1},
    {TOP "\"pelvis\": {\"source\": \"pelvis\", \"target\": \"pelvis\", \"up\": [0, 1, 0], "
         "\"horizontal\": 1, \"vertical\": 1, \"translationWeight\": 1, \"rotationWeight\": 1}}",
     maniBuildReasonForm, maniGraphFormName, "\"pelvis\", \"target", 1},
    {TOP "\"pelvis\": 1}", maniBuildReasonForm, maniGraphFormValue, "1}", 1},
    {TOP "\"matchNames\": 0}", maniBuildReasonForm, maniGraphFormValue, "0}", 1},
    // Read, then refused by the build: the second pair maps the head
    // again; a reference rotation not of length 1.
    {TOP "\"pairs\": [{\"target\": \"chest\", \"source\": \"chest\", \"rotationWeight\": 1}, "
         "{\"target\": \"head\", \"source\": \"head\", \"rotationWeight\": 2}]}",
     maniBuildReasonRetargetPair, 1, NULL, 0},
    {TOP "\"targetReference\": [{\"bone\": \"hand\", \"rotation\": [0, 0, 0, 2]}]}",
     maniBuildReasonRetargetReference, 8, NULL, 0},
};

static void TestReadRefusals(void)
{
    CheckRefusals(Read, kRefusals, sizeof kRefusals / sizeof kRefusals[0]);
    maniBuilder* builder = NewBuilder(NULL);
    CHECK(maniBuildRetargetMapJson(NULL, "", 0, &s_source, &s_target).result == mani_errorInvalid,
          "no builder");
    CHECK(
        maniBuildRetargetMapJson(builder, NULL, 1, &s_source, &s_target).result ==
                mani_errorInvalid &&
            maniBuildRetargetMapJson(builder, "", 0, NULL, &s_target).result == mani_errorInvalid &&
            maniBuildRetargetMapJson(builder, "", 0, &s_source, NULL).result == mani_errorInvalid &&
            maniBuilderMisuses(builder) == 3,
        "misuses");
    maniDestroyBuilder(builder);
}

static void TestWriteRefusals(void)
{
    maniRetargetMapSource source = Map();
    maniRetargetChainSource chain = kChains[1];
    chain.rotationWeight = 2.0f;
    source.chains = &chain;
    source.chainCount = 1;
    maniBuildResult r;
    CHECK(Write(&source, &r) == 0 && r.result == mani_errorInvalid &&
              r.reason == maniBuildReasonRetargetChain && r.item == 0,
          "refused as the build refuses");
    source = Map();
    CHECK(maniWriteRetargetMapJson(NULL, &source).result == mani_errorInvalid, "no builder");
    maniBuilder* builder = NewBuilder(NULL);
    CHECK(maniWriteRetargetMapJson(builder, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == 1,
          "no source, a misuse");
    maniDestroyBuilder(builder);
}

static maniBuildResult Call(maniBuilder* builder, bool reading)
{
    maniRetargetMapSource source = Map();
    return reading ? maniBuildRetargetMapJson(builder, kExpected, sizeof kExpected - 1, &s_source,
                                              &s_target)
                   : maniWriteRetargetMapJson(builder, &source);
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2], kExpected, sizeof kExpected - 1);
    }
    CHECK(OpenRig(kSourceRig, 8, s_skeletonFiles[0], &s_source) &&
              OpenRig(kTargetRig, 10, s_skeletonFiles[1], &s_target),
          "the rigs");
    References();
    TestWrite();
    TestMinimal();
    TestDefaults();
    TestReadRefusals();
    TestWriteRefusals();
    TestAllocationFailure(Call);
    return s_failures == 0 ? 0 : 1;
}
