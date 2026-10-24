// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The JSON text form of mirror maps: a map with a moved
// reference, bone pairs, rules of every place, name pairs and names
// written as the pinned text and read back to the same file; empty lists
// on one line; members left out; every refusal of the text at its
// offset, then the build's by place; the allocator failing at each
// allocation. Run as test_mirror_json --seed <file> it writes the pinned
// text, the seed of the fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "map_json_assets.h"
#include "map_json_harness.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_built[16384];
static _Alignas(16) uint8_t s_read[16384];
static char s_text[16384];
static maniSkeleton s_skeleton;

static const maniMirrorPairSource kPairs[1] = {{kPropR, kPropR}};
static const maniMirrorRuleSource kRules[5] = {{"L", 1, maniMirrorSuffix, "R", 1, 0},
                                               {"R", 1, maniMirrorSuffix, "L", 1, 0},
                                               {"_left", 5, maniMirrorAnywhere, "_right", 6, 0},
                                               {"_right", 6, maniMirrorAnywhere, "_left", 5, 0},
                                               {"Left\"", 5, maniMirrorPrefix, "Right\"", 6, 0}};
static const maniMirrorNamePairSource kNamePairs[1] = {{"footL", 5, 5, "footR"}};
static const maniMirrorNameSource kNames[2] = {{"curveL", 6, 0}, {"idle", 4, 0}};
static maniTransform s_reference[kBones];

static maniMirrorMapSource Map(void)
{
    for (uint32_t b = 0; b < kBones; ++b)
    {
        s_reference[b] = maniSkeletonRest(&s_skeleton, b);
    }
    s_reference[kHead].translation.y = 0.65f;
    s_reference[kArmR].rotation = (maniQuat){0.0f, 0.0f, 0.0f, -1.0f};
    return (maniMirrorMapSource){.skeleton = &s_skeleton,
                                 .reference = s_reference,
                                 .pairs = kPairs,
                                 .rules = kRules,
                                 .namePairs = kNamePairs,
                                 .names = kNames,
                                 .pairCount = 1,
                                 .ruleCount = 5,
                                 .namePairCount = 1,
                                 .nameCount = 2,
                                 .axis = maniMirrorAxisZ};
}

static const char kExpected[] =
    "{\n"
    "  \"format\": \"maul-anim mirror map\",\n"
    "  \"version\": 1,\n"
    "  \"axis\": \"z\",\n"
    "  \"reference\": [\n"
    "    {\"bone\": \"head\", \"translation\": [0, 0.65, 0], \"rotation\": [0, 0, 0, 1], "
    "\"scale\": [1, 1, 1]},\n"
    "    {\"bone\": \"armR\", \"translation\": [-0.2, 0.5, 0], \"rotation\": [0, 0, 0, -1], "
    "\"scale\": [1, 1, 1]}\n"
    "  ],\n"
    "  \"pairs\": [\n"
    "    {\"first\": \"propR\", \"second\": \"propR\"}\n"
    "  ],\n"
    "  \"rules\": [\n"
    "    {\"find\": \"L\", \"place\": \"suffix\", \"replace\": \"R\"},\n"
    "    {\"find\": \"R\", \"place\": \"suffix\", \"replace\": \"L\"},\n"
    "    {\"find\": \"_left\", \"place\": \"anywhere\", \"replace\": \"_right\"},\n"
    "    {\"find\": \"_right\", \"place\": \"anywhere\", \"replace\": \"_left\"},\n"
    "    {\"find\": \"Left\\\"\", \"place\": \"prefix\", \"replace\": \"Right\\\"\"}\n"
    "  ],\n"
    "  \"namePairs\": [\n"
    "    {\"first\": \"footL\", \"second\": \"footR\"}\n"
    "  ],\n"
    "  \"names\": [\n"
    "    \"curveL\",\n"
    "    \"idle\"\n"
    "  ]\n"
    "}\n";

static size_t Write(const maniMirrorMapSource* source, maniBuildResult* resultOut)
{
    maniBuilder* builder = NewBuilder(NULL);
    *resultOut = maniWriteMirrorMapJson(builder, source);
    size_t size = 0;
    if (resultOut->result == mani_success)
    {
        size = CopyText(builder, s_text, sizeof s_text);
    }
    maniDestroyBuilder(builder);
    return size;
}

static size_t Built(const maniMirrorMapSource* source)
{
    maniBuilder* builder = NewBuilder(NULL);
    size_t size = 0;
    CHECK(maniBuildMirrorMap(builder, source).result == mani_success &&
              maniCopyBuilt(builder, s_built, sizeof s_built, &size) == mani_success,
          "built");
    maniDestroyBuilder(builder);
    return size;
}

static maniBuildResult ReadInto(const char* text, size_t* sizeOut)
{
    maniBuilder* builder = NewBuilder(NULL);
    maniBuildResult r = maniBuildMirrorMapJson(builder, text, strlen(text), &s_skeleton);
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

static bool ReadsAs(const char* text, const maniMirrorMapSource* source)
{
    size_t want = Built(source);
    size_t size = 0;
    maniBuildResult r = ReadInto(text, &size);
    return r.result == mani_success && size == want && memcmp(s_read, s_built, size) == 0;
}

static void TestWrite(void)
{
    maniMirrorMapSource source = Map();
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

static const char kMinimal[] = "{\n"
                               "  \"format\": \"maul-anim mirror map\",\n"
                               "  \"version\": 1,\n"
                               "  \"axis\": \"x\",\n"
                               "  \"reference\": [],\n"
                               "  \"pairs\": [],\n"
                               "  \"rules\": [],\n"
                               "  \"namePairs\": [],\n"
                               "  \"names\": []\n"
                               "}\n";

// Every list empty on one line, the rest given in full written as none;
// every list left out read as empty; a reference entry's members left
// out its bone's rest.
static void TestMinimal(void)
{
    maniMirrorMapSource source = {.skeleton = &s_skeleton, .axis = maniMirrorAxisX};
    maniBuildResult r;
    size_t size = Write(&source, &r);
    CHECK(r.result == mani_success && size == sizeof kMinimal - 1 &&
              memcmp(s_text, kMinimal, size) == 0,
          "empty lists on one line");
    (void)Map();
    s_reference[kHead] = maniSkeletonRest(&s_skeleton, kHead);
    s_reference[kArmR] = maniSkeletonRest(&s_skeleton, kArmR);
    source.reference = s_reference;
    size = Write(&source, &r);
    CHECK(r.result == mani_success && size == sizeof kMinimal - 1 &&
              memcmp(s_text, kMinimal, size) == 0,
          "the rest written as no reference");
    CHECK(ReadsAs("{\"version\": 1, \"axis\": \"x\", \"format\": \"maul-anim mirror map\", "
                  "\"reference\": [{\"bone\": \"head\"}]}",
                  &source),
          "lists and members left out");
}

#define TOP "{\"format\": \"maul-anim mirror map\", \"version\": 1, \"axis\": \"y\", "

static const Refusal kRefusals[] = {
    {TOP "\"names\": [x]}", maniBuildReasonJson, 1, "x]", 1},
    {TOP "\"bogus\": 1}", maniBuildReasonForm, maniGraphFormMember, "\"bogus", 1},
    {TOP "\"axis\": \"y\"}", maniBuildReasonForm, maniGraphFormRepeat, "\"axis", 2},
    {"{\"format\": \"maul-anim mirror map\", \"version\": 1}", maniBuildReasonForm,
     maniGraphFormMissing, "{", 1},
    {"{\"format\": \"maul-anim retarget map\", \"version\": 1, \"axis\": \"x\"}",
     maniBuildReasonForm, maniGraphFormVersion, "\"maul-anim retarget map", 1},
    {"{\"format\": \"maul-anim mirror map\", \"version\": 1.5, \"axis\": \"x\"}",
     maniBuildReasonForm, maniGraphFormVersion, "1.5", 1},
    {"{\"format\": \"maul-anim mirror map\", \"version\": 1, \"axis\": \"w\"}", maniBuildReasonForm,
     maniGraphFormValue, "\"w", 1},
    {TOP "\"reference\": [{\"bone\": \"head\"}, {\"bone\": \"hips\"}, {\"bone\": \"head\"}]}",
     maniBuildReasonForm, maniGraphFormRepeat, "\"head", 2},
    {TOP "\"reference\": [{\"bone\": \"neck\"}]}", maniBuildReasonForm, maniGraphFormName, "\"neck",
     1},
    {TOP "\"reference\": [{\"bone\": \"head\", \"scale\": [1, 1, \"1\"]}]}", maniBuildReasonForm,
     maniGraphFormValue, "\"1\"", 1},
    {TOP "\"reference\": [{\"bone\": \"head\", \"pose\": 1}]}", maniBuildReasonForm,
     maniGraphFormMember, "\"pose", 1},
    {TOP "\"pairs\": [{\"first\": \"propR\"}]}", maniBuildReasonForm, maniGraphFormMissing,
     "{\"first", 1},
    {TOP "\"pairs\": [{\"first\": \"propR\", \"second\": \"propL\"}]}", maniBuildReasonForm,
     maniGraphFormName, "\"propL", 1},
    {TOP "\"rules\": [{\"find\": \"L\", \"place\": \"middle\", \"replace\": \"R\"}]}",
     maniBuildReasonForm, maniGraphFormValue, "\"middle", 1},
    {TOP "\"rules\": [{\"find\": \"L\", \"place\": \"suffix\"}]}", maniBuildReasonForm,
     maniGraphFormMissing, "{\"find", 1},
    {TOP "\"rules\": [{\"find\": 1, \"place\": \"suffix\", \"replace\": \"R\"}]}",
     maniBuildReasonForm, maniGraphFormValue, "1,", 2},
    {TOP "\"namePairs\": [{\"first\": \"footL\", \"second\": null}]}", maniBuildReasonForm,
     maniGraphFormValue, "null", 1},
    {TOP "\"names\": [\"curveL\", 2]}", maniBuildReasonForm, maniGraphFormValue, "2]", 1},
    {TOP "\"names\": {}}", maniBuildReasonForm, maniGraphFormValue, "{}", 1},
    // Read, then refused by the build: a rule's partner that does not
    // pair back; an empty name.
    {TOP "\"rules\": [{\"find\": \"L\", \"place\": \"suffix\", \"replace\": \"R\"}]}",
     maniBuildReasonMirrorPair, kArmL, NULL, 0},
    {TOP "\"names\": [\"curveL\", \"\"]}", maniBuildReasonMirrorName, 1, NULL, 0},
};

static void TestReadRefusals(void)
{
    CheckRefusals(Read, kRefusals, sizeof kRefusals / sizeof kRefusals[0]);
    maniBuilder* builder = NewBuilder(NULL);
    CHECK(maniBuildMirrorMapJson(NULL, "", 0, &s_skeleton).result == mani_errorInvalid,
          "no builder");
    CHECK(maniBuildMirrorMapJson(builder, NULL, 1, &s_skeleton).result == mani_errorInvalid &&
              maniBuildMirrorMapJson(builder, "", 0, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == 2,
          "misuses");
    maniDestroyBuilder(builder);
}

static void TestWriteRefusals(void)
{
    maniMirrorMapSource source = Map();
    source.ruleCount = 1;
    maniBuildResult r;
    CHECK(Write(&source, &r) == 0 && r.result == mani_errorInvalid &&
              r.reason == maniBuildReasonMirrorPair && r.item == kArmL,
          "refused as the build refuses");
    source = Map();
    CHECK(maniWriteMirrorMapJson(NULL, &source).result == mani_errorInvalid, "no builder");
    maniBuilder* builder = NewBuilder(NULL);
    CHECK(maniWriteMirrorMapJson(builder, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(builder) == 1,
          "no source, a misuse");
    maniDestroyBuilder(builder);
}

static maniBuildResult Call(maniBuilder* builder, bool reading)
{
    maniMirrorMapSource source = Map();
    return reading ? maniBuildMirrorMapJson(builder, kExpected, sizeof kExpected - 1, &s_skeleton)
                   : maniWriteMirrorMapJson(builder, &source);
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2], kExpected, sizeof kExpected - 1);
    }
    CHECK(OpenRig(kMirrorRig, kBones, s_skeletonFile, &s_skeleton), "the rig");
    TestWrite();
    TestMinimal();
    TestReadRefusals();
    TestWriteRefusals();
    TestAllocationFailure(Call);
    return s_failures == 0 ? 0 : 1;
}
