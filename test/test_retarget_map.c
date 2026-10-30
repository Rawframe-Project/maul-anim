// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Retargeting maps between a source of three spine bones and
// a two-bone arm and a target of four spine bones and a three-bone arm:
// each target bone's plan (a pair, an interpolated chain's places and
// slerp points, one to one with extra bones at their reference, one to
// one reversed, the pelvis with its height ratio, names matched); given
// reference poses taken over the rests; every builder refusal with its
// item; the file opened, its identities, and every crafted word refused
// at its offset.
// Run as test_retarget_map --seed <file> it writes a map, the seed of
// the fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "negative_kinds.h"
#include "retarget_file.h"
#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/retarget.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFiles[2][4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static maniSkeleton s_source;
static maniSkeleton s_target;
static size_t s_size;

typedef struct Bone
{
    const char* name;
    int32_t parent;
    maniVec3 offset;
} Bone;

// Root, hips, three spine bones (0.2 apart), an arm of two, a head.
static const Bone kSource[8] = {{"root", -1, {0.0f, 0.0f, 0.0f}}, {"hips", 0, {0.0f, 1.0f, 0.0f}},
                                {"s1", 1, {0.0f, 0.2f, 0.0f}},    {"s2", 2, {0.0f, 0.2f, 0.0f}},
                                {"chest", 3, {0.0f, 0.2f, 0.0f}}, {"armL", 4, {0.2f, 0.1f, 0.0f}},
                                {"handL", 5, {0.3f, 0.0f, 0.0f}}, {"head", 4, {0.0f, 0.3f, 0.0f}}};

// Root, a pelvis at 0.9, four spine bones (0.15 apart), an arm of three,
// a head.
static const Bone kTarget[10] = {
    {"root", -1, {0.0f, 0.0f, 0.0f}},    {"pelvis", 0, {0.0f, 0.9f, 0.0f}},
    {"sp1", 1, {0.0f, 0.15f, 0.0f}},     {"sp2", 2, {0.0f, 0.15f, 0.0f}},
    {"sp3", 3, {0.0f, 0.15f, 0.0f}},     {"chest", 4, {0.0f, 0.15f, 0.0f}},
    {"upperarm", 5, {0.2f, 0.1f, 0.0f}}, {"forearm", 6, {0.15f, 0.0f, 0.0f}},
    {"hand", 7, {0.15f, 0.0f, 0.0f}},    {"head", 5, {0.0f, 0.3f, 0.0f}}};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

static void Skeleton(const Bone* bones, uint32_t count, int side, maniSkeleton* out)
{
    maniBoneSource sources[16];
    for (uint32_t b = 0; b < count; ++b)
    {
        sources[b] = (maniBoneSource){bones[b].name,
                                      (uint32_t)strlen(bones[b].name),
                                      bones[b].parent,
                                      {bones[b].offset, {0, 0, 0, 1}, {1, 1, 1}}};
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {sources, count};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFiles[side], sizeof s_skeletonFiles[side],
                                    &size) == mani_success &&
              maniOpenSkeleton(s_skeletonFiles[side], size, out).result == mani_success,
          "a skeleton");
    maniDestroyBuilder(builder);
}

static const maniRetargetChainSource kSpine = {
    2, 4, 2, 5, maniRetargetRotationInterpolated, 1.0f, maniRetargetTranslationOwn, false};
static const maniRetargetChainSource kArm = {
    5, 6, 6, 8, maniRetargetRotationOneToOne, 0.5f, maniRetargetTranslationScaled, true};
static const maniRetargetPelvisSource kPelvis = {
    1, 1, {0.0f, 1.0f, 0.0f}, 1.0f, 0.5f, {0.0f, 1.0f, 0.0f}, {0.25f, 0.0f, 0.0f}, 1.0f, 0.75f};

static maniRetargetMapSource Map(const maniRetargetChainSource* chains, uint32_t chainCount)
{
    return (maniRetargetMapSource){.source = &s_source,
                                   .target = &s_target,
                                   .chains = chains,
                                   .pelvis = &kPelvis,
                                   .chainCount = chainCount,
                                   .matchNames = true};
}

// Whether the source builds to the size bytes at built whatever bytes pad
// it and its pairs, chains and pelvis.
static bool PaddingIgnored(const maniRetargetMapSource* source, const uint8_t* built, size_t size)
{
    bool same = true;
    for (int k = 0; k < 2; ++k)
    {
        uint8_t pattern = kTestPadPatterns[k];
        maniRetargetMapSource s = *source;
        maniRetargetPairSource* pairs = TestCopyOf(s.pairs, s.pairCount, sizeof *pairs);
        maniRetargetChainSource* chains = TestCopyOf(s.chains, s.chainCount, sizeof *chains);
        maniRetargetPelvisSource* pelvis = TestCopyOf(s.pelvis, 1, sizeof *pelvis);
        TEST_PAD(pairs, pairs != NULL ? s.pairCount : 0, kFieldsRetargetPairSource, pattern);
        TEST_PAD(chains, chains != NULL ? s.chainCount : 0, kFieldsRetargetChainSource, pattern);
        TEST_PAD(pelvis, pelvis != NULL ? 1 : 0, kFieldsRetargetPelvisSource, pattern);
        s.pairs = pairs != NULL ? pairs : s.pairs;
        s.chains = chains != NULL ? chains : s.chains;
        s.pelvis = pelvis;
        TEST_PAD(&s, 1, kFieldsRetargetMapSource, pattern);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        size_t padded = 0;
        uint8_t* bytes = maniBuildRetargetMap(builder, &s).result == mani_success
                             ? TestCopyBuilt(builder, &padded)
                             : NULL;
        same = same && bytes != NULL && padded == size && memcmp(bytes, built, size) == 0;
        free(bytes);
        maniDestroyBuilder(builder);
        free(pairs);
        free(chains);
        free(pelvis);
    }
    return same;
}

static maniBuildResult Build(const maniRetargetMapSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildRetargetMap(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success,
              "copied");
        CHECK(PaddingIgnored(source, s_file, s_size), "the same map under either padding");
    }
    maniDestroyBuilder(builder);
    return r;
}

static float LoadF32(const uint8_t* p)
{
    uint32_t bits = maniLoadU32(p);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof v);
    return v;
}

// Whether target bone b's plan is kind, a, b2, t (within 1e-6), weight,
// translation and flags.
static bool PlanIs(const maniRetargetMap* m, uint32_t b, uint32_t kind, uint32_t a, uint32_t b2,
                   float t, float weight, uint32_t translation, uint32_t flags)
{
    const uint8_t* e = m->plan + (size_t)b * MANI_RETARGET_PLAN_SIZE;
    return maniLoadU32(e + maniRetargetPlanKind) == kind &&
           maniLoadU32(e + maniRetargetPlanA) == a && maniLoadU32(e + maniRetargetPlanB) == b2 &&
           fabsf(LoadF32(e + maniRetargetPlanT) - t) <= 1e-6f &&
           LoadF32(e + maniRetargetPlanWeight) == weight &&
           maniLoadU32(e + maniRetargetPlanTranslation) == translation &&
           maniLoadU32(e + maniRetargetPlanFlags) == flags;
}

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniRetargetMap retargetMap;
    return maniOpenRetargetMap(bytes, size, &retargetMap);
}

static void TestPlan(void)
{
    const maniRetargetChainSource chains[2] = {kSpine, kArm};
    maniRetargetMapSource source = Map(chains, 2);
    maniRetargetMap m;
    CHECK(Build(&source).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success,
          "built and opened");
    CHECK(TestOtherVersionsRefused(s_file, s_size, OpenFile), "another format version refused");
    CHECK(maniRetargetMap_GetSourceIdentity(&m) == maniSkeleton_GetIdentity(&s_source) &&
              maniRetargetMap_GetTargetIdentity(&m) == maniSkeleton_GetIdentity(&s_target) &&
              maniRetargetMap_GetIdentity(&m) != 0 && m.sourceBoneCount == 8 &&
              m.targetBoneCount == 10,
          "the skeletons' identities and counts");
    const uint32_t none = MANI_NO_BONE;
    const uint32_t one = maniRetargetPlanOne;
    const uint32_t two = maniRetargetPlanTwo;
    CHECK(PlanIs(&m, 0, one, 0, none, 0.0f, 1.0f, 0, MANI_RETARGET_PLAN_SAME),
          "the root by name, in its source's frame (both rest at the origin)");
    CHECK(PlanIs(&m, 1, one, 1, none, 0.0f, 0.75f, maniRetargetPlanPelvis, 0),
          "the pelvis: its rotation weight, its own translation");
    // Source places 0, 0.5, 1; target 0, 1/3, 2/3, 1.
    CHECK(PlanIs(&m, 2, one, 2, none, 0.0f, 1.0f, 0, 0) &&
              PlanIs(&m, 3, two, 2, 3, 2.0f / 3.0f, 1.0f, 0, 0) &&
              PlanIs(&m, 4, two, 3, 4, 1.0f / 3.0f, 1.0f, 0, 0) &&
              PlanIs(&m, 5, one, 4, none, 0.0f, 1.0f, 0, 0),
          "an interpolated chain: the ends on the ends, the middle bones between two");
    CHECK(PlanIs(&m, 6, one, 5, none, 0.0f, 0.5f, maniRetargetTranslationScaled, 1) &&
              PlanIs(&m, 7, one, 6, none, 0.0f, 0.5f, maniRetargetTranslationScaled, 1) &&
              PlanIs(&m, 8, maniRetargetPlanReference, none, none, 0.0f, 0.0f, 0, 0),
          "one to one: the extra target bone at its reference");
    CHECK(PlanIs(&m, 9, one, 7, none, 0.0f, 1.0f, 0, 0), "the head by name");
    const uint8_t* h = m.header;
    CHECK(maniLoadU32(h + maniRetargetHeaderPelvisSource) == 1 &&
              maniLoadU32(h + maniRetargetHeaderPelvisTarget) == 1 &&
              fabsf(LoadF32(h + maniRetargetHeaderHorizontal) - 0.9f) <= 1e-6f &&
              fabsf(LoadF32(h + maniRetargetHeaderVertical) - 0.45f) <= 1e-6f &&
              LoadF32(h + maniRetargetHeaderAbsolute + 4) == 1.0f &&
              LoadF32(h + maniRetargetHeaderToSource) == 0.25f &&
              LoadF32(h + maniRetargetHeaderSourcePelvis + 4) == 1.0f &&
              LoadF32(h + maniRetargetHeaderTargetPelvis + 4) == 0.9f &&
              fabsf(LoadF32(h + maniRetargetHeaderRatio) - 0.9f) <= 1e-6f,
          "the pelvis: heights 1 and 0.9, the scales their ratio times the factors");
    // A pair on the head: the name leaves it.
    const maniRetargetPairSource head = {9, 4, 1.0f, maniRetargetTranslationOwn, false};
    source.pairs = &head;
    source.pairCount = 1;
    CHECK(Build(&source).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success &&
              PlanIs(&m, 9, one, 4, none, 0.0f, 1.0f, 0, 0),
          "a name never overrides a pair");
    // Reversed, without names or a pelvis: from the end.
    maniRetargetChainSource reversed = kArm;
    reversed.rotation = maniRetargetRotationOneToOneReversed;
    source = (maniRetargetMapSource){
        .source = &s_source, .target = &s_target, .chains = &reversed, .chainCount = 1};
    CHECK(Build(&source).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success,
          "a reversed chain");
    CHECK(PlanIs(&m, 6, maniRetargetPlanReference, none, none, 0.0f, 0.0f, 0, 0) &&
              PlanIs(&m, 7, one, 5, none, 0.0f, 0.5f, maniRetargetTranslationScaled, 1) &&
              PlanIs(&m, 8, one, 6, none, 0.0f, 0.5f, maniRetargetTranslationScaled, 1) &&
              PlanIs(&m, 0, maniRetargetPlanReference, none, none, 0.0f, 0.0f, 0, 0),
          "one to one reversed; nothing by name");
    CHECK(maniLoadU32(m.header + maniRetargetHeaderPelvisSource) == none &&
              LoadF32(m.header + maniRetargetHeaderUp + 4) == 1.0f &&
              LoadF32(m.header + maniRetargetHeaderHorizontal) == 1.0f,
          "no pelvis: its fixed form");
}

// A reference pose given for the target: its locals stored and its model
// rotations taken from them; an interpolated chain placed by its
// lengths, which the pose changes.
static void TestReference(void)
{
    maniTransform locals[10];
    for (uint32_t b = 0; b < 10; ++b)
    {
        locals[b] = maniSkeleton_GetRest(&s_target, b);
    }
    // sp2 three times as long: the places 0, 0.6, 0.8, 1.
    locals[3].translation = (maniVec3){0.0f, 0.45f, 0.0f};
    locals[2].rotation = (maniQuat){0.0f, 0.0f, 0.38268343f, 0.92387953f};
    maniRetargetMapSource source = Map(&kSpine, 1);
    source.targetReference = locals;
    maniRetargetMap m;
    CHECK(Build(&source).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success,
          "a given reference");
    const uint8_t* e3 = m.targetBones + 3 * MANI_RETARGET_TARGET_SIZE;
    const uint8_t* e4 = m.targetBones + 4 * MANI_RETARGET_TARGET_SIZE;
    CHECK(LoadF32(e3 + 36) == 0.45f && fabsf(LoadF32(e3 + 8) - 0.38268343f) <= 1e-6f &&
              fabsf(LoadF32(e4 + 8) - 0.38268343f) <= 1e-6f,
          "its locals and model rotations stored");
    CHECK(PlanIs(&m, 3, maniRetargetPlanTwo, 3, 4, 0.2f, 1.0f, 0, 0) &&
              PlanIs(&m, 4, maniRetargetPlanTwo, 3, 4, 0.6f, 1.0f, 0, 0),
          "the places from its lengths");
}

// A target chain of no length placed by index; two source bones at one
// place give a target bone there the later one.
static void TestPlaces(void)
{
    maniTransform target[10];
    maniTransform source[8];
    for (uint32_t b = 0; b < 10; ++b)
    {
        target[b] = maniSkeleton_GetRest(&s_target, b);
    }
    for (uint32_t b = 0; b < 8; ++b)
    {
        source[b] = maniSkeleton_GetRest(&s_source, b);
    }
    for (uint32_t b = 3; b <= 5; ++b)
    {
        target[b].translation = (maniVec3){0.0f, 0.0f, 0.0f};
    }
    maniRetargetMapSource map = Map(&kSpine, 1);
    map.targetReference = target;
    maniRetargetMap m;
    CHECK(Build(&map).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success &&
              PlanIs(&m, 3, maniRetargetPlanTwo, 2, 3, 2.0f / 3.0f, 1.0f, 0, 0) &&
              PlanIs(&m, 4, maniRetargetPlanTwo, 3, 4, 1.0f / 3.0f, 1.0f, 0, 0),
          "a chain of no length: places by index");
    // s2 on s1: source places 0, 0, 1; the target's start at 0 takes s2.
    source[3].translation = (maniVec3){0.0f, 0.0f, 0.0f};
    map = Map(&kSpine, 1);
    map.sourceReference = source;
    CHECK(Build(&map).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success &&
              PlanIs(&m, 2, maniRetargetPlanOne, 3, MANI_NO_BONE, 0.0f, 1.0f, 0, 0),
          "two source bones at one place: the later");
}

static bool Refused(const maniRetargetMapSource* source, maniBuildReason reason, uint32_t item,
                    uint32_t other)
{
    maniBuildResult r = Build(source);
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item &&
           r.other == other;
}

static void TestRefusals(void)
{
    maniRetargetMapSource source = Map(&kSpine, 1);
    source.target = NULL;
    CHECK(Refused(&source, maniBuildReasonNone, 0, 0), "no target skeleton");
    source = Map(&kSpine, 1);
    source.pairCount = 1;
    CHECK(Refused(&source, maniBuildReasonNone, 0, 0), "a count without its list");
    maniTransform locals[10];
    for (uint32_t b = 0; b < 10; ++b)
    {
        locals[b] = maniSkeleton_GetRest(&s_target, b);
    }
    locals[4].translation.x = NAN;
    source = Map(&kSpine, 1);
    source.targetReference = locals;
    CHECK(Refused(&source, maniBuildReasonRetargetReference, 4, 1), "a reference not finite");
    locals[4] = maniSkeleton_GetRest(&s_target, 4);
    locals[6].rotation.w = 0.9f;
    CHECK(Refused(&source, maniBuildReasonRetargetReference, 6, 1),
          "a reference rotation not of length 1");
    maniRetargetPairSource pairs[2] = {{9, 7, 1.0f, maniRetargetTranslationOwn, false},
                                       {9, 7, 1.0f, maniRetargetTranslationOwn, false}};
    source = Map(&kSpine, 1);
    source.pairs = pairs;
    source.pairCount = 2;
    CHECK(Refused(&source, maniBuildReasonRetargetPair, 1, 9), "a target bone twice");
    source.pairCount = 1;
    const maniRetargetPairSource badPairs[4] = {{10, 7, 1.0f, 0, false},
                                                {9, 8, 1.0f, 0, false},
                                                {9, 7, 1.5f, 0, false},
                                                {9, 7, 1.0f, 3, false}};
    int refused = 0;
    for (int k = 0; k < 4; ++k)
    {
        source.pairs = &badPairs[k];
        refused += Refused(&source, maniBuildReasonRetargetPair, 0, badPairs[k].target);
    }
    CHECK(refused == 4, "pairs: bones past, a weight past 1, an unknown translation");
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        const maniRetargetPairSource negative = {9, 7, 1.0f, kNegativeKinds[k], false};
        source.pairs = &negative;
        CHECK(Refused(&source, maniBuildReasonRetargetPair, 0, 9), "a negative pair translation");
    }
    // A pair on the chain's sp3: the chain refused at that bone.
    const maniRetargetPairSource onChain = {4, 3, 1.0f, 0, false};
    source.pairs = &onChain;
    CHECK(Refused(&source, maniBuildReasonRetargetChain, 0, 4), "a chain bone mapped before");
    source.pairs = NULL;
    source.pairCount = 0;
    maniRetargetChainSource bad[6];
    for (int k = 0; k < 6; ++k)
    {
        bad[k] = kSpine;
    }
    bad[0].sourceEnd = 8;
    bad[1].targetStart = 5;
    bad[2].sourceStart = 4;
    bad[3].rotation = 4;
    bad[4].translation = 3;
    bad[5].rotationWeight = -0.1f;
    refused = 0;
    for (int k = 0; k < 6; ++k)
    {
        source.chains = &bad[k];
        refused += Refused(&source, maniBuildReasonRetargetChain, 0, 0);
    }
    CHECK(refused == 6, "chains: a bone past, ends not below starts, modes and weights");
    for (int k = 0; k < NEGATIVE_KIND_COUNT; ++k)
    {
        maniRetargetChainSource negative[2] = {kSpine, kSpine};
        negative[0].rotation = kNegativeKinds[k];
        negative[1].translation = kNegativeKinds[k];
        source.chains = &negative[0];
        CHECK(Refused(&source, maniBuildReasonRetargetChain, 0, 0), "a negative chain rotation");
        source.chains = &negative[1];
        CHECK(Refused(&source, maniBuildReasonRetargetChain, 0, 0), "a negative chain translation");
    }
    maniRetargetPelvisSource pelvis[8];
    for (int k = 0; k < 8; ++k)
    {
        pelvis[k] = kPelvis;
    }
    pelvis[0].source = 8;
    pelvis[1].target = 2;
    pelvis[2].up = (maniVec3){0.0f, 2.0f, 0.0f};
    pelvis[3].horizontal = -1.0f;
    pelvis[4].absolute.z = 1.5f;
    pelvis[5].translationWeight = NAN;
    pelvis[6].source = 0;
    pelvis[7].up = (maniVec3){0.0f, -1.0f, 0.0f};
    source = Map(&kSpine, 1);
    refused = 0;
    for (int k = 0; k < 8; ++k)
    {
        source.pelvis = &pelvis[k];
        refused += Refused(&source, maniBuildReasonRetargetPelvis, 0, 0);
    }
    CHECK(refused == 8, "the pelvis: bones past or mapped, the axis, numbers, heights at 0");
}

static void Rehash(uint8_t* file, size_t size)
{
    maniStoreU64(file + 32, maniHashBytes(MANI_HASH_INIT, file + MANI_CONTAINER_HEADER_SIZE,
                                          size - MANI_CONTAINER_HEADER_SIZE));
    maniContainer c;
    CHECK(maniOpenContainer(file, size, maniKindRetargetMap, MANI_RETARGET_FORMAT_VERSION, &c)
                  .result == mani_success,
          "container");
    const uint8_t* data[MANI_RETARGET_SECTIONS];
    uint64_t sizes[MANI_RETARGET_SECTIONS];
    for (uint32_t t = 1; t <= MANI_RETARGET_SECTIONS; ++t)
    {
        uint64_t at = 0;
        data[t - 1] = file;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &data[t - 1], &sizes[t - 1], &at);
    }
    maniStoreU64(file + 40, maniRetargetComputeIdentity(data, sizes));
}

static uint32_t Bits(float f)
{
    uint32_t bits = 0;
    memcpy(&bits, &f, sizeof bits);
    return bits;
}

// Whether the file with the word at byte at set to value is refused at
// byte wrong.
static bool WordRefused(size_t at, uint32_t value, size_t wrong)
{
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + at, value);
    Rehash(s_crafted, s_size);
    maniRetargetMap m;
    maniFileResult r = maniOpenRetargetMap(s_crafted, s_size, &m);
    return r.result == mani_errorFormat && r.offset == wrong &&
           r.tag == TestTagAt(s_crafted, s_size, wrong);
}

// A source chain one bone past MANI_MAX_IK_CHAIN_BONES: the chain
// refused at the limit; one bone shorter, built.
static void TestChainPastBound(void)
{
    enum
    {
        kChain = MANI_MAX_IK_CHAIN_BONES + 1
    };
    static _Alignas(16) uint8_t file[16384];
    static char names[kChain][4];
    maniBoneSource bones[kChain];
    for (uint32_t b = 0; b < kChain; ++b)
    {
        (void)snprintf(names[b], sizeof names[b], "%03u", (unsigned)b);
        bones[b] = (maniBoneSource){
            names[b], 3, (int32_t)b - 1, {{0.0f, 0.1f, 0.0f}, {0, 0, 0, 1}, {1, 1, 1}}};
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, kChain};
    maniSkeleton chain;
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenSkeleton(file, size, &chain).result == mani_success,
          "a chain of 33 bones");
    maniDestroyBuilder(builder);
    maniRetargetChainSource c = kSpine;
    c.sourceStart = 0;
    c.sourceEnd = kChain - 1;
    maniRetargetMapSource map = Map(&c, 1);
    map.source = &chain;
    maniBuildResult r = Build(&map);
    CHECK(r.result == mani_errorLimit && r.reason == maniBuildReasonRetargetChain && r.item == 0 &&
              r.other == 0,
          "a source chain past the bound at the limit");
    c.sourceStart = 1;
    CHECK(Build(&map).result == mani_success, "a source chain at the bound");
}

static void TestFile(void)
{
    const maniRetargetChainSource chains[2] = {kSpine, kArm};
    maniRetargetMapSource source = Map(chains, 2);
    maniRetargetMap m;
    CHECK(Build(&source).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success,
          "a map");
    memcpy(s_crafted, s_file, s_size);
    Rehash(s_crafted, s_size);
    CHECK(maniOpenRetargetMap(s_crafted, s_size, &m).result == mani_success,
          "rehashed unchanged, opened");
    CHECK(maniOpenRetargetMap(s_file, s_size, &m).result == mani_success, "the original");
    size_t h = (size_t)(m.header - s_file);
    size_t sb = (size_t)(m.sourceBones - s_file);
    size_t tb = (size_t)(m.targetBones - s_file);
    size_t p = (size_t)(m.plan - s_file);
    CHECK(WordRefused(h + maniRetargetHeaderSourceBones, 0, h + maniRetargetHeaderSourceBones) &&
              WordRefused(h + maniRetargetHeaderTargetBones, 9, h + maniRetargetHeaderTargetBones),
          "bone counts zero or not the sections'");
    CHECK(WordRefused(h + maniRetargetHeaderPelvisSource, 8, h + maniRetargetHeaderPelvisSource),
          "a pelvis bone past");
    CHECK(WordRefused(h + maniRetargetHeaderUp + 4, Bits(2.0f), h + maniRetargetHeaderUp) &&
              WordRefused(h + maniRetargetHeaderUp, Bits(NAN), h + maniRetargetHeaderUp),
          "the axis not of length 1 or not finite");
    CHECK(WordRefused(h + maniRetargetHeaderVertical, Bits(-1.0f), h + maniRetargetHeaderVertical),
          "a scale below 0");
    CHECK(WordRefused(h + maniRetargetHeaderToSource + 8, Bits(1.5f),
                      h + maniRetargetHeaderToSource + 8) &&
              WordRefused(h + maniRetargetHeaderTranslationWeight, Bits(-0.5f),
                          h + maniRetargetHeaderTranslationWeight),
          "a blend or the weight outside [0, 1]");
    CHECK(WordRefused(h + maniRetargetHeaderTargetPelvis + 4, Bits(INFINITY),
                      h + maniRetargetHeaderTargetPelvis + 4),
          "a pelvis position not finite");
    CHECK(WordRefused(h + maniRetargetHeaderRatio, Bits(0.0f), h + maniRetargetHeaderRatio) &&
              WordRefused(h + maniRetargetHeaderRatio, Bits(INFINITY), h + maniRetargetHeaderRatio),
          "a height ratio not positive and finite");
    CHECK(WordRefused(sb + MANI_RETARGET_SOURCE_SIZE * 3 + 12, Bits(0.5f),
                      sb + MANI_RETARGET_SOURCE_SIZE * 3) &&
              WordRefused(tb + MANI_RETARGET_TARGET_SIZE * 2 + 20, Bits(NAN),
                          tb + MANI_RETARGET_TARGET_SIZE * 2 + 20) &&
              WordRefused(tb + MANI_RETARGET_TARGET_SIZE * 2 + 40, Bits(INFINITY),
                          tb + MANI_RETARGET_TARGET_SIZE * 2 + 40),
          "references: a rotation not of length 1, numbers not finite");
    CHECK(WordRefused(tb + MANI_RETARGET_TARGET_SIZE * 2 + 52, Bits(NAN),
                      tb + MANI_RETARGET_TARGET_SIZE * 2 + 52),
          "a reference's last number not finite");
    // Target bone 3 (two sources), 2 (one), 8 (reference), 1 (the pelvis).
    size_t two = p + 3 * MANI_RETARGET_PLAN_SIZE;
    size_t one = p + 2 * MANI_RETARGET_PLAN_SIZE;
    size_t ref = p + 8 * MANI_RETARGET_PLAN_SIZE;
    size_t pel = p + 1 * MANI_RETARGET_PLAN_SIZE;
    CHECK(WordRefused(one, 3, one) && WordRefused(one + 4, 8, one + 4) &&
              WordRefused(one + 8, 3, one + 8) && WordRefused(one + 12, Bits(0.5f), one + 12) &&
              WordRefused(one + 12, Bits(-0.0f), one + 12) &&
              WordRefused(one + 16, Bits(1.5f), one + 16) && WordRefused(one + 20, 3, one + 20) &&
              WordRefused(one + 24, 4, one + 24),
          "a one-source plan: kind, bones, place, weight, translation, flags");
    CHECK(WordRefused(p + 16, Bits(0.5f), p + 24),
          "the root's plan in its source's frame at a weight below 1");
    CHECK(WordRefused(two + 8, 2, two + 8) && WordRefused(two + 12, Bits(1.0f), two + 12) &&
              WordRefused(two + 12, Bits(0.0f), two + 12) &&
              WordRefused(two + 24, MANI_RETARGET_PLAN_SAME, two + 24),
          "a two-source plan: b equal to a, a place not inside (0, 1), in a source's frame");
    CHECK(WordRefused(ref + 4, 0, ref + 4) && WordRefused(ref + 16, Bits(-0.0f), ref + 16) &&
              WordRefused(ref + 20, 1, ref + 20) && WordRefused(ref + 24, 1, ref + 24),
          "a reference plan: a bone, a weight, a translation, flags");
    CHECK(WordRefused(pel, maniRetargetPlanTwo, pel) && WordRefused(pel + 4, 2, pel + 4) &&
              WordRefused(pel + 20, 0, pel + 20) && WordRefused(pel + 24, 1, pel + 24),
          "the pelvis's plan: one source, the pelvis's, its translation, no flags");
    // A word changed and only the payload's hash made right: the
    // identity no longer the sections'.
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + one + 16, Bits(0.25f));
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniFileResult identity = maniOpenRetargetMap(s_crafted, s_size, &m);
    CHECK(identity.result == mani_errorFormat && identity.offset == 40,
          "an identity not the sections' refused");
    // Without a pelvis its fixed form.
    source = (maniRetargetMapSource){
        .source = &s_source, .target = &s_target, .chains = &kSpine, .chainCount = 1};
    CHECK(Build(&source).result == mani_success &&
              maniOpenRetargetMap(s_file, s_size, &m).result == mani_success,
          "a map without a pelvis");
    h = (size_t)(m.header - s_file);
    CHECK(WordRefused(h + maniRetargetHeaderHorizontal, Bits(0.5f),
                      h + maniRetargetHeaderHorizontal) &&
              WordRefused(h + maniRetargetHeaderUp + 4, Bits(-1.0f), h + maniRetargetHeaderUp) &&
              WordRefused(h + maniRetargetHeaderSourcePelvis, Bits(0.5f),
                          h + maniRetargetHeaderSourcePelvis) &&
              WordRefused(h + maniRetargetHeaderRatio, Bits(0.5f), h + maniRetargetHeaderRatio),
          "no pelvis: scales 1, the axis +y, the rest 0");
}

// The map of TestPlan written to path.
static bool Seed(const char* path)
{
    const maniRetargetChainSource chains[2] = {kSpine, kArm};
    maniRetargetMapSource source = Map(chains, 2);
    if (Build(&source).result != mani_success)
    {
        return false;
    }
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return false;
    }
    size_t written = fwrite(s_file, 1, s_size, out);
    return fclose(out) == 0 && written == s_size;
}

int main(int argc, char** argv)
{
    Skeleton(kSource, 8, 0, &s_source);
    Skeleton(kTarget, 10, 1, &s_target);
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return Seed(argv[2]) && s_failures == 0 ? 0 : 1;
    }
    TestPlan();
    TestReference();
    TestPlaces();
    TestRefusals();
    TestChainPastBound();
    TestFile();
    return s_failures == 0 ? 0 : 1;
}
