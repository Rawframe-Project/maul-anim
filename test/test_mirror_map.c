// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Mirror maps of a rig with left and right clavicles, arms,
// hands and legs and a prop under the right hand: bones paired by
// suffix rules (the prop to itself, so the hierarchy's first unmatched
// bone), prefix and anywhere rules, a rule naming no bone giving way,
// pairs before rules, names paired both ways and by the rules
// (a repeat with the same partner kept once); every builder refusal with
// its item; the file opened, its readers, and every crafted word refused
// at its offset.
// Run as test_mirror_map --seed <file> it writes a map, the seed of the
// fuzz target.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "mirror_file.h"
#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/mirror.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;
static maniSkeleton s_skeleton;

enum
{
    kRoot,
    kHips,
    kSpine,
    kChest,
    kHead,
    kClavL,
    kArmL,
    kHandL,
    kClavR,
    kArmR,
    kHandR,
    kPropR,
    kLegL,
    kLegR,
    kBones
};

#define REST {0.0f, 0.0f, 0.0f, 1.0f}

static const maniBoneSource kRig[kBones] = {
    {"root", 4, -1, {{0, 0, 0}, REST, {1, 1, 1}}},
    {"hips", 4, kRoot, {{0, 1, 0}, REST, {1, 1, 1}}},
    {"spine", 5, kHips, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
    {"chest", 5, kSpine, {{0, 0.2f, 0}, REST, {1, 1, 1}}},
    {"head", 4, kChest, {{0, 0.3f, 0}, REST, {1, 1, 1}}},
    {"clavL", 5, kChest, {{0.1f, 0.1f, 0}, {0, 0, 0.25881904f, 0.96592583f}, {1, 1, 1}}},
    {"armL", 4, kClavL, {{0.2f, 0, 0}, REST, {1, 1, 1}}},
    {"handL", 5, kArmL, {{0.3f, 0, 0}, REST, {1, 1, 1}}},
    {"clavR", 5, kChest, {{-0.1f, 0.1f, 0}, {0, 0.70710678f, 0, 0.70710678f}, {1, 1, 1}}},
    {"armR", 4, kClavR, {{0, 0, 0.2f}, REST, {1, 1, 1}}},
    {"handR", 5, kArmR, {{0, 0, 0.3f}, REST, {1, 1, 1}}},
    {"propR", 5, kHandR, {{0, 0, 0.1f}, REST, {1, 1, 1}}},
    {"legL", 4, kHips, {{0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
    {"legR", 4, kHips, {{-0.1f, -0.1f, 0}, REST, {1, 1, 1}}},
};

static const maniMirrorRuleSource kRules[2] = {{"L", 1, maniMirrorSuffix, "R", 1},
                                               {"R", 1, maniMirrorSuffix, "L", 1}};
static const maniMirrorNamePairSource kNamePairs[1] = {{"footL", 5, 5, "footR"}};
// curveL by the rules; footL again by the rules, to the pair's partner;
// idle by none.
static const maniMirrorNameSource kNames[3] = {{"curveL", 6}, {"footL", 5}, {"idle", 4}};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    return builder;
}

static void Setup(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {kRig, kBones};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "the rig");
    maniDestroyBuilder(builder);
}

static maniMirrorMapSource Source(void)
{
    return (maniMirrorMapSource){.skeleton = &s_skeleton,
                                 .rules = kRules,
                                 .namePairs = kNamePairs,
                                 .names = kNames,
                                 .ruleCount = 2,
                                 .namePairCount = 1,
                                 .nameCount = 3,
                                 .axis = maniMirrorAxisX};
}

// Whether the source builds to the size bytes at built whatever bytes pad
// it and its pairs, rules and names.
static bool PaddingIgnored(const maniMirrorMapSource* source, const uint8_t* built, size_t size)
{
    bool same = true;
    for (int k = 0; k < 2; ++k)
    {
        uint8_t pattern = kTestPadPatterns[k];
        maniMirrorMapSource s = *source;
        maniMirrorPairSource* pairs = TestCopyOf(s.pairs, s.pairCount, sizeof *pairs);
        maniMirrorRuleSource* rules = TestCopyOf(s.rules, s.ruleCount, sizeof *rules);
        maniMirrorNamePairSource* namePairs =
            TestCopyOf(s.namePairs, s.namePairCount, sizeof *namePairs);
        maniMirrorNameSource* names = TestCopyOf(s.names, s.nameCount, sizeof *names);
        TEST_PAD(pairs, pairs != NULL ? s.pairCount : 0, kFieldsMirrorPairSource, pattern);
        TEST_PAD(rules, rules != NULL ? s.ruleCount : 0, kFieldsMirrorRuleSource, pattern);
        TEST_PAD(namePairs, namePairs != NULL ? s.namePairCount : 0, kFieldsMirrorNamePairSource,
                 pattern);
        TEST_PAD(names, names != NULL ? s.nameCount : 0, kFieldsMirrorNameSource, pattern);
        s.pairs = pairs != NULL ? pairs : s.pairs;
        s.rules = rules != NULL ? rules : s.rules;
        s.namePairs = namePairs != NULL ? namePairs : s.namePairs;
        s.names = names != NULL ? names : s.names;
        TEST_PAD(&s, 1, kFieldsMirrorMapSource, pattern);
        maniBuilder* builder = NewBuilder();
        size_t padded = 0;
        uint8_t* bytes = maniBuildMirrorMap(builder, &s).result == mani_success
                             ? TestCopyBuilt(builder, &padded)
                             : NULL;
        same = same && bytes != NULL && padded == size && memcmp(bytes, built, size) == 0;
        free(bytes);
        maniDestroyBuilder(builder);
        free(pairs);
        free(rules);
        free(namePairs);
        free(names);
    }
    return same;
}

static maniBuildResult Build(const maniMirrorMapSource* source, maniMirrorMap* out)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildMirrorMap(builder, source);
    s_size = 0;
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
                  maniOpenMirrorMap(s_file, s_size, out).result == mani_success,
              "the map opens");
        CHECK(PaddingIgnored(source, s_file, s_size), "the same map under either padding");
    }
    maniDestroyBuilder(builder);
    return r;
}

static bool NameIs(const maniMirrorMap* map, const char* name, const char* want)
{
    size_t length = 0;
    const char* got = maniMirrorName(map, name, strlen(name), &length);
    return length == strlen(want) && memcmp(got, want, length) == 0;
}

static void TestPairs(void)
{
    maniMirrorMapSource source = Source();
    maniMirrorMap map;
    CHECK(Build(&source, &map).result == mani_success, "a map by rules builds");
    static const uint32_t kPartner[kBones] = {kRoot,  kHips,  kSpine, kChest, kHead,
                                              kClavR, kArmR,  kHandR, kClavL, kArmL,
                                              kHandL, kPropR, kLegR,  kLegL};
    bool partners = true;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        partners = partners && maniMirrorPartner(&map, b) == kPartner[b];
    }
    CHECK(partners, "each bone's partner by the suffix rules, the prop its own");
    CHECK(maniMirrorMapUnmatchedBone(&map) == kPropR, "the prop the first unmatched bone");
    CHECK(maniMirrorPartner(&map, kBones) == MANI_NO_BONE &&
              maniMirrorPartner(NULL, 0) == MANI_NO_BONE,
          "no partner past the bones");
    CHECK(maniMirrorMapAxis(&map) == maniMirrorAxisX &&
              maniMirrorMapSkeletonIdentity(&map) == maniSkeletonIdentity(&s_skeleton) &&
              maniMirrorMapIdentity(&map) != 0 && maniMirrorMapIdentity(NULL) == 0 &&
              maniMirrorMapSkeletonIdentity(NULL) == 0 &&
              maniMirrorMapAxis(NULL) == maniMirrorAxisX &&
              maniMirrorMapUnmatchedBone(NULL) == MANI_NO_BONE,
          "the readers");
    CHECK(NameIs(&map, "footL", "footR") && NameIs(&map, "footR", "footL") &&
              NameIs(&map, "curveL", "curveR") && NameIs(&map, "idle", "idle") &&
              NameIs(&map, "curveR", "curveR") && map.nameCount == 3,
          "names paired both ways and by the rules, a repeat kept once");
    const char* idle = "idle";
    CHECK(maniMirrorName(&map, idle, 4, NULL) == idle &&
              maniMirrorName(NULL, idle, 4, NULL) == idle,
          "an unpaired name itself");
    // Pairs come first: the legs crossed with the arms' clavicles is
    // refused below; here the head paired with itself and the legs as
    // the rules would pair them, and the axis z.
    const maniMirrorPairSource pairs[2] = {{kHead, kHead}, {kLegR, kLegL}};
    source.pairs = pairs;
    source.pairCount = 2;
    source.axis = maniMirrorAxisZ;
    CHECK(Build(&source, &map).result == mani_success && maniMirrorPartner(&map, kLegL) == kLegR &&
              maniMirrorPartner(&map, kHead) == kHead && maniMirrorMapAxis(&map) == maniMirrorAxisZ,
          "pairs before rules");
    // Without rules every bone is its own: a symmetric hierarchy.
    source = Source();
    source.ruleCount = 0;
    source.nameCount = 0;
    CHECK(Build(&source, &map).result == mani_success && maniMirrorPartner(&map, kArmL) == kArmL &&
              maniMirrorMapUnmatchedBone(&map) == MANI_NO_BONE && map.nameCount == 2,
          "no rules: each bone its own");
}

static bool Partners(const maniMirrorMapSource* source, const uint32_t (*pairs)[2], uint32_t count)
{
    maniMirrorMap map;
    if (Build(source, &map).result != mani_success)
    {
        return false;
    }
    uint32_t paired = 0;
    for (uint32_t b = 0; b < kBones; ++b)
    {
        uint32_t want = b;
        for (uint32_t i = 0; i < count; ++i)
        {
            want = pairs[i][0] == b ? pairs[i][1] : pairs[i][1] == b ? pairs[i][0] : want;
        }
        paired += maniMirrorPartner(&map, b) == want ? 1u : 0u;
    }
    return paired == kBones;
}

static void TestRulePlaces(void)
{
    maniMirrorMapSource source = Source();
    source.nameCount = 0;
    // Prefixes of other lengths: each hand with its side's leg.
    const maniMirrorRuleSource prefix[2] = {{"hand", 4, maniMirrorPrefix, "leg", 3},
                                            {"leg", 3, maniMirrorPrefix, "hand", 4}};
    source.rules = prefix;
    const uint32_t hands[2][2] = {{kHandL, kLegL}, {kHandR, kLegR}};
    CHECK(Partners(&source, hands, 2), "prefix rules of other lengths");
    // Anywhere: the first L or R of the name.
    const maniMirrorRuleSource anywhere[2] = {{"L", 1, maniMirrorAnywhere, "R", 1},
                                              {"R", 1, maniMirrorAnywhere, "L", 1}};
    source.rules = anywhere;
    const uint32_t sides[5][2] = {
        {kClavL, kClavR}, {kArmL, kArmR}, {kHandL, kHandR}, {kLegL, kLegR}, {kPropR, kPropR}};
    CHECK(Partners(&source, sides, 5), "anywhere rules");
    // A first rule that matches but names no bone gives way to the next.
    const maniMirrorRuleSource next[3] = {{"L", 1, maniMirrorSuffix, "X", 1}, kRules[0], kRules[1]};
    source.rules = next;
    source.ruleCount = 3;
    CHECK(Partners(&source, sides, 5), "a rule naming no bone gives way");
}

static bool Refused(const maniMirrorMapSource* source, maniBuildReason reason, uint32_t item,
                    uint32_t other)
{
    maniMirrorMap map;
    maniBuildResult r = Build(source, &map);
    if (r.reason != reason || r.item != item || r.other != other)
    {
        printf("  reason %u item %u other %u\n", r.reason, r.item, r.other);
    }
    return r.result == mani_errorInvalid && r.reason == reason && r.item == item &&
           r.other == other;
}

static void TestPairRefusals(void)
{
    maniMirrorMapSource source = Source();
    const maniMirrorPairSource far[1] = {{kHead, kBones}};
    source.pairs = far;
    source.pairCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorPair, kBones, 0), "a bone past the rig");
    const maniMirrorPairSource twice[2] = {{kLegL, kLegR}, {kArmL, kLegL}};
    source.pairs = twice;
    source.pairCount = 2;
    CHECK(Refused(&source, maniBuildReasonMirrorPair, kLegL, kLegR), "a bone in two pairs");
    const maniMirrorPairSource firstFar[1] = {{kBones, kHead}};
    source.pairs = firstFar;
    source.pairCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorPair, kBones, 0), "a first bone past the rig");
    const maniMirrorPairSource firstTwice[2] = {{kLegL, kLegR}, {kLegL, kArmL}};
    source.pairs = firstTwice;
    source.pairCount = 2;
    CHECK(Refused(&source, maniBuildReasonMirrorPair, kLegL, kLegR), "a first bone in two pairs");
    // The left arm paired with the right leg: the right arm's rule names
    // the left arm, taken.
    const maniMirrorPairSource taken[1] = {{kArmL, kLegR}};
    source.pairs = taken;
    source.pairCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorPair, kArmR, kArmL), "a rule's partner taken");
    // Rules one way: the left clavicle names the right, which names none.
    source = Source();
    source.ruleCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorPair, kClavL, kClavR),
          "a partner not pairing back");
}

static void TestOtherRefusals(void)
{
    maniMirrorMapSource source = Source();
    maniMirrorRuleSource rules[2] = {kRules[0], kRules[1]};
    source.rules = rules;
    rules[1].findLength = 0;
    CHECK(Refused(&source, maniBuildReasonMirrorRule, 1, 0), "an empty find");
    rules[1] = kRules[1];
    rules[1].place = 3;
    CHECK(Refused(&source, maniBuildReasonMirrorRule, 1, 0), "an unknown place");
    rules[1] = kRules[1];
    rules[1].replace = "\xff";
    CHECK(Refused(&source, maniBuildReasonMirrorRule, 1, 0), "a replace not UTF-8");
    source = Source();
    maniTransform reference[kBones];
    for (uint32_t b = 0; b < kBones; ++b)
    {
        reference[b] = maniSkeletonRest(&s_skeleton, b);
    }
    reference[kArmR].rotation = (maniQuat){0.0f, 0.0f, 0.0f, 0.0f};
    source.reference = reference;
    CHECK(Refused(&source, maniBuildReasonMirrorReference, kArmR, 0), "a rotation of length 0");
    reference[kArmR].rotation = (maniQuat){0.0f, 0.0f, 0.0f, 1.0f};
    reference[kHead].translation.y = HUGE_VALF;
    CHECK(Refused(&source, maniBuildReasonMirrorReference, kHead, 0), "a translation not finite");
    source = Source();
    source.axis = 3;
    CHECK(Refused(&source, maniBuildReasonOption, 0, 0), "an unknown axis");
    source = Source();
    const maniMirrorNamePairSource bad[2] = {{"stepLeft", 8, 9, "stepRight"},
                                             {"stepLeft", 8, 5, "other"}};
    source.namePairs = bad;
    source.namePairCount = 2;
    source.nameCount = 0;
    CHECK(Refused(&source, maniBuildReasonMirrorName, 1, 0), "a name paired twice");
    const maniMirrorNamePairSource empty[1] = {{"stepLeft", 8, 0, ""}};
    source.namePairs = empty;
    source.namePairCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorName, 0, 0), "an empty partner");
    source = Source();
    const maniMirrorNameSource names[1] = {{"\xc3", 1}};
    source.names = names;
    source.nameCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorName, 1, 0), "a listed name not UTF-8");
    source = Source();
    const maniMirrorRuleSource drop[1] = {{"L", 1, maniMirrorSuffix, "", 0}};
    const maniMirrorNameSource whole[1] = {{"L", 1}};
    source.rules = drop;
    source.ruleCount = 1;
    source.names = whole;
    source.nameCount = 1;
    CHECK(Refused(&source, maniBuildReasonMirrorName, 1, 0), "a listed name a rule empties");
    maniBuilder* builder = NewBuilder();
    source = Source();
    CHECK(maniBuildMirrorMap(NULL, &source).result == mani_errorInvalid, "a NULL builder");
    CHECK(maniBuildMirrorMap(builder, NULL).result == mani_errorInvalid, "a NULL source");
    source.rules = NULL;
    CHECK(maniBuildMirrorMap(builder, &source).result == mani_errorInvalid, "rules NULL");
    CHECK(maniBuilderMisuses(builder) == 2, "misuses counted");
    maniDestroyBuilder(builder);
}

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindMirrorMap, MANI_MIRROR_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "the section");
    return at;
}

static bool RefusedAt(uint64_t offset, uint32_t value, uint64_t want)
{
    memcpy(s_crafted, s_file, s_size);
    maniStoreU32(s_crafted + offset, value);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniMirrorMap map;
    maniFileResult r = maniOpenMirrorMap(s_crafted, s_size, &map);
    if (r.offset != want)
    {
        printf("  at %llu, expected %llu\n", (unsigned long long)r.offset,
               (unsigned long long)want);
    }
    return r.result == mani_errorFormat && r.offset == want &&
           r.tag == TestTagAt(s_crafted, s_size, want);
}

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniMirrorMap mirrorMap;
    return maniOpenMirrorMap(bytes, size, &mirrorMap);
}

static void TestFile(void)
{
    maniMirrorMapSource source = Source();
    maniMirrorMap map;
    CHECK(Build(&source, &map).result == mani_success, "a map builds");
    uint64_t h = SectionAt(maniMirrorTagHeader);
    uint64_t bones = SectionAt(maniMirrorTagBones);
    uint64_t names = SectionAt(maniMirrorTagNames);
    CHECK(RefusedAt(h + 8, 0, h + 8), "no bones");
    CHECK(RefusedAt(h + 8, kBones + 1, h + 8), "a bone count the bones do not hold");
    CHECK(RefusedAt(h + 12, 3, h + 12), "an unknown axis");
    CHECK(RefusedAt(h + 16, 2, h + 16), "a name count the names do not hold");
    CHECK(RefusedAt(h + 20, kBones, h + 20), "an unmatched bone past the bones");
    uint64_t arm = bones + (uint64_t)kArmL * MANI_MIRROR_BONE_SIZE;
    CHECK(RefusedAt(arm, kHandR, arm), "a partner pairing back to another bone");
    CHECK(RefusedAt(arm, kBones, arm), "a partner past the bones");
    uint32_t flags = maniLoadU32(s_file + arm + 4);
    CHECK(RefusedAt(arm + 4, flags | 0x400u, arm + 4), "an unknown flag");
    CHECK(RefusedAt(arm + 4, MANI_MIRROR_FLAG_APPROXIMATE | 2u, arm + 4),
          "an approximate scale with a permutation");
    CHECK(RefusedAt(arm + 4, 4u, arm + 4), "two axes from one");
    CHECK(RefusedAt(arm + 4, 0x30u, arm + 4), "an axis past the three");
    CHECK(maniLoadU32(s_file + h + 20) == kPropR &&
              RefusedAt(arm + 4, flags | MANI_MIRROR_FLAG_UNMATCHED, arm + 4),
          "a bone unmatched before the header's first");
    CHECK(RefusedAt(arm + 8, 0x7F800000u, arm + 8), "a correction not finite");
    CHECK(RefusedAt(arm + 28, 0x7F800000u, arm + 28), "a later component not finite");
    CHECK(RefusedAt(arm + 24, 0x40000000u, arm + 24), "a correction not of length 1");
    uint64_t head = bones + (uint64_t)kHead * MANI_MIRROR_BONE_SIZE;
    CHECK(RefusedAt(h + 20, kHead, head + 4), "the header's first unmatched bone not flagged");
    CHECK(RefusedAt(arm + 8, maniLoadU32(s_file + arm + 8) ^ 0x80000000u, 40),
          "a fitting change against the identity");
    // The second name's hash made the first's.
    memcpy(s_crafted, s_file, s_size);
    memcpy(s_crafted + names + MANI_MIRROR_NAME_SIZE, s_file + names, 8);
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniMirrorMap repeated;
    maniFileResult order = maniOpenMirrorMap(s_crafted, s_size, &repeated);
    CHECK(order.result == mani_errorFormat && order.offset == names + MANI_MIRROR_NAME_SIZE,
          "names out of hash order");
    CHECK(RefusedAt(names + 20, 0, names + 16), "an empty partner name");
    CHECK(RefusedAt(names + 16, 1u << 20, names + 16), "a partner name past the names");
    CHECK(RefusedAt(names + 8, 0, names + 8), "a partner name of another hash");
    memcpy(s_crafted, s_file, s_size);
    s_crafted[s_size - 1] ^= 1u;
    maniStoreU64(s_crafted + 32,
                 maniHashBytes(MANI_HASH_INIT, s_crafted + MANI_CONTAINER_HEADER_SIZE,
                               s_size - MANI_CONTAINER_HEADER_SIZE));
    maniMirrorMap crafted;
    maniFileResult r = maniOpenMirrorMap(s_crafted, s_size, &crafted);
    CHECK(r.result == mani_errorFormat && (r.offset == 40 || r.offset > names),
          "a changed name refused");
    CHECK(maniOpenMirrorMap(s_file, s_size, NULL).result == mani_errorInvalid, "a NULL view");
    CHECK(TestOtherVersionsRefused(s_file, s_size, OpenFile), "another format version refused");
}

// The map of TestPairs written to path.
static bool Seed(const char* path)
{
    maniMirrorMapSource source = Source();
    maniMirrorMap map;
    if (Build(&source, &map).result != mani_success)
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
    Setup();
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return Seed(argv[2]) && s_failures == 0 ? 0 : 1;
    }
    TestPairs();
    TestRulePlaces();
    TestPairRefusals();
    TestOtherRefusals();
    TestFile();
    return s_failures == 0 ? 0 : 1;
}
