// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skeletons: building from bones in any order, opening the file, reading
// it, every refusal of the builder and of opening, and the bytes pinned.
// Run as test_skeleton --seed <file> it writes the fuzz target's seed.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "file_tag.h"
#include "skeleton_file.h"
#include "source_padding.h"
#include "test_budget.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PINNED_SKELETON_HASH 0x7cd1e2c993825645ull

enum
{
    kCapacity = 4096,
};

static _Alignas(16) uint8_t s_file[kCapacity];
static size_t s_size;

#define BONE(n, p, tx, qw)                                                                         \
    {                                                                                              \
        n, sizeof(n) - 1, p,                                                                       \
        {                                                                                          \
            {tx, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, qw}, {1.0f, 1.0f, 1.0f}                           \
        }                                                                                          \
    }

// Two roots; the source lists a child before its sibling's subtree.
static const maniBoneSource kBones[6] = {
    BONE("hips", -1, 1.0f, 1.0f),    BONE("leftLeg", 0, 2.0f, 1.0f), BONE("spine", 0, 3.0f, 2.0f),
    BONE("leftFoot", 1, 4.0f, 1.0f), BONE("head", 2, 5.0f, 1.0f),    BONE("prop", -1, 6.0f, 1.0f),
};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void BuildSample(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {kBones, 6};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds");
    CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success, "copies");
    maniDestroyBuilder(builder);
}

static maniFileResult Open(void)
{
    maniSkeleton skeleton;
    return maniOpenSkeleton(s_file, s_size, &skeleton);
}

static void TestBuildsDepthFirst(void)
{
    BuildSample();
    maniSkeleton s;
    CHECK(maniOpenSkeleton(s_file, s_size, &s).result == mani_success, "opens");
    CHECK(maniSkeleton_GetBoneCount(&s) == 6, "count");
    // hips, leftLeg, leftFoot, spine, head, prop.
    static const char* const names[6] = {"hips", "leftLeg", "leftFoot", "spine", "head", "prop"};
    static const uint32_t parents[6] = {MANI_NO_BONE, 0, 1, 0, 3, MANI_NO_BONE};
    static const uint32_t ends[6] = {5, 3, 3, 5, 5, 6};
    static const float tx[6] = {1.0f, 2.0f, 4.0f, 3.0f, 5.0f, 6.0f};
    for (uint32_t i = 0; i < 6; ++i)
    {
        size_t length = 0;
        const char* name = maniSkeleton_GetBoneName(&s, i, &length);
        CHECK(length == strlen(names[i]) && memcmp(name, names[i], length) == 0, "name");
        CHECK(maniSkeleton_GetParent(&s, i) == parents[i], "parent");
        CHECK(maniSkeleton_GetSubtreeEnd(&s, i) == ends[i], "subtree end");
        CHECK(maniSkeleton_GetRest(&s, i).translation.x == tx[i], "rest");
        CHECK(maniSkeleton_FindBone(&s, names[i], strlen(names[i])) == i, "find");
    }
    // spine's rotation (0, 0, 0, 2) is normalized.
    CHECK(maniSkeleton_GetRest(&s, 3).rotation.w == 1.0f, "normalized");
    CHECK(maniSkeleton_FindBone(&s, "tail", 4) == MANI_NO_BONE, "no such bone");
    CHECK(maniSkeleton_FindBone(&s, "hip", 3) == MANI_NO_BONE, "a prefix");
    CHECK(maniSkeleton_FindBone(&s, NULL, 0) == MANI_NO_BONE, "empty");
}

static void TestGettersPastTheCount(void)
{
    BuildSample();
    maniSkeleton s;
    CHECK(maniOpenSkeleton(s_file, s_size, &s).result == mani_success, "opens");
    CHECK(maniSkeleton_GetParent(&s, 6) == MANI_NO_BONE, "parent past");
    CHECK(maniSkeleton_GetSubtreeEnd(&s, 9) == 9, "end past");
    maniTransform rest = maniSkeleton_GetRest(&s, 6);
    CHECK(rest.rotation.w == 1.0f && rest.scale.y == 1.0f && rest.translation.z == 0.0f,
          "identity past");
    size_t length = 7;
    CHECK(maniSkeleton_GetBoneName(&s, 6, &length) == NULL && length == 0, "name past");
}

static void TestOrderMap(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {kBones, 6};
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds");
    uint32_t map[6] = {0};
    uint32_t count = 0;
    CHECK(maniBuilder_CopySkeletonOrder(builder, map, 6, &count) == mani_success && count == 6,
          "map");
    static const uint32_t expected[6] = {0, 1, 3, 2, 4, 5};
    CHECK(memcmp(map, expected, sizeof map) == 0, "where bones went");
    uint32_t partial[2] = {0};
    CHECK(maniBuilder_CopySkeletonOrder(builder, partial, 2, &count) == mani_errorCapacity &&
              count == 6,
          "short map");
    CHECK(partial[0] == 0 && partial[1] == 1, "short map filled");
    CHECK(maniBuilder_CopySkeletonOrder(builder, NULL, 0, &count) == mani_errorCapacity &&
              count == 6,
          "count only");
    size_t size = 0;
    CHECK(maniBuilder_CopyBuilt(builder, s_file, 10, &size) == mani_errorCapacity && size > 10,
          "short buffer");
    CHECK(maniBuilder_CopyBuilt(builder, NULL, 0, &size) == mani_errorCapacity, "size only");
    CHECK(maniBuilder_GetMisuse(builder) == 0, "no misuse");
    CHECK(maniBuilder_CopyBuilt(builder, NULL, 5, &size) == mani_errorInvalid, "null buffer");
    CHECK(maniBuilder_GetMisuse(builder) == 1, "counted");
    maniDestroyBuilder(builder);
}

static void TestDefRefusals(void)
{
    maniBuilder* builder = NULL;
    maniBuilderDef def = maniDefaultBuilderDef();
    CHECK(maniCreateBuilder(NULL, &builder) == mani_errorInvalid, "null def");
    CHECK(maniCreateBuilder(&def, NULL) == mani_errorInvalid, "null out");
    def.version += 1;
    CHECK(maniCreateBuilder(&def, &builder) == mani_errorVersion && builder == NULL, "version");
    def = maniDefaultBuilderDef();
    def.cookie = 0;
    CHECK(maniCreateBuilder(&def, &builder) == mani_errorInvalid, "cookie");
    memset(&def, 0, sizeof def);
    CHECK(maniCreateBuilder(&def, &builder) == mani_errorInvalid && builder == NULL,
          "a zeroed def refused by its cookie, before its version");
    CHECK((MANI_ABI_VERSION & UINT32_C(0x80000000)) != 0, "the stamp's top bit set");
    def = maniDefaultBuilderDef();
    def.allocator.context = &def;
    def.allocator.free = NULL;
    def.allocator.alloc = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "context alone is zeroed");
    maniDestroyBuilder(builder);
    maniDestroyBuilder(NULL);
}

static maniBuildResult BuildOf(const maniBoneSource* bones, uint32_t count)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, count};
    maniBuildResult r = maniBuildSkeleton(builder, &source);
    maniDestroyBuilder(builder);
    return r;
}

static void ExpectRefusal(const maniBoneSource* bones, uint32_t count, maniBuildReason reason,
                          uint32_t item, uint32_t other, const char* what)
{
    maniBuildResult r = BuildOf(bones, count);
    CHECK(r.result == mani_errorInvalid, what);
    CHECK(r.reason == reason && r.item == item && r.other == other, what);
}

static void TestBuildRefusals(void)
{
    maniBoneSource b[3] = {kBones[0], kBones[1], kBones[2]};
    ExpectRefusal(b, 0, maniBuildReasonCount, 0, 0, "no bones");
    maniBuildResult past = BuildOf(b, MANI_MAX_BONES + 1);
    CHECK(past.result == mani_errorLimit && past.reason == maniBuildReasonCount,
          "bones past MANI_MAX_BONES at the limit");
    b[2].parent = 3;
    ExpectRefusal(b, 3, maniBuildReasonParent, 2, 0, "parent past");
    b[2].parent = -2;
    ExpectRefusal(b, 3, maniBuildReasonParent, 2, 0, "parent below -1");
    b[2].parent = 2;
    ExpectRefusal(b, 3, maniBuildReasonParent, 2, 0, "own parent");
    b[1].parent = 2;
    b[2].parent = 1;
    ExpectRefusal(b, 3, maniBuildReasonCycle, 1, 0, "cycle");
    b[1] = kBones[1];
    b[2] = kBones[2];
    b[2].nameLength = 0;
    ExpectRefusal(b, 3, maniBuildReasonName, 2, 0, "empty name");
    b[2].name = NULL;
    b[2].nameLength = 3;
    ExpectRefusal(b, 3, maniBuildReasonName, 2, 0, "null name");
    b[2].name = "a\xC0\xAF";
    ExpectRefusal(b, 3, maniBuildReasonName, 2, 0, "overlong UTF-8");
    // Refused at the length, before a byte of the name is read.
    b[2].nameLength = (size_t)MANI_MAX_NAME_BYTES + 1;
    maniBuildResult longName = BuildOf(b, 3);
    CHECK(longName.result == mani_errorLimit && longName.reason == maniBuildReasonName &&
              longName.item == 2 && longName.other == 0,
          "a name past MANI_MAX_NAME_BYTES");
    b[2] = kBones[2];
    b[2].name = "hips";
    b[2].nameLength = 4;
    ExpectRefusal(b, 3, maniBuildReasonDuplicateName, 2, 0, "duplicate");
    b[2] = kBones[2];
    b[2].rest.scale.y = INFINITY;
    ExpectRefusal(b, 3, maniBuildReasonNotFinite, 2, 0, "infinite scale");
    b[2] = kBones[2];
    b[2].rest.translation.z = NAN;
    ExpectRefusal(b, 3, maniBuildReasonNotFinite, 2, 0, "NaN translation");
    b[2] = kBones[2];
    b[2].rest.rotation = (maniQuat){0.0f, 0.0f, 0.0f, 0.0f};
    ExpectRefusal(b, 3, maniBuildReasonRotation, 2, 0, "zero rotation");

    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildSkeleton(builder, NULL).result == mani_errorInvalid, "null source");
    CHECK(maniBuilder_GetMisuse(builder) == 1, "a misuse");
    CHECK(maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, NULL) == mani_errorInvalid,
          "nothing built");
    CHECK(maniBuilder_CopySkeletonOrder(builder, NULL, 0, NULL) == mani_errorInvalid,
          "no skeleton");
    maniDestroyBuilder(builder);
    CHECK(maniBuildSkeleton(NULL, NULL).result == mani_errorInvalid, "null builder");
}

static TestOutcome BuildWork(maniBuilder* builder, void* context)
{
    (void)context;
    maniSkeletonSource source = {kBones, 6};
    return TestOutcomeOf(maniBuildSkeleton(builder, &source).result);
}

// Every allocation failing in turn while creating a builder and building.
static void TestAllocationFailures(void)
{
    CHECK(TestSweep(BuildWork, NULL, 64), "allocation failures handled, nothing kept");
}

// Restores the payload hash after a test changes a section.
static void Rehash(void)
{
    maniStoreU64(s_file + 32, maniHashBytes(MANI_HASH_INIT, s_file + MANI_CONTAINER_HEADER_SIZE,
                                            s_size - MANI_CONTAINER_HEADER_SIZE));
}

static uint64_t SectionAt(uint32_t tag)
{
    return maniLoadU64(s_file + MANI_CONTAINER_HEADER_SIZE + (uint64_t)(tag - 1) * 24 + 8);
}

static void ExpectFormat(uint64_t offset, const char* what)
{
    Rehash();
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat, what);
    CHECK(r.offset == offset && r.tag == TestTagAt(s_file, s_size, offset), what);
    BuildSample();
}

// Rewrites the sample's container from its sections, with section tag
// replaced by size bytes of data (or dropped for a NULL data), keeping
// the identity: the table's own checks are reached.
static void Rewrite(uint32_t tag, const void* data, uint64_t size)
{
    static _Alignas(16) uint8_t copy[kCapacity];
    memcpy(copy, s_file, s_size);
    maniSectionSource sections[MANI_SKELETON_SECTIONS];
    uint32_t count = 0;
    for (uint32_t t = 1; t <= MANI_SKELETON_SECTIONS; ++t)
    {
        uint64_t at = MANI_CONTAINER_HEADER_SIZE + (uint64_t)(t - 1) * 24;
        const uint8_t* bytes = copy + maniLoadU64(copy + at + 8);
        uint64_t length = maniLoadU64(copy + at + 16);
        if (t == tag && data == NULL)
        {
            continue;
        }
        sections[count++] =
            t == tag ? (maniSectionSource){t, data, size} : (maniSectionSource){t, bytes, length};
    }
    uint64_t total = 0;
    CHECK(maniContainerSize(sections, count, &total) && total <= sizeof s_file, "rewrite fits");
    maniWriteContainer(s_file, maniKindSkeleton, MANI_SKELETON_FORMAT_VERSION,
                       maniLoadU64(copy + 40), sections, count);
    s_size = (size_t)total;
}

static void ExpectRewriteRefused(uint64_t offset, const char* what)
{
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == offset, what);
    BuildSample();
}

static void TestTableRefusals(void)
{
    BuildSample();
    Rewrite(maniSkeletonTagNameOrder, NULL, 0);
    ExpectRewriteRefused(12, "six sections");
    static const uint8_t odd[3] = {0xFF, 0xFF, 0};
    Rewrite(maniSkeletonTagParents, odd, 3);
    ExpectRewriteRefused(MANI_CONTAINER_HEADER_SIZE + 16, "parents of odd size");
    static const uint8_t none[1] = {0};
    Rewrite(maniSkeletonTagParents, none, 0);
    ExpectRewriteRefused(MANI_CONTAINER_HEADER_SIZE + 16, "no bones");
    static const uint8_t rests[40 * 5] = {0};
    Rewrite(maniSkeletonTagRests, rests, sizeof rests);
    ExpectRewriteRefused(MANI_CONTAINER_HEADER_SIZE + 2 * 24 + 16, "rests of the wrong size");
    // The names with one byte more than their spans cover.
    static uint8_t names[64];
    uint64_t at = MANI_CONTAINER_HEADER_SIZE + 6 * 24;
    uint64_t length = maniLoadU64(s_file + at + 16);
    memcpy(names, s_file + maniLoadU64(s_file + at + 8), (size_t)length);
    names[length] = 'x';
    Rewrite(maniSkeletonTagNameBytes, names, length + 1);
    ExpectRewriteRefused(at + 16, "trailing name bytes");
}

static maniFileResult OpenFile(const void* bytes, size_t size)
{
    maniSkeleton skeleton;
    return maniOpenSkeleton(bytes, size, &skeleton);
}

static void TestOpenRefusals(void)
{
    BuildSample();
    uint64_t parents = SectionAt(maniSkeletonTagParents);
    uint64_t ends = SectionAt(maniSkeletonTagSubtreeEnds);
    uint64_t rests = SectionAt(maniSkeletonTagRests);
    uint64_t hashes = SectionAt(maniSkeletonTagNameHashes);
    uint64_t order = SectionAt(maniSkeletonTagNameOrder);
    uint64_t spans = SectionAt(maniSkeletonTagNameSpans);
    uint64_t names = SectionAt(maniSkeletonTagNameBytes);

    // A parent at or after its child.
    maniStoreU16(s_file + parents + 2 * 2, 2);
    ExpectFormat(parents + 4, "parent not earlier");
    // Not depth-first: prop (5) under leftFoot (2), whose subtree closed
    // at 3; the walk closes head, spine and hips correctly and runs out.
    maniStoreU16(s_file + parents + 2 * 5, 2);
    ExpectFormat(parents + 10, "not depth-first");
    // The first bone with a parent.
    maniStoreU16(s_file + parents, 0);
    ExpectFormat(parents, "first bone's parent");
    // A wrong subtree end, closed by the walk.
    maniStoreU16(s_file + ends + 2 * 1, 4);
    ExpectFormat(ends + 2, "subtree end");
    // A wrong end closed at the file's end.
    maniStoreU16(s_file + ends + 2 * 5, 5);
    ExpectFormat(ends + 10, "last subtree end");

    float nan = NAN;
    memcpy(s_file + rests + 40 * 2 + 4, &nan, 4);
    ExpectFormat(rests + 84, "NaN rest");
    float off = 1.25f;
    memcpy(s_file + rests + 40 * 3 + 24, &off, 4);
    ExpectFormat(rests + 132, "rotation not unit");
    // Within the tolerance of 2^-20 in squared length, a rotation opens.
    float near = 1.0f + 0x1p-23f;
    memcpy(s_file + rests + 40 * 3 + 24, &near, 4);
    Rehash();
    CHECK(Open().result == mani_errorFormat && Open().offset == 40, "near unit is no refusal");
    BuildSample();

    maniStoreU32(s_file + spans + 8, 5);
    ExpectFormat(spans + 8, "span not contiguous");
    maniStoreU32(s_file + spans + 4, 0);
    ExpectFormat(spans + 4, "empty name");
    maniStoreU32(s_file + spans + 8 * 5 + 4, 999);
    ExpectFormat(spans + 44, "span past the bytes");
    s_file[names + 1] = 0xFF;
    ExpectFormat(names, "invalid UTF-8");
    s_file[hashes + 8] ^= 1;
    ExpectFormat(hashes + 8, "hash not the name's");
    maniStoreU16(s_file + order + 2, 6);
    ExpectFormat(order + 2, "order past the count");
    maniStoreU16(s_file + order + 2, 0xFFFF);
    ExpectFormat(order + 2, "order far past the count");
    uint16_t first = maniLoadU16(s_file + order);
    maniStoreU16(s_file + order + 2, first);
    ExpectFormat(order + 2, "order repeats");

    // The identity is in the header, outside the payload's hash.
    s_file[44] ^= 1;
    maniFileResult r = Open();
    CHECK(r.result == mani_errorFormat && r.offset == 40, "identity");
    BuildSample();

    maniSkeleton s;
    CHECK(maniOpenSkeleton(s_file, s_size, NULL).result == mani_errorInvalid, "null view");
    CHECK(TestOtherVersionsRefused(s_file, s_size, OpenFile), "another format version refused");
    CHECK(maniOpenSkeleton(s_file + 16, s_size, &s).result == mani_errorFormat, "not a file");
}

// Every byte but the writer's version is covered by a check or a hash.
static void TestEveryByteFlip(void)
{
    BuildSample();
    for (size_t i = 0; i < s_size; ++i)
    {
        bool isFree = i >= 16 && i < 22;
        s_file[i] ^= 0x20;
        bool opened = Open().result == mani_success;
        s_file[i] ^= 0x20;
        CHECK(opened == isFree, "byte flip");
    }
}

static void TestPinnedBytes(void)
{
    BuildSample();
    maniSkeleton s;
    CHECK(maniOpenSkeleton(s_file, s_size, &s).result == mani_success, "opens");
    memset(s_file + 16, 0, 6);
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    // The identity is in the header (bytes 40 to 47), so the file's hash
    // pins it too.
    CHECK(hash == PINNED_SKELETON_HASH, "pinned file");
    if (hash != PINNED_SKELETON_HASH)
    {
        printf("skeleton hash 0x%016llx\n", (unsigned long long)hash);
    }
}

// Writes the sample skeleton file, the fuzz target's seed.
static int WriteSeed(const char* path)
{
    BuildSample();
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(s_file, 1, s_size, out);
    return fclose(out) == 0 && written == s_size ? 0 : 1;
}

// The same file whatever bytes pad the source and its bones.
static void TestPaddingIgnored(void)
{
    uint8_t* built[2];
    size_t sizes[2];
    for (int k = 0; k < 2; ++k)
    {
        maniBoneSource bones[6];
        memcpy(bones, kBones, sizeof bones);
        TEST_PAD(bones, 6, kFieldsBoneSource, kTestPadPatterns[k]);
        maniSkeletonSource source = {bones, 6};
        TEST_PAD(&source, 1, kFieldsSkeletonSource, kTestPadPatterns[k]);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        CHECK(maniBuildSkeleton(builder, &source).result == mani_success, "builds padded");
        built[k] = TestCopyBuilt(builder, &sizes[k]);
        maniDestroyBuilder(builder);
    }
    CHECK(TestSameBuilt(built, sizes), "the same bytes under either padding");
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestBuildsDepthFirst();
    TestGettersPastTheCount();
    TestOrderMap();
    TestDefRefusals();
    TestBuildRefusals();
    TestAllocationFailures();
    TestTableRefusals();
    TestOpenRefusals();
    TestEveryByteFlip();
    TestPinnedBytes();
    TestPaddingIgnored();
    return s_failures == 0 ? 0 : 1;
}
