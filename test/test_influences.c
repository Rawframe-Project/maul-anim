// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Influences: built from any number of influences a vertex,
// sorted, renormalized and padded; every refusal of the builder; opening
// refusing crafted files, offsets reported; binding to the skin; the
// bytes pinned. Run as test_influences --seed <file> it writes the
// sample file for the fuzz corpus.

// MSVC's CRT deprecates fopen; the seed writer uses it.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "influence_file.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

enum
{
    kBones = 6,
    kJoints = 5,
};

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_skinFile[4096];
static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;
static maniSkeleton s_skeleton;
static maniSkin s_skin;

#define BONE(n, p)                                                                                 \
    {                                                                                              \
        n, sizeof(n) - 1, p,                                                                       \
        {                                                                                          \
            {0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}                       \
        }                                                                                          \
    }

static const maniBoneSource kSkeletonBones[kBones] = {BONE("root", -1), BONE("a", 0), BONE("b", 1),
                                                      BONE("c", 2),     BONE("d", 0), BONE("e", 4)};

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

static void OpenSkin(void)
{
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {kSkeletonBones, kBones, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success,
          "skeleton");
    CHECK(maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success, "opens");
    const uint32_t joints[kJoints] = {1, 2, 3, 4, 5};
    maniAffine binds[kJoints];
    for (int j = 0; j < kJoints; ++j)
    {
        binds[j] = (maniAffine){{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, -0.5f * (float)j, 0}};
    }
    maniSkinSource skin = {&s_skeleton, joints, binds, kJoints, 0};
    CHECK(maniBuildSkin(builder, &skin).result == mani_success &&
              maniCopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) == mani_success,
          "skin");
    CHECK(maniOpenSkin(s_skinFile, size, &s_skin).result == mani_success, "skin opens");
    maniDestroyBuilder(builder);
}

static maniBuildResult BuildFrom(const maniInfluenceSource* source)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildInfluences(builder, source);
    if (r.result == mani_success)
    {
        CHECK(maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success, "copies");
    }
    maniDestroyBuilder(builder);
    return r;
}

// Four vertices: five weighted influences unsorted, with a zero and a
// joint repeated at weight 0; one; none weighted; two equal weights.
static const uint32_t kOffsets[5] = {0, 7, 8, 10, 12};
static const uint32_t kJointsIn[12] = {3, 0, 4, 2, 1, 0, 2, 4, 1, 2, 3, 1};
static const float kWeightsIn[12] = {0.1f, 0.4f, 0.2f, 0.05f, 0.6f, 0.0f,
                                     0.0f, 2.0f, 0.0f, 0.0f,  0.5f, 0.5f};
static const float kBlend[4] = {0.0f, 1.0f, 0.25f, 0.5f};

static maniInfluenceSource Sample(void)
{
    return (maniInfluenceSource){
        .skin = &s_skin,
        .offsets = kOffsets,
        .joints = kJointsIn,
        .weights = kWeightsIn,
        .dualQuaternionWeights = kBlend,
        .vertexCount = 4,
    };
}

static void BuildSample(void)
{
    maniInfluenceSource source = Sample();
    CHECK(BuildFrom(&source).result == mani_success, "builds");
}

static void TestBuildsAndReads(void)
{
    BuildSample();
    maniInfluences v;
    CHECK(maniOpenInfluences(s_file, s_size, &v).result == mani_success, "opens");
    CHECK(maniInfluencesVertexCount(&v) == 4 && maniInfluencesPerVertex(&v) == 8,
          "four vertices, two groups");
    CHECK(maniInfluencesFitSkin(&v, &s_skin), "bound to its skin");
    // Vertex 0: 0.6 (joint 1), 0.4 (0), 0.2 (4), 0.1 (3), 0.05 (2).
    const uint32_t joints[5] = {1, 0, 4, 3, 2};
    const float weights[5] = {0.6f, 0.4f, 0.2f, 0.1f, 0.05f};
    double total = 0.0;
    for (int k = 0; k < 5; ++k)
    {
        total += (double)weights[k];
    }
    bool sorted = true;
    double sum = 0.0;
    for (uint32_t k = 0; k < 8; ++k)
    {
        float w = 0.0f;
        uint32_t j = maniInfluence(&v, 0, k, &w);
        double want = k < 5 ? (double)weights[k] / total : 0.0;
        sorted = sorted && j == (k < 5 ? joints[k] : 0) && w == (float)want;
        sum += (double)w;
    }
    CHECK(sorted, "sorted by weight, renormalized, padded with joint 0 at 0");
    CHECK(fabs(sum - 1.0) < 1e-6, "summing to 1");
    float w = 0.0f;
    CHECK(maniInfluence(&v, 1, 0, &w) == 4 && w == 1.0f, "one influence: weight 1");
    CHECK(maniInfluence(&v, 2, 0, &w) == 0 && w == 0.0f, "none weighted: all 0");
    CHECK(maniInfluence(&v, 3, 0, &w) == 1 && w == 0.5f && maniInfluence(&v, 3, 1, NULL) == 3,
          "equal weights: the lower joint first");
    CHECK(maniInfluence(&v, 4, 0, &w) == 0 && w == 0.0f && maniInfluence(&v, 0, 8, &w) == 0 &&
              w == 0.0f,
          "past the vertices or slots: joint 0 at 0");
    CHECK(maniInfluencesDualQuaternionWeight(&v, 2) == 0.25f &&
              maniInfluencesDualQuaternionWeight(&v, 4) == 0.0f &&
              maniInfluencesDualQuaternionWeight(&v, UINT32_MAX) == 0.0f,
          "blend weights; 0 past the vertices");
    maniVec3 center;
    CHECK(!maniInfluencesCenter(&v, 0, &center), "no centers yet");
    maniInfluenceSource plain = Sample();
    plain.dualQuaternionWeights = NULL;
    plain.vertexCount = 2;
    CHECK(BuildFrom(&plain).result == mani_success, "one group");
    CHECK(maniOpenInfluences(s_file, s_size, &v).result == mani_success &&
              maniInfluencesPerVertex(&v) == 8 && maniInfluencesDualQuaternionWeight(&v, 1) == 0.0f,
          "no blend weights: 0");
    // Another skin: a skin of the same joints and other binds.
    maniSkin other = s_skin;
    other.identity ^= 1;
    CHECK(!maniInfluencesFitSkin(&v, &other), "not another skin");
}

// A vertex of exactly four influences takes one group, not two.
static void TestWholeGroup(void)
{
    static const uint32_t offsets[2] = {0, 4};
    static const uint32_t joints[4] = {0, 1, 2, 3};
    static const float weights[4] = {0.4f, 0.3f, 0.2f, 0.1f};
    maniInfluenceSource source = {.skin = &s_skin,
                                  .offsets = offsets,
                                  .joints = joints,
                                  .weights = weights,
                                  .vertexCount = 1};
    maniInfluences v;
    CHECK(BuildFrom(&source).result == mani_success &&
              maniOpenInfluences(s_file, s_size, &v).result == mani_success &&
              maniInfluencesPerVertex(&v) == 4,
          "four influences: one group");
}

static void ExpectRefused(const maniInfluenceSource* source, maniBuildReason reason, uint32_t item,
                          uint32_t other, const char* what)
{
    maniBuildResult r = BuildFrom(source);
    CHECK(r.result == mani_errorInvalid && r.reason == reason && r.item == item && r.other == other,
          what);
}

static void TestBuildRefusals(void)
{
    uint32_t offsets[5];
    uint32_t joints[12];
    float weights[12];
    float blend[4];
    memcpy(offsets, kOffsets, sizeof offsets);
    memcpy(joints, kJointsIn, sizeof joints);
    memcpy(weights, kWeightsIn, sizeof weights);
    memcpy(blend, kBlend, sizeof blend);
    maniInfluenceSource s = Sample();
    s.offsets = offsets;
    s.joints = joints;
    s.weights = weights;
    s.dualQuaternionWeights = blend;
    weights[2] = -0.1f;
    ExpectRefused(&s, maniBuildReasonWeight, 0, 2, "a negative weight");
    weights[2] = NAN;
    ExpectRefused(&s, maniBuildReasonNotFinite, 0, 2, "a NaN weight");
    weights[2] = INFINITY;
    ExpectRefused(&s, maniBuildReasonNotFinite, 0, 2, "an infinite weight");
    weights[2] = 0.2f;
    joints[7] = kJoints;
    ExpectRefused(&s, maniBuildReasonJoint, 1, 7, "a joint past the skin");
    joints[7] = 4;
    joints[4] = 0;
    ExpectRefused(&s, maniBuildReasonRepeatedJoint, 0, 1, "a joint weighted twice");
    joints[4] = 1;
    blend[3] = 1.5f;
    ExpectRefused(&s, maniBuildReasonWeight, 3, 0, "a blend past 1");
    blend[3] = NAN;
    ExpectRefused(&s, maniBuildReasonWeight, 3, 0, "a NaN blend");
    blend[3] = 0.5f;
    offsets[2] = 6;
    ExpectRefused(&s, maniBuildReasonCount, 1, 0, "offsets not increasing");
    offsets[2] = 8;
    offsets[0] = 1;
    ExpectRefused(&s, maniBuildReasonCount, 0, 0, "offsets not from 0");
    offsets[0] = 0;
    s.vertexCount = 0;
    ExpectRefused(&s, maniBuildReasonCount, 0, 0, "no vertices");
    s.vertexCount = 4;
    // 65 weighted influences on one vertex: one more than 16 groups.
    static uint32_t manyOffsets[2] = {0, 65};
    static uint32_t manyJoints[65];
    static float manyWeights[65];
    static maniSkin wide;
    wide = s_skin;
    wide.jointCount = 70;
    for (uint32_t i = 0; i < 65; ++i)
    {
        manyJoints[i] = i;
        manyWeights[i] = 1.0f;
    }
    maniInfluenceSource many = {&wide, manyOffsets, manyJoints, manyWeights, NULL, 1, 0};
    ExpectRefused(&many, maniBuildReasonCount, 0, 64, "65 weighted influences");
    manyOffsets[1] = 64;
    CHECK(BuildFrom(&many).result == mani_success, "64: sixteen groups");
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildInfluences(builder, NULL).result == mani_errorInvalid, "no source");
    s.skin = NULL;
    CHECK(maniBuildInfluences(builder, &s).result == mani_errorInvalid, "no skin");
    CHECK(maniBuildInfluences(NULL, &s).result == mani_errorInvalid, "no builder");
    maniDestroyBuilder(builder);
}

// The sample rebuilt with one section's bytes replaced (or added), the
// identity and payload hash made right.
static void Craft(uint32_t tag, const void* bytes, uint64_t size)
{
    maniContainer c;
    CHECK(maniOpenContainer(s_file, s_size, maniKindInfluences, MANI_INFLUENCE_FORMAT_VERSION, &c)
                  .result == mani_success,
          "container");
    const uint8_t* data[MANI_INFLUENCE_SECTIONS];
    uint64_t sizes[MANI_INFLUENCE_SECTIONS];
    maniSectionSource sections[MANI_INFLUENCE_SECTIONS];
    uint32_t count = 0;
    static const uint8_t kNone = 0;
    for (uint32_t t = 1; t <= MANI_INFLUENCE_SECTIONS; ++t)
    {
        uint64_t at = 0;
        data[t - 1] = &kNone;
        sizes[t - 1] = 0;
        (void)maniContainerSection(&c, t, &data[t - 1], &sizes[t - 1], &at);
        if (t == tag)
        {
            data[t - 1] = bytes;
            sizes[t - 1] = size;
        }
        if (sizes[t - 1] > 0 || t == tag)
        {
            sections[count++] = (maniSectionSource){t, data[t - 1], sizes[t - 1]};
        }
    }
    uint64_t fileSize = 0;
    CHECK(maniContainerSize(sections, count, &fileSize) && fileSize <= sizeof s_crafted, "fits");
    maniWriteContainer(s_crafted, maniKindInfluences, MANI_INFLUENCE_FORMAT_VERSION,
                       maniInfluenceComputeIdentity(data, sizes), sections, count);
    s_size = (size_t)fileSize;
    memcpy(s_file, s_crafted, s_size);
}

// The section's offset in s_file.
static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindInfluences, MANI_INFLUENCE_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "section");
    return at;
}

static void ExpectFormat(uint64_t offset, const char* what)
{
    maniInfluences v;
    maniFileResult r = maniOpenInfluences(s_file, s_size, &v);
    CHECK(r.result == mani_errorFormat && r.offset == offset, what);
    if (r.offset != offset)
    {
        printf("  %s: at %llu, expected %llu\n", what, (unsigned long long)r.offset,
               (unsigned long long)offset);
    }
}

static void TestOpenRefusals(void)
{
    BuildSample();
    uint8_t binding[MANI_INFLUENCE_BINDING_SIZE];
    memcpy(binding, s_file + SectionAt(maniInfluenceTagBinding), sizeof binding);
    const uint32_t fields[4][2] = {{8, 0}, {12, 0}, {16, 0}, {16, 17}};
    const char* names[4] = {"no joints", "no vertices", "no groups", "17 groups"};
    for (int i = 0; i < 4; ++i)
    {
        uint8_t b[MANI_INFLUENCE_BINDING_SIZE];
        memcpy(b, binding, sizeof b);
        maniStoreU32(b + fields[i][0], fields[i][1]);
        Craft(maniInfluenceTagBinding, b, sizeof b);
        ExpectFormat(SectionAt(maniInfluenceTagBinding) + fields[i][0], names[i]);
        BuildSample();
    }
    uint8_t b[MANI_INFLUENCE_BINDING_SIZE];
    memcpy(b, binding, sizeof b);
    maniStoreU32(b + 20, 1);
    Craft(maniInfluenceTagBinding, b, sizeof b);
    ExpectFormat(SectionAt(maniInfluenceTagBinding) + 20, "the binding's zero");
    BuildSample();
    // Values: a joint past the skin, weights past 1 and NaN, a blend past
    // 1, a center not finite.
    {
        uint8_t copy[2 * 32];
        memcpy(copy, s_file + SectionAt(maniInfluenceTagJoints), sizeof copy);
        maniStoreU16(copy + 2 * 9, kJoints);
        Craft(maniInfluenceTagJoints, copy, sizeof copy);
    }
    ExpectFormat(SectionAt(maniInfluenceTagJoints) + 2 * 9, "a joint past the skin");
    BuildSample();
    const float wrong[2] = {1.5f, NAN};
    for (int i = 0; i < 2; ++i)
    {
        uint8_t copy[4 * 32];
        memcpy(copy, s_file + SectionAt(maniInfluenceTagWeights), sizeof copy);
        memcpy(copy + 4 * 5, &wrong[i], 4);
        Craft(maniInfluenceTagWeights, copy, sizeof copy);
        ExpectFormat(SectionAt(maniInfluenceTagWeights) + 4 * 5, "a weight outside 0 to 1");
        BuildSample();
        uint8_t blend[16];
        memcpy(blend, kBlend, sizeof blend);
        memcpy(blend + 8, &wrong[i], 4);
        Craft(maniInfluenceTagDualQuaternionWeights, blend, sizeof blend);
        ExpectFormat(SectionAt(maniInfluenceTagDualQuaternionWeights) + 8,
                     "a blend outside 0 to 1");
        BuildSample();
    }
    float centers[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    Craft(maniInfluenceTagCenters, centers, sizeof centers);
    maniInfluences v;
    maniVec3 center;
    CHECK(maniOpenInfluences(s_file, s_size, &v).result == mani_success &&
              maniInfluencesCenter(&v, 2, &center) && center.y == 2.0f &&
              !maniInfluencesCenter(&v, 4, &center),
          "centers read");
    centers[7] = INFINITY;
    Craft(maniInfluenceTagCenters, centers, sizeof centers);
    ExpectFormat(SectionAt(maniInfluenceTagCenters) + 4 * 7, "a center not finite");
    BuildSample();
    // Sizes: a short weights section, a short blend section, an empty
    // section present.
    uint8_t copy[4 * 32];
    memcpy(copy, s_file + SectionAt(maniInfluenceTagWeights), sizeof copy);
    Craft(maniInfluenceTagWeights, copy, sizeof copy - 4);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 2 * MANI_CONTAINER_ENTRY_SIZE + 16,
                 "weights a slot short");
    BuildSample();
    Craft(maniInfluenceTagDualQuaternionWeights, kBlend, 12);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 3 * MANI_CONTAINER_ENTRY_SIZE + 16,
                 "blends a vertex short");
    BuildSample();
    Craft(maniInfluenceTagCenters, centers, 0);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 4 * MANI_CONTAINER_ENTRY_SIZE + 16,
                 "an empty section present");
    BuildSample();
    s_file[40] ^= 1;
    ExpectFormat(40, "the identity");
    maniInfluences none;
    CHECK(maniOpenInfluences(s_file, s_size, NULL).result == mani_errorInvalid, "no view");
    CHECK(maniOpenInfluences(NULL, 64, &none).result == mani_errorInvalid, "no bytes");
}

static int WriteSeed(const char* path)
{
    OpenSkin();
    BuildSample();
    FILE* out = fopen(path, "wb");
    if (out == NULL)
    {
        return 1;
    }
    size_t written = fwrite(s_file, 1, s_size, out);
    return fclose(out) == 0 && written == s_size ? 0 : 1;
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    OpenSkin();
    TestBuildsAndReads();
    TestWholeGroup();
    TestBuildRefusals();
    TestOpenRefusals();
    return s_failures == 0 ? 0 : 1;
}
