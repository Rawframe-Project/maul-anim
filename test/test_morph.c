// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Morph targets (mani-0021): built with sparse targets sorted and their
// repeated vertices summed, a target touching every vertex stored
// densely, FBX in-betweens; channel weights walked to target weights as
// ufbx does, against a piecewise linear reference; the job against a
// binary64 reference, ranges, in place and sparse against dense giving
// the same bits; every refusal of the builder, the opener and the job;
// the bytes pinned. Run as test_morph --seed <file> it writes the sample
// file for the fuzz corpus.

// MSVC's CRT deprecates fopen; the seed writer uses it.
#define _CRT_SECURE_NO_WARNINGS

#include "bytes.h"
#include "container.h"
#include "morph_file.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/morph.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PINNED_MORPH_FILE 0x2199e0ea0414cb8cull
#define PINNED_MORPH_JOB  0x8f6d7cfcab3a9593ull

enum
{
    kVertices = 12,
};

static _Alignas(16) uint8_t s_file[16384];
static _Alignas(16) uint8_t s_crafted[16384];
static size_t s_size;

static uint64_t s_random = 0x2545F4914F6CDD1Dull;

static uint32_t Next(void)
{
    s_random ^= s_random << 13;
    s_random ^= s_random >> 7;
    s_random ^= s_random << 17;
    return (uint32_t)(s_random >> 32);
}

static float Uniform(float low, float high)
{
    return low + (high - low) * ((float)(Next() >> 8) / 16777216.0f);
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// The sample: "smile", one sparse target given out of order with vertex
// 5 twice; "blink", FBX in-betweens at -1, 0.5 and 1, dense; "wide", one
// target given for every vertex in reverse, so stored densely.
static const uint32_t kSmileIndices[4] = {5, 2, 5, 9};
static const float kSmilePositions[12] = {1, 0, 0, 0, 2, 0, 0.5f, 0.25f, 0, 0, 0, 3};
static const float kSmileNormals[12] = {0, 1, 0, 1, 0, 0, 0, 0.5f, 0, 0, 0, 1};
static float s_blink[3][3 * kVertices];
static uint32_t s_wideIndices[kVertices];
static float s_wideTangents[3 * kVertices];

static maniMorphTargetSource s_smile[1];
static maniMorphTargetSource s_blinks[3];
static maniMorphTargetSource s_wide[1];
static maniMorphChannelSource s_channels[3];

static void SampleSource(maniMorphSource* source)
{
    for (int k = 0; k < 3; ++k)
    {
        for (uint32_t i = 0; i < 3 * kVertices; ++i)
        {
            s_blink[k][i] = (float)(k + 1) * 0.125f * (float)((i % 7) + 1);
        }
    }
    for (uint32_t i = 0; i < kVertices; ++i)
    {
        s_wideIndices[i] = kVertices - 1 - i;
        s_wideTangents[3 * i] = 0.25f * (float)i;
        s_wideTangents[3 * i + 1] = -0.5f;
        s_wideTangents[3 * i + 2] = 0.125f;
    }
    s_smile[0] =
        (maniMorphTargetSource){1.0f, 4, kSmileIndices, kSmilePositions, kSmileNormals, NULL};
    s_blinks[0] = (maniMorphTargetSource){-1.0f, kVertices, NULL, s_blink[0], NULL, NULL};
    s_blinks[1] = (maniMorphTargetSource){0.5f, kVertices, NULL, s_blink[1], NULL, NULL};
    s_blinks[2] = (maniMorphTargetSource){1.0f, kVertices, NULL, s_blink[2], NULL, NULL};
    s_wide[0] = (maniMorphTargetSource){1.0f, kVertices, s_wideIndices, NULL, NULL, s_wideTangents};
    s_channels[0] = (maniMorphChannelSource){"smile", 5, 1, s_smile};
    s_channels[1] = (maniMorphChannelSource){"blink", 5, 3, s_blinks};
    s_channels[2] = (maniMorphChannelSource){"wide", 4, 1, s_wide};
    *source = (maniMorphSource){s_channels, 3, kVertices};
}

static void BuildSample(void)
{
    maniMorphSource source;
    SampleSource(&source);
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildMorphTargets(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success,
          "sample built");
    maniDestroyBuilder(builder);
}

static void TestBuildsAndReads(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    CHECK(maniMorphVertexCount(&m) == kVertices && maniMorphChannelCount(&m) == 3 &&
              maniMorphTargetCount(&m) == 5,
          "counts");
    uint32_t length = 0;
    const char* name = maniMorphChannelName(&m, 1, &length);
    CHECK(length == 5 && memcmp(name, "blink", 5) == 0, "a channel's name");
    CHECK(maniMorphChannelName(&m, 3, &length) == NULL && length == 0, "no channel past the count");
    CHECK(maniMorphFindChannel(&m, "wide", 4) == 2 &&
              maniMorphFindChannel(&m, "wid", 3) == MANI_NO_CHANNEL &&
              maniMorphFindChannel(&m, NULL, 1) == MANI_NO_CHANNEL,
          "find");
    uint32_t first = 0;
    CHECK(maniMorphChannelTargets(&m, 1, &first) == 3 && first == 1 &&
              maniMorphChannelTargets(&m, 2, &first) == 1 && first == 4 &&
              maniMorphChannelTargets(&m, 3, &first) == 0 && first == 0,
          "a channel's targets");
    CHECK(maniMorphTargetFullWeight(&m, 1) == -1.0f && maniMorphTargetFullWeight(&m, 2) == 0.5f &&
              maniMorphTargetFullWeight(&m, 5) == 0.0f,
          "full weights");
    // Smile: sorted, vertex 5 summed; wide: dense.
    const uint8_t* smile = m.targets;
    const uint8_t* wide = m.targets + 4 * MANI_MORPH_TARGET_SIZE;
    CHECK(maniLoadU32(smile + 4) == (maniMorphPositions | maniMorphNormals) &&
              maniLoadU32(smile + 8) == 3 && maniLoadU32(m.indices) == 2 &&
              maniLoadU32(m.indices + 4) == 5 && maniLoadU32(m.indices + 8) == 9,
          "sparse sorted and merged");
    float d[3];
    memcpy(d, m.deltas + 12, sizeof d);
    CHECK(d[0] == 1.5f && d[1] == 0.25f && d[2] == 0.0f, "vertex 5 summed in source order");
    memcpy(d, m.deltas + 4 * 9 + 12, sizeof d);
    CHECK(d[0] == 0.0f && d[1] == 1.5f && d[2] == 0.0f, "normals summed too");
    CHECK(maniLoadU32(wide + 4) == (maniMorphTangents | maniMorphDense) &&
              maniLoadU32(wide + 8) == kVertices,
          "every vertex: dense");
    uint64_t deltas = maniLoadU64(wide + 24);
    memcpy(d, m.deltas + 4 * (deltas + 3 * 7), sizeof d);
    CHECK(d[0] == 1.0f && d[1] == -0.5f, "dense in vertex order (given in reverse)");
}

// The walk by its definition: the keys and the zero shape in order of
// full weight, the weight between the two around it, the end segments
// extended.
static void ReferenceWeights(const float* full, uint32_t count, double w, double* out)
{
    double at[65];
    int32_t key[65];
    uint32_t n = 0;
    bool zero = false;
    for (uint32_t i = 0; i < count; ++i)
    {
        if (!zero && full[i] > 0.0f)
        {
            at[n] = 0.0;
            key[n++] = -1;
            zero = true;
        }
        at[n] = (double)full[i];
        key[n++] = (int32_t)i;
    }
    if (!zero)
    {
        at[n] = 0.0;
        key[n++] = -1;
    }
    uint32_t k = 0;
    while (k + 2 < n && w >= at[k + 1])
    {
        ++k;
    }
    double t = (w - at[k]) / (at[k + 1] - at[k]);
    for (uint32_t i = 0; i < count; ++i)
    {
        out[i] = 0.0;
    }
    if (key[k] >= 0)
    {
        out[key[k]] = 1.0 - t;
    }
    if (key[k + 1] >= 0)
    {
        out[key[k + 1]] = t;
    }
}

static void TestWeights(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    const float full[3] = {-1.0f, 0.5f, 1.0f};
    double worst = 0.0;
    bool through = true;
    for (int i = -60; i <= 60; ++i)
    {
        float w = (float)i * 0.05f;
        const float channels[3] = {w, w, w};
        float targets[5];
        CHECK(maniMorphTargetWeights(&m, channels, targets) == mani_success, "weights");
        through = through && targets[0] == w && targets[4] == w;
        double want[3];
        ReferenceWeights(full, 3, (double)w, want);
        for (int k = 0; k < 3; ++k)
        {
            double e = fabs((double)targets[1 + k] - want[k]);
            worst = e > worst ? e : worst;
        }
    }
    CHECK(through, "a key at 1 passes the weight through, either sign");
    CHECK(worst < 1e-6, "in-betweens as the piecewise linear reference");
    if (worst >= 1e-6)
    {
        printf("  weights apart by %g\n", worst);
    }
    // At a key, that key alone; at 0, nothing.
    const float atKey[3] = {0.0f, 0.5f, 0.0f};
    float targets[5];
    CHECK(maniMorphTargetWeights(&m, atKey, targets) == mani_success && targets[0] == 0.0f &&
              targets[1] == 0.0f && targets[2] == 1.0f && targets[3] == 0.0f && targets[4] == 0.0f,
          "at a key and at 0");
    const float bad[3] = {0.0f, NAN, 0.0f};
    memset(targets, 0x7F, sizeof targets);
    float kept[5];
    memcpy(kept, targets, sizeof kept);
    CHECK(maniMorphTargetWeights(&m, bad, targets) == mani_errorInvalid &&
              memcmp(kept, targets, sizeof kept) == 0,
          "a weight not finite: nothing written");
    CHECK(maniMorphTargetWeights(&m, NULL, targets) == mani_errorInvalid, "no weights");
}

// Vertices: position, normal, tangent (four floats), interleaved.
typedef struct Vertex
{
    float position[3];
    float normal[3];
    float tangent[4];
} Vertex;

static Vertex s_in[kVertices];
static Vertex s_out[kVertices];
static Vertex s_other[kVertices];

static void RandomVertices(void)
{
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        for (int k = 0; k < 3; ++k)
        {
            s_in[v].position[k] = Uniform(-1, 1);
            s_in[v].normal[k] = Uniform(-1, 1);
            s_in[v].tangent[k] = Uniform(-1, 1);
        }
        s_in[v].tangent[3] = v % 2 == 0 ? 1.0f : -1.0f;
    }
}

static void Streams(const Vertex* in, Vertex* out, maniSkinStream streams[3])
{
    streams[0] = (maniSkinStream){in->position,   out->position,   sizeof(Vertex),
                                  sizeof(Vertex), maniStreamPoint, 0};
    streams[1] = (maniSkinStream){in->normal,     out->normal,      sizeof(Vertex),
                                  sizeof(Vertex), maniStreamNormal, 0};
    streams[2] = (maniSkinStream){in->tangent,    out->tangent,      sizeof(Vertex),
                                  sizeof(Vertex), maniStreamTangent, 0};
}

// The source's delta of one attribute (0 positions, 1 normals, 2
// tangents) for vertex v in target t of the sample, summed over repeats.
static void SourceDelta(uint32_t t, int attribute, uint32_t v, double out[3])
{
    out[0] = out[1] = out[2] = 0.0;
    const maniMorphTargetSource* all[5] = {&s_smile[0], &s_blinks[0], &s_blinks[1], &s_blinks[2],
                                           &s_wide[0]};
    const maniMorphTargetSource* s = all[t];
    const float* list = attribute == 0 ? s->positions : (attribute == 1 ? s->normals : s->tangents);
    for (uint32_t i = 0; list != NULL && i < s->count; ++i)
    {
        if ((s->indices == NULL ? i : s->indices[i]) == v)
        {
            for (int k = 0; k < 3; ++k)
            {
                out[k] += (double)list[3 * i + k];
            }
        }
    }
}

static double Reference(const float* weights, uint32_t v, int attribute, double out[3])
{
    const float* in =
        attribute == 0 ? s_in[v].position : (attribute == 1 ? s_in[v].normal : s_in[v].tangent);
    for (int k = 0; k < 3; ++k)
    {
        out[k] = (double)in[k];
    }
    for (uint32_t t = 0; t < 5; ++t)
    {
        double d[3];
        SourceDelta(t, attribute, v, d);
        for (int k = 0; k < 3; ++k)
        {
            out[k] += (double)weights[t] * d[k];
        }
    }
    double length = sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    for (int k = 0; k < 3 && attribute > 0; ++k)
    {
        out[k] /= length;
    }
    return length;
}

static void TestJob(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    double worst = 0.0;
    bool wKept = true;
    for (int round = 0; round < 50; ++round)
    {
        RandomVertices();
        const float channels[3] = {Uniform(-1.5f, 1.5f), Uniform(-1.5f, 1.5f),
                                   Uniform(-1.5f, 1.5f)};
        float weights[5];
        CHECK(maniMorphTargetWeights(&m, channels, weights) == mani_success, "weights");
        maniSkinStream streams[3];
        Streams(s_in, s_out, streams);
        maniMorphJob job = {&m, weights, 0, kVertices, 3, 0, streams};
        CHECK(maniApplyMorphTargets(&job) == mani_success, "applies");
        for (uint32_t v = 0; v < kVertices; ++v)
        {
            const float* got[3] = {s_out[v].position, s_out[v].normal, s_out[v].tangent};
            for (int a = 0; a < 3; ++a)
            {
                double want[3];
                (void)Reference(weights, v, a, want);
                for (int k = 0; k < 3; ++k)
                {
                    double size = fabs(want[k]) > 1.0 ? fabs(want[k]) : 1.0;
                    double e = fabs((double)got[a][k] - want[k]) / size;
                    worst = e > worst ? e : worst;
                }
            }
            wKept = wKept && s_out[v].tangent[3] == s_in[v].tangent[3];
        }
    }
    CHECK(worst < 1e-5, "within binary32 of the binary64 reference");
    if (worst >= 1e-5)
    {
        printf("  job apart by %g\n", worst);
    }
    CHECK(wKept, "the tangents' w kept");
}

// Ranges, in place, and a sparse target against the same deltas dense:
// the same bits.
static void TestSameBits(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    RandomVertices();
    const float weights[5] = {0.75f, -0.25f, 0.5f, 0.125f, -1.5f};
    maniSkinStream streams[3];
    Streams(s_in, s_out, streams);
    maniMorphJob whole = {&m, weights, 0, kVertices, 3, 0, streams};
    CHECK(maniApplyMorphTargets(&whole) == mani_success, "whole");
    const uint32_t cuts[4] = {0, 3, 6, kVertices};
    for (int i = 0; i < 3; ++i)
    {
        maniSkinStream part[3];
        Streams(&s_in[cuts[i]], &s_other[cuts[i]], part);
        maniMorphJob job = {&m, weights, cuts[i], cuts[i + 1] - cuts[i], 3, 0, part};
        CHECK(maniApplyMorphTargets(&job) == mani_success, "a range");
    }
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "ranges give the same bits");
    memcpy(s_other, s_in, sizeof s_other);
    maniSkinStream place[3];
    Streams(s_other, s_other, place);
    maniMorphJob inPlace = {&m, weights, 0, kVertices, 3, 0, place};
    CHECK(maniApplyMorphTargets(&inPlace) == mani_success, "in place");
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "in place gives the same bits");
    // One target sparse on vertices 1 to 11, one dense with vertex 0's
    // delta 0: the same bits (inputs have no zero, so +0 changes none).
    static uint32_t indices[kVertices - 1];
    static float positions[3 * kVertices];
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        for (int k = 0; k < 3; ++k)
        {
            positions[3 * v + k] = v == 0 ? 0.0f : Uniform(-1, 1);
        }
        if (v > 0)
        {
            indices[v - 1] = v;
        }
    }
    maniMorphTargetSource sparse = {1.0f, kVertices - 1, indices, positions + 3, NULL, NULL};
    maniMorphTargetSource dense = {1.0f, kVertices, NULL, positions, NULL, NULL};
    maniMorphChannelSource a = {"a", 1, 1, &sparse};
    maniMorphChannelSource b = {"a", 1, 1, &dense};
    maniMorphSource sa = {&a, 1, kVertices};
    maniMorphSource sb = {&b, 1, kVertices};
    static _Alignas(16) uint8_t fa[4096];
    static _Alignas(16) uint8_t fb[4096];
    size_t na = 0;
    size_t nb = 0;
    maniBuilder* builder = NewBuilder();
    maniMorphTargets ma;
    maniMorphTargets mb;
    CHECK(maniBuildMorphTargets(builder, &sa).result == mani_success &&
              maniCopyBuilt(builder, fa, sizeof fa, &na) == mani_success &&
              maniBuildMorphTargets(builder, &sb).result == mani_success &&
              maniCopyBuilt(builder, fb, sizeof fb, &nb) == mani_success &&
              maniOpenMorphTargets(fa, na, &ma).result == mani_success &&
              maniOpenMorphTargets(fb, nb, &mb).result == mani_success,
          "sparse and dense built");
    maniDestroyBuilder(builder);
    CHECK((maniLoadU32(ma.targets + 4) & maniMorphDense) == 0 &&
              (maniLoadU32(mb.targets + 4) & maniMorphDense) != 0,
          "one sparse, one dense");
    const float one = 0.625f;
    maniSkinStream points[1];
    points[0] = (maniSkinStream){s_in->position, s_out->position, sizeof(Vertex),
                                 sizeof(Vertex), maniStreamPoint, 0};
    maniMorphJob ja = {&ma, &one, 0, kVertices, 1, 0, points};
    CHECK(maniApplyMorphTargets(&ja) == mani_success, "sparse");
    memcpy(s_other, s_out, sizeof s_other);
    maniMorphJob jb = {&mb, &one, 0, kVertices, 1, 0, points};
    CHECK(maniApplyMorphTargets(&jb) == mani_success, "dense");
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "sparse and dense give the same bits");
}

static void ExpectBuildRefused(const maniMorphSource* source, maniBuildReason reason, uint32_t item,
                               uint32_t other, const char* what)
{
    maniBuilder* builder = NewBuilder();
    maniBuildResult r = maniBuildMorphTargets(builder, source);
    CHECK(r.result == mani_errorInvalid && r.reason == reason && r.item == item && r.other == other,
          what);
    if (r.reason != reason || r.item != item || r.other != other)
    {
        printf("  %s: reason %u item %u other %u\n", what, (unsigned)r.reason, (unsigned)r.item,
               (unsigned)r.other);
    }
    maniDestroyBuilder(builder);
}

static void TestBuildRefusals(void)
{
    maniMorphSource source;
    SampleSource(&source);
    source.channelCount = 0;
    ExpectBuildRefused(&source, maniBuildReasonCount, 0, 0, "no channels");
    SampleSource(&source);
    source.vertexCount = 0;
    ExpectBuildRefused(&source, maniBuildReasonCount, 0, 0, "no vertices");
    SampleSource(&source);
    s_channels[1].targetCount = 0;
    ExpectBuildRefused(&source, maniBuildReasonCount, 1, 0, "a channel without targets");
    SampleSource(&source);
    s_blinks[1].count = kVertices - 1;
    ExpectBuildRefused(&source, maniBuildReasonCount, 2, 0, "dense, short");
    SampleSource(&source);
    s_blinks[2].fullWeight = 0.5f;
    ExpectBuildRefused(&source, maniBuildReasonWeight, 3, 0, "full weights not increasing");
    SampleSource(&source);
    s_smile[0].fullWeight = 0.0f;
    ExpectBuildRefused(&source, maniBuildReasonWeight, 0, 0, "a full weight of 0");
    SampleSource(&source);
    s_wide[0].fullWeight = INFINITY;
    ExpectBuildRefused(&source, maniBuildReasonWeight, 4, 0, "a full weight not finite");
    SampleSource(&source);
    s_wideIndices[6] = kVertices;
    ExpectBuildRefused(&source, maniBuildReasonVertex, 4, 6, "an index past the vertices");
    SampleSource(&source);
    s_blink[1][3 * 4 + 1] = NAN;
    ExpectBuildRefused(&source, maniBuildReasonNotFinite, 2, 4, "a delta not finite");
    SampleSource(&source);
    s_channels[2].name = "smile";
    s_channels[2].nameLength = 5;
    ExpectBuildRefused(&source, maniBuildReasonDuplicateName, 2, 0, "a repeated name");
    SampleSource(&source);
    s_channels[1].nameLength = 0;
    ExpectBuildRefused(&source, maniBuildReasonName, 1, 0, "an empty name");
    SampleSource(&source);
    s_channels[1].name = "\xff\xfe";
    s_channels[1].nameLength = 2;
    ExpectBuildRefused(&source, maniBuildReasonName, 1, 0, "a name not UTF-8");
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildMorphTargets(builder, NULL).result == mani_errorInvalid &&
              maniBuildMorphTargets(NULL, &source).result == mani_errorInvalid,
          "NULL arguments");
    maniDestroyBuilder(builder);
}

// s_file again, one section replaced (tag in 1 to 6), the identity right.
static void Craft(uint32_t tag, const void* bytes, uint64_t size)
{
    maniContainer c;
    CHECK(maniOpenContainer(s_file, s_size, maniKindMorphTargets, MANI_MORPH_FORMAT_VERSION, &c)
                  .result == mani_success,
          "container");
    const uint8_t* data[MANI_MORPH_SECTIONS];
    uint64_t sizes[MANI_MORPH_SECTIONS];
    maniSectionSource sections[MANI_MORPH_SECTIONS];
    uint32_t count = 0;
    static const uint8_t kNone = 0;
    for (uint32_t t = 1; t <= MANI_MORPH_SECTIONS; ++t)
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
        if (sizes[t - 1] > 0)
        {
            sections[count++] = (maniSectionSource){t, data[t - 1], sizes[t - 1]};
        }
    }
    uint64_t fileSize = 0;
    CHECK(maniContainerSize(sections, count, &fileSize) && fileSize <= sizeof s_crafted, "fits");
    maniWriteContainer(s_crafted, maniKindMorphTargets, MANI_MORPH_FORMAT_VERSION,
                       maniMorphComputeIdentity(data, sizes), sections, count);
    s_size = (size_t)fileSize;
    memcpy(s_file, s_crafted, s_size);
}

static uint64_t SectionAt(uint32_t tag)
{
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindMorphTargets, MANI_MORPH_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "section");
    return at;
}

static void ExpectFormat(uint64_t offset, const char* what)
{
    maniMorphTargets m;
    maniFileResult r = maniOpenMorphTargets(s_file, s_size, &m);
    CHECK(r.result == mani_errorFormat && r.offset == offset, what);
    if (r.offset != offset)
    {
        printf("  %s: at %llu, expected %llu\n", what, (unsigned long long)r.offset,
               (unsigned long long)offset);
    }
}

// A copy of one section with a u32 (or float) stored at a byte, crafted
// into the file, and the offset expected.
static void CraftU32(uint32_t tag, uint64_t byte, uint32_t value, uint64_t expected,
                     const char* what)
{
    BuildSample();
    maniContainer c;
    const uint8_t* data = NULL;
    uint64_t size = 0;
    uint64_t at = 0;
    CHECK(maniOpenContainer(s_file, s_size, maniKindMorphTargets, MANI_MORPH_FORMAT_VERSION, &c)
                      .result == mani_success &&
              maniContainerSection(&c, tag, &data, &size, &at),
          "section");
    static uint8_t copy[8192];
    memcpy(copy, data, size);
    maniStoreU32(copy + byte, value);
    Craft(tag, copy, size);
    ExpectFormat(SectionAt(tag) + expected, what);
}

static uint32_t Bits(float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof bits);
    return bits;
}

static void TestOpenRefusals(void)
{
    const uint32_t T = MANI_MORPH_TARGET_SIZE;
    CraftU32(maniMorphTagHeader, 0, 0, 0, "no vertices");
    CraftU32(maniMorphTagHeader, 12, 1, 12, "the header's zero");
    CraftU32(maniMorphTagChannels, 16, 2, 16, "a channel not following the last");
    CraftU32(maniMorphTagChannels, 4, 0, 4, "a channel without targets");
    CraftU32(maniMorphTagChannels, 24, 6, 24, "a channel's name not following");
    CraftU32(maniMorphTagChannels, 12, 0, 12, "an empty name");
    CraftU32(maniMorphTagTargets, 2 * T, Bits(-2.0f), 2 * T, "full weights not increasing");
    CraftU32(maniMorphTagTargets, 0, Bits(0.0f), 0, "a full weight of 0");
    CraftU32(maniMorphTagTargets, 0, Bits(NAN), 0, "a full weight NaN");
    CraftU32(maniMorphTagTargets, 4, 16, 4, "unknown attributes");
    CraftU32(maniMorphTagTargets, T + 8, kVertices - 1, T + 8, "dense, short");
    CraftU32(maniMorphTagTargets, 12, 1, 12, "a target's zero");
    CraftU32(maniMorphTagTargets, 16, 1, 16, "indices not following");
    CraftU32(maniMorphTagTargets, T + 24, 1, T + 24, "deltas not following");
    CraftU32(maniMorphTagIndices, 4, 2, 4, "indices repeated");
    CraftU32(maniMorphTagIndices, 8, kVertices, 8, "an index past the vertices");
    CraftU32(maniMorphTagDeltas, 4 * 7, Bits(INFINITY), 4 * 7, "a delta not finite");
    BuildSample();
    uint8_t header[MANI_MORPH_HEADER_SIZE];
    memcpy(header, s_file + SectionAt(maniMorphTagHeader), sizeof header);
    Craft(maniMorphTagHeader, header, 12);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 16, "a short header");
    BuildSample();
    Craft(maniMorphTagNames, "smileblinkwid", 13);
    ExpectFormat(SectionAt(maniMorphTagChannels) + 2 * MANI_MORPH_CHANNEL_SIZE + 12,
                 "a name past the bytes");
    BuildSample();
    Craft(maniMorphTagNames, "smileblinkwide!", 15);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 5 * MANI_CONTAINER_ENTRY_SIZE + 16,
                 "name bytes left over");
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, NULL).result == mani_errorInvalid &&
              maniOpenMorphTargets(s_file, s_size, &m).result == mani_success,
          "a NULL view");
    s_file[40] ^= 1;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_errorFormat, "the identity");
    BuildSample();
}

static void TestJobRefusals(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    float weights[5] = {1, 1, 1, 1, 1};
    maniSkinStream streams[3];
    Streams(s_in, s_out, streams);
    memcpy(s_other, s_out, sizeof s_other);
    maniMorphJob job = {&m, weights, 0, kVertices, 3, 0, streams};
    streams[1].kind = maniStreamDirection;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "a direction stream");
    streams[1].kind = maniStreamNormal;
    streams[2].outStride = 12;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "a tangent stride under 16");
    streams[2].outStride = sizeof(Vertex);
    weights[3] = INFINITY;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "a weight not finite");
    weights[3] = 1.0f;
    job.first = 5;
    job.count = kVertices - 4;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "a range past the vertices");
    job.first = 0;
    job.count = kVertices;
    job.weights = NULL;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "no weights");
    CHECK(maniApplyMorphTargets(NULL) == mani_errorInvalid, "no job");
    CHECK(memcmp(s_other, s_out, sizeof s_out) == 0, "nothing written");
    job.weights = weights;
    job.first = kVertices;
    job.count = 0;
    CHECK(maniApplyMorphTargets(&job) == mani_success, "an empty range at the end");
}

static void TestPinned(void)
{
    BuildSample();
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_file, (int32_t)s_size);
    CHECK(hash == PINNED_MORPH_FILE, "pinned file");
    if (hash != PINNED_MORPH_FILE)
    {
        printf("morph file hash 0x%016llx\n", (unsigned long long)hash);
    }
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    s_random = 0x0123456789ABCDEFull;
    RandomVertices();
    const float channels[3] = {0.8f, -0.4f, 1.3f};
    float weights[5];
    CHECK(maniMorphTargetWeights(&m, channels, weights) == mani_success, "weights");
    maniSkinStream streams[3];
    Streams(s_in, s_out, streams);
    maniMorphJob job = {&m, weights, 0, kVertices, 3, 0, streams};
    CHECK(maniApplyMorphTargets(&job) == mani_success, "applies");
    hash = maniHash64(MANI_HASH_INIT, s_out, (int32_t)sizeof s_out);
    CHECK(hash == PINNED_MORPH_JOB, "pinned job");
    if (hash != PINNED_MORPH_JOB)
    {
        printf("morph job hash 0x%016llx\n", (unsigned long long)hash);
    }
}

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

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    TestBuildsAndReads();
    TestWeights();
    TestJob();
    TestSameBits();
    TestBuildRefusals();
    TestOpenRefusals();
    TestJobRefusals();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
