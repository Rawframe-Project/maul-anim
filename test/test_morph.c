// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Morph targets: built over an attribute table (positions,
// normals, tangents, a texture coordinate and a color set) with sparse
// targets sorted and their repeated vertices summed, a target touching
// every vertex stored densely, FBX in-betweens; channel weights walked to target weights as
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

#define PINNED_MORPH_FILE 0x5772cedfe7070060ull
#define PINNED_MORPH_JOB  0x505a297b9b1d7aabull

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

// The attributes: positions, normals, tangents, texture coordinate set
// 0 and color set 1.
static const maniMorphAttributeSource kAttributes[5] = {{maniMorphPosition, 0},
                                                        {maniMorphNormal, 0},
                                                        {maniMorphTangent, 0},
                                                        {maniMorphTexcoord, 0},
                                                        {maniMorphColor, 1}};

// The sample: "smile", one sparse target of positions and normals given
// out of order with vertex 5 twice; "blink", FBX in-betweens at -1, 0.5
// and 1, dense positions (the middle one texture coordinates too);
// "wide", tangents and colors given for every vertex in reverse, so
// stored densely.
static const uint32_t kSmileIndices[4] = {5, 2, 5, 9};
static const float kSmilePositions[12] = {1, 0, 0, 0, 2, 0, 0.5f, 0.25f, 0, 0, 0, 3};
static const float kSmileNormals[12] = {0, 1, 0, 1, 0, 0, 0, 0.5f, 0, 0, 0, 1};
static float s_blink[3][3 * kVertices];
static float s_blinkTexcoords[2 * kVertices];
static uint32_t s_wideIndices[kVertices];
static float s_wideTangents[3 * kVertices];
static float s_wideColors[4 * kVertices];

static const float* s_smileDeltas[5];
static const float* s_blinkDeltas[3][5];
static const float* s_wideDeltas[5];
static maniMorphTargetSource s_smile[1];
static maniMorphTargetSource s_blinks[3];
static maniMorphTargetSource s_wide[1];
static maniMorphChannelSource s_channels[3];
static maniMorphAttributeSource s_attributes[5];

static void SampleData(void)
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
        s_blinkTexcoords[2 * i] = 0.0625f * (float)i;
        s_blinkTexcoords[2 * i + 1] = -0.25f;
        for (uint32_t k = 0; k < 4; ++k)
        {
            s_wideColors[4 * i + k] = 0.03125f * (float)(i + k);
        }
    }
}

static void SampleSource(maniMorphSource* source)
{
    SampleData();
    memcpy(s_attributes, kAttributes, sizeof s_attributes);
    memset(s_smileDeltas, 0, sizeof s_smileDeltas);
    memset(s_blinkDeltas, 0, sizeof s_blinkDeltas);
    memset(s_wideDeltas, 0, sizeof s_wideDeltas);
    s_smileDeltas[0] = kSmilePositions;
    s_smileDeltas[1] = kSmileNormals;
    for (int k = 0; k < 3; ++k)
    {
        s_blinkDeltas[k][0] = s_blink[k];
    }
    s_blinkDeltas[1][3] = s_blinkTexcoords;
    s_wideDeltas[2] = s_wideTangents;
    s_wideDeltas[4] = s_wideColors;
    s_smile[0] = (maniMorphTargetSource){1.0f, 4, kSmileIndices, s_smileDeltas, NULL, 0, 0};
    s_blinks[0] = (maniMorphTargetSource){-1.0f, kVertices, NULL, s_blinkDeltas[0], NULL, 0, 0};
    s_blinks[1] = (maniMorphTargetSource){0.5f, kVertices, NULL, s_blinkDeltas[1], NULL, 0, 0};
    s_blinks[2] = (maniMorphTargetSource){1.0f, kVertices, NULL, s_blinkDeltas[2], "shut", 4, 0};
    s_wide[0] = (maniMorphTargetSource){1.0f, kVertices, s_wideIndices, s_wideDeltas, NULL, 0, 0};
    s_channels[0] = (maniMorphChannelSource){"smile", 5, 1, s_smile};
    s_channels[1] = (maniMorphChannelSource){"blink", 5, 3, s_blinks};
    s_channels[2] = (maniMorphChannelSource){"wide", 4, 1, s_wide};
    *source = (maniMorphSource){s_channels, 3, kVertices, s_attributes, 5, 0};
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
              maniMorphTargetCount(&m) == 5 && maniMorphAttributeCount(&m) == 5,
          "counts");
    uint32_t set = 9;
    CHECK(maniMorphAttribute(&m, 4, &set) == maniMorphColor && set == 1 &&
              maniMorphAttribute(&m, 3, &set) == maniMorphTexcoord && set == 0 &&
              maniMorphAttribute(&m, 5, &set) == UINT32_MAX && set == 0,
          "the attribute table");
    uint32_t length = 0;
    const char* name = maniMorphChannelName(&m, 1, &length);
    CHECK(length == 5 && memcmp(name, "blink", 5) == 0, "a channel's name");
    CHECK(maniMorphChannelName(&m, 3, &length) == NULL && length == 0, "no channel past the count");
    CHECK(maniMorphFindChannel(&m, "wide", 4) == 2 &&
              maniMorphFindChannel(&m, "wid", 3) == MANI_NO_CHANNEL &&
              maniMorphFindChannel(&m, NULL, 1) == MANI_NO_CHANNEL &&
              maniMorphFindChannel(&m, "", 0) == MANI_NO_CHANNEL,
          "find");
    const char* target = maniMorphTargetName(&m, 2, &length);
    CHECK(length == 7 && memcmp(target, "blink/1", 7) == 0, "a target's default name");
    target = maniMorphTargetName(&m, 1, &length);
    CHECK(length == 5 && memcmp(target, "blink", 5) == 0, "a first target: the channel's name");
    CHECK(maniMorphFindTarget(&m, "shut", 4) == 3 && maniMorphFindTarget(&m, "wide", 4) == 4 &&
              maniMorphFindTarget(&m, "blink/2", 7) == MANI_NO_CHANNEL &&
              maniMorphFindTarget(&m, NULL, 0) == MANI_NO_CHANNEL &&
              maniMorphTargetName(&m, 5, &length) == NULL && length == 0,
          "a given name; find");
    uint32_t first = 0;
    CHECK(maniMorphChannelTargets(&m, 1, &first) == 3 && first == 1 &&
              maniMorphChannelTargets(&m, 2, &first) == 1 && first == 4 &&
              maniMorphChannelTargets(&m, 3, &first) == 0 && first == 0,
          "a channel's targets");
    CHECK(maniMorphTargetFullWeight(&m, 1) == -1.0f && maniMorphTargetFullWeight(&m, 2) == 0.5f &&
              maniMorphTargetFullWeight(&m, 5) == 0.0f,
          "full weights");
    // Smile: sorted, vertex 5 summed; wide: dense, tangents then colors.
    const uint8_t* smile = m.targets;
    const uint8_t* blink = m.targets + 2 * MANI_MORPH_TARGET_SIZE;
    const uint8_t* wide = m.targets + 4 * MANI_MORPH_TARGET_SIZE;
    CHECK(maniLoadU32(smile + 4) == 3u && maniLoadU32(smile + 12) == 0 &&
              maniLoadU32(smile + 8) == 3 && maniLoadU32(m.indices) == 2 &&
              maniLoadU32(m.indices + 4) == 5 && maniLoadU32(m.indices + 8) == 9,
          "sparse sorted and merged");
    float d[3];
    memcpy(d, m.deltas + 12, sizeof d);
    CHECK(d[0] == 1.5f && d[1] == 0.25f && d[2] == 0.0f, "vertex 5 summed in source order");
    memcpy(d, m.deltas + 4 * 9 + 12, sizeof d);
    CHECK(d[0] == 0.0f && d[1] == 1.5f && d[2] == 0.0f, "normals summed too");
    CHECK(maniLoadU32(blink + 4) == 9u && maniLoadU32(blink + 12) == MANI_MORPH_DENSE,
          "positions and texture coordinates");
    float uv[2];
    memcpy(uv, m.deltas + 4 * (maniLoadU64(blink + 24) + 3 * kVertices + 2 * 5), sizeof uv);
    CHECK(uv[0] == 0.3125f && uv[1] == -0.25f, "texture coordinates after the positions");
    CHECK(maniLoadU32(wide + 4) == 0x14u && maniLoadU32(wide + 12) == MANI_MORPH_DENSE &&
              maniLoadU32(wide + 8) == kVertices,
          "every vertex: dense");
    uint64_t deltas = maniLoadU64(wide + 24);
    memcpy(d, m.deltas + 4 * (deltas + 3 * 7), sizeof d);
    CHECK(d[0] == 1.0f && d[1] == -0.5f, "dense in vertex order (given in reverse)");
    float color[4];
    memcpy(color, m.deltas + 4 * (deltas + 3 * kVertices + 4 * 7), sizeof color);
    CHECK(color[0] == 0.125f && color[3] == 0.21875f, "colors after the tangents");
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

// Vertices: position, normal, tangent (four floats), texture coordinate
// and color, interleaved.
typedef struct Vertex
{
    float position[3];
    float normal[3];
    float tangent[4];
    float texcoord[2];
    float color[4];
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
        s_in[v].texcoord[0] = Uniform(0.1f, 1);
        s_in[v].texcoord[1] = Uniform(0.1f, 1);
        for (int k = 0; k < 4; ++k)
        {
            s_in[v].color[k] = Uniform(0.1f, 1);
        }
    }
}

// A stream over the vertices: an attribute's floats at its member.
static maniMorphStream Stream(const float* in, float* out, maniMorphSemantic semantic, uint32_t set,
                              uint32_t components)
{
    return (maniMorphStream){in, out, sizeof(Vertex), sizeof(Vertex), semantic, set, components, 0};
}

static void Streams(const Vertex* in, Vertex* out, maniMorphStream streams[5])
{
    streams[0] = Stream(in->position, out->position, maniMorphPosition, 0, 3);
    streams[1] = Stream(in->normal, out->normal, maniMorphNormal, 0, 3);
    streams[2] = Stream(in->tangent, out->tangent, maniMorphTangent, 0, 4);
    streams[3] = Stream(in->texcoord, out->texcoord, maniMorphTexcoord, 0, 2);
    streams[4] = Stream(in->color, out->color, maniMorphColor, 1, 4);
}

// The floats compared of each table entry, and a vertex's member for it.
static const uint32_t kCompared[5] = {3, 3, 3, 2, 4};

static const float* Member(const Vertex* x, uint32_t a)
{
    const float* members[5] = {x->position, x->normal, x->tangent, x->texcoord, x->color};
    return members[a];
}

// The source's deltas of table entry a for vertex v in target t of the
// sample, summed over repeats.
static void SourceDelta(uint32_t t, uint32_t a, uint32_t v, double out[4])
{
    out[0] = out[1] = out[2] = out[3] = 0.0;
    const maniMorphTargetSource* all[5] = {&s_smile[0], &s_blinks[0], &s_blinks[1], &s_blinks[2],
                                           &s_wide[0]};
    const maniMorphTargetSource* s = all[t];
    const float* list = s->deltas[a];
    uint32_t width = kCompared[a];
    for (uint32_t i = 0; list != NULL && i < s->count; ++i)
    {
        if ((s->indices == NULL ? i : s->indices[i]) == v)
        {
            for (uint32_t k = 0; k < width; ++k)
            {
                out[k] += (double)list[width * i + k];
            }
        }
    }
}

// In binary64: the input plus the weighted deltas, normals and tangents
// renormalized.
static void Reference(const float* weights, uint32_t v, uint32_t a, double out[4])
{
    const float* in = Member(&s_in[v], a);
    for (uint32_t k = 0; k < kCompared[a]; ++k)
    {
        out[k] = (double)in[k];
    }
    for (uint32_t t = 0; t < 5; ++t)
    {
        double d[4];
        SourceDelta(t, a, v, d);
        for (uint32_t k = 0; k < kCompared[a]; ++k)
        {
            out[k] += (double)weights[t] * d[k];
        }
    }
    double length = sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    for (int k = 0; k < 3 && (a == 1 || a == 2); ++k)
    {
        out[k] /= length;
    }
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
        maniMorphStream streams[5];
        Streams(s_in, s_out, streams);
        maniMorphJob job = {&m, weights, 0, kVertices, 5, 0, streams};
        CHECK(maniApplyMorphTargets(&job) == mani_success, "applies");
        for (uint32_t v = 0; v < kVertices; ++v)
        {
            for (uint32_t a = 0; a < 5; ++a)
            {
                double want[4];
                Reference(weights, v, a, want);
                const float* got = Member(&s_out[v], a);
                for (uint32_t k = 0; k < kCompared[a]; ++k)
                {
                    double size = fabs(want[k]) > 1.0 ? fabs(want[k]) : 1.0;
                    double e = fabs((double)got[k] - want[k]) / size;
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

// A range ending at 5 leaves the vertex past it alone though smile's
// sparse target moves vertex 5; at weight 0 every target leaves the
// streams a copy of their input, bit for bit (-0 kept).
static void TestRangeAndZero(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    RandomVertices();
    const float ones[5] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    maniMorphStream streams[5];
    memset(s_other, 0, sizeof s_other);
    Vertex guard = s_other[5];
    Streams(s_in, s_other, streams);
    maniMorphJob job = {&m, ones, 0, 5, 5, 0, streams};
    CHECK(maniApplyMorphTargets(&job) == mani_success &&
              memcmp(&s_other[5], &guard, sizeof guard) == 0,
          "a range writes no vertex past its end");
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        s_in[v].position[0] = -0.0f;
        s_in[v].position[1] = -0.0f;
        s_in[v].position[2] = -0.0f;
    }
    const float zeros[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    maniMorphStream position = Stream(s_in->position, s_out->position, maniMorphPosition, 0, 3);
    job = (maniMorphJob){&m, zeros, 0, kVertices, 1, 0, &position};
    bool same = maniApplyMorphTargets(&job) == mani_success;
    for (uint32_t v = 0; same && v < kVertices; ++v)
    {
        same = memcmp(s_out[v].position, s_in[v].position, sizeof s_in[v].position) == 0;
    }
    CHECK(same, "weights of 0: the input's bits");
}

// A sparse target on vertices 1 to 11 and a dense one with vertex 0's
// delta 0 give the same bits (inputs have no zero, so +0 changes none).
static void TestSparseAsDense(void)
{
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
    const float* sparseDeltas[1] = {positions + 3};
    const float* denseDeltas[1] = {positions};
    maniMorphTargetSource sparse = {1.0f, kVertices - 1, indices, sparseDeltas, NULL, 0, 0};
    maniMorphTargetSource dense = {1.0f, kVertices, NULL, denseDeltas, NULL, 0, 0};
    maniMorphChannelSource a = {"a", 1, 1, &sparse};
    maniMorphChannelSource b = {"a", 1, 1, &dense};
    maniMorphSource sa = {&a, 1, kVertices, kAttributes, 1, 0};
    maniMorphSource sb = {&b, 1, kVertices, kAttributes, 1, 0};
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
    CHECK(maniLoadU32(ma.targets + 12) == 0 && maniLoadU32(mb.targets + 12) == MANI_MORPH_DENSE,
          "one sparse, one dense");
    const float one = 0.625f;
    maniMorphStream points[1] = {Stream(s_in->position, s_out->position, maniMorphPosition, 0, 3)};
    maniMorphJob ja = {&ma, &one, 0, kVertices, 1, 0, points};
    CHECK(maniApplyMorphTargets(&ja) == mani_success, "sparse");
    memcpy(s_other, s_out, sizeof s_other);
    maniMorphJob jb = {&mb, &one, 0, kVertices, 1, 0, points};
    CHECK(maniApplyMorphTargets(&jb) == mani_success, "dense");
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "sparse and dense give the same bits");
}

// Ranges and in place give the same bits; an RGB stream takes the color
// deltas' first three; a stream of an attribute the file lacks is copied.
static void TestSameBits(void)
{
    BuildSample();
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_file, s_size, &m).result == mani_success, "opens");
    RandomVertices();
    const float weights[5] = {0.75f, -0.25f, 0.5f, 0.125f, -1.5f};
    maniMorphStream streams[5];
    Streams(s_in, s_out, streams);
    maniMorphJob whole = {&m, weights, 0, kVertices, 5, 0, streams};
    CHECK(maniApplyMorphTargets(&whole) == mani_success, "whole");
    memset(s_other, 0, sizeof s_other);
    const uint32_t cuts[4] = {0, 3, 6, kVertices};
    for (int i = 0; i < 3; ++i)
    {
        maniMorphStream part[5];
        Streams(&s_in[cuts[i]], &s_other[cuts[i]], part);
        maniMorphJob job = {&m, weights, cuts[i], cuts[i + 1] - cuts[i], 5, 0, part};
        CHECK(maniApplyMorphTargets(&job) == mani_success, "a range");
    }
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "ranges give the same bits");
    memcpy(s_other, s_in, sizeof s_other);
    maniMorphStream place[5];
    Streams(s_other, s_other, place);
    maniMorphJob inPlace = {&m, weights, 0, kVertices, 5, 0, place};
    CHECK(maniApplyMorphTargets(&inPlace) == mani_success, "in place");
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "in place gives the same bits");
    TestSparseAsDense();
    const float full[5] = {1, 1, 1, 1, 1};
    maniMorphStream rgb[2] = {Stream(s_in->color, s_out->color, maniMorphColor, 1, 3),
                              Stream(s_in->texcoord, s_out->texcoord, maniMorphTexcoord, 1, 2)};
    memcpy(s_out, s_in, sizeof s_out);
    s_out[7].color[3] = 9.0f;
    maniMorphJob colors = {&m, full, 0, kVertices, 2, 0, rgb};
    CHECK(maniApplyMorphTargets(&colors) == mani_success, "colors");
    CHECK(s_out[7].color[0] == s_in[7].color[0] + s_wideColors[4 * 4] &&
              s_out[7].color[2] == s_in[7].color[2] + s_wideColors[4 * 4 + 2] &&
              s_out[7].color[3] == 9.0f,
          "RGB takes three");
    CHECK(memcmp(s_out[3].texcoord, s_in[3].texcoord, 8) == 0, "a set the file lacks: copied");
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
    s_wideColors[4 * 6 + 3] = NAN;
    ExpectBuildRefused(&source, maniBuildReasonNotFinite, 4, 6, "a color delta not finite");
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
    SampleSource(&source);
    s_blinks[2].name = "\xc0";
    s_blinks[2].nameLength = 1;
    ExpectBuildRefused(&source, maniBuildReasonName, 3, 1, "a target's name not UTF-8");
    SampleSource(&source);
    s_blinks[2].nameLength = 0;
    ExpectBuildRefused(&source, maniBuildReasonName, 3, 1, "a target's empty name");
    SampleSource(&source);
    s_attributes[3].semantic = 5;
    ExpectBuildRefused(&source, maniBuildReasonAttribute, 3, 0, "an unknown semantic");
    SampleSource(&source);
    s_attributes[1].set = 1;
    ExpectBuildRefused(&source, maniBuildReasonAttribute, 1, 0, "normals of set 1");
    SampleSource(&source);
    s_attributes[4] = (maniMorphAttributeSource){maniMorphTexcoord, 0};
    ExpectBuildRefused(&source, maniBuildReasonAttribute, 4, 3, "an attribute twice");
    SampleSource(&source);
    source.attributeCount = MANI_MORPH_MAX_ATTRIBUTES + 1;
    ExpectBuildRefused(&source, maniBuildReasonCount, 0, 0, "too many attributes");
    SampleSource(&source);
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildMorphTargets(builder, NULL).result == mani_errorInvalid &&
              maniBuildMorphTargets(NULL, &source).result == mani_errorInvalid,
          "NULL arguments");
    source.attributes = NULL;
    CHECK(maniBuildMorphTargets(builder, &source).result == mani_errorInvalid, "no attributes");
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
    CraftU32(maniMorphTagHeader, 12, 33, 12, "33 attributes");
    CraftU32(maniMorphTagAttributes, 8 * 3, 7, 8 * 3, "an unknown semantic");
    CraftU32(maniMorphTagAttributes, 4, 1, 4, "positions of set 1");
    {
        BuildSample();
        uint8_t table[5 * MANI_MORPH_ATTRIBUTE_SIZE];
        memcpy(table, s_file + SectionAt(maniMorphTagAttributes), sizeof table);
        maniStoreU32(table + 8 * 4, maniMorphTexcoord);
        maniStoreU32(table + 8 * 4 + 4, 0);
        Craft(maniMorphTagAttributes, table, sizeof table);
        ExpectFormat(SectionAt(maniMorphTagAttributes) + 8 * 4, "an attribute twice");
    }
    CraftU32(maniMorphTagChannels, 16, 2, 16, "a channel not following the last");
    CraftU32(maniMorphTagChannels, 4, 0, 4, "a channel without targets");
    CraftU32(maniMorphTagChannels, 24, 6, 24, "a channel's name not following");
    CraftU32(maniMorphTagChannels, 12, 0, 12, "an empty name");
    CraftU32(maniMorphTagTargets, 2 * T, Bits(-2.0f), 2 * T, "full weights not increasing");
    CraftU32(maniMorphTagTargets, 0, Bits(0.0f), 0, "a full weight of 0");
    CraftU32(maniMorphTagTargets, 0, Bits(NAN), 0, "a full weight NaN");
    CraftU32(maniMorphTagTargets, 4, 32, 4, "an attribute past the table");
    CraftU32(maniMorphTagTargets, T + 8, kVertices - 1, T + 8, "dense, short");
    CraftU32(maniMorphTagTargets, 12, 2, 12, "unknown flags");
    CraftU32(maniMorphTagTargets, 16, 1, 16, "indices not following");
    CraftU32(maniMorphTagTargets, T + 24, 1, T + 24, "deltas not following");
    CraftU32(maniMorphTagTargets, T + 32, 0, T + 32, "a target's name not following");
    CraftU32(maniMorphTagTargets, 2 * T + 36, 0, 2 * T + 36, "a target's empty name");
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
    Craft(maniMorphTagNames, "smileblinkwidesmileblinkblink/1shutwide!", 40);
    ExpectFormat(MANI_CONTAINER_HEADER_SIZE + 6 * MANI_CONTAINER_ENTRY_SIZE + 16,
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
    maniMorphStream streams[5];
    Streams(s_in, s_out, streams);
    memcpy(s_other, s_out, sizeof s_other);
    maniMorphJob job = {&m, weights, 0, kVertices, 5, 0, streams};
    streams[1].semantic = 5;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "an unknown semantic");
    streams[1].semantic = maniMorphNormal;
    streams[1].set = 1;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "normals of set 1");
    streams[1].set = 0;
    streams[2].components = 3;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "tangents of three");
    streams[2].components = 4;
    streams[4].components = 2;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "colors of two");
    streams[4].components = 4;
    streams[2].outStride = 12;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "a tangent stride under 16");
    streams[2].outStride = sizeof(Vertex);
    streams[0].out = NULL;
    CHECK(maniApplyMorphTargets(&job) == mani_errorInvalid, "no output");
    streams[0].out = s_out->position;
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
    // The writer's library version (bytes 16 to 21) changes with every release.
    memset(s_file + 16, 0, 6);
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
    maniMorphStream streams[5];
    Streams(s_in, s_out, streams);
    maniMorphJob job = {&m, weights, 0, kVertices, 5, 0, streams};
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
    TestRangeAndZero();
    TestBuildRefusals();
    TestOpenRefusals();
    TestJobRefusals();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
