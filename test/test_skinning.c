// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skinning jobs (mani-0021): linear blend skinning against a binary64
// reference (normals by the true inverse transpose) over random
// influences and palettes with scale, shear and mirrors; vertices without
// weight copied; ranges, strides and in-place runs giving the same bits;
// refusals leaving the outputs; row palettes; the bits pinned.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define PINNED_LINEAR_HASH 0x884d9250a5cd8cf6ull

enum
{
    kBones = 9,
    kJoints = 8,
    kVertices = 300,
};

static _Alignas(16) uint8_t s_skeletonFile[4096];
static _Alignas(16) uint8_t s_skinFile[4096];
static _Alignas(16) uint8_t s_file[65536];
static size_t s_size;
static maniSkeleton s_skeleton;
static maniSkin s_skin;
static maniInfluences s_influences;
static maniAffine s_palette[kJoints];

// Vertex streams, interleaved: position, normal, tangent (four floats),
// and a further point.
typedef struct Vertex
{
    float position[3];
    float normal[3];
    float tangent[4];
    float extra[3];
} Vertex;

static Vertex s_in[kVertices];
static Vertex s_out[kVertices];
static Vertex s_other[kVertices];

static uint64_t s_random = 0x9E3779B97F4A7C15ull;

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

static void OpenSkin(void)
{
    maniBoneSource bones[kBones];
    static char names[kBones][4];
    for (uint32_t i = 0; i < kBones; ++i)
    {
        names[i][0] = 'b';
        names[i][1] = (char)('a' + i);
        bones[i] = (maniBoneSource){names[i],
                                    2,
                                    i == 0 ? -1 : (int32_t)(i - 1),
                                    {{0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1, 1, 1}}};
    }
    maniBuilder* builder = NewBuilder();
    maniSkeletonSource source = {bones, kBones, 0};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
                  mani_success &&
              maniOpenSkeleton(s_skeletonFile, size, &s_skeleton).result == mani_success,
          "skeleton");
    uint32_t joints[kJoints];
    maniAffine binds[kJoints];
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        joints[j] = j + 1;
        binds[j] = (maniAffine){{1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0}};
    }
    maniSkinSource skin = {&s_skeleton, joints, binds, kJoints, 0};
    CHECK(maniBuildSkin(builder, &skin).result == mani_success &&
              maniCopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) == mani_success &&
              maniOpenSkin(s_skinFile, size, &s_skin).result == mani_success,
          "skin");
    maniDestroyBuilder(builder);
}

// Random influences: 0 to 9 a vertex, distinct joints, one vertex in ten
// without weight.
static void BuildInfluences(void)
{
    static uint32_t offsets[kVertices + 1];
    static uint32_t joints[kVertices * 9];
    static float weights[kVertices * 9];
    uint32_t n = 0;
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        offsets[v] = n;
        uint32_t count = v % 10 == 0 ? 0 : 1 + Next() % 8;
        uint32_t start = Next() % kJoints;
        for (uint32_t i = 0; i < count; ++i)
        {
            joints[n] = (start + i) % kJoints;
            weights[n] = Uniform(0.01f, 1.0f);
            ++n;
        }
    }
    offsets[kVertices] = n;
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, NULL, kVertices, 0};
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
              maniOpenInfluences(s_file, s_size, &s_influences).result == mani_success,
          "influences");
    maniDestroyBuilder(builder);
}

// Random palette matrices: a rotation, a scale (non-uniform, sometimes
// mirrored), a shear, a translation.
static void RandomPalette(void)
{
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        float x = Uniform(-1, 1), y = Uniform(-1, 1), z = Uniform(-1, 1), w = Uniform(-1, 1);
        float n = sqrtf(x * x + y * y + z * z + w * w);
        x /= n, y /= n, z /= n, w /= n;
        float r[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + w * z),     2 * (x * z - w * y),
                      2 * (x * y - w * z),     1 - 2 * (x * x + z * z), 2 * (y * z + w * x),
                      2 * (x * z + w * y),     2 * (y * z - w * x),     1 - 2 * (x * x + y * y)};
        float s[3] = {Uniform(0.5f, 2.0f), Uniform(0.5f, 2.0f), Uniform(0.5f, 2.0f)};
        s[1] = j % 3 == 0 ? -s[1] : s[1];
        float shear = j % 2 == 0 ? Uniform(-0.3f, 0.3f) : 0.0f;
        maniAffine* m = &s_palette[j];
        for (int c = 0; c < 3; ++c)
        {
            for (int k = 0; k < 3; ++k)
            {
                m->m[3 * c + k] = r[3 * c + k] * s[c] + (c == 1 ? shear * r[k] : 0.0f);
            }
        }
        m->m[9] = Uniform(-2, 2);
        m->m[10] = Uniform(-2, 2);
        m->m[11] = Uniform(-2, 2);
    }
}

static void RandomVertices(void)
{
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        Vertex* x = &s_in[v];
        for (int k = 0; k < 3; ++k)
        {
            x->position[k] = Uniform(-1, 1);
            x->normal[k] = Uniform(-1, 1);
            x->tangent[k] = Uniform(-1, 1);
            x->extra[k] = Uniform(-1, 1);
        }
        x->tangent[3] = v % 2 == 0 ? 1.0f : -1.0f;
    }
}

static maniSkinStream Stream(const void* in, void* out, maniStreamKind kind)
{
    return (maniSkinStream){in, out, sizeof(Vertex), sizeof(Vertex), kind, 0};
}

static maniSkinJob Job(uint32_t first, uint32_t count, const maniSkinStream* streams,
                       uint32_t streamCount)
{
    return (maniSkinJob){&s_influences, s_palette, kJoints, first, count, streamCount, streams};
}

static void Streams(const Vertex* in, Vertex* out, maniSkinStream streams[4])
{
    streams[0] = Stream(in->position, out->position, maniStreamPoint);
    streams[1] = Stream(in->normal, out->normal, maniStreamNormal);
    streams[2] = Stream(in->tangent, out->tangent, maniStreamTangent);
    streams[3] = Stream(in->extra, out->extra, maniStreamPoint);
}

// The binary64 reference of one vertex: the blended matrix, positions,
// directions renormalized, normals by the true inverse transpose.
static void Reference(uint32_t v, Vertex* want)
{
    double m[12] = {0};
    double total = 0.0;
    for (uint32_t slot = 0; slot < maniInfluencesPerVertex(&s_influences); ++slot)
    {
        float w = 0.0f;
        uint32_t j = maniInfluence(&s_influences, v, slot, &w);
        for (int k = 0; k < 12; ++k)
        {
            m[k] += (double)s_palette[j].m[k] * (double)w;
        }
        total += (double)w;
    }
    *want = s_in[v];
    if (total == 0.0)
    {
        return;
    }
    const Vertex* x = &s_in[v];
    double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[3] * (m[1] * m[8] - m[2] * m[7]) +
                 m[6] * (m[1] * m[5] - m[2] * m[4]);
    // Inverse transpose's columns: the cross products over det.
    double it[9] = {(m[4] * m[8] - m[5] * m[7]) / det, (m[5] * m[6] - m[3] * m[8]) / det,
                    (m[3] * m[7] - m[4] * m[6]) / det, (m[7] * m[2] - m[8] * m[1]) / det,
                    (m[8] * m[0] - m[6] * m[2]) / det, (m[6] * m[1] - m[7] * m[0]) / det,
                    (m[1] * m[5] - m[2] * m[4]) / det, (m[2] * m[3] - m[0] * m[5]) / det,
                    (m[0] * m[4] - m[1] * m[3]) / det};
    double xp[3];
    double xe[3];
    double xn[3];
    double xt[3];
    for (int k = 0; k < 3; ++k)
    {
        xp[k] = (double)x->position[k];
        xe[k] = (double)x->extra[k];
        xn[k] = (double)x->normal[k];
        xt[k] = (double)x->tangent[k];
    }
    double p[3];
    double e[3];
    double n[3];
    double t[3];
    for (int k = 0; k < 3; ++k)
    {
        p[k] = m[k] * xp[0] + m[3 + k] * xp[1] + m[6 + k] * xp[2] + m[9 + k];
        e[k] = m[k] * xe[0] + m[3 + k] * xe[1] + m[6 + k] * xe[2] + m[9 + k];
        n[k] = it[k] * xn[0] + it[3 + k] * xn[1] + it[6 + k] * xn[2];
        t[k] = m[k] * xt[0] + m[3 + k] * xt[1] + m[6 + k] * xt[2];
    }
    double ln = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    double lt = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    for (int k = 0; k < 3; ++k)
    {
        want->position[k] = (float)p[k];
        want->extra[k] = (float)e[k];
        want->normal[k] = (float)(n[k] / ln);
        want->tangent[k] = (float)(t[k] / lt);
    }
}

static double Apart(const Vertex* a, const Vertex* b)
{
    const float* x = &a->position[0];
    const float* y = &b->position[0];
    double worst = 0.0;
    for (int k = 0; k < 13; ++k)
    {
        double size = fabs((double)y[k]) > 1.0 ? fabs((double)y[k]) : 1.0;
        double e = fabs((double)x[k] - (double)y[k]) / size;
        worst = e > worst ? e : worst;
    }
    return worst;
}

static void TestLinearAgainstReference(void)
{
    double worst = 0.0;
    bool copied = true;
    for (int round = 0; round < 20; ++round)
    {
        RandomPalette();
        RandomVertices();
        maniSkinStream streams[4];
        Streams(s_in, s_out, streams);
        maniSkinJob job = Job(0, kVertices, streams, 4);
        CHECK(maniSkinLinear(&job) == mani_success, "skins");
        for (uint32_t v = 0; v < kVertices; ++v)
        {
            Vertex want;
            Reference(v, &want);
            double e = Apart(&s_out[v], &want);
            worst = e > worst ? e : worst;
            copied = copied && (v % 10 != 0 || memcmp(&s_out[v], &s_in[v], sizeof(Vertex)) == 0);
        }
    }
    CHECK(worst < 2e-5, "within binary32 of the binary64 reference");
    if (worst >= 2e-5)
    {
        printf("  linear apart by %g\n", worst);
    }
    CHECK(copied, "a vertex without weight is copied");
}

// Split into ranges, in place, and over separate strides: the same bits.
static void TestRangesAndPlaces(void)
{
    RandomPalette();
    RandomVertices();
    maniSkinStream streams[4];
    Streams(s_in, s_out, streams);
    maniSkinJob whole = Job(0, kVertices, streams, 4);
    CHECK(maniSkinLinear(&whole) == mani_success, "whole");
    memset(s_other, 0, sizeof s_other);
    const uint32_t cuts[4] = {0, 7, 150, kVertices};
    for (int i = 0; i < 3; ++i)
    {
        maniSkinStream part[4];
        Streams(&s_in[cuts[i]], &s_other[cuts[i]], part);
        maniSkinJob job = Job(cuts[i], cuts[i + 1] - cuts[i], part, 4);
        CHECK(maniSkinLinear(&job) == mani_success, "a range");
    }
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "ranges give the same bits");
    memcpy(s_other, s_in, sizeof s_other);
    maniSkinStream place[4];
    Streams(s_other, s_other, place);
    maniSkinJob inPlace = Job(0, kVertices, place, 4);
    CHECK(maniSkinLinear(&inPlace) == mani_success, "in place");
    CHECK(memcmp(s_out, s_other, sizeof s_out) == 0, "in place gives the same bits");
    // Positions alone, packed tight.
    static float packedIn[kVertices * 3];
    static float packedOut[kVertices * 3];
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        memcpy(&packedIn[3 * v], s_in[v].position, 12);
    }
    maniSkinStream packed = {packedIn, packedOut, 12, 12, maniStreamPoint, 0};
    maniSkinJob tight = Job(0, kVertices, &packed, 1);
    CHECK(maniSkinLinear(&tight) == mani_success, "packed");
    bool same = true;
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        same = same && memcmp(&packedOut[3 * v], s_out[v].position, 12) == 0;
    }
    CHECK(same, "a packed stream gives the same bits");
}

// A palette collapsing every axis: directions and normals come out zero.
static void TestCollapse(void)
{
    RandomVertices();
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        s_palette[j] = (maniAffine){{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3}};
    }
    maniSkinStream streams[4];
    Streams(s_in, s_out, streams);
    maniSkinJob job = Job(0, kVertices, streams, 4);
    CHECK(maniSkinLinear(&job) == mani_success, "skins");
    const Vertex* v = &s_out[1];
    CHECK(v->normal[0] == 0.0f && v->normal[1] == 0.0f && v->tangent[2] == 0.0f &&
              v->tangent[3] == s_in[1].tangent[3] && v->position[0] == 1.0f &&
              v->position[2] == 3.0f,
          "collapsed: points to the translation, directions zero, w kept");
}

static void ExpectRefused(const maniSkinJob* job, const char* what)
{
    memcpy(s_other, s_out, sizeof s_other);
    CHECK(maniSkinLinear(job) == mani_errorInvalid, what);
    CHECK(memcmp(s_other, s_out, sizeof s_other) == 0, "nothing written");
}

static void TestRefusals(void)
{
    RandomPalette();
    maniSkinStream streams[4];
    Streams(s_in, s_out, streams);
    maniSkinJob job = Job(0, kVertices, streams, 4);
    job.paletteCount = kJoints - 1;
    ExpectRefused(&job, "a palette count not the skin's");
    job = Job(kVertices - 5, 6, streams, 4);
    ExpectRefused(&job, "a range past the vertices");
    job = Job(kVertices + 1, 0, streams, 4);
    ExpectRefused(&job, "a first past the vertices");
    job = Job(0, kVertices, streams, 4);
    streams[2].inStride = 12;
    ExpectRefused(&job, "a tangent stride under 16");
    streams[2].inStride = sizeof(Vertex);
    streams[1].kind = 4;
    ExpectRefused(&job, "an unknown kind");
    streams[1].kind = maniStreamNormal;
    streams[0].out = NULL;
    ExpectRefused(&job, "no output");
    streams[0].out = s_out->position;
    job.palette = NULL;
    ExpectRefused(&job, "no palette");
    job.palette = s_palette;
    job.streams = NULL;
    ExpectRefused(&job, "no streams");
    CHECK(maniSkinLinear(NULL) == mani_errorInvalid, "no job");
    job = Job(kVertices, 0, streams, 4);
    CHECK(maniSkinLinear(&job) == mani_success, "an empty range at the end");
}

static void TestRows(void)
{
    RandomPalette();
    float rows[12 * kJoints];
    CHECK(maniPaletteRows(s_palette, kJoints, rows) == mani_success, "rows");
    bool right = true;
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        const float* m = s_palette[j].m;
        const float* r = rows + 12 * j;
        right = right && r[0] == m[0] && r[1] == m[3] && r[2] == m[6] && r[3] == m[9] &&
                r[4] == m[1] && r[7] == m[10] && r[8] == m[2] && r[11] == m[11];
    }
    CHECK(right, "row-major 3x4, translation last");
    CHECK(maniPaletteRows(NULL, 1, rows) == mani_errorInvalid, "no palette");
    CHECK(maniPaletteRows(NULL, 0, NULL) == mani_success, "nothing to write");
}

static void TestPinned(void)
{
    s_random = 0x0123456789ABCDEFull;
    RandomPalette();
    RandomVertices();
    maniSkinStream streams[4];
    Streams(s_in, s_out, streams);
    maniSkinJob job = Job(0, kVertices, streams, 4);
    CHECK(maniSkinLinear(&job) == mani_success, "skins");
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_out, (int32_t)sizeof s_out);
    CHECK(hash == PINNED_LINEAR_HASH, "pinned linear");
    if (hash != PINNED_LINEAR_HASH)
    {
        printf("linear hash 0x%016llx\n", (unsigned long long)hash);
    }
}

int main(void)
{
    OpenSkin();
    BuildInfluences();
    TestLinearAgainstReference();
    TestRangesAndPlaces();
    TestCollapse();
    TestRefusals();
    TestRows();
    TestPinned();
    return s_failures == 0 ? 0 : 1;
}
