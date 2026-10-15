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

#define PINNED_LINEAR_HASH  0x884d9250a5cd8cf6ull
#define PINNED_DUAL_HASH    0x10edaa4707ee5fafull
#define PINNED_BLENDED_HASH 0x19b3c5385f5ecb28ull

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
static float s_dual[20 * kJoints];

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
    return (maniSkinJob){&s_influences, s_palette, s_dual,      kJoints,
                         first,         count,     streamCount, streams};
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

// The rotation of a 3x3 matrix's polar decomposition by Higham's
// iteration, R <- (R + R^-T) / 2, in binary64; a mirror's R is the
// nearest rotation of the matrix with its first column negated, as the
// library splits mirrors.
static void Polar(const double a[9], double r[9])
{
    double det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[3] * (a[1] * a[8] - a[2] * a[7]) +
                 a[6] * (a[1] * a[5] - a[2] * a[4]);
    memcpy(r, a, 9 * sizeof(double));
    if (det < 0.0)
    {
        r[0] = -r[0], r[1] = -r[1], r[2] = -r[2];
    }
    for (int it = 0; it < 60; ++it)
    {
        double d = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[3] * (r[1] * r[8] - r[2] * r[7]) +
                   r[6] * (r[1] * r[5] - r[2] * r[4]);
        // R^-T's columns are the cross products of R's over det.
        double t[9] = {(r[4] * r[8] - r[5] * r[7]) / d, (r[5] * r[6] - r[3] * r[8]) / d,
                       (r[3] * r[7] - r[4] * r[6]) / d, (r[7] * r[2] - r[8] * r[1]) / d,
                       (r[8] * r[0] - r[6] * r[2]) / d, (r[6] * r[1] - r[7] * r[0]) / d,
                       (r[1] * r[5] - r[2] * r[4]) / d, (r[2] * r[3] - r[0] * r[5]) / d,
                       (r[0] * r[4] - r[1] * r[3]) / d};
        for (int k = 0; k < 9; ++k)
        {
            r[k] = 0.5 * (r[k] + t[k]);
        }
    }
}

static void QuatOf(const double r[9], double q[4])
{
    // Shepperd: the largest of w, x, y, z first.
    double tr = r[0] + r[4] + r[8];
    if (tr > 0.0)
    {
        double s = sqrt(tr + 1.0) * 2.0;
        q[3] = 0.25 * s, q[0] = (r[5] - r[7]) / s, q[1] = (r[6] - r[2]) / s,
        q[2] = (r[1] - r[3]) / s;
    }
    else if (r[0] > r[4] && r[0] > r[8])
    {
        double s = sqrt(1.0 + r[0] - r[4] - r[8]) * 2.0;
        q[3] = (r[5] - r[7]) / s, q[0] = 0.25 * s, q[1] = (r[3] + r[1]) / s,
        q[2] = (r[6] + r[2]) / s;
    }
    else if (r[4] > r[8])
    {
        double s = sqrt(1.0 + r[4] - r[0] - r[8]) * 2.0;
        q[3] = (r[6] - r[2]) / s, q[0] = (r[3] + r[1]) / s, q[1] = 0.25 * s,
        q[2] = (r[7] + r[5]) / s;
    }
    else
    {
        double s = sqrt(1.0 + r[8] - r[0] - r[4]) * 2.0;
        q[3] = (r[1] - r[3]) / s, q[0] = (r[6] + r[2]) / s, q[1] = (r[7] + r[5]) / s,
        q[2] = 0.25 * s;
    }
}

// The two-phase dual quaternion matrix of a vertex in binary64: S = R^T A
// summed, dual quaternions in the first's hemisphere, normalized, [R S | t].
static bool DualReference(uint32_t v, double m[12])
{
    double real[4] = {0}, dual[4] = {0}, sum[9] = {0}, lead[4] = {0};
    bool any = false;
    for (uint32_t slot = 0; slot < maniInfluencesPerVertex(&s_influences); ++slot)
    {
        float wf = 0.0f;
        uint32_t j = maniInfluence(&s_influences, v, slot, &wf);
        double w = (double)wf;
        if (w == 0.0)
        {
            break;
        }
        double a[9];
        for (int k = 0; k < 9; ++k)
        {
            a[k] = (double)s_palette[j].m[k];
        }
        double r[9];
        Polar(a, r);
        double q[4];
        QuatOf(r, q);
        const double t[3] = {(double)s_palette[j].m[9], (double)s_palette[j].m[10],
                             (double)s_palette[j].m[11]};
        double e[4] = {0.5 * (t[0] * q[3] + t[1] * q[2] - t[2] * q[1]),
                       0.5 * (t[1] * q[3] + t[2] * q[0] - t[0] * q[2]),
                       0.5 * (t[2] * q[3] + t[0] * q[1] - t[1] * q[0]),
                       -0.5 * (t[0] * q[0] + t[1] * q[1] + t[2] * q[2])};
        if (!any)
        {
            memcpy(lead, q, sizeof lead);
            any = true;
        }
        double sw =
            q[0] * lead[0] + q[1] * lead[1] + q[2] * lead[2] + q[3] * lead[3] < 0.0 ? -w : w;
        for (int k = 0; k < 4; ++k)
        {
            real[k] += q[k] * sw;
            dual[k] += e[k] * sw;
        }
        // S = R^T A, row i column c at 3i + c.
        for (int i = 0; i < 3; ++i)
        {
            for (int c = 0; c < 3; ++c)
            {
                sum[3 * i + c] += w * (r[3 * i] * a[3 * c] + r[3 * i + 1] * a[3 * c + 1] +
                                       r[3 * i + 2] * a[3 * c + 2]);
            }
        }
    }
    if (!any)
    {
        return false;
    }
    double n = sqrt(real[0] * real[0] + real[1] * real[1] + real[2] * real[2] + real[3] * real[3]);
    double q[4], e[4];
    for (int k = 0; k < 4; ++k)
    {
        q[k] = real[k] / n;
        e[k] = dual[k] / n;
    }
    double r[9] = {1 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] + q[3] * q[2]),
                   2 * (q[0] * q[2] - q[3] * q[1]),     2 * (q[0] * q[1] - q[3] * q[2]),
                   1 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] + q[3] * q[0]),
                   2 * (q[0] * q[2] + q[3] * q[1]),     2 * (q[1] * q[2] - q[3] * q[0]),
                   1 - 2 * (q[0] * q[0] + q[1] * q[1])};
    for (int c = 0; c < 3; ++c)
    {
        for (int k = 0; k < 3; ++k)
        {
            m[3 * c + k] = r[k] * sum[c] + r[3 + k] * sum[3 + c] + r[6 + k] * sum[6 + c];
        }
    }
    m[9] = 2 * (-e[3] * q[0] + e[0] * q[3] - e[1] * q[2] + e[2] * q[1]);
    m[10] = 2 * (-e[3] * q[1] + e[1] * q[3] - e[2] * q[0] + e[0] * q[2]);
    m[11] = 2 * (-e[3] * q[2] + e[2] * q[3] - e[0] * q[1] + e[1] * q[0]);
    return true;
}

static void TestDualAgainstReference(void)
{
    double worst = 0.0;
    for (int round = 0; round < 20; ++round)
    {
        RandomPalette();
        RandomVertices();
        CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
        maniSkinStream streams[4];
        Streams(s_in, s_out, streams);
        maniSkinJob job = Job(0, kVertices, streams, 4);
        CHECK(maniSkinDualQuaternion(&job) == mani_success, "skins");
        for (uint32_t v = 0; v < kVertices; ++v)
        {
            double m[12];
            if (!DualReference(v, m))
            {
                continue;
            }
            const Vertex* x = &s_in[v];
            for (int k = 0; k < 3; ++k)
            {
                double p = m[k] * (double)x->position[0] + m[3 + k] * (double)x->position[1] +
                           m[6 + k] * (double)x->position[2] + m[9 + k];
                double size = fabs(p) > 1.0 ? fabs(p) : 1.0;
                double e = fabs(p - (double)s_out[v].position[k]) / size;
                worst = e > worst ? e : worst;
            }
        }
    }
    CHECK(worst < 2e-5, "dual quaternion within binary32 of the binary64 reference");
    if (worst >= 2e-5)
    {
        printf("  dual apart by %g\n", worst);
    }
}

// One influence: dual quaternion skinning moves the vertex as linear does.
static void TestDualOneInfluence(void)
{
    RandomPalette();
    RandomVertices();
    CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
    maniSkinStream a[4];
    maniSkinStream b[4];
    Streams(s_in, s_out, a);
    Streams(s_in, s_other, b);
    maniSkinJob linear = Job(0, kVertices, a, 4);
    maniSkinJob dual = Job(0, kVertices, b, 4);
    CHECK(maniSkinLinear(&linear) == mani_success && maniSkinDualQuaternion(&dual) == mani_success,
          "skins");
    double worst = 0.0;
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        float w = 0.0f;
        (void)maniInfluence(&s_influences, v, 1, &w);
        if (w == 0.0f)
        {
            double e = Apart(&s_other[v], &s_out[v]);
            worst = e > worst ? e : worst;
        }
    }
    CHECK(worst < 1e-5, "one influence: as linear");
}

// The candy wrapper: a point at 1 from the axis, half on a joint turned
// +90 degrees about x and half on one turned -90. Linear blending
// collapses it onto the axis; dual quaternions keep it at 1.
static void TestCandyWrapper(void)
{
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        float sgn = j % 2 == 0 ? 1.0f : -1.0f;
        s_palette[j] = (maniAffine){{1, 0, 0, 0, 0, sgn, 0, -sgn, 0, 0, 0, 0}};
    }
    CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
    // Every vertex on joints 2k and 2k + 1 at a half each.
    static uint32_t offsets[kVertices + 1];
    static uint32_t joints[kVertices * 2];
    static float weights[kVertices * 2];
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        offsets[v] = 2 * v;
        joints[2 * v] = 2 * (v % (kJoints / 2));
        joints[2 * v + 1] = joints[2 * v] + 1;
        weights[2 * v] = 0.5f;
        weights[2 * v + 1] = 0.5f;
        s_in[v] = (Vertex){{0.3f, 1.0f, 0.0f}, {0, 1, 0}, {1, 0, 0, 1}, {0, 0, 0}};
    }
    offsets[kVertices] = 2 * kVertices;
    static _Alignas(16) uint8_t file[65536];
    size_t size = 0;
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, NULL, kVertices, 0};
    maniBuilder* builder = NewBuilder();
    maniInfluences halves;
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenInfluences(file, size, &halves).result == mani_success,
          "halves");
    maniDestroyBuilder(builder);
    maniSkinStream streams[4];
    maniSkinJob job = Job(0, kVertices, streams, 4);
    job.influences = &halves;
    Streams(s_in, s_out, streams);
    CHECK(maniSkinDualQuaternion(&job) == mani_success, "dual");
    Streams(s_in, s_other, streams);
    CHECK(maniSkinLinear(&job) == mani_success, "linear");
    bool kept = true;
    bool collapsed = true;
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        const float* p = s_out[v].position;
        const float* l = s_other[v].position;
        kept = kept && fabsf(sqrtf(p[1] * p[1] + p[2] * p[2]) - 1.0f) < 1e-6f && p[0] == 0.3f;
        collapsed = collapsed && sqrtf(l[1] * l[1] + l[2] * l[2]) < 1e-6f;
    }
    CHECK(kept, "dual quaternions keep the radius");
    CHECK(collapsed, "linear blending collapses it onto the axis");
}

// Blended: b 0 gives linear's bits, 1 dual quaternion's, between the mix.
static void TestBlended(void)
{
    static uint32_t offsets[kVertices + 1];
    static uint32_t joints[kVertices * 2];
    static float weights[kVertices * 2];
    static float blend[kVertices];
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        offsets[v] = 2 * v;
        joints[2 * v] = v % kJoints;
        joints[2 * v + 1] = (v + 3) % kJoints;
        weights[2 * v] = Uniform(0.1f, 1.0f);
        weights[2 * v + 1] = Uniform(0.1f, 1.0f);
        blend[v] = v % 3 == 0 ? 0.0f : (v % 3 == 1 ? 1.0f : Uniform(0.0f, 1.0f));
    }
    offsets[kVertices] = 2 * kVertices;
    static _Alignas(16) uint8_t file[65536];
    size_t size = 0;
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, blend, kVertices, 0};
    maniBuilder* builder = NewBuilder();
    maniInfluences blended;
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenInfluences(file, size, &blended).result == mani_success,
          "blended influences");
    maniDestroyBuilder(builder);
    RandomPalette();
    RandomVertices();
    CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
    static Vertex linear[kVertices];
    static Vertex dual[kVertices];
    maniSkinStream streams[4];
    maniSkinJob job = Job(0, kVertices, streams, 4);
    job.influences = &blended;
    Streams(s_in, linear, streams);
    CHECK(maniSkinLinear(&job) == mani_success, "linear");
    Streams(s_in, dual, streams);
    CHECK(maniSkinDualQuaternion(&job) == mani_success, "dual");
    Streams(s_in, s_out, streams);
    CHECK(maniSkinBlended(&job) == mani_success, "blended");
    bool ends = true;
    double worst = 0.0;
    for (uint32_t v = 0; v < kVertices; ++v)
    {
        float b = blend[v];
        if (b == 0.0f || b == 1.0f)
        {
            ends =
                ends && memcmp(&s_out[v], b == 0.0f ? &linear[v] : &dual[v], sizeof(Vertex)) == 0;
            continue;
        }
        for (int k = 0; k < 3; ++k)
        {
            double want = (1.0 - (double)b) * (double)linear[v].position[k] +
                          (double)b * (double)dual[v].position[k];
            double e = fabs(want - (double)s_out[v].position[k]);
            worst = e > worst ? e : worst;
        }
    }
    CHECK(ends, "blend 0 and 1: linear's and dual quaternion's bits");
    CHECK(worst < 2e-5, "between: the mix of the two");
    uint64_t hash = maniHash64(MANI_HASH_INIT, s_out, (int32_t)sizeof s_out);
    CHECK(hash == PINNED_BLENDED_HASH, "pinned blended");
    if (hash != PINNED_BLENDED_HASH)
    {
        printf("blended hash 0x%016llx\n", (unsigned long long)hash);
    }
}

static void TestDualPalette(void)
{
    RandomPalette();
    CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
    double worst = 0.0;
    bool positive = true;
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        const float* d = s_dual + 20 * j;
        positive = positive && d[3] >= 0.0f && d[11] == 0.0f && d[15] == 0.0f && d[19] == 0.0f;
        double q[4] = {(double)d[0], (double)d[1], (double)d[2], (double)d[3]};
        double r[9] = {1 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] + q[3] * q[2]),
                       2 * (q[0] * q[2] - q[3] * q[1]),     2 * (q[0] * q[1] - q[3] * q[2]),
                       1 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] + q[3] * q[0]),
                       2 * (q[0] * q[2] + q[3] * q[1]),     2 * (q[1] * q[2] - q[3] * q[0]),
                       1 - 2 * (q[0] * q[0] + q[1] * q[1])};
        // R S back to A.
        for (int c = 0; c < 3; ++c)
        {
            for (int k = 0; k < 3; ++k)
            {
                double a = r[k] * (double)d[8 + c] + r[3 + k] * (double)d[12 + c] +
                           r[6 + k] * (double)d[16 + c];
                double e = fabs(a - (double)s_palette[j].m[3 * c + k]);
                worst = e > worst ? e : worst;
            }
        }
    }
    CHECK(positive, "w at least 0, rows padded with 0");
    CHECK(worst < 1e-5, "R S gives the matrix back");
    float kept[20 * kJoints];
    memcpy(kept, s_dual, sizeof kept);
    s_palette[3].m[4] = NAN;
    CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_errorInvalid,
          "a matrix not finite");
    CHECK(memcmp(kept, s_dual, sizeof kept) == 0, "out unchanged");
    CHECK(maniPaletteDualQuaternions(NULL, 1, s_dual) == mani_errorInvalid, "no palette");
    maniSkinStream streams[4];
    Streams(s_in, s_out, streams);
    maniSkinJob job = Job(0, kVertices, streams, 4);
    job.dualQuaternions = NULL;
    CHECK(maniSkinDualQuaternion(&job) == mani_errorInvalid &&
              maniSkinBlended(&job) == mani_errorInvalid,
          "no dual quaternions");
    job.dualQuaternions = s_dual;
    job.palette = NULL;
    CHECK(maniSkinBlended(&job) == mani_errorInvalid, "blended needs both");
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
    CHECK(maniPaletteDualQuaternions(s_palette, kJoints, s_dual) == mani_success &&
              maniSkinDualQuaternion(&job) == mani_success,
          "dual");
    hash = maniHash64(MANI_HASH_INIT, s_out, (int32_t)sizeof s_out);
    CHECK(hash == PINNED_DUAL_HASH, "pinned dual quaternion");
    if (hash != PINNED_DUAL_HASH)
    {
        printf("dual hash 0x%016llx\n", (unsigned long long)hash);
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
    TestDualAgainstReference();
    TestDualOneInfluence();
    TestCandyWrapper();
    TestDualPalette();
    TestBlended();
    return s_failures == 0 ? 0 : 1;
}
