// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Skinning jobs: linear blend skinning against a binary64
// reference (normals by the true inverse transpose) over random
// influences and palettes with scale, shear and mirrors; vertices without
// weight copied; ranges, strides and in-place runs giving the same bits;
// refusals leaving the outputs; row palettes; the bits pinned.

#include "source_padding.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/morph.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"
#include "maul-anim/skinning.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// The skinning scene: linear, dual quaternion and blended skinning, the
// centers file and job, and morph targets then skinning, hashed in the
// order main runs them; every platform and backend must give this value.
#define PINNED_SCENE_HASH 0x82576defee4a4362ull

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

static uint64_t s_scene = MANI_HASH_INIT;

// Adds an output to the pinned scene.
static void Pin(const void* bytes, size_t size)
{
    s_scene = maniHash64(s_scene, bytes, (int32_t)size);
}

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
    maniSkeletonSource source = {bones, kBones};
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
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
    maniSkinSource skin = {&s_skeleton, joints, binds, kJoints};
    CHECK(maniBuildSkin(builder, &skin).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_skinFile, sizeof s_skinFile, &size) ==
                  mani_success &&
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
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, NULL, kVertices};
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_file, sizeof s_file, &s_size) == mani_success &&
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
    return (maniSkinStream){in, out, sizeof(Vertex), sizeof(Vertex), kind};
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
    for (uint32_t slot = 0; slot < maniInfluences_GetPerVertex(&s_influences); ++slot)
    {
        float w = 0.0f;
        uint32_t j = maniInfluences_GetInfluence(&s_influences, v, slot, &w);
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
    maniSkinStream packed = {packedIn, packedOut, 12, 12, maniStreamPoint};
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
    CHECK(maniConvertPaletteToRows(s_palette, kJoints, rows) == mani_success, "rows");
    bool right = true;
    for (uint32_t j = 0; j < kJoints; ++j)
    {
        const float* m = s_palette[j].m;
        const float* r = rows + 12 * j;
        right = right && r[0] == m[0] && r[1] == m[3] && r[2] == m[6] && r[3] == m[9] &&
                r[4] == m[1] && r[7] == m[10] && r[8] == m[2] && r[11] == m[11];
    }
    CHECK(right, "row-major 3x4, translation last");
    CHECK(maniConvertPaletteToRows(NULL, 1, rows) == mani_errorInvalid, "no palette");
    CHECK(maniConvertPaletteToRows(NULL, 0, NULL) == mani_success, "nothing to write");
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
    for (uint32_t slot = 0; slot < maniInfluences_GetPerVertex(&s_influences); ++slot)
    {
        float wf = 0.0f;
        uint32_t j = maniInfluences_GetInfluence(&s_influences, v, slot, &wf);
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
        CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success,
              "dual");
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
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
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
        (void)maniInfluences_GetInfluence(&s_influences, v, 1, &w);
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
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
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
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, NULL, kVertices};
    maniBuilder* builder = NewBuilder();
    maniInfluences halves;
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
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
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, blend, kVertices};
    maniBuilder* builder = NewBuilder();
    maniInfluences blended;
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenInfluences(file, size, &blended).result == mani_success,
          "blended influences");
    maniDestroyBuilder(builder);
    RandomPalette();
    RandomVertices();
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
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
    Pin(s_out, sizeof s_out);
}

static void TestDualPalette(void)
{
    RandomPalette();
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
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
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_errorInvalid,
          "a matrix not finite");
    CHECK(memcmp(kept, s_dual, sizeof kept) == 0, "out unchanged");
    CHECK(maniConvertPaletteToDualQuaternions(NULL, 1, s_dual) == mani_errorInvalid, "no palette");
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
    Pin(s_out, sizeof s_out);
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success &&
              maniSkinDualQuaternion(&job) == mani_success,
          "dual");
    Pin(s_out, sizeof s_out);
}

// Morph targets over the vertices: a sparse target of positions and
// normals given out of order with a vertex twice, and a channel of
// in-betweens at -1, 0.5 and 1, dense positions, the last with tangents.
static const uint32_t kSparse[3] = {120, 7, 120};
static const float kSparsePositions[9] = {0.5f, 0, -0.25f, 0, 1, 0, 0.125f, 0.5f, 0};
static const float kSparseNormals[9] = {0, 0, 1, 1, 0, 0, 0, 0.5f, 0.5f};
static float s_ramp[3][3 * kVertices];
static float s_rampTangents[3 * kVertices];
static _Alignas(16) uint8_t s_morphFile[32768];

static _Alignas(64) uint8_t s_poseMemory[4096];

// A pose's palette, then morph targets, then skinning by that palette.
static void TestMorphThenSkin(void)
{
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success,
          "pose");
    for (uint32_t b = 0; b < kBones; ++b)
    {
        // One draw a statement: an initializer list's order is unspecified.
        float r[10];
        for (int k = 0; k < 10; ++k)
        {
            r[k] = k < 7 ? Uniform(-1, 1) : Uniform(0.5f, 2.0f);
        }
        float n = sqrtf(r[3] * r[3] + r[4] * r[4] + r[5] * r[5] + r[6] * r[6]);
        maniTransform t = {
            {r[0], r[1], r[2]}, {r[3] / n, r[4] / n, r[5] / n, r[6] / n}, {r[7], r[8], r[9]}};
        maniPose_SetLocal(pose, b, t);
    }
    maniAffine saved[kJoints];
    memcpy(saved, s_palette, sizeof saved);
    CHECK(maniPose_ComputeModel(pose, &s_skeleton) == mani_success &&
              maniPose_ComputePalette(pose, &s_skeleton, &s_skin, s_palette, kJoints, NULL) ==
                  mani_success,
          "palette");
    Pin(s_palette, sizeof s_palette);
    for (uint32_t i = 0; i < 3 * kVertices; ++i)
    {
        for (int k = 0; k < 3; ++k)
        {
            s_ramp[k][i] = (float)(k + 1) * 0.0625f * (float)((i % 5) + 1);
        }
        s_rampTangents[i] = 0.03125f * (float)(i % 9);
    }
    const float* sparse[3] = {kSparsePositions, kSparseNormals, NULL};
    const float* ramp[3][3] = {
        {s_ramp[0], NULL, NULL}, {s_ramp[1], NULL, NULL}, {s_ramp[2], NULL, s_rampTangents}};
    maniMorphTargetSource smile[1] = {{1.0f, 3, kSparse, sparse, NULL, 0}};
    maniMorphTargetSource ramps[3] = {{-1.0f, kVertices, NULL, ramp[0], NULL, 0},
                                      {0.5f, kVertices, NULL, ramp[1], NULL, 0},
                                      {1.0f, kVertices, NULL, ramp[2], NULL, 0}};
    maniMorphChannelSource channels[2] = {{"smile", 5, 1, smile}, {"ramp", 4, 3, ramps}};
    maniMorphAttributeSource attributes[3] = {
        {maniMorphPosition, 0}, {maniMorphNormal, 0}, {maniMorphTangent, 0}};
    maniMorphSource source = {channels, 2, kVertices, attributes, 3};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildMorphTargets(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_morphFile, sizeof s_morphFile, &size) ==
                  mani_success,
          "morph targets built");
    maniDestroyBuilder(builder);
    maniMorphTargets m;
    CHECK(maniOpenMorphTargets(s_morphFile, size, &m).result == mani_success, "morph targets open");
    const float channelWeights[2] = {0.7f, 0.8f};
    float weights[4];
    CHECK(maniMorphTargets_ComputeTargetWeights(&m, channelWeights, weights,
                                                maniMorphTargets_GetTargetCount(&m),
                                                NULL) == mani_success,
          "weights");
    memcpy(s_other, s_in, sizeof s_other);
    maniMorphStream morph[3] = {
        {s_in->position, s_other->position, sizeof(Vertex), sizeof(Vertex), maniMorphPosition, 0,
         3},
        {s_in->normal, s_other->normal, sizeof(Vertex), sizeof(Vertex), maniMorphNormal, 0, 3},
        {s_in->tangent, s_other->tangent, sizeof(Vertex), sizeof(Vertex), maniMorphTangent, 0, 4}};
    maniMorphJob morphJob = {&m, weights, 0, kVertices, 3, morph};
    CHECK(maniApplyMorphTargets(&morphJob) == mani_success, "morphs");
    maniSkinStream streams[4];
    Streams(s_other, s_out, streams);
    maniSkinJob job = Job(0, kVertices, streams, 4);
    CHECK(maniSkinLinear(&job) == mani_success, "skins the morphed vertices");
    // The writer's library version (bytes 16 to 21) changes with every release.
    memset(s_morphFile + 16, 0, 6);
    Pin(s_morphFile, size);
    Pin(s_other, sizeof s_other);
    Pin(s_out, sizeof s_out);
    memcpy(s_palette, saved, sizeof saved);
}

// A cylinder along y, 24 rings of 12 vertices, each ring weighted between
// the two joints its height falls between (the first ring on joint 0
// alone), and its triangles.
enum
{
    kRings = 24,
    kSegments = 12,
    kMesh = kRings * kSegments,
    kTriangles = (kRings - 1) * kSegments * 2,
};

static float s_rest[kMesh][3];
static uint32_t s_triangles[kTriangles * 3];
static _Alignas(16) uint8_t s_meshFile[65536];
static _Alignas(16) uint8_t s_centersFile[65536];
static size_t s_centersSize;
static maniInfluences s_mesh;
static maniInfluences s_centered;

static void BuildMesh(void)
{
    static uint32_t offsets[kMesh + 1];
    static uint32_t joints[kMesh * 2];
    static float weights[kMesh * 2];
    uint32_t n = 0;
    for (uint32_t r = 0; r < kRings; ++r)
    {
        float h = 7.0f * (float)r / (float)(kRings - 1);
        uint32_t a = (uint32_t)h < 7 ? (uint32_t)h : 6;
        float f = h - (float)a;
        for (uint32_t k = 0; k < kSegments; ++k)
        {
            uint32_t v = r * kSegments + k;
            float angle = 6.2831853f * (float)k / (float)kSegments;
            s_rest[v][0] = 0.4f * cosf(angle);
            s_rest[v][1] = h;
            s_rest[v][2] = 0.4f * sinf(angle);
            offsets[v] = n;
            joints[n] = a;
            weights[n++] = r == 0 ? 1.0f : 1.0f - f;
            if (r > 0)
            {
                joints[n] = a + 1;
                weights[n++] = f;
            }
        }
    }
    offsets[kMesh] = n;
    uint32_t t = 0;
    for (uint32_t r = 0; r + 1 < kRings; ++r)
    {
        for (uint32_t k = 0; k < kSegments; ++k)
        {
            uint32_t a = r * kSegments + k;
            uint32_t b = r * kSegments + (k + 1) % kSegments;
            uint32_t c = a + kSegments;
            uint32_t d = b + kSegments;
            const uint32_t two[6] = {a, b, c, b, d, c};
            memcpy(&s_triangles[3 * t], two, sizeof two);
            t += 2;
        }
    }
    maniInfluenceSource source = {&s_skin, offsets, joints, weights, NULL, kMesh};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildInfluences(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_meshFile, sizeof s_meshFile, &size) ==
                  mani_success &&
              maniOpenInfluences(s_meshFile, size, &s_mesh).result == mani_success,
          "mesh influences");
    maniCenterSource centers = {&s_mesh, &s_rest[0][0], 12, s_triangles, kTriangles, 0.0f};
    CHECK(maniBuildCenters(builder, &centers).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_centersFile, sizeof s_centersFile, &s_centersSize) ==
                  mani_success &&
              maniOpenInfluences(s_centersFile, s_centersSize, &s_centered).result == mani_success,
          "centers");
    maniDestroyBuilder(builder);
}

// The same centers whatever bytes pad their source.
static void TestCentersPaddingIgnored(void)
{
    uint8_t* built[2];
    size_t sizes[2];
    for (int k = 0; k < 2; ++k)
    {
        maniCenterSource source = {&s_mesh, &s_rest[0][0], 12, s_triangles, kTriangles, 0.0f};
        TEST_PAD(&source, 1, kFieldsCenterSource, kTestPadPatterns[k]);
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        CHECK(maniBuildCenters(builder, &source).result == mani_success, "builds padded");
        built[k] = TestCopyBuilt(builder, &sizes[k]);
        maniDestroyBuilder(builder);
    }
    CHECK(TestSameBuilt(built, sizes), "the same centers under either padding");
}

// Eq. 4 by brute force in binary64: every triangle, every pair, libm's exp.
static void BruteCenter(uint32_t vertex, double out[3])
{
    double wv[kJoints] = {0};
    for (uint32_t slot = 0; slot < maniInfluences_GetPerVertex(&s_mesh); ++slot)
    {
        float w = 0.0f;
        uint32_t j = maniInfluences_GetInfluence(&s_mesh, vertex, slot, &w);
        wv[j] += (double)w;
    }
    double sum[3] = {0, 0, 0};
    double total = 0.0;
    for (uint32_t t = 0; t < kTriangles; ++t)
    {
        double wt[kJoints] = {0};
        const uint32_t* c = &s_triangles[3 * t];
        for (int i = 0; i < 3; ++i)
        {
            for (uint32_t slot = 0; slot < maniInfluences_GetPerVertex(&s_mesh); ++slot)
            {
                float w = 0.0f;
                uint32_t j = maniInfluences_GetInfluence(&s_mesh, c[i], slot, &w);
                wt[j] += (double)w / 3.0;
            }
        }
        double similarity = 0.0;
        for (int j = 0; j < kJoints; ++j)
        {
            for (int k = j + 1; k < kJoints; ++k)
            {
                double d = wv[j] * wt[k] - wv[k] * wt[j];
                similarity += wv[j] * wv[k] * wt[j] * wt[k] * exp(-(d * d) / 0.01);
            }
        }
        double e1[3], e2[3], centroid[3];
        for (int k = 0; k < 3; ++k)
        {
            e1[k] = (double)s_rest[c[1]][k] - (double)s_rest[c[0]][k];
            e2[k] = (double)s_rest[c[2]][k] - (double)s_rest[c[0]][k];
            centroid[k] =
                ((double)s_rest[c[0]][k] + (double)s_rest[c[1]][k] + (double)s_rest[c[2]][k]) / 3.0;
        }
        double x = e1[1] * e2[2] - e1[2] * e2[1];
        double y = e1[2] * e2[0] - e1[0] * e2[2];
        double z = e1[0] * e2[1] - e1[1] * e2[0];
        double area = 0.5 * sqrt(x * x + y * y + z * z);
        for (int k = 0; k < 3; ++k)
        {
            sum[k] += similarity * area * centroid[k];
        }
        total += similarity * area;
    }
    for (int k = 0; k < 3; ++k)
    {
        out[k] = total > 0.0 ? sum[k] / total : (double)s_rest[vertex][k];
    }
}

static void TestCentersBuilt(void)
{
    double worst = 0.0;
    bool rigid = true;
    for (uint32_t v = 0; v < kMesh; ++v)
    {
        double want[3];
        BruteCenter(v, want);
        maniVec3 got;
        CHECK(maniInfluences_GetCenter(&s_centered, v, &got), "a center");
        const float g[3] = {got.x, got.y, got.z};
        for (int k = 0; k < 3; ++k)
        {
            double e = fabs((double)g[k] - want[k]);
            worst = e > worst ? e : worst;
        }
        if (v < kSegments)
        {
            rigid = rigid && g[0] == s_rest[v][0] && g[1] == s_rest[v][1] && g[2] == s_rest[v][2];
        }
    }
    CHECK(worst < 2e-5, "centers as Eq. 4 by brute force");
    if (worst >= 2e-5)
    {
        printf("  centers apart by %g\n", worst);
    }
    CHECK(rigid, "a vertex on one joint keeps its position");
    // The copy keeps everything else.
    CHECK(maniInfluences_GetVertexCount(&s_centered) == kMesh &&
              maniInfluences_GetPerVertex(&s_centered) == maniInfluences_GetPerVertex(&s_mesh) &&
              maniInfluences_IsSkinFit(&s_centered, &s_skin),
          "the influences kept");
    static uint8_t copy[65536];
    memcpy(copy, s_centersFile, s_centersSize);
    // The writer's library version (bytes 16 to 21) changes with every release.
    memset(copy + 16, 0, 6);
    Pin(copy, s_centersSize);
}

static void TestCentersRefused(void)
{
    maniBuilder* builder = NewBuilder();
    maniCenterSource source = {&s_mesh, &s_rest[0][0], 12, s_triangles, kTriangles, 0.0f};
    source.stride = 8;
    maniBuildResult r = maniBuildCenters(builder, &source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonCount, "a short stride");
    source.stride = 12;
    source.sigma = -1.0f;
    r = maniBuildCenters(builder, &source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonOption, "a negative sigma");
    source.sigma = NAN;
    r = maniBuildCenters(builder, &source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonOption, "a NaN sigma");
    source.sigma = 0.0f;
    float kept = s_rest[17][1];
    s_rest[17][1] = INFINITY;
    r = maniBuildCenters(builder, &source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonNotFinite && r.item == 17,
          "a position not finite");
    s_rest[17][1] = kept;
    s_triangles[3 * 5 + 2] = kMesh;
    r = maniBuildCenters(builder, &source);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonTriangle && r.item == 5,
          "a triangle past the vertices");
    s_triangles[3 * 5 + 2] = 0;
    BuildMesh();
    CHECK(maniBuildCenters(builder, NULL).result == mani_errorInvalid, "no source");
    source.positions = NULL;
    CHECK(maniBuildCenters(builder, &source).result == mani_errorInvalid, "no positions");
    maniDestroyBuilder(builder);
    // The job needs centers.
    maniSkinStream streams[4];
    Streams(s_in, s_out, streams);
    maniSkinJob job = Job(0, kMesh, streams, 4);
    job.influences = &s_mesh;
    CHECK(maniSkinCenters(&job) == mani_errorInvalid, "influences without centers");
}

// The job: one influence as linear; the reference formula in binary64;
// the candy wrapper about centers on the axis.
static void TestCentersJob(void)
{
    RandomPalette();
    CHECK(maniConvertPaletteToDualQuaternions(s_palette, kJoints, s_dual) == mani_success, "dual");
    for (uint32_t v = 0; v < kMesh; ++v)
    {
        s_in[v] = (Vertex){{s_rest[v][0], s_rest[v][1], s_rest[v][2]},
                           {s_rest[v][0], 0.0f, s_rest[v][2]},
                           {0.0f, 1.0f, 0.0f, 1.0f},
                           {0.0f, 0.0f, 0.0f}};
    }
    maniSkinStream a[4];
    maniSkinStream b[4];
    Streams(s_in, s_out, a);
    Streams(s_in, s_other, b);
    maniSkinJob centers = Job(0, kMesh, a, 4);
    maniSkinJob linear = Job(0, kMesh, b, 4);
    centers.influences = &s_centered;
    linear.influences = &s_centered;
    CHECK(maniSkinCenters(&centers) == mani_success && maniSkinLinear(&linear) == mani_success,
          "skins");
    double one = 0.0;
    double worst = 0.0;
    for (uint32_t v = 0; v < kMesh; ++v)
    {
        if (v < kSegments)
        {
            double e = Apart(&s_out[v], &s_other[v]);
            one = e > one ? e : one;
            continue;
        }
        // v' = R (S v - S c) + sum w_j (R_j S c + t_j), in binary64.
        double real[4] = {0}, lead[4] = {0}, rigid[12] = {0}, sum[9] = {0};
        bool any = false;
        for (uint32_t slot = 0; slot < maniInfluences_GetPerVertex(&s_centered); ++slot)
        {
            float wf = 0.0f;
            uint32_t j = maniInfluences_GetInfluence(&s_centered, v, slot, &wf);
            double w = (double)wf;
            if (w == 0.0)
            {
                break;
            }
            double am[9];
            for (int k = 0; k < 9; ++k)
            {
                am[k] = (double)s_palette[j].m[k];
            }
            double r[9];
            Polar(am, r);
            double q[4];
            QuatOf(r, q);
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
            }
            for (int k = 0; k < 9; ++k)
            {
                rigid[k] += r[k] * w;
            }
            for (int k = 0; k < 3; ++k)
            {
                rigid[9 + k] += (double)s_palette[j].m[9 + k] * w;
            }
            for (int i = 0; i < 3; ++i)
            {
                for (int c = 0; c < 3; ++c)
                {
                    sum[3 * i + c] += w * (r[3 * i] * am[3 * c] + r[3 * i + 1] * am[3 * c + 1] +
                                           r[3 * i + 2] * am[3 * c + 2]);
                }
            }
        }
        double n =
            sqrt(real[0] * real[0] + real[1] * real[1] + real[2] * real[2] + real[3] * real[3]);
        double q[4] = {real[0] / n, real[1] / n, real[2] / n, real[3] / n};
        double r[9] = {1 - 2 * (q[1] * q[1] + q[2] * q[2]), 2 * (q[0] * q[1] + q[3] * q[2]),
                       2 * (q[0] * q[2] - q[3] * q[1]),     2 * (q[0] * q[1] - q[3] * q[2]),
                       1 - 2 * (q[0] * q[0] + q[2] * q[2]), 2 * (q[1] * q[2] + q[3] * q[0]),
                       2 * (q[0] * q[2] + q[3] * q[1]),     2 * (q[1] * q[2] - q[3] * q[0]),
                       1 - 2 * (q[0] * q[0] + q[1] * q[1])};
        maniVec3 cf;
        CHECK(maniInfluences_GetCenter(&s_centered, v, &cf), "center");
        const double c[3] = {(double)cf.x, (double)cf.y, (double)cf.z};
        const double p[3] = {(double)s_in[v].position[0], (double)s_in[v].position[1],
                             (double)s_in[v].position[2]};
        double sp[3], sc[3];
        for (int i = 0; i < 3; ++i)
        {
            sp[i] = sum[3 * i] * p[0] + sum[3 * i + 1] * p[1] + sum[3 * i + 2] * p[2];
            sc[i] = sum[3 * i] * c[0] + sum[3 * i + 1] * c[1] + sum[3 * i + 2] * c[2];
        }
        for (int k = 0; k < 3; ++k)
        {
            double want = r[k] * (sp[0] - sc[0]) + r[3 + k] * (sp[1] - sc[1]) +
                          r[6 + k] * (sp[2] - sc[2]) + rigid[k] * sc[0] + rigid[3 + k] * sc[1] +
                          rigid[6 + k] * sc[2] + rigid[9 + k];
            double size = fabs(want) > 1.0 ? fabs(want) : 1.0;
            double e = fabs(want - (double)s_out[v].position[k]) / size;
            worst = e > worst ? e : worst;
        }
    }
    CHECK(one < 1e-5, "one influence: as linear");
    CHECK(worst < 5e-5, "centers within binary32 of the binary64 formula");
    if (worst >= 5e-5)
    {
        printf("  centers job apart by %g\n", worst);
    }
    Pin(s_out, kMesh * sizeof(Vertex));
}

static void TestScenePinned(void)
{
    CHECK(s_scene == PINNED_SCENE_HASH, "the scene's bits the same on every platform");
    if (s_scene != PINNED_SCENE_HASH)
    {
        printf("scene hash 0x%016llx\n", (unsigned long long)s_scene);
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
    TestMorphThenSkin();
    TestDualAgainstReference();
    TestDualOneInfluence();
    TestCandyWrapper();
    TestDualPalette();
    TestBlended();
    BuildMesh();
    TestCentersBuilt();
    TestCentersPaddingIgnored();
    TestCentersRefused();
    TestCentersJob();
    TestScenePinned();
    return s_failures == 0 ? 0 : 1;
}
