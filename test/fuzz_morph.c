// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a morph targets file, and on success reading every
// channel, turning weights into target weights and applying them over
// the first vertices; then the same bytes with the payload's hash made
// right, so that mutations reach the checks behind it. Run as fuzz_morph
// <corpus dir>; the seed comes from test_morph --seed <file>.

#include "maul-anim/morph.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

enum
{
    kVertices = 256,
};

static float s_points[kVertices * 3];
static float s_normals[kVertices * 3];
static float s_tangents[kVertices * 4];
static float s_texcoords[kVertices * 2];
static float s_colors[kVertices * 4];

static void Exercise(const uint8_t* bytes, size_t size)
{
    maniMorphTargets m;
    if (maniOpenMorphTargets(bytes, size, &m).result != 0)
    {
        return;
    }
    uint32_t channels = maniMorphChannelCount(&m);
    uint32_t targets = maniMorphTargetCount(&m);
    float* channelWeights = malloc(sizeof(float) * channels);
    float* targetWeights = malloc(sizeof(float) * targets);
    if (channelWeights == NULL || targetWeights == NULL)
    {
        free(channelWeights);
        free(targetWeights);
        return;
    }
    for (uint32_t c = 0; c < channels; ++c)
    {
        uint32_t length = 0;
        const char* name = maniMorphChannelName(&m, c, &length);
        if (name == NULL || maniMorphFindChannel(&m, name, length) > c)
        {
            abort();
        }
        channelWeights[c] = (float)(c % 5) * 0.5f - 0.75f;
    }
    if (maniMorphTargetWeights(&m, channelWeights, targetWeights) != mani_success)
    {
        abort();
    }
    for (uint32_t t = 0; t < targets; ++t)
    {
        uint32_t length = 0;
        const char* name = maniMorphTargetName(&m, t, &length);
        if (name == NULL || maniMorphFindTarget(&m, name, length) > t)
        {
            abort();
        }
        // Extrapolation can overflow to infinity; the job refuses that.
        targetWeights[t] = targetWeights[t] == targetWeights[t] ? targetWeights[t] : 0.0f;
    }
    memset(s_points, 0, sizeof s_points);
    memset(s_normals, 0, sizeof s_normals);
    memset(s_tangents, 0, sizeof s_tangents);
    memset(s_texcoords, 0, sizeof s_texcoords);
    memset(s_colors, 0, sizeof s_colors);
    // Every attribute the file names, then read back.
    for (uint32_t a = 0; a < maniMorphAttributeCount(&m); ++a)
    {
        uint32_t set = 0;
        if (maniMorphAttribute(&m, a, &set) > maniMorphColor)
        {
            abort();
        }
    }
    uint32_t count = maniMorphVertexCount(&m) < kVertices ? maniMorphVertexCount(&m) : kVertices;
    const maniMorphStream streams[5] = {{s_points, s_points, 12, 12, maniMorphPosition, 0, 3},
                                        {s_normals, s_normals, 12, 12, maniMorphNormal, 0, 3},
                                        {s_tangents, s_tangents, 16, 16, maniMorphTangent, 0, 4},
                                        {s_texcoords, s_texcoords, 8, 8, maniMorphTexcoord, 0, 2},
                                        {s_colors, s_colors, 16, 16, maniMorphColor, 0, 4}};
    maniMorphJob job = {&m, targetWeights, 0, count, 5, streams};
    (void)maniApplyMorphTargets(&job);
    free(channelWeights);
    free(targetWeights);
}

int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    // The file is opened from a 16-byte aligned copy, as a host would;
    // aligned by hand, as aligned_alloc is not everywhere.
    uint8_t* block = malloc(size + 16);
    if (block == NULL)
    {
        return 0;
    }
    uint8_t* copy = block + (16 - (uintptr_t)block % 16) % 16;
    if (size > 0)
    {
        memcpy(copy, data, size);
    }
    Exercise(copy, size);
    uint64_t fileSize = 0;
    if (size >= 64)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= 64 && fileSize <= size && fileSize - 64 < 0x7FFFFFFF)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + 64, (int32_t)(fileSize - 64));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
