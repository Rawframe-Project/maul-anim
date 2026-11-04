// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Opening a morph targets file, and on success reading every
// channel, turning weights into target weights and applying them over
// the first vertices; then the same bytes with the payload's hash made
// right, so that mutations reach the checks behind it. Run as fuzz_morph
// <corpus dir>; the seed comes from test_morph --seed <file>.

#include "container.h"

#include "maul-anim/morph.h"

#include <math.h>
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

// Up to 16 bytes inside an opened file changed, picked from its own
// bytes, as a host's memory may change after the open: every call after
// reads only within the file and ends, whatever it gives. Restore puts
// them back.
typedef struct Changed
{
    uint64_t at[16];
    uint8_t kept[16];
} Changed;

static Changed Change(uint8_t* bytes, uint64_t size)
{
    Changed c;
    uint64_t state = size * 0x9E3779B97F4A7C15u;
    for (uint64_t i = 0; i < size && i < 64; ++i)
    {
        state = (state ^ bytes[i]) * 0x100000001B3u;
    }
    for (int i = 0; i < 16; ++i)
    {
        state ^= state >> 29;
        state *= 0xBF58476D1CE4E5B9u;
        c.at[i] = state % size;
        c.kept[i] = bytes[c.at[i]];
        bytes[c.at[i]] ^= (uint8_t)(state >> 56) | 1u;
    }
    return c;
}

static void Restore(uint8_t* bytes, const Changed* c)
{
    for (int i = 15; i >= 0; --i)
    {
        bytes[c->at[i]] = c->kept[i];
    }
}

static void Exercise(uint8_t* bytes, size_t size)
{
    maniMorphTargets m;
    if (maniOpenMorphTargets(bytes, size, &m).result != 0)
    {
        return;
    }
    uint32_t channels = maniMorphTargets_GetChannelCount(&m);
    uint32_t targets = maniMorphTargets_GetTargetCount(&m);
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
        size_t length = 0;
        const char* name = maniMorphTargets_GetChannelName(&m, c, &length);
        if (name == NULL || maniMorphTargets_FindChannel(&m, name, length) > c)
        {
            abort();
        }
        channelWeights[c] = (float)(c % 5) * 0.5f - 0.75f;
    }
    if (maniMorphTargets_ComputeTargetWeights(&m, channelWeights, targetWeights,
                                              maniMorphTargets_GetTargetCount(&m),
                                              NULL) != mani_success)
    {
        abort();
    }
    for (uint32_t t = 0; t < targets; ++t)
    {
        size_t length = 0;
        const char* name = maniMorphTargets_GetTargetName(&m, t, &length);
        if (name == NULL || maniMorphTargets_FindTarget(&m, name, length) > t)
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
    for (uint32_t a = 0; a < maniMorphTargets_GetAttributeCount(&m); ++a)
    {
        uint32_t set = 0;
        if (maniMorphTargets_GetAttribute(&m, a, &set) > maniMorphColor)
        {
            abort();
        }
    }
    uint32_t count = maniMorphTargets_GetVertexCount(&m) < kVertices
                         ? maniMorphTargets_GetVertexCount(&m)
                         : kVertices;
    const maniMorphStream streams[5] = {{s_points, s_points, 12, 12, maniMorphPosition, 0, 3},
                                        {s_normals, s_normals, 12, 12, maniMorphNormal, 0, 3},
                                        {s_tangents, s_tangents, 16, 16, maniMorphTangent, 0, 4},
                                        {s_texcoords, s_texcoords, 8, 8, maniMorphTexcoord, 0, 2},
                                        {s_colors, s_colors, 16, 16, maniMorphColor, 0, 4}};
    maniMorphJob job = {&m, targetWeights, 0, count, 5, streams};
    (void)maniApplyMorphTargets(&job);
    // Again over bytes changed after the open: names, weights and the job.
    Changed c = Change(bytes, size);
    for (uint32_t i = 0; i < channels; ++i)
    {
        size_t length = 0;
        const char* name = maniMorphTargets_GetChannelName(&m, i, &length);
        (void)maniMorphTargets_FindChannel(&m, name, length);
    }
    for (uint32_t t = 0; t < targets; ++t)
    {
        size_t length = 0;
        const char* name = maniMorphTargets_GetTargetName(&m, t, &length);
        (void)maniMorphTargets_FindTarget(&m, name, length);
    }
    (void)maniMorphTargets_ComputeTargetWeights(&m, channelWeights, targetWeights, targets, NULL);
    for (uint32_t t = 0; t < targets; ++t)
    {
        targetWeights[t] = targetWeights[t] == targetWeights[t] && fabsf(targetWeights[t]) < 1e30f
                               ? targetWeights[t]
                               : 0.0f;
    }
    (void)maniApplyMorphTargets(&job);
    Restore(bytes, &c);
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
    if (size >= MANI_CONTAINER_HEADER_SIZE)
    {
        memcpy(&fileSize, copy + 24, sizeof fileSize);
    }
    if (fileSize >= MANI_CONTAINER_HEADER_SIZE && fileSize <= size)
    {
        uint64_t hash = maniHash64(MANI_HASH_INIT, copy + MANI_CONTAINER_HEADER_SIZE,
                                   (int64_t)(fileSize - MANI_CONTAINER_HEADER_SIZE));
        memcpy(copy + 32, &hash, sizeof hash);
        Exercise(copy, size);
    }
    free(block);
    return 0;
}
