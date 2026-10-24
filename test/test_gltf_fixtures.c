// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Khronos sample models (test/data/gltf, CC0) imported whole: every
// animation built as a clip at 30 per second and its samples checked
// against tools/gen_gltf_expect.py, which reads the same files with its
// own reader and evaluates glTF's Appendix C in binary64. Step, linear
// and cubic translation, rotation and scale, morph weights, a GLB, data
// URIs and external buffers supplied from disk.

#include "gltf_fixture_expect.h"
#include "gltf_names.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PINNED_IMPORTED_HASH 0xaa872a042850f642ull

#ifndef MANI_TEST_DATA
#error "MANI_TEST_DATA names test/data"
#endif

static _Alignas(16) uint8_t s_skeletonFile[65536];
static _Alignas(16) uint8_t s_clipFile[262144];
static _Alignas(32) uint8_t s_poseMemory[16384];

// A file of test/data/gltf, or NULL; the caller frees it.
static uint8_t* ReadFile(const char* name, size_t* sizeOut)
{
    char path[512];
    snprintf(path, sizeof path, "%s/gltf/%s", MANI_TEST_DATA, name);
    FILE* f = fopen(path, "rb");
    uint8_t* bytes = NULL;
    long size = -1;
    if (f != NULL && fseek(f, 0, SEEK_END) == 0)
    {
        size = ftell(f);
    }
    if (size > 0 && fseek(f, 0, SEEK_SET) == 0)
    {
        bytes = malloc((size_t)size);
    }
    if (bytes != NULL && fread(bytes, 1, (size_t)size, f) != (size_t)size)
    {
        free(bytes);
        bytes = NULL;
    }
    if (f != NULL)
    {
        fclose(f);
    }
    *sizeOut = bytes != NULL ? (size_t)size : 0;
    CHECK(bytes != NULL, name);
    return bytes;
}

// The imported scene: every file built from the fixtures, in order, the
// writer's library version (bytes 16 to 21) left out as it changes with
// every release; every platform and backend must give PINNED_IMPORTED_HASH.
static uint64_t s_imported = MANI_HASH_INIT;

static void Pin(const uint8_t* file, size_t size)
{
    s_imported = maniHash64(s_imported, file, 16);
    s_imported = maniHash64(s_imported, file + 22, (int32_t)(size - 22));
}

static maniBuilder* NewBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "builder");
    return builder;
}

// The skeleton of the document's first skin, or of the track's node.
static bool BuildSkeleton(maniBuilder* builder, const maniGltf* gltf, uint32_t node,
                          maniSkeleton* skeleton)
{
    maniGltfOptions options = maniDefaultGltfOptions();
    maniImportResult r = maniGltfSkeletonCount(gltf) > 0
                             ? maniGltfBuildSkeleton(builder, gltf, 0, &options)
                             : maniGltfBuildNodeSkeleton(builder, gltf, node, &options);
    size_t size = 0;
    bool built =
        r.result == mani_success &&
        maniCopyBuilt(builder, s_skeletonFile, sizeof s_skeletonFile, &size) == mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, skeleton).result == mani_success;
    if (built)
    {
        Pin(s_skeletonFile, size);
    }
    return built;
}

static double SampleTime(uint32_t k, const FixtureTrack* t)
{
    return k + 1 == t->samples && k > 0 ? (double)t->duration : (double)k / 30.0;
}

// The largest difference from the oracle over a TRS track's samples
// (rotations up to sign).
static double TrsError(const maniClip* clip, const maniSkeleton* skeleton, uint32_t bone,
                       const FixtureTrack* t, maniPose* pose)
{
    double worst = 0;
    for (uint32_t k = 0; k < t->samples; ++k)
    {
        if (maniSampleClip(clip, skeleton, (float)SampleTime(k, t), pose) != mani_success)
        {
            return HUGE_VAL;
        }
        maniTransform local = maniPoseLocal(pose, bone);
        float v[4] = {local.translation.x, local.translation.y, local.translation.z, 0};
        if (t->kind == 1)
        {
            memcpy(v, &local.rotation, sizeof v);
        }
        else if (t->kind == 2)
        {
            memcpy(v, &local.scale, sizeof local.scale);
        }
        const double* e = t->values + (size_t)k * t->width;
        double plus = 0;
        double minus = 0;
        for (uint32_t c = 0; c < t->width; ++c)
        {
            plus = fmax(plus, fabs((double)v[c] - e[c]));
            minus = fmax(minus, fabs((double)v[c] + e[c]));
        }
        worst = fmax(worst, t->kind == 1 ? fmin(plus, minus) : plus);
    }
    return worst;
}

// The same for a weights channel's float tracks, found by name.
static double WeightError(const maniClip* clip, const FixtureTrack* t)
{
    uint32_t tracks[16];
    const char* name = t->names;
    for (uint32_t c = 0; c < t->width && c < 16; ++c)
    {
        const char* end = strchr(name, '|');
        size_t length = end != NULL ? (size_t)(end - name) : strlen(name);
        tracks[c] = maniClipFindFloatTrack(clip, name, (uint32_t)length);
        if (tracks[c] == MANI_NO_TRACK)
        {
            return HUGE_VAL;
        }
        name += length + (end != NULL ? 1 : 0);
    }
    float values[16];
    uint32_t count = maniClipFloatTrackCount(clip);
    double worst = 0;
    for (uint32_t k = 0; k < t->samples && count <= 16; ++k)
    {
        if (maniSampleClipFloats(clip, (float)SampleTime(k, t), values, count, NULL) !=
            mani_success)
        {
            return HUGE_VAL;
        }
        for (uint32_t c = 0; c < t->width; ++c)
        {
            worst = fmax(worst, fabs((double)values[tracks[c]] - t->values[k * t->width + c]));
        }
    }
    return count <= 16 ? worst : HUGE_VAL;
}

static void CheckTrack(const FixtureTrack* t)
{
    size_t size = 0;
    uint8_t* bytes = ReadFile(t->file, &size);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    maniSkeleton skeleton;
    bool open = bytes != NULL && maniOpenGltf(builder, bytes, size, &gltf).result == mani_success &&
                BuildSkeleton(builder, gltf, t->node, &skeleton);
    CHECK(open, t->file);
    maniGltfOptions options = maniDefaultGltfOptions();
    uint32_t bound = 0;
    bool built =
        open && maniGltfBuildClip(
                    builder, gltf, t->animation, &skeleton, &options,
                    &(maniImportClipOptions){.sampleRate = 30.0f, .loop = maniLoopClamp}, &bound)
                        .result == mani_success;
    maniClip clip;
    size_t clipSize = 0;
    built = built &&
            maniCopyBuilt(builder, s_clipFile, sizeof s_clipFile, &clipSize) == mani_success &&
            maniOpenClip(s_clipFile, clipSize, &clip).result == mani_success;
    CHECK(built, "the clip builds");
    if (built)
    {
        Pin(s_clipFile, clipSize);
        CHECK(maniClipSampleCount(&clip) == t->samples && maniClipDuration(&clip) == t->duration,
              "the oracle's grid");
        double error = HUGE_VAL;
        if (t->kind == 3)
        {
            error = WeightError(&clip, t);
        }
        else
        {
            uint32_t length = 0;
            const char* name = maniGltfNodeName(gltf, t->node, &length);
            uint32_t bone = maniSkeletonFindBone(&skeleton, name, length);
            maniPose* pose = NULL;
            if (bone != MANI_NO_BONE &&
                maniInitPose(&skeleton, s_poseMemory, sizeof s_poseMemory, &pose) == mani_success)
            {
                error = TrsError(&clip, &skeleton, bone, t, pose);
            }
        }
        CHECK(error < 2e-6, "samples equal Appendix C");
        if (!(error < 2e-6))
        {
            printf("  %s animation %u node %u kind %u: error %g\n", t->file, t->animation, t->node,
                   t->kind, error);
        }
    }
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    free(bytes);
}

// The same skin with its buffers in separate files, supplied by URI.
static void TestExternalBuffers(void)
{
    size_t size = 0;
    uint8_t* json = ReadFile("SimpleSkin/SimpleSkin.gltf", &size);
    maniBuilder* builder = NewBuilder();
    maniGltf* gltf = NULL;
    CHECK(json != NULL && maniOpenGltf(builder, json, size, &gltf).result == mani_success, "opens");
    uint8_t* buffers[8] = {0};
    uint32_t count = gltf != NULL ? maniGltfBufferCount(gltf) : 0;
    CHECK(count == 4, "four buffers");
    for (uint32_t i = 0; i < count && i < 8; ++i)
    {
        uint32_t length = 0;
        const char* uri = maniGltfBufferUri(gltf, i, &length);
        char name[256];
        snprintf(name, sizeof name, "SimpleSkin/%.*s", (int)length, uri != NULL ? uri : "");
        size_t bytes = 0;
        buffers[i] = ReadFile(name, &bytes);
        CHECK(buffers[i] != NULL &&
                  maniGltfSupplyBuffer(gltf, i, buffers[i], bytes).result == mani_success,
              "supplied");
    }
    maniSkeleton skeleton;
    maniGltfOptions options = maniDefaultGltfOptions();
    CHECK(gltf != NULL && BuildSkeleton(builder, gltf, 0, &skeleton), "the skeleton");
    CHECK(gltf != NULL &&
              maniGltfBuildSkin(builder, gltf, 0, &skeleton, &options).result == mani_success,
          "the skin");
    uint32_t bound = 0;
    CHECK(gltf != NULL &&
              maniGltfBuildClip(
                  builder, gltf, 0, &skeleton, &options,
                  &(maniImportClipOptions){.sampleRate = 30.0f, .loop = maniLoopClamp}, &bound)
                      .result == mani_success &&
              bound == 1,
          "the clip");
    maniCloseGltf(gltf);
    maniDestroyBuilder(builder);
    for (uint32_t i = 0; i < 8; ++i)
    {
        free(buffers[i]);
    }
    free(json);
}

int main(void)
{
    for (size_t i = 0; i < sizeof kFixtureTracks / sizeof kFixtureTracks[0]; ++i)
    {
        CheckTrack(&kFixtureTracks[i]);
    }
    TestExternalBuffers();
    CHECK(s_imported == PINNED_IMPORTED_HASH,
          "the imported files' bits the same on every platform");
    if (s_imported != PINNED_IMPORTED_HASH)
    {
        printf("imported hash 0x%016llx\n", (unsigned long long)s_imported);
    }
    return s_failures == 0 ? 0 : 1;
}
