// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The guide's import and compression snippets (docs/guide.md, "Getting
// data in"), as written there, over Khronos' SimpleSkin: its skeleton
// and its animation imported, the clip compressed. MSVC's CRT deprecates
// fopen, which reading the sample uses: CMake defines
// _CRT_SECURE_NO_WARNINGS for the fixture tests.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static _Alignas(16) uint8_t s_skeletonFile[1 << 14];
static _Alignas(16) uint8_t s_clipFile[1 << 16];
static _Alignas(16) uint8_t s_smallFile[1 << 16];

static uint8_t* ReadFile(const char* name, size_t* sizeOut)
{
    char path[512];
    snprintf(path, sizeof path, "%s/gltf/%s", MANI_TEST_DATA, name);
    FILE* f = fopen(path, "rb");
    if (f == NULL)
    {
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t* bytes = size > 0 ? malloc((size_t)size) : NULL;
    size_t read = bytes != NULL ? fread(bytes, 1, (size_t)size, f) : 0;
    fclose(f);
    *sizeOut = read;
    return bytes;
}

int main(void)
{
    size_t size = 0;
    uint8_t* bytes = ReadFile("SimpleSkinEmbedded.gltf", &size);
    CHECK(bytes != NULL, "SimpleSkin read");
    maniBuilderDef def = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&def, &builder) == mani_success, "a builder");
    uint8_t* skeletonFile = s_skeletonFile;
    size_t skeletonCapacity = sizeof s_skeletonFile;
    uint8_t* clipFile = s_clipFile;
    size_t clipCapacity = sizeof s_clipFile;
    uint8_t* smallFile = s_smallFile;
    size_t smallCapacity = sizeof s_smallFile;
    // clang-format off
    maniGltf* gltf = NULL;
    maniImportResult read = maniOpenGltf(builder, bytes, size, &gltf);   // bytes kept alive
    maniGltfOptions options = maniDefaultGltfOptions();
    maniImportResult skeletonImported = maniGltfBuildSkeleton(builder, gltf, 0, &options);
    size_t skeletonSize = 0;
    maniResult skeletonCopied = maniCopyBuilt(builder, skeletonFile, skeletonCapacity, &skeletonSize);
    maniSkeleton skeleton;
    maniFileResult skeletonOpened = maniOpenSkeleton(skeletonFile, skeletonSize, &skeleton);

    maniImportClipOptions clipOptions = maniDefaultImportClipOptions();   // 30 samples a second
    maniImportResult clipImported =
        maniGltfBuildClip(builder, gltf, 0, &skeleton, &options, &clipOptions, NULL);
    size_t clipSize = 0;
    maniResult clipCopied = maniCopyBuilt(builder, clipFile, clipCapacity, &clipSize);
    maniClip clip;
    maniFileResult clipOpened = maniOpenClip(clipFile, clipSize, &clip);
    maniCloseGltf(gltf);
    // clang-format on
    CHECK(read.result == mani_success && skeletonImported.result == mani_success &&
              skeletonCopied == mani_success && skeletonOpened.result == mani_success &&
              clipImported.result == mani_success && clipCopied == mani_success &&
              clipOpened.result == mani_success && maniClipDuration(&clip) > 0.0f,
          "the import snippet reads SimpleSkin's skeleton and animation");
    // clang-format off
    maniCompressOptions tolerance = maniDefaultCompressOptions();   // 0.1 mm at 3 cm
    maniCompressReport report;
    maniResult compressed = maniCompressClip(builder, &clip, &skeleton, &tolerance, &report);
    size_t smallSize = 0;
    maniResult smallCopied = maniCopyBuilt(builder, smallFile, smallCapacity, &smallSize);
    maniClip small;
    maniFileResult smallOpened = maniOpenClip(smallFile, smallSize, &small);
    // report.shellError is the largest error found at any sample.
    // clang-format on
    CHECK(compressed == mani_success && smallCopied == mani_success &&
              smallOpened.result == mani_success && report.shellError <= 1e-4 &&
              maniClipDuration(&small) == maniClipDuration(&clip),
          "the compression snippet keeps the clip within its tolerance");
    maniDestroyBuilder(builder);
    free(bytes);
    return s_failures == 0 ? 0 : 1;
}
