// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The import and compression snippets of docs/user/importing.md, as
// written there, over Khronos' SimpleSkin: its skeleton
// and its animation imported, the clip compressed; and its budget
// allocator, under which a file of under 2 KB asking for 8.6 GB of
// viewless morph arrays is refused with Capacity, nothing kept. Run as test_guide_import
// --default-huge it opens that file with the C library's allocator
// alone, for CI to run under a memory limit (ulimit -v), where malloc
// must refuse it too. MSVC's CRT deprecates fopen, which reading the
// sample uses: CMake defines _CRT_SECURE_NO_WARNINGS for the fixture
// tests.

#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/skeleton.h"

#include <stdalign.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

// clang-format off
typedef struct Budget
{
    size_t used;
    size_t limit;
} Budget;

static void* BudgetAlloc(size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    if (alignment > alignof(max_align_t) || size > budget->limit - budget->used)
    {
        return NULL;
    }
    void* memory = malloc(size);
    budget->used += memory != NULL ? size : 0;
    return memory;
}

static void BudgetFree(void* memory, size_t size, size_t alignment, void* context)
{
    Budget* budget = context;
    (void)alignment;
    if (memory != NULL)
    {
        budget->used -= size;
        free(memory);
    }
}
// clang-format on

// A primitive of 2^24 vertices without views (the most an accessor
// without a view may hold) morphing 32 attributes, POSITION, NORMAL, 14
// texture coordinates and 16 colors: a full array of 268 MB each, 8.6
// GB live at once, asked for by under 2 KB.
static char s_huge[4096];

static size_t Huge(void)
{
    char attributes[2048];
    size_t n = (size_t)snprintf(attributes, sizeof attributes, "\"POSITION\":0,\"NORMAL\":0");
    for (int k = 0; k < 14; ++k)
    {
        n += (size_t)snprintf(attributes + n, sizeof attributes - n, ",\"TEXCOORD_%d\":1", k);
    }
    for (int k = 0; k < 16; ++k)
    {
        n += (size_t)snprintf(attributes + n, sizeof attributes - n, ",\"COLOR_%d\":2", k);
    }
    return (size_t)snprintf(
        s_huge, sizeof s_huge,
        "{\"asset\":{\"version\":\"2.0\"},\"accessors\":["
        "{\"componentType\":5126,\"type\":\"VEC3\",\"count\":16777216},"
        "{\"componentType\":5126,\"type\":\"VEC2\",\"count\":16777216},"
        "{\"componentType\":5126,\"type\":\"VEC4\",\"count\":16777216}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{%s},\"targets\":[{%s}]}]}]}",
        attributes, attributes);
}

// The huge file's morph targets built by a builder of def.
static maniResult BuildHuge(const maniBuilderDef* def)
{
    maniBuilder* builder = NULL;
    if (maniCreateBuilder(def, &builder) != mani_success)
    {
        return mani_errorCapacity;
    }
    maniGltf* gltf = NULL;
    size_t size = Huge();
    maniImportResult r = maniOpenGltf(builder, s_huge, size, &gltf);
    maniGltfOptions options = maniDefaultGltfOptions();
    if (r.result == mani_success)
    {
        r = maniGltfBuildMorphTargets(builder, gltf, 0, 0, &options);
        maniCloseGltf(gltf);
    }
    maniDestroyBuilder(builder);
    return r.result;
}

static void TestBudget(void)
{
    // clang-format off
    Budget budget = {0, (size_t)256 << 20};   // 256 MiB for one file's import
    maniBuilderDef guarded = maniDefaultBuilderDef();
    guarded.allocator = (maniAllocator){BudgetAlloc, BudgetFree, &budget};
    maniBuilder* careful = NULL;
    maniResult created = maniCreateBuilder(&guarded, &careful);
    // clang-format on
    CHECK(created == mani_success, "a builder with a budget");
    maniDestroyBuilder(careful);
    CHECK(BuildHuge(&guarded) == mani_errorCapacity && budget.used == 0,
          "a huge viewless count refused within the budget, nothing kept");
}

int main(int argc, char** argv)
{
    if (argc >= 2 && strcmp(argv[1], "--default-huge") == 0)
    {
        maniBuilderDef def = maniDefaultBuilderDef();
        maniResult r = BuildHuge(&def);
        printf("default allocator: %s\n", r == mani_errorCapacity ? "capacity" : "other");
        return r == mani_errorCapacity ? 0 : 1;
    }
    TestBudget();
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
