// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// FBX fixtures (test/data/fbx): scenes written for maul-anim,
// in ASCII and in binary forms (13- and 25-byte records, deflated
// arrays, a big-endian file) from tools/fbx_fixtures.py, imported whole
// and checked against ufbx's values for them (fbx_fixture_expect.h, by
// tools/fbx_differential.c --expect): whether each opens, its rate,
// rest worlds (the placement times each bone's model matrix), skins'
// inverse binds, and every clip's duration, bone worlds and float
// tracks at five sample times, built on the first skeleton (or the
// first top model's subtree) with the placement folded.

#include "fbx_fixture_expect.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/import.h"
#include "maul-anim/pose.h"
#include "maul-anim/skeleton.h"
#include "maul-anim/skin.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PINNED_IMPORTED_HASH 0x4bef537a64175951ull

#ifndef MANI_TEST_DATA
#error "MANI_TEST_DATA names test/data"
#endif

static _Alignas(16) uint8_t s_skeletonFile[65536];
static _Alignas(16) uint8_t s_skinFile[65536];
static _Alignas(16) uint8_t s_clipFile[262144];
static _Alignas(32) uint8_t s_poseMemory[16384];

// The imported scene: every file built from the fixtures, in order, the
// writer's library version (bytes 16 to 21) left out as it changes with
// every release; every platform and backend must give PINNED_IMPORTED_HASH.
static uint64_t s_imported = MANI_HASH_INIT;

static void Pin(const uint8_t* file, size_t size)
{
    s_imported = maniHash64(s_imported, file, 16);
    s_imported = maniHash64(s_imported, file + 22, (int64_t)(size - 22));
}

// Nothing built yet.
static const uint32_t kNone = UINT32_MAX;

// One fixture open, and what was last built from it: a skeleton (by
// kind, index and whether its placement is folded), a skin, a clip.
typedef struct Fixture
{
    maniBuilder* builder;
    maniFbx* fbx;
    bool skeletonByModel;
    uint32_t skeletonIndex;
    bool skeletonFolded;
    maniSkeleton skeleton;
    maniAffine placement;
    maniPose* pose;
    uint32_t skinIndex;
    maniSkin skin;
    uint32_t clipIndex;
    maniClip clip;
    // The clips' skeleton, from the basis record.
    bool basisByModel;
    uint32_t basis;
} Fixture;

// A file of test/data/fbx, or NULL; the caller frees it.
static uint8_t* ReadFile(const char* name, size_t* sizeOut)
{
    char path[512];
    snprintf(path, sizeof path, "%s/fbx/%s", MANI_TEST_DATA, name);
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

static bool Near(double a, double b)
{
    double size = fabs(b) > 1.0 ? fabs(b) : 1.0;
    return fabs(a - b) <= 1e-4 * size;
}

static bool SameValues(const double* ours, const FbxExpected* e)
{
    bool same = true;
    for (uint32_t i = 0; same && i < e->count; ++i)
    {
        same = Near(ours[i], e->values[i]);
    }
    return same;
}

// a times b, both 3x4 column-major affine matrices.
static void Compose(const maniAffine* a, const maniAffine* b, double out[12])
{
    for (int col = 0; col < 4; ++col)
    {
        for (int row = 0; row < 3; ++row)
        {
            double v = (double)a->m[row] * (double)b->m[col * 3] +
                       (double)a->m[3 + row] * (double)b->m[col * 3 + 1] +
                       (double)a->m[6 + row] * (double)b->m[col * 3 + 2];
            out[col * 3 + row] = col == 3 ? v + (double)a->m[9 + row] : v;
        }
    }
}

// The skeleton (or model subtree) given, its placement folded or not,
// built unless it is the one last built; a rest pose of it.
static bool UseSkeleton(Fixture* f, bool byModel, uint32_t index, bool folded)
{
    if (f->pose != NULL && f->skeletonByModel == byModel && f->skeletonIndex == index &&
        f->skeletonFolded == folded)
    {
        return maniPose_SetRest(f->pose, &f->skeleton) == mani_success;
    }
    f->pose = NULL;
    f->skinIndex = kNone;
    f->clipIndex = kNone;
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = folded ? 1 : 0;
    maniImportResult r = byModel ? maniBuildFbxModelSkeleton(f->builder, f->fbx, index, &options)
                                 : maniBuildFbxSkeleton(f->builder, f->fbx, index, &options);
    maniImportResult p = byModel
                             ? maniFbx_GetModelPlacement(f->fbx, index, &options, &f->placement)
                             : maniFbx_GetSkeletonPlacement(f->fbx, index, &options, &f->placement);
    size_t size = 0;
    bool built =
        r.result == mani_success && p.result == mani_success &&
        maniBuilder_CopyBuilt(f->builder, s_skeletonFile, sizeof s_skeletonFile, &size) ==
            mani_success &&
        maniOpenSkeleton(s_skeletonFile, size, &f->skeleton).result == mani_success &&
        maniInitPose(&f->skeleton, s_poseMemory, sizeof s_poseMemory, &f->pose) == mani_success &&
        maniPose_SetRest(f->pose, &f->skeleton) == mani_success;
    if (built)
    {
        Pin(s_skeletonFile, size);
    }
    f->skeletonByModel = byModel;
    f->skeletonIndex = index;
    f->skeletonFolded = folded;
    f->pose = built ? f->pose : NULL;
    return built;
}

// A bone's world in the pose: the placement times its model matrix.
static bool World(Fixture* f, const char* name, double out[12])
{
    uint32_t bone = maniSkeleton_FindBone(&f->skeleton, name, (uint32_t)strlen(name));
    if (bone == MANI_NO_BONE || maniPose_ComputeModel(f->pose, &f->skeleton) != mani_success)
    {
        return false;
    }
    maniAffine model = maniPose_GetModel(f->pose, bone);
    Compose(&f->placement, &model, out);
    return true;
}

static bool CheckRest(Fixture* f, const FbxExpected* e, bool byModel)
{
    double ours[12];
    return UseSkeleton(f, byModel, e->index, false) && World(f, e->name, ours) &&
           SameValues(ours, e);
}

static bool CheckBind(Fixture* f, const FbxExpected* e)
{
    uint32_t s = maniFbx_GetSkinSkeleton(f->fbx, e->index);
    if (s == UINT32_MAX || !UseSkeleton(f, false, s, false))
    {
        return false;
    }
    if (f->skinIndex != e->index)
    {
        maniFbxOptions options = maniDefaultFbxOptions();
        size_t size = 0;
        bool built =
            maniBuildFbxSkin(f->builder, f->fbx, e->index, &f->skeleton, &options).result ==
                mani_success &&
            maniBuilder_CopyBuilt(f->builder, s_skinFile, sizeof s_skinFile, &size) ==
                mani_success &&
            maniOpenSkin(s_skinFile, size, &f->skin).result == mani_success;
        if (built)
        {
            Pin(s_skinFile, size);
        }
        f->skinIndex = built ? e->index : kNone;
    }
    uint32_t j = (uint32_t)e->time;
    if (f->skinIndex == kNone || j >= maniSkin_GetJointCount(&f->skin))
    {
        return false;
    }
    size_t length = 0;
    const char* name =
        maniSkeleton_GetBoneName(&f->skeleton, maniSkin_GetJoint(&f->skin, j), &length);
    maniAffine bind = maniSkin_GetInverseBind(&f->skin, j);
    double ours[12];
    for (int i = 0; i < 12; ++i)
    {
        ours[i] = (double)bind.m[i];
    }
    return length == strlen(e->name) && memcmp(name, e->name, length) == 0 && SameValues(ours, e);
}

// Clip k on the clips' skeleton, built unless it is the one last built.
static bool UseClip(Fixture* f, uint32_t k)
{
    if (!UseSkeleton(f, f->basisByModel, f->basis, true))
    {
        return false;
    }
    if (f->clipIndex == k)
    {
        return true;
    }
    maniFbxOptions options = maniDefaultFbxOptions();
    options.foldPlacement = 1;
    size_t size = 0;
    bool built =
        maniBuildFbxClip(f->builder, f->fbx, k, &f->skeleton, &options,
                         &(maniImportClipOptions){.sampleRate = 0.0f, .loop = maniLoopClamp}, NULL)
                .result == mani_success &&
        maniBuilder_CopyBuilt(f->builder, s_clipFile, sizeof s_clipFile, &size) == mani_success &&
        maniOpenClip(s_clipFile, size, &f->clip).result == mani_success;
    if (built)
    {
        Pin(s_clipFile, size);
    }
    f->clipIndex = built ? k : kNone;
    return built;
}

static bool CheckSample(Fixture* f, const FbxExpected* e)
{
    double ours[12];
    return UseClip(f, e->index) &&
           maniClip_Sample(&f->clip, &f->skeleton, (float)e->time, f->pose) == mani_success &&
           World(f, e->name, ours) && SameValues(ours, e);
}

static bool CheckFloat(Fixture* f, const FbxExpected* e)
{
    float values[256];
    if (!UseClip(f, e->index))
    {
        return false;
    }
    uint32_t tracks = maniClip_GetFloatTrackCount(&f->clip);
    uint32_t t = maniClip_FindFloatTrack(&f->clip, e->name, (uint32_t)strlen(e->name));
    double ours = 0.0;
    bool sampled =
        tracks <= 256 && t != MANI_NO_TRACK &&
        maniClip_SampleFloats(&f->clip, (float)e->time, values, tracks, NULL) == mani_success;
    ours = sampled ? (double)values[t] : 0.0;
    return sampled && SameValues(&ours, e);
}

static bool CheckRate(const Fixture* f, const FbxExpected* e)
{
    maniFbxSettings s;
    if (maniFbx_ReadSettings(f->fbx, &s) != mani_success)
    {
        return false;
    }
    double ours =
        s.rateDenominator != 0 ? (double)s.rateNumerator / s.rateDenominator : s.customRate;
    return SameValues(&ours, e);
}

static bool Check(Fixture* f, const FbxExpected* e, bool opened)
{
    if (strcmp(e->kind, "open") == 0)
    {
        return opened == (e->values[0] == 1.0);
    }
    if (strcmp(e->kind, "basis") == 0)
    {
        f->basisByModel = e->values[0] == 1.0;
        f->basis = e->index;
        return true;
    }
    if (!opened)
    {
        return false;
    }
    if (strcmp(e->kind, "rate") == 0)
    {
        return CheckRate(f, e);
    }
    if (strcmp(e->kind, "rest") == 0 || strcmp(e->kind, "model") == 0)
    {
        return CheckRest(f, e, strcmp(e->kind, "model") == 0);
    }
    if (strcmp(e->kind, "bind") == 0)
    {
        return CheckBind(f, e);
    }
    if (strcmp(e->kind, "duration") == 0)
    {
        double ours = UseClip(f, e->index) ? (double)maniClip_GetDuration(&f->clip) : -1.0;
        return SameValues(&ours, e);
    }
    if (strcmp(e->kind, "sample") == 0)
    {
        return CheckSample(f, e);
    }
    return strcmp(e->kind, "float") == 0 && CheckFloat(f, e);
}

// Every record of fixture file k, from record *at on.
static void TestFile(uint32_t k, size_t* at)
{
    const char* name = kFbxFixtureFiles[k];
    size_t size = 0;
    uint8_t* bytes = ReadFile(name, &size);
    maniBuilderDef def = maniDefaultBuilderDef();
    Fixture f = {0};
    f.skinIndex = kNone;
    f.clipIndex = kNone;
    bool ready = bytes != NULL && maniCreateBuilder(&def, &f.builder) == mani_success;
    bool opened = ready && maniOpenFbx(f.builder, bytes, size, &f.fbx).result == mani_success;
    size_t n = sizeof kFbxExpected / sizeof kFbxExpected[0];
    uint32_t checked = 0;
    for (; *at < n && kFbxExpected[*at].file == k; ++*at)
    {
        const FbxExpected* e = &kFbxExpected[*at];
        bool right = ready && Check(&f, e, opened);
        if (!right)
        {
            printf("FAIL: %s: %s %u t=%g %s\n", name, e->kind, e->index, e->time, e->name);
            ++s_failures;
        }
        ++checked;
    }
    CHECK(checked > 0, name);
    maniCloseFbx(f.fbx);
    maniDestroyBuilder(f.builder);
    free(bytes);
}

int main(void)
{
    size_t at = 0;
    uint32_t files = (uint32_t)(sizeof kFbxFixtureFiles / sizeof kFbxFixtureFiles[0]);
    for (uint32_t k = 0; k < files; ++k)
    {
        TestFile(k, &at);
    }
    CHECK(at == sizeof kFbxExpected / sizeof kFbxExpected[0], "every record in file order");
    CHECK(s_imported == PINNED_IMPORTED_HASH,
          "the imported files' bits the same on every platform");
    if (s_imported != PINNED_IMPORTED_HASH)
    {
        printf("imported hash 0x%016llx\n", (unsigned long long)s_imported);
    }
    return s_failures == 0 ? 0 : 1;
}
