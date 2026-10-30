// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Calls back in from the host's code: a builder's allocator,
// as it allocates and as it frees inside a build, calls every function
// taking that builder (each with arguments refused otherwise as invalid,
// so the busy error is the call back in alone), destroys it and closes
// a glTF and an FBX document opened with it: each refused with
// mani_errorBusy (nothing destroyed or closed) and counted once, its
// outputs left as they were (an opened document's handle NULL), the
// build finishing as it does without; a measure's free hook that
// measures again into the same report leaves what the measure wrote. A
// trainer's allocator and task hooks call training, restoring and
// destroying it, likewise refused, counted, left alone and harmless.
// A destroy or close is gone to the host's code from its start: from
// every free it makes, the object's own block's among them, the calls
// on the builder or trainer are refused as busy, and a closing
// document reads as empty and refuses its changes.

#include "checkpoint_host.h"
#include "cleared.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/import.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/learnedtrain.h"
#include "maul-anim/matchbuild.h"
#include "maul-anim/mirrorbuild.h"
#include "maul-anim/retargetbuild.h"
#include "maul-anim/skeleton.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The calls made back in: every function taking a builder, then its
// destroy and the two documents' closes.
enum
{
    kBuilderCalls = 40,
    kDestroy = kBuilderCalls,
    kCloseGltf,
    kCloseFbx,
    kCalls,
};

static maniBuilder* s_builder;
static maniGltf* s_gltf;
static maniFbx* s_fbx;
// The call to make back in from the next allocation (or free), -1 for
// none, and what it returned.
static int s_call = -1;
static bool s_onFree;
static maniResult s_result;

// The outputs of the calls made back in, filled with a sentinel before
// each: a refused call leaves its own, since they may be the ones the
// call in progress is filling, and sets an opened document's to NULL.
static struct
{
    maniClipReport clipReport;
    maniCompressReport compressReport;
    size_t size;
    uint32_t count;
    maniGltf* gltf;
    maniFbx* fbx;
    uint32_t bound;
} s_out;

// Whether call k left its output as it was, or set the handle it
// creates to NULL.
static bool OutputLeft(int k)
{
    switch (k)
    {
    case 5:
        return Kept(&s_out.clipReport, sizeof s_out.clipReport);
    case 6:
        return Kept(&s_out.compressReport, sizeof s_out.compressReport);
    case 9:
        return Kept(&s_out.size, sizeof s_out.size);
    case 10:
        return Kept(&s_out.count, sizeof s_out.count);
    case 15:
        return s_out.gltf == NULL;
    case 19:
    case 28:
        return Kept(&s_out.bound, sizeof s_out.bound);
    case 22:
        return s_out.fbx == NULL;
    default:
        return true;
    }
}

static maniResult CallBuilder(int k)
{
    maniBuilder* b = s_builder;
    switch (k)
    {
    case 0:
        return maniBuildSkeleton(b, NULL).result;
    case 1:
        return maniBuildSkin(b, NULL).result;
    case 2:
        return maniBuildInfluences(b, NULL).result;
    case 3:
        return maniBuildCenters(b, NULL).result;
    case 4:
        return maniBuildClip(b, NULL).result;
    case 5:
        return maniBuilder_MeasureClip(b, NULL, NULL, &s_out.clipReport);
    case 6:
        return maniCompressClip(b, NULL, NULL, NULL, &s_out.compressReport);
    case 7:
        return maniBuildSpringSet(b, NULL).result;
    case 8:
        return maniBuildRagdoll(b, NULL).result;
    case 9:
        return maniBuilder_CopyBuilt(b, NULL, 1, &s_out.size);
    case 10:
        return maniBuilder_CopySkeletonOrder(b, NULL, 1, &s_out.count);
    case 11:
        return maniBuildMorphTargets(b, NULL).result;
    case 12:
        return maniBuildGraph(b, NULL).result;
    case 13:
        return maniWriteGraphJson(b, NULL).result;
    case 14:
        return maniBuildGraphJson(b, NULL, 0, NULL).result;
    case 15:
        return maniOpenGltf(b, NULL, 0, &s_out.gltf).result;
    case 16:
        return maniBuildGltfSkeleton(b, NULL, 0, NULL).result;
    case 17:
        return maniBuildGltfNodeSkeleton(b, NULL, 0, NULL).result;
    case 18:
        return maniBuildGltfSkin(b, NULL, 0, NULL, NULL).result;
    case 19:
        return maniBuildGltfClip(b, NULL, 0, NULL, NULL, NULL, &s_out.bound).result;
    case 20:
        return maniBuildGltfInfluences(b, NULL, 0, 0, NULL).result;
    case 21:
        return maniBuildGltfMorphTargets(b, NULL, 0, 0, NULL).result;
    case 22:
        return maniOpenFbx(b, NULL, 0, &s_out.fbx).result;
    case 23:
        return maniBuildFbxSkeleton(b, NULL, 0, NULL).result;
    case 24:
        return maniBuildFbxModelSkeleton(b, NULL, 0, NULL).result;
    case 25:
        return maniBuildFbxSkin(b, NULL, 0, NULL, NULL).result;
    case 26:
        return maniBuildFbxInfluences(b, NULL, 0, NULL).result;
    case 27:
        return maniBuildFbxMorphTargets(b, NULL, 0, NULL).result;
    case 28:
        return maniBuildFbxClip(b, NULL, 0, NULL, NULL, NULL, &s_out.bound).result;
    case 29:
        return maniBuildLearnedModel(b, NULL).result;
    case 30:
        return maniBuildLearnedModelFromTrainer(b, NULL, (maniLearnedWeights)0).result;
    case 31:
        return maniBuildMotionDatabase(b, NULL).result;
    case 32:
        return maniBuildRetargetMap(b, NULL).result;
    case 33:
        return maniBuildRetargetMapJson(b, NULL, 0, NULL, NULL).result;
    case 34:
        return maniWriteRetargetMapJson(b, NULL).result;
    case 35:
        return maniBuildRetargetedClip(b, NULL).result;
    case 36:
        return maniBuildMirrorMap(b, NULL).result;
    case 37:
        return maniBuildMirrorMapJson(b, NULL, 0, NULL).result;
    case 38:
        return maniWriteMirrorMapJson(b, NULL).result;
    case 39:
        return maniBuildMirroredClip(b, NULL).result;
    case kDestroy:
        maniDestroyBuilder(b);
        return mani_errorBusy;
    case kCloseGltf:
        maniCloseGltf(s_gltf);
        return mani_errorBusy;
    default:
        maniCloseFbx(s_fbx);
        return mani_errorBusy;
    }
}

// Aligned blocks from malloc: the offset to the block kept before it.
static void* Aligned(size_t size, size_t alignment)
{
    alignment = alignment < sizeof(void*) ? sizeof(void*) : alignment;
    uint8_t* raw = malloc(size + alignment + sizeof(void*));
    if (raw == NULL)
    {
        return NULL;
    }
    uintptr_t at = ((uintptr_t)raw + sizeof(void*) + alignment - 1) & ~(uintptr_t)(alignment - 1);
    memcpy((uint8_t*)at - sizeof(void*), &raw, sizeof raw);
    return (void*)at;
}

static void Unaligned(void* block)
{
    if (block != NULL)
    {
        void* raw = NULL;
        memcpy(&raw, (uint8_t*)block - sizeof(void*), sizeof raw);
        free(raw);
    }
}

static void CallBack(void)
{
    int k = s_call;
    s_call = -1;
    s_result = CallBuilder(k);
}

static void* BuilderAlloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    if (s_call >= 0 && !s_onFree)
    {
        CallBack();
    }
    return Aligned(size, alignment);
}

// The report a measure in progress fills, measured into again from the
// next free, and what that returned.
static maniClipReport* s_measureInto;
static maniResult s_measured;

// Called from every free while set, before the block goes.
static void (*s_eachFree)(const void* memory);

static void BuilderFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    if (s_call >= 0 && s_onFree)
    {
        CallBack();
    }
    if (s_eachFree != NULL)
    {
        s_eachFree(memory);
    }
    if (s_measureInto != NULL)
    {
        maniClipReport* report = s_measureInto;
        s_measureInto = NULL;
        s_measured = maniBuilder_MeasureClip(s_builder, NULL, NULL, report);
    }
    Unaligned(memory);
}

static const char kGltf[] = "{\"asset\": {\"version\": \"2.0\"}}";
static const char kFbx[] = "; FBX 7.4.0 project file\n"
                           "FBXHeaderExtension:  {\n"
                           "\tFBXHeaderVersion: 1003\n"
                           "\tFBXVersion: 7400\n"
                           "}\n";

// A skeleton of three bones built, its bytes copied.
static bool Build(uint8_t* out, size_t capacity, size_t* size)
{
    static const maniBoneSource kBones[3] = {
        {"a", 1, -1, {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"b", 1, 0, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}},
        {"c", 1, 1, {{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}}}};
    const maniSkeletonSource source = {kBones, 3};
    return maniBuildSkeleton(s_builder, &source).result == mani_success &&
           maniBuilder_CopyBuilt(s_builder, out, capacity, size) == mani_success;
}

// A clip of the three bones whose last bone steps to a key between two
// samples, so its report is not all zeros: measured alone, then with its
// free hook measuring again into the same report, written by then.
static void TestMeasureLeft(void)
{
    static _Alignas(16) uint8_t s_skeletonFile[1024];
    static _Alignas(16) uint8_t s_clipFile[4096];
    size_t size = 0;
    maniSkeleton skeleton;
    CHECK(Build(s_skeletonFile, sizeof s_skeletonFile, &size) &&
              maniOpenSkeleton(s_skeletonFile, size, &skeleton).result == mani_success,
          "a skeleton to measure a clip of");
    static const float kTimes[2] = {0.0f, 0.01f};
    static const float kValues[6] = {0.0f, 1.0f, 0.0f, 0.5f, 1.0f, 0.0f};
    const maniBoneTrackSource track = {2, {kTimes, kValues, 2, maniInterpolationStep}, {0}, {0}};
    const maniClipSource source = {
        .skeleton = &skeleton, .boneTracks = &track, .boneTrackCount = 1, .duration = 0.1f};
    maniClip clip;
    CHECK(maniBuildClip(s_builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(s_builder, s_clipFile, sizeof s_clipFile, &size) ==
                  mani_success &&
              maniOpenClip(s_clipFile, size, &clip).result == mani_success,
          "the clip");
    maniClipReport alone;
    CHECK(maniBuilder_MeasureClip(s_builder, &source, &clip, &alone) == mani_success &&
              !Cleared(&alone, sizeof alone),
          "measured alone, not all zeros");
    uint32_t before = maniBuilder_GetMisuse(s_builder);
    maniClipReport report;
    Fill(&report, sizeof report);
    s_measureInto = &report;
    s_measured = mani_success;
    CHECK(maniBuilder_MeasureClip(s_builder, &source, &clip, &report) == mani_success &&
              s_measureInto == NULL && s_measured == mani_errorBusy &&
              maniBuilder_GetMisuse(s_builder) == before + 1,
          "the measure from its free hook refused and counted once");
    CHECK(memcmp(&report, &alone, sizeof report) == 0, "the report the measure wrote left");
}

static void TestBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    def.allocator = (maniAllocator){BuilderAlloc, BuilderFree, NULL};
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success, "a builder");
    CHECK(maniOpenGltf(s_builder, kGltf, sizeof kGltf - 1, &s_gltf).result == mani_success &&
              maniOpenFbx(s_builder, kFbx, sizeof kFbx - 1, &s_fbx).result == mani_success,
          "documents opened with it");
    static uint8_t s_alone[1024];
    static uint8_t s_built[1024];
    size_t alone = 0;
    CHECK(Build(s_alone, sizeof s_alone, &alone), "a build without calls back in");
    bool refused = true;
    bool same = true;
    bool left = true;
    for (int onFree = 0; onFree < 2; ++onFree)
    {
        for (int k = 0; k < kCalls; ++k)
        {
            uint32_t before = maniBuilder_GetMisuse(s_builder);
            s_onFree = onFree == 1;
            s_call = k;
            s_result = mani_success;
            Fill(&s_out, sizeof s_out);
            size_t size = 0;
            same = same && Build(s_built, sizeof s_built, &size) && size == alone &&
                   memcmp(s_built, s_alone, size) == 0;
            refused = refused && s_call == -1 && s_result == mani_errorBusy &&
                      maniBuilder_GetMisuse(s_builder) == before + 1;
            left = left && OutputLeft(k);
        }
    }
    CHECK(refused, "every call back in refused and counted once");
    CHECK(left, "every refused call's outputs left, a document's handle NULL");
    CHECK(same, "each build as without it");
    CHECK(maniGltf_GetNodeCount(s_gltf) == 0 && maniFbx_GetSkeletonCount(s_fbx) == 0,
          "the documents still open");
    maniCloseGltf(s_gltf);
    maniCloseFbx(s_fbx);
    TestMeasureLeft();
    uint32_t before = maniBuilder_GetMisuse(s_builder);
    CHECK(maniBuildSkeleton(s_builder, NULL).result == mani_errorInvalid &&
              maniBuilder_GetMisuse(s_builder) == before + 1,
          "outside the allocator the same calls refused as invalid");
    maniDestroyBuilder(s_builder);
}

// ---- Destroys and closes, gone to the host's code from their start.

// What the frees of a destroy or close saw: how many ran, whether one
// was the object's own block, and whether every call made from them
// was refused or read the object as gone.
static int s_frees;
static bool s_ownBlock;
static bool s_gone;

// From a builder's destroy: every call taking it, a build that would
// succeed, and the destroy again, each refused and counted once. The
// destroy is tried only once the others were refused, since one the
// builder let through would free it twice.
static void BuilderGone(const void* memory)
{
    ++s_frees;
    s_ownBlock = s_ownBlock || memory == s_builder;
    uint32_t before = maniBuilder_GetMisuse(s_builder);
    bool refused = true;
    for (int k = 0; k < kDestroy; ++k)
    {
        refused = refused && CallBuilder(k) == mani_errorBusy;
    }
    uint8_t out[1024];
    size_t size = 0;
    refused = refused && !Build(out, sizeof out, &size);
    if (refused)
    {
        maniDestroyBuilder(s_builder);
    }
    s_gone = s_gone && refused && maniBuilder_GetMisuse(s_builder) == before + kDestroy + 2;
}

static void TestDestroyBuilder(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    def.allocator = (maniAllocator){BuilderAlloc, BuilderFree, NULL};
    uint8_t out[1024];
    size_t size = 0;
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success && Build(out, sizeof out, &size),
          "a builder holding a file");
    s_gltf = NULL;
    s_fbx = NULL;
    s_frees = 0;
    s_ownBlock = false;
    s_gone = true;
    s_eachFree = BuilderGone;
    maniDestroyBuilder(s_builder);
    s_eachFree = NULL;
    CHECK(s_frees >= 2 && s_ownBlock, "the destroy's frees seen, its own block's among them");
    CHECK(s_gone, "from each, every call on the builder refused as busy and counted once");
}

static const char kGltfNodes[] = "{\"asset\": {\"version\": \"2.0\"}, \"nodes\": "
                                 "[{\"name\": \"hip\", \"children\": [1]}, {\"name\": \"leg\"}]}";
static const char kFbxModels[] = "; FBX 7.4.0 project file\n"
                                 "Objects: {\n"
                                 " Model: 100, \"Model::hip\", \"LimbNode\" {\n }\n"
                                 " Model: 200, \"Model::leg\", \"LimbNode\" {\n }\n"
                                 "}\n"
                                 "Connections: {\n"
                                 " C: \"OO\",100,0\n"
                                 " C: \"OO\",200,100\n"
                                 "}\n";

// From a glTF document's close: every reader finds it empty, and its
// changes and the close again are refused and counted on the builder.
static void GltfGone(const void* memory)
{
    (void)memory;
    ++s_frees;
    const maniGltf* g = s_gltf;
    const maniGltfOptions options = maniDefaultGltfOptions();
    size_t length = 1;
    uint32_t count = 1;
    maniAffine placement;
    bool empty = maniGltf_GetBufferCount(g) == 0 && maniGltf_GetBufferUri(g, 0, &length) == NULL &&
                 length == 0 && maniGltf_GetSkeletonCount(g) == 0 &&
                 maniGltf_GetSkinCount(g) == 0 && maniGltf_GetSkinSkeleton(g, 0) == UINT32_MAX &&
                 maniGltf_GetNodeCount(g) == 0 &&
                 maniGltf_GetSkeletonPlacement(g, 0, &options, &placement).result != mani_success;
    length = 1;
    empty = empty && maniGltf_GetNodeBoneName(g, 0, &length) == NULL && length == 0 &&
            maniGltf_GetRenamedCount(g) == 0 && maniGltf_GetRenamedNode(g, 0) == UINT32_MAX &&
            maniGltf_GetNodeMesh(g, 0) == UINT32_MAX && maniGltf_GetNodeSkin(g, 0) == UINT32_MAX &&
            maniGltf_GetAnimationCount(g) == 0;
    length = 1;
    empty = empty && maniGltf_GetAnimationName(g, 0, &length) == NULL && length == 0 &&
            maniGltf_GetMeshCount(g) == 0 && maniGltf_GetPrimitiveCount(g, 0) == 0 &&
            maniGltf_GetPrimitiveTargetCount(g, 0, 0) == 0 &&
            maniGltf_GetMorphWeights(g, 0, NULL, 0, &count).result != mani_success && count == 0;
    uint32_t before = maniBuilder_GetMisuse(s_builder);
    maniSpringSetSource springs;
    maniRagdollSource ragdoll;
    const maniVec3 z = {0.0f, 0.0f, 1.0f};
    const maniVec3 y = {0.0f, 1.0f, 0.0f};
    bool refused =
        maniGltf_SupplyBuffer(s_gltf, 0, "x", 1).result == mani_errorBusy &&
        maniGltf_ReadSprings(s_gltf, NULL, &options, &springs).result == mani_errorBusy &&
        maniGltf_ReadRagdoll(s_gltf, NULL, &options, z, y, &ragdoll).result == mani_errorBusy;
    maniCloseGltf(s_gltf);
    s_gone = s_gone && empty && refused && maniBuilder_GetMisuse(s_builder) == before + 4;
}

// The same from an FBX document's close.
static void FbxGone(const void* memory)
{
    (void)memory;
    ++s_frees;
    const maniFbx* f = s_fbx;
    const maniFbxOptions options = maniDefaultFbxOptions();
    size_t length = 1;
    uint32_t count = 1;
    maniAffine placement;
    bool empty =
        maniFbx_GetVersion(f) == 0 && maniFbx_GetModelCount(f) == 0 &&
        maniFbx_GetModelBoneName(f, 0, &length) == NULL && length == 0 &&
        maniFbx_GetRenamedCount(f) == 0 && maniFbx_GetRenamedModel(f, 0) == UINT32_MAX &&
        maniFbx_GetSkeletonCount(f) == 0 && maniFbx_GetSkinCount(f) == 0 &&
        maniFbx_GetSkinSkeleton(f, 0) == UINT32_MAX && maniFbx_GetSkinModel(f, 0) == UINT32_MAX &&
        maniFbx_GetSkinMethod(f, 0) == maniSkinningLinear && maniFbx_GetMorphCount(f) == 0 &&
        maniFbx_GetMorphModel(f, 0) == UINT32_MAX && maniFbx_GetMorphTargetCount(f, 0) == 0 &&
        maniFbx_GetMorphWeights(f, 0, NULL, 0, &count).result != mani_success && count == 0 &&
        maniFbx_GetClipCount(f) == 0 &&
        maniFbx_GetSkeletonPlacement(f, 0, &options, &placement).result != mani_success &&
        maniFbx_GetModelPlacement(f, 0, &options, &placement).result != mani_success;
    length = 1;
    maniFbxSettings settings;
    empty = empty && maniFbx_GetClipName(f, 0, &length) == NULL && length == 0 &&
            maniFbx_ReadSettings(f, &settings) == mani_success;
    uint32_t before = maniBuilder_GetMisuse(s_builder);
    maniCloseFbx(s_fbx);
    s_gone = s_gone && empty && maniBuilder_GetMisuse(s_builder) == before + 1;
}

static void TestCloseDocuments(void)
{
    maniBuilderDef def = maniDefaultBuilderDef();
    def.allocator = (maniAllocator){BuilderAlloc, BuilderFree, NULL};
    CHECK(maniCreateBuilder(&def, &s_builder) == mani_success &&
              maniOpenGltf(s_builder, kGltfNodes, sizeof kGltfNodes - 1, &s_gltf).result ==
                  mani_success &&
              maniOpenFbx(s_builder, kFbxModels, sizeof kFbxModels - 1, &s_fbx).result ==
                  mani_success,
          "documents with nodes and models");
    CHECK(maniGltf_GetNodeCount(s_gltf) == 2 && maniFbx_GetModelCount(s_fbx) == 2,
          "open, they read as full");
    s_frees = 0;
    s_gone = true;
    s_eachFree = GltfGone;
    maniCloseGltf(s_gltf);
    s_eachFree = NULL;
    CHECK(s_frees > 0 && s_gone, "from each free of a glTF close, the document empty and its "
                                 "changes and close refused as busy");
    s_frees = 0;
    s_gone = true;
    s_eachFree = FbxGone;
    maniCloseFbx(s_fbx);
    s_eachFree = NULL;
    CHECK(s_frees > 0 && s_gone,
          "from each free of an FBX close, the document empty and its close refused");
    maniDestroyBuilder(s_builder);
}

// ---- A trainer's allocator and task hooks.

enum
{
    kTrain,
    kRestore,
    kDestroyTrainer,
    kTrainerCalls,
};

typedef enum Where
{
    kAlloc,
    kFree,
    kEnqueue,
    kFinish,
    kPlaces,
} Where;

static maniLearnedTrainer* s_trainer;
static int s_trainerCall = -1;
static Where s_where;
static maniResult s_trainerResult;
static maniLearnedTrainerReport s_trainerReport;
static bool s_trainerLeft = true;

static void TrainerCallBack(Where where)
{
    if (s_trainerCall < 0 || s_where != where)
    {
        return;
    }
    int k = s_trainerCall;
    s_trainerCall = -1;
    switch (k)
    {
    case kTrain:
        Fill(&s_trainerReport, sizeof s_trainerReport);
        s_trainerResult = maniLearnedTrainer_Train(s_trainer, 0, &s_trainerReport);
        s_trainerLeft = s_trainerLeft && Kept(&s_trainerReport, sizeof s_trainerReport);
        break;
    case kRestore:
        s_trainerResult = maniLearnedTrainer_RestoreCheckpoint(s_trainer, NULL, 0).result;
        break;
    default:
        maniDestroyLearnedTrainer(s_trainer);
        s_trainerResult = mani_errorBusy;
        break;
    }
}

static void* TrainerAlloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    TrainerCallBack(kAlloc);
    return Aligned(size, alignment);
}

static void (*s_eachTrainerFree)(const void* memory);

static void TrainerFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    TrainerCallBack(kFree);
    if (s_eachTrainerFree != NULL)
    {
        s_eachTrainerFree(memory);
    }
    Unaligned(memory);
}

static void* Enqueue(maniTaskFunction* task, int32_t count, int32_t minRange, void* context,
                     void* user)
{
    (void)minRange;
    (void)user;
    TrainerCallBack(kEnqueue);
    task(0, count, context);
    return &s_trainer;
}

static void Finish(void* handle, void* user)
{
    (void)handle;
    (void)user;
    TrainerCallBack(kFinish);
}

static void TestTrainer(void)
{
    static CheckpointHost s_host;
    maniBuilderDef builderDef = maniDefaultBuilderDef();
    maniBuilder* builder = NULL;
    CHECK(maniCreateBuilder(&builderDef, &builder) == mani_success && HostAssets(&s_host, builder),
          "the trainer's assets");
    maniDestroyBuilder(builder);
    static const maniLearnedTagFilter kEvery = {0, 0};
    maniLearnedTrainerSource source = {.skeleton = &s_host.skeleton,
                                       .database = &s_host.database,
                                       .clips = s_host.clips,
                                       .projectors = &kEvery,
                                       .clipCount = 1,
                                       .projectorCount = 1,
                                       .options = {2, 4, 1, 1, 1, 1, 4, 2, 3, 7, 0.001f, 0.1f}};
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    def.allocator = (maniAllocator){TrainerAlloc, TrainerFree, NULL};
    def.tasks = (maniTaskHooks){Enqueue, Finish, NULL};
    bool refused = true;
    bool trained = true;
    for (int where = 0; where < kPlaces; ++where)
    {
        for (int k = 0; k < kTrainerCalls; ++k)
        {
            // A trainer at its start each time: a stage begins and ends
            // within the call, allocating, freeing and running tasks.
            trained = trained &&
                      maniCreateLearnedTrainer(&def, &source, &s_trainer).result == mani_success;
            s_where = (Where)where;
            s_trainerCall = k;
            s_trainerResult = mani_success;
            maniLearnedTrainerReport report;
            trained = trained && maniLearnedTrainer_Train(s_trainer, 4, &report) == mani_success;
            refused = refused && s_trainerCall == -1 && s_trainerResult == mani_errorBusy &&
                      maniLearnedTrainer_GetMisuse(s_trainer) == 1;
            s_trainerCall = -1;
            maniDestroyLearnedTrainer(s_trainer);
        }
    }
    CHECK(trained, "training goes on");
    CHECK(refused, "every call back in from the trainer's code refused and counted once");
    CHECK(s_trainerLeft, "training refused with its report left");
}

// From a trainer's destroy: training, restoring and the destroy again,
// each refused and counted once (the destroy tried only once the others
// were, as for the builder).
static void TrainerGone(const void* memory)
{
    ++s_frees;
    s_ownBlock = s_ownBlock || memory == s_trainer;
    uint32_t before = maniLearnedTrainer_GetMisuse(s_trainer);
    maniLearnedTrainerReport report;
    bool refused =
        maniLearnedTrainer_Train(s_trainer, 1, &report) == mani_errorBusy &&
        maniLearnedTrainer_RestoreCheckpoint(s_trainer, NULL, 0).result == mani_errorBusy;
    if (refused)
    {
        maniDestroyLearnedTrainer(s_trainer);
    }
    s_gone = s_gone && refused && maniLearnedTrainer_GetMisuse(s_trainer) == before + 3;
}

static void TestDestroyTrainer(void)
{
    static CheckpointHost s_host;
    maniLearnedTrainer* plain = MakeCheckpointHost(&s_host);
    CHECK(plain != NULL, "the trainer's assets");
    maniDestroyLearnedTrainer(plain);
    static const maniLearnedTagFilter kEvery = {0, 0};
    maniLearnedTrainerSource source = {.skeleton = &s_host.skeleton,
                                       .database = &s_host.database,
                                       .clips = s_host.clips,
                                       .projectors = &kEvery,
                                       .clipCount = 1,
                                       .projectorCount = 1,
                                       .options = {2, 4, 1, 1, 1, 1, 4, 2, 3, 7, 0.001f, 0.1f}};
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    def.allocator = (maniAllocator){TrainerAlloc, TrainerFree, NULL};
    maniLearnedTrainerReport report;
    CHECK(maniCreateLearnedTrainer(&def, &source, &s_trainer).result == mani_success &&
              maniLearnedTrainer_Train(s_trainer, 4, &report) == mani_success,
          "a trainer partway");
    s_frees = 0;
    s_ownBlock = false;
    s_gone = true;
    s_eachTrainerFree = TrainerGone;
    maniDestroyLearnedTrainer(s_trainer);
    s_eachTrainerFree = NULL;
    CHECK(s_frees >= 2 && s_ownBlock, "the destroy's frees seen, its own block's among them");
    CHECK(s_gone, "from each, every call on the trainer refused as busy and counted once");
}

int main(void)
{
    TestBuilder();
    TestDestroyBuilder();
    TestCloseDocuments();
    TestTrainer();
    TestDestroyTrainer();
    return s_failures == 0 ? 0 : 1;
}
