// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Calls back in from the host's code: a builder's allocator,
// as it allocates and as it frees inside a build, calls every function
// taking that builder (each with arguments refused otherwise as invalid,
// so the state error is the call back in alone), destroys it and closes
// a glTF and an FBX document opened with it: each refused with
// mani_errorState (nothing destroyed or closed) and counted once, the
// build finishing as it does without. A trainer's allocator and task
// hooks call training, restoring and destroying it, likewise refused,
// counted and harmless.

#include "checkpoint_host.h"
#include "test_harness.h"

#include "maul-anim/base.h"
#include "maul-anim/build.h"
#include "maul-anim/clip.h"
#include "maul-anim/graphbuild.h"
#include "maul-anim/import.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/learnedtrain.h"
#include "maul-anim/matchbuild.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// The calls made back in: every function taking a builder, then its
// destroy and the two documents' closes.
enum
{
    kBuilderCalls = 32,
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
        return maniMeasureClip(b, NULL, NULL, NULL);
    case 6:
        return maniCompressClip(b, NULL, NULL, NULL, NULL);
    case 7:
        return maniBuildSpringSet(b, NULL).result;
    case 8:
        return maniBuildRagdoll(b, NULL).result;
    case 9:
        return maniCopyBuilt(b, NULL, 1, NULL);
    case 10:
        return maniCopySkeletonOrder(b, NULL, 1, NULL);
    case 11:
        return maniBuildMorphTargets(b, NULL).result;
    case 12:
        return maniBuildGraph(b, NULL).result;
    case 13:
        return maniWriteGraphJson(b, NULL).result;
    case 14:
        return maniBuildGraphJson(b, NULL, 0, NULL).result;
    case 15:
        return maniOpenGltf(b, NULL, 0, NULL).result;
    case 16:
        return maniGltfBuildSkeleton(b, NULL, 0, NULL).result;
    case 17:
        return maniGltfBuildNodeSkeleton(b, NULL, 0, NULL).result;
    case 18:
        return maniGltfBuildSkin(b, NULL, 0, NULL, NULL).result;
    case 19:
        return maniGltfBuildClip(b, NULL, 0, NULL, NULL, NULL, NULL).result;
    case 20:
        return maniGltfBuildInfluences(b, NULL, 0, 0, NULL).result;
    case 21:
        return maniGltfBuildMorphTargets(b, NULL, 0, 0, NULL).result;
    case 22:
        return maniOpenFbx(b, NULL, 0, NULL).result;
    case 23:
        return maniFbxBuildSkeleton(b, NULL, 0, NULL).result;
    case 24:
        return maniFbxBuildModelSkeleton(b, NULL, 0, NULL).result;
    case 25:
        return maniFbxBuildSkin(b, NULL, 0, NULL, NULL).result;
    case 26:
        return maniFbxBuildInfluences(b, NULL, 0, NULL).result;
    case 27:
        return maniFbxBuildMorphTargets(b, NULL, 0, NULL).result;
    case 28:
        return maniFbxBuildClip(b, NULL, 0, NULL, NULL, NULL, NULL).result;
    case 29:
        return maniBuildLearnedModel(b, NULL).result;
    case 30:
        return maniBuildLearnedModelFromTrainer(b, NULL, (maniLearnedWeights)0).result;
    case 31:
        return maniBuildMotionDatabase(b, NULL).result;
    case kDestroy:
        maniDestroyBuilder(b);
        return mani_errorState;
    case kCloseGltf:
        maniCloseGltf(s_gltf);
        return mani_errorState;
    default:
        maniCloseFbx(s_fbx);
        return mani_errorState;
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

static void BuilderFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    if (s_call >= 0 && s_onFree)
    {
        CallBack();
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
           maniCopyBuilt(s_builder, out, capacity, size) == mani_success;
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
    for (int onFree = 0; onFree < 2; ++onFree)
    {
        for (int k = 0; k < kCalls; ++k)
        {
            uint32_t before = maniBuilderMisuses(s_builder);
            s_onFree = onFree == 1;
            s_call = k;
            s_result = mani_success;
            size_t size = 0;
            same = same && Build(s_built, sizeof s_built, &size) && size == alone &&
                   memcmp(s_built, s_alone, size) == 0;
            refused = refused && s_call == -1 && s_result == mani_errorState &&
                      maniBuilderMisuses(s_builder) == before + 1;
        }
    }
    CHECK(refused, "every call back in refused and counted once");
    CHECK(same, "each build as without it");
    CHECK(maniGltfNodeCount(s_gltf) == 0 && maniFbxSkeletonCount(s_fbx) == 0,
          "the documents still open");
    maniCloseGltf(s_gltf);
    maniCloseFbx(s_fbx);
    uint32_t before = maniBuilderMisuses(s_builder);
    CHECK(maniBuildSkeleton(s_builder, NULL).result == mani_errorInvalid &&
              maniBuilderMisuses(s_builder) == before + 1,
          "outside the allocator the same calls refused as invalid");
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
        s_trainerResult = maniTrainLearned(s_trainer, 0, NULL);
        break;
    case kRestore:
        s_trainerResult = maniRestoreLearnedCheckpoint(s_trainer, NULL, 0).result;
        break;
    default:
        maniDestroyLearnedTrainer(s_trainer);
        s_trainerResult = mani_errorState;
        break;
    }
}

static void* TrainerAlloc(size_t size, size_t alignment, void* context)
{
    (void)context;
    TrainerCallBack(kAlloc);
    return Aligned(size, alignment);
}

static void TrainerFree(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    TrainerCallBack(kFree);
    Unaligned(memory);
}

static void* Enqueue(maniTaskFn* task, int32_t count, int32_t minRange, void* context, void* user)
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
            trained = trained && maniTrainLearned(s_trainer, 4, &report) == mani_success;
            refused = refused && s_trainerCall == -1 && s_trainerResult == mani_errorState &&
                      maniLearnedTrainerMisuses(s_trainer) == 1;
            s_trainerCall = -1;
            maniDestroyLearnedTrainer(s_trainer);
        }
    }
    CHECK(trained, "training goes on");
    CHECK(refused, "every call back in from the trainer's code refused and counted once");
}

int main(void)
{
    TestBuilder();
    TestTrainer();
    return s_failures == 0 ? 0 : 1;
}
