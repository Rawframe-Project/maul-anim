// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The learned model trainer over a database of the walk (tag
// 1) and the run (tag 2): its def and source refused for each fault
// with its reason and item; its stages and report; the model built from
// it, whose networks give the trainer's own bits and whose file is the
// same on any host, one whose worker threads flush denormals and round
// toward zero too; the learning rate's decay, each stage's generator
// and filter, and the Decompressor kept, against the stages run alone;
// checkpoints at any point restored to the same model and loss, and
// refused when of another source, damaged or out of form; memory
// running out at creation, a stage's beginning or a restore. Run as
// test_learned_train --seed <file> it writes the checkpoint fuzz
// target's seed: its host's trainer stopped in its first Projector's
// stage.

// The seed writer uses fopen, which the Microsoft C library deprecates;
// the request comes before any header that reaches the C library.
#define _CRT_SECURE_NO_WARNINGS

#include "checkpoint_host.h"
#include "container.h"
#include "file_tag.h"
#include "fpenv.h"
#include "graph_match_fixture.h"
#include "learned_file.h"
#include "learned_train.h"
#include "source_padding.h"
#include "train_decompress.h"
#include "train_net.h"
#include "train_projector.h"
#include "train_random.h"
#include "train_solo.h"
#include "train_stepper.h"

#include "maul-anim/learned.h"
#include "maul-anim/learnedbuild.h"
#include "maul-anim/learnedtrain.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static _Alignas(16) uint8_t s_boneFile[1 << 15];
static maniMotionDatabase s_bones;
static const maniClip* s_trainClips[2];
static const maniAllocator kAllocator = {NULL, NULL, NULL};
static const maniLearnedTagFilter kFilters[2] = {{0, 0}, {2, 0}};
static const maniLearnedNameSource kEvents[2] = {{"left", 4}, {"right", 5}};

static void OpenBones(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 2, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    BuildDatabase(channels, 2, s_boneFile, sizeof s_boneFile, &s_bones);
    s_trainClips[0] = &s_clips[0];
    s_trainClips[1] = &s_clips[1];
}

// Latent 4, hidden 8, one hidden layer each, batch 4, window 4, 30
// iterations a stage.
static maniLearnedTrainerSource Source(void)
{
    maniLearnedTrainerSource s = {
        &s_skeleton, &s_bones, s_trainClips, kFilters, NULL, kEvents, 2, 2, 0, 2, {0}};
    s.options = maniDefaultLearnedTrainerOptions();
    s.options.latentCount = 4;
    s.options.hiddenWidth = 8;
    s.options.compressorLayers = 1;
    s.options.decompressorLayers = 1;
    s.options.stepperLayers = 1;
    s.options.projectorLayers = 1;
    s.options.batchSize = 4;
    s.options.stepperWindow = 4;
    s.options.iterations = 30;
    s.options.seed = 7;
    s.options.learningRate = 0.003f;
    return s;
}

static maniLearnedTrainer* Create(const maniLearnedTrainerDef* def,
                                  const maniLearnedTrainerSource* source)
{
    maniLearnedTrainer* t = NULL;
    maniBuildResult r = maniCreateLearnedTrainer(def, source, &t);
    CHECK(r.result == mani_success && t != NULL, "a trainer");
    return t;
}

static bool RefusedWith(const maniLearnedTrainerSource* s, maniResult result,
                        maniBuildReason reason, uint32_t item)
{
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = (maniLearnedTrainer*)&def;
    maniBuildResult r = maniCreateLearnedTrainer(&def, s, &t);
    return r.result == result && r.reason == reason && r.item == item && t == NULL;
}

static bool RefusedFor(const maniLearnedTrainerSource* s, maniBuildReason reason, uint32_t item)
{
    return RefusedWith(s, mani_errorInvalid, reason, item);
}

// Refused for passing a fixed bound.
static bool RefusedPast(const maniLearnedTrainerSource* s, maniBuildReason reason, uint32_t item)
{
    return RefusedWith(s, mani_errorLimit, reason, item);
}

// A host that runs ranges of one item, last first.
static void* Backwards(maniTaskFunction* task, int32_t count, int32_t minRange, void* context,
                       void* user)
{
    (void)minRange;
    (void)user;
    for (int32_t k = count; k-- > 0;)
    {
        task(k, k + 1, context);
    }
    return NULL;
}

static void Finish(void* task, void* user)
{
    (void)task;
    (void)user;
}

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// Flush-to-zero, denormals-are-zero, round toward zero, exceptions masked.
static const uint64_t s_hostSetting = 0x1f80u | (1u << 15) | (1u << 6) | (3u << 13);
#elif defined(__aarch64__) || defined(_M_ARM64)
// Flush-to-zero and round toward zero.
static const uint64_t s_hostSetting = (1ull << 24) | (3ull << 22);
#else
static const uint64_t s_hostSetting = 0;
#endif

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// MXCSR's sticky exception flags, which arithmetic sets on its own.
static const uint64_t s_flags = 0x3fu;
#else
static const uint64_t s_flags = 0;
#endif

// Hook calls that found a setting other than the host's: the hooks are
// host code, called in the host's setting.
static uint32_t s_hooksElsewhere;

// A host whose threads flush denormals and round toward zero. It runs
// each range with that setting in the register, as such a worker would,
// and puts the caller's back after.
static void* Hostile(maniTaskFunction* task, int32_t count, int32_t minRange, void* context,
                     void* user)
{
    (void)minRange;
    (void)user;
    if ((maniFloatControlRead() & ~s_flags) != (s_hostSetting & ~s_flags))
    {
        s_hooksElsewhere += 1;
    }
    for (int32_t k = 0; k < count; ++k)
    {
        uint64_t own = maniFloatControlRead();
        maniFloatControlWrite(s_hostSetting);
        task(k, k + 1, context);
        maniFloatControlWrite(own);
    }
    return NULL;
}

// An allocator that fails from its limit-th allocation on, aligning by
// hand (no aligned_alloc on every target).
typedef struct Budget
{
    uint32_t count;
    uint32_t limit;
} Budget;

static void* Allocate(size_t size, size_t alignment, void* context)
{
    Budget* b = context;
    if (b->count >= b->limit)
    {
        return NULL;
    }
    b->count += 1;
    uint8_t* raw = malloc(size + alignment + sizeof(void*));
    if (raw == NULL)
    {
        return NULL;
    }
    uintptr_t at = ((uintptr_t)(raw + sizeof(void*)) + alignment - 1) & ~(uintptr_t)(alignment - 1);
    memcpy((void*)(at - sizeof(void*)), &raw, sizeof raw);
    return (void*)at;
}

static void Free(void* memory, size_t size, size_t alignment, void* context)
{
    (void)size;
    (void)alignment;
    (void)context;
    void* raw = NULL;
    memcpy(&raw, (uint8_t*)memory - sizeof(void*), sizeof raw);
    free(raw);
}

static void TestDef(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = NULL;
    CHECK(maniCreateLearnedTrainer(&def, &s, NULL).result == mani_errorInvalid &&
              maniCreateLearnedTrainer(NULL, &s, &t).result == mani_errorInvalid &&
              maniCreateLearnedTrainer(&def, NULL, &t).result == mani_errorInvalid,
          "NULL arguments");
    def.version += 1;
    CHECK(maniCreateLearnedTrainer(&def, &s, &t).result == mani_errorVersion, "another version");
    def = maniDefaultLearnedTrainerDef();
    def.cookie = 0;
    CHECK(maniCreateLearnedTrainer(&def, &s, &t).result == mani_errorInvalid, "no cookie");
    memset(&def, 0, sizeof def);
    CHECK(maniCreateLearnedTrainer(&def, &s, &t).result == mani_errorInvalid,
          "a zeroed def refused by its cookie, before its version");
    def = maniDefaultLearnedTrainerDef();
    def.tasks.finishTask = Finish;
    CHECK(maniCreateLearnedTrainer(&def, &s, &t).result == mani_errorInvalid,
          "task hooks with one function");
    def = maniDefaultLearnedTrainerDef();
    def.allocator.free = Free;
    CHECK(maniCreateLearnedTrainer(&def, &s, &t).result == mani_errorInvalid,
          "an allocator with one function");
    def = maniDefaultLearnedTrainerDef();
    s.projectors = NULL;
    CHECK(maniCreateLearnedTrainer(&def, &s, &t).result == mani_errorInvalid && t == NULL,
          "no filters");
}

static void TestOptions(void)
{
    const maniLearnedTrainerSource base = Source();
    maniLearnedTrainerSource s = base;
    bool all = true;
    uint32_t* counts[8] = {&s.options.latentCount,      &s.options.hiddenWidth,
                           &s.options.compressorLayers, &s.options.decompressorLayers,
                           &s.options.stepperLayers,    &s.options.projectorLayers,
                           &s.options.batchSize,        &s.options.stepperWindow};
    const uint32_t bad[8][2] = {{0, MANI_MAX_LEARNED_LATENT + 1},
                                {0, MANI_MAX_LEARNED_WIDTH + 1},
                                {MANI_MAX_LEARNED_LAYERS, MANI_MAX_LEARNED_LAYERS},
                                {MANI_MAX_LEARNED_LAYERS, MANI_MAX_LEARNED_LAYERS},
                                {MANI_MAX_LEARNED_LAYERS, MANI_MAX_LEARNED_LAYERS},
                                {MANI_MAX_LEARNED_LAYERS, MANI_MAX_LEARNED_LAYERS},
                                {6, MANI_MAX_LEARNED_BATCH + 4},
                                {1, MANI_MAX_LEARNED_WINDOW + 1}};
    for (uint32_t k = 0; k < 8; ++k)
    {
        for (uint32_t b = 0; b < 2; ++b)
        {
            s = base;
            *counts[k] = bad[k][b];
            // The second of each pair, and every layer count, passes a
            // bound; the first is below its range.
            maniResult result = b == 1 || (k >= 2 && k <= 5) ? mani_errorLimit : mani_errorInvalid;
            all = all && RefusedWith(&s, result, maniBuildReasonOption, k);
        }
    }
    CHECK(all, "counts past their limits, with their places");
    s = base;
    s.options.batchSize = 0;
    CHECK(RefusedFor(&s, maniBuildReasonOption, 6), "an empty batch");
    s = base;
    s.options.iterations = 0;
    CHECK(RefusedFor(&s, maniBuildReasonOption, 8), "no iterations");
    const float rates[3] = {0.0f, -1.0f, HUGE_VALF};
    for (uint32_t k = 0; k < 3; ++k)
    {
        s = base;
        s.options.learningRate = rates[k];
        all = all && RefusedFor(&s, maniBuildReasonOption, 10);
        s = base;
        s.options.eventSeconds = k == 0 ? -0.5f : rates[k];
        all = all && RefusedFor(&s, maniBuildReasonOption, 11);
    }
    CHECK(all, "a rate or seconds out of range");
    s = base;
    s.options.stepperWindow = 12;
    CHECK(RefusedFor(&s, maniBuildReasonOption, 7), "a window longer than every entry");
    s.options.stepperWindow = 11;
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniDestroyLearnedTrainer(t);
    s = base;
    s.options.hiddenWidth = 16384;
    s.options.projectorLayers = 2;
    CHECK(RefusedPast(&s, maniBuildReasonOption, 1), "one network's weights past the limit");
    // Two Projectors of 12,000 by 12,000 weights each: under the limit
    // alone, past it together.
    s.options.hiddenWidth = 12000;
    CHECK(RefusedPast(&s, maniBuildReasonOption, 1), "the model's weights past the limit");
    s.projectorCount = 1;
    t = Create(&def, &s);
    maniDestroyLearnedTrainer(t);
}

static void TestSource(void)
{
    const maniLearnedTrainerSource base = Source();
    maniLearnedTrainerSource s = base;
    static _Alignas(16) uint8_t file[4096];
    maniSkeleton other;
    maniSkeletonSource source = {kBones, 2};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    CHECK(maniBuildSkeleton(builder, &source).result == mani_success &&
              maniBuilder_CopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenSkeleton(file, size, &other).result == mani_success,
          "another skeleton");
    maniDestroyBuilder(builder);
    s.skeleton = &other;
    CHECK(RefusedFor(&s, maniBuildReasonDatabase, 0), "a database of another skeleton");
    s = base;
    s.clipCount = 1;
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = NULL;
    maniBuildResult r = maniCreateLearnedTrainer(&def, &s, &t);
    CHECK(r.result == mani_errorInvalid && r.reason == maniBuildReasonClip && r.item == 1 &&
              r.other == 2,
          "a clip count not the database's");
    const maniClip* swapped[2] = {&s_clips[1], &s_clips[0]};
    s = base;
    s.clips = swapped;
    CHECK(RefusedFor(&s, maniBuildReasonClip, 0), "clips out of the database's order");
    const maniClip* missing[2] = {&s_clips[0], NULL};
    s.clips = missing;
    CHECK(RefusedFor(&s, maniBuildReasonClip, 1), "a clip missing");
    s = base;
    s.projectorCount = 0;
    CHECK(RefusedFor(&s, maniBuildReasonCount, 0), "no Projector");
    s = base;
    s.eventCount = MANI_MAX_LEARNED_EVENTS + 1;
    CHECK(RefusedPast(&s, maniBuildReasonCount, 0), "too many events");
    const maniLearnedTagFilter filters[2] = {{0, 0}, {4, 0}};
    s = base;
    s.projectors = filters;
    CHECK(RefusedFor(&s, maniBuildReasonTags, 1), "a filter no frame passes");
    const maniLearnedNameSource twice[2] = {{"left", 4}, {"left", 4}};
    s = base;
    s.events = twice;
    CHECK(RefusedFor(&s, maniBuildReasonEvent | maniBuildReasonDuplicateName, 1),
          "an event named twice");
    const maniLearnedNameSource empty[1] = {{"", 0}};
    s = base;
    s.floatTracks = empty;
    s.floatTrackCount = 1;
    CHECK(RefusedFor(&s, maniBuildReasonFloatTrack | maniBuildReasonName, 0),
          "a float track without a name");
}

static _Alignas(16) uint8_t s_model[2][1 << 16];
static size_t s_modelSize[2];

// The model a finished trainer gives, into s_model[slot].
static void Build(const maniLearnedTrainer* t, uint32_t slot, maniLearnedWeights format)
{
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildLearnedModelFromTrainer(builder, t, format).result == mani_success &&
              maniBuilder_CopyBuilt(builder, s_model[slot], sizeof s_model[slot],
                                    &s_modelSize[slot]) == mani_success,
          "the model");
    maniDestroyBuilder(builder);
}

static void TestStages(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 0, &report) == mani_success && report.stage == 0 &&
              report.stageCount == 4 && report.iteration == 0 && report.loss == 0.0f && !t->begun,
          "nothing run: stage 0 not begun");
    maniBuilder* builder = NewBuilder();
    CHECK(maniBuildLearnedModelFromTrainer(builder, t, maniLearnedWeightsFloat32).result ==
                  mani_errorInvalid &&
              maniBuilder_GetMisuse(builder) == 1,
          "no model before every stage is done");
    CHECK(maniBuildLearnedModelFromTrainer(builder, NULL, maniLearnedWeightsFloat32).result ==
                  mani_errorInvalid &&
              maniBuildLearnedModelFromTrainer(NULL, t, maniLearnedWeightsFloat32).result ==
                  mani_errorInvalid &&
              maniBuilder_GetMisuse(builder) == 2,
          "NULL arguments");
    maniDestroyBuilder(builder);
    CHECK(maniLearnedTrainer_Train(t, 29, &report) == mani_success && report.stage == 0 &&
              report.iteration == 29 && report.loss > 0.0f && t->latents == NULL,
          "29 iterations of stage 0");
    CHECK(maniLearnedTrainer_Train(t, 1, &report) == mani_success && report.stage == 1 &&
              report.iteration == 0 && report.loss == 0.0f && t->latents != NULL && !t->begun &&
              t->networks[0].parameters != NULL,
          "stage 0 ends with its last iteration");
    CHECK(maniLearnedTrainer_Train(t, 31, &report) == mani_success && report.stage == 2 &&
              report.iteration == 1 && t->networks[1].parameters != NULL,
          "the Stepper's 30, then the first Projector's first");
    CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success && report.stage == 4 &&
              report.stageCount == 4 && report.iteration == 0 && !t->begun,
          "every stage done");
    CHECK(maniLearnedTrainer_Train(t, 5, &report) == mani_success && report.stage == 4,
          "nothing more to run");
    CHECK(maniLearnedTrainer_Train(NULL, 1, &report) == mani_errorInvalid &&
              maniLearnedTrainer_Train(t, 1, NULL) == mani_errorInvalid,
          "NULL arguments");
    Build(t, 0, maniLearnedWeightsFloat32);
    maniLearnedModel model;
    CHECK(maniOpenLearnedModel(s_model[0], s_modelSize[0], &model).result == mani_success &&
              model.projectorCount == 2 && model.latentCount == 4 && model.stepRate == 10.0f &&
              model.featureCount == 5 && model.eventCount == 2,
          "the model opens");
    Build(t, 1, maniLearnedWeightsInt16);
    CHECK(maniOpenLearnedModel(s_model[1], s_modelSize[1], &model).result == mani_success &&
              model.weightFormat == maniLearnedWeightsInt16,
          "its 16-bit form");
    maniDestroyLearnedTrainer(t);
    maniDestroyLearnedTrainer(NULL);
}

// A kept network's forward pass from raw inputs to raw outputs, as the
// trainer computes it.
static void Forward(const maniTrainKept* k, const float* in, float* out)
{
    static float values[256];
    uint32_t n = k->net.sizes[0];
    for (uint32_t i = 0; i < n; ++i)
    {
        values[i] = (in[i] - k->inputOffsets[i]) / k->inputScales[i];
    }
    maniTrainForward(&k->net, k->parameters, values, 1);
    const float* y = values + maniTrainLayerAt(&k->net, k->net.layerCount, 1);
    for (uint32_t o = 0; o < k->net.sizes[k->net.layerCount]; ++o)
    {
        out[o] = y[o] * k->outputScales[o] + k->outputOffsets[o];
    }
}

static void TestModelBits(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success && report.stage == 4,
          "trained");
    Build(t, 0, maniLearnedWeightsFloat32);
    maniLearnedModel model;
    CHECK(maniOpenLearnedModel(s_model[0], s_modelSize[0], &model).result == mani_success,
          "the model");
    static float scratch[1024];
    bool step = true;
    bool project = true;
    for (uint32_t f = 0; f < t->data.frameCount; f += 3)
    {
        float state[9];
        float want[9];
        float rate[9];
        memcpy(state, t->data.features + f * 5, 5 * sizeof(float));
        memcpy(state + 5, t->latents + f * 4, 4 * sizeof(float));
        Forward(&t->networks[1], state, rate);
        for (uint32_t j = 0; j < 9; ++j)
        {
            want[j] = state[j] + t->data.frameSeconds * rate[j];
        }
        step = step &&
               maniLearnedModel_Step(&model, state, scratch, sizeof scratch) == mani_success &&
               memcmp(state, want, sizeof want) == 0;
        for (uint32_t p = 0; p < 2; ++p)
        {
            float out[9];
            Forward(&t->networks[2 + p], t->data.features + f * 5, want);
            project = project &&
                      maniLearnedModel_Project(&model, p, t->data.features + f * 5, out,
                                               (model.featureCount + model.latentCount), NULL,
                                               scratch, sizeof scratch) == mani_success &&
                      memcmp(out, want, sizeof out) == 0;
        }
    }
    CHECK(step, "the model's Stepper gives the trainer's bits");
    CHECK(project, "its Projectors give the trainer's bits");
    // The Decompressor: 27 bone values, the root's six, two events.
    static _Alignas(32) uint8_t memory[4096];
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, memory, sizeof memory, &pose) == mani_success, "a pose");
    bool decompress = true;
    for (uint32_t f = 0; f < t->data.frameCount; f += 4)
    {
        float state[9];
        float want[35];
        float events[2];
        memcpy(state, t->data.features + f * 5, 5 * sizeof(float));
        memcpy(state + 5, t->latents + f * 4, 4 * sizeof(float));
        Forward(&t->networks[0], state, want);
        maniLearnedOutput output = {
            pose, NULL, 0, events, 2, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
        decompress = decompress && maniLearnedModel_Decompress(&model, state, &output, scratch,
                                                               sizeof scratch) == mani_success;
        for (uint32_t b = 0; b < 3; ++b)
        {
            maniVec3 x = maniPose_GetLocal(pose, b).translation;
            decompress = decompress && x.x == want[9 * b] && x.y == want[9 * b + 1] &&
                         x.z == want[9 * b + 2];
        }
        decompress = decompress && output.rootVelocity.x == want[27] &&
                     output.rootVelocity.z == want[29] &&
                     output.rootAngularVelocity.y == want[31] && events[0] == want[33] &&
                     events[1] == want[34];
    }
    CHECK(decompress, "its Decompressor gives the trainer's bits");
    maniDestroyLearnedTrainer(t);
}

// The first stage's 2,001 iterations, through two decays, as the stage's
// own steps at the rate times 0.99 to the whole thousands before each.
static void TestRate(void)
{
    maniLearnedTrainerSource s = Source();
    s.options.iterations = 5000;
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 2001, &report) == mani_success && report.stage == 0 &&
              report.iteration == 2001,
          "2,001 iterations");
    maniTrainRandom r;
    maniTrainSeed(&r, 7);
    const maniTrainDecompressShape shape = {4, 8, 1, 1, 4, 0};
    maniTrainDecompress own;
    CHECK(maniTrainDecompressInit(&kAllocator, &t->data, &shape, &r, &own), "the stage alone");
    for (uint64_t step = 1; step <= 2001; ++step)
    {
        float rate = (float)((double)0.003f * pow(0.99, (double)((step - 1) / 1000)));
        (void)maniTrainDecompressStep(&own, &t->data, &r, NULL, step, rate);
    }
    CHECK(own.count == t->decompress.count &&
              memcmp(own.parameters, t->decompress.parameters, sizeof(float) * own.count) == 0,
          "the learning rate decayed every 1,000 iterations");
    maniTrainDecompressRelease(&kAllocator, &own);
    maniDestroyLearnedTrainer(t);
}

// A trained stage's first iteration as the stage's own: the Stepper's
// generator seeded by the seed plus 1, the second Projector's by the
// seed plus 3, with its filter.
static void TestStageSeeds(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 31, &report) == mani_success && report.stage == 1 &&
              report.iteration == 1,
          "the Stepper's first iteration");
    maniTrainRandom r;
    maniTrainSeed(&r, 7);
    const maniTrainDecompressShape first = {4, 8, 1, 1, 4, 0};
    maniTrainDecompress own;
    CHECK(maniTrainDecompressInit(&kAllocator, &t->data, &first, &r, &own), "the stage alone");
    for (uint64_t step = 1; step <= 30; ++step)
    {
        (void)maniTrainDecompressStep(&own, &t->data, &r, NULL, step, 0.003f);
    }
    const maniTrainKept* k = &t->networks[0];
    uint32_t out = k->net.sizes[k->net.layerCount];
    bool kept = k->net.parameterCount == own.decompressor.parameterCount && out == 35 &&
                memcmp(k->parameters, own.parameters + own.compressor.parameterCount,
                       sizeof(float) * k->net.parameterCount) == 0 &&
                memcmp(k->outputOffsets, own.outputOffsets, sizeof(float) * out) == 0 &&
                memcmp(k->outputScales, own.outputScales, sizeof(float) * out) == 0;
    for (uint32_t i = 0; i < k->net.sizes[0]; ++i)
    {
        kept = kept && k->inputOffsets[i] == 0.0f && k->inputScales[i] == 1.0f;
    }
    CHECK(kept, "the Decompressor kept: its parameters, its inputs as they are");
    maniTrainDecompressRelease(&kAllocator, &own);
    maniTrainSeed(&r, 8);
    maniTrainSoloShape shape = {4, 8, 1, 4, 4, 0};
    maniTrainStepper stepper;
    CHECK(maniTrainStepperInit(&kAllocator, &t->data, t->latents, &shape, &r, &stepper),
          "the Stepper alone");
    (void)maniTrainStepperStep(&stepper, &r, NULL, 1, 0.003f);
    uint64_t count = stepper.solo.net.parameterCount;
    CHECK(memcmp(stepper.solo.parameters, t->stepper.solo.parameters, sizeof(float) * count) == 0,
          "the Stepper's generator: the seed plus 1");
    maniTrainStepperRelease(&kAllocator, &stepper);
    CHECK(maniLearnedTrainer_Train(t, 60, &report) == mani_success && report.stage == 3 &&
              report.iteration == 1,
          "the second Projector's first iteration");
    maniTrainSeed(&r, 10);
    shape.window = 0;
    maniTrainProjector projector;
    CHECK(maniTrainProjectorInit(&kAllocator, &s_bones, &t->data, t->latents, &shape, 2, 0, &r,
                                 &projector),
          "the Projector alone");
    (void)maniTrainProjectorStep(&projector, &r, NULL, 1, 0.003f);
    count = projector.solo.net.parameterCount;
    CHECK(memcmp(projector.solo.parameters, t->projector.solo.parameters, sizeof(float) * count) ==
              0,
          "the second Projector's generator and filter");
    maniTrainProjectorRelease(&kAllocator, &projector);
    maniDestroyLearnedTrainer(t);
}

static void TestHosts(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success, "trained alone");
    Build(t, 0, maniLearnedWeightsFloat32);
    maniDestroyLearnedTrainer(t);
    def.tasks = (maniTaskHooks){Backwards, Finish, NULL};
    t = Create(&def, &s);
    CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success, "trained on a host");
    Build(t, 1, maniLearnedWeightsFloat32);
    maniDestroyLearnedTrainer(t);
    CHECK(s_modelSize[0] == s_modelSize[1] && memcmp(s_model[0], s_model[1], s_modelSize[0]) == 0,
          "the same model on any host");
    def.tasks = (maniTaskHooks){Hostile, Finish, NULL};
    t = Create(&def, &s);
    uint64_t own = maniFloatControlRead();
    maniFloatControlWrite(s_hostSetting);
    s_hooksElsewhere = 0;
    maniResult trained = maniLearnedTrainer_Train(t, 1000, &report);
    maniFloatControlWrite(own);
    CHECK(trained == mani_success, "trained on a hostile host");
    CHECK(s_hooksElsewhere == 0, "the task hooks run in the host's setting");
    Build(t, 1, maniLearnedWeightsFloat32);
    maniDestroyLearnedTrainer(t);
    CHECK(s_modelSize[0] == s_modelSize[1] && memcmp(s_model[0], s_model[1], s_modelSize[0]) == 0,
          "the same model on a host whose workers flush denormals and round toward zero");
}

static _Alignas(16) uint8_t s_checkpoint[1 << 17];

// Trains to stop, checkpoints, restores into a new trainer and trains it
// through; its model into s_model[1].
static bool Resumed(uint64_t stop, size_t* sizeOut)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* a = Create(&def, &s);
    maniLearnedTrainerReport report;
    bool ok = maniLearnedTrainer_Train(a, stop, &report) == mani_success;
    float loss = report.loss;
    size_t size = 0;
    size_t refused = 0;
    size_t written = 0;
    ok = ok && maniLearnedTrainer_SaveCheckpoint(a, NULL, 0, &size) == mani_errorCapacity &&
         size <= sizeof s_checkpoint &&
         maniLearnedTrainer_SaveCheckpoint(a, s_checkpoint, size - 1, &refused) ==
             mani_errorCapacity &&
         refused == size &&
         maniLearnedTrainer_SaveCheckpoint(a, s_checkpoint, size, &written) == mani_success &&
         written == size;
    maniDestroyLearnedTrainer(a);
    maniLearnedTrainer* b = Create(&def, &s);
    ok = ok && maniLearnedTrainer_Train(b, 3, &report) == mani_success;
    ok = ok && maniLearnedTrainer_RestoreCheckpoint(b, s_checkpoint, size).result == mani_success &&
         maniLearnedTrainer_Train(b, 0, &report) == mani_success && report.loss == loss;
    ok = ok && maniLearnedTrainer_Train(b, 1000, &report) == mani_success && report.stage == 4;
    Build(b, 1, maniLearnedWeightsFloat32);
    maniDestroyLearnedTrainer(b);
    *sizeOut = size;
    return ok && s_modelSize[0] == s_modelSize[1] &&
           memcmp(s_model[0], s_model[1], s_modelSize[0]) == 0;
}

static void TestCheckpoints(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success, "trained through");
    Build(t, 0, maniLearnedWeightsFloat32);
    maniDestroyLearnedTrainer(t);
    // Not begun, in the first stage, at the stages' ends, in the Stepper,
    // in each Projector, done.
    const uint64_t stops[8] = {0, 12, 30, 47, 60, 75, 99, 120};
    size_t sizes[8];
    bool all = true;
    for (uint32_t k = 0; k < 8; ++k)
    {
        all = all && Resumed(stops[k], &sizes[k]);
    }
    CHECK(all, "a checkpoint at any point resumes to the same model");
    CHECK(sizes[1] > sizes[0] && sizes[2] < sizes[1] && sizes[7] < sizes[6],
          "a stage's moments only while it trains");
    CHECK(maniLearnedTrainer_SaveCheckpoint(NULL, NULL, 0, NULL) == mani_errorInvalid &&
              maniLearnedTrainer_SaveCheckpoint(NULL, s_checkpoint, sizeof s_checkpoint, NULL) ==
                  mani_errorInvalid,
          "NULL arguments");
}

// The checkpoint of a trainer of s after stop iterations, its size.
static size_t Checkpoint(const maniLearnedTrainerSource* s, uint64_t stop)
{
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, stop, &report) == mani_success, "trained");
    size_t size = CheckpointSize(t);
    CHECK(maniLearnedTrainer_SaveCheckpoint(t, s_checkpoint, sizeof s_checkpoint, NULL) ==
              mani_success,
          "saved");
    maniDestroyLearnedTrainer(t);
    return size;
}

// The payload hash written again after a crafted change.
static void WriteHash(size_t size)
{
    maniStoreU64(s_checkpoint + 32,
                 maniHashBytes(MANI_HASH_INIT, s_checkpoint + MANI_CONTAINER_HEADER_SIZE,
                               (uint64_t)size - MANI_CONTAINER_HEADER_SIZE));
}

// A section's offset by tag.
static uint64_t SectionAt(uint32_t tag)
{
    uint32_t count = maniLoadU32(s_checkpoint + 12);
    for (uint32_t k = 0; k < count; ++k)
    {
        const uint8_t* entry = s_checkpoint + MANI_CONTAINER_HEADER_SIZE + 24 * (size_t)k;
        if (maniLoadU32(entry) == tag)
        {
            return maniLoadU64(entry + 8);
        }
    }
    return 0;
}

// The trainer a checkpoint is restored into by RestoreFile.
static maniLearnedTrainer* s_restoring;

static maniFileResult RestoreFile(const void* bytes, size_t size)
{
    return maniLearnedTrainer_RestoreCheckpoint(s_restoring, bytes, size);
}

static bool RefusedAt(maniLearnedTrainer* t, size_t size, uint64_t offset)
{
    maniFileResult r = maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size);
    return r.result == mani_errorFormat && r.offset == offset &&
           r.tag == TestTagAt(s_checkpoint, size, offset);
}

static void StoreF32(uint8_t* p, float x)
{
    uint32_t bits = 0;
    memcpy(&bits, &x, sizeof bits);
    maniStoreU32(p, bits);
}

static void TestRestoreRefused(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 40, &report) == mani_success && report.stage == 1, "trained");
    maniLearnedTrainerSource other = s;
    other.options.seed = 8;
    size_t size = Checkpoint(&other, 5);
    CHECK(RefusedAt(t, size, 40), "a checkpoint of another seed");
    other = s;
    other.events = NULL;
    other.eventCount = 0;
    size = Checkpoint(&other, 5);
    CHECK(RefusedAt(t, size, 40), "of other events");
    static const maniLearnedTagFilter kOther[2] = {{0, 0}, {1, 0}};
    other = s;
    other.projectors = kOther;
    size = Checkpoint(&other, 5);
    CHECK(RefusedAt(t, size, 40), "of other filters");
    size = Checkpoint(&s, 45);
    uint64_t progress = SectionAt(1);
    s_checkpoint[size - 1] ^= 1;
    CHECK(maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size).result == mani_errorFormat,
          "a damaged byte");
    s_checkpoint[size - 1] ^= 1;
    CHECK(maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size - 16).result ==
              mani_errorFormat,
          "a cut file");
    CHECK(maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint + 1, size - 1).result ==
                  mani_errorInvalid &&
              maniLearnedTrainer_RestoreCheckpoint(NULL, s_checkpoint, size).result ==
                  mani_errorInvalid &&
              maniLearnedTrainer_RestoreCheckpoint(t, NULL, size).result == mani_errorInvalid,
          "misaligned bytes and NULL arguments");
    s_restoring = t;
    CHECK(TestOtherVersionsRefused(s_checkpoint, size, RestoreFile),
          "another format version refused");
    // A checkpoint is state: before 1.0 a writer of any other major, minor
    // or patch version is another ABI version.
    bool releases = true;
    for (uint32_t k = 0; k < 3; ++k)
    {
        uint16_t kept = maniLoadU16(s_checkpoint + 16 + 2 * k);
        maniStoreU16(s_checkpoint + 16 + 2 * k, (uint16_t)(kept + 1));
        maniFileResult r = maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size);
        releases = releases && r.result == mani_errorVersion && r.offset == 16;
        maniStoreU16(s_checkpoint + 16 + 2 * k, kept);
    }
    CHECK(releases, "a checkpoint of another release refused");
    // Crafted: each field of the progress out of form, a moment negative,
    // a parameter not finite, an input scale of 0.
    const uint64_t fields[4] = {progress, progress + 40, progress + 44, progress + 48};
    const uint32_t values[4] = {31, 5, 2, 0x7F800000u};
    bool all = true;
    for (uint32_t k = 0; k < 4; ++k)
    {
        uint32_t kept = maniLoadU32(s_checkpoint + fields[k]);
        maniStoreU32(s_checkpoint + fields[k], values[k]);
        WriteHash(size);
        all = all && RefusedAt(t, size, fields[k]);
        maniStoreU32(s_checkpoint + fields[k], kept);
    }
    CHECK(all, "the progress out of form");
    maniStoreU32(s_checkpoint + progress + 40, 4);
    WriteHash(size);
    CHECK(RefusedAt(t, size, progress + 44), "a stage begun past the last");
    maniStoreU32(s_checkpoint + progress + 40, 1);
    uint64_t v = SectionAt(5) + 8;
    StoreF32(s_checkpoint + v, -1.0f);
    WriteHash(size);
    CHECK(RefusedAt(t, size, v), "a negative second moment");
    StoreF32(s_checkpoint + v, 0.0f);
    uint64_t p = SectionAt(3) + 4;
    StoreF32(s_checkpoint + p, NAN);
    WriteHash(size);
    CHECK(RefusedAt(t, size, p), "a parameter not finite");
    StoreF32(s_checkpoint + p, 0.0f);
    maniTrainNet net;
    CHECK(maniLearnedTrainerShape(t, 1, &net), "the Decompressor's shape");
    uint64_t scale = SectionAt(16) + 4 * (net.parameterCount + net.sizes[0]) + 8;
    StoreF32(s_checkpoint + scale, 0.0f);
    WriteHash(size);
    CHECK(RefusedAt(t, size, scale), "an input scale of 0");
    StoreF32(s_checkpoint + scale, 1.0f);
    WriteHash(size);
    CHECK(maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size).result == mani_success &&
              maniLearnedTrainer_Train(t, 0, &report) == mani_success && report.stage == 1 &&
              report.iteration == 15,
          "restored: the Stepper's 15th iteration");
    // A stage begun: its generator not all 0.
    for (uint32_t k = 0; k < 4; ++k)
    {
        maniStoreU64(s_checkpoint + progress + 8 + 8 * k, 0);
    }
    WriteHash(size);
    CHECK(RefusedAt(t, size, progress + 8), "a generator of 0 in a stage begun");
    // A stage not begun: its generator and loss 0.
    size = Checkpoint(&s, 0);
    progress = SectionAt(1);
    const uint64_t unset[2] = {progress + 8, progress + 48};
    for (uint32_t k = 0; k < 2; ++k)
    {
        maniStoreU32(s_checkpoint + unset[k], 1);
        WriteHash(size);
        CHECK(RefusedAt(t, size, unset[k]), "a generator or loss before the stage began");
        maniStoreU32(s_checkpoint + unset[k], 0);
    }
    maniDestroyLearnedTrainer(t);
}

// Checkpoints restored save back to the same bytes: before training, in
// a stage, and between stages.
static void TestCanonical(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    maniLearnedTrainer* t = Create(&def, &s);
    static uint8_t again[sizeof s_checkpoint];
    const uint64_t stops[4] = {0, 7, 30, 45};
    for (uint32_t k = 0; k < 4; ++k)
    {
        size_t size = Checkpoint(&s, stops[k]);
        CHECK(maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size).result == mani_success &&
                  CheckpointSize(t) == size &&
                  maniLearnedTrainer_SaveCheckpoint(t, again, sizeof again, NULL) == mani_success &&
                  memcmp(again, s_checkpoint, size) == 0,
              "saved back alike");
    }
    maniDestroyLearnedTrainer(t);
}

static void TestMemory(void)
{
    maniLearnedTrainerSource s = Source();
    maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
    Budget budget = {0, 0};
    def.allocator = (maniAllocator){Allocate, Free, &budget};
    maniLearnedTrainer* t = NULL;
    uint32_t refused = 0;
    for (budget.limit = 0; budget.limit < 200; ++budget.limit)
    {
        budget.count = 0;
        maniBuildResult r = maniCreateLearnedTrainer(&def, &s, &t);
        if (r.result == mani_success)
        {
            break;
        }
        refused += r.result == mani_errorCapacity && t == NULL ? 1u : 0u;
    }
    CHECK(t != NULL && refused == budget.limit, "creation refused until memory suffices");
    budget.limit = budget.count;
    maniLearnedTrainerReport report;
    CHECK(maniLearnedTrainer_Train(t, 1, &report) == mani_errorCapacity && report.stage == 0 &&
              report.iteration == 0,
          "no memory to begin a stage");
    budget.limit = UINT32_MAX;
    CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success && report.stage == 4,
          "then trained through");
    Build(t, 1, maniLearnedWeightsFloat32);
    size_t size = CheckpointSize(t);
    CHECK(maniLearnedTrainer_SaveCheckpoint(t, s_checkpoint, sizeof s_checkpoint, NULL) ==
              mani_success,
          "saved");
    budget.limit = budget.count;
    CHECK(maniLearnedTrainer_RestoreCheckpoint(t, s_checkpoint, size).result ==
                  mani_errorCapacity &&
              maniLearnedTrainer_Train(t, 0, &report) == mani_success && report.stage == 0,
          "no memory to restore: the trainer starts over");
    budget.limit = UINT32_MAX;
    maniDestroyLearnedTrainer(t);
}

static CheckpointHost s_host;

// The host's trainer after eight iterations: the third stage's second.
static int WriteSeed(const char* path)
{
    maniLearnedTrainer* trainer = MakeCheckpointHost(&s_host);
    maniLearnedTrainerReport report;
    size_t size = trainer != NULL ? CheckpointSize(trainer) : 0;
    uint8_t* bytes = NULL;
    bool ok = trainer != NULL && maniLearnedTrainer_Train(trainer, 8, &report) == mani_success &&
              report.stage == 2 && (size = CheckpointSize(trainer)) > 0 &&
              (bytes = malloc(size)) != NULL &&
              maniLearnedTrainer_SaveCheckpoint(trainer, bytes, size, NULL) == mani_success;
    FILE* file = ok ? fopen(path, "wb") : NULL;
    ok = file != NULL && fwrite(bytes, 1, size, file) == size;
    ok = file != NULL && fclose(file) == 0 && ok;
    free(bytes);
    maniDestroyLearnedTrainer(trainer);
    return ok ? 0 : 1;
}

// The same checkpoint and model whatever bytes pad the source's names.
static void TestPaddingIgnored(void)
{
    uint8_t* checkpoints[2] = {NULL, NULL};
    size_t checkpointSizes[2] = {0, 0};
    uint8_t* models[2] = {NULL, NULL};
    size_t modelSizes[2] = {0, 0};
    for (int k = 0; k < 2; ++k)
    {
        maniLearnedNameSource events[2];
        memcpy(events, kEvents, sizeof events);
        TEST_PAD(events, 2, kFieldsLearnedNameSource, kTestPadPatterns[k]);
        maniLearnedTrainerSource s = Source();
        s.events = events;
        TEST_PAD(&s, 1, kFieldsLearnedTrainerSource, kTestPadPatterns[k]);
        maniLearnedTrainerDef def = maniDefaultLearnedTrainerDef();
        def.allocator = TestPatternAllocator(kTestPadPatterns[k]);
        maniLearnedTrainer* t = Create(&def, &s);
        maniLearnedTrainerReport report;
        CHECK(maniLearnedTrainer_Train(t, 1000, &report) == mani_success && report.stage == 4,
              "trained padded");
        checkpointSizes[k] = CheckpointSize(t);
        checkpoints[k] = (uint8_t*)malloc(checkpointSizes[k]);
        if (checkpoints[k] != NULL)
        {
            memset(checkpoints[k], kTestPadPatterns[k], checkpointSizes[k]);
        }
        if (checkpoints[k] != NULL &&
            maniLearnedTrainer_SaveCheckpoint(t, checkpoints[k], checkpointSizes[k], NULL) !=
                mani_success)
        {
            free(checkpoints[k]);
            checkpoints[k] = NULL;
        }
        maniBuilder* builder = TestPatternBuilder(kTestPadPatterns[k]);
        if (maniBuildLearnedModelFromTrainer(builder, t, maniLearnedWeightsFloat32).result ==
            mani_success)
        {
            models[k] = TestCopyBuilt(builder, &modelSizes[k]);
        }
        maniDestroyBuilder(builder);
        maniDestroyLearnedTrainer(t);
    }
    CHECK(TestSameBuilt(checkpoints, checkpointSizes), "the same checkpoint under either padding");
    CHECK(TestSameBuilt(models, modelSizes), "the same model under either padding");
}

int main(int argc, char** argv)
{
    if (argc == 3 && strcmp(argv[1], "--seed") == 0)
    {
        return WriteSeed(argv[2]);
    }
    Open();
    OpenBones();
    TestDef();
    TestOptions();
    TestSource();
    TestStages();
    TestModelBits();
    TestRate();
    TestStageSeeds();
    TestHosts();
    TestCheckpoints();
    TestRestoreRefused();
    TestCanonical();
    TestMemory();
    TestPaddingIgnored();
    return s_failures == 0 ? 0 : 1;
}
