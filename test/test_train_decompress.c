// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's first training stage (mani-0025): the training data
// of a database of the walk and the run (counts, layout, a frame's pose
// and root velocity as the clip gives them, the events' states, the
// pairs); the Compressor and Decompressor trained, the loss falling; the
// same bits whatever order a host runs the chunks in; a chunk's gradient
// against central differences of its loss; every frame's latent values.

#include "graph_match_fixture.h"
#include "train_data.h"
#include "train_decompress.h"
#include "train_decompress_loss.h"
#include "train_net.h"
#include "train_random.h"

#include "maul-anim/learnedbuild.h"

#include <math.h>
#include <string.h>

static _Alignas(16) uint8_t s_boneFile[1 << 15];
static maniMotionDatabase s_bones;
static const maniLearnedNameSource kEvents[2] = {{"left", 4, 0}, {"right", 5, 0}};
static const maniAllocator kAllocator = {NULL, NULL, NULL};

// The walk (looping, 10 frames) and the run (clamped, 11): the arm's
// position now and the root half a second ahead.
static void OpenBones(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 2, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    BuildDatabase(channels, 2, s_boneFile, sizeof s_boneFile, &s_bones);
}

static bool Data(maniTrainData* d)
{
    static const maniClip* clips[2];
    clips[0] = &s_clips[0];
    clips[1] = &s_clips[1];
    const maniTrainDataSource source = {&s_skeleton, &s_bones, clips, NULL, kEvents, 0, 2, 0.1f, 0};
    return maniTrainDataBuild(&kAllocator, &source, d);
}

static void TestData(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    CHECK(d.frameCount == 21 && d.featureCount == 5 && d.boneCount == 3 && d.scaledBoneCount == 0 &&
              d.outputCount == 27 + 6 + 2 &&
              d.inputCount == 3 * (3 + 6 + 3 + 3 + 3 + 6 + 3 + 6) + 6 + 2 && d.pairCount == 19 &&
              d.frameSeconds == 0.1f && d.groupStarts[maniTrainGroupScale] == 99 &&
              d.groupStarts[maniTrainGroupExtra] == 105,
          "the counts and layout");
    // Frame 3: the walk at 0.3 s.
    static _Alignas(32) uint8_t memory[4096];
    maniPose* pose = NULL;
    CHECK(maniInitPose(&s_skeleton, memory, sizeof memory, &pose) == mani_success &&
              maniSampleClip(&s_clips[0], &s_skeleton, 0.3f, pose) == mani_success,
          "the walk at 0.3 s");
    bool same = true;
    for (uint32_t b = 0; b < 3; ++b)
    {
        maniTransform t = maniPoseLocal(pose, b);
        const float* o = d.outputs + 3 * (size_t)d.outputCount + 9 * b;
        maniVec3 x = maniQuatRotate(t.rotation, maniVec3Make(1, 0, 0));
        maniVec3 y = maniQuatRotate(t.rotation, maniVec3Make(0, 1, 0));
        same = same && o[0] == t.translation.x && o[1] == t.translation.y &&
               o[2] == t.translation.z && o[3] == x.x && o[4] == x.y && o[5] == x.z &&
               o[6] == y.x && o[7] == y.y && o[8] == y.z;
    }
    CHECK(same, "a frame's local pose as the clip's");
    const float* root = d.outputs + 3 * (size_t)d.outputCount + 27;
    CHECK(fabsf(root[0] - 1.0f) <= 1e-4f && fabsf(root[1]) <= 1e-6f && fabsf(root[2]) <= 1e-6f &&
              fabsf(root[3]) <= 1e-6f && fabsf(root[4]) <= 1e-6f && fabsf(root[5]) <= 1e-6f,
          "the walk's root at 1 m/s, not turning");
    float features[5];
    CHECK(maniMotionDatabaseFeatures(&s_bones, 3, features) == mani_success &&
              memcmp(features, d.features + 15, sizeof features) == 0,
          "the database's features");
    // The walk's left at 0 s and right at 0.5 s; nothing in the run.
    bool events = true;
    for (uint32_t f = 0; f < 21; ++f)
    {
        const float* e = d.outputs + (size_t)f * d.outputCount + 33;
        events = events && e[0] == (f == 0 ? 1.0f : 0.0f) && e[1] == (f == 5 ? 1.0f : 0.0f);
        events = events && memcmp(e, d.inputs + (size_t)f * d.inputCount + 105, 8) == 0;
    }
    CHECK(events, "the events' states, for 0.1 s after each");
    CHECK(d.pairs[0] == 0 && d.pairs[8] == 8 && d.pairs[9] == 10 && d.pairs[18] == 19,
          "pairs within each entry");
    maniTrainDataRelease(&kAllocator, &d);
}

static const maniTrainDecompressShape kShape = {4, 16, 1, 1, 4, 0};

// A host that runs ranges of one item, last first.
static void* Backwards(maniTaskFn* task, int32_t count, int32_t minRange, void* context, void* user)
{
    (void)minRange;
    (void)user;
    for (int32_t k = count; k-- > 0;)
    {
        task(k, k + 1, context);
    }
    return NULL;
}

static void TestTrain(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 1234);
    maniTrainDecompress t;
    CHECK(maniTrainDecompressInit(&kAllocator, &d, &kShape, &r, &t), "the stage");
    double first = 0.0;
    double last = 0.0;
    for (uint64_t step = 1; step <= 600; ++step)
    {
        float loss = maniTrainDecompressStep(&t, &d, &r, NULL, step, 0.002f);
        first += step <= 30 ? (double)loss : 0.0;
        last += step > 570 ? (double)loss : 0.0;
    }
    CHECK(last < 0.3 * first, "the loss falls to under 0.3 of its start");
    static float latents[21 * 4];
    maniTrainLatents(&t, &d, latents);
    bool finite = true;
    for (uint32_t k = 0; k < 21 * 4; ++k)
    {
        finite = finite && fabsf(latents[k]) < 1e30f;
    }
    CHECK(finite, "every frame's latent values");
    maniTrainDecompressRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

static void TestHosts(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainDecompress t[2];
    const maniTaskHooks hooks[2] = {{NULL, NULL, NULL}, {Backwards, NULL, NULL}};
    for (uint32_t h = 0; h < 2; ++h)
    {
        maniTrainRandom r;
        maniTrainSeed(&r, 77);
        CHECK(maniTrainDecompressInit(&kAllocator, &d, &kShape, &r, &t[h]), "the stage");
        for (uint64_t step = 1; step <= 20; ++step)
        {
            (void)maniTrainDecompressStep(&t[h], &d, &r, &hooks[h], step, 0.001f);
        }
    }
    CHECK(memcmp(t[0].parameters, t[1].parameters, sizeof(float) * t[0].count) == 0 &&
              memcmp(t[0].m, t[1].m, sizeof(float) * t[0].count) == 0,
          "the same bits on any host");
    maniTrainDecompressRelease(&kAllocator, &t[0]);
    maniTrainDecompressRelease(&kAllocator, &t[1]);
    maniTrainDataRelease(&kAllocator, &d);
}

// The loss of chunk 0 (pairs 0 and 12) at the stage's parameters.
static float ChunkLoss(maniTrainDecompress* t)
{
    uint64_t own = t->compressor.parameterCount;
    maniTrainTranspose(&t->compressor, t->parameters, t->transposed);
    maniTrainTranspose(&t->decompressor, t->parameters + own, t->transposed + own);
    t->chunks[0].pairs[0] = 0;
    t->chunks[0].pairs[1] = 12;
    maniTrainDecompressRun(t, &t->chunks[0], MANI_TRAIN_PAIRS);
    return t->chunks[0].loss;
}

static void TestGradient(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 5);
    maniTrainDecompress t;
    CHECK(maniTrainDecompressInit(&kAllocator, &d, &kShape, &r, &t), "the stage");
    (void)ChunkLoss(&t);
    static float gradient[4096];
    CHECK(t.count <= 4096, "room");
    memcpy(gradient, t.chunks[0].gradients, sizeof(float) * t.count);
    uint32_t checked = 0;
    uint32_t near = 0;
    for (uint64_t k = 0; k < t.count; k += 7)
    {
        if (fabsf(gradient[k]) < 0.05f)
        {
            continue;
        }
        float kept = t.parameters[k];
        float h = 2e-3f;
        t.parameters[k] = kept + h;
        double up = (double)ChunkLoss(&t);
        t.parameters[k] = kept - h;
        double down = (double)ChunkLoss(&t);
        t.parameters[k] = kept;
        double want = (up - down) / (2.0 * (double)h);
        checked += 1;
        near += fabs((double)gradient[k] - want) <= 0.02 + 0.05 * fabs(want) ? 1u : 0u;
    }
    CHECK(checked >= 20 && near == checked,
          "a chunk's gradient as central differences of its loss");
    maniTrainDecompressRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

int main(void)
{
    Open();
    OpenDatabase();
    OpenBones();
    TestData();
    TestTrain();
    TestHosts();
    TestGradient();
    return s_failures == 0 ? 0 : 1;
}
