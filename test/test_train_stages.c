// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A learned model's Stepper and Projector stages (mani-0025), over a
// database of the walk (tag 1) and the run (tag 2) and made-up latent
// values (more than the features): the Stepper's windows (none across
// entries), normalization, a chunk's loss against Holden's terms
// computed apart in binary64 (its own forward pass and roll-out), its
// gradient against central differences of that, the windows drawn, a
// step's AdamW update by the chunks' gradients summed, the loss falling
// and the same bits on any host; the Projector's normalization and
// noise, its queries drawn as Holden's, its targets the nearest frames
// passing its tags, its loss and gradient as the Stepper's, a finite
// gradient where the output is the query, the loss falling, the same
// bits on any host, and tags no frame a jump reaches passes refused.

#include "graph_match_fixture.h"
#include "train_adam.h"
#include "train_data.h"
#include "train_net.h"
#include "train_projector.h"
#include "train_random.h"
#include "train_solo.h"
#include "train_stepper.h"
#include "train_tasks.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static _Alignas(16) uint8_t s_boneFile[1 << 15];
static maniMotionDatabase s_bones;
static const maniAllocator kAllocator = {NULL, NULL, NULL};
// Latent values a frame (more than the features, five), and a state's
// values.
#define LATENT 6u
#define STATE  (5u + LATENT)

static float s_latents[21 * LATENT];

// The walk (looping, 10 frames, tag 1) and the run (clamped, 11, tag
// 2): the arm's position now and the root half a second ahead.
static void OpenBones(void)
{
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 2, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    BuildDatabase(channels, 2, s_boneFile, sizeof s_boneFile, &s_bones);
    for (uint32_t f = 0; f < 21; ++f)
    {
        for (uint32_t k = 0; k < LATENT; ++k)
        {
            s_latents[f * LATENT + k] = 4.0f * (float)(k + 1) * sinf(0.7f * (float)f + (float)k);
        }
    }
}

static bool Data(maniTrainData* d)
{
    static const maniClip* clips[2];
    clips[0] = &s_clips[0];
    clips[1] = &s_clips[1];
    const maniTrainDataSource source = {&s_skeleton, &s_bones, clips, NULL, NULL, 0, 0, 0.1f, 0};
    return maniTrainDataBuild(&kAllocator, &source, d);
}

// Latent LATENT, hidden 12 by 2, batch 8 (two chunks), window 4.
static const maniTrainSoloShape kShape = {LATENT, 12, 2, 8, 4, 0};

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

// A network's forward pass in binary64 from its parameters (weights input
// by input, then biases, layer by layer).
static void Dense(const maniTrainNet* n, const float* p, const double* x, double* y)
{
    double a[64];
    double b[64];
    memcpy(a, x, sizeof(double) * n->sizes[0]);
    for (uint32_t l = 0; l < n->layerCount; ++l)
    {
        uint32_t in = n->sizes[l];
        uint32_t out = n->sizes[l + 1];
        const float* w = p + n->offsets[l];
        for (uint32_t o = 0; o < out; ++o)
        {
            b[o] = (double)w[(size_t)in * out + o];
            for (uint32_t i = 0; i < in; ++i)
            {
                b[o] += a[i] * (double)w[(size_t)i * out + o];
            }
            b[o] = l + 1 < n->layerCount && b[o] < 0.0 ? 0.0 : b[o];
        }
        memcpy(a, b, sizeof(double) * out);
    }
    memcpy(y, a, sizeof(double) * n->sizes[n->layerCount]);
}

// Per value of rows (count by width) from first: the mean and the
// population deviation in binary64; with group, the deviation over all.
static void Moments(const float* rows, uint32_t width, uint32_t count, uint32_t first,
                    uint32_t values, bool group, double* means, double* deviations)
{
    double all = 0.0;
    for (uint32_t j = 0; j < values; ++j)
    {
        double sum = 0.0;
        for (uint32_t f = 0; f < count; ++f)
        {
            sum += (double)rows[(size_t)f * width + first + j];
        }
        means[j] = sum / count;
        all += sum;
    }
    double mean = all / ((double)count * values);
    double groupSquares = 0.0;
    for (uint32_t j = 0; j < values; ++j)
    {
        double squares = 0.0;
        for (uint32_t f = 0; f < count; ++f)
        {
            double x = (double)rows[(size_t)f * width + first + j];
            squares += (x - means[j]) * (x - means[j]);
            groupSquares += (x - mean) * (x - mean);
        }
        deviations[j] = sqrt(squares / count);
    }
    for (uint32_t j = 0; group && j < values; ++j)
    {
        double deviation = sqrt(groupSquares / ((double)count * values));
        deviations[j] = deviation > 0.0 ? deviation : 1.0;
    }
}

static bool Close(float value, double want)
{
    return fabs((double)value - want) <= 1e-5 * (1.0 + fabs(want));
}

// After one step from fresh moments: the parameters as AdamW moves the
// kept ones by the chunks' gradients summed in order, and the loss the
// chunks' sum.
static bool OneStep(const maniTrainSolo* s, const float* before, float loss, const float* losses)
{
    static float sum[4096];
    static float p[4096];
    static float m[4096];
    static float v[4096];
    static float largest[4096];
    uint64_t count = s->net.parameterCount;
    if (count > 4096)
    {
        return false;
    }
    maniTrainSum(s->gradients, s->chunkCount, count, sum);
    memcpy(p, before, sizeof(float) * count);
    memset(m, 0, sizeof(float) * count);
    memset(v, 0, sizeof(float) * count);
    memset(largest, 0, sizeof(float) * count);
    maniTrainAdam adam = maniTrainAdamDefaults();
    maniTrainAdamStep(&adam, 1, 0.001f, p, sum, m, v, largest, count);
    float want = 0.0f;
    for (uint32_t k = 0; k < s->chunkCount; ++k)
    {
        want += losses[k];
    }
    bool chunks = false;
    for (uint64_t k = 0; s->chunkCount == 2 && k < count; ++k)
    {
        chunks = chunks || sum[k] != s->gradients[k];
    }
    return chunks && loss == want && memcmp(p, s->parameters, sizeof(float) * count) == 0;
}

// --- The Stepper ---

static double StepperTruth(const maniTrainData* d, uint32_t frame, uint32_t j)
{
    return j < 5 ? (double)d->features[frame * 5 + j] : (double)s_latents[frame * LATENT + j - 5];
}

// Holden's terms for chunk 0's windows, rolled out in binary64.
static double StepperReference(const maniTrainStepper* t, const maniTrainData* d)
{
    const maniTrainSolo* s = &t->solo;
    const maniTrainStepperChunk* c = &t->chunks[0];
    double dt = (double)d->frameSeconds;
    double loss = 0.0;
    for (uint32_t w = 0; w < 4; ++w)
    {
        double state[4][STATE];
        for (uint32_t j = 0; j < STATE; ++j)
        {
            state[0][j] = StepperTruth(d, c->windows[w], j);
        }
        for (uint32_t k = 1; k < 4; ++k)
        {
            double x[STATE];
            double y[STATE];
            for (uint32_t j = 0; j < STATE; ++j)
            {
                x[j] = (state[k - 1][j] - (double)s->inputOffsets[j]) / (double)s->inputScales[j];
            }
            Dense(&s->net, s->parameters, x, y);
            for (uint32_t j = 0; j < STATE; ++j)
            {
                double rate = y[j] * (double)s->outputScales[j] + (double)s->outputOffsets[j];
                state[k][j] = state[k - 1][j] + dt * rate;
            }
        }
        for (uint32_t k = 0; k < 4; ++k)
        {
            for (uint32_t j = 0; j < STATE; ++j)
            {
                double truth = StepperTruth(d, c->windows[w] + k, j);
                double width = j < 5 ? 5.0 : LATENT;
                loss += (j < 5 ? 2.0 : 7.5) / (8 * 4 * width) * fabs(state[k][j] - truth);
                if (k == 0)
                {
                    continue;
                }
                double before = StepperTruth(d, c->windows[w] + k - 1, j);
                double rate = ((state[k][j] - state[k - 1][j]) - (truth - before)) / dt;
                loss += (j < 5 ? 0.2 : 0.5) / (8 * 3 * width) * fabs(rate);
            }
        }
    }
    return loss;
}

static void SetWindows(maniTrainStepper* t)
{
    const uint32_t kWindows[4] = {0, 3, 10, 17};
    memcpy(t->chunks[0].windows, kWindows, sizeof kWindows);
    maniTrainSoloPrepare(&t->solo);
    maniTrainStepperRun(t, &t->chunks[0]);
}

static bool StepperNormalized(const maniTrainStepper* t, const maniTrainData* d)
{
    const maniTrainSolo* s = &t->solo;
    double means[STATE];
    double deviations[STATE];
    Moments(d->features, 5, 21, 0, 5, true, means, deviations);
    Moments(s_latents, LATENT, 21, 0, LATENT, true, means + 5, deviations + 5);
    bool ok = true;
    for (uint32_t j = 0; j < STATE; ++j)
    {
        ok = ok && Close(s->inputOffsets[j], means[j]) && Close(s->inputScales[j], deviations[j]);
    }
    float rates[19 * STATE];
    for (uint32_t p = 0; p < 19; ++p)
    {
        for (uint32_t j = 0; j < STATE; ++j)
        {
            uint32_t a = d->pairs[p];
            rates[p * STATE + j] = (float)((StepperTruth(d, a + 1, j) - StepperTruth(d, a, j)) /
                                           (double)d->frameSeconds);
        }
    }
    Moments(rates, STATE, 19, 0, STATE, false, means, deviations);
    for (uint32_t j = 0; j < STATE; ++j)
    {
        ok = ok && Close(s->outputOffsets[j], means[j]) && Close(s->outputScales[j], deviations[j]);
    }
    return ok;
}

static void TestStepperWindows(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 3);
    maniTrainStepper t;
    CHECK(maniTrainStepperInit(&kAllocator, &d, s_latents, &kShape, &r, &t), "the stage");
    bool starts = t.windowCount == 15;
    for (uint32_t k = 0; starts && k < 15; ++k)
    {
        starts = t.windowStarts[k] == (k < 7 ? k : k + 3);
    }
    CHECK(starts, "the windows of 4 frames within one entry");
    CHECK(t.seconds == 0.1f && t.solo.chunkCount == 2 && t.solo.net.layerCount == 3 &&
              t.solo.net.sizes[0] == STATE && t.solo.net.sizes[1] == 12 &&
              t.solo.net.sizes[3] == STATE,
          "the network's shape");
    CHECK(StepperNormalized(&t, &d), "the normalization");
    maniTrainRandom copy = r;
    static float before[4096];
    memcpy(before, t.solo.parameters, sizeof(float) * t.solo.net.parameterCount);
    float loss = maniTrainStepperStep(&t, &r, NULL, 1, 0.001f);
    float losses[2] = {t.chunks[0].loss, t.chunks[1].loss};
    CHECK(OneStep(&t.solo, before, loss, losses), "a step: AdamW by the chunks' gradients summed");
    bool drawn = true;
    for (uint32_t k = 0; k < 2; ++k)
    {
        for (uint32_t s = 0; s < 4; ++s)
        {
            drawn = drawn && t.chunks[k].windows[s] == t.windowStarts[maniTrainBelow(&copy, 15)];
        }
    }
    CHECK(drawn, "the windows drawn chunk by chunk");
    maniTrainStepperRelease(&kAllocator, &t);
    maniTrainSoloShape shape = kShape;
    shape.window = 11;
    CHECK(maniTrainStepperInit(&kAllocator, &d, s_latents, &shape, &r, &t) && t.windowCount == 1 &&
              t.windowStarts[0] == 10,
          "the run's one window of 11");
    maniTrainStepperRelease(&kAllocator, &t);
    shape.window = 12;
    CHECK(!maniTrainStepperInit(&kAllocator, &d, s_latents, &shape, &r, &t),
          "no window of 12 in an entry");
    shape.window = 1;
    CHECK(!maniTrainStepperInit(&kAllocator, &d, s_latents, &shape, &r, &t), "a window of 1");
    shape = kShape;
    shape.batch = 6;
    CHECK(!maniTrainStepperInit(&kAllocator, &d, s_latents, &shape, &r, &t),
          "a batch of a chunk and a half");
    maniTrainDataRelease(&kAllocator, &d);
}

// gradient's values above 0.002 every third against central differences
// of reference.
static bool AsDifferences(float* parameters, const float* gradient, uint64_t count,
                          double (*reference)(const void*, const maniTrainData*), const void* stage,
                          const maniTrainData* d, uint32_t* checkedOut)
{
    uint32_t checked = 0;
    bool ok = true;
    for (uint64_t k = 0; k < count; k += 3)
    {
        if (fabsf(gradient[k]) < 0.002f)
        {
            continue;
        }
        float kept = parameters[k];
        float up = kept + 1e-5f;
        float down = kept - 1e-5f;
        parameters[k] = up;
        double high = reference(stage, d);
        parameters[k] = down;
        double low = reference(stage, d);
        parameters[k] = kept;
        double slope = (high - low) / ((double)up - (double)down);
        checked += 1;
        if (!(fabs((double)gradient[k] - slope) <= 0.02 + 0.05 * fabs(slope)))
        {
            printf("  parameter %u: gradient %.6g, differences %.6g\n", (unsigned)k,
                   (double)gradient[k], slope);
            ok = false;
        }
    }
    *checkedOut = checked;
    return ok;
}

static double StepperOf(const void* stage, const maniTrainData* d)
{
    return StepperReference(stage, d);
}

static void TestStepperChunk(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 5);
    maniTrainStepper t;
    CHECK(maniTrainStepperInit(&kAllocator, &d, s_latents, &kShape, &r, &t), "the stage");
    SetWindows(&t);
    double want = StepperReference(&t, &d);
    float loss = t.chunks[0].loss;
    if (!(fabs((double)loss - want) <= 1e-4 * want))
    {
        printf("  loss %.9g, Holden's terms %.9g\n", (double)loss, want);
    }
    CHECK(want > 0.0 && fabs((double)loss - want) <= 1e-4 * want,
          "a chunk's loss as Holden's terms");
    static float gradient[4096];
    uint64_t count = t.solo.net.parameterCount;
    CHECK(count <= 4096, "room");
    memcpy(gradient, t.solo.gradients, sizeof(float) * count);
    uint32_t checked = 0;
    CHECK(AsDifferences(t.solo.parameters, gradient, count, StepperOf, &t, &d, &checked) &&
              checked >= 20,
          "a chunk's gradient as central differences of Holden's terms");
    bool second = true;
    for (uint64_t k = 0; k < count; ++k)
    {
        second = second && t.solo.gradients[count + k] == 0.0f;
    }
    CHECK(second, "only the chunk's own gradients");
    maniTrainStepperRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

static void TestStepperTrain(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainStepper t[2];
    const maniTaskHooks hooks[2] = {{NULL, NULL, NULL}, {Backwards, NULL, NULL}};
    double first = 0.0;
    double last = 0.0;
    for (uint32_t h = 0; h < 2; ++h)
    {
        maniTrainRandom r;
        maniTrainSeed(&r, 1234);
        CHECK(maniTrainStepperInit(&kAllocator, &d, s_latents, &kShape, &r, &t[h]), "the stage");
        for (uint64_t step = 1; step <= 400; ++step)
        {
            float loss = maniTrainStepperStep(&t[h], &r, &hooks[h], step, 0.003f);
            first += h == 0 && step <= 20 ? (double)loss : 0.0;
            last += h == 0 && step > 380 ? (double)loss : 0.0;
        }
    }
    CHECK(last < 0.1 * first, "the loss falls to under a tenth of its start");
    uint64_t count = t[0].solo.net.parameterCount;
    CHECK(memcmp(t[0].solo.parameters, t[1].solo.parameters, sizeof(float) * count) == 0 &&
              memcmp(t[0].solo.largest, t[1].solo.largest, sizeof(float) * count) == 0,
          "the same bits on any host");
    maniTrainStepperRelease(&kAllocator, &t[0]);
    maniTrainStepperRelease(&kAllocator, &t[1]);
    maniTrainDataRelease(&kAllocator, &d);
}

// --- The Projector ---

// The nearest frame passing tag 2 (the run's frames 10 to 20), the
// lowest on a tie.
static uint32_t Nearest(const maniTrainData* d, const float* q)
{
    uint32_t best = 0;
    double least = HUGE_VAL;
    for (uint32_t f = 10; f < 21; ++f)
    {
        double cost = 0.0;
        for (uint32_t j = 0; j < 5; ++j)
        {
            double gap = (double)q[j] - (double)d->features[f * 5 + j];
            cost += gap * gap;
        }
        best = cost < least ? f : best;
        least = cost < least ? cost : least;
    }
    return best;
}

static double ProjectorReference(const maniTrainProjector* t, const maniTrainData* d)
{
    const maniTrainSolo* s = &t->solo;
    const maniTrainProjectorChunk* c = &t->chunks[0];
    double loss = 0.0;
    for (uint32_t k = 0; k < 4; ++k)
    {
        const float* q = c->queries + k * 5;
        uint32_t target = Nearest(d, q);
        double x[5];
        double y[STATE];
        for (uint32_t j = 0; j < 5; ++j)
        {
            x[j] = ((double)q[j] - (double)s->inputOffsets[j]) / (double)s->inputScales[j];
        }
        Dense(&s->net, s->parameters, x, y);
        double apart = 0.0;
        double wanted = 0.0;
        for (uint32_t j = 0; j < STATE; ++j)
        {
            y[j] = y[j] * (double)s->outputScales[j] + (double)s->outputOffsets[j];
            double truth = StepperTruth(d, target, j);
            loss += (j < 5 ? 1.0 / (8 * 5) : 5.0 / (8 * LATENT)) * fabs(y[j] - truth);
            apart += j < 5 ? (y[j] - (double)q[j]) * (y[j] - (double)q[j]) : 0.0;
            wanted += j < 5 ? (truth - (double)q[j]) * (truth - (double)q[j]) : 0.0;
        }
        loss += 0.3 / 8 * fabs(sqrt(apart) - sqrt(wanted));
    }
    return loss;
}

static double ProjectorOf(const void* stage, const maniTrainData* d)
{
    return ProjectorReference(stage, d);
}

static bool ProjectorNormalized(const maniTrainProjector* t, const maniTrainData* d)
{
    const maniTrainSolo* s = &t->solo;
    double means[STATE];
    double deviations[STATE];
    double group[5];
    double groupDeviation[5];
    Moments(d->features, 5, 21, 0, 5, false, means, deviations);
    Moments(s_latents, LATENT, 21, 0, LATENT, false, means + 5, deviations + 5);
    Moments(d->features, 5, 21, 0, 5, true, group, groupDeviation);
    bool ok = true;
    for (uint32_t j = 0; j < STATE; ++j)
    {
        ok = ok && Close(s->outputOffsets[j], means[j]) && Close(s->outputScales[j], deviations[j]);
    }
    for (uint32_t j = 0; j < 5; ++j)
    {
        ok = ok && Close(s->inputOffsets[j], group[j]) &&
             Close(s->inputScales[j], groupDeviation[j]) && Close(t->noise[j], deviations[j] + 1.0);
    }
    return ok;
}

static void TestProjectorQueries(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 9);
    maniTrainProjector t;
    CHECK(maniTrainProjectorInit(&kAllocator, &s_bones, &d, s_latents, &kShape, 2, 0, &r, &t),
          "the stage");
    CHECK(t.solo.net.sizes[0] == 5 && t.solo.net.sizes[3] == STATE && t.solo.chunkCount == 2,
          "the network's shape");
    CHECK(ProjectorNormalized(&t, &d), "the normalization and noise");
    maniTrainRandom copy = r;
    static float before[4096];
    memcpy(before, t.solo.parameters, sizeof(float) * t.solo.net.parameterCount);
    float loss = maniTrainProjectorStep(&t, &r, NULL, 1, 0.001f);
    float losses[2] = {t.chunks[0].loss, t.chunks[1].loss};
    CHECK(OneStep(&t.solo, before, loss, losses), "a step: AdamW by the chunks' gradients summed");
    bool drawn = true;
    bool targets = true;
    for (uint32_t k = 0; k < 2; ++k)
    {
        for (uint32_t s = 0; s < 4; ++s)
        {
            uint32_t frame = maniTrainBelow(&copy, 21);
            float spread = maniTrainUniform(&copy);
            const float* q = t.chunks[k].queries + s * 5;
            for (uint32_t j = 0; j < 5; ++j)
            {
                float g = maniTrainGaussian(&copy);
                drawn = drawn && q[j] == d.features[frame * 5 + j] + t.noise[j] * spread * g;
            }
            targets = targets && t.chunks[k].targets[s] == Nearest(&d, q);
        }
    }
    CHECK(drawn, "the queries drawn as Holden's");
    CHECK(targets, "the targets the nearest frames passing the tags");
    maniTrainProjectorRelease(&kAllocator, &t);
    CHECK(!maniTrainProjectorInit(&kAllocator, &s_bones, &d, s_latents, &kShape, 4, 0, &r, &t),
          "tags no entry has");
    CHECK(!maniTrainProjectorInit(&kAllocator, &s_bones, &d, s_latents, &kShape, 0, 3, &r, &t),
          "tags excluding every entry");
    maniTrainDataRelease(&kAllocator, &d);
}

static void TestProjectorChunk(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 5);
    maniTrainProjector t;
    CHECK(maniTrainProjectorInit(&kAllocator, &s_bones, &d, s_latents, &kShape, 2, 0, &r, &t),
          "the stage");
    // Queries near frames 1, 12, 20 and exactly frame 15.
    const uint32_t kFrames[4] = {1, 12, 20, 15};
    for (uint32_t s = 0; s < 4; ++s)
    {
        for (uint32_t j = 0; j < 5; ++j)
        {
            float shift = s == 3 ? 0.0f : 0.3f * sinf((float)(s * 5 + j));
            t.chunks[0].queries[s * 5 + j] = d.features[kFrames[s] * 5 + j] + shift;
        }
    }
    maniTrainSoloPrepare(&t.solo);
    maniTrainProjectorRun(&t, &t.chunks[0]);
    CHECK(t.chunks[0].targets[3] == 15, "a frame's own features find it");
    double want = ProjectorReference(&t, &d);
    float loss = t.chunks[0].loss;
    if (!(fabs((double)loss - want) <= 1e-4 * want))
    {
        printf("  loss %.9g, Holden's terms %.9g\n", (double)loss, want);
    }
    CHECK(want > 0.0 && fabs((double)loss - want) <= 1e-4 * want,
          "a chunk's loss as Holden's terms");
    static float gradient[4096];
    uint64_t count = t.solo.net.parameterCount;
    CHECK(count <= 4096, "room");
    memcpy(gradient, t.solo.gradients, sizeof(float) * count);
    uint32_t checked = 0;
    CHECK(AsDifferences(t.solo.parameters, gradient, count, ProjectorOf, &t, &d, &checked) &&
              checked >= 20,
          "a chunk's gradient as central differences of Holden's terms");
    maniTrainProjectorRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

// The bones database again with the run an entry no jump reaches.
static void TestProjectorNoJump(void)
{
    static _Alignas(16) uint8_t file[1 << 15];
    static const float kNow[1] = {0.0f};
    const maniMatchChannelSource channels[2] = {
        {maniMatchBonePosition, 2, 0, 1, kNow, NULL, 0, 1.0f},
        {maniMatchTrajectoryPosition, 0, 0, 1, kAhead, NULL, 0, 1.0f}};
    const maniClip* clips[2] = {&s_clips[0], &s_clips[1]};
    const maniMatchEntrySource entries[2] = {{0, MANI_MATCH_LOOP, 0.0f, 1.0f, 0.0f, 1u},
                                             {1, MANI_MATCH_NO_JUMP, 0.0f, 1.0f, 0.0f, 2u}};
    maniMotionDatabaseSource source = {&s_skeleton, clips, channels,       entries,       2, 2,
                                       2,           10.0f, maniMatchAxisY, maniMatchAxisZ};
    maniBuilder* builder = NewBuilder();
    size_t size = 0;
    maniMotionDatabase database;
    CHECK(maniBuildMotionDatabase(builder, &source).result == mani_success &&
              maniCopyBuilt(builder, file, sizeof file, &size) == mani_success &&
              maniOpenMotionDatabase(file, size, &database).result == mani_success,
          "a database of a run no jump reaches");
    maniDestroyBuilder(builder);
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 9);
    maniTrainProjector t;
    CHECK(!maniTrainProjectorInit(&kAllocator, &database, &d, s_latents, &kShape, 2, 0, &r, &t),
          "tags only an entry no jump reaches has");
    CHECK(maniTrainProjectorInit(&kAllocator, &database, &d, s_latents, &kShape, 1, 0, &r, &t),
          "the walk's tags");
    maniTrainProjectorRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

// A query the output equals: the distance 0, its derivative 0.
static void TestProjectorAtQuery(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainRandom r;
    maniTrainSeed(&r, 9);
    maniTrainProjector t;
    CHECK(maniTrainProjectorInit(&kAllocator, &s_bones, &d, s_latents, &kShape, 0, 0, &r, &t),
          "the stage");
    memset(t.solo.parameters, 0, sizeof(float) * t.solo.net.parameterCount);
    for (uint32_t s = 0; s < 4; ++s)
    {
        memcpy(t.chunks[0].queries + s * 5, t.solo.outputOffsets, 5 * sizeof(float));
    }
    maniTrainSoloPrepare(&t.solo);
    maniTrainProjectorRun(&t, &t.chunks[0]);
    bool finite = true;
    for (uint64_t k = 0; k < t.solo.net.parameterCount; ++k)
    {
        finite = finite && fabsf(t.solo.gradients[k]) < 1e30f;
    }
    CHECK(finite, "a finite gradient where the output is the query");
    maniTrainProjectorRelease(&kAllocator, &t);
    maniTrainDataRelease(&kAllocator, &d);
}

static void TestProjectorTrain(void)
{
    maniTrainData d;
    CHECK(Data(&d), "the data");
    maniTrainProjector t[2];
    const maniTaskHooks hooks[2] = {{NULL, NULL, NULL}, {Backwards, NULL, NULL}};
    double first = 0.0;
    double last = 0.0;
    for (uint32_t h = 0; h < 2; ++h)
    {
        maniTrainRandom r;
        maniTrainSeed(&r, 1234);
        CHECK(
            maniTrainProjectorInit(&kAllocator, &s_bones, &d, s_latents, &kShape, 0, 0, &r, &t[h]),
            "the stage");
        for (uint64_t step = 1; step <= 2000; ++step)
        {
            float loss = maniTrainProjectorStep(&t[h], &r, &hooks[h], step, 0.003f);
            first += h == 0 && step <= 20 ? (double)loss : 0.0;
            last += h == 0 && step > 1980 ? (double)loss : 0.0;
        }
    }
    CHECK(last < 0.4 * first, "the loss falls to under 0.4 of its start");
    uint64_t count = t[0].solo.net.parameterCount;
    CHECK(memcmp(t[0].solo.parameters, t[1].solo.parameters, sizeof(float) * count) == 0 &&
              memcmp(t[0].solo.v, t[1].solo.v, sizeof(float) * count) == 0,
          "the same bits on any host");
    maniTrainProjectorRelease(&kAllocator, &t[0]);
    maniTrainProjectorRelease(&kAllocator, &t[1]);
    maniTrainDataRelease(&kAllocator, &d);
}

int main(void)
{
    Open();
    OpenBones();
    TestStepperWindows();
    TestStepperChunk();
    TestStepperTrain();
    TestProjectorQueries();
    TestProjectorChunk();
    TestProjectorNoJump();
    TestProjectorAtQuery();
    TestProjectorTrain();
    return s_failures == 0 ? 0 : 1;
}
