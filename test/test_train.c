// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The trainer's core (mani-0025): the generator against its authors'
// reference outputs, Lemire's and Box and Muller's values, and its
// ranges; a network's start within PyTorch's bounds and its gradients
// over two samples (ReLU and ELU, by its parameters and its inputs)
// against central differences of a binary64 forward pass; a batch's gradient the same bits whatever
// order and grouping the host runs its chunks in; AdamW with amsgrad against the formula in
// binary64; a small network fit to a sine, the same bits on a second run.

#include "test_harness.h"
#include "train_adam.h"
#include "train_net.h"
#include "train_random.h"
#include "train_tasks.h"

#include "maul-anim/base.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static void TestRandom(void)
{
    // SplitMix64 from 0: its first output as its author gives it.
    maniTrainRandom r;
    maniTrainSeed(&r, 0);
    CHECK(r.s[0] == 0xE220A8397B1DCDAFull, "SplitMix64's first output");
    // xoshiro256** from the state 1, 2, 3, 4: its reference outputs.
    r = (maniTrainRandom){{1, 2, 3, 4}};
    const uint64_t kOutputs[5] = {11520, 0, 1509978240, 1215971899390074240ull,
                                  1216172134540287360ull};
    bool reference = true;
    for (uint32_t k = 0; k < 5; ++k)
    {
        reference = reference && maniTrainNext(&r) == kOutputs[k];
    }
    CHECK(reference, "xoshiro256**'s reference outputs");
    // Lemire's multiplication from the same state: the high words of
    // the top 32 bits times 7.
    r = (maniTrainRandom){{1, 2, 3, 4}};
    const uint32_t kBelow[5] = {0, 0, 0, 6, 3};
    bool below = true;
    for (uint32_t k = 0; k < 5; ++k)
    {
        below = below && maniTrainBelow(&r, 7) == kBelow[k];
    }
    CHECK(below, "values below 7 by Lemire's multiplication");
    // Box and Muller's from two uniform values, u = 1 - the first.
    bool gaussian = true;
    maniTrainSeed(&r, 99);
    for (int k = 0; k < 100; ++k)
    {
        maniTrainRandom copy = r;
        double u = 1.0 - (double)maniTrainUniform(&copy);
        double v = (double)maniTrainUniform(&copy);
        double want = sqrt(-2.0 * log(u)) * cos(6.283185307179586 * v);
        gaussian =
            gaussian && fabs((double)maniTrainGaussian(&r) - want) <= 1e-5 * (1.0 + fabs(want));
    }
    CHECK(gaussian, "Gaussian values as Box and Muller's");
    maniTrainRandom a;
    maniTrainRandom b;
    maniTrainSeed(&a, 1234);
    maniTrainSeed(&b, 1234);
    bool same = true;
    bool ranged = true;
    double sum = 0.0;
    double squares = 0.0;
    uint32_t counts[3] = {0, 0, 0};
    for (int k = 0; k < 100000; ++k)
    {
        same = same && maniTrainNext(&a) == maniTrainNext(&b);
        float u = maniTrainUniform(&a);
        ranged = ranged && u >= 0.0f && u < 1.0f;
        uint32_t three = maniTrainBelow(&a, 3);
        ranged = ranged && three < 3;
        counts[three < 3 ? three : 0] += 1;
        float g = maniTrainGaussian(&a);
        ranged = ranged && fabsf(g) < 10.0f;
        sum += (double)g;
        squares += (double)g * (double)g;
        maniTrainUniform(&b);
        maniTrainBelow(&b, 3);
        maniTrainGaussian(&b);
    }
    CHECK(same && ranged, "a seed's values, in their ranges");
    CHECK(fabs(sum / 100000.0) < 0.02 && fabs(squares / 100000.0 - 1.0) < 0.02,
          "Gaussian values of mean 0 and variance 1");
    CHECK(counts[0] > 32000 && counts[1] > 32000 && counts[2] > 32000, "three values alike");
}

// A binary64 forward pass of the same network, its loss sum(c * out).
static double Loss(const maniTrainNet* n, const double* parameters, const double* input,
                   const double* c)
{
    double a[16];
    double b[16];
    memcpy(a, input, sizeof(double) * n->sizes[0]);
    for (uint32_t l = 0; l < n->layerCount; ++l)
    {
        uint32_t in = n->sizes[l];
        uint32_t out = n->sizes[l + 1];
        const double* w = parameters + n->offsets[l];
        for (uint32_t o = 0; o < out; ++o)
        {
            b[o] = w[(size_t)in * out + o];
            for (uint32_t i = 0; i < in; ++i)
            {
                b[o] += a[i] * w[(size_t)i * out + o];
            }
        }
        bool last = l + 1 == n->layerCount;
        for (uint32_t o = 0; o < out; ++o)
        {
            double x = b[o];
            a[o] = last || x > 0.0 ? x : (n->activation == maniTrainElu ? exp(x) - 1.0 : 0.0);
        }
    }
    double loss = 0.0;
    for (uint32_t o = 0; o < n->sizes[n->layerCount]; ++o)
    {
        loss += c[o] * a[o];
    }
    return loss;
}

static float s_parameters[4096];
static float s_transposed[4096];
static float s_gradients[4096];
static float s_values[1024];
static float s_scratch[1024];

static bool GradientsMatch(maniTrainActivation activation)
{
    static const uint32_t kSizes[4] = {3, 5, 4, 2};
    maniTrainNet n;
    CHECK(maniTrainNetInit(&n, kSizes, 3, activation), "a network");
    maniTrainRandom r;
    maniTrainSeed(&r, 7);
    maniTrainNetStart(&n, s_parameters, &r);
    // Two samples: their losses summed, sum(c * out) each.
    const float input[6] = {0.7f, -0.3f, 0.45f, -0.2f, 0.9f, 0.1f};
    const float c[4] = {1.5f, -0.75f, -0.5f, 1.25f};
    memcpy(s_values, input, sizeof input);
    maniTrainForward(&n, s_parameters, s_values, 2);
    maniTrainTranspose(&n, s_parameters, s_transposed);
    memset(s_gradients, 0, sizeof s_gradients);
    float inputGradient[6];
    maniTrainBackward(&n, s_transposed, s_values, c, 2, s_gradients, inputGradient, s_scratch);
    static double p[4096];
    double x[6];
    double cd[4];
    for (uint64_t k = 0; k < n.parameterCount; ++k)
    {
        p[k] = (double)s_parameters[k];
    }
    for (uint32_t i = 0; i < 6; ++i)
    {
        x[i] = (double)input[i];
    }
    for (uint32_t i = 0; i < 4; ++i)
    {
        cd[i] = (double)c[i];
    }
    const double h = 1e-6;
    bool ok = true;
    for (uint64_t k = 0; k < n.parameterCount; ++k)
    {
        double kept = p[k];
        p[k] = kept + h;
        double up = Loss(&n, p, x, cd) + Loss(&n, p, x + 3, cd + 2);
        p[k] = kept - h;
        double down = Loss(&n, p, x, cd) + Loss(&n, p, x + 3, cd + 2);
        p[k] = kept;
        double want = (up - down) / (2.0 * h);
        ok = ok && fabs((double)s_gradients[k] - want) <= 1e-4 * (1.0 + fabs(want));
    }
    for (uint32_t i = 0; i < 6; ++i)
    {
        double kept = x[i];
        x[i] = kept + h;
        double up = Loss(&n, p, x + 3 * (i / 3), cd + 2 * (i / 3));
        x[i] = kept - h;
        double down = Loss(&n, p, x + 3 * (i / 3), cd + 2 * (i / 3));
        x[i] = kept;
        double want = (up - down) / (2.0 * h);
        ok = ok && fabs((double)inputGradient[i] - want) <= 1e-4 * (1.0 + fabs(want));
    }
    // PyTorch's start: every value within 1 / sqrt(its layer's inputs).
    for (uint32_t l = 0; l < n.layerCount; ++l)
    {
        float bound = 1.0f / sqrtf((float)n.sizes[l]);
        uint64_t count = (uint64_t)n.sizes[l] * n.sizes[l + 1] + n.sizes[l + 1];
        for (uint64_t k = 0; k < count; ++k)
        {
            ok = ok && fabsf(s_parameters[n.offsets[l] + k]) <= bound;
        }
    }
    return ok;
}

static void TestGradients(void)
{
    CHECK(GradientsMatch(maniTrainRelu), "ReLU gradients as central differences");
    CHECK(GradientsMatch(maniTrainElu), "ELU gradients as central differences");
    maniTrainNet n;
    const uint32_t zero[3] = {3, 0, 2};
    const uint32_t wide[3] = {3, MANI_LEARNED_MAX_WIDTH + 1, 2};
    const uint32_t fine[3] = {3, 4, 2};
    uint32_t many[MANI_LEARNED_MAX_LAYERS + 2];
    for (uint32_t k = 0; k < MANI_LEARNED_MAX_LAYERS + 2; ++k)
    {
        many[k] = 2;
    }
    CHECK(!maniTrainNetInit(&n, zero, 2, maniTrainRelu) &&
              !maniTrainNetInit(&n, wide, 2, maniTrainRelu) &&
              !maniTrainNetInit(&n, fine, 0, maniTrainRelu) &&
              !maniTrainNetInit(&n, many, MANI_LEARNED_MAX_LAYERS + 1, maniTrainRelu) &&
              maniTrainNetInit(&n, many, MANI_LEARNED_MAX_LAYERS, maniTrainRelu) &&
              maniTrainNetInit(&n, fine, 2, maniTrainRelu) &&
              n.parameterCount == 3 * 4 + 4 + 4 * 2 + 2 && n.valueCount == 9 && n.widest == 4,
          "shapes and limits");
}

// A batch of 12 samples, 3 chunks: each chunk's gradients into its own
// buffer, summed in chunk order.
#define BATCH 12u

typedef struct Batch
{
    const maniTrainNet* n;
    float inputs[BATCH * 3];
    float chunks[BATCH / MANI_TRAIN_CHUNK][4096];
    float values[BATCH / MANI_TRAIN_CHUNK][1024];
    float scratch[BATCH / MANI_TRAIN_CHUNK][1024];
    int32_t order[BATCH / MANI_TRAIN_CHUNK];
} Batch;

static void Chunk(int32_t start, int32_t end, void* context)
{
    Batch* b = context;
    for (int32_t c = start; c < end; ++c)
    {
        uint32_t first = (uint32_t)c * MANI_TRAIN_CHUNK;
        memcpy(b->values[c], b->inputs + 3 * first, sizeof(float) * 3 * MANI_TRAIN_CHUNK);
        maniTrainForward(b->n, s_parameters, b->values[c], MANI_TRAIN_CHUNK);
        const float* out =
            b->values[c] + maniTrainLayerAt(b->n, b->n->layerCount, MANI_TRAIN_CHUNK);
        float d[MANI_TRAIN_CHUNK * 2];
        for (uint32_t k = 0; k < MANI_TRAIN_CHUNK * 2; ++k)
        {
            d[k] = out[k] - 0.25f;
        }
        memset(b->chunks[c], 0, sizeof b->chunks[c]);
        maniTrainBackward(b->n, s_transposed, b->values[c], d, MANI_TRAIN_CHUNK, b->chunks[c], NULL,
                          b->scratch[c]);
    }
}

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

// A host that hands back a handle and runs everything at the finish.
typedef struct Deferred
{
    maniTaskFn* task;
    int32_t count;
    void* context;
    int finished;
} Deferred;

static void* Defer(maniTaskFn* task, int32_t count, int32_t minRange, void* context, void* user)
{
    (void)minRange;
    Deferred* d = user;
    *d = (Deferred){task, count, context, 0};
    return d;
}

static void Finish(void* handle, void* user)
{
    Deferred* d = handle;
    (void)user;
    d->task(1, d->count, d->context);
    d->task(0, 1, d->context);
    d->finished += 1;
}

static void TestChunks(void)
{
    static const uint32_t kSizes[3] = {3, 6, 2};
    maniTrainNet n;
    CHECK(maniTrainNetInit(&n, kSizes, 2, maniTrainElu), "a network");
    maniTrainRandom r;
    maniTrainSeed(&r, 11);
    maniTrainNetStart(&n, s_parameters, &r);
    maniTrainTranspose(&n, s_parameters, s_transposed);
    static Batch b;
    b.n = &n;
    for (uint32_t k = 0; k < BATCH * 3; ++k)
    {
        b.inputs[k] = 2.0f * maniTrainUniform(&r) - 1.0f;
    }
    static float sums[3][4096];
    Deferred deferred = {NULL, 0, NULL, 0};
    const maniTaskHooks hooks[3] = {
        {NULL, NULL, NULL}, {Backwards, NULL, NULL}, {Defer, Finish, &deferred}};
    for (uint32_t h = 0; h < 3; ++h)
    {
        maniTrainRun(&hooks[h], Chunk, BATCH / MANI_TRAIN_CHUNK, &b);
        maniTrainSum(&b.chunks[0][0], BATCH / MANI_TRAIN_CHUNK, 4096, sums[h]);
    }
    CHECK(deferred.finished == 1, "the handle finished");
    CHECK(memcmp(sums[0], sums[1], sizeof sums[0]) == 0 &&
              memcmp(sums[0], sums[2], sizeof sums[0]) == 0,
          "a batch's gradient the same bits on any host");
    float plain[4096];
    memset(plain, 0, sizeof plain);
    for (uint32_t c = 0; c < BATCH / MANI_TRAIN_CHUNK; ++c)
    {
        for (uint64_t k = 0; k < n.parameterCount; ++k)
        {
            plain[k] += b.chunks[c][k];
        }
    }
    CHECK(memcmp(plain, sums[0], sizeof(float) * n.parameterCount) == 0, "summed in chunk order");
    maniTrainRun(&hooks[0], Chunk, 0, &b);
}

static void TestAdam(void)
{
    maniTrainAdam a = maniTrainAdamDefaults();
    CHECK(a.beta1 == 0.9f && a.beta2 == 0.999f && a.epsilon == 1e-8f && a.decay == 0.001f,
          "PyTorch's defaults, Holden's decay");
    float p[3] = {0.5f, -1.0f, 2.0f};
    float m[3] = {0, 0, 0};
    float v[3] = {0, 0, 0};
    float largest[3] = {0, 0, 0};
    double pd[3] = {0.5, -1.0, 2.0};
    double md[3] = {0, 0, 0};
    double vd[3] = {0, 0, 0};
    double ld[3] = {0, 0, 0};
    bool near = true;
    for (uint64_t step = 1; step <= 50; ++step)
    {
        // Gradients that shrink, so the largest second moment is kept.
        const float g[3] = {0.3f / (float)step, -2.0f / (float)step, 0.0f};
        maniTrainAdamStep(&a, step, 0.01f, p, g, m, v, largest, 3);
        for (uint32_t k = 0; k < 3; ++k)
        {
            double gd = (double)g[k];
            pd[k] *= 1.0 - 0.01 * 0.001;
            md[k] = 0.9 * md[k] + 0.1 * gd;
            vd[k] = 0.999 * vd[k] + 0.001 * gd * gd;
            ld[k] = vd[k] > ld[k] ? vd[k] : ld[k];
            double size = 0.01 / (1.0 - pow(0.9, (double)step));
            double denominator = sqrt(ld[k]) / sqrt(1.0 - pow(0.999, (double)step)) + 1e-8;
            pd[k] -= size * md[k] / denominator;
            near = near && fabs((double)p[k] - pd[k]) <= 1e-5;
        }
    }
    CHECK(near, "AdamW with amsgrad as its formula");
    CHECK(largest[1] > v[1], "the largest second moment kept");
}

// Fits 1 -> 16 -> 16 -> 1 (ReLU) to sin over [-pi, pi]: batches of 32
// drawn from the generator, a mean squared error.
static double Fit(float* parameters, uint32_t steps)
{
    static const uint32_t kSizes[4] = {1, 16, 16, 1};
    maniTrainNet n;
    CHECK(maniTrainNetInit(&n, kSizes, 3, maniTrainRelu), "a network");
    maniTrainRandom r;
    maniTrainSeed(&r, 1234);
    maniTrainNetStart(&n, parameters, &r);
    static float m[512];
    static float v[512];
    static float largest[512];
    memset(m, 0, sizeof m);
    memset(v, 0, sizeof v);
    memset(largest, 0, sizeof largest);
    maniTrainAdam a = maniTrainAdamDefaults();
    for (uint32_t step = 1; step <= steps; ++step)
    {
        maniTrainTranspose(&n, parameters, s_transposed);
        memset(s_gradients, 0, sizeof(float) * n.parameterCount);
        for (uint32_t c = 0; c < 32 / MANI_TRAIN_CHUNK; ++c)
        {
            float target[MANI_TRAIN_CHUNK];
            for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
            {
                float x = 6.2831853f * maniTrainUniform(&r) - 3.1415927f;
                s_values[s] = x;
                target[s] = (float)sin((double)x);
            }
            maniTrainForward(&n, parameters, s_values, MANI_TRAIN_CHUNK);
            const float* out = s_values + maniTrainLayerAt(&n, 3, MANI_TRAIN_CHUNK);
            float d[MANI_TRAIN_CHUNK];
            for (uint32_t s = 0; s < MANI_TRAIN_CHUNK; ++s)
            {
                d[s] = 2.0f * (out[s] - target[s]) / 32.0f;
            }
            maniTrainBackward(&n, s_transposed, s_values, d, MANI_TRAIN_CHUNK, s_gradients, NULL,
                              s_scratch);
        }
        maniTrainAdamStep(&a, step, 0.01f, parameters, s_gradients, m, v, largest,
                          n.parameterCount);
    }
    double error = 0.0;
    for (int k = 0; k < 64; ++k)
    {
        float x = -3.1415927f + 6.2831853f * (float)k / 63.0f;
        s_values[0] = x;
        maniTrainForward(&n, parameters, s_values, 1);
        double e = (double)s_values[maniTrainLayerAt(&n, 3, 1)] - sin((double)x);
        error += e * e / 64.0;
    }
    return error;
}

static void TestFit(void)
{
    static float first[512];
    static float second[512];
    double before = Fit(first, 1);
    double after = Fit(first, 2000);
    double again = Fit(second, 2000);
    CHECK(before > 0.1 && after < 0.01, "a sine fit: its error falls below 0.01");
    CHECK(after == again && memcmp(first, second, sizeof first) == 0,
          "a second run, the same bits");
}

int main(void)
{
    TestRandom();
    TestGradients();
    TestChunks();
    TestAdam();
    TestFit();
    return s_failures == 0 ? 0 : 1;
}
