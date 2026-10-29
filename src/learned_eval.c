// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Evaluating a learned model's networks, as Holden's nnet.h:
// the inputs normalized, each layer's outputs started from its biases and
// added each input times its weights (inputs of exactly 0 skipped, as the
// ReLU makes many), input by input with the output innermost, ReLU on
// every layer but the last, the outputs denormalized. Every sum runs in
// that order in binary32 with no contraction, so the bits are the same on
// every target, vectorized or not.

#include "learned_eval.h"

#include "bytes.h"
#include "fpenv.h"
#include "learned_file.h"
#include "vecmath.h"

#include "maul-anim/base.h"
#include "maul-anim/learned.h"
#include "maul-anim/pose.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static float LoadF32(const uint8_t* p)
{
    float v = 0.0f;
    memcpy(&v, p, sizeof v);
    return v;
}

// A network's place in the file: its layers, what it takes and gives,
// and where its values and weights start (in values and weights).
typedef struct Network
{
    uint32_t firstLayer;
    uint32_t layerCount;
    uint32_t in;
    uint32_t out;
    uint64_t values;
    uint64_t weights;
} Network;

static uint32_t LayerCount(const maniLearnedModel* m, uint32_t n)
{
    return maniLoadU32(m->networks + (size_t)n * MANI_LEARNED_NETWORK_SIZE);
}

static uint32_t LayerSize(const maniLearnedModel* m, uint32_t l, uint32_t word)
{
    return maniLoadU32(m->layers + (size_t)l * MANI_LEARNED_LAYER_SIZE + 4 * word);
}

// Network n, its offsets summed over the networks and layers before it.
static Network Locate(const maniLearnedModel* m, uint32_t n)
{
    Network w = {0, 0, 0, 0, 0, 0};
    bool narrow = m->weightFormat == maniLearnedWeightsInt16;
    for (uint32_t k = 0; k <= n; ++k)
    {
        w.layerCount = LayerCount(m, k);
        w.in = LayerSize(m, w.firstLayer, 0);
        w.out = LayerSize(m, w.firstLayer + w.layerCount - 1, 1);
        if (k == n)
        {
            break;
        }
        w.values += 2 * (uint64_t)w.in + 2 * (uint64_t)w.out;
        for (uint32_t l = w.firstLayer; l < w.firstLayer + w.layerCount; ++l)
        {
            uint64_t inputs = LayerSize(m, l, 0);
            uint64_t outputs = LayerSize(m, l, 1);
            w.values += outputs + (narrow ? inputs : 0u);
            w.weights += inputs * outputs;
        }
        w.firstLayer += w.layerCount;
    }
    return w;
}

// Outputs first to end of one layer: to = biases + each input of from
// times its weights.
static void Layer(const maniLearnedModel* m, uint32_t inputs, uint32_t outputs, uint32_t first,
                  uint32_t end, const uint8_t* values, uint64_t weights, const float* from,
                  float* to)
{
    for (uint32_t o = first; o < end; ++o)
    {
        to[o] = LoadF32(values + 4 * (size_t)o);
    }
    if (m->weightFormat == maniLearnedWeightsFloat32)
    {
        const uint8_t* w = m->weights + 4 * weights;
        for (uint32_t i = 0; i < inputs; ++i)
        {
            float x = from[i];
            if (x == 0.0f)
            {
                continue;
            }
            const uint8_t* row = w + 4 * (size_t)i * outputs;
            for (uint32_t o = first; o < end; ++o)
            {
                to[o] += x * LoadF32(row + 4 * (size_t)o);
            }
        }
        return;
    }
    const uint8_t* w = m->weights + 2 * weights;
    const uint8_t* scales = values + 4 * (size_t)outputs;
    for (uint32_t i = 0; i < inputs; ++i)
    {
        float x = from[i] * LoadF32(scales + 4 * (size_t)i);
        if (x == 0.0f)
        {
            continue;
        }
        const uint8_t* row = w + 2 * (size_t)i * outputs;
        for (uint32_t o = first; o < end; ++o)
        {
            to[o] += x * (float)(int16_t)maniLoadU16(row + 2 * (size_t)o);
        }
    }
}

// ReLU on count values in place.
static void Rectify(float* values, uint32_t count)
{
    for (uint32_t o = 0; o < count; ++o)
    {
        values[o] = values[o] > 0.0f ? values[o] : 0.0f;
    }
}

// Network n on input, its last layer's outputs from first to end only (end
// past them meaning all); returns its outputs (those at their places),
// which lie in the scratch. The last layer's inputs (a network of more
// than one layer) are kept in keep as they are computed, or taken from
// kept in place of every layer before the last.
static float* EvaluatePart(const maniLearnedModel* m, uint32_t n, const float* input,
                           uint32_t first, uint32_t end, float* scratch, float* keep,
                           const float* kept)
{
    Network w = Locate(m, n);
    end = end < w.out ? end : w.out;
    float* a = scratch;
    float* b = scratch + m->widest;
    const uint8_t* v = m->values + 4 * w.values;
    for (uint32_t i = 0; i < (kept != NULL ? 0u : w.in); ++i)
    {
        a[i] = (input[i] - LoadF32(v + 4 * (size_t)i)) / LoadF32(v + 4 * ((size_t)w.in + i));
    }
    const uint8_t* outputs = v + 8 * (size_t)w.in;
    v += 8 * ((size_t)w.in + w.out);
    uint64_t weights = w.weights;
    bool narrow = m->weightFormat == maniLearnedWeightsInt16;
    uint32_t last = w.firstLayer + w.layerCount - 1;
    // The hidden layers; reusing, only their places are passed.
    for (uint32_t l = w.firstLayer; l < last; ++l)
    {
        uint32_t inputs = LayerSize(m, l, 0);
        uint32_t count = LayerSize(m, l, 1);
        if (kept == NULL)
        {
            Layer(m, inputs, count, 0u, count, v, weights, a, b);
            Rectify(b, count);
        }
        v += 4 * ((size_t)count + (narrow ? inputs : 0u));
        weights += (uint64_t)inputs * count;
        float* t = a;
        a = b;
        b = t;
    }
    uint32_t inputs = LayerSize(m, last, 0);
    if (keep != NULL)
    {
        memcpy(keep, a, sizeof(float) * inputs);
    }
    Layer(m, inputs, LayerSize(m, last, 1), first, end, v, weights, kept != NULL ? kept : a, b);
    for (uint32_t o = first; o < end; ++o)
    {
        b[o] = b[o] * LoadF32(outputs + 4 * ((size_t)w.out + o)) + LoadF32(outputs + 4 * (size_t)o);
    }
    return b;
}

static float* Evaluate(const maniLearnedModel* m, uint32_t n, const float* input, float* scratch)
{
    return EvaluatePart(m, n, input, 0, UINT32_MAX, scratch, NULL, NULL);
}

uint32_t maniLearnedHiddenCount(const maniLearnedModel* model)
{
    Network w = Locate(model, MANI_LEARNED_DECOMPRESSOR);
    return w.layerCount > 1 ? LayerSize(model, w.firstLayer + w.layerCount - 1, 0) : 0u;
}

static bool AllFinite(const float* values, uint32_t count)
{
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!(fabsf(values[k]) < HUGE_VALF))
        {
            return false;
        }
    }
    return true;
}

bool maniLearnedDecompressMotion(const maniLearnedModel* model, const float* state, float* out,
                                 void* scratch, float* hidden)
{
    uint32_t first = 9 * model->boneCount + 3 * model->scaledBoneCount;
    uint32_t end = model->outputCount;
    const float* a =
        EvaluatePart(model, MANI_LEARNED_DECOMPRESSOR, state, first, end, scratch, hidden, NULL);
    if (!AllFinite(a + first, end - first))
    {
        return false;
    }
    memcpy(out, a + first, sizeof(float) * (size_t)(end - first));
    return true;
}

size_t maniLearnedModel_GetScratchSize(const maniLearnedModel* model)
{
    return model != NULL ? 2 * sizeof(float) * (size_t)model->widest : 0;
}

static bool ScratchFits(const maniLearnedModel* m, const void* scratch, size_t size)
{
    return scratch != NULL && (uintptr_t)scratch % sizeof(float) == 0 &&
           size >= maniLearnedModel_GetScratchSize(m);
}

static MANI_NOINLINE maniResult LearnedProjectWork(const maniLearnedModel* model,
                                                   uint32_t projector, const float* query,
                                                   float* stateOut, uint32_t capacity,
                                                   uint32_t* countOut, void* scratch,
                                                   size_t scratchSize)
{
    if (model == NULL || (stateOut == NULL && capacity > 0))
    {
        return mani_errorInvalid;
    }
    uint32_t count = model->featureCount + model->latentCount;
    if (countOut != NULL)
    {
        *countOut = count;
    }
    // A state is never empty, so a NULL state only measures.
    if (stateOut == NULL || capacity < count)
    {
        return mani_errorCapacity;
    }
    if (query == NULL || projector >= model->projectorCount ||
        !ScratchFits(model, scratch, scratchSize) || !AllFinite(query, model->featureCount))
    {
        return mani_errorInvalid;
    }
    const float* out = Evaluate(model, MANI_LEARNED_PROJECTORS + projector, query, scratch);
    // Finite weights, every one checked at open, can still carry a value
    // past the float range: the model's data, refused.
    if (!AllFinite(out, count))
    {
        return mani_errorFormat;
    }
    memcpy(stateOut, out, sizeof(float) * count);
    return mani_success;
}

maniResult maniLearnedModel_Project(const maniLearnedModel* model, uint32_t projector,
                                    const float* query, float* stateOut, uint32_t capacity,
                                    uint32_t* countOut, void* scratch, size_t scratchSize)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = LearnedProjectWork(model, projector, query, stateOut, capacity, countOut,
                                           scratch, scratchSize);
    maniFloatEnvLeave(env);
    return result;
}

static MANI_NOINLINE maniResult LearnedStepWork(const maniLearnedModel* model, float* state,
                                                void* scratch, size_t scratchSize)
{
    uint32_t count = model != NULL ? model->featureCount + model->latentCount : 0;
    if (model == NULL || state == NULL || !ScratchFits(model, scratch, scratchSize) ||
        !AllFinite(state, count))
    {
        return mani_errorInvalid;
    }
    const float* rate = Evaluate(model, MANI_LEARNED_STEPPER, state, scratch);
    float seconds = 1.0f / model->stepRate;
    // Checked whole before any is written, so a refused step leaves the
    // state as it was.
    for (uint32_t k = 0; k < count; ++k)
    {
        if (!(fabsf(state[k] + seconds * rate[k]) < HUGE_VALF))
        {
            return mani_errorFormat;
        }
    }
    for (uint32_t k = 0; k < count; ++k)
    {
        state[k] += seconds * rate[k];
    }
    return mani_success;
}

maniResult maniLearnedModel_Step(const maniLearnedModel* model, float* state, void* scratch,
                                 size_t scratchSize)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = LearnedStepWork(model, state, scratch, scratchSize);
    maniFloatEnvLeave(env);
    return result;
}

// A rotation from the first two columns of its matrix: the first
// normalized, the third the normalized cross of it and the second (any
// axis square to the first when they are parallel), the second their
// cross; then Shepperd's method, w not negative.
static maniQuat FromColumns(maniVec3 x, maniVec3 y)
{
    maniVec3 c0 = maniVec3Normalize(x, maniVec3Make(1.0f, 0.0f, 0.0f));
    maniVec3 least = fabsf(c0.x) <= fabsf(c0.y) && fabsf(c0.x) <= fabsf(c0.z)
                         ? maniVec3Make(1.0f, 0.0f, 0.0f)
                         : (fabsf(c0.y) <= fabsf(c0.z) ? maniVec3Make(0.0f, 1.0f, 0.0f)
                                                       : maniVec3Make(0.0f, 0.0f, 1.0f));
    maniVec3 square = maniVec3Normalize(maniVec3Cross(c0, least), least);
    maniVec3 c2 = maniVec3Normalize(maniVec3Cross(c0, y), square);
    maniVec3 c1 = maniVec3Cross(c2, c0);
    float trace = c0.x + c1.y + c2.z;
    maniQuat q;
    if (trace >= c0.x && trace >= c1.y && trace >= c2.z)
    {
        q = (maniQuat){c1.z - c2.y, c2.x - c0.z, c0.y - c1.x, 1.0f + trace};
    }
    else if (c0.x >= c1.y && c0.x >= c2.z)
    {
        q = (maniQuat){1.0f + c0.x - c1.y - c2.z, c0.y + c1.x, c2.x + c0.z, c1.z - c2.y};
    }
    else if (c1.y >= c2.z)
    {
        q = (maniQuat){c0.y + c1.x, 1.0f + c1.y - c0.x - c2.z, c1.z + c2.y, c2.x - c0.z};
    }
    else
    {
        q = (maniQuat){c2.x + c0.z, c1.z + c2.y, 1.0f + c2.z - c0.x - c1.y, c0.y - c1.x};
    }
    q = maniQuatNormalize(q);
    return q.w < 0.0f ? (maniQuat){-q.x, -q.y, -q.z, -q.w} : q;
}

static maniVec3 Vec3(const float* v)
{
    return maniVec3Make(v[0], v[1], v[2]);
}

// The pose from the outputs: each bone's translation and rotation, its
// scale given (scaled bones, in order) or the model's.
static void Pose(const maniLearnedModel* m, const float* out, maniPose* pose)
{
    const float* scaled = out + 9 * (size_t)m->boneCount;
    const uint8_t* scales = m->bones + 4 * (size_t)m->scaledBoneCount;
    uint32_t next = 0;
    for (uint32_t b = 0; b < m->boneCount; ++b)
    {
        const float* t = out + 9 * (size_t)b;
        maniVec3 scale;
        if (next < m->scaledBoneCount && maniLoadU32(m->bones + 4 * (size_t)next) == b)
        {
            scale = Vec3(scaled + 3 * (size_t)next++);
        }
        else
        {
            const uint8_t* s = scales + 12 * (size_t)b;
            scale = maniVec3Make(LoadF32(s), LoadF32(s + 4), LoadF32(s + 8));
        }
        maniPose_SetLocal(pose, b,
                          (maniTransform){Vec3(t), FromColumns(Vec3(t + 3), Vec3(t + 6)), scale});
    }
}

static MANI_NOINLINE maniResult LearnedDecompressWork(const maniLearnedModel* model,
                                                      const float* state, maniLearnedOutput* output,
                                                      void* scratch, size_t scratchSize)
{
    return maniLearnedDecompressFrom(model, state, output, scratch, scratchSize, NULL);
}

maniResult maniLearnedModel_Decompress(const maniLearnedModel* model, const float* state,
                                       maniLearnedOutput* output, void* scratch, size_t scratchSize)
{
    uint64_t env = maniFloatEnvEnter();
    maniResult result = LearnedDecompressWork(model, state, output, scratch, scratchSize);
    maniFloatEnvLeave(env);
    return result;
}

maniResult maniLearnedDecompressFrom(const maniLearnedModel* model, const float* state,
                                     maniLearnedOutput* output, void* scratch, size_t scratchSize,
                                     const float* hidden)
{
    uint32_t count = model != NULL ? model->featureCount + model->latentCount : 0;
    if (model == NULL || state == NULL || output == NULL ||
        !ScratchFits(model, scratch, scratchSize) || !AllFinite(state, count) ||
        (output->pose != NULL && (maniPose_GetBoneCount(output->pose) != model->boneCount ||
                                  maniPose_IsDifference(output->pose))))
    {
        return mani_errorInvalid;
    }
    if ((output->floatTracks != NULL && output->floatTrackCapacity < model->floatTrackCount) ||
        (output->events != NULL && output->eventCapacity < model->eventCount))
    {
        return mani_errorCapacity;
    }
    const float* out =
        EvaluatePart(model, MANI_LEARNED_DECOMPRESSOR, state, 0, UINT32_MAX, scratch, NULL, hidden);
    // Every output finite before any is written; finite outputs make a
    // finite pose, as the rotation's columns are normalized in binary64.
    if (!AllFinite(out, model->outputCount))
    {
        return mani_errorFormat;
    }
    if (output->pose != NULL)
    {
        Pose(model, out, output->pose);
    }
    const float* root = out + 9 * (size_t)model->boneCount + 3 * (size_t)model->scaledBoneCount;
    output->rootVelocity = Vec3(root);
    output->rootAngularVelocity = Vec3(root + 3);
    if (output->floatTracks != NULL)
    {
        memcpy(output->floatTracks, root + 6, sizeof(float) * model->floatTrackCount);
    }
    if (output->events != NULL)
    {
        memcpy(output->events, root + 6 + model->floatTrackCount,
               sizeof(float) * model->eventCount);
    }
    return mani_success;
}
