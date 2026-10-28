// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// A dense network in training; see train_net.h.

#include "train_net.h"

#include "elementary_float.h"
#include "train_random.h"

#include "maul-anim/learned.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

bool maniTrainNetInit(maniTrainNet* n, const uint32_t* sizes, uint32_t layerCount,
                      maniTrainActivation activation)
{
    if (layerCount < 1 || layerCount > MANI_MAX_LEARNED_LAYERS)
    {
        return false;
    }
    *n = (maniTrainNet){.layerCount = layerCount, .activation = activation};
    uint64_t weights = 0;
    for (uint32_t l = 0; l <= layerCount; ++l)
    {
        if (sizes[l] < 1 || sizes[l] > MANI_MAX_LEARNED_WIDTH)
        {
            return false;
        }
        n->sizes[l] = sizes[l];
        n->valueCount += sizes[l];
        n->widest = sizes[l] > n->widest ? sizes[l] : n->widest;
    }
    for (uint32_t l = 0; l < layerCount; ++l)
    {
        n->offsets[l] = n->parameterCount;
        uint64_t count = (uint64_t)sizes[l] * sizes[l + 1];
        weights += count;
        n->parameterCount += count + sizes[l + 1];
    }
    return weights <= MANI_MAX_LEARNED_WEIGHTS;
}

void maniTrainNetStart(const maniTrainNet* n, float* parameters, maniTrainRandom* r)
{
    for (uint32_t l = 0; l < n->layerCount; ++l)
    {
        float bound = 1.0f / sqrtf((float)n->sizes[l]);
        uint64_t count = (uint64_t)n->sizes[l] * n->sizes[l + 1] + n->sizes[l + 1];
        float* p = parameters + n->offsets[l];
        for (uint64_t k = 0; k < count; ++k)
        {
            p[k] = bound * (2.0f * maniTrainUniform(r) - 1.0f);
        }
    }
}

uint64_t maniTrainLayerAt(const maniTrainNet* n, uint32_t l, uint32_t count)
{
    uint64_t at = 0;
    for (uint32_t k = 0; k < l; ++k)
    {
        at += n->sizes[k];
    }
    return at * count;
}

static float Activate(maniTrainActivation activation, float x)
{
    if (x > 0.0f)
    {
        return x;
    }
    return activation == maniTrainElu ? maniExpF(x) - 1.0f : 0.0f;
}

void maniTrainForward(const maniTrainNet* n, const float* parameters, float* values, uint32_t count)
{
    for (uint32_t l = 0; l < n->layerCount; ++l)
    {
        uint32_t in = n->sizes[l];
        uint32_t out = n->sizes[l + 1];
        const float* weights = parameters + n->offsets[l];
        const float* biases = weights + (size_t)in * out;
        const float* from = values + maniTrainLayerAt(n, l, count);
        float* to = values + maniTrainLayerAt(n, l + 1, count);
        bool last = l + 1 == n->layerCount;
        for (uint32_t s = 0; s < count; ++s)
        {
            const float* x = from + (size_t)s * in;
            float* y = to + (size_t)s * out;
            memcpy(y, biases, sizeof(float) * out);
            for (uint32_t i = 0; i < in; ++i)
            {
                if (x[i] == 0.0f)
                {
                    continue;
                }
                const float* row = weights + (size_t)i * out;
                for (uint32_t o = 0; o < out; ++o)
                {
                    y[o] += x[i] * row[o];
                }
            }
            for (uint32_t o = 0; !last && o < out; ++o)
            {
                y[o] = Activate(n->activation, y[o]);
            }
        }
    }
}

void maniTrainTranspose(const maniTrainNet* n, const float* parameters, float* transposed)
{
    for (uint32_t l = 0; l < n->layerCount; ++l)
    {
        uint32_t in = n->sizes[l];
        uint32_t out = n->sizes[l + 1];
        const float* weights = parameters + n->offsets[l];
        float* t = transposed + n->offsets[l];
        for (uint32_t i = 0; i < in; ++i)
        {
            for (uint32_t o = 0; o < out; ++o)
            {
                t[(size_t)o * in + i] = weights[(size_t)i * out + o];
            }
        }
    }
}

// The derivative of the activation at a value it gave.
static float Slope(maniTrainActivation activation, float y)
{
    if (y > 0.0f)
    {
        return 1.0f;
    }
    return activation == maniTrainElu ? y + 1.0f : 0.0f;
}

// Layer l's weight and bias gradients from its inputs and the gradient
// by its outputs, sample after sample.
static void Accumulate(const maniTrainNet* n, uint32_t l, const float* x, const float* dz,
                       uint32_t count, float* gradients)
{
    uint32_t in = n->sizes[l];
    uint32_t out = n->sizes[l + 1];
    float* dw = gradients + n->offsets[l];
    float* db = dw + (size_t)in * out;
    for (uint32_t s = 0; s < count; ++s)
    {
        const float* xs = x + (size_t)s * in;
        const float* d = dz + (size_t)s * out;
        for (uint32_t i = 0; i < in; ++i)
        {
            if (xs[i] == 0.0f)
            {
                continue;
            }
            float* row = dw + (size_t)i * out;
            for (uint32_t o = 0; o < out; ++o)
            {
                row[o] += xs[i] * d[o];
            }
        }
        for (uint32_t o = 0; o < out; ++o)
        {
            db[o] += d[o];
        }
    }
}

// The gradient by layer l's inputs: each output's gradient times its
// weights, output by output.
static void Propagate(const maniTrainNet* n, uint32_t l, const float* transposed, const float* dz,
                      uint32_t count, float* dx)
{
    uint32_t in = n->sizes[l];
    uint32_t out = n->sizes[l + 1];
    const float* t = transposed + n->offsets[l];
    memset(dx, 0, sizeof(float) * (size_t)count * in);
    for (uint32_t s = 0; s < count; ++s)
    {
        float* d = dx + (size_t)s * in;
        for (uint32_t o = 0; o < out; ++o)
        {
            float g = dz[(size_t)s * out + o];
            const float* column = t + (size_t)o * in;
            for (uint32_t i = 0; i < in; ++i)
            {
                d[i] += g * column[i];
            }
        }
    }
}

void maniTrainBackward(const maniTrainNet* n, const float* transposed, const float* values,
                       const float* outputGradient, uint32_t count, float* gradients,
                       float* inputGradient, float* scratch)
{
    float* dz = scratch;
    float* dx = scratch + (size_t)MANI_TRAIN_CHUNK * n->widest;
    uint32_t last = n->sizes[n->layerCount];
    memcpy(dz, outputGradient, sizeof(float) * (size_t)count * last);
    for (uint32_t l = n->layerCount; l-- > 0;)
    {
        const float* x = values + maniTrainLayerAt(n, l, count);
        Accumulate(n, l, x, dz, count, gradients);
        if (l == 0 && inputGradient == NULL)
        {
            break;
        }
        Propagate(n, l, transposed, dz, count, l == 0 ? inputGradient : dx);
        if (l == 0)
        {
            break;
        }
        uint32_t in = n->sizes[l];
        for (size_t k = 0; k < (size_t)count * in; ++k)
        {
            dz[k] = dx[k] * Slope(n->activation, x[k]);
        }
    }
}
