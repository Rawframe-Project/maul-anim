// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// AdamW with amsgrad (mani-0025); see train_adam.h.

#include "train_adam.h"

#include <math.h>
#include <stdint.h>

maniTrainAdam maniTrainAdamDefaults(void)
{
    return (maniTrainAdam){0.9f, 0.999f, 1e-8f, 0.001f};
}

// x^k for a whole k, by squaring: the same bits on every target, where
// pow's are the platform's.
static double Power(double x, uint64_t k)
{
    double result = 1.0;
    while (k > 0)
    {
        if ((k & 1u) != 0)
        {
            result *= x;
        }
        x *= x;
        k >>= 1;
    }
    return result;
}

void maniTrainAdamStep(const maniTrainAdam* a, uint64_t step, float rate, float* parameters,
                       const float* gradients, float* m, float* v, float* largest, uint64_t count)
{
    float keep = 1.0f - rate * a->decay;
    float size = (float)((double)rate / (1.0 - Power((double)a->beta1, step)));
    float root = (float)sqrt(1.0 - Power((double)a->beta2, step));
    float first = 1.0f - a->beta1;
    float second = 1.0f - a->beta2;
    for (uint64_t k = 0; k < count; ++k)
    {
        float g = gradients[k];
        parameters[k] *= keep;
        m[k] += (g - m[k]) * first;
        v[k] = v[k] * a->beta2 + g * g * second;
        largest[k] = v[k] > largest[k] ? v[k] : largest[k];
        float denominator = sqrtf(largest[k]) / root + a->epsilon;
        parameters[k] -= size * (m[k] / denominator);
    }
}
