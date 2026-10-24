// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// AdamW with amsgrad, as PyTorch's torch.optim.AdamW steps
// one tensor: the weight decay taken from the parameters first, the
// moments updated, the largest second moment kept, then the step. The
// bias corrections are computed in binary64 from the step count, every
// parameter's arithmetic in binary32, in parameter order.

#ifndef MAUL_ANIM_SRC_TRAIN_ADAM_H
#define MAUL_ANIM_SRC_TRAIN_ADAM_H

#include <stdint.h>

typedef struct maniTrainAdam
{
    float beta1;
    float beta2;
    float epsilon;
    float decay;
} maniTrainAdam;

// PyTorch's defaults (0.9, 0.999, 1e-8) and Holden's weight decay 0.001.
maniTrainAdam maniTrainAdamDefaults(void);

// Step number step (from 1) of count parameters at learning rate rate:
// p *= 1 - rate * decay; m += (g - m) * (1 - beta1);
// v = v * beta2 + g * g * (1 - beta2); largest = max(largest, v);
// p -= rate / (1 - beta1^step) * (m / (sqrt(largest) / sqrt(1 -
// beta2^step) + epsilon)).
void maniTrainAdamStep(const maniTrainAdam* a, uint64_t step, float rate, float* parameters,
                       const float* gradients, float* m, float* v, float* largest, uint64_t count);

#endif // MAUL_ANIM_SRC_TRAIN_ADAM_H
