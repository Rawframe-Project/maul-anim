// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Running a trainer's chunks on the host's task hooks (mani-0025), and
// summing their gradients in chunk order.

#ifndef MAUL_ANIM_SRC_TRAIN_TASKS_H
#define MAUL_ANIM_SRC_TRAIN_TASKS_H

#include "maul-anim/base.h"

#include <stdint.h>

// Runs task over [0, count): on the hooks, one item a range at least, or
// on this thread when the hooks have no enqueue.
void maniTrainRun(const maniTaskHooks* hooks, maniTaskFn* task, int32_t count, void* context);

// sum = chunk 0 + chunk 1 + ..., value by value in chunk order; chunk c
// starts at chunks + c * count.
void maniTrainSum(const float* chunks, uint32_t chunkCount, uint64_t count, float* sum);

#endif // MAUL_ANIM_SRC_TRAIN_TASKS_H
